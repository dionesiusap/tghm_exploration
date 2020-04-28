#include <tghm_exploration/topology_map.h>
#include <tghm_exploration/costmap_tools.h>

#include <mutex>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>

namespace tghm
{
  using costmap_2d::LETHAL_OBSTACLE;
  using costmap_2d::NO_INFORMATION;
  using costmap_2d::FREE_SPACE;
  
  TGHM::TGHM(costmap_2d::Costmap2D* costmap,
             double lambda,
             geometry_msgs::Point pose)
    : costmap_(costmap)
    , lambda_(lambda)
  {
    TopologyNode root;
    root.centroid = pose;
    root.score = 0;
    root.visited = true;
    map_nodes_.push_back(root);
    root_node_ = &map_nodes_[0];
    current_node_ = &map_nodes_[0];
  }

  TopologyNode TGHM::getNextGoal()
  {
    // sanity check that robot is inside costmap bounds before searching
    unsigned int pos_x_map, pos_y_map;
    if (!costmap_->worldToMap(current_node_->centroid.x, current_node->centroid.y,
                              pos_x_map, pos_y_map))
    {
      ROS_ERROR("getNextGoal: Robot out of costmap bounds, cannot search for candidate nodes");
      return current_node_;
    }
    // lock the costmap during the generation of nodes
    std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

    generateNewCandidateNodes(pos_x_map, pos_y_map);
    updateCandidateNodes();

    TopologyNode* best_node = &candidate_nodes_[0];
    for (std::vector<TopologyNode>::iterator it = candidate_nodes_.begin()
         it != candidate_nodes_.end(); ++it)
    {
      if (*it.score > best_node->score) {
        best_node = it;
      }
    }

    TopologyNode new_map_node = *best_node;
    map_node_.push_back(new_map_node);

    current_node_ = best_node;
    return best_node;
  }

  void TGHM::generateNewCandidateNodes(unsigned int pos_x_map,
                                       unsigned int pos_y_map)
  {
    grid_map_ = costmap_->getCharMap();
    size_x_ = costmap_->getSizeInCellsX();
    size_y_ = costmap_->getSizeInCellsY();

    // initialize flag arrays to keep track of visited and node cells
    std::vector<bool> node_flag(size_x_ * size_y_, false);
    std::vector<bool> visited_flag(size_x_ * size_y_, false);

    // initialize bfs
    std::queue<unsigned int> bfs;

    // find closest clear cell to start search
    unsigned int clear, pos = costmap_->getIndex(pos_x_map, pos_y_map);
    unsigned int reference;
    if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
      bfs.push(clear);
      reference = clear;
    } else {
      bfs.push(pos);
      reference = pos;
      ROS_WARN("Could not find nearby clear cell to start search");
    }
    visited_flag[bfs.front()] = true;

    while (!bfs.empty()) {
      unsigned int idx = bfs.front();
      bfs.pop();
      // iterate over 4-connected neighbourhood
      for (unsigned nbr : nhood4(idx, *costmap_)) {
        if isCellObservableFrom(nbr, reference) {
          // add to queue all free, unvisited cells, use descending search in case
          // initialized on non-free cell
          if (grid_map_[nbr] <= grid_map_[idx] && !visited_flag[nbr]) {
            visited_flag[nbr] = true;
            bfs.push(nbr);
            // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
            // neighbour)
          } else if (isNewNodeCell(nbr, node_flag)) {
            node_flag[nbr] = true;
            TopologyNode new_node = createNode(nbr, pos, target, node_flag);
            candidate_nodes_.push_back(new_node);
          }
        }
      }
    }
  }

  TopologyNode TGHM::createNode(unsigned int initial_cell,
                                std::vector<bool>& node_flag)
  {
    // initialize node structure
    TopologyNode output;
    output.centroid.x = 0;
    output.centroid.y = 0;
    output.score = 0;
    output.unknown_cells = 0;
    output.neighbors.push_back(current_node_);
    std::uint32_t temp_size = 1;

    unsigned int ix, iy;
    double initial_x, initial_y;
    costmap_->indexToCells(initial_cell, ix, iy);
    costmap_->mapToWorld(ix, iy, initial_x, initial_y);

    // push initial gridcell onto queue
    std::queue<unsigned int> bfs;
    bfs.push(initial_cell);

    while (!bfs.empty()) {
      unsigned int idx = bfs.front();
      bfs.pop();

      // try adding cells in 8-connected neighborhood to frontier
      for (unsigned int nbr : nhood8(idx, *costmap_)) {
        // check if neighbour is a potential frontier cell
        if (isNewNodeCell(nbr, node_flag)) {
          // mark cell as unknown cell in node
          node_flag[nbr] = true;
          unsigned int mx, my;
          double wx, wy;
          costmap_->indexToCells(nbr, mx, my);
          costmap_->mapToWorld(mx, my, wx, wy);

          // update node size
          temp_size++;

          // update centroid of node
          output.centroid.x += wx;
          output.centroid.y += wy;

          // add to queue for breadth first search
          bfs.push(nbr);
        }
      }
    }

    output.centroid.x /= temp_size;
    output.centroid.y /= temp_size;

    return output;
  }

  void TGHM::updateCandidateNodes()
  {
    std::vector<unsigned int> idxs;
    for (int i = 0; i < candidate_nodes_.size(); i++) {
      if (candidate_nodes_.unknown_cells * costmap_->getResolution() <
          min_unknown_size_) {
        idx.push_back(i);
      }
    }
    
    for (i : idxs) {
      candidate_nodes_.erase(candidate_nodes_.begin()+i);
    }
  }

  bool TGHM::isNewNodeCell(unsigned int idx,
                           const std::vector<bool>& node_flag)
  {
    // check that cell is unknown and not already marked as node
    if (grid_map_[idx] != NO_INFORMATION || node_flag[idx]) {
      return false;
    }

    // node cells should have at least one cell in 4-connected neighbourhood
    // that is free
    for (unsigned int nbr : nhood4(idx, *costmap_)) {
      if (grid_map_[nbr] == FREE_SPACE) {
        return true;
      }
    }

    return false;
  }

  bool TGHM::isCellObservableFrom(unsigned int idx,
                                  unsigned int reference)
  {
    unsigned int idx_x_map, idx_y_map;
    unsigned int ref_x_map, ref_y_map;
    double idx_x_world, idx_y_world;
    double ref_x_world, ref_y_world;
    
    costmap_->indexToCells(idx, idx_x_map, idx_y_map);
    costmap_->indexToCells(reference, ref_x_map, ref_y_map);

    costmap_->mapToWorld(idx_x_map, idx_y_map, idx_x_world, idx_y_world);
    costmap_->mapToWorld(ref_x_map, ref_y_map, ref_x_world, ref_y_world);

    double distance = sqrt(pow((double(ref_x_map) - double(idx_x_map)), 2.0) +
                           pow((double(ref_y_map) - double(idx_y_map)), 2.0));

    if (distance <= sensor_max_range_) {
      for (unsigned int nbr : nhood4(idx, *costmap_)) {
        if (grid_map_[nbr] == FREE_SPACE || grid_map_[nbr] == NO_INFORMATION)
          return true;
      }

    }
    
    return false;
  }

  void TGHM::calculateNodeCost(TopologyNode* node)
  {
    unsigned int node_idx = costmap_->getIndex(node->centroid.x, node->centroid.y);

    // push node centroid into queue
    std::queue<unsigned int> bfs;
    bfs.push(node_idx);

    while (!bfs.empty()) {
      unsigned int idx = bfs.front();
      bfs.pop();

      for (unsigned int nbr : nhood8(idx, *costmap_)) {
        if (isCellObservableFrom(idx, node_idx)) {
          if (grid_map_[idx] == NO_INFORMATION) {
            node->unknown_cells++;
          }
          bfs.push(nbr);
        }
      }
    }

    node->score = node->unknown_cells * 
                    exp(-lambda_ * getTopologicalDistance(node, current_node_));
  }

  unsigned int TGHM::getTopologicalDistance(TopologyNode* candidate_node,
                                            TopologyNode* map_node)
  {
    bool found = false;
    resetNodeSearch();
    candidate_node->temp_v = true;
    
    std::queue<TopologyNode*> bfs;
    bfs.push(candidate_node);

    while (!bfs.empty() || !found) {
      TopologyNode* curr_node = bfs.front();
      bfs.pop();

      for (std::vector<TopologyNode*>::iterator it = curr_node->neighbors.begin()
          it != curr_node->neighbors.end(); ++it)
      {
        if (!*it->temp_v) {
          *it->temp_v = true;
          *it->level = curr_node->level + 1;
          bfs.push(*it);
        }
        if (*it->centroid == map_node->centroid) {
          found == true;
          return *it->level;
        }
      }
    }

    ROS_ERROR("getTopologicalDistance: MAP NODE NOT FOUND");
    return 0;
  }

  void TGHM::resetNodeSearch()
  {
    for (std::vector<TopologyNode>::iterator it = map_nodes_.begin()
         it != map_nodes_.end(); ++it)
      {
        *it->temp_v = false;
        *it->level = 0;
      }
  }

}