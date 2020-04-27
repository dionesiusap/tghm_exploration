#include <explore/topology_map.h>
#include <explore/costmap_tools.h>

#include <mutex>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>

namespace topology_map
{
  using costmap_2d::LETHAL_OBSTACLE;
  using costmap_2d::NO_INFORMATION;
  using costmap_2d::FREE_SPACE;
  
  TopologyMap::TopologyMap(costmap_2d::Costmap2D* costmap,
                           double lambda,
                           geometry_msgs::Point pose)
    : costmap_(costmap)
    , lambda_(lambda)
  {
    TopologyNode root;
    root.centroid = pose;
    root.cost = 0;
    root.visited = true;
    current_node_ = root;
  }

  TopologyNode TopologyMap::getNextGoal(geometry_msgs::Point pose)
  {
    // sanity check that robot is inside costmap bounds before searching
    unsigned int pos_x_map, pos_y_map;
    if (!costmap_->worldToMap(position.x, position.y, pos_x_map, pos_y_map)) {
      ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
      return frontier_list;
    }

    generateCandidateNodes(pos_x_map, pos_y_map);
  }

  void TopologyMap::generateCandidateNodes(unsigned int pos_x_map,
                                           unsigned int pos_y_map)
  {
    // lock the costmap during the generation of nodes
    std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
    
    grid_map_ = costmap_->getCharMap();
    size_x_ = costmap_->getSizeInCellsX();
    size_y_ = costmap_->getSizeInCellsY();

    // initialize flag arrays to keep track of visited and frontier cells
    std::vector<bool> node_flag(size_x_ * size_y_, false);
    std::vector<bool> visited_flag(size_x_ * size_y_, false);

    // initialize bfs
    std::queue<unsigned int> bfs;

    // find closest clear cell to start search
    unsigned int clear, pos = costmap_->getIndex(pos_x_map, pos_y_map);
    if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
      bfs.push(clear);
    } else {
      bfs.push(pos);
      ROS_WARN("Could not find nearby clear cell to start search");
    }
    visited_flag[bfs.front()] = true;

    while (!bfs.empty()) {
      unsigned int idx = bfs.front();
      bfs.pop();
      // iterate over 4-connected neighbourhood
      for (unsigned nbr : nhood4(idx, *costmap_)) {
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
          // TODO: filter
          candidate_nodes_.push_back(new_node);
        }
      }
    }
  }

  TopologyNode TopologyMap::createNode(unsigned int initial_cell,
                                       std::vector<bool>& node_flag)
  {
    // initialize node structure
    TopologyNode output;
    output.centroid.x = 0;
    output.centroid.y = 0;
    output.size = 1;
    output.parents.push_back(&current_node_);

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
          // mark cell as frontier
          node_flag[nbr] = true;
          unsigned int mx, my;
          double wx, wy;
          costmap_->indexToCells(nbr, mx, my);
          costmap_->mapToWorld(mx, my, wx, wy);

          // update node size
          output.size++;

          // update centroid of node
          output.centroid.x += wx;
          output.centroid.y += wy;

          // add to queue for breadth first search
          bfs.push(nbr);
        }
      }
    }

    output.centroid.x /= output.size;
    output.centroid.y /= output.size;

    return output
  }

  bool TopologyMap::isNewNodeCell(unsigned int idx,
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
}