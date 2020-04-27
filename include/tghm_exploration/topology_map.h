#ifndef TOPOLOGY_MAP_H_
#define TOPOLOGY_MAP_H_

#include <costmap_2d/costmap_2d.h>

namespace topology_map
{
/**
 * @brief Represents a node in topology map
 *
 */
struct TopologyNode {
  std::uint32_t size;
  double cost;
  bool visited;
  geometry_msgs::Point centroid;
  std::vector<TopologyNode*> parents;
  std::vector<TopologyNode*> children;
};

/**
 * @brief Thread-safe implementation of a node-generation task for
 * TGHM algorithm for an input costmap.
 */
class TopologyMap {
public:
  TopologyMap(){}

  TopologyMap(costmap_2d::Costmap2D* costmap,
              double lambda,
              geometry_msgs::Point pose);

  TopologyNode getNextGoal(geometry_msgs::Point pose);
  
  void updateTopologyMap(geometry_msgs::Point pose);

protected:
  void generateCandidateNodes();
  TopologyNode createNode(unsigned int initial_cell,
                          std::vector<bool>& node_flag);
  bool isNewNodeCell(unsigned int idx,
                     const std::vector<bool>& node_flag);
  double calculateNodeCost();

private:
  costmap_2d::Costmap2D* costmap_;
  unsigned char grid_map_;
  unsigned int size_x_, size_y_;

  TopologyNode topology_map_;
  TopologyNode current_node_;
  std::vector<TopologyNode> candidate_nodes_;
  double lambda_;
};
}
#endif
