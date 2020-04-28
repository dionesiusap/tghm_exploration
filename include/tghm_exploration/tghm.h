#ifndef TGHM_H_
#define TGHM_H_

#include <costmap_2d/costmap_2d.h>

namespace tghm
{
/**
 * @brief Represents a node in topology map
 *
 */
struct TopologyNode {
  std::uint32_t unknown_cells;
  double score;
  bool visited;
  geometry_msgs::Point centroid;
  std::vector<TopologyNode*> neighbors;
  // search utilities
  bool temp_v;
  unsigned int level;
};

/**
 * @brief Thread-safe implementation of a goal-generation task for
 * TGHM algorithm for an input costmap.
 */
class TGHM {
public:
  TGHM(){}

  TGHM(costmap_2d::Costmap2D* costmap,
       double lambda,
       double sensor_max_range,
       double unknown_threshold,
       geometry_msgs::Point pose);

  TopologyNode getNextGoal();

  std::vector<TopologyNode> getMapVector();
  
protected:
  void generateNewCandidateNodes(unsigned int pos_x_map,
                                 unsigned int pos_y_map);

  TopologyNode createNode(unsigned int initial_cell,
                          std::vector<bool>& node_flag);

  void updateCandidateNodes();
  
  bool isNewNodeCell(unsigned int idx,
                     const std::vector<bool>& node_flag);
  
  bool isCellObservableFrom(unsigned int idx,
                        unsigned int reference);
  
  void calculateNodeCost(TopologyNode* node);
  
  unsigned int getTopologicalDistance(TopologyNode* candidate_node,
                                      TopologyNode* map_node);
  
  void resetNodeSearch();

private:
  costmap_2d::Costmap2D* costmap_;
  unsigned char grid_map_;
  unsigned int size_x_, size_y_;

  std::vector<TopologyNode> map_nodes_;
  std::vector<TopologyNode> candidate_nodes_;
  TopologyNode* current_node_;
  TopologyNode* root_node_;
  
  // exploration parameters
  double min_unknown_size_;
  double lambda_;
  double sensor_max_range_;
  unsigned int unknown_threshold_;
};
}
#endif
