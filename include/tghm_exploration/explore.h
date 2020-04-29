/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2020, Dionesius Agung.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#ifndef EXPLORE_H_
#define EXPLORE_H_

#include <tghm_exploration/costmap_client.h>
#include <tghm_exploration/tghm.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

namespace explore
{
/**
 * @class Explore
 * @brief A class adhering to the robot_actions::Action interface that moves the
 * robot base to explore its environment.
 */
class Explore
{
public:
  Explore();
  ~Explore();

  void start();
  void stop();

private:
  /**
   * @brief  Make a global plan
   */
  void makePlan();

  /**
   * @brief  Publish a nodes as markers
   */
  void visualizeNodes(
    const std::vector<tghm::TopologyNode>& map,
    const geometry_msgs::Point& target);

  void reachedGoal(const actionlib::SimpleClientGoalState& status,
                   const move_base_msgs::MoveBaseResultConstPtr& result,
                   const geometry_msgs::Point& target);

  bool goalOnBlacklist(const geometry_msgs::Point& goal);

  ros::NodeHandle private_nh_;
  ros::NodeHandle relative_nh_;
  ros::Publisher marker_array_publisher_;
  tf::TransformListener tf_listener_;

  Costmap2DClient costmap_client_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
      move_base_client_;
  unsigned int status_;
  tghm::TGHM tghm_;
  ros::Timer exploring_timer_;

  geometry_msgs::Point prev_goal_;
  double prev_distance_;
  ros::Time last_progress_;
  size_t last_markers_count_;

  // parameters
  double planner_frequency_;
  bool visualize_;
  double lambda_;
  double sensor_max_range_;
  double unknown_threshold_;
  double frontier_threshold_;

  // flags
  bool first_plan_;

  // sim timer
  ros::Time start_time_, end_time_;
};
}

#endif
