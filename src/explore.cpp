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

#include <tghm_explore/explore.h>

#include <thread>

inline static bool operator==(const geometry_msgs::Point& one,
                              const geometry_msgs::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace explore
{
Explore::Explore()
  : private_nh_("~")
  , tf_listener_(ros::Duration(10.0))
  , costmap_client_(private_nh_, relative_nh_, &tf_listener_)
  , move_base_client_("move_base", true)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  private_nh_.param("planner_frequency", planner_frequency_, 1.0);
  private_nh_.param("progress_timeout", timeout, 30.0);
  progress_timeout_ = ros::Duration(timeout);
  private_nh_.param("visualize", visualize_, false);
  private_nh_.param("lambda", lambda_, 1e-3);
  private_nh_.param("sensor_max_range", sensor_max_range_, 1e-3);
  private_nh_.param("unknown_threshold", unknown_threshold_, 1e-3);

  tghm_ = tghm::TGHM(costmap_client_.getCostmap(),
                     lambda_,
                     sensor_max_range_,
                     unknown_threshold_,
                     costmap_client_.getRobotPose().position);

  if (visualize_) {
    marker_array_publisher_ =
        private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
  }

  ROS_INFO("Waiting to connect to move_base server");
  move_base_client_.waitForServer();
  ROS_INFO("Connected to move_base server");

  start_time_ = ros::Time::now();
  ROS_INFO_STREAM("START: " << start_time_);

  exploring_timer_ =
      relative_nh_.createTimer(ros::Duration(1. / planner_frequency_),
                               [this](const ros::TimerEvent&) { makePlan(); });
}

Explore::~Explore()
{
  stop();
}

void Explore::visualizeNodes(const std::vector<tghm::TopologyNode>& map,
                             const geometry_msgs::Point& target)
{
  std_msgs::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;
  std_msgs::ColorRGBA purple;
  purple.r = 1.0;
  purple.g = 0;
  purple.b = 1.0;
  purple.a = 1.0;

  visualization_msgs::MarkerArray markers_msg;
  std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
  visualization_msgs::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
  m.lifetime = ros::Duration(0);
  m.frame_locked = true;

  m.action = visualization_msgs::Marker::ADD;
  size_t id = 0;
  for (auto& node : map) {
    m.type = visualization_msgs::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = node.centroid;
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;
    m.points = {};
    if (node.centroid == target_position) {
      m.color = purple;
    } else {
      m.color = green;
    }
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_.publish(markers_msg);
}

void Explore::makePlan()
{
  if (status_ == ACTIVE) {
    return;
  }

  auto pose = costmap_client_.getRobotPose();

  tghm::TopologyNode target_node = tghm_.getNextGoal();

  if (target_node.score == -9999) {
    return;
    stop();
  }

  geometry_msgs::Point target_position = target_node.centroid;

  if (visualize_) {
    visualizeNodes(tghm_.getTopologicalMapVector(), target_position);
  }

  // send goal to move_base if we have something new to pursue
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.pose.position = target_position;
  goal.target_pose.pose.orientation.w = 1.;
  goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.target_pose.header.stamp = ros::Time::now();
  move_base_client_.sendGoal(
      goal, [this, target_position](
                const actionlib::SimpleClientGoalState& status_,
                const move_base_msgs::MoveBaseResultConstPtr& result) {
        reachedGoal(status_, result, target_position);
      });
}

void Explore::reachedGoal(const actionlib::SimpleClientGoalState& status_,
                          const move_base_msgs::MoveBaseResultConstPtr&,
                          const geometry_msgs::Point& frontier_goal)
{
  ROS_DEBUG("Reached goal with status: %s", status_.toString().c_str());
}

void Explore::start()
{
  exploring_timer_.start();
}

void Explore::stop()
{
  move_base_client_.cancelAllGoals();
  exploring_timer_.stop();
  ROS_INFO("Exploration stopped.");

  end_time_ = ros::Time::now();
  ros::Duration duration = end_time_ - start_time_;
  ROS_INFO_STREAM("DURATION OF EXPLORATION: " << duration.toSec() << "secs");
}

}  // namespace explore

int main(int argc, char** argv)
{
  ros::init(argc, argv, "explore");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Info)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  explore::Explore explore;
  ros::spin();

  return 0;
}
