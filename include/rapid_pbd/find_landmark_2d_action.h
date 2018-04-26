#ifndef _RAPID_PBD_FIND_LANDMARK_2D_ACTION_H_
#define _RAPID_PBD_FIND_LANDMARK_2D_ACTION_H_

#include <string>

#include "actionlib/server/simple_action_server.h"
#include "rapid_pbd_msgs/FindLandmark2DAction.h"
#include "ros/ros.h"
#include "custom_landmark_2d/matcher.h"
#include "sensor_msgs/CameraInfo.h"
#include "tf/transform_listener.h"

#include "rapid_pbd/program_db.h"
#include "rapid_pbd/robot_config.h"

namespace rapid {
namespace pbd {
class FindLandmark2DAction {
 public:
  FindLandmark2DAction(const std::string& rgb_topic, const std::string& depth_topic,
                       const std::string& cam_info_topic, const SceneDb& scene_db,
                       const RobotConfig& robot_config);
  void Start();
  void Execute(const rapid_pbd_msgs::FindLandmark2DGoalConstPtr& goal);

 private:
  std::string rgb_topic_;
  std::string depth_topic_;
  sensor_msgs::CameraInfoConstPtr cam_info_;

  SceneDb scene_db_;
  const RobotConfig& robot_config_;
  actionlib::SimpleActionServer<rapid_pbd_msgs::FindLandmark2DAction> as_;
  custom_landmark_2d::Matcher matcher_;
  ros::NodeHandle nh_;
  ros::Publisher objects_pub_;
  tf::TransformListener tf_listener_;
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_FIND_LANDMARK_2D_ACTION_H_
