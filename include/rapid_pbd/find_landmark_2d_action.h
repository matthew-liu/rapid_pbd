#ifndef _RAPID_PBD_FIND_LANDMARK_2D_ACTION_H_
#define _RAPID_PBD_FIND_LANDMARK_2D_ACTION_H_

#include <string>

#include "actionlib/server/simple_action_server.h"
#include "rapid_pbd_msgs/FindLandmark2DAction.h"
#include "ros/ros.h"
#include "custom_landmark_2d/matcher.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "tf/transform_listener.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "rapid_pbd/program_db.h"
#include "rapid_pbd/robot_config.h"

namespace rapid {
namespace pbd {
class FindLandmark2DAction {
 public:
  FindLandmark2DAction(const std::string& cam_info_topic, const std::string& cloud_topic,
                       const SceneDb& scene_db, const RobotConfig& robot_config, const std::string& template_dir);
  void Start();
  void Execute(const rapid_pbd_msgs::FindLandmark2DGoalConstPtr& goal);
  void Callback(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& depth);
  void CloudtoLandmark(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud, rapid_pbd_msgs::Landmark& landmark);

 private:
  int using_img;
  int updating_img;
  std::string template_dir_; // the absolute file path of the directory that stores all templates
  std::string cloud_topic_;

  cv_bridge::CvImagePtr rgb_ptr_;
  cv_bridge::CvImagePtr depth_ptr_;
  sensor_msgs::CameraInfoConstPtr cam_info_;
  sensor_msgs::PointCloud2ConstPtr cloud_ptr_;

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
