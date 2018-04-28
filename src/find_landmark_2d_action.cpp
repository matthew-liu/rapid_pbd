#include "rapid_pbd/find_landmark_2d_action.h"

#include <sstream>
#include <string>

#include "actionlib/server/simple_action_server.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "rapid_pbd/action_names.h"
#include "rapid_pbd_msgs/FindLandmark2DAction.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"

#include "rapid_pbd/program_db.h"
#include "rapid_pbd/robot_config.h"

namespace rapid {
namespace pbd {

FindLandmark2DAction::FindLandmark2DAction(
    const std::string& rgb_topic, const std::string& depth_topic,
    const std::string& cam_info_topic, const SceneDb& scene_db,
    const RobotConfig& robot_config)
    : rgb_topic_(rgb_topic),
      depth_topic_(depth_topic),
      cam_info_(ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cam_info_topic)),
      scene_db_(scene_db),
      robot_config_(robot_config),
      as_(kFindLandmark2DActionName,
          boost::bind(&FindLandmark2DAction::Execute, this, _1), false),
      matcher_(),
      nh_(),
      objects_pub_(nh_.advertise<sensor_msgs::PointCloud2>("generated_cloud", 1, true)),
      tf_listener_() {

matcher_.set_cam_model(cam_info_);
ROS_INFO("FindLandmark2DAction node starts safely, yay!\n");
}

void FindLandmark2DAction::Start() { as_.start(); }

void FindLandmark2DAction::Execute(const rapid_pbd_msgs::FindLandmark2DGoalConstPtr& goal) {}

}  // namespace pbd
}  // namespace rapid
