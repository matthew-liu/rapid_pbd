#include "rapid_pbd/find_landmark_2d_action.h"

#include <sstream>
#include <string>

#include "actionlib/server/simple_action_server.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "rapid_pbd/action_names.h"
#include "rapid_pbd_msgs/FindLandmark2DAction.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"
#include "cv_bridge/cv_bridge.h"

#include "rapid_pbd/program_db.h"
#include "rapid_pbd/robot_config.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace rapid {
namespace pbd {

FindLandmark2DAction::FindLandmark2DAction(
    const std::string& cam_info_topic, const SceneDb& scene_db,
    const RobotConfig& robot_config)
    : cam_info_(ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cam_info_topic)),
      scene_db_(scene_db),
      robot_config_(robot_config),
      as_(kFindLandmark2DActionName,
          boost::bind(&FindLandmark2DAction::Execute, this, _1), false),
      matcher_(),
      nh_(),
      objects_pub_(nh_.advertise<sensor_msgs::PointCloud2>("generated_cloud", 1, true)),
      tf_listener_(),
      template_dir_("/home/liux44/catkin_ws_indigo/src/") {

matcher_.set_cam_model(cam_info_);
}

void FindLandmark2DAction::Start() { as_.start(); }

void FindLandmark2DAction::Execute(const rapid_pbd_msgs::FindLandmark2DGoalConstPtr& goal) {
    std::string object_name = goal->object_name;
    std::string temp_path;
    temp_path.append(template_dir_).append(object_name).append(".jpg");

    rapid_pbd_msgs::FindLandmark2DResult result;

    // Load template image
    cv::Mat templ = cv::imread( temp_path, 1 );
    if ( !templ.data ) {
        ROS_ERROR("No template image found in path: %s\n", temp_path.c_str());
        as_.setAborted(result);
        return;
    }
    matcher_.set_template(templ);
    matcher_.match_limit = goal->match_limit;

    std::vector<PointCloudC::Ptr> object_clouds;
    while(updating_img) {} // keep waiting
    using_img = 1;
    matcher_.Match(rgb_ptr_->image, depth_ptr_->image, &object_clouds);
    using_img = 0;
    ROS_INFO("#matched 3D objects: %lu", object_clouds.size());

    // concatenate the vector of pointClouds into a single one & publish it
    PointCloudC::Ptr pcl_cloud(new PointCloudC());
    for (std::vector<PointCloudC::Ptr>::iterator it = object_clouds.begin(); it != object_clouds.end(); it++) {
      *pcl_cloud += **it;
    }
    ROS_INFO("pcl cloud size: %lu\n", pcl_cloud->size());

    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*pcl_cloud, ros_cloud);
    ros_cloud.header.frame_id = cam_info_->header.frame_id; // head_camera_rgb_optical_frame

    objects_pub_.publish(ros_cloud);
    as_.setSucceeded(result);
}

void FindLandmark2DAction::Callback(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& depth) {
    // convert sensor_msgs::Images to cv::Mats
    if (using_img) return;
    updating_img = 1;
    try {
      rgb_ptr_ = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8);
      depth_ptr_ = cv_bridge::toCvCopy(depth); // 32FC1
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    updating_img = 0;
}

}  // namespace pbd
}  // namespace rapid
