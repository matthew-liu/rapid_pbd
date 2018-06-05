#include "rapid_pbd/find_landmark_2d_action.h"

#include <sstream>
#include <string>

#include "actionlib/server/simple_action_server.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/common/common.h"
#include "rapid_pbd/action_names.h"
#include "rapid_pbd_msgs/FindLandmark2DAction.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"
#include "cv_bridge/cv_bridge.h"

#include "rapid_pbd/program_db.h"
#include "rapid_pbd/robot_config.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace rapid {
namespace pbd {

FindLandmark2DAction::FindLandmark2DAction(
    const std::string& cam_info_topic, const std::string& cloud_topic,
    const SceneDb& scene_db, const RobotConfig& robot_config, const std::string& template_dir)
    : cam_info_(ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cam_info_topic)),
      cloud_topic_(cloud_topic),
      scene_db_(scene_db),
      robot_config_(robot_config),
      as_(kFindLandmark2DActionName,
          boost::bind(&FindLandmark2DAction::Execute, this, _1), false),
      matcher_(),
      nh_(),
      objects_pub_(nh_.advertise<sensor_msgs::PointCloud2>("generated_cloud", 1, true)),
      tf_listener_(),
      template_dir_(template_dir) {

matcher_.set_cam_model(cam_info_);
}

void FindLandmark2DAction::Start() { as_.start(); }

void FindLandmark2DAction::CloudtoLandmark(const PointCloudC::Ptr cloud, rapid_pbd_msgs::Landmark& landmark) {
  // The pose of the object
  //
  // The origin is at the center of the oriented bounding box around the
  // object. The z direction points "up" relative to the surface, while the x
  // and y directions are parallel to the sides of the box. There are no other
  // constraints for which direction is x or y.
  geometry_msgs::PoseStamped pose_stamped;

  pose_stamped.header.frame_id = robot_config_.base_link();
  pose_stamped.pose.orientation.w = 1;
  pose_stamped.pose.orientation.x = 0;
  pose_stamped.pose.orientation.y = 0;

  // The dimensions of the oriented bounding box around the object.
  //
  // The dimensions correspond to the x/y/z axis directions of the pose.
  geometry_msgs::Vector3 dimensions;

  PointC minPt, maxPt;
  pcl::getMinMax3D(*cloud, minPt, maxPt);

  pose_stamped.pose.position.x = (maxPt.x + minPt.x) / 2;
  pose_stamped.pose.position.y = (maxPt.y + minPt.y) / 2;
  pose_stamped.pose.position.z = (maxPt.z + minPt.z) / 2;

  dimensions.x = maxPt.x - minPt.x;
  dimensions.y = maxPt.y - minPt.y;
  dimensions.z = maxPt.z - minPt.z;

  landmark.pose_stamped = pose_stamped;
  landmark.object_dims = dimensions;
}

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
    
    ROS_INFO("current match limit: %f", matcher_.match_limit);

    std::vector<PointCloudC::Ptr> object_clouds;
    while(updating_img) {} // keep waiting
    using_img = 1;
    if (!rgb_ptr_) {
      ROS_ERROR("Failed to get rgb image");
      as_.setAborted(result);
      return;
    }
    if (!depth_ptr_) {
      ROS_ERROR("Failed to get depth image");
      as_.setAborted(result);
      return;
    }
    matcher_.Match(rgb_ptr_->image, depth_ptr_->image, &object_clouds);
    using_img = 0;
    ROS_INFO("#matched 3D objects: %lu", object_clouds.size());

    // look up object transform
    std::string base_link(robot_config_.base_link());
    tf_listener_.waitForTransform(base_link, cam_info_->header.frame_id,
                                 ros::Time(0), ros::Duration(5.0));
    tf::StampedTransform object_transform;
    try {
      tf_listener_.lookupTransform(base_link, cam_info_->header.frame_id,
                                  ros::Time(0), object_transform);
    } catch (tf::TransformException& e) {
      ROS_ERROR("%s", e.what());
      as_.setAborted(result, std::string(e.what()));
      return;
    }

    Eigen::Affine3d affine;
    tf::transformTFToEigen(object_transform, affine);

    // PointCloudC::Ptr pcl_cloud(new PointCloudC());
    int num = 0;
    for (std::vector<PointCloudC::Ptr>::iterator it = object_clouds.begin(); it != object_clouds.end(); it++) {
      PointCloudC::Ptr object_cloud(new PointCloudC());
      pcl::transformPointCloud(**it, *object_cloud, affine);

      // *pcl_cloud += *object_cloud;

      rapid_pbd_msgs::Landmark landmark;
      std::stringstream ss;
      ss << object_name << "_" << num++;
      landmark.name = ss.str();
      landmark.type = rapid_pbd_msgs::Landmark::CUSTOM_LANDMARK_2D;
      CloudtoLandmark(object_cloud, landmark);

      result.landmarks.push_back(landmark);
    }
    // sensor_msgs::PointCloud2 ros_cloud;
    // pcl::toROSMsg(*pcl_cloud, ros_cloud);
    // ros_cloud.header.frame_id = base_link;
    // objects_pub_.publish(ros_cloud);

    // Save cloud if requested
    if (goal->save_cloud) {
      if (!cloud_ptr_) {
        ROS_ERROR("Failed to get point cloud on topic: %s.", cloud_topic_.c_str());
        as_.setAborted(result);
        return;
      }
      // look up cloud transform
      tf_listener_.waitForTransform(base_link, cloud_ptr_->header.frame_id,
                                   ros::Time(0), ros::Duration(5.0));
      tf::StampedTransform cloud_transform;
      try {
        tf_listener_.lookupTransform(base_link, cloud_ptr_->header.frame_id,
                                    ros::Time(0), cloud_transform);
      } catch (tf::TransformException& e) {
        ROS_ERROR("%s", e.what());
        as_.setAborted(result, std::string(e.what()));
        return;
      }

      // transform frame & turn into pcl cloud
      sensor_msgs::PointCloud2 cloud_msg;
      pcl_ros::transformPointCloud(robot_config_.base_link(), cloud_transform, *cloud_ptr_,
                                   cloud_msg);
      PointCloudC::Ptr cloud(new PointCloudC);
      pcl::fromROSMsg(cloud_msg, *cloud);

      // downsample the pcl cloud, turn it back to cloud_msg, and save it
      PointCloudC::Ptr downsampled_cloud(new PointCloudC());
      pcl::VoxelGrid<PointC> vox;
      vox.setInputCloud(cloud);
      float leaf_size = 0.01;
      ros::param::param<float>("vox_leaf_size", leaf_size, 0.01);
      vox.setLeafSize(leaf_size, leaf_size, leaf_size);
      vox.filter(*downsampled_cloud);
      sensor_msgs::PointCloud2 downsampled_cloud_msg;
      pcl::toROSMsg(*downsampled_cloud, downsampled_cloud_msg);
      // result.cloud_db_id = scene_db_.Insert(downsampled_cloud_msg);
    }

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
    cloud_ptr_ = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(cloud_topic_, ros::Duration(10.0));
    updating_img = 0;
}

}  // namespace pbd
}  // namespace rapid
