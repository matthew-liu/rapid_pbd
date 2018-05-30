#include "rapid_pbd/visualizer.h"

#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/TransformStamped.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rapid_pbd/joint_state.h"
#include "rapid_pbd_msgs/EditorEvent.h"
#include "rapid_pbd_msgs/Landmark.h"
#include "rapid_pbd_msgs/Program.h"
#include "robot_markers/builder.h"
#include "sensor_msgs/PointCloud2.h"
#include "surface_perception/visualization.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "rapid_pbd/robot_config.h"
#include "rapid_pbd/world.h"

namespace msgs = rapid_pbd_msgs;
using sensor_msgs::PointCloud2;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

namespace rapid {
namespace pbd {
Visualizer::Visualizer(const SceneDb& scene_db,
                       const robot_markers::Builder& marker_builder,
                       const RobotConfig& robot_config)
    : scene_db_(scene_db),
      marker_builder_(marker_builder),
      robot_config_(robot_config),
      step_vizs_(),
      nh_() {}

void Visualizer::Init() {
  marker_builder_.Init();
  marker_builder_.SetNamespace("robot");
  marker_builder_.SetFrameId(robot_config_.base_link());
}

void Visualizer::Publish(const std::string& program_id, const World& world) {
  CreateStepVizIfNotExists(program_id);

  // Publish the robot visualization
  MarkerArray robot_markers;
  std::map<std::string, double> joint_positions;
  world.joint_state.ToMap(&joint_positions);
  marker_builder_.SetJointPositions(joint_positions);
  marker_builder_.Build(&robot_markers);
  step_vizs_[program_id].robot_pub.publish(robot_markers);

  std::string base_link(robot_config_.base_link());

  // Publish the scene
  PointCloud2 scene;
  if (world.scene_id != "" && scene_db_.Get(world.scene_id, &scene)) {
    if (world.scene_id != step_vizs_[program_id].last_scene_id) {
      step_vizs_[program_id].scene_pub.publish(scene);
    }
  } else {
    pcl::PointCloud<pcl::PointXYZRGB> blank;
    pcl::PointXYZRGB pt;
    blank.points.push_back(pt);
    pcl::toROSMsg(blank, scene);
    scene.header.frame_id = base_link;
    step_vizs_[program_id].scene_pub.publish(scene);
  }
  step_vizs_[program_id].last_scene_id = world.scene_id;

  // Publish landmark markers
  MarkerArray scene_markers;
  GetSegmentationMarker(world.surface_box_landmarks, robot_config_,
                        &scene_markers, msgs::Landmark::SURFACE_BOX);
  if (scene_markers.markers.size() > 0) {
    step_vizs_[program_id].surface_seg_pub.publish(scene_markers);
  } else {
    for (size_t i = 0; i < 100; ++i) {
      Marker blank;
      blank.ns = "segmentation";
      blank.id = i;
      blank.header.frame_id = base_link;
      blank.type = Marker::CUBE;
      blank.pose.orientation.w = 1;
      blank.scale.x = 0.05;
      blank.scale.y = 0.05;
      blank.scale.z = 0.05;
      scene_markers.markers.push_back(blank);
    }
    step_vizs_[program_id].surface_seg_pub.publish(scene_markers);
  }

  // Publish landmark_2d markers
  MarkerArray landmark_2d_markers;
  GetSegmentationMarker(world.custom_2d_landmarks, robot_config_,
                        &landmark_2d_markers, msgs::Landmark::CUSTOM_LANDMARK_2D);
  if (landmark_2d_markers.markers.size() > 0) {
    step_vizs_[program_id].landmark_2d_pub.publish(landmark_2d_markers);
  } else {
    for (size_t i = 0; i < 100; ++i) {
      Marker blank;
      blank.ns = "landmark_2d";
      blank.id = i;
      blank.header.frame_id = base_link;
      blank.type = Marker::CUBE;
      blank.pose.orientation.w = 1;
      blank.scale.x = 0.05;
      blank.scale.y = 0.05;
      blank.scale.z = 0.05;
      landmark_2d_markers.markers.push_back(blank);
    }
    step_vizs_[program_id].landmark_2d_pub.publish(landmark_2d_markers);
  }
}

void Visualizer::StopPublishing(const std::string& program_id) {
  if (step_vizs_.find(program_id) != step_vizs_.end()) {
    step_vizs_.erase(program_id);
  }
}

void Visualizer::CreateStepVizIfNotExists(const std::string& program_id) {
  // Create the publisher if it doesn't exist.
  if (step_vizs_.find(program_id) == step_vizs_.end()) {
    step_vizs_[program_id].robot_pub =
        nh_.advertise<MarkerArray>("robot/" + program_id, 10, true);
    step_vizs_[program_id].scene_pub =
        nh_.advertise<PointCloud2>("scene/" + program_id, 10, true);
    step_vizs_[program_id].surface_seg_pub = nh_.advertise<MarkerArray>(
        "surface_segmentation/" + program_id, 10, true);
    step_vizs_[program_id].landmark_2d_pub = nh_.advertise<MarkerArray>(
        "landmark_2d/" + program_id, 10, true);
    step_vizs_[program_id].last_scene_id = "";
  }
}

RuntimeVisualizer::RuntimeVisualizer(const RobotConfig& robot_config,
                                     const ros::Publisher& surface_box_pub,
                                     const ros::Publisher& landmark_2d_pub)
    : robot_config_(robot_config), surface_box_pub_(surface_box_pub),
      landmark_2d_pub_(landmark_2d_pub) {}

void RuntimeVisualizer::PublishSurfaceBoxes(
    const std::vector<rapid_pbd_msgs::Landmark>& box_landmarks) const {
  MarkerArray scene_markers;
  GetSegmentationMarker(box_landmarks, robot_config_, &scene_markers, msgs::Landmark::SURFACE_BOX);
  surface_box_pub_.publish(scene_markers);
}

void RuntimeVisualizer::PublishLandmark2D(
    const std::vector<rapid_pbd_msgs::Landmark>& custom_2d_landmarks) const {
  MarkerArray scene_markers;
  GetSegmentationMarker(custom_2d_landmarks, robot_config_, &scene_markers, msgs::Landmark::CUSTOM_LANDMARK_2D);
  landmark_2d_pub_.publish(scene_markers);
}

void GetSegmentationMarker(const std::vector<msgs::Landmark>& landmarks,
                           const RobotConfig& robot_config,
                           visualization_msgs::MarkerArray* scene_markers,
                           std::string landmark_type) {
  std::vector<surface_perception::Object> objects;
  for (size_t li = 0; li < landmarks.size(); ++li) {
    const msgs::Landmark& landmark = landmarks[li];
    surface_perception::Object object;
    object.pose_stamped = landmark.pose_stamped;
    if (landmark_type == msgs::Landmark::SURFACE_BOX) {
      object.dimensions = landmark.surface_box_dims;
    } else if (landmark_type == msgs::Landmark::CUSTOM_LANDMARK_2D) {
      object.dimensions = landmark.object_dims;
    }
    objects.push_back(object);
  }
  surface_perception::ObjectMarkers(objects, &scene_markers->markers);

  std::string object_ns;
  std::string text_ns;
  if (landmark_type == msgs::Landmark::SURFACE_BOX) {
    object_ns = "segmentation";
    text_ns = "segmentation_names";
  } else if (landmark_type == msgs::Landmark::CUSTOM_LANDMARK_2D) {
    object_ns = "landmark_2d";
    text_ns = "landmark_2d_names";
  }

  std::string base_link(robot_config.base_link());
  // make object markers
  for (size_t i = 0; i < objects.size(); ++i) {
    scene_markers->markers[i].ns = object_ns;
    scene_markers->markers[i].id = i;
  }
  // add text markers
  for (size_t i = 0; i < objects.size(); ++i) {
    Marker marker = scene_markers->markers[i];
    marker.type = Marker::TEXT_VIEW_FACING;
    marker.ns = text_ns;
    marker.text = landmarks[i].name;
    marker.pose.position.z += 0.15;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1;
    scene_markers->markers.push_back(marker);
  }
  int num_objects = objects.size();
  // ROS_INFO("markers size: %ld", objects.size());
  for (size_t i = num_objects; i < 100; ++i) {
    Marker blank;
    blank.ns = object_ns;
    blank.id = i;
    blank.header.frame_id = base_link;
    blank.type = Marker::CUBE;
    blank.pose.orientation.w = 1;
    blank.scale.x = 0.05;
    blank.scale.y = 0.05;
    blank.scale.z = 0.05;
    scene_markers->markers.push_back(blank);

    blank.ns = text_ns;
    scene_markers->markers.push_back(blank);
  }
}
}  // namespace pbd
}  // namespace rapid
