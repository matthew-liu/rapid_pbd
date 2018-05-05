#include <string>

#include "mongodb_store/message_store.h"
#include "rapid_pbd/program_db.h"
#include "rapid_pbd/robot_config.h"
#include "rapid_pbd/find_landmark_2d_action.h"
#include "ros/ros.h"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "sensor_msgs/Image.h"

namespace pbd = rapid::pbd;
using namespace message_filters;

int main(int argc, char** argv) {
  ros::init(argc, argv, "find_landmark_2d_action_node");
  if (argc < 4) {
    ROS_ERROR("Must supply rgb, depth, & cam_info topics as args");
    return 1;
  }
  std::string rgb_topic(argv[1]);
  std::string depth_topic(argv[2]);
  std::string cam_info_topic(argv[3]);

  std::string robot("");
  bool is_robot_specified = ros::param::get("robot", robot);
  if (!is_robot_specified) {
    ROS_ERROR("robot param must be specified.");
    return 1;
  }

  pbd::RobotConfig* robot_config;
  if (robot == "pr2") {
    robot_config = new pbd::Pr2RobotConfig();
  } else if (robot == "fetch") {
    robot_config = new pbd::FetchRobotConfig();
  } else if (robot == "baxter") {
    robot_config = new pbd::BaxterRobotConfig();
  } else {
    ROS_ERROR("Unsupported robot \"%s\"", robot.c_str());
    return 1;
  }

  ros::NodeHandle nh;
  mongodb_store::MessageStoreProxy proxy(nh, pbd::kMongoSceneCollectionName,
                                         pbd::kMongoDbName);
  pbd::SceneDb scene_db(&proxy);

  // initialize the action server
  rapid::pbd::FindLandmark2DAction action(cam_info_topic, scene_db, *robot_config);

  // set up the image topics synchronizer
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, rgb_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, depth_topic, 1);

  typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;

  Synchronizer<SyncPolicy> sync(SyncPolicy(10), rgb_sub, depth_sub);
  sync.registerCallback(boost::bind(&pbd::FindLandmark2DAction::Callback, &action, _1, _2));

  action.Start();
  ros::spin();
  return 0;
}
