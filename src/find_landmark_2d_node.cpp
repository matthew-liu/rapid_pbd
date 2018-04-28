#include <string>

#include "mongodb_store/message_store.h"
#include "rapid_pbd/program_db.h"
#include "rapid_pbd/robot_config.h"
#include "rapid_pbd/find_landmark_2d_action.h"
#include "ros/ros.h"

namespace pbd = rapid::pbd;

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

  rapid::pbd::FindLandmark2DAction action(rgb_topic, depth_topic, cam_info_topic, scene_db, *robot_config);
  action.Start();
  ros::spin();
  return 0;
}
