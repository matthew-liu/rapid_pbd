#include "rapid_pbd/program_executor.h"

#include <sstream>

#include "rapid_pbd_msgs/Action.h"
#include "rapid_pbd_msgs/ExecuteProgramAction.h"
#include "rapid_pbd_msgs/FreezeArm.h"
#include "rapid_pbd_msgs/Program.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"

#include "rapid_pbd/action_names.h"
#include "rapid_pbd/motion_planning_constants.h"
#include "rapid_pbd/program_db.h"
#include "rapid_pbd/runtime_robot_state.h"
#include "rapid_pbd/step_executor.h"
#include "rapid_pbd/visualizer.h"
#include "rapid_pbd/world.h"

using rapid_pbd_msgs::Action;
using rapid_pbd_msgs::ExecuteProgramFeedback;
using rapid_pbd_msgs::ExecuteProgramGoal;
using rapid_pbd_msgs::ExecuteProgramResult;
using rapid_pbd_msgs::FreezeArm;
using rapid_pbd_msgs::Program;
using rapid_pbd_msgs::Step;

namespace rapid {
namespace pbd {
ProgramExecutionServer::ProgramExecutionServer(
    const std::string& action_name, const ros::Publisher& is_running_pub,
    ActionClients* action_clients, const RuntimeRobotState& robot_state,
    const RuntimeVisualizer& runtime_viz, const ProgramDb& program_db,
    const ros::Publisher& planning_scene_pub)
    : nh_(),
      server_(action_name,
              boost::bind(&ProgramExecutionServer::Execute, this, _1), false),
      freeze_arm_client_(nh_.serviceClient<FreezeArm>(kFreezeArmService)),
      is_running_pub_(is_running_pub),
      action_clients_(action_clients),
      robot_state_(robot_state),
      runtime_viz_(runtime_viz),
      program_db_(program_db),
      planning_scene_pub_(planning_scene_pub) {}

void ProgramExecutionServer::Start() {
  server_.start();
  PublishIsRunning(false);
}

void ProgramExecutionServer::Execute(
    const rapid_pbd_msgs::ExecuteProgramGoalConstPtr& goal) {
  Program program;
  if (goal->db_id != "") {
    bool success = program_db_.Get(goal->db_id, &program);
    if (!success) {
      std::string error("Unable to find program with db_id: " + goal->db_id);
      Cancel(error);
      Finish();
      ros::spinOnce();
      return;
    }
  } else if (goal->name != "") {
    bool success = program_db_.GetByName(goal->name, &program);
    if (!success) {
      std::string error("Unable to find program with name: " + goal->name);
      Cancel(error);
      Finish();
      ros::spinOnce();
      return;
    }
  } else {
    program = goal->program;
  }

  if (!IsValid(program)) {
    std::string error("Program \"" + program.name +
                      "\" was not constructed properly.");
    ExecuteProgramResult result;
    result.error = error;
    server_.setAborted(result, error);
    return;
  }

  PublishIsRunning(true);

  // Enable controllers.

  std::string robot("");
  bool is_robot_specified = ros::param::get("robot", robot);
  if (!is_robot_specified) {
    std::string msg("robot param must be specified. Program \"" + program.name +
                    "\" was preempted.");
    Cancel(msg);
    Finish();
    ros::spinOnce();
    return;
  }
  if (robot == "pr2" || robot == "fetch") {
    while (ros::ok() &&
           !freeze_arm_client_.waitForExistence(ros::Duration(5))) {
      ROS_WARN("Waiting for freeze arm service.");
    }
    FreezeArm::Request req;
    FreezeArm::Response res;
    req.actuator_group = Action::LEFT_ARM;
    freeze_arm_client_.call(req, res);
    req.actuator_group = Action::RIGHT_ARM;
    freeze_arm_client_.call(req, res);
  }

  World world;
  runtime_viz_.PublishSurfaceBoxes(world.surface_box_landmarks);
  runtime_viz_.PublishLandmark2D(world.custom_2d_landmarks);
  std::vector<boost::shared_ptr<StepExecutor> > executors;
  for (size_t i = 0; i < program.steps.size(); ++i) {
    const Step& step = program.steps[i];
    boost::shared_ptr<StepExecutor> executor(
        new StepExecutor(step, action_clients_, robot_state_, &world,
                         runtime_viz_, planning_scene_pub_));
    executors.push_back(executor);
    executors.back()->Init();
  }

  for (size_t i = 0; i < program.steps.size(); ++i) {
    ExecuteProgramFeedback feedback;
    feedback.step_number = i;
    server_.publishFeedback(feedback);

    std::string error("");
    error = executors[i]->Start();
    if (error != "") {
      executors[i]->Cancel();
      Cancel(error);
      Finish();
      ros::spinOnce();
      return;
    }
    while (!executors[i]->IsDone(&error)) {
      if (server_.isPreemptRequested() || !ros::ok()) {
        executors[i]->Cancel();
        std::string msg("Program \"" + program.name + "\" was preempted.");
        Cancel(msg);
        Finish();
        ros::spinOnce();
        return;
      }
      if (error != "") {
        executors[i]->Cancel();
        Cancel(error);
        Finish();
        ros::spinOnce();
        return;
      }
      ros::spinOnce();
    }
    if (error != "") {
      Cancel(error);
      Finish();
      ros::spinOnce();
      return;
    }
  }

  Finish();
  server_.setSucceeded();
}

bool ProgramExecutionServer::IsValid(const rapid_pbd_msgs::Program& program) {
  for (size_t i = 0; i < program.steps.size(); ++i) {
    const Step& step = program.steps[i];
    if (!StepExecutor::IsValid(step)) {
      return false;
    }
  }
  return true;
}

void ProgramExecutionServer::PublishIsRunning(bool is_running) {
  std_msgs::Bool msg;
  msg.data = is_running;
  is_running_pub_.publish(msg);
}

void ProgramExecutionServer::Cancel(const std::string& error) {
  ExecuteProgramResult result;
  result.error = error;
  server_.setAborted(result, error);
}

void ProgramExecutionServer::Finish() {
  moveit_msgs::CollisionObject surface;
  surface.id = kCollisionSurfaceName;
  surface.operation = moveit_msgs::CollisionObject::REMOVE;

  moveit_msgs::PlanningScene scene;
  scene.world.collision_objects.push_back(surface);
  scene.is_diff = true;
  planning_scene_pub_.publish(scene);

  PublishIsRunning(false);
}
}  // namespace pbd
}  // namespace rapid
