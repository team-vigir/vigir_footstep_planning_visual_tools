#include "../include/vigir_footstep_planning_rviz_plugin/step_plan_helper.h"
#include <vigir_footstep_planning_msgs/step_plan.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>

namespace vigir_footstep_planning_rviz_plugin
{

StepPlanHelper::StepPlanHelper(QObject *parent)
  : QObject( parent )
//  , edit_step_ac("/vigir/footstep_planning/edit_step", true)
  , execute_step_plan_ac("/vigir/footstep_planning/execute_step_plan", true)
  , fixed_frame_("")
  , step_edited(false)
{
  connectToActionServer();
  robot_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/robot_pose",1);
}

StepPlanHelper::~StepPlanHelper()
{
}

void StepPlanHelper::setFixedFrame(QString fixed_frame)
{
  fixed_frame_ = fixed_frame.toStdString();
}

void StepPlanHelper::setCurrentStepPlan(StepPlanMsg step_plan)
{
  current_step_plan = step_plan;
  previous_step_plans.push_back(current_step_plan);
  checkSteps();
}

void StepPlanHelper::connectToActionServer()
{

/*  if(edit_step_ac.waitForServer(ros::Duration(1,0)))
    ROS_INFO("Connected to Action Server (/vigir/footstep_planning/edit_step)");
  else
    ROS_INFO("Could not connect to Action Server (/vigir/footstep_planning/edit_step)");
*/
  if(execute_step_plan_ac.waitForServer(ros::Duration(1,0)))
    ROS_INFO("Connected to Action Server (/vigir/footstep_planning/execute_step_plan)");
  else
    ROS_INFO("Could not connect to Action Server (/vigir/footstep_planning/execute_step_plan)");
}

bool StepPlanHelper::checkConnection()
{
  //return edit_step_ac.isServerConnected();
}

// Edit Step Update Foot

void StepPlanHelper::editStep(vigir_footstep_planning_msgs::EditStep edit_step)
{
/*  if(edit_step_ac.isServerConnected())
  {
    vigir_footstep_planning_msgs::EditStepGoal goal;
    goal.step_plan = current_step_plan;
    goal.edit_step = edit_step;
    edit_step_ac.sendGoal(goal,
                          boost::bind(&StepPlanHelper::editStepCallback, this, _1, _2),
                          EditStepActionClient::SimpleActiveCallback(),
                          EditStepActionClient::SimpleFeedbackCallback());
  }
  else
    ROS_WARN("edit_step not available! Please activate an \"/vigir/footstep_planning/edit_step\" action server.");
*/
}

void StepPlanHelper::editStepCallback(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::EditStepResultConstPtr& result)
{
  if(checkForErrors(result->status))
    return;
  if(result->step_plans.size() == 1)
  {
    current_step_plan = result->step_plans[0];
    step_edited = true;

  }
  else
  {
    std::vector<StepPlanMsg> step_plans = result->step_plans;
    combineStepPlans(step_plans);
  }
  checkSteps();
}

void StepPlanHelper::checkSteps()
{
  for (int i = 0; i < current_step_plan.steps.size(); ++i)
  {
    StepMsg step = current_step_plan.steps[i];
    if(!step.valid || step.colliding)
    {
      //      ROS_ERROR("%i", i);
      Q_EMIT(stepValidUpdate(step.step_index, false));
      if(step.colliding)
        ROS_INFO("colliding");
    }
    else
      Q_EMIT(stepValidUpdate(step.step_index, true));
  }
}

void StepPlanHelper::combineStepPlans(std::vector<StepPlanMsg>& step_plans)
{
  if (step_plans.size() != 2)
  {
    ROS_WARN("Function stitchStepPlans() only meant for stitching when step is deleted.");
    return;
  }
  vigir_footstep_planning::StepPlan result(step_plans[0]);
  vigir_footstep_planning_msgs::ErrorStatus error_status;
  ROS_INFO("first step plan size: %i", step_plans[0].steps.size());
  ROS_INFO("second step plan size: %i", step_plans[1].steps.size());
  for(int i = 0; i < step_plans[1].steps.size(); ++i)
  {
    step_plans[1].steps[i].step_index = i;
    ROS_INFO("set index of 2nd plan to %i", step_plans[1].steps[i].step_index);
  }
  error_status = result.appendStepPlan(step_plans[1]);
  if(checkForErrors(error_status))
    return;

  result.toMsg(current_step_plan);
  previous_step_plans.push_back(current_step_plan);
  step_edited = false;
  ROS_INFO("size prev step_plan: %i", previous_step_plans.size());
  Q_EMIT(createdStepPlan(current_step_plan));
}

bool StepPlanHelper::checkForErrors(ErrorStatusMsg error_status)
{
  if(!error_status.error == ErrorStatusMsg::NO_ERROR)
  {
    ROS_ERROR("%s", error_status.error_msg.c_str());
    return true;
  }
  if(error_status.warning != ErrorStatusMsg::NO_WARNING)
  {
    ROS_WARN("%s", error_status.warning_msg.c_str());
  }
  return false;
}

void StepPlanHelper::addStep(const std::string& frame_id, const Ogre::Vector3& position, const Ogre::Quaternion& orientation, unsigned int which_foot, unsigned int step_index)
{
  //todo: EditStep benutzen?
  StepMsg step;
  geometry_msgs::Pose pose;
  pose.position.x = position.x; pose.position.y = position.y; pose.position.z = position.z;
  pose.orientation.w = orientation.w; pose.orientation.x = orientation.x; pose.orientation.y = orientation.y; pose.orientation.z = orientation.z;
  step.step_index = step_index;
  step.foot.foot_index = which_foot;
  step.foot.pose = pose;
  step.header.frame_id = frame_id;
  step.header.stamp = ros::Time::now();
  step.foot.header = step.header;

  vigir_footstep_planning::StepPlan current(current_step_plan);
  current.insertStep(step);
  current.toMsg(current_step_plan);
}

void StepPlanHelper::setRobotPose(Ogre::Vector3 position, Ogre::Quaternion orientation)
{
  geometry_msgs::PoseStamped robot_pose;
  robot_pose.header.stamp = ros::Time::now();
  robot_pose.header.frame_id = fixed_frame_;

  robot_pose.pose.position.x = position.x;
  robot_pose.pose.position.y = position.y;
  robot_pose.pose.position.z = position.z;
  robot_pose.pose.orientation.x = orientation.x;
  robot_pose.pose.orientation.y = orientation.y;
  robot_pose.pose.orientation.z = orientation.z;
  robot_pose.pose.orientation.w = orientation.w;

  if(ros::ok())
  {
    robot_pose_publisher.publish(robot_pose);
  }
}

void StepPlanHelper::setPreviousStepPlan()
{
  if(previous_step_plans.size() > 1 )
  {
    if(!step_edited)
      previous_step_plans.pop_back();
    current_step_plan = previous_step_plans.back();
    step_edited = false;
  }
  Q_EMIT(createdStepPlan(current_step_plan));
}

void StepPlanHelper::acceptModifiedStepPlan()
{
  previous_step_plans.push_back(current_step_plan);
  step_edited=false;
  Q_EMIT(createdStepPlan(current_step_plan));
}

void StepPlanHelper::executeStepPlan()
{
  /*
  if(execute_step_plan_ac.isServerConnected())
  {
    vigir_footstep_planning_msgs::ExecuteStepPlanGoal goal;
    goal.step_plan = current_step_plan;
    execute_step_plan_ac.sendGoal(goal,
                          boost::bind(&StepPlanHelper::executeStepPlanCallback, this, _1, _2),
                          ExecuteStepPlanActionClient::SimpleActiveCallback(),
                          ExecuteStepPlanActionClient::SimpleFeedbackCallback());
  }
  else
    ROS_WARN("execute_step_plan not available! Please activate an \"/vigir/footstep_planning/execute_step_plan\" action server.");
*/
}

void StepPlanHelper::executeStepPlanCallback(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::ExecuteStepPlanResultConstPtr& result)
{
}


} // end namespace vigir_footstep_planning_rviz_plugin



