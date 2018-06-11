#include "../include/vigir_footstep_planning_rviz_plugin/step_plan_helper.h"
#include <vigir_footstep_planning_msgs/step_plan.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>

namespace vigir_footstep_planning_rviz_plugin
{

StepPlanHelper::StepPlanHelper(QObject *parent)
  : QObject( parent )
  , edit_step_ac("edit_step", true)
  , execute_step_plan_ac("/johnny5/step_control_module/execute_step_plan", true)
  , update_step_plan_ac("update_step_plan", true)
  , update_foot_ac("update_foot")
  //, set_step_plan_ac(0)//("", true)
 // , get_step_plan_ac(0)//("", true)
  , frame_id("")
  , execution_state(-1)
  , last_performed_step(-1)
{
  robot_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/robot_pose",1);
}

StepPlanHelper::~StepPlanHelper()
{
}

void StepPlanHelper::setFrameID(QString frameID)
{
  frame_id = frameID.toStdString();
}

void StepPlanHelper::setCurrentStepPlan(StepPlanMsg step_plan)
{
  current_step_plan = step_plan;
  previous_step_plans.push_back(current_step_plan);
  checkSteps();
  updateStepPlanPositions();
}



void StepPlanHelper::connectToActionServer()
{
  if(edit_step_ac.waitForServer(ros::Duration(1,0)))
  {
    Q_EMIT(actionClientConnected("edit_step", true));
  }
  else
    Q_EMIT(actionClientConnected("edit_step", false));

  bool connected = execute_step_plan_ac.waitForServer(ros::Duration(1,0));
  if(connected)
    Q_EMIT(actionClientConnected("/johnny5/step_control_module/execute_step_plan", true));
  else
    Q_EMIT(actionClientConnected("/johnny5/step_control_module/execute_step_plan", false));

  if(update_step_plan_ac.waitForServer(ros::Duration(1,0)))
    Q_EMIT(actionClientConnected("update_step_plan", true));
  else
    Q_EMIT(actionClientConnected("update_step_plan", false));

  if(update_foot_ac.waitForServer(ros::Duration(1,0)))
    Q_EMIT(actionClientConnected("update_foot", true));
  else
    Q_EMIT(actionClientConnected("update_foot", false));
}

bool StepPlanHelper::checkConnection()
{
  return edit_step_ac.isServerConnected();
}

// Edit Step Update Foot

void StepPlanHelper::editStep(vigir_footstep_planning_msgs::EditStep edit_step)
{
  if(edit_step_ac.isServerConnected())
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
    ROS_WARN("edit_step not available! Please activate an \"/johnny5/footstep_planning/edit_step\" action server.");

}

void StepPlanHelper::editStepCallback(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::EditStepResultConstPtr& result)
{
  if(checkForErrors(result->status))
    return;
  if(result->step_plans.size() == 1)
  {
    current_step_plan = result->step_plans[0];
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
    ROS_WARN("Function combineStepPlans() only meant for stitching when one step is deleted.");
    return;
  }
  vigir_footstep_planning::StepPlan result(step_plans[0]);
  vigir_footstep_planning_msgs::ErrorStatus error_status;
  ROS_INFO("first step plan size: %i", (int)step_plans[0].steps.size());
  ROS_INFO("second step plan size: %i", (int)step_plans[1].steps.size());
  for(int i = 0; i < step_plans[1].steps.size(); ++i)
  {
    step_plans[1].steps[i].step_index = i;
    ROS_INFO("set index of 2nd plan to %i", (int) step_plans[1].steps[i].step_index);
  }
  error_status = result.appendStepPlan(step_plans[1]);
  if(checkForErrors(error_status))
    return;

  result.toMsg(current_step_plan);
  previous_step_plans.push_back(current_step_plan);
  ROS_INFO("size prev step_plan: %i", (int)previous_step_plans.size());
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
  robot_pose.header.frame_id = frame_id;

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
    previous_step_plans.pop_back();
    StepPlanMsg step_plan = previous_step_plans.back();
    if(step_plan.steps.size() == current_step_plan.steps.size())
      Q_EMIT(updatedStepPlan(step_plan));
    else
      Q_EMIT(createdStepPlan(step_plan));

    current_step_plan = step_plan;
  }
}

void StepPlanHelper::acceptModifiedStepPlan()
{
  previous_step_plans.push_back(current_step_plan);
  Q_EMIT(updatedStepPlan(current_step_plan));
}

void StepPlanHelper::executeStepPlan()
{
  if(execute_step_plan_ac.isServerConnected())
  {
    Q_EMIT(displayInfo(QString("step plan sent to robot")));
    vigir_footstep_planning_msgs::ExecuteStepPlanGoal goal;
    goal.step_plan = current_step_plan;
    execute_step_plan_ac.sendGoal(goal,
                          boost::bind(&StepPlanHelper::executeStepPlanCallback, this, _1, _2),
                          ExecuteStepPlanActionClient::SimpleActiveCallback(),
                          boost::bind(&StepPlanHelper::executeStepPlanFeedbackCallback, this, _1));
  }
  else
    ROS_WARN("execute_step_plan not available! Please activate an \"/johnny5/footstep_planning/execute_step_plan\" action server.");

}

void StepPlanHelper::executeStepPlanCallback(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::ExecuteStepPlanResultConstPtr& result)
{
  execution_state = -1;

  if(result->status.status==vigir_footstep_planning_msgs::FootstepExecutionStatus::NO_ERROR)
  {
    Q_EMIT(displayInfo("Execution of Step Plan finished without Error."));
  }

  switch(result->controller_state)
  {
  case 0: //NOT_READY
    Q_EMIT(displayInfo(QString("not ready.")));
    break;
  case 1: // READY
    Q_EMIT(displayInfo(QString("ready.")));
    break;
  case 2: // ACTIVE
    Q_EMIT(displayInfo(QString("active.")));
    break;
  case 3: // PAUSED
    Q_EMIT(displayInfo(QString("paused.")));
    break;
  case 4: // FINISHED
    Q_EMIT(displayInfo(QString("finished.")));
    break;
  case 5: // FAILED
    Q_EMIT(displayError(QString("Failed to execute Step Plan.")));
    break;
  }

}

void StepPlanHelper::executeStepPlanFeedbackCallback(const vigir_footstep_planning_msgs::ExecuteStepPlanFeedbackConstPtr& feedback)
{
  if(execution_state != feedback->controller_state)
  {

    switch(feedback->controller_state)
    {
    case 0: //NOT_READY
      Q_EMIT(displayInfo(QString("not ready.")));
      break;
    case 1: // READY
      Q_EMIT(displayInfo(QString("ready.")));
      break;
    case 2: // ACTIVE
      Q_EMIT(displayInfo(QString("active.")));
      break;
    case 3: // PAUSED
      Q_EMIT(displayInfo(QString("paused.")));
      break;
    case 4: // FINISHED
      Q_EMIT(displayInfo(QString("finished.")));
      break;
    case 5: // FAILED
      Q_EMIT(displayError(QString("Failed to execute Step Plan.")));
      break;
    }

    execution_state = feedback->controller_state;
  }
  if(!(feedback->last_performed_step_index == last_performed_step))
  {
    QString message = QStringLiteral("Completed Step %1").arg(feedback->last_performed_step_index)
        + QStringLiteral(" of %1").arg(current_step_plan.steps.size()-1);
    Q_EMIT(displayInfo(message));
    last_performed_step = feedback->last_performed_step_index;
  }
}

void StepPlanHelper::updateStepPlan(vigir_footstep_planning_msgs::UpdateMode update_mode)
{
  if(update_step_plan_ac.isServerConnected())
  {
    vigir_footstep_planning_msgs::UpdateStepPlanGoal goal;
    goal.step_plan = current_step_plan;
    goal.update_mode = update_mode;
    update_step_plan_ac.sendGoal(goal,
                          boost::bind(&StepPlanHelper::updateStepPlanCallback, this, _1, _2),
                          UpdateStepPlanActionClient::SimpleActiveCallback(),
                          UpdateStepPlanActionClient::SimpleFeedbackCallback());
  }
  else
  {
    ROS_INFO("update step plan server not connected");
  }
}

void StepPlanHelper::updateStepPlanCallback(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::UpdateStepPlanResultConstPtr& result)
{
  ROS_INFO("updateStepPlanCallback();");
  if(checkForErrors(result->status))
    return;

  current_step_plan = result->step_plan;
  previous_step_plans.push_back(result->step_plan);
  checkSteps();
  Q_EMIT(updatedStepPlan(result->step_plan));
}

void StepPlanHelper::updateStepPlanPositions()
{
  if(update_positions)
  {
    vigir_footstep_planning_msgs::UpdateMode update_mode;
    update_mode.mode = vigir_footstep_planning_msgs::UpdateMode::UPDATE_MODE_3D;
    updateStepPlan(update_mode);
  }
  else
  {
    acceptModifiedStepPlan();
  }
}

void StepPlanHelper::setUpdateStepPlanPositions(bool update)
{
  this->update_positions = update;
}


void StepPlanHelper::updateStepPlanCost()
{
  vigir_footstep_planning_msgs::UpdateMode update_mode;
  update_mode.mode = vigir_footstep_planning_msgs::UpdateMode::UPDATE_MODE_COST;
  updateStepPlan(update_mode);
}





} // end namespace vigir_footstep_planning_rviz_plugin



