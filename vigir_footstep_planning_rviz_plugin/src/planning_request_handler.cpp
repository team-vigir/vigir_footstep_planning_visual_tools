#include <vigir_footstep_planning_rviz_plugin/planning_request_handler.h>
#include <vigir_footstep_planning_msgs/step_plan.h>
#include <QMetaType>


namespace vigir_footstep_planning_rviz_plugin
{
PlanningRequestHandler::PlanningRequestHandler(QObject* parent)
  : RequestHandlerBase( parent )
{
}

PlanningRequestHandler::~PlanningRequestHandler()
{
}

void PlanningRequestHandler::sendPlanningRequest(bool append)
{
  if(request_->planning_mode == RequestMsg::PLANNING_MODE_2D
     || request_->planning_mode == RequestMsg::PLANNING_MODE_3D)
  {
    setHeaderStamp();
    if(append)
      addStepPlan(); //current_step_plan.goal is used as new start
    else
      sendRequest();
    return;
  }
  ROS_ERROR("Could not send planning request because planning mode is not set correctly");
}

void PlanningRequestHandler::setGoal(Ogre::Vector3 position, Ogre::Quaternion orientation)
{     
  geometry_msgs::Pose pose;
  pose.orientation.x = orientation.x;
  pose.orientation.y = orientation.y;
  pose.orientation.z = orientation.z;
  pose.orientation.w = orientation.w;

  pose.position.x = position.x;
  pose.position.y = position.y;
  pose.position.z = position.z;

  if(generate_feet_ac.isServerConnected())
  {
    vigir_footstep_planning_msgs::GenerateFeetPoseGoal goal;
    goal.request.header.stamp = ros::Time::now();
    goal.request.header.frame_id = request_->header.frame_id;
    goal.request.pose = pose;
    goal.request.flags = FeetPoseRequestMsg::FLAG_3D;
    generate_feet_ac.sendGoal(goal,
                              boost::bind(&PlanningRequestHandler::goalPoseCallback, this, _1, _2),
                              GenerateFeetPoseActionClient::SimpleActiveCallback(),
                              GenerateFeetPoseActionClient::SimpleFeedbackCallback());
  }
}

void PlanningRequestHandler::goalPoseCallback(const actionlib::SimpleClientGoalState& state, const GenerateFeetPoseResult& result)
{
  if(result->status.error == ErrorStatusMsg::NO_ERROR)
  {
    if(result->status.warning != ErrorStatusMsg::NO_WARNING)
    {
      ROS_WARN("%s", result->status.warning_msg.c_str());
    }
    request_->goal = result->feet;
    Q_EMIT(goalFeetAnswer(result->feet));
  }
  else{
    ROS_ERROR("%s", result->status.error_msg.c_str());
  }
}

void PlanningRequestHandler::replanToIndex(int index)
{
  replan_goal_index = index;
  setReplanGoal(index);
}

// Set Goal for replanning. The step plan is replanned from current chosen last step or from the beginning to foot with this index.
void PlanningRequestHandler::setReplanGoal(int index)
{
  if(current_step_plan.steps.size() <= index)
  {
    ROS_ERROR("Index %i invalid, maximum %i is allowed", index, (int)current_step_plan.steps.size());
    return;
  }
  FootMsg replan_goal = current_step_plan.steps[index].foot;
  geometry_msgs::Pose pose = replan_goal.pose;

  //compute shift for orientation
  float x_shift= 0.04, y_shift = 0.11, z_shift = -0.085;
  if(replan_goal.foot_index == FootMsg::LEFT)
  {
    y_shift *= -1;
  }

  computeShift(x_shift, y_shift, z_shift, pose.orientation);
  pose.position.x += x_shift;
  pose.position.y += y_shift;
  pose.position.z += z_shift;
  if(generate_feet_ac.isServerConnected())
  {
    vigir_footstep_planning_msgs::GenerateFeetPoseGoal goal;
    goal.request.header.stamp = ros::Time::now();
    goal.request.header.frame_id = request_->header.frame_id;
    goal.request.pose = pose;
    generate_feet_ac.sendGoal(goal,
                              boost::bind(&PlanningRequestHandler::setReplanGoalCallback, this, _1, _2),
                              GenerateFeetPoseActionClient::SimpleActiveCallback(),
                              GenerateFeetPoseActionClient::SimpleFeedbackCallback());
  }
}

void PlanningRequestHandler::setReplanGoalCallback(const actionlib::SimpleClientGoalState& state, const GenerateFeetPoseResult& result)
{
  if(result->status.error == ErrorStatusMsg::NO_ERROR)
  {
    // set replan goal and replan
    request_->goal = result->feet;

    // replan
    if(last_step_index > replan_goal_index)
    {
      last_step_index = 0;
    }
    addStepPlan(); // current_step_plan ends at replan goal

  }
  else
  {
    ROS_ERROR("%s", result->status.error_msg.c_str());
  }
}

// for appending in planning request handler check if only a replan goal was set for
// replanning only a part of the step plan append "add" to the beginning and additionaly add
// all steps with step index > replan_goal_index to the step plan
void PlanningRequestHandler::appendStepPlan(StepPlanMsg add)
{
  vigir_footstep_planning::StepPlan current(current_step_plan);

  std::vector<StepMsg> end_steps;
  if(replan_goal_index > last_step_index)
  {
    for (int i = replan_goal_index + 1; i < current.size(); ++i)
    {
      //indizes verändern?
      end_steps.push_back(current_step_plan.steps[i]);
    }
  }

  for(int i = current_step_plan.steps.size()-1; i > last_step_index; --i)
  {
    ROS_INFO("remove step %i", i);
    current.removeStep(i);
  }
  // step plan to last_step_index + step plan to replan_goal (or end)
  ErrorStatusMsg error_status = current.appendStepPlan(add);
  // step plan to replan goal + last steps
  if(replan_goal_index > last_step_index)
  {
    StepPlanMsg end_step_plan = current_step_plan;
    end_step_plan.steps = end_steps;
    ErrorStatusMsg error_status2 = current.appendStepPlan(end_step_plan);
  }
  if(error_status.error == ErrorStatusMsg::NO_ERROR)
  {
    StepPlanMsg step_plan;
    current.toMsg(step_plan);
    setCurrentStepPlan(step_plan);
    Q_EMIT(createdStepPlan(step_plan));
  }
  else
    ROS_ERROR("%s", error_status.error_msg.c_str());
}

// --------------

// More Settings chosen by ui

void PlanningRequestHandler::setStartStepIndex(int start_step)
{
  switch (start_step)
  {
  case 0:
    request_->start_foot_selection = vigir_footstep_planning_msgs::StepPlanRequest::AUTO;
    break;
  case 1:
    request_->start_foot_selection = vigir_footstep_planning_msgs::StepPlanRequest::LEFT;
    break;
  case 2:
    request_->start_foot_selection = vigir_footstep_planning_msgs::StepPlanRequest::RIGHT;
    break;
  }
}

void PlanningRequestHandler::setMaxPlanningTime(double t)
{
  request_->max_planning_time = (float) t;
}

void PlanningRequestHandler::setMaxNofSteps(int noSteps)
{
  request_->max_number_steps = (float) noSteps;
}

void PlanningRequestHandler::setMaxPathLengthRatio(double ratio)
{
  request_->max_path_length_ratio = (float) ratio;
}



} // end namespace vigir_footstep_planning_rviz_plugin



