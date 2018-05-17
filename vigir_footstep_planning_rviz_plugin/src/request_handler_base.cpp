#include <vigir_footstep_planning_rviz_plugin/request_handler_base.h>
#include <QMetaType>
#include <vigir_footstep_planning_msgs/step_plan.h>


namespace vigir_footstep_planning_rviz_plugin
{

RequestHandlerBase::RequestHandlerBase(QObject* parent)
  : QObject( parent )
  , ac("/johnny5/footstep_planning/step_plan_request", true)
  , generate_feet_ac("/johnny5/footstep_planning/generate_feet_pose", true)
  , last_step_index(0)
  , replan_goal_index(0)
{
  request_ = new vigir_footstep_planning_msgs::StepPlanRequest();
  setFrameID(""); //default
  connectToActionServer();
  feedback_requested_=false;
  qRegisterMetaType<vigir_footstep_planning_msgs::StepPlan>("vigir_footstep_planning_msgs::StepPlan");
  qRegisterMetaType<vigir_footstep_planning_msgs::PlanningFeedback>("vigir_footstep_planning_msgs::PlanningFeedback");
  qRegisterMetaType<vigir_footstep_planning_msgs::Feet>("vigir_footstep_planning_msgs::Feet");

}

RequestHandlerBase::~RequestHandlerBase()
{
  delete request_;
}

// ----- Send simple Planning / Pattern Request -----
void RequestHandlerBase::sendRequest()
{
  if(ac.isServerConnected())
  {
    requestStartPose(); //set Current Start
    vigir_footstep_planning_msgs::StepPlanRequestGoal goal;
    goal.plan_request = *request_;
    ac.sendGoal(goal,
                boost::bind(&RequestHandlerBase::resultCallback, this, _1, _2),
                StepPlanRequestActionClient::SimpleActiveCallback(),
                boost::bind(&RequestHandlerBase::feedbackCallback, this, _1));
  }
}

void RequestHandlerBase::resultCallback(const actionlib::SimpleClientGoalState& state, const RequestResult& result)
{
  if(result->status.error == vigir_footstep_planning_msgs::ErrorStatus::NO_ERROR)
  {
    ROS_INFO("No errors, set current step plan");
    setCurrentStepPlan(result->step_plan);
    Q_EMIT(createdStepPlan(result->step_plan));
  }
  else{
    ROS_INFO("Errors during computation of step plan. %s", result->status.error_msg.c_str());
  }
}

void RequestHandlerBase::feedbackCallback(const vigir_footstep_planning_msgs::StepPlanRequestFeedbackConstPtr& feedback)
{
  if(feedback_requested_)
  {
    Q_EMIT(receivedPlanningFeedback(feedback->feedback));
  }
}
// ----------

void RequestHandlerBase::cancelGoals()
{
 ROS_INFO("cancel");
  ac.cancelAllGoals();
//  generate_feet_ac.cancelAllGoals();
}



// ---- general set functions for request message: ---
void RequestHandlerBase::setHeaderStamp()
{
  request_->header.stamp =ros::Time::now();
}

void RequestHandlerBase::setFrameID(QString frame_id)
{
  request_->header.frame_id = frame_id.toStdString();
}

void RequestHandlerBase::setPlanningMode(int planning_mode)
{
  request_->planning_mode = planning_mode;
}

void RequestHandlerBase::setStartFoot(int start_foot_selection)
{
  request_->start_foot_selection = start_foot_selection;
}

// ---- set request_handler members: ---
void RequestHandlerBase::setFeedbackRequested(bool requested)
{
  feedback_requested_=requested;
}

void RequestHandlerBase::setCurrentStepPlan(const StepPlanMsg& step_plan)
{
  current_step_plan = step_plan;
  setCurrentGoal(current_step_plan.steps.size() - 1);
  last_step_index = current_step_plan.steps.size() - 1;
  replan_goal_index = 0;
}

// --- Action Server Connection:
void RequestHandlerBase::connectToActionServer()
{
  bool connected = ac.waitForServer(ros::Duration(1,0));
  if(!connected)
  {
    ROS_INFO("Could not connect to Action Server. (/johnny5/footstep_planning/step_plan_request)");
  }
  else
  {
    ROS_INFO("Connected to Action Server. (/johnny5/footstep_planning/step_plan_request)");
    connected = generate_feet_ac.waitForServer(ros::Duration(1,0));
    if(!connected) ROS_WARN("Could not connect to Action Server (/vigir_footstep_planning/generate_feet_pose");
    else ROS_INFO("Connected to Action Server. (/johnny5/footstep_planning/generate_feet_pose");
  }
}

bool RequestHandlerBase::checkConnection()
{
  return ac.isServerConnected();
}
// -------

// ----- Get Current Start Pose Action
void RequestHandlerBase::requestStartPose()
{
  if(generate_feet_ac.isServerConnected())
  {
    vigir_footstep_planning_msgs::GenerateFeetPoseGoal goal;
    goal.request.header.stamp = ros::Time::now();
    goal.request.header.frame_id = request_->header.frame_id;

    goal.request.flags = vigir_footstep_planning_msgs::FeetPoseRequest::FLAG_CURRENT;
    generate_feet_ac.sendGoal(goal,
                              boost::bind(&RequestHandlerBase::startPoseCallback, this, _1, _2),
                              GenerateFeetPoseActionClient::SimpleActiveCallback(),
                              GenerateFeetPoseActionClient::SimpleFeedbackCallback());
  }
}

void RequestHandlerBase::startPoseCallback(const actionlib::SimpleClientGoalState& state, const GenerateFeetPoseResult& result)
{
  if(result->status.error == ErrorStatusMsg::NO_ERROR)
  {
    if(result->status.warning != ErrorStatusMsg::NO_WARNING)
    {
      ROS_WARN("%s", result->status.warning_msg.c_str());
    }
    Q_EMIT(startFeetAnswer(result->feet));
    request_->start = result->feet;
  }
  else{
    ROS_ERROR("%s", result->status.error_msg.c_str());
    ROS_WARN("Defaulting start pose to origin");

    vigir_footstep_planning_msgs::GenerateFeetPoseGoal goal;
    goal.request.header.stamp = ros::Time::now();
    goal.request.header.frame_id = request_->header.frame_id;

    goal.request.pose.position.x = 0; goal.request.pose.position.y = 0; goal.request.pose.position.z = 0;
    goal.request.pose.orientation.x = 0; goal.request.pose.orientation.y = 0; goal.request.pose.orientation.z = 0;
    goal.request.pose.orientation.w = 1;

    generate_feet_ac.sendGoal(goal,
                              boost::bind(&RequestHandlerBase::startPoseCallback, this, _1, _2),
                              GenerateFeetPoseActionClient::SimpleActiveCallback(),
                              GenerateFeetPoseActionClient::SimpleFeedbackCallback());
  }
}

// ------------


// ----- Set Current Goal (for adding a step plan) either the end of pattern plan or a chosen foot at given index
void RequestHandlerBase::setCurrentGoal(int last_index)
{
  if(current_step_plan.steps.size() <= last_index)
  {
    ROS_ERROR("Last index %i invalid, maximum %i is allowed", last_index, (int)current_step_plan.steps.size());
    return;
  }
  last_step_index = last_index;
  FootMsg last = current_step_plan.steps[last_index].foot;
  geometry_msgs::Pose pose = last.pose;

  //compute shift for orientation
  float x_shift= 0.04, y_shift = 0.11, z_shift = 0;
  if(last.foot_index == FootMsg::LEFT)
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
                              boost::bind(&RequestHandlerBase::setCurrentGoalCallback, this, _1, _2),
                              GenerateFeetPoseActionClient::SimpleActiveCallback(),
                              GenerateFeetPoseActionClient::SimpleFeedbackCallback());
  }
}

void RequestHandlerBase::setCurrentGoalCallback(const actionlib::SimpleClientGoalState& state, const GenerateFeetPoseResult& result)
{
  if(result->status.error == ErrorStatusMsg::NO_ERROR)
  {
    if(result->status.warning != ErrorStatusMsg::NO_WARNING)
    {
      ROS_WARN("%s", result->status.warning_msg.c_str());
    }
    current_step_plan.goal = result->feet;
  }
  else{
    ROS_ERROR("%s", result->status.error_msg.c_str());
  }
}
// --------------

// add a step plan
void RequestHandlerBase::addStepPlan()
{
  ROS_INFO("last step index: %i" , last_step_index);

  //copy current request (to not change start of original request)
  RequestMsg nextRequest = *request_;
  if(last_step_index > 0)
  {
    nextRequest.start = current_step_plan.goal;

    switch (current_step_plan.steps[last_step_index].foot.foot_index)
    {
    case FootMsg::LEFT:
      nextRequest.start_foot_selection = RequestMsg::RIGHT;
      break;
    case FootMsg::RIGHT:
      nextRequest.start_foot_selection = RequestMsg::LEFT;
      break;
    }
  }
  if(ac.isServerConnected())
  {
    vigir_footstep_planning_msgs::StepPlanRequestGoal goal;
    goal.plan_request = nextRequest;
    ac.sendGoal(goal,
                boost::bind(&RequestHandlerBase::addStepPlanCallback, this, _1, _2),
                StepPlanRequestActionClient::SimpleActiveCallback(),
                boost::bind(&RequestHandlerBase::feedbackCallback, this, _1));
  }
}

void RequestHandlerBase::addStepPlanCallback(const actionlib::SimpleClientGoalState& state, const RequestResult& result)
{
  if(result->status.error == ErrorStatusMsg::NO_ERROR)
  {
    appendStepPlan(result->step_plan);
  }
  else
  {
    ROS_INFO("Errors during computation of step plan. %s", result->status.error_msg.c_str());
  }
}

void RequestHandlerBase::appendStepPlan(StepPlanMsg add)
{

  vigir_footstep_planning::StepPlan current(current_step_plan);

  for(int i = current_step_plan.steps.size()-1; i > last_step_index; --i)
  {
    ROS_INFO("remove step %i", i);
    current.removeStep(i);
  }

  ErrorStatusMsg error_status = current.appendStepPlan(add);

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


//  - helper functions
void RequestHandlerBase::computeShift(float &x_shift, float &y_shift, float &z_shift, const geometry_msgs::Quaternion& orientation)
{
  float w = orientation.w, x =orientation.x, y=orientation.y, z=orientation.z;
  float x_shift_rot = x_shift*(w*w+x*x-y*y-z*z) + y_shift*(2*x*y - 2*w*z) + z_shift*(2*x*z +2*w*y);
  float y_shift_rot = x_shift*(2*x*y + 2*w*z) + y_shift*(w*w-x*x+y*y-z*z) + z_shift*(2*y*z - 2*w*x);
  float z_shift_rot = x_shift*(2*x*z - 2*w*y) + y_shift*(2*y*z + 2*w*x) + z_shift*(w*w-x*x-y*y+z*z);
  x_shift = x_shift_rot;
  y_shift = y_shift_rot;
  z_shift = z_shift_rot;
}


} // end namespace vigir_footstep_planning_rviz_plugin



