#include <vigir_footstep_planning_rviz_plugin/request_handler.h>
#include <QMetaType>

namespace vigir_footstep_planning_rviz_plugin
{

RequestHandler::RequestHandler(QObject* parent)
  : QObject( parent ),
    ac("/vigir/footstep_planning/step_plan_request", true)
{
/*  request_ = new vigir_footstep_planning_msgs::StepPlanRequest();
  ac.waitForServer();
  feedback_requested_=false;*/
  qRegisterMetaType<vigir_footstep_planning_msgs::StepPlan>("vigir_footstep_planning_msgs::StepPlan");
  qRegisterMetaType<vigir_footstep_planning_msgs::PlanningFeedback>("vigir_footstep_planning_msgs::PlanningFeedback");
}

RequestHandler::~RequestHandler()
{
  delete request_;
}

void RequestHandler::setHeader()
{
  request_->header.stamp =ros::Time::now();
}

void RequestHandler::setPlanningMode(int planning_mode)
{
  if (planning_mode < 3)
  {
    request_->planning_mode = planning_mode;
  }
  else
    ROS_INFO("Could not set planning mode.");
}

void RequestHandler::setPatternMode(int mode)
{
  request_->planning_mode = vigir_footstep_planning_msgs::StepPlanRequest::PLANNING_MODE_PATTERN;
  request_->pattern_parameters.mode = mode;
}


void RequestHandler::sendRequest()
{
  vigir_footstep_planning_msgs::StepPlanRequestGoal goal;
  goal.plan_request = *request_;
  // Need boost::bind to pass in the 'this' pointer
  ac.sendGoal(goal,
              boost::bind(&RequestHandler::resultCallback, this, _1, _2),
              StepPlanRequestActionClient::SimpleActiveCallback(),
              boost::bind(&RequestHandler::feedbackCallback, this, _1));

}

void RequestHandler::resultCallback(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::StepPlanRequestResultConstPtr& result)
{
  result_=result;
  if(result->status.error == vigir_footstep_planning_msgs::ErrorStatus::NO_ERROR)
  {
    vigir_footstep_planning_msgs::StepPlan stepPlan = result->step_plan;
    Q_EMIT(createdStepPlan(stepPlan));
  }
  else
  {
      ROS_ERROR("%s", error_status.error_msg.c_str());
  }
}

void RequestHandler::feedbackCallback(const vigir_footstep_planning_msgs::StepPlanRequestFeedbackConstPtr& feedback)
{
  if(feedback_requested_)
  {
    Q_EMIT(receivedPlanningFeedback(feedback->feedback));
  }
}


void RequestHandler::sendPatternRequest(int patternMode)
{
  setHeader();
  setPlanningMode(vigir_footstep_planning_msgs::StepPlanRequest::PLANNING_MODE_PATTERN);
  setPatternMode(patternMode);
  sendRequest();
}

void RequestHandler::sendSimplePatternRequest(int patternMode, int no_steps, double step_dist, int turn_angle, double dz)
{
  if(turn_angle < 361 && turn_angle >= 0)
  {
    setTurnAngle(turn_angle);
  }
  else ROS_INFO("Invalid Turn Angle!");
  setHeader();
  setPlanningMode(vigir_footstep_planning_msgs::StepPlanRequest::PLANNING_MODE_PATTERN);
  setNoSteps(no_steps);
  setStepDistance(step_dist);
  setdz(dz);
  setPatternMode(patternMode);
  sendRequest();
}

void RequestHandler::sendPlanningRequest()
{
  setHeader();
  sendRequest();
}

void RequestHandler::setFrameID(QString frame_id)
{
  request_->header.frame_id = frame_id.toStdString();
}

// Set Pattern Parameters: ---------------------
void RequestHandler::setNoSteps(int no_steps)
{
  request_->pattern_parameters.steps = no_steps;
}

void RequestHandler::setStepDistance(double step_dist)
{
  request_->pattern_parameters.step_distance_forward = (float) step_dist;
}

void RequestHandler::setSideStep(double step_side)
{
  request_->pattern_parameters.step_distance_sideward = (float) step_side;
}

void RequestHandler::setTurnAngle(int turn_angle)
{
  request_->pattern_parameters.turn_angle= (float) turn_angle*(2*3.14159/360);
}

void RequestHandler::setdz(double dz)
{
  request_->pattern_parameters.dz = (float) dz;
}

void RequestHandler::setStartStepIndex(int start_index)
{
  request_->start_step_index = start_index;
}

void RequestHandler::setRoll(int roll)
{
  request_->pattern_parameters.roll = (float) roll*(2*3.14159/360);
}

void RequestHandler::setPitch(int pitch)
{
  request_->pattern_parameters.pitch = (float) pitch*(2*3.14159/360);
}

void RequestHandler::setStartFoot(int start_foot)
{
  request_->start_foot_selection = start_foot;
}

void RequestHandler::setClosingStep(int state)
{
  if (state == Qt::Unchecked)
  {
    request_->pattern_parameters.close_step = false;
  }
  if (state == Qt::Checked)
  {
    request_->pattern_parameters.close_step = true;
  }
}
void RequestHandler::setExtraSeperation(int state)
{
  if (state == Qt::Unchecked)
  {
    request_->pattern_parameters.extra_seperation = false;
  }
  if (state == Qt::Checked)
  {
    request_->pattern_parameters.extra_seperation = true;
  }
}

void RequestHandler::setUseTerrainModel(int state)
{
  if (state == Qt::Unchecked)
  {
    request_->pattern_parameters.use_terrain_model = false;
  }
  if (state == Qt::Checked)
  {
    request_->pattern_parameters.use_terrain_model = true;
  }
}

void RequestHandler::setOverride3D(int state)
{
  if (state == Qt::Unchecked)
  {
    request_->pattern_parameters.override = false;
  }
  if (state == Qt::Checked)
  {
    request_->pattern_parameters.override = true;
  }
}

//--------- End set Pattern Parameters

vigir_footstep_planning_msgs::StepPlan RequestHandler::getResultStepPlan()
{
 // return result_->step_plan;
}

void RequestHandler::setGoal(Ogre::Vector3 position, Ogre::Quaternion orientation)
{
  request_->goal.left.pose.orientation.x = 0; //orientation.x;
  request_->goal.left.pose.orientation.y = 0; //orientation.y;
  request_->goal.left.pose.orientation.z = 0; //orientation.z;
  request_->goal.left.pose.orientation.w = 1; //orientation.w;

  request_->goal.right.pose.orientation.x = 0;//orientation.x;
  request_->goal.right.pose.orientation.y = 0; //orientation.y;
  request_->goal.right.pose.orientation.z = 0; //orientation.z;
  request_->goal.right.pose.orientation.w = 1; //orientation.w;

  request_->goal.left.pose.position.x = position.x;
  request_->goal.left.pose.position.y = position.y+0.11;
  request_->goal.left.pose.position.z = position.z+0.085;

  request_->goal.right.pose.position.x = position.x;
  request_->goal.right.pose.position.y = position.y-0.11;
  request_->goal.right.pose.position.z = position.z+0.085;

  request_->goal.header.frame_id = request_->header.frame_id;
  request_->goal.left.header.frame_id = request_->header.frame_id;
  request_->goal.right.header.frame_id = request_->header.frame_id;
  request_->goal.header.frame_id = request_->header.frame_id;
}

void RequestHandler::setFeedbackRequested(bool requested)
{
  feedback_requested_=requested;
}



} // end namespace vigir_footstep_planning_rviz_plugin



