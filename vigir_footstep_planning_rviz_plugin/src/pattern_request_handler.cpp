#include <vigir_footstep_planning_rviz_plugin/pattern_request_handler.h>
#include <vigir_footstep_planning_msgs/step_plan.h>

#include <QMetaType>

namespace vigir_footstep_planning_rviz_plugin
{

PatternRequestHandler::PatternRequestHandler(QObject* parent)
  : RequestHandlerBase( parent )
{
}

PatternRequestHandler::~PatternRequestHandler()
{
  
}

void PatternRequestHandler::sendPatternRequest(int patternMode, bool append)
{
  setHeaderStamp();
  setPlanningMode(vigir_footstep_planning_msgs::StepPlanRequest::PLANNING_MODE_PATTERN);
  setPatternMode(patternMode);
  if(append && current_step_plan.steps.size() > 0)
    addStepPlan(); //current_step_plan.goal is used as new start
  else
    sendRequest();
}

// Set Pattern Parameters: ---------------------
void PatternRequestHandler::setPatternMode(int mode)
{
  request_->pattern_parameters.mode = mode;
}

void PatternRequestHandler::setNoSteps(int no_steps)
{
  request_->pattern_parameters.steps = no_steps;
}

void PatternRequestHandler::setStepDistance(double step_dist)
{
  request_->pattern_parameters.step_distance_forward = (float) step_dist;
}

void PatternRequestHandler::setSideStep(double step_side)
{
  request_->pattern_parameters.step_distance_sideward = (float) step_side;
}

void PatternRequestHandler::setTurnAngle(int turn_angle)
{
  request_->pattern_parameters.turn_angle= (float) turn_angle*(2*3.14159/360);
}

void PatternRequestHandler::setdz(double dz)
{
  request_->pattern_parameters.dz = (float) dz;
}

// start_foot_selection
// uint8 AUTO  = 0
// uint8 LEFT  = 1
// uint8 RIGHT = 2
void PatternRequestHandler::setStartStepIndex(int start_step)
{
  request_->start_step_index = start_step;
}

void PatternRequestHandler::setRoll(int roll)
{
  request_->pattern_parameters.roll = (float) roll*(2*3.14159/360);
}

void PatternRequestHandler::setPitch(int pitch)
{
  request_->pattern_parameters.pitch = (float) pitch*(2*3.14159/360);
}

void PatternRequestHandler::setClosingStep(int state)
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
void PatternRequestHandler::setExtraSeperation(int state)
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

void PatternRequestHandler::setUseTerrainModel(int state)
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

void PatternRequestHandler::setOverride3D(int state)
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

// -----------------------

void PatternRequestHandler::appendStepPlan(StepPlanMsg add)
{
  vigir_footstep_planning::StepPlan current(current_step_plan);
  vigir_footstep_planning::StepPlan to_be_added(add);

  for(int i = current_step_plan.steps.size()-1; i > last_step_index; --i)
  {
    current.removeStep(i);
  }

  int i = 0;
  while(add.steps[i].step_duration == 0)
  {
    to_be_added.removeStep(0);
    ++i;
  }

  to_be_added.toMsg(add);

// ROS_ERROR("deleted %i steps", i);

  ErrorStatusMsg error_status = current.appendStepPlan(add);

  if(error_status.error == ErrorStatusMsg::NO_ERROR)
  {
    StepPlanMsg step_plan;
    current.toMsg(step_plan);
    Q_EMIT(stepPlanGenerationFinished(true));
    Q_EMIT(createdSequence(step_plan)); // sequence is not set as step plan only after it has been updated
  }
  else
  {
    Q_EMIT(stepPlanGenerationFinished(false));
    if(current_step_plan.steps.size() > 0)
      Q_EMIT(createdStepPlan(current_step_plan));
  }
}

} // end namespace vigir_footstep_planning_rviz_plugin



