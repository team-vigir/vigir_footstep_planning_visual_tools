
#include <vigir_footstep_planning_rviz_plugin/footstep_planning_panel.h>
#include <rviz/config.h>
#include <QIcon>
#include "../../../../../build/vigir_footstep_planning_rviz_plugin/ui_paneldesign.h"


namespace vigir_footstep_planning_rviz_plugin
{
FootstepPlanningPanel::FootstepPlanningPanel( QWidget* parent )
  : QWidget( parent )
  , ui_(new Ui::PanelDesign)
{
  ui_->setupUi(this);
  // set icons:
// path relative to runtime directory
  QIcon icon_left("./vigir/vigir_footstep_planning/vigir_footstep_planning_visual_tools/vigir_footstep_planning_rviz_plugin/media/footLeft.png");
  if(!icon_left.isNull())
    ui_->leftFootToolButton->setIcon(icon_left);
  QIcon icon_right("./vigir/vigir_footstep_planning/vigir_footstep_planning_visual_tools/vigir_footstep_planning_rviz_plugin/media/footRight.png");
  if(!icon_right.isNull())
    ui_->rightFootToolButton->setIcon(icon_right);
  QIcon icon_both("./vigir/vigir_footstep_planning/vigir_footstep_planning_visual_tools/vigir_footstep_planning_rviz_plugin/media/bothFeet.png");
  if(!icon_both.isNull())
    ui_->bothFeetToolButton->setIcon(icon_both);

  makePatternConnections();
  makePlanningConnections();
  // set Step plan if computed in other widget
  connect(ui_->patternWidget->request_handler_, SIGNAL(createdStepPlan( vigir_footstep_planning_msgs::StepPlan)), ui_->planningWidget, SLOT(setCurrentStepPlan(vigir_footstep_planning_msgs::StepPlan)));
  connect(ui_->planningWidget->request_handler_, SIGNAL(createdStepPlan( vigir_footstep_planning_msgs::StepPlan)), ui_->patternWidget, SLOT(setCurrentStepPlan(vigir_footstep_planning_msgs::StepPlan)));
  connect(ui_->patternWidget->request_handler_, SIGNAL(createdStepPlan(vigir_footstep_planning_msgs::StepPlan)), ui_->propertyWidget, SLOT(displayStepPlanProperties(vigir_footstep_planning_msgs::StepPlan)));
  connect(ui_->planningWidget->request_handler_, SIGNAL(createdStepPlan(vigir_footstep_planning_msgs::StepPlan)), ui_->propertyWidget, SLOT(displayStepPlanProperties(vigir_footstep_planning_msgs::StepPlan)));

  connect(ui_->bothFeetToolButton, SIGNAL(toggled(bool)), this, SLOT(emitPlaceBothActivated(bool)));
  connect(ui_->leftFootToolButton, SIGNAL(toggled(bool)), this, SLOT(emitPlaceLeftActivated(bool)));
  connect(ui_->rightFootToolButton, SIGNAL(toggled(bool)), this, SLOT(emitPlaceRightActivated(bool)));

  connect(ui_->interactionModeComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(emitInteractionModeChanged(int)));

  connect(ui_->undoPushButton, SIGNAL(clicked()), this, SIGNAL(undo()));
  connect(ui_->acceptPushButton, SIGNAL(clicked()), this, SIGNAL(acceptModifiedStepPlan()));

  connect(ui_->executePushButton, SIGNAL(clicked()), this, SIGNAL(executeRequested()));
}


void FootstepPlanningPanel::makePatternConnections()
{
  connect(this, SIGNAL(resetValues()), ui_->patternWidget, SLOT(resetValues()));

  //connect(ui_->abortPushButton, SIGNAL(clicked()), ui_->patternWidget, SLOT(abort()));
  // update ui on step plan created
  connect(ui_->patternWidget->request_handler_, SIGNAL(createdStepPlan( vigir_footstep_planning_msgs::StepPlan))
          , this, SLOT(updateFromTo(vigir_footstep_planning_msgs::StepPlan)));

  // PatternRequestHandler signals
  connect(ui_->patternWidget->request_handler_, SIGNAL(createdStepPlan(vigir_footstep_planning_msgs::StepPlan))
              , this, SLOT(emitCreatedStepPlan(vigir_footstep_planning_msgs::StepPlan)));
  connect(ui_->patternWidget->request_handler_, SIGNAL(startFeetAnswer(vigir_footstep_planning_msgs::Feet)), this
              , SLOT(emitStartFeetAnswer(vigir_footstep_planning_msgs::Feet)));

  connect(ui_->patternWidget->ui->sequenceCheckBox, SIGNAL(toggled(bool)), ui_->planningWidget->ui->sequenceCheckBox, SLOT(setChecked(bool)));
}

void FootstepPlanningPanel::makePlanningConnections()
{
  connect(this, SIGNAL(resetValues()), ui_->planningWidget, SLOT(resetValues()));
  connect(ui_->abortPushButton, SIGNAL(clicked()), ui_->planningWidget, SLOT(abort()));

  // update ui on step plan created
  connect(ui_->planningWidget->request_handler_, SIGNAL(createdStepPlan( vigir_footstep_planning_msgs::StepPlan)), this, SLOT(updateFromTo(vigir_footstep_planning_msgs::StepPlan)));

  // PlanningRequestHandler signals
  connect(ui_->planningWidget->request_handler_, SIGNAL(createdStepPlan(vigir_footstep_planning_msgs::StepPlan))
              , this, SLOT(emitCreatedStepPlan(vigir_footstep_planning_msgs::StepPlan)));
  connect(ui_->planningWidget->request_handler_, SIGNAL(startFeetAnswer(vigir_footstep_planning_msgs::Feet)), this
              , SLOT(emitStartFeetAnswer(vigir_footstep_planning_msgs::Feet)));
  connect(ui_->planningWidget->request_handler_, SIGNAL(goalFeetAnswer(vigir_footstep_planning_msgs::Feet))
              , this, SLOT(emitGoalFeetAnswer(vigir_footstep_planning_msgs::Feet)));
  connect(ui_->planningWidget->request_handler_, SIGNAL(receivedPlanningFeedback(vigir_footstep_planning_msgs::PlanningFeedback))
              , this, SLOT(emitSendPlanningFeedback(vigir_footstep_planning_msgs::PlanningFeedback)));

  connect(ui_->planningWidget, SIGNAL(feetToolActivated(bool)), this, SLOT(emitFeetToolActivated(bool)));

  connect(ui_->planningWidget->ui->sequenceCheckBox, SIGNAL(toggled(bool)), ui_->patternWidget->ui->sequenceCheckBox, SLOT(setChecked(bool)));

}

FootstepPlanningPanel::~FootstepPlanningPanel()
{
  delete ui_;
}

void FootstepPlanningPanel::save( rviz::Config config ) const
{
  ui_->patternWidget->save(config);
  ui_->planningWidget->save(config);
}

void FootstepPlanningPanel::load( const rviz::Config& config )
{
  ui_->patternWidget->load(config);
  ui_->planningWidget->load(config);
}
// Reset TODO reset Scene, reset Values
void FootstepPlanningPanel::on_reset_pushButton_clicked()
{
  Q_EMIT(clearScene());
  Q_EMIT(resetValues());
}

void FootstepPlanningPanel::on_clearIMPushButton_clicked()
{
    Q_EMIT(clearIM());
}

// ----------- From to spin boxes
void FootstepPlanningPanel::updateFromTo(vigir_footstep_planning_msgs::StepPlan step_plan )
{
  ui_->displayStepsToSpinBox->setMaximum(step_plan.steps.size()-1);
  ui_->displayStepsToSpinBox->setValue(step_plan.steps.size()-1);
  ui_->displayStepsFromSpinBox->setMaximum(step_plan.steps.size()-1);
}

void FootstepPlanningPanel::on_displayStepsFromSpinBox_valueChanged(int arg1)
{
  Q_EMIT(displayRangeChanged(arg1, ui_->displayStepsToSpinBox->value()));
}

void FootstepPlanningPanel::on_displayStepsToSpinBox_valueChanged(int arg1)
{
  Q_EMIT(displayRangeChanged(ui_->displayStepsFromSpinBox->value(), arg1));
}

// --------------------------------------------------------------------------------------------

// pass to planning and pattern widget / request handlers -------------------------------------------------

bool FootstepPlanningPanel::checkConnection()
{
  return (ui_->planningWidget->request_handler_->checkConnection() || ui_->patternWidget->request_handler_->checkConnection());
}

void FootstepPlanningPanel::setFrameID(const QString frame_id)
{
  ui_->planningWidget->setFrameID(frame_id);
  ui_->patternWidget->setFrameID(frame_id);
}

void FootstepPlanningPanel::setLastStep(int index)
{
  ui_->planningWidget->setLastStep(index);
  ui_->patternWidget->setLastStep(index);
}

void FootstepPlanningPanel::replanToIndex(int index)
{
  ui_->planningWidget->replanToIndex(index);
}

void FootstepPlanningPanel::handleGoalPose(Ogre::Vector3 position, Ogre::Quaternion orientation)
{
  ui_->planningWidget->handleGoalPose(position, orientation);
}

void FootstepPlanningPanel::releasePlaceFeet()
{
  ui_->bothFeetToolButton->setChecked(false);
  ui_->rightFootToolButton->setChecked(false);
  ui_->leftFootToolButton->setChecked(false);
}

void FootstepPlanningPanel::setFeedbackRequested(bool requested)
{
  ui_->planningWidget->setFeedbackRequested(requested);
}

void FootstepPlanningPanel::startPoseRequested()
{
  ui_->planningWidget->startPoseRequested();
}

void FootstepPlanningPanel::setStepPlan(vigir_footstep_planning_msgs::StepPlan step_plan)
{
  ui_->planningWidget->setCurrentStepPlan(step_plan);
  ui_->patternWidget->setCurrentStepPlan(step_plan);
}



// emit signals
void FootstepPlanningPanel::emitCreatedStepPlan(vigir_footstep_planning_msgs::StepPlan step_plan) {Q_EMIT(createdStepPlan(step_plan)); }
void FootstepPlanningPanel::emitStartFeetAnswer(vigir_footstep_planning_msgs::Feet start){Q_EMIT(startFeetAnswer(start));}
void FootstepPlanningPanel::emitGoalFeetAnswer(vigir_footstep_planning_msgs::Feet goal){Q_EMIT(goalFeetAnswer(goal));}
void FootstepPlanningPanel::emitSendPlanningFeedback(vigir_footstep_planning_msgs::PlanningFeedback feedback){Q_EMIT(sendPlanningFeedback(feedback));}
void FootstepPlanningPanel::emitFeetToolActivated(bool active){Q_EMIT(feetToolActivated(active));}
void FootstepPlanningPanel::emitInteractionModeChanged(int index){Q_EMIT(interactionModeChanged(index+1));}
void FootstepPlanningPanel::emitPlaceLeftActivated(bool active)
{
  if(active)
  {
    ui_->bothFeetToolButton->setChecked(false);
    ui_->rightFootToolButton->setChecked(false);
  }
  Q_EMIT(feetToolActivated(LEFT, active));
}
void FootstepPlanningPanel::emitPlaceRightActivated(bool active)
{
  if(active)
  {
    ui_->bothFeetToolButton->setChecked(false);
    ui_->leftFootToolButton->setChecked(false);
  }
  Q_EMIT(feetToolActivated(RIGHT, active));
}
void FootstepPlanningPanel::emitPlaceBothActivated(bool active)
{
  if(active)
  {
    ui_->leftFootToolButton->setChecked(false);
    ui_->rightFootToolButton->setChecked(false);
  }
  Q_EMIT(feetToolActivated(BOTH, active));
}

} // end namespace vigir_footstep_planning_rviz_plugin
