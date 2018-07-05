
#include <vigir_footstep_planning_rviz_plugin/footstep_planning_panel.h>
#include <rviz/config.h>
#include <QIcon>


namespace vigir_footstep_planning_rviz_plugin
{
FootstepPlanningPanel::FootstepPlanningPanel( QWidget* parent )
  : QWidget( parent )
  , ui_(new Ui::PanelDesign)
{
  ui_->setupUi(this);
  // Status Prompt output connections:
  /*
  connect(ui_->parameterSetComboBox, SIGNAL(actionClientConnected(QString, bool)), ui_->messageDisplay, SLOT( displayConnection(QString, bool)));
  connect(ui_->patternWidget->request_handler_, SIGNAL(actionClientConnected(QString, bool)), ui_->messageDisplay, SLOT( displayConnection(QString, bool)));
  connect(ui_->planningWidget->request_handler_, SIGNAL(actionClientConnected(QString, bool)), ui_->messageDisplay, SLOT( displayConnection(QString, bool)));

  connect(ui_->patternWidget->request_handler_, SIGNAL(displayInfo(QString)), ui_->messageDisplay, SLOT(displayMessage(QString)));
  connect(ui_->patternWidget->request_handler_, SIGNAL(displayError(QString)), ui_->messageDisplay, SLOT(displayError(QString)));
  connect(ui_->patternWidget->request_handler_, SIGNAL(displaySuccess(QString)), ui_->messageDisplay, SLOT(displaySuccess(QString)));

  connect(ui_->planningWidget->request_handler_, SIGNAL(displayInfo(QString)), ui_->messageDisplay, SLOT(displayMessage(QString)));
  connect(ui_->planningWidget->request_handler_, SIGNAL(displayError(QString)), ui_->messageDisplay, SLOT(displayError(QString)));
  connect(ui_->planningWidget->request_handler_, SIGNAL(displaySuccess(QString)), ui_->messageDisplay, SLOT(displaySuccess(QString)));
  */
// ---
  ui_->parameterSetComboBox->initialize();
  ui_->patternWidget->initialize();
  ui_->planningWidget->initialize();

  // set icons:
  ros::NodeHandle nh;
  std::string icons_path;
  if(nh.getParam("icons_path", icons_path))
  {
    /*
    QIcon icon_left(QString::fromStdString(icons_path + "footLeft.png"));
    if(!icon_left.isNull())
      ui_->leftFootToolButton->setIcon(icon_left);
    QIcon icon_right(QString::fromStdString(icons_path + "footRight.png"));
    if(!icon_right.isNull())
      ui_->rightFootToolButton->setIcon(icon_right);*/
    QIcon icon_both(QString::fromStdString(icons_path + "bothFeet.png"));
    if(!icon_both.isNull())
    {
      ui_->bothFeetToolButton->setIcon(icon_both);
      ui_->planningWidget->ui->planningSetGoalToolButton->setIcon(icon_both);
    }
    QIcon icon_im(QString::fromStdString("/opt/ros/kinetic/share/rviz/icons/classes/InteractiveMarkers.png"));
    if(!icon_im.isNull())
      ui_->clearIMPushButton->setIcon(icon_im);
    QIcon icon_step_plan(QString::fromStdString(icons_path + "stepPlan.png"));
    if(!icon_im.isNull())
      ui_->clearStepPlanPushButton->setIcon(icon_step_plan);
  }


  ui_->displayOptionsGroupBox->setChecked(false);

  //ui_->moreOptionsGroupBox->setChecked(false);
  ui_->patternWidget->ui->sequenceCheckBox->setVisible(false);
  ui_->planningWidget->ui->sequenceCheckBox->setVisible(false);
  ui_->sequenceDescription->setVisible(false);



  makePatternConnections();
  makePlanningConnections();
  // set Step plan if computed in other widget
  connect(ui_->patternWidget->request_handler_, SIGNAL(createdStepPlan( vigir_footstep_planning_msgs::StepPlan)), ui_->planningWidget, SLOT(setCurrentStepPlan(vigir_footstep_planning_msgs::StepPlan)));
  connect(ui_->planningWidget->request_handler_, SIGNAL(createdStepPlan( vigir_footstep_planning_msgs::StepPlan)), ui_->patternWidget, SLOT(setCurrentStepPlan(vigir_footstep_planning_msgs::StepPlan)));
 // connect(ui_->patternWidget->request_handler_, SIGNAL(createdStepPlan(vigir_footstep_planning_msgs::StepPlan)), ui_->propertyWidget, SLOT(displayStepPlanProperties(vigir_footstep_planning_msgs::StepPlan)));
 // connect(ui_->planningWidget->request_handler_, SIGNAL(createdStepPlan(vigir_footstep_planning_msgs::StepPlan)), ui_->propertyWidget, SLOT(displayStepPlanProperties(vigir_footstep_planning_msgs::StepPlan)));

  // progress bar
  connect(ui_->planningWidget->request_handler_, SIGNAL(stepPlanGenerationStarted()), this, SLOT(initializeGenerationProgressbar()));
  connect(ui_->planningWidget->request_handler_, SIGNAL(stepPlanGenerationFinished(bool)), this, SLOT(updateProgressBar(bool)));
  connect(ui_->patternWidget->request_handler_, SIGNAL(stepPlanGenerationStarted()), this, SLOT(initializeGenerationProgressbar()));
  connect(ui_->patternWidget->request_handler_, SIGNAL(stepPlanGenerationFinished(bool)), this, SLOT(updateProgressBar(bool)));


  connect(ui_->bothFeetToolButton, SIGNAL(toggled(bool)), this, SLOT(emitPlaceBothActivated(bool)));
//  connect(ui_->leftFootToolButton, SIGNAL(toggled(bool)), this, SLOT(emitPlaceLeftActivated(bool)));
//  connect(ui_->rightFootToolButton, SIGNAL(toggled(bool)), this, SLOT(emitPlaceRightActivated(bool)));

  connect(ui_->interactionModeComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(emitInteractionModeChanged(int)));

  connect(ui_->undoToolButton, SIGNAL(clicked()), this, SIGNAL(undo()));
 // connect(ui_->removeSequenceToolButton, SIGNAL(clicked()), this, SIGNAL(undo()));

  connect(ui_->executePushButton, SIGNAL(clicked()), this, SIGNAL(executeRequested()));

  connect(ui_->refreshParameterSetsToolButton, SIGNAL(clicked()), ui_->parameterSetComboBox, SLOT(updateParameterSetNames()));

  // reset:
  // clear all button invokes clearAll() and clearStepPlan() (IM are cleared automatically when step plan is cleared)
  connect(ui_->clearAllPushButton, SIGNAL(clicked()), this, SIGNAL(clearAll()));
  connect(ui_->clearAllPushButton, SIGNAL(clicked()), this, SIGNAL(clearStepPlan()));
  connect(ui_->clearStepPlanPushButton, SIGNAL(clicked()), this, SIGNAL(clearStepPlan()));
  connect(ui_->clearIMPushButton, SIGNAL(clicked()), this, SIGNAL(clearIM()));

}


void FootstepPlanningPanel::makePatternConnections()
{
 //todo connect(this, SIGNAL(resetValues()), ui_->patternWidget, SLOT(resetValues()));

  //connect(ui_->abortPushButton, SIGNAL(clicked()), ui_->patternWidget, SLOT(abort()));
  // update ui on step plan created
  connect(ui_->patternWidget->request_handler_, SIGNAL(createdStepPlan( vigir_footstep_planning_msgs::StepPlan))
          , this, SLOT(updateFromTo(vigir_footstep_planning_msgs::StepPlan)));

  // PatternRequestHandler signals
  connect(ui_->patternWidget->request_handler_, SIGNAL(createdStepPlan(vigir_footstep_planning_msgs::StepPlan))
              , this, SIGNAL(createdStepPlan(vigir_footstep_planning_msgs::StepPlan)));
  connect(ui_->patternWidget->request_handler_, SIGNAL(createdSequence(vigir_footstep_planning_msgs::StepPlan)),
          this, SIGNAL(createdSequence(vigir_footstep_planning_msgs::StepPlan)));
  connect(ui_->patternWidget->request_handler_, SIGNAL(startFeetAnswer(vigir_footstep_planning_msgs::Feet)), this
              , SLOT(emitStartFeetAnswer(vigir_footstep_planning_msgs::Feet)));

  connect(ui_->addSequenceToolButton, SIGNAL(toggled(bool)), ui_->patternWidget->ui->sequenceCheckBox, SLOT(setChecked(bool)));
  connect(ui_->patternWidget->ui->sequenceCheckBox, SIGNAL(toggled(bool)), ui_->planningWidget->ui->sequenceCheckBox, SLOT(setChecked(bool)));

  connect(ui_->parameterSetComboBox, SIGNAL(currentTextChanged(QString)), ui_->patternWidget, SLOT(setParameterSet(QString)));

  connect(this, SIGNAL(clearStepPlan()), ui_->patternWidget->request_handler_, SLOT(resetStepPlan()));
}

void FootstepPlanningPanel::makePlanningConnections()
{
 //todo connect(this, SIGNAL(resetValues()), ui_->planningWidget, SLOT(resetValues()));
  connect(ui_->abortPushButton, SIGNAL(clicked()), ui_->planningWidget, SLOT(abort()));

  // update ui on step plan created
  connect(ui_->planningWidget->request_handler_, SIGNAL(createdStepPlan( vigir_footstep_planning_msgs::StepPlan)), this, SLOT(updateFromTo(vigir_footstep_planning_msgs::StepPlan)));

  // PlanningRequestHandler signals
  connect(ui_->planningWidget->request_handler_, SIGNAL(createdStepPlan(vigir_footstep_planning_msgs::StepPlan))
              , this, SIGNAL(createdStepPlan(vigir_footstep_planning_msgs::StepPlan)));
  connect(ui_->planningWidget->request_handler_, SIGNAL(createdSequence(vigir_footstep_planning_msgs::StepPlan)), this,
          SIGNAL(createdSequence(vigir_footstep_planning_msgs::StepPlan)));
  connect(ui_->planningWidget->request_handler_, SIGNAL(startFeetAnswer(vigir_footstep_planning_msgs::Feet)), this
              , SLOT(emitStartFeetAnswer(vigir_footstep_planning_msgs::Feet)));
  connect(ui_->planningWidget->request_handler_, SIGNAL(goalFeetAnswer(vigir_footstep_planning_msgs::Feet))
              , this, SLOT(emitGoalFeetAnswer(vigir_footstep_planning_msgs::Feet)));
  connect(ui_->planningWidget->request_handler_, SIGNAL(receivedPlanningFeedback(vigir_footstep_planning_msgs::PlanningFeedback))
              , this, SLOT(emitSendPlanningFeedback(vigir_footstep_planning_msgs::PlanningFeedback)));

  connect(ui_->planningWidget, SIGNAL(feetToolActivated(bool)), this, SLOT(emitFeetToolActivated(bool)));

  connect(ui_->addSequenceToolButton, SIGNAL(toggled(bool)), ui_->planningWidget->ui->sequenceCheckBox, SLOT(setChecked(bool)));
  connect(ui_->planningWidget->ui->sequenceCheckBox, SIGNAL(toggled(bool)), ui_->patternWidget->ui->sequenceCheckBox, SLOT(setChecked(bool)));

  connect(ui_->parameterSetComboBox, SIGNAL(currentTextChanged(QString)), ui_->planningWidget, SLOT(setParameterSet(QString)));

  connect(this, SIGNAL(clearStepPlan()), ui_->planningWidget->request_handler_, SLOT(resetStepPlan()));
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

// ----------- From to spin boxes
void FootstepPlanningPanel::updateFromTo(vigir_footstep_planning_msgs::StepPlan step_plan )
{
  ui_->displayStepsToSpinBox->setMaximum(step_plan.steps.size()-1);
  //Problems because value changed is emitted and old step plan visuals will be updated:
  //ui_->displayStepsToSpinBox->setValue(step_plan.steps.size()-1);
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
//  ui_->rightFootToolButton->setChecked(false);
//  ui_->leftFootToolButton->setChecked(false);
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
    //ui_->rightFootToolButton->setChecked(false);
  }
  Q_EMIT(feetToolActivated(LEFT_FOOT, active));
}
void FootstepPlanningPanel::emitPlaceRightActivated(bool active)
{
  if(active)
  {
    ui_->bothFeetToolButton->setChecked(false);
  //  ui_->leftFootToolButton->setChecked(false);
  }
  Q_EMIT(feetToolActivated(RIGHT_FOOT, active));
}
void FootstepPlanningPanel::emitPlaceBothActivated(bool active)
{
  if(active)
  {
 //   ui_->leftFootToolButton->setChecked(false);
 //   ui_->rightFootToolButton->setChecked(false);
  }
  Q_EMIT(feetToolActivated(BOTH, active));
}

// Progress bar ------------------------------------------------------------------
void FootstepPlanningPanel::initializeExecutionProgressbar(int max)
{
  ui_->statusProgressBar->setMaximum(max);
  ui_->statusProgressBar->setValue(1);
  ui_->statusProgressBar->setTextVisible(true);
  ui_->statusProgressBar->setFormat("Step Plant sent to robot...");
}

void FootstepPlanningPanel::updateProgressBar(int val)
{
  // first progress step = step plan sent to robot
  ui_->statusProgressBar->setValue(val+1); // step index starts at 0
  ui_->statusProgressBar->setFormat(QString::number(val) + QString(" / ") + QString::number(ui_->statusProgressBar->maximum()-1));
}

void FootstepPlanningPanel::initializeGenerationProgressbar()
{
  ui_->statusProgressBar->setRange(0,0);
}

void FootstepPlanningPanel::updateProgressBar(bool success)
{
  ui_->statusProgressBar->setRange(0,1);
  ui_->statusProgressBar->setValue(1);
  ui_->statusProgressBar->setValue(ui_->statusProgressBar->maximum());
  ui_->statusProgressBar->setTextVisible(true);

  if(success)
    ui_->statusProgressBar->setFormat("No errors.");
  else
    ui_->statusProgressBar->setFormat("!Error during computation of step plan!");

}

} // end namespace vigir_footstep_planning_rviz_plugin
