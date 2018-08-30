#include <vigir_footstep_planning_rviz_plugin/footstep_planning_panel.h>
#include <rviz/config.h>
#include <QIcon>


namespace vigir_footstep_planning_rviz_plugin
{
FootstepPlanningPanel::FootstepPlanningPanel( QWidget* parent )
  : QWidget(parent)
{
  setupUi(this);

  parameterSetComboBox->initialize();
  patternWidget->initialize();
  planningWidget->initialize();

  setIcons();

  displayOptionsGroupBox->setChecked(false);

  //moreOptionsGroupBox->setChecked(false);
  patternWidget->ui->sequenceCheckBox->setVisible(false);
  planningWidget->ui->sequenceCheckBox->setVisible(false);
  sequenceDescription->setVisible(false);

  makeConnections();
}

void FootstepPlanningPanel::setIcons()
{
  ros::NodeHandle nh;
  std::string icons_path;
  if(nh.getParam("icons_path", icons_path))
  {
    QIcon icon_start(QString::fromStdString(icons_path + "feetGreen.png"));
    QIcon icon_goal(QString::fromStdString(icons_path + "feetRed.png"));

    if(!icon_goal.isNull() && !icon_start.isNull())
    {
      startFeetToolButton->setIcon(icon_start);
      planningWidget->ui->planningSetGoalToolButton->setIcon(icon_goal);
    }
    QIcon icon_im(QString::fromStdString("/opt/ros/kinetic/share/rviz/icons/classes/InteractiveMarkers.png"));
    if(!icon_im.isNull())
      clearIMPushButton->setIcon(icon_im);
    QIcon icon_step_plan(QString::fromStdString(icons_path + "stepPlan.png"));
    if(!icon_im.isNull())
      clearStepPlanPushButton->setIcon(icon_step_plan);
  }
}

void FootstepPlanningPanel::makeConnections()
{
  // Created Step Plan ------------------------------------------
  // update ui on step plan created
  connect(patternWidget->request_handler_, &PatternRequestHandler::createdStepPlan, this, &FootstepPlanningPanel::updateFromTo );
  connect(planningWidget->request_handler_, &PlanningRequestHandler::createdStepPlan, this, &FootstepPlanningPanel::updateFromTo);
  // pass created step plan to display
  connect(patternWidget->request_handler_, &PatternRequestHandler::createdStepPlan, this, &FootstepPlanningPanel::createdStepPlan);
  connect(patternWidget->request_handler_, &PatternRequestHandler::createdSequence, this, &FootstepPlanningPanel::createdSequence);
  connect(planningWidget->request_handler_, &PlanningRequestHandler::createdStepPlan, this, &FootstepPlanningPanel::createdStepPlan);
  connect(planningWidget->request_handler_, &PlanningRequestHandler::createdSequence, this, &FootstepPlanningPanel::createdSequence);
  // set Step plan if computed in other widget
  connect(patternWidget->request_handler_,  &PatternRequestHandler::createdStepPlan, planningWidget->request_handler_, &PlanningRequestHandler::setCurrentStepPlan);
  connect(planningWidget->request_handler_, &PlanningRequestHandler::createdStepPlan, patternWidget->request_handler_, &PatternRequestHandler::setCurrentStepPlan);

  // pass planning feedback to display
  connect(planningWidget->request_handler_, &PlanningRequestHandler::receivedPlanningFeedback, this, &FootstepPlanningPanel::sendPlanningFeedback);

  // ---------------------------

  // toggle sequence:
  connect(addSequenceToolButton, &QToolButton::toggled, patternWidget->ui->sequenceCheckBox, &QCheckBox::setChecked);
  connect(addSequenceToolButton, &QToolButton::toggled, planningWidget->ui->sequenceCheckBox, &QCheckBox::setChecked);

  // set parameter set
  connect(parameterSetComboBox, &QComboBox::currentTextChanged, patternWidget, &PatternWidget::setParameterSet);
  connect(parameterSetComboBox, &QComboBox::currentTextChanged, planningWidget, &PlanningWidget::setParameterSet);


  // clear, reset, undo ...
  connect(clearAllPushButton, &QPushButton::clicked, this, &FootstepPlanningPanel::clearAll);
  connect(clearAllPushButton, &QPushButton::clicked, this, &FootstepPlanningPanel::clearStepPlan);
  connect(clearIMPushButton, &QPushButton::clicked, this, &FootstepPlanningPanel::clearIM);
  connect(clearStepPlanPushButton, &QPushButton::clicked, this, &FootstepPlanningPanel::clearStepPlan);

  connect(this, &FootstepPlanningPanel::clearStepPlan, planningWidget->request_handler_, &PlanningRequestHandler::resetStepPlan);
  connect(this, &FootstepPlanningPanel::clearStepPlan, patternWidget->request_handler_, &PatternRequestHandler::resetStepPlan);

  connect(undoToolButton, &QToolButton::clicked, this, &FootstepPlanningPanel::undo);
  connect(removeSequenceToolButton, &QToolButton::clicked, this, &FootstepPlanningPanel::undoSequence);


  // pass start feet to display
  connect(patternWidget->request_handler_, &PatternRequestHandler::startFeetAnswer, this, &FootstepPlanningPanel::startFeetAnswer);
  connect(planningWidget->request_handler_, &PlanningRequestHandler::startFeetAnswer, this, &FootstepPlanningPanel::startFeetAnswer);

  // abort
  connect(abortPushButton, &QPushButton::clicked, planningWidget, &PlanningWidget::abort);
  connect(abortPushButton, &QPushButton::clicked, this, &FootstepPlanningPanel::abort);

  // progress bar
  connect(planningWidget->request_handler_, &PlanningRequestHandler::stepPlanGenerationStarted, this, &FootstepPlanningPanel::initializeGenerationProgressbar);
  connect(planningWidget->request_handler_, &PlanningRequestHandler::stepPlanGenerationFinished, this, &FootstepPlanningPanel::updateProgressBarGenerationState);
  connect(patternWidget->request_handler_, &PatternRequestHandler::stepPlanGenerationStarted, this, &FootstepPlanningPanel::initializeGenerationProgressbar);
  connect(patternWidget->request_handler_, &PatternRequestHandler::stepPlanGenerationFinished, this, &FootstepPlanningPanel::updateProgressBarGenerationState);

  // activate feet tool
  connect(startFeetToolButton, &QToolButton::toggled, this, [=](bool active){ Q_EMIT(feetToolActivated(START_FEET, active)); });
  connect(planningWidget, &PlanningWidget::setGoalActivated, this, [=](bool active) { Q_EMIT(feetToolActivated(GOAL_FEET, active));});

  // execute
  connect(executePushButton, &QPushButton::clicked, this, &FootstepPlanningPanel::executeRequested);
  //set default interaction mode
  connect(interactionModeComboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, [=](int index){
    InteractionMode mode;
    if(index==0)
      mode = PLANE;
    if(index == 1)
      mode = SIXDOF;
    if(index == 2)
      mode = FULLSIXDOF;
    Q_EMIT(interactionModeChanged(mode));});

  // Emit changed signal
  connect(interactionModeComboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &FootstepPlanningPanel::changed);
  connect(displayOptionsGroupBox, &QGroupBox::toggled, this, &FootstepPlanningPanel::changed);
  connect(patternWidget, &PatternWidget::changed, this, &FootstepPlanningPanel::changed);
  connect(planningWidget, &PlanningWidget::changed, this, &FootstepPlanningPanel::changed);

  // refresh parameter sets
  connect(refreshParameterSetsToolButton, SIGNAL(clicked()), parameterSetComboBox, SLOT(updateParameterSetNames()));

}

FootstepPlanningPanel::~FootstepPlanningPanel()
{
}

void FootstepPlanningPanel::save( rviz::Config config ) const
{
  patternWidget->save(config);
  planningWidget->save(config);
  config.mapSetValue("panel::DisplayOptionsGroupBox", displayOptionsGroupBox->isChecked());
  config.mapSetValue("panel::InteractionModeComboBox", interactionModeComboBox->currentIndex());
}

void FootstepPlanningPanel::load( const rviz::Config& config )
{
  patternWidget->load(config);
  planningWidget->load(config);
  bool checked;
  config.mapGetBool("panel::DisplayOptionsGroupBox", &checked);
  displayOptionsGroupBox->setChecked(checked);
  int index;
  config.mapGetInt("panel::InteractionModeComboBox", &index);
  interactionModeComboBox->setCurrentIndex(index);
}

// ----------- From to spin boxes
void FootstepPlanningPanel::updateFromTo(vigir_footstep_planning_msgs::StepPlan step_plan )
{
  displayStepsToSpinBox->setMaximum(step_plan.steps.size()-1);
  displayStepsFromSpinBox->setMaximum(step_plan.steps.size()-1);
}

void FootstepPlanningPanel::on_displayStepsFromSpinBox_valueChanged(int arg1)
{
  Q_EMIT(displayRangeChanged(arg1, displayStepsToSpinBox->value()));
}

void FootstepPlanningPanel::on_displayStepsToSpinBox_valueChanged(int arg1)
{
  Q_EMIT(displayRangeChanged(displayStepsFromSpinBox->value(), arg1));
}

// --------------------------------------------------------------------------------------------

// pass to planning and pattern widget / request handlers -------------------------------------------------

bool FootstepPlanningPanel::checkConnection()
{
  return (planningWidget->request_handler_->checkConnection() || patternWidget->request_handler_->checkConnection());
}

void FootstepPlanningPanel::setFrameID(const QString frame_id)
{
  planningWidget->setFrameID(frame_id);
  patternWidget->setFrameID(frame_id);
}

void FootstepPlanningPanel::setLastStep(int index)
{
  planningWidget->setLastStep(index);
  patternWidget->setLastStep(index);
}

void FootstepPlanningPanel::replanToIndex(int index)
{
  planningWidget->replanToIndex(index);
}

void FootstepPlanningPanel::updateGoalPose(vigir_footstep_planning_msgs::Feet goal_feet)
{
  planningWidget->updateGoalPose(goal_feet);
}

void FootstepPlanningPanel::releasePlaceFeet()
{
  startFeetToolButton->setChecked(false);
}

void FootstepPlanningPanel::setFeedbackRequested(bool requested)
{
  planningWidget->setFeedbackRequested(requested);
}

void FootstepPlanningPanel::startPoseRequested()
{
  planningWidget->startPoseRequested();
}

void FootstepPlanningPanel::setStepPlan(vigir_footstep_planning_msgs::StepPlan step_plan)
{
  planningWidget->request_handler_->setCurrentStepPlan(step_plan);
  patternWidget->request_handler_->setCurrentStepPlan(step_plan);
}


// Progress bar ------------------------------------------------------------------
void FootstepPlanningPanel::initializeExecutionProgressbar(int max)
{
  statusProgressBar->setMaximum(max);
  statusProgressBar->setValue(1);
  statusProgressBar->setTextVisible(true);
  statusProgressBar->setFormat("Step Plant sent to robot...");
}

void FootstepPlanningPanel::updateProgressBar(int val)
{
  // first progress step = step plan sent to robot
  statusProgressBar->setValue(val+1); // step index starts at 0
  statusProgressBar->setFormat(QString::number(val) + QString(" / ") + QString::number(statusProgressBar->maximum()-1));
}

void FootstepPlanningPanel::initializeGenerationProgressbar()
{
  statusProgressBar->setRange(0,0);
}

void FootstepPlanningPanel::updateProgressBarExecutionState(bool success)
{
  statusProgressBar->setRange(0,1);
  statusProgressBar->setValue(1);
  statusProgressBar->setValue(statusProgressBar->maximum());
  statusProgressBar->setTextVisible(true);

  if(success)
    statusProgressBar->setFormat("Execution finished, no errors.");
  else
    statusProgressBar->setFormat("!Error during execution of step plan!");
}


void FootstepPlanningPanel::updateProgressBarGenerationState(bool success)
{
  statusProgressBar->setRange(0,1);
  statusProgressBar->setValue(1);
  statusProgressBar->setValue(statusProgressBar->maximum());
  statusProgressBar->setTextVisible(true);

  if(success)
    statusProgressBar->setFormat("No errors.");
  else
    statusProgressBar->setFormat("!Error during computation of step plan!");

}

} // end namespace vigir_footstep_planning_rviz_plugin
