#include <vigir_footstep_planning_rviz_plugin/planning_widget.h>

#include <rviz/config.h>

namespace vigir_footstep_planning_rviz_plugin {

PlanningWidget::PlanningWidget(QWidget *parent) :
    WidgetBase(parent),
    ui(new Ui::PlanningWidget)
{
    ui->setupUi(this);
    request_handler_ = new PlanningRequestHandler();

    // ui signals
    connect(ui->planningSetGoalToolButton, SIGNAL(toggled(bool)), this, SLOT(emitFeetToolActivated(bool)));

    connect(ui->startFootComboBox, SIGNAL(currentIndexChanged(int)), request_handler_, SLOT(setStartStepIndex(int)));
    connect(ui->maxNofStepsSpinBox, SIGNAL(valueChanged(int)), request_handler_, SLOT(setMaxNofSteps(int)));
    connect(ui->maxTimeDoubleSpinBox, SIGNAL(valueChanged(double)), request_handler_, SLOT(setMaxPlanningTime(double)));
    connect(ui->ratioDoubleSpinBox, SIGNAL(valueChanged(double)), request_handler_, SLOT(setMaxPathLengthRatio(double)));
    connect(ui->autoComputationCheckBox, SIGNAL(toggled(bool)), request_handler_, SLOT(setActivateOnPlaceFeet(bool)));
    connect(ui->sequenceCheckBox, SIGNAL(toggled(bool)), request_handler_, SLOT(setAppend(bool)));
}

PlanningWidget::~PlanningWidget()
{
  delete ui;
  delete request_handler_;
}

void PlanningWidget::initialize()
{
  ui->moreOptionsGroupBox->setChecked(false);

  //todo
  // request_handler_->initialize();
}

void PlanningWidget::save( rviz::Config config ) const
{
  config.mapSetValue("sequence", ui->sequenceCheckBox->isChecked());
}

void PlanningWidget::load( const rviz::Config& config )
{
  bool checked;
  config.mapGetBool("sequence", &checked);
  ui->sequenceCheckBox->setChecked(checked);
}

// ========= Handling Display - Request Handler Communication  ===========================================
void PlanningWidget::startPoseRequested()
{
  request_handler_->requestStartPose();
}

void PlanningWidget::handleGoalPose(Ogre::Vector3 position, Ogre::Quaternion orientation)
{
  ui->planningSetGoalToolButton->setChecked(false);
  request_handler_->setGoal(position, orientation);
}

void PlanningWidget::setCurrentStepPlan(vigir_footstep_planning_msgs::StepPlan step_plan)
{
  request_handler_->setCurrentStepPlan(step_plan);
}

void PlanningWidget::setFeedbackRequested(bool requested)
{
  request_handler_->setFeedbackRequested(requested);
}

void PlanningWidget::setFrameID(const QString frame_id)
{
  request_handler_->setFrameID(frame_id);
}

void PlanningWidget::setParameterSet(QString parameter_set_name)
{
  request_handler_->setParameterSet(parameter_set_name);
}

// invoked from foot visual
void PlanningWidget::setLastStep(int last_index)
{
  request_handler_->setCurrentGoal(last_index);
}

void PlanningWidget::replanToIndex(int index)
{
  request_handler_->replanToIndex(index);
}


// ======= Panel Communication ========================================
void PlanningWidget::resetValues()
{
  //Todo
}

void PlanningWidget::abort()
{
  request_handler_->cancelGoals();
}

// ========== UI Slots =================================================================

void PlanningWidget::on_planningComputeCommandLinkButton_clicked()
{
  request_handler_->setPlanningMode(ui->planningModeComboBox->currentIndex());
  request_handler_->sendPlanningRequest(ui->sequenceCheckBox->isChecked());
}

// Emit signals:
void PlanningWidget::emitFeetToolActivated(bool activated)
{
  Q_EMIT(feetToolActivated(activated));
}



}// End namespace vigir_footstep_planning_rviz_plugin
