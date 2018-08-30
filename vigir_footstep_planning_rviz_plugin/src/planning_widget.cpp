#include <vigir_footstep_planning_rviz_plugin/planning_widget.h>

#include <rviz/config.h>

namespace vigir_footstep_planning_rviz_plugin {

PlanningWidget::PlanningWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PlanningWidget)
{
    ui->setupUi(this);
    request_handler_ = new PlanningRequestHandler();


    connect(ui->planningSetGoalToolButton, &QToolButton::toggled, this, &PlanningWidget::setGoalActivated);

    connect(ui->startFootComboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), request_handler_, &PlanningRequestHandler::setStartStepIndex);
    connect(ui->startFootComboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &PlanningWidget::changed);

    connect(ui->maxNofStepsSpinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), request_handler_, &PlanningRequestHandler::setMaxNofSteps);
    connect(ui->maxNofStepsSpinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &PlanningWidget::changed);


    connect(ui->maxTimeDoubleSpinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), request_handler_, &PlanningRequestHandler::setMaxPlanningTime);
    connect(ui->maxTimeDoubleSpinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &PlanningWidget::changed);

    connect(ui->ratioDoubleSpinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), request_handler_, &PlanningRequestHandler::setMaxPathLengthRatio);
    connect(ui->ratioDoubleSpinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),this, &PlanningWidget::changed);

    connect(ui->autoComputationCheckBox, &QCheckBox::toggled, request_handler_, &PlanningRequestHandler::setActivateOnPlaceFeet);
    connect(ui->autoComputationCheckBox, &QCheckBox::toggled, this, &PlanningWidget::changed);

    connect(ui->sequenceCheckBox, &QCheckBox::toggled, request_handler_, &PlanningRequestHandler::setAppend);
    connect(ui->sequenceCheckBox, &QCheckBox::toggled, this, &PlanningWidget::changed);

    connect(ui->moreOptionsGroupBox, &QGroupBox::toggled, this, &PlanningWidget::changed);
}

PlanningWidget::~PlanningWidget()
{
  
  delete ui;
  delete request_handler_;
}

void PlanningWidget::initialize()
{
  ui->moreOptionsGroupBox->setChecked(false);
}

void PlanningWidget::save( rviz::Config config ) const
{
  config.mapSetValue("planning::PlanningModeComboBox", ui->planningModeComboBox->currentIndex());
  config.mapSetValue("planning::AutoComputationCheckBox", ui->autoComputationCheckBox->isChecked());
  config.mapSetValue("planning::MoreOptionsGroupBox", ui->moreOptionsGroupBox->isChecked());
  config.mapSetValue("planning::StartFootComboBox", ui->startFootComboBox->currentIndex());
  config.mapSetValue("planning::MaxTimeSpinBox", ui->maxTimeDoubleSpinBox->value());
  config.mapSetValue("planning::MaxNofStepsSpingBox", ui->maxNofStepsSpinBox->value());
  config.mapSetValue("planning::RatioDoubleSpinBox", ui->ratioDoubleSpinBox->value());
}

void PlanningWidget::load( const rviz::Config& config )
{
  bool checked;
  float val_f;
  int val_i;

  config.mapGetInt("planning::PlanningModeComboBox", &val_i);
  ui->planningModeComboBox->setCurrentIndex(val_i);

  config.mapGetBool("planning::AutoComputationCheckBox", &checked);
  ui->autoComputationCheckBox->setChecked(checked);

  config.mapGetBool("planning::MoreOptionsGroupBox", &checked);
  ui->moreOptionsGroupBox->setChecked(checked);

  config.mapGetInt("planning::StartFootComboBox", &val_i);
  ui->startFootComboBox->setCurrentIndex(val_i);

  config.mapGetFloat("planning::MaxTimeSpinBox", &val_f);
  ui->maxTimeDoubleSpinBox->setValue(static_cast<double>(val_f));

  config.mapGetInt("planning::MaxNofStepsSpingBox", &val_i);
  ui->maxNofStepsSpinBox->setValue(val_i);

  config.mapGetFloat("planning::RatioDoubleSpinBox", &val_f);
  ui->ratioDoubleSpinBox->setValue(static_cast<double>(val_f));
}

// ========= Handling Display - Request Handler Communication  ===========================================
void PlanningWidget::startPoseRequested()
{
  request_handler_->requestStartPose();
}

void PlanningWidget::updateGoalPose(vigir_footstep_planning_msgs::Feet goal_pose)
{
  ui->planningSetGoalToolButton->setChecked(false);
  request_handler_->setGoal(goal_pose);
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



}// End namespace vigir_footstep_planning_rviz_plugin
