#include <vigir_footstep_planning_rviz_plugin/pattern_widget.h>

#include <rviz/config.h>

namespace vigir_footstep_planning_rviz_plugin {

PatternWidget::PatternWidget(QWidget *parent) :
    WidgetBase(parent),
    ui(new Ui::PatternWidget)
{
    ui->setupUi(this);
    request_handler_ = new PatternRequestHandler();

    //Pattern Planning Parameter Signal Slot connection, set request handler values --------------------------
    connect(ui->patternNOStepsSpinBox,SIGNAL(valueChanged(int)), request_handler_, SLOT(setNoSteps(int)));
    connect(ui->patternStepDistanceSpinBox,SIGNAL(valueChanged(double)), request_handler_, SLOT(setStepDistance(double)));
    connect(ui->patternSideStepDoubleSpinBox, SIGNAL(valueChanged(double)), request_handler_, SLOT(setSideStep(double)));
    connect(ui->patternRotationSpinBox, SIGNAL(valueChanged(int)), request_handler_, SLOT(setTurnAngle(int)));
    connect(ui->patternDeltaZDoubleSpinBox, SIGNAL(valueChanged(double)), request_handler_, SLOT(setdz(double)));
    //More Pattern Parameter
    connect(ui->patternStartIndexSpinBox, SIGNAL(valueChanged(int)), request_handler_,SLOT(setStartStepIndex(int)));
    connect(ui->patternRollSpinBox, SIGNAL(valueChanged(int)),request_handler_,SLOT(setRoll(int)));
    connect(ui->patternPitchSpinBox, SIGNAL(valueChanged(int)), request_handler_,SLOT(setPitch(int)));
    connect(ui->patternStartFootComboBox, SIGNAL(currentIndexChanged(int)), request_handler_, SLOT(setStartFoot(int)));
    //CheckBoxes:
    connect(ui->patternClosingStepCheckBox, SIGNAL(stateChanged(int)), request_handler_, SLOT(setClosingStep(int)));
    connect(ui->patternExtraSeperationCheckBox, SIGNAL(stateChanged(int)), request_handler_,SLOT(setExtraSeperation(int)));
    connect(ui->patternTerrainModelCheckBox, SIGNAL(stateChanged(int)), request_handler_,SLOT(setUseTerrainModel(int)));
    connect(ui->patternOverride3DCheckBox, SIGNAL(stateChanged(int)), request_handler_,SLOT(setOverride3D(int)));
}

PatternWidget::~PatternWidget()
{
  delete ui;
  delete request_handler_;
}

void PatternWidget::initialize()
{
  request_handler_->initialize();
  ui->patternMoreOptionsCheckbox->setChecked(false);
}

void PatternWidget::save( rviz::Config config ) const
{
  config.mapSetValue("sequenceCheckBox", ui->sequenceCheckBox->isChecked());

  config.mapSetValue("NOStepsSpinBox", ui->patternNOStepsSpinBox->value());
  config.mapSetValue("StepDistanceSpinBox", ui->patternStepDistanceSpinBox->value());
  config.mapSetValue("SideStepDoubleSpinBox", ui->patternSideStepDoubleSpinBox->value());
  config.mapSetValue("RotationSpinBox", ui->patternRotationSpinBox->value());
  config.mapSetValue("DeltaZDoubleSpinBox", ui->patternDeltaZDoubleSpinBox->value());
  config.mapSetValue("ClosingStepCheckBox", ui->patternClosingStepCheckBox->isChecked());
  // More options
  config.mapSetValue("MoreOptionsCheckbox", ui->patternMoreOptionsCheckbox->isChecked());
  config.mapSetValue("StartIndexSpinBox", ui->patternStartIndexSpinBox->value());
  config.mapSetValue("RollSpinBox", ui->patternRollSpinBox->value());
  config.mapSetValue("PitchSpinBox", ui->patternPitchSpinBox->value());
  config.mapSetValue("ExtraSeperationCheckBox", ui->patternExtraSeperationCheckBox->isChecked());
  config.mapSetValue("TerrainModelCheckBox", ui->patternTerrainModelCheckBox->isChecked());
  config.mapSetValue("Override3DCheckBox", ui->patternOverride3DCheckBox->isChecked());
  config.mapSetValue("StartFootComboBox", ui->patternStartFootComboBox->currentIndex());
}

void PatternWidget::load( const rviz::Config& config )
{
  bool checked;
  float val_f;
  int val_i;

  config.mapGetBool("sequenceCheckBox", &checked);
  ui->sequenceCheckBox->setChecked(checked);

  config.mapGetInt("NOStepsSpinBox", &val_i);
  ui->patternNOStepsSpinBox->setValue(val_i);

  config.mapGetFloat("StepDistanceSpinBox", &val_f);
  ui->patternStepDistanceSpinBox->setValue(static_cast<double>(val_f));

  config.mapGetFloat("SideStepDoubleSpinBox",&val_f);
  ui->patternSideStepDoubleSpinBox->setValue(static_cast<double>(val_f));

  config.mapGetInt("RotationSpinBox", &val_i);
  ui->patternRotationSpinBox->setValue(val_i);

  config.mapGetFloat("DeltaZDoubleSpinBox", &val_f);
  ui->patternDeltaZDoubleSpinBox->setValue(static_cast<double>(val_f));

  config.mapGetBool("ClosingStepCheckBox", &checked);
  ui->patternClosingStepCheckBox->setChecked(checked);

  // More options --------------------------------------
  config.mapGetBool("MoreOptionsCheckbox", &checked);
  ui->patternMoreOptionsCheckbox->setChecked(checked);

  config.mapGetInt("StartIndexSpinBox", &val_i);
  ui->patternStartIndexSpinBox->setValue(val_i);

  config.mapGetInt("RollSpinBox", &val_i);
  ui->patternRollSpinBox->setValue(val_i);

  config.mapGetInt("PitchSpinBox", &val_i);
  ui->patternPitchSpinBox->setValue(val_i);

  config.mapGetBool("ExtraSeperationCheckBox", &checked);
  ui->patternExtraSeperationCheckBox->setChecked(checked);

  config.mapGetBool("TerrainModelCheckBox", &checked);
  ui->patternTerrainModelCheckBox->setChecked(checked);

  config.mapGetBool("Override3DCheckBox", &checked);
  ui->patternOverride3DCheckBox->setChecked(checked);

  config.mapGetInt("StartFootComboBox", &val_i);
  ui->patternStartFootComboBox->setCurrentIndex(val_i);
}

// ========= Handling Display - Request Handler Communication  ===========================================

void PatternWidget::startPoseRequested()
{
  request_handler_->requestStartPose();
}

void PatternWidget::setCurrentStepPlan(vigir_footstep_planning_msgs::StepPlan step_plan)
{
  request_handler_->setCurrentStepPlan(step_plan);
}

void PatternWidget::setFrameID(const QString frame_id)
{
  request_handler_->setFrameID(frame_id);
}

void PatternWidget::setParameterSet(QString parameter_set_name)
{
  request_handler_->setParameterSet(parameter_set_name);
}

// invoked from foot visual
void PatternWidget::setLastStep(int last_index)
{
  request_handler_->setCurrentGoal(last_index);
}

// ========== Panel Communication =============================
void PatternWidget::resetValues()
{
  //ROS_INFO("ResetValues PatternWidget, not yet implemented completely, TODO");
}

void PatternWidget::abort()
{
  request_handler_->cancelGoals();
}

// ========== UI Slots =================================================================
void PatternWidget::on_patternMoreOptionsCheckbox_stateChanged(int arg1)
{
    if(arg1 == Qt::Checked)
      ui->patternMoreOptionsGroupBox->show();
    if(arg1 == Qt::Unchecked)
      ui->patternMoreOptionsGroupBox->hide();
}
// ---
// ----------- Send request ------
void PatternWidget::on_patternSamplingPushButton_clicked()
{
  request_handler_->sendPatternRequest(vigir_footstep_planning_msgs::PatternParameters::SAMPLING, ui->sequenceCheckBox->isChecked());
}

void PatternWidget::on_patternForwardPushButton_clicked()
{
  request_handler_->sendPatternRequest(vigir_footstep_planning_msgs::PatternParameters::FORWARD, ui->sequenceCheckBox->isChecked());
}

void PatternWidget::on_patternRotateLeftPushButton_clicked()
{
  request_handler_->sendPatternRequest(vigir_footstep_planning_msgs::PatternParameters::ROTATE_LEFT, ui->sequenceCheckBox->isChecked());
}

void PatternWidget::on_patternRotateRightPushButton_clicked()
{
  request_handler_->sendPatternRequest(vigir_footstep_planning_msgs::PatternParameters::ROTATE_RIGHT, ui->sequenceCheckBox->isChecked());
}

void PatternWidget::on_patternStrafeLeftPushButton_clicked()
{
  request_handler_->sendPatternRequest(vigir_footstep_planning_msgs::PatternParameters::STRAFE_LEFT, ui->sequenceCheckBox->isChecked());
}

void PatternWidget::on_patternStrafeRightPushButton_clicked()
{
  request_handler_->sendPatternRequest(vigir_footstep_planning_msgs::PatternParameters::STRAFE_RIGHT, ui->sequenceCheckBox->isChecked());
}

void PatternWidget::on_patternBackwardPushButton_clicked()
{
  request_handler_->sendPatternRequest(vigir_footstep_planning_msgs::PatternParameters::BACKWARD, ui->sequenceCheckBox->isChecked());
}

void PatternWidget::on_patternStepUpPushButton_clicked()
{
  request_handler_->sendPatternRequest(vigir_footstep_planning_msgs::PatternParameters::STEP_UP, ui->sequenceCheckBox->isChecked());
}

void PatternWidget::on_patternStepOverPushButton_clicked()
{
  request_handler_->sendPatternRequest(vigir_footstep_planning_msgs::PatternParameters::STEP_OVER, ui->sequenceCheckBox->isChecked());
}

void PatternWidget::on_patternStepDownPushButton_clicked()
{
  request_handler_->sendPatternRequest(vigir_footstep_planning_msgs::PatternParameters::STEP_DOWN, ui->sequenceCheckBox->isChecked());
}

void PatternWidget::on_patternWideStancePushButton_clicked()
{
  request_handler_->sendPatternRequest(vigir_footstep_planning_msgs::PatternParameters::WIDE_STANCE, ui->sequenceCheckBox->isChecked());
}

void PatternWidget::on_patternCenterLeftPushButton_clicked()
{
  request_handler_->sendPatternRequest(vigir_footstep_planning_msgs::PatternParameters::FEET_REALIGN_ON_LEFT, ui->sequenceCheckBox->isChecked());
}

void PatternWidget::on_patternCenterFeetPushButton_clicked()
{
  request_handler_->sendPatternRequest(vigir_footstep_planning_msgs::PatternParameters::FEET_REALIGN_ON_CENTER, ui->sequenceCheckBox->isChecked());
}

void PatternWidget::on_patternCenterRightPushButton_clicked()
{
  request_handler_->sendPatternRequest(vigir_footstep_planning_msgs::PatternParameters::FEET_REALIGN_ON_RIGHT, ui->sequenceCheckBox->isChecked());
}
// ------------------ end send request ---------------------





}// End namespace vigir_footstep_planning_rviz_plugin
