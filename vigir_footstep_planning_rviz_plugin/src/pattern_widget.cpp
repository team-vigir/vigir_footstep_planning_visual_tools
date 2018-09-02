#include <vigir_footstep_planning_rviz_plugin/pattern_widget.h>

#include <rviz/config.h>

namespace vigir_footstep_planning_rviz_plugin {

PatternWidget::PatternWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PatternWidget)
{
    ui->setupUi(this);
    request_handler_ = new PatternRequestHandler();

    //Pattern Planning Parameter Signal Slot connection, set request handler values --------------------------
    connect(ui->patternNOStepsSpinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), request_handler_, &PatternRequestHandler::setNoSteps);
    connect(ui->patternNOStepsSpinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &PatternWidget::changed);

    connect(ui->patternStepDistanceSpinBox,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), request_handler_, &PatternRequestHandler::setStepDistance);
    connect(ui->patternStepDistanceSpinBox,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &PatternWidget::changed);

    connect(ui->patternSideStepDoubleSpinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), request_handler_, &PatternRequestHandler::setSideStep);
    connect(ui->patternSideStepDoubleSpinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &PatternWidget::changed);

    connect(ui->patternRotationSpinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), request_handler_, &PatternRequestHandler::setTurnAngle);
    connect(ui->patternRotationSpinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &PatternWidget::changed);

    connect(ui->patternDeltaZDoubleSpinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), request_handler_, &PatternRequestHandler::setdz);
    connect(ui->patternDeltaZDoubleSpinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &PatternWidget::changed);

    //More Pattern Parameter
    connect(ui->patternStartIndexSpinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), request_handler_, &PatternRequestHandler::setStartStepIndex);
    connect(ui->patternStartIndexSpinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &PatternWidget::changed);

    connect(ui->patternRollSpinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), request_handler_, &PatternRequestHandler::setRoll);
    connect(ui->patternRollSpinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &PatternWidget::changed);

    connect(ui->patternPitchSpinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), request_handler_,&PatternRequestHandler::setPitch);
    connect(ui->patternPitchSpinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &PatternWidget::changed);

    connect(ui->patternStartFootComboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), request_handler_, &PatternRequestHandler::setStartFoot);
    connect(ui->patternStartFootComboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &PatternWidget::changed);


    //CheckBoxes:
    connect(ui->patternClosingStepCheckBox, &QCheckBox::stateChanged, request_handler_, &PatternRequestHandler::setClosingStep);
    connect(ui->patternClosingStepCheckBox, &QCheckBox::stateChanged, this, &PatternWidget::changed);

    connect(ui->patternExtraSeperationCheckBox, &QCheckBox::stateChanged, request_handler_, &PatternRequestHandler::setExtraSeperation);
    connect(ui->patternExtraSeperationCheckBox, &QCheckBox::stateChanged, this, &PatternWidget::changed);

    connect(ui->patternTerrainModelCheckBox, &QCheckBox::stateChanged, request_handler_, &PatternRequestHandler::setUseTerrainModel);
    connect(ui->patternTerrainModelCheckBox, &QCheckBox::stateChanged, this, &PatternWidget::changed);

    connect(ui->patternOverride3DCheckBox, &QCheckBox::stateChanged, request_handler_, &PatternRequestHandler::setOverride3D);
    connect(ui->patternOverride3DCheckBox, &QCheckBox::stateChanged, this, &PatternWidget::changed);

    connect(ui->patternMoreOptionsGroupBox, &QGroupBox::toggled, this, &PatternWidget::changed);
}

PatternWidget::~PatternWidget()
{
  
  delete ui;
  delete request_handler_;
}

void PatternWidget::initialize()
{
  ui->patternMoreOptionsGroupBox->setChecked(false);
}

void PatternWidget::save( rviz::Config config ) const
{
  config.mapSetValue("pattern::NOStepsSpinBox", ui->patternNOStepsSpinBox->value());
  config.mapSetValue("pattern::StepDistanceSpinBox", ui->patternStepDistanceSpinBox->value());
  config.mapSetValue("pattern::SideStepDoubleSpinBox", ui->patternSideStepDoubleSpinBox->value());
  config.mapSetValue("pattern::RotationSpinBox", ui->patternRotationSpinBox->value());
  config.mapSetValue("pattern::DeltaZDoubleSpinBox", ui->patternDeltaZDoubleSpinBox->value());
  config.mapSetValue("pattern::ClosingStepCheckBox", ui->patternClosingStepCheckBox->isChecked());
  // More options
  config.mapSetValue("pattern::MoreOptionsGroupBox", ui->patternMoreOptionsGroupBox->isChecked());
  config.mapSetValue("pattern::StartIndexSpinBox", ui->patternStartIndexSpinBox->value());
  config.mapSetValue("pattern::RollSpinBox", ui->patternRollSpinBox->value());
  config.mapSetValue("pattern::PitchSpinBox", ui->patternPitchSpinBox->value());
  config.mapSetValue("pattern::ExtraSeperationCheckBox", ui->patternExtraSeperationCheckBox->isChecked());
  config.mapSetValue("pattern::TerrainModelCheckBox", ui->patternTerrainModelCheckBox->isChecked());
  config.mapSetValue("pattern::Override3DCheckBox", ui->patternOverride3DCheckBox->isChecked());
  config.mapSetValue("pattern::StartFootComboBox", ui->patternStartFootComboBox->currentIndex());
}

void PatternWidget::load( const rviz::Config& config )
{
  bool checked;
  float val_f;
  int val_i;

  config.mapGetInt("pattern::NOStepsSpinBox", &val_i);
  ui->patternNOStepsSpinBox->setValue(val_i);

  config.mapGetFloat("pattern::StepDistanceSpinBox", &val_f);
  ui->patternStepDistanceSpinBox->setValue(static_cast<double>(val_f));

  config.mapGetFloat("pattern::SideStepDoubleSpinBox",&val_f);
  ui->patternSideStepDoubleSpinBox->setValue(static_cast<double>(val_f));

  config.mapGetInt("pattern::RotationSpinBox", &val_i);
  ui->patternRotationSpinBox->setValue(val_i);

  config.mapGetFloat("pattern::DeltaZDoubleSpinBox", &val_f);
  ui->patternDeltaZDoubleSpinBox->setValue(static_cast<double>(val_f));

  config.mapGetBool("pattern::ClosingStepCheckBox", &checked);
  ui->patternClosingStepCheckBox->setChecked(checked);

  // More options --------------------------------------
  config.mapGetBool("pattern::MoreOptionsGroupBox", &checked);
  ui->patternMoreOptionsGroupBox->setChecked(checked);

  config.mapGetInt("pattern::StartIndexSpinBox", &val_i);
  ui->patternStartIndexSpinBox->setValue(val_i);

  config.mapGetInt("pattern::RollSpinBox", &val_i);
  ui->patternRollSpinBox->setValue(val_i);

  config.mapGetInt("pattern::PitchSpinBox", &val_i);
  ui->patternPitchSpinBox->setValue(val_i);

  config.mapGetBool("pattern::ExtraSeperationCheckBox", &checked);
  ui->patternExtraSeperationCheckBox->setChecked(checked);

  config.mapGetBool("pattern::TerrainModelCheckBox", &checked);
  ui->patternTerrainModelCheckBox->setChecked(checked);

  config.mapGetBool("pattern::Override3DCheckBox", &checked);
  ui->patternOverride3DCheckBox->setChecked(checked);

  config.mapGetInt("pattern::StartFootComboBox", &val_i);
  ui->patternStartFootComboBox->setCurrentIndex(val_i);
}

// ========= Handling Display - Request Handler Communication  ===========================================

void PatternWidget::startPoseRequested()
{
  request_handler_->requestStartPose();
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
void PatternWidget::abort()
{
  request_handler_->cancelGoals();
}

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
