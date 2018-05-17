#include <vigir_footstep_planning_rviz_plugin/planning_widget.h>

#include <rviz/config.h>

namespace vigir_footstep_planning_rviz_plugin {

PlanningWidget::PlanningWidget(QWidget *parent) :
    WidgetBase(parent),
    ui(new Ui::PlanningWidget)
{
    ui->setupUi(this);
    ui->moreOptionsGroupBox->setChecked(false);
    request_handler_ = new PlanningRequestHandler();

    // ui signals
    connect(ui->planningSetGoalToolButton, SIGNAL(toggled(bool)), this, SLOT(emitFeetToolActivated(bool)));
}

PlanningWidget::~PlanningWidget()
{
  delete ui;
  delete request_handler_;
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
  //ROS_INFO("PlanningWidget::resetValues not yet implemented yet completely. TODO");
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
