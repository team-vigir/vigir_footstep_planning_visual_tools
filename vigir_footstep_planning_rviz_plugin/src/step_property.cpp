#include <vigir_footstep_planning_rviz_plugin/step_property.h>
#include <vigir_footstep_planning_rviz_plugin/common/ogre_visualization_msgs_functions.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/float_property.h>


typedef vigir_footstep_planning_msgs::Step StepMsg;
typedef vigir_footstep_planning_msgs::StepPlan StepPlanMsg;

namespace vigir_footstep_planning_rviz_plugin {

StepProperty::StepProperty(const StepMsg& step, Property *parent) :
    rviz::StatusProperty("Step" + QString::number(step.step_index), "", Ok, parent)
  , internal_update(false)
  , selected(false)
{
  addStep(step);
}


StepProperty::StepProperty() :
  rviz::StatusProperty("", "", Ok, 0)
{

}

StepProperty::~StepProperty()
{
}

// ---------------- Add properties:
void StepProperty::addStep(const StepMsg& step)
{
  addStepProperties(step);
  position_ = new rviz::VectorProperty("Position",
                                       getOgreVector(step.foot.pose.position),
                                       "current position of the step",
                                       this, SLOT(onPoseEdited()),
                                       this);
  orientation_ = new rviz::QuaternionProperty("Orientation",
                                              getOgreQuaternion(step.foot.pose.orientation),
                                              "current orientation of the step",
                                              this,
                                              SLOT(onPoseEdited()),
                                              this);

  edit_step_property = new Property("Step Edited:", QVariant(),"Display values of step editing.", this);

  original_position_ = new rviz::VectorProperty("Reference Position",
                                                getOgreVector(step.foot.pose.position),
                                                "Position of step when it was selected.",
                                                edit_step_property);
  position_shift_ = new rviz::VectorProperty("Position Shift",
                                             Ogre::Vector3::ZERO,
                                             "Shift of step from reference position.",
                                             edit_step_property,
                                             SLOT(onShiftChanged()),
                                             this);
  original_position_->setReadOnly(true);
}

void StepProperty::addStepProperties(const StepMsg & step)
{

  property_container_ = new rviz::Property("Properties", QVariant(), "Properties of this step", this);
  is_valid_ = new rviz::StatusProperty("Valid",
                                        step.valid ? "Step reached from predecessor" : "Step not reached from predecessor",
                                        step.valid ? Ok : Warn,
                                        property_container_);
  cost_ = new rviz::FloatProperty("Cost", step.cost, "Cost to make a step", property_container_);
  risk_ = new rviz::FloatProperty("Risk", step.risk, "Risk of step", property_container_);
  colliding_ = new rviz::StatusProperty("Colliding",
                                        step.colliding ? "Step is colliding" : "Step not colliding",
                                        step.colliding ? Warn : Ok,
                                        property_container_);
  if(!step.valid || step.colliding)
    this->setLevel(Warn);
  else
    this->setLevel(Ok);

  sway_duration_ = new rviz::FloatProperty("Sway duration", step.sway_duration, "Sway duration", property_container_);
  step_duration_ = new rviz::FloatProperty("Step duration", step.step_duration, "Step duration", property_container_);
  swing_height_ = new rviz::FloatProperty("Swing height", step.sway_duration, "Swing height", property_container_);

}

// ---------------- Update properties:

void StepProperty::updateStep(vigir_footstep_planning_msgs::Step updated_step)
{
  updatePosition(updated_step);
  updateStepProperty(updated_step);
}

void StepProperty::updateStep(vigir_footstep_planning_msgs::EditStep edit_step)
{
  if(edit_step.plan_mode == vigir_footstep_planning_msgs::EditStep::EDIT_MODE_FULL)
  {
    updateStep(edit_step.step);
  }
}

void StepProperty::updatePosition(const StepMsg & step)
{
  internal_update = true;
  position_->setVector(getOgreVector(step.foot.pose.position));
  orientation_->setQuaternion(getOgreQuaternion(step.foot.pose.orientation));

  if(selected)
  {
    Ogre::Vector3 shift = position_->getVector() - original_position_->getVector();
    position_shift_->setVector(shift);
  }
  else
    original_position_->setVector(position_->getVector());
}

void StepProperty::updateStepProperty(const vigir_footstep_planning_msgs::Step &step)
{
  is_valid_->setLevel(step.valid ? Ok : Warn);
  is_valid_->setValue(step.valid ? "Step reached from predecessor" : "Step not reached from predecessor");

  cost_->setFloat(step.cost);
  risk_->setFloat(step.risk);

  colliding_->setLevel(step.colliding ? Warn : Ok);
  colliding_->setValue(step.colliding ? "Step is colliding" : "Step not colliding");

  updateStatus();
}

// -----------------------------------

void StepProperty::setValid(bool valid)
{
  is_valid_->setLevel(valid ? Ok : Warn);
  is_valid_->setValue(valid ? "Step reached from predecessor" : "Step not reached from predecessor");

  updateStatus();
}

void StepProperty::setExpanded(bool expanded)
{
  expanded ? this->expand() : this->collapse();
  selected = expanded;
}

// ---------------- Invoked when edited:
void StepProperty::onPoseEdited()
{
  if(!internal_update)
  {
    Q_EMIT(stepPoseChanged(position_->getVector(), orientation_->getQuaternion()));
  }
  position_shift_->setVector(position_->getVector() - original_position_->getVector());
  //internal upadate is finished:
  internal_update = false;
}
void StepProperty::onShiftChanged()
{
  position_->setVector(position_shift_->getVector() + original_position_->getVector());
}

// update Level of step property
void StepProperty::updateStatus()
{
  if(is_valid_->getLevel() == Ok && colliding_->getLevel() == Ok)
  {
    this->setLevel(Ok);
  }
  else
    this->setLevel(Warn);
}


}// End namespace vigir_footstep_planning_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning_rviz_plugin::StepProperty,rviz::StatusProperty )
