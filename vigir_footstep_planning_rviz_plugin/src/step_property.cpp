#include <vigir_footstep_planning_rviz_plugin/step_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/float_property.h>

typedef vigir_footstep_planning_msgs::Step StepMsg;
typedef vigir_footstep_planning_msgs::StepPlan StepPlanMsg;

namespace vigir_footstep_planning_rviz_plugin {

StepProperty::StepProperty(const StepMsg& step, Property *parent) :
    rviz::StatusProperty("Step" + QString::number(step.step_index), "", rviz::StatusLevel::Ok, parent)
  , internal_update(false)
  , selected(false)
{
  addStep(step);
}


StepProperty::StepProperty() :
  rviz::StatusProperty("", "", rviz::StatusLevel::Ok, 0)
{

}

StepProperty::~StepProperty()
{
}



void StepProperty::addStep(const StepMsg& step)
{
  addStepProperties(step);
  position_ = new rviz::VectorProperty("Position",
                                       Ogre::Vector3(step.foot.pose.position.x, step.foot.pose.position.y, step.foot.pose.position.z),
                                       "current position of the step",
                                       this, SLOT(invokePoseUpdate()),
                                       this);

  edit_step_property = new Property("Step Edited:", QVariant(),"Display values of step editing.", this);

  original_position_ = new rviz::VectorProperty("Reference Position",
                                                Ogre::Vector3(step.foot.pose.position.x, step.foot.pose.position.y, step.foot.pose.position.z),
                                                "Position of step when it was selected.",
                                                edit_step_property);
  position_shift_ = new rviz::VectorProperty("Position Shift",
                                             Ogre::Vector3::ZERO,
                                             "Shift of step from original position.",
                                             edit_step_property,
                                             SLOT(onShiftChanged()),
                                             this);


  orientation_ = new rviz::QuaternionProperty("Orientation",
                                              Ogre::Quaternion(step.foot.pose.orientation.w, step.foot.pose.orientation.x, step.foot.pose.orientation.y, step.foot.pose.orientation.z),
                                              "current orientation of the step",
                                              this,
                                              SLOT(invokePoseUpdate()),
                                              this);
  original_orientation_ = new rviz::QuaternionProperty("Reference Orientation",
                                                       Ogre::Quaternion(step.foot.pose.orientation.w, step.foot.pose.orientation.x, step.foot.pose.orientation.y, step.foot.pose.orientation.z),
                                                       "Orientation of step when it was selected.",
                                                       edit_step_property);

  original_orientation_->setReadOnly(true);
  original_position_->setReadOnly(true);
}


void StepProperty::updateStep(vigir_footstep_planning_msgs::Step updated_step)
{
  updatePosition(updated_step);
  updateStepProperty(updated_step);
}

void StepProperty::updateStep(vigir_footstep_planning_msgs::EditStep edit_step)
{
  if(edit_step.plan_mode == vigir_footstep_planning_msgs::EditStep::EDIT_MODE_FULL)
  {
    updatePosition(edit_step.step);
  }
}

void StepProperty::updatePosition(const StepMsg & step)
{
  internal_update = true;
  position_->setVector(Ogre::Vector3(step.foot.pose.position.x,
                                   step.foot.pose.position.y,
                                   step.foot.pose.position.z));
  orientation_->setQuaternion(Ogre::Quaternion(step.foot.pose.orientation.w,
                                         step.foot.pose.orientation.x,
                                         step.foot.pose.orientation.y,
                                         step.foot.pose.orientation.z));

  if(selected)
  {
    Ogre::Vector3 shift = position_->getVector() - original_position_->getVector();
    position_shift_->setVector(shift);
  }
  else
  {
    original_position_->setVector(position_->getVector());
    original_orientation_->setQuaternion(orientation_->getQuaternion());
  }

  /*
  if(!(shift.x == 0 && shift.y == 0 && shift.z == 0) && internal_update)
  {
    this->expand();
    edit_step_property->expand();
  }*/
}

void StepProperty::addStepProperties(const StepMsg & step)
{

  property_container_ = new rviz::Property("Step Properties", QVariant(), "Properties of this step", this);
  is_valid_ = new rviz::StatusProperty("Valid",
                                        step.valid ? "Step reached from predecessor" : "Step not reached from predecessor",
                                        step.valid ? rviz::StatusLevel::Ok : rviz::StatusLevel::Warn,
                                        property_container_);
  cost_ = new rviz::FloatProperty("Cost", step.cost, "Cost to make a step", property_container_);
  risk_ = new rviz::FloatProperty("Risk", step.risk, "Risk of step", property_container_);
  colliding_ = new rviz::StatusProperty("Colliding",
                                        step.colliding ? "Step is colliding" : "Step not colliding",
                                        step.colliding ? rviz::StatusLevel::Warn : rviz::StatusLevel::Ok,
                                        property_container_);
  if(!step.valid || step.colliding)
    this->setLevel(rviz::StatusLevel::Warn);
  else
    this->setLevel(rviz::StatusLevel::Ok);

  sway_duration_ = new rviz::FloatProperty("Sway duration", step.sway_duration, "Sway duration", property_container_);
  step_duration_ = new rviz::FloatProperty("Step duration", step.step_duration, "Step duration", property_container_);
  swing_height_ = new rviz::FloatProperty("Swing height", step.sway_duration, "Swing height", property_container_);

}

void StepProperty::updateStepProperty(const StepMsg & step)
{
  is_valid_->setLevel(step.valid ? rviz::StatusLevel::Ok : rviz::StatusLevel::Warn);
  is_valid_->setDescription(step.valid ? "Step reached from predecessor" : "Step not reached from predecessor");

  cost_->setFloat(step.cost);
  risk_->setFloat(step.risk);

  colliding_->setLevel(step.colliding ? rviz::StatusLevel::Warn : rviz::StatusLevel::Ok);
  colliding_->setDescription(step.colliding ? "Step is colliding" : "Step not colliding");

  if(!step.valid || step.colliding)
    this->setLevel(rviz::StatusLevel::Warn);
  else
    this->setLevel(rviz::StatusLevel::Ok);

  sway_duration_->setFloat(step.sway_duration);
  step_duration_->setFloat(step.step_duration);
  swing_height_->setFloat(step.swing_height);
}

void StepProperty::setValid(bool valid)
{
  is_valid_->setLevel(valid ? rviz::StatusLevel::Ok : rviz::StatusLevel::Warn);
  is_valid_->setDescription(valid ? "Step reached from predecessor" : "Step not reached from predecessor");
  if(!valid)
    this->setLevel(rviz::StatusLevel::Warn);
  else
    this->setLevel(rviz::StatusLevel::Ok);
}

void StepProperty::setExpanded(bool expanded)
{
  expanded ? this->expand() : this->collapse();
  selected = expanded;
//  edit_step_property->setHidden(!expanded);
}

void StepProperty::invokePoseUpdate()
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




}// End namespace vigir_footstep_planning_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning_rviz_plugin::StepProperty,rviz::StatusProperty )
