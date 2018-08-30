#ifndef PROPERTY_WIDGET_H
#define PROPERTY_WIDGET_H

#ifndef Q_MOC_RUN
#include <rviz/properties/status_property.h>
#include <vigir_footstep_planning_rviz_plugin/common/common.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#endif

namespace rviz
{
class VectorProperty;
class QuaternionProperty;
class FloatProperty;
}

namespace vigir_footstep_planning_rviz_plugin
{
class StepProperty : public rviz::StatusProperty
{
    Q_OBJECT
public:
    StepProperty(const vigir_footstep_planning_msgs::Step & step,
                   Property *  	parent = 0
                   /*const char *  	changed_slot = 0,
                   QObject *  	receiver = 0 */
                   );
    StepProperty();
    ~StepProperty();

    void addStep(const vigir_footstep_planning_msgs::Step& step);
    void updateStep(vigir_footstep_planning_msgs::Step updated_step);
    void setValid(bool valid);
    void setVisible(bool visible);

public Q_SLOTS:
    void updateStep(vigir_footstep_planning_msgs::EditStep edit_step);
    void setExpanded(bool expanded);
private Q_SLOTS:
    void onPoseEdited();
    void onShiftChanged();
    void updateStatus();

Q_SIGNALS:
    void stepPoseChanged(Ogre::Vector3 position, Ogre::Quaternion orientation);

private:
    void addStepProperties(const vigir_footstep_planning_msgs::Step& step);
    void updateStepProperty(const vigir_footstep_planning_msgs::Step& step);
    void updatePosition(const vigir_footstep_planning_msgs::Step& step);


    rviz::VectorProperty* position_;
    rviz::QuaternionProperty* orientation_;

    rviz::Property* edit_step_property;
    rviz::VectorProperty* original_position_;
    rviz::VectorProperty* position_shift_;

    rviz::Property* property_container_;
    rviz::StatusProperty* is_valid_;
    rviz::FloatProperty* cost_;
    rviz::FloatProperty* risk_;
    rviz::StatusProperty* colliding_;
    rviz::FloatProperty* sway_duration_;
    rviz::FloatProperty* step_duration_;
    rviz::FloatProperty* swing_height_;

    bool internal_update; //to not emit poseChanged when values are set
    bool selected; //is selected = in manipulation mode
};

} // end namespace vigir_footstep_planning_rviz_plugin
#endif // PROPERTY_WIDGET_H
