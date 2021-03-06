//=================================================================================================
// Copyright (c) 2018, Stephanie Ferreira, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================


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
