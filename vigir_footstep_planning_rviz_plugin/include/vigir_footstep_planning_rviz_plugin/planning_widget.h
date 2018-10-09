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


#ifndef PLANNINGWIDGET_H
#define PLANNINGWIDGET_H

#include <QWidget>
#include "../../../../../build/vigir_footstep_planning_rviz_plugin/ui_planning_widget.h"
#include <vigir_footstep_planning_rviz_plugin/planning_request_handler.h>

namespace rviz
{
class Config;
}

namespace vigir_footstep_planning_rviz_plugin
{
// Handels user interface for planning
class PlanningWidget : public QWidget
{
    Q_OBJECT
public:
    explicit PlanningWidget(QWidget *parent = 0);
    ~PlanningWidget();
    Ui::PlanningWidget *ui;
    PlanningRequestHandler* request_handler_;
    // save and load ui
    void save( rviz::Config config ) const;
    void load( const rviz::Config& config );
    // set request setting from display
    void setFrameID(const QString frame_id);
    void initialize();

private Q_SLOTS:
    void on_planningComputeCommandLinkButton_clicked();

public Q_SLOTS:
// invoked from Display
    void startPoseRequested();
    void updateGoalPose(vigir_footstep_planning_msgs::Feet goal_pose);
    void setFeedbackRequested(bool requested);
// invoked from Foot Visuals
    void setLastStep(int last_index);
    void replanToIndex(int index);
    // invoked from panel
    void abort();
    void setParameterSet(QString parameter_set_name);

Q_SIGNALS:
  void setGoalActivated(bool active);
  void changed();
};

} // end namespace vigir_footstep_planning_rviz_plugin
#endif // PLANNINGFORM_H
