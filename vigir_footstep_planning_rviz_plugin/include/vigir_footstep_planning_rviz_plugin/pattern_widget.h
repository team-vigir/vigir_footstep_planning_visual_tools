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


#ifndef PATTERNWIDGET_H
#define PATTERNWIDGET_H

#include "../../../../../build/vigir_footstep_planning_rviz_plugin/ui_pattern_planning.h"
#include <vigir_footstep_planning_rviz_plugin/pattern_request_handler.h>
#include <QWidget>

namespace rviz
{
class Config;
}

namespace vigir_footstep_planning_rviz_plugin
{
// Handels user interface for pattern planning
class PatternWidget : public QWidget
{
    Q_OBJECT
public:
    explicit PatternWidget(QWidget *parent = 0);
    ~PatternWidget();
    Ui::PatternWidget *ui;
    PatternRequestHandler* request_handler_;

    void initialize();

    // save and load ui
    void save( rviz::Config config ) const;
    void load( const rviz::Config& config );
    // set request setting from display
    void setFrameID(const QString frame_id);


public Q_SLOTS:
    // invoked from panel
    void abort();
    //invoked from display
    void startPoseRequested();
    //invoked from StepVisual
    void setLastStep(int last_index);
    void setParameterSet(QString parameter_set_name);

Q_SIGNALS:
    void changed();

private Q_SLOTS:
    void on_patternForwardPushButton_clicked();
    void on_patternBackwardPushButton_clicked();
    void on_patternRotateLeftPushButton_clicked();
    void on_patternRotateRightPushButton_clicked();
    void on_patternStrafeLeftPushButton_clicked();
    void on_patternStrafeRightPushButton_clicked();
    void on_patternStepUpPushButton_clicked();
    void on_patternStepOverPushButton_clicked();
    void on_patternStepDownPushButton_clicked();
    void on_patternWideStancePushButton_clicked();
    void on_patternCenterLeftPushButton_clicked();
    void on_patternCenterFeetPushButton_clicked();
    void on_patternCenterRightPushButton_clicked();
    void on_patternSamplingPushButton_clicked();
};

} // end namespace vigir_footstep_planning_rviz_plugin
#endif // PLANNINGFORM_H
