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

#ifndef FOOTSTEP_PLANNING_PANEL_H
#define FOOTSTEP_PLANNING_PANEL_H

#ifndef Q_MOC_RUN

#include <QWidget>
#include <vigir_footstep_planning_rviz_plugin/common/common.h>
#include "../../../../../build/vigir_footstep_planning_rviz_plugin/ui_paneldesign.h"


#endif

namespace rviz
{
class Config;
}


namespace vigir_footstep_planning_rviz_plugin
{
// Main window widget class for footstep planning, associated widget of StepPlanDisplay
//  - handles communication between request handlers and display
//  - handles communication inbetween request handlers
//  - display interactions forwarded to display (clearScene, displayRange)
class FootstepPlanningPanel: public QWidget, public Ui::PanelDesign
{
Q_OBJECT
public:
  FootstepPlanningPanel(QWidget* parent = 0 );
  ~FootstepPlanningPanel();
  void save( rviz::Config config ) const;
  void load( const rviz::Config& config );

  bool checkConnection();
  void setFrameID(const QString frame_id);
  void releasePlaceFeet();

public Q_SLOTS:
  void setStepPlan(vigir_footstep_planning_msgs::StepPlan step_plan);
  void updateGoalPose(vigir_footstep_planning_msgs::Feet goal_feet);
  void updateFromTo(vigir_footstep_planning_msgs::StepPlan step_plan);

  void initializeExecutionProgressbar(int max);
  void updateProgressBar(int val);
  void updateProgressBarExecutionState(bool success);

  // pass through to planning and pattern widget
  void startPoseRequested();
  void setFeedbackRequested(bool requested);
  void setLastStep(int index);
  void replanToIndex(int index);

private Q_SLOTS:
  void releaseSetGoalToggle(bool released);
  void on_displayStepsFromSpinBox_valueChanged(int arg1);
  void on_displayStepsToSpinBox_valueChanged(int arg1);
  void initializeGenerationProgressbar();
  void updateProgressBarGenerationState(bool success);

Q_SIGNALS:
  // Display options
  void displayRangeChanged(int from, int to);
  void clearAll();
  void clearStepPlan();
  void clearIM();
  void undo();
  void undoSequence();
  void abort();
  void interactionModeChanged(int interaction_mode);
  void executeRequested();
  // pass through of RequestHandlerBase signals (PlanningRequestHandler + PatternRequestHandler)
  void createdStepPlan(vigir_footstep_planning_msgs::StepPlan step_plan);
  void createdSequence(vigir_footstep_planning_msgs::StepPlan sequence);
  void startFeetAnswer(vigir_footstep_planning_msgs::Feet start);
  // PlanningRequestHandler signals
  void sendPlanningFeedback(vigir_footstep_planning_msgs::PlanningFeedback feedback);
  void feetToolActivated(PlaceFeetMode mode, bool active);
  void changed();

private:
  void setIcons();
  void makeConnections();
};

} // end namespace rviz_plugin_tutorials

#endif // FOOTSTEP_PLANNING_PANEL_H
