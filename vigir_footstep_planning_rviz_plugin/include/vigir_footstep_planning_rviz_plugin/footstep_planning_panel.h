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
  void feetToolActivated(PlantFeetMode mode, bool active);
  void changed();

private:
  void setIcons();
  void makeConnections();
};

} // end namespace rviz_plugin_tutorials

#endif // FOOTSTEP_PLANNING_PANEL_H
