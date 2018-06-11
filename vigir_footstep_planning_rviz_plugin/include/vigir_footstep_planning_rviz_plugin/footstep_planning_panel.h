#ifndef FOOTSTEP_PLANNING_PANEL_H
#define FOOTSTEP_PLANNING_PANEL_H

#ifndef Q_MOC_RUN

#include <QWidget>
#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>
#include <vigir_footstep_planning_rviz_plugin/plant_feet_tool.h>
#include <OgreQuaternion.h>
#include <OgreVector3.h>
#include "../../../../../build/vigir_footstep_planning_rviz_plugin/ui_paneldesign.h"


#endif

namespace rviz
{
class Config;
}

namespace Ui
{
class PanelDesign;
}

namespace vigir_footstep_planning_rviz_plugin
{
// Main window widget class for footstep planning, associated widget of StepPlanDisplay
//  - handles communication between request handlers and display
//  - handles communication inbetween request handlers
//  - display interactions forwarded to display (clearScene, displayRange)
class FootstepPlanningPanel: public QWidget
{
Q_OBJECT
public:
  FootstepPlanningPanel(QWidget* parent = 0 );
  ~FootstepPlanningPanel();
  void save( rviz::Config config ) const;
  void load( const rviz::Config& config );

  bool checkConnection();
  void setFrameID(const QString frame_id);
  void handleGoalPose(Ogre::Vector3 position, Ogre::Quaternion orientation);
  void releasePlaceFeet();
  Ui::PanelDesign* ui_;

public Q_SLOTS:
  void setStepPlan(vigir_footstep_planning_msgs::StepPlan step_plan);
private:
  void makePatternConnections();
  void makePlanningConnections();
private Q_SLOTS:
  void updateFromTo(vigir_footstep_planning_msgs::StepPlan step_plan);
  void releaseSetGoalToggle(bool released);
  void on_displayStepsFromSpinBox_valueChanged(int arg1);
  void on_displayStepsToSpinBox_valueChanged(int arg1);
  void on_reset_pushButton_clicked();
  void on_clearIMPushButton_clicked();
// pass through to planning and pattern widget
  void setFeedbackRequested(bool requested);
  void startPoseRequested();
  void setLastStep(int index);
  void replanToIndex(int index);
// emit signals:
  void emitCreatedStepPlan(vigir_footstep_planning_msgs::StepPlan step_plan);
  void emitStartFeetAnswer(vigir_footstep_planning_msgs::Feet start);
  void emitGoalFeetAnswer(vigir_footstep_planning_msgs::Feet goal);
  void emitSendPlanningFeedback(vigir_footstep_planning_msgs::PlanningFeedback feedback);
  void emitFeetToolActivated(bool active);
  void emitPlaceLeftActivated(bool active);
  void emitPlaceRightActivated(bool active);
  void emitPlaceBothActivated(bool active);
  void emitInteractionModeChanged(int interaction_mode);

Q_SIGNALS:
  // Display options
  void displayRangeChanged(int from, int to);
  void clearScene();
  void resetValues();
  void clearIM();
  void undo();
  void interactionModeChanged(int interaction_mode);
  void executeRequested();
  // pass through of RequestHandlerBase signals (PlanningRequestHandler + PatternRequestHandler)
  void createdStepPlan(vigir_footstep_planning_msgs::StepPlan step_plan);
  void startFeetAnswer(vigir_footstep_planning_msgs::Feet start);
  // PlanningRequestHandler signals
  void goalFeetAnswer(vigir_footstep_planning_msgs::Feet goal);
  void sendPlanningFeedback(vigir_footstep_planning_msgs::PlanningFeedback feedback);
  void feetToolActivated(bool active);
  void feetToolActivated(PlantFeetMode mode, bool active);
};

} // end namespace rviz_plugin_tutorials

#endif // FOOTSTEP_PLANNING_PANEL_H
