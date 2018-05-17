#ifndef STEP_PLAN_DISPLAY_H
#define STEP_PLAN_DISPLAY_H

#ifndef Q_MOC_RUN

#include <boost/circular_buffer.hpp>
#include <rviz/display.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <vigir_footstep_planning_rviz_plugin/plant_feet_tool.h>
#include <vigir_footstep_planning_rviz_plugin/foot_visual.h>
#include <memory>
#endif


typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::EditStepAction> EditStepActionClient;
typedef vigir_footstep_planning_msgs::Step StepMsg;


namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class StringProperty;
class BoolProperty;
class ToolManager;
class Tool;
class InteractionTool;
class InteractiveMarkerDisplay;
}

namespace interactive_markers
{
  class InteractiveMarkerServer;
}

namespace vigir_footstep_planning_rviz_plugin
{
class FootstepPlanningPanel;
class StepPlanHelper;
// for signal slot connection:
using rviz::Tool;

// StepPlanDisplay defines a plugin for rviz to display and interact with
// Step Plans. It inherits from rviz::Display and functions as a display
// for step plans and additionaly it starting a widget (FootstepPlanningPanel) for
// requesting step plans.
class StepPlanDisplay: public rviz::Display
{
Q_OBJECT
public:
  StepPlanDisplay();
  virtual ~StepPlanDisplay();

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

protected:
  virtual void onInitialize() override;
  virtual void fixedFrameChanged() override;
  virtual void update(float wall_dt, float ros_dt) override; // called periodically from visualization manager

public Q_SLOTS:
  void displayStepPlan(const vigir_footstep_planning_msgs::StepPlan& step_plan);
  void setInteractionMode(int interaction_mode);
private Q_SLOTS:
  void updateFrameID();
  void displayIndex();
  void activateFeetTool(bool active);
  void activateFeetTool(PlantFeetMode mode, bool active);
  void setStartVisible();
  void updateDisplay(int from, int to);
  virtual void reset();
  void visualizeFeedback(vigir_footstep_planning_msgs::PlanningFeedback feedback);
  void setDisplayFeedback();
  void updateACConnected();
  void checkTool(Tool* tool);
  void addStartFeet(vigir_footstep_planning_msgs::Feet start);
  void setStepValid(unsigned int index, bool valid);
  void visualizeCut(int new_end);
  void visualizeReplan(int end);
  void handleFeetPose(Ogre::Vector3 position, Ogre::Quaternion orientation, PlantFeetMode mode);
  void visualizeStepCost();


Q_SIGNALS:
  void feedbackRequestedChanged(bool requested);
  void requestStartPose();
  void stepIsValid(bool valid);

private:
  void initializeDisplayProperties();
  void makeFeetToolConnections();
  void makeConnections();
  void disconnectFeetTool();
  void makeStepVisualConnections(const StepVisual* visual);
  void makeStepPlanHelperConnections();
  void processMessage( const vigir_footstep_planning::msgs::StepPlan::ConstPtr& msg);
  void addStartFeet(const vigir_footstep_planning_msgs::Feet& start, const Ogre::Vector3& frame_position, const Ogre::Quaternion& frame_orientation);
  void displaySteps(const std::vector<vigir_footstep_planning_msgs::Step>& steps, const Ogre::Vector3& frame_position, const Ogre::Quaternion& frame_orientation);
  bool transformToFixedFrame(Ogre::Vector3& position, Ogre::Quaternion& orientation, const std_msgs::Header& header);
  boost::circular_buffer<boost::shared_ptr<StepVisual> > step_visuals_;
  boost::circular_buffer<boost::shared_ptr<StepVisual> > start_visuals_;

  vigir_footstep_planning_msgs::StepPlan current_step_plan;

  rviz::StringProperty* frame_id_property_;
  rviz::BoolProperty* display_index_;
  rviz::BoolProperty* display_feedback_;
  rviz::BoolProperty* display_start_;
  rviz::StatusProperty* ac_connected_;
  rviz::BoolProperty* visualize_valid_;
  rviz::BoolProperty* visualize_cost_;

  FootstepPlanningPanel* panel_;
  rviz::ToolManager* tool_manager_;
  PlantFeetTool* feet_tool_;
  int index_feet_tool;
  rviz::Tool* interact_tool_;
  rviz::InteractiveMarkerDisplay* interactive_marker_display_;
  // handles interactive markers created in StepVisual
  interactive_markers::InteractiveMarkerServer* interactive_marker_server_;
  bool displayFeedback;

  StepPlanHelper* step_plan_helper_;
  InteractionMode interaction_mode_;

  int last_step_index;
};

} // end namespace vigir_footstep_planning_rviz_plugin

#endif // STEP_PLAN_DISPLAY_H
