#ifndef STEP_PLAN_DISPLAY_H
#define STEP_PLAN_DISPLAY_H

#ifndef Q_MOC_RUN
#include <rviz/display.h>
#include <vigir_footstep_planning_rviz_plugin/common/common.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <actionlib/client/simple_action_client.h>

#include <boost/circular_buffer.hpp>
#include <memory>
#endif

typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::EditStepAction> EditStepActionClient;

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
class Property;
class VectorProperty;
class QuaternionProperty;
}

namespace interactive_markers
{
  class InteractiveMarkerServer;
}

namespace vigir_footstep_planning_rviz_plugin
{
class FootstepPlanningPanel;
class StepPlanHelper;
class FeetVisual;
class PlantFeetTool;
class StepVisual;

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
  virtual void update(float wall_dt, float ros_dt) override; // called periodically from visualization manager

public Q_SLOTS:
  void displayStepPlan(const vigir_footstep_planning_msgs::StepPlan& step_plan);
  void setInteractionMode(int interaction_mode);
  void setGoalFeetPlannerFrameProperty(Ogre::Vector3 position, Ogre::Quaternion orientation);
  void setStartFeetPlannerFrameProperty(Ogre::Vector3 position, Ogre::Quaternion orientation);

private Q_SLOTS:
  void updateFrameID();
  void displayIndex();
  void activateFeetTool(PlantFeetMode mode, bool active);
  void setStartVisible();
  void updateDisplay(int from, int to);
  void resetStepPlan();
  void visualizeFeedback(vigir_footstep_planning_msgs::PlanningFeedback feedback);
  void setDisplayFeedback();
  void updateACConnected();
  void checkTool(rviz::Tool* tool);
  void addStartFeetMsg(vigir_footstep_planning_msgs::Feet start);
  void addGoalFeet(vigir_footstep_planning_msgs::Feet goal);
  void setStepValid(unsigned int index, bool valid);
  void visualizeCut(int new_end);
  void visualizeReplan(int end);
  void visualizeStepCost();
  void setUpdateStepPlan();
  void updateStepVisuals(vigir_footstep_planning_msgs::StepPlan updated_step_plan);
  void addGoalFeetRobotFrameProperties(vigir_footstep_planning_msgs::Feet goal_feet);
  void addStartFeetRobotFrameProperties(vigir_footstep_planning_msgs::Feet goal_feet);
  void startPoseUpdated();
  void goalPoseUpdated();
  void setGoalVisible();
  void updateGoalPose(Ogre::Vector3 position, Ogre::Quaternion orientation);
  void updateStartPose(Ogre::Vector3 position, Ogre::Quaternion orientation);
  void resetGoal();
  void updateVisualizeValid();

Q_SIGNALS:
  void feedbackRequestedChanged(bool requested);
  void requestStartPose();
  void stepIsValid(bool valid);

private:
  void initializeDisplayProperties();
  void makeFeetToolConnections();
  void makePanelConnections();
  void makeStepVisualConnections(const StepVisual* visual);
  void makeStepPlanHelperConnections();
  void processMessage( const vigir_footstep_planning::msgs::StepPlan::ConstPtr& msg);
  void addStartFeet(const vigir_footstep_planning_msgs::Feet& start, const Ogre::Vector3& frame_position, const Ogre::Quaternion& frame_orientation);
  void displaySteps(const std::vector<vigir_footstep_planning_msgs::Step>& steps, const Ogre::Vector3& frame_position, const Ogre::Quaternion& frame_orientation);
  bool transformToFixedFrame(Ogre::Vector3& position, Ogre::Quaternion& orientation, const std_msgs::Header& header);

  void addFootProperty(vigir_footstep_planning_msgs::Foot foot, rviz::Property* parent);
  void updateFootProperty(rviz::Property* parent, vigir_footstep_planning_msgs::Foot updated_foot);

  boost::circular_buffer<boost::shared_ptr<StepVisual> > step_visuals_; // Steps of current step plan
  boost::circular_buffer<boost::shared_ptr<StepVisual> > feedback_visuals_;

  FeetVisual* start_feet_;
  FeetVisual* goal_feet_;

  vigir_footstep_planning_msgs::StepPlan current_step_plan;

  rviz::StatusProperty* ac_connected_;
  rviz::StringProperty* frame_id_property_;

  rviz::BoolProperty* display_index_;
  rviz::BoolProperty* display_feedback_;
  rviz::BoolProperty* visualize_valid_;
  rviz::BoolProperty* visualize_cost_;

  rviz::BoolProperty* update_step_plan_positions_;
  rviz::Property* step_properties_container_;

  rviz::BoolProperty* goal_property_container_;
  rviz::BoolProperty* start_property_container_;

  std::vector<rviz::Property*> goal_properties;
  std::vector<rviz::Property*> start_properties;
  rviz::VectorProperty* goal_position_property_;
  rviz::QuaternionProperty* goal_orientation_property_;
  rviz::VectorProperty* start_position_property_;
  rviz::QuaternionProperty* start_orientation_property_;

  FootstepPlanningPanel* panel_;
  StepPlanHelper* step_plan_helper_;
  rviz::ToolManager* tool_manager_;
  PlantFeetTool* feet_tool_;
  rviz::Tool* interact_tool_;

  InteractionMode interaction_mode_;
  interactive_markers::InteractiveMarkerServer* im_server_steps;
  interactive_markers::InteractiveMarkerServer* im_server_feet;

  bool displayFeedback;
  int last_step_index;

  // Parameters:
  ros::NodeHandle nh;
};

} // end namespace vigir_footstep_planning_rviz_plugin

#endif // STEP_PLAN_DISPLAY_H
