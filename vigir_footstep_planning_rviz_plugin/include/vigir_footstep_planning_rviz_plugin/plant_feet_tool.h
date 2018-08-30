#ifndef PLANT_FEET_TOOL_H
#define PLANT_FEET_TOOL_H

#ifndef Q_MOC_RUN

#include <rviz/tool.h>
#include <QKeyEvent>
#include <actionlib/client/simple_action_client.h>
#include <vigir_footstep_planning_rviz_plugin/common/common.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>

#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ViewportMouseEvent;
class RenderPanel;
}

typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::GenerateFeetPoseAction> GenerateFeetPoseActionClient;
typedef vigir_footstep_planning_msgs::GenerateFeetPoseResultConstPtr GenerateFeetPoseResult;

namespace vigir_footstep_planning_rviz_plugin
{

class FeetVisual;

// - Tool handling first placing of a pair of feet (either goal (mode=GOAL_FEET) or start (mode=START_FEET)
// - Generating of valid feet message for given position & orientation
class PlantFeetTool: public rviz::Tool
{
  Q_OBJECT
public:// A tool to place a pair of feet representing the goal pose of a step plan
  PlantFeetTool();
  ~PlantFeetTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );
  virtual int processKeyEvent (QKeyEvent *event, rviz::RenderPanel *panel);

  void setMode(PlantFeetMode mode);

public Q_SLOTS:
  void setValidFeet(Ogre::Vector3 position, Ogre::Quaternion orientation, std::string frame_id, FeetType type);

Q_SIGNALS:
  void newStartPose(vigir_footstep_planning_msgs::Feet goal_feet);
  void newGoalPose(vigir_footstep_planning_msgs::Feet start_feet);

private:
  void setValidGoalCallback(const actionlib::SimpleClientGoalState& state, const GenerateFeetPoseResult& result);
  void setValidStartCallback(const actionlib::SimpleClientGoalState& state, const GenerateFeetPoseResult& result);
  void setRobotPose(const Ogre::Vector3& position, const Ogre::Quaternion& orientation, const std::string& frame_id);

  ros::NodeHandle nh;
  ros::Publisher robot_pose_publisher;
  GenerateFeetPoseActionClient generate_feet_ac;

  Ogre::SceneNode* moving_feet_node_;
  FeetVisual* moving_feet_;

  PlantFeetMode mode;
};

} // end namespace vigir_footstep_planning_rviz_plugin

#endif // PLANT_FEET_TOOL_H
