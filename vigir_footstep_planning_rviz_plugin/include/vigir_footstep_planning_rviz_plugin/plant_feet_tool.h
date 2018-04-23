#ifndef PLANT_FEET_TOOL_H
#define PLANT_FEET_TOOL_H

#ifndef Q_MOC_RUN

#include <rviz/tool.h>
#include <QKeyEvent>
#include <QObject>
#include <boost/circular_buffer.hpp>

#include <vigir_footstep_planning_rviz_plugin/foot_visual.h>
#include <interactive_markers/menu_handler.h>

#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class VectorProperty;
class ViewportMouseEvent;
class RenderPanel;
class Arrow;
}

namespace interactive_markers
{
  class InteractiveMarkerServer;
}

typedef visualization_msgs::InteractiveMarker InteractiveMarkerMsg;
typedef visualization_msgs::InteractiveMarkerControl InteractiveMarkerControlMsg;
typedef visualization_msgs::Marker MarkerMsg;
typedef visualization_msgs::InteractiveMarkerFeedback InteractiveMarkerFeedbackMsg;

namespace vigir_footstep_planning_rviz_plugin
{
enum PlantFeetMode{BOTH = 0, LEFT = 1, RIGHT = 2};

// A tool to place a pair of feet representing the goal of a step plan
class PlantFeetTool: public rviz::Tool
{
Q_OBJECT
public:
  PlantFeetTool();
  ~PlantFeetTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );
  virtual int processKeyEvent (QKeyEvent *event, rviz::RenderPanel *panel);

  void setMode(PlantFeetMode mode);
  void setInteractiveMarkerServer(interactive_markers::InteractiveMarkerServer* server);

public Q_SLOTS:
  // Tool gets the correct goal feet from panel
  void addGoalFeet(vigir_footstep_planning_msgs::Feet goal);
  void reset();

private:
  Ogre::SceneNode* moving_feet_node_;
  StepVisual* moving_left_;
  StepVisual* moving_right_;
  rviz::Arrow* show_dir_;
  bool display_dir_;
  void updateMovingFeetVisibility();

  interactive_markers::InteractiveMarkerServer* interactive_marker_server_;
  interactive_markers::MenuHandler menu_handler;
  void processButtonFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void processMenuFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void processInteractionFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void enableSixDOFInteraction();
  void enablePlaneInteraction();
  void enableButtonInteraction();
  visualization_msgs::Marker makeMarker();
  visualization_msgs::InteractiveMarker makeInteractiveMarker();
  void addPlaneControl(visualization_msgs::InteractiveMarker& im);
  void addSixDOFControl(visualization_msgs::InteractiveMarker& im);
  void addMoveControl(visualization_msgs::InteractiveMarker& im);
  rviz::VectorProperty* current_feet_property_;
  PlantFeetMode mode;
  boost::circular_buffer<boost::shared_ptr<StepVisual> > goal_visuals_;
  void addGoalFeet(vigir_footstep_planning_msgs::Feet goal, Ogre::Vector3 frame_position, Ogre::Quaternion frame_orientation);
  bool interaction3D;
Q_SIGNALS:
  void feetDropped(Ogre::Vector3 position, Ogre::Quaternion orientation, PlantFeetMode mode);
  void activateTool(bool activate);
  void newStartPose(Ogre::Vector3 position, Ogre::Quaternion orientation);
//  void newStartFeet(Ogre::Vector3 position_left, Ogre::Vector3 position_right, Ogre::Quaternion orientation);
};

} // end namespace rviz_plugin_tutorials

#endif // PLANT_FEET_TOOL_H
