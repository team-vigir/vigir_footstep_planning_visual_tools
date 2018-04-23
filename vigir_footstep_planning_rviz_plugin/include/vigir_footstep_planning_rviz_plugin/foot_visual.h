#ifndef STEP_VISUAL_H
#define STEP_VISUAL_H

#ifndef Q_MOC_RUN

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>
#include <rviz/ogre_helpers/stl_loader.h>
#include <QObject>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <interactive_markers/menu_handler.h>

#endif

typedef visualization_msgs::InteractiveMarkerFeedbackConstPtr FeedbackConstPtr;
typedef vigir_footstep_planning_msgs::Foot FootMsg;

typedef visualization_msgs::InteractiveMarkerControl InteractiveMarkerControlMsg;
typedef visualization_msgs::InteractiveMarker InteractiveMarkerMsg;
typedef visualization_msgs::Marker MarkerMsg;

namespace Ogre
{
class Vector3;
class Quaternion;
}

namespace rviz
{
class Shape;
class MeshShape;
class ViewportMouseEvent;
class MovableText;
}

namespace interactive_markers
{
  class InteractiveMarkerServer;
}


namespace vigir_footstep_planning_rviz_plugin
{
// interaction mode which is toggled when interaction is activated.
enum InteractionMode{PLANE = 1, SIXDOF = 2,  FULLSIXDOF=3};
// Represents the visual object of a step
// - creates a visual object at given position
// - creates interactive marker around step and handles communication for this step
class StepVisual: public QObject
{
Q_OBJECT
public:
  // default index=-1 for goal and start visuals, which are not part of step_plan.steps
  StepVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, unsigned int which_foot, std::string frame_id = "", int index=-1);
  virtual ~StepVisual();

  // create visual functions
  void createByMessage(const vigir_footstep_planning_msgs::Step msg);
  void createVisualAt(const Ogre::Vector3& position, const Ogre::Quaternion& orientation);
  void createVisualAt(const geometry_msgs::Point& position, const geometry_msgs::Quaternion& orientation);

  void initializeInteractiveMarker(interactive_markers::InteractiveMarkerServer* marker_server);

  void setFramePosition( const Ogre::Vector3& position );
  void setFrameOrientation( const Ogre::Quaternion& orientation );
  void setPosition(const Ogre::Vector3& position );

  void setColor( float r, float g, float b, float a );
  void setVisible(bool visible);
  void displayIndex(bool visible);
  void visualizeCost(float max);


Q_SIGNALS:
  void stepEdited(vigir_footstep_planning_msgs::EditStep edit);
  void cutStepPlanHere(int index);
  void replanToHere(int index);

public Q_SLOTS:
  void setValid(bool valid);
  void disableInteractiveMarker();
  void setButtonInteractiveMarker();
  void setInteractionMode(InteractionMode interaction_mode);

private:
  void processInteractionFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void processButtonFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void processMenuFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void processSetInteractionMode(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void footToValid();
  void setPlaneInteractiveMarker();
  void setSixDOFInteractiveMarker();
  void setFullSixDOFInteractiveMarker();
  visualization_msgs::Marker makeMarker(bool transparent);
  visualization_msgs::InteractiveMarker makeInteractiveMarker(const std::string& type_name);
  void addSixDOFControl(visualization_msgs::InteractiveMarker &im);
  void addPlaneControl(visualization_msgs::InteractiveMarker &im, bool with_rotation);
  geometry_msgs::Pose toPoseMessage();

  void removeStep();
  void eraseInteractiveMarkers();

  void setCheckState(int checked_index); //0 for None
  boost::shared_ptr<rviz::MeshShape> foot_;

  Ogre::SceneNode* frame_node_;
  std::string frame_id_;
  Ogre::SceneManager* scene_manager_;

  ogre_tools::STLLoader* stl_loader_;

  InteractionMode interaction_mode_;
  interactive_markers::InteractiveMarkerServer* interactive_marker_server_;
  interactive_markers::MenuHandler menu_handler;
  interactive_markers::MenuHandler::EntryHandle mode_begin; //= InteractionMode::NONE

  rviz::MovableText* text_;
  Ogre::SceneNode* text_node_;
  bool text_visible;
  bool interaction;
  void createFootMesh(const std::string& path);
  void createIndexText();
  unsigned int foot_index; //Either left or right

  int index; //index in Step Plan
  float cost;
  float risk;
  vigir_footstep_planning_msgs::Step current_step;

};

} // end namespace vigir_footstep_planning_rviz_plugin

#endif // FOOT_VISUAL_H
