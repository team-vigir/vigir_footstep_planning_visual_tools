#ifndef STEP_VISUAL_H
#define STEP_VISUAL_H

#ifndef Q_MOC_RUN
#include <QObject>
#include <vigir_footstep_planning_rviz_plugin/common/common.h>
#include <interactive_markers/menu_handler.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#endif


namespace Ogre
{
class SceneManager;
class SceneNode;
}

namespace ogre_tools
{
  class STLLoader;
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
// Represents the visual object of a step
// - creates a visual object at given position
// - creates interactive marker around step and handles communication for this step
class StepVisual: public QObject
{
Q_OBJECT
public:
  // default index=-1 for goal and start visuals, which are not part of step_plan.steps
  StepVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, unsigned int which_foot, std::string frame_id="", int index=-1);
  virtual ~StepVisual();
  // Initializing -----------------------------------
  void setFootProperties();

  // create visual functions
  void createByMessage(const vigir_footstep_planning_msgs::Step& msg);
  void createByFootMsg(const vigir_footstep_planning_msgs::Foot& msg);
  void createVisualAt(const Ogre::Vector3& position, const Ogre::Quaternion& orientation);

  void initializeInteractiveMarker(interactive_markers::InteractiveMarkerServer* marker_server);

  void setFramePosition( const Ogre::Vector3& position );
  void setFrameOrientation( const Ogre::Quaternion& orientation );
  // --------------------------------------------------

  // Updating -----------------------------------------
  void setPosition(const Ogre::Vector3& position );
  void setOrientation(const Ogre::Quaternion& orientation);
 // void setPose(const geometry_msgs::Pose& pose);


  void setColor( float r, float g, float b, float a );
  void setVisible(bool visible);
  void displayIndex(bool visible);
  void visualizeCost(float max);

  void updateStepMsg(const vigir_footstep_planning_msgs::Step updated_msg);
  void updateFootMsg(const vigir_footstep_planning_msgs::Foot updated_msg);
  // ---------------------------------------------------
  Ogre::Vector3 getPosition();
  Ogre::Quaternion getOrientation();

public Q_SLOTS:
  void visualizeValid(bool valid); // update validity while step is being edited but not yet released, invoked by step plan helper
  void setInteractionMode(InteractionMode interaction_mode); // change default interaction mode invoked by panel
  void editedPose(Ogre::Vector3 new_position, Ogre::Quaternion new_orientation); // invoked by editing of Step Property
  void setButtonInteractiveMarker();
  void setVisualizeValid(bool visualize);

Q_SIGNALS:
  void stepChanged(vigir_footstep_planning_msgs::EditStep edit);
  void cutStepPlanHere(int index);
  void replanToHere(int index);
  void endStepPlanHere(int index);
  void updateStepsPos();
  void footDropped();
  void selected(bool selected);

private:
  void createFootMesh();
  void createIndexText();

  void processInteractionFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void processButtonFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void processMenuFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void processSetInteractionMode(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void setPlaneInteractiveMarker();
  void setSixDOFInteractiveMarker();
  void setFullSixDOFInteractiveMarker();

  void deleteInteractiveMarker();
  void setCheckState(int checked_index); //0 for None
  void resetInteractiveMarker();

  visualization_msgs::Marker makeMarker();

  boost::shared_ptr<rviz::MeshShape> foot_;
  rviz::MovableText* text_;

  Ogre::SceneManager* scene_manager_;
  Ogre::SceneNode* frame_node_;
  Ogre::SceneNode* foot_node_;
  Ogre::SceneNode* text_node_;

  interactive_markers::InteractiveMarkerServer* interactive_marker_server_;
  interactive_markers::MenuHandler menu_handler;
  interactive_markers::MenuHandler::EntryHandle mode_begin; //= InteractionMode::NONE

  InteractionMode interaction_mode_;

  bool visible_;
  bool snap_to_valid;
  bool display_index;
  bool visualize_valid;
  std::string frame_id_;

  // Step Properties:
  unsigned int foot_index; //Either left or right
  int index; //index in Step Plan
  float cost;
  float risk;
  vigir_footstep_planning_msgs::Step current_step;

  Ogre::Vector3 scale_; //(0.001,0.001,0.001);
  Ogre::Vector3 mesh_origin_; //(0,0,0.0275); mesh origin
  Ogre::Vector3 text_position_;
  float im_scale;
};

} // end namespace vigir_footstep_planning_rviz_plugin

#endif // STEP_VISUAL_H
