#ifndef FEET_VISUAL_H
#define FEET_VISUAL_H

#ifndef Q_MOC_RUN
#include <QObject>
#include <vigir_footstep_planning_rviz_plugin/common/common.h>
#include <interactive_markers/menu_handler.h>
#include <boost/circular_buffer.hpp>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#endif


namespace Ogre
{
class SceneManager;
class SceneNode;
}

namespace interactive_markers
{
  class InteractiveMarkerServer;
}


namespace interactive_markers
{
  class InteractiveMarkerServer;
}

namespace vigir_footstep_planning_rviz_plugin
{
class StepVisual;

// Represents the visual object of a set of feet (either start or goal position
// - creates a visual object at given position
// - creates interactive marker around step and handles communication for this step
class FeetVisual: public QObject
{

Q_OBJECT
public:
  FeetVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, FeetType type);
  virtual ~FeetVisual();

  void setFeetPositioning();

  // create visual functions
//  void createByMessage(const vigir_footstep_planning_msgs::Feet msg);
  void createFeetAt(const Ogre::Vector3& position, const Ogre::Quaternion& orientation);
  void createByMessage(const vigir_footstep_planning_msgs::Feet msg);

  void setFramePosition( const Ogre::Vector3& position );
  void setFrameOrientation( const Ogre::Quaternion& orientation );

  void setVisible(bool visible);
  void setRobotPose(const Ogre::Vector3& position, const Ogre::Quaternion& orientation); // set position of visuals in robot frame (during interaction)
  void setPlannerPose(const vigir_footstep_planning_msgs::Feet& msg); // set position_ & orientation_ in Planner Frame (for interaction)
  void updateFeetMsg(const vigir_footstep_planning_msgs::Feet msg, Ogre::Vector3 frame_position, Ogre::Quaternion frame_orientation);

  void initializeInteractiveMarker(interactive_markers::InteractiveMarkerServer* marker_server);

public Q_SLOTS:
  void setButtonInteractiveMarker();

Q_SIGNALS:
  void feetPoseChanged(Ogre::Vector3 position, Ogre::Quaternion orientation);

private:
  void computePositioning(Ogre::Vector3& position, const Ogre::Quaternion& orientation);

  void setSixDOFInteractiveMarker();
  void setPlaneInteractiveMarker();

  void processButtonFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void processMenuFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void processInteractionFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void deleteInteractiveMarker();
  void resetInteractiveMarker();
  visualization_msgs::Marker makeMarker();
  visualization_msgs::InteractiveMarker getInteractiveMarker();


  // current feet properties
  FeetType feet_type; //START / GOAL
  Ogre::Vector3 position_; // current position of centrer of feet
  Ogre::Quaternion orientation_; // current orientation of feet
  vigir_footstep_planning_msgs::Feet feet_msg;
  std::string frame_id;

  // Visuals
  boost::circular_buffer<boost::shared_ptr<StepVisual> > feet_visuals_;
  Ogre::SceneNode* frame_node_;
  Ogre::SceneManager* scene_manager_;
  Ogre::SceneNode* show_dir_node_;

  // helper values for positioning the feet relative to given point
  Ogre::Vector3 pos_left; // position of left foot with center of feet as origin
  Ogre::Vector3 pos_right; // position of right foot with center of feet as origin
  //size of interactive marker handles
  float im_scale;
  float seperation;

  interactive_markers::InteractiveMarkerServer* interactive_marker_server_;
  interactive_markers::MenuHandler menu_handler;

  bool interaction_enabled; //false=button, true=interaction_mode
  InteractionMode interaction_mode; // interaction which is toggled after button click
};

} // end namespace vigir_footstep_planning_rviz_plugin

#endif // FOOT_VISUAL_H
