#ifndef FEET_VISUAL_H
#define FEET_VISUAL_H

#ifndef Q_MOC_RUN

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>
#include <QObject>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <interactive_markers/menu_handler.h>
#include <boost/circular_buffer.hpp>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#endif

typedef visualization_msgs::InteractiveMarkerFeedbackConstPtr FeedbackConstPtr;
typedef vigir_footstep_planning_msgs::Foot FootMsg;

typedef visualization_msgs::InteractiveMarkerControl InteractiveMarkerControlMsg;
typedef visualization_msgs::InteractiveMarker InteractiveMarkerMsg;
typedef visualization_msgs::Marker MarkerMsg;

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

enum FeetType{GOAL = 0, START = 1};
enum FootIndex{LEFT = 0, RIGHT = 1};

// Represents the visual object of a set of feet (either start or goal position
// - creates a visual object at given position
// - creates interactive marker around step and handles communication for this step
class FeetVisual: public QObject
{

Q_OBJECT
public:
  FeetVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, FeetType type);
  virtual ~FeetVisual();

  // create visual functions
//  void createByMessage(const vigir_footstep_planning_msgs::Feet msg);
  void createFeetAt(const Ogre::Vector3& position, const Ogre::Quaternion& orientation);
  void createByMessage(const vigir_footstep_planning_msgs::Feet msg);

  void setFramePosition( const Ogre::Vector3& position );
  void setFrameOrientation( const Ogre::Quaternion& orientation );

  void setVisible(bool visible);

  void initializeInteractiveMarker(interactive_markers::InteractiveMarkerServer* marker_server);




private:
  boost::circular_buffer<boost::shared_ptr<StepVisual> > feet_visuals_;

  FeetType feet_type;

  Ogre::SceneNode* frame_node_;
  Ogre::SceneManager* scene_manager_;

  void setFeetPositioning();
  void computePositioning(Ogre::Vector3& position, const Ogre::Quaternion& orientation);

  Ogre::Vector3 pos_left;
  Ogre::Vector3 pos_right;

  vigir_footstep_planning_msgs::Feet current_feet;

  Ogre::Vector3 position_; //relative to frame_id of step or fixed frame if no frame id
  Ogre::Quaternion orientation_;

  void normalizeQuaternion(geometry_msgs::Quaternion& orientation);

  /*
  // interaction:
  bool interaction;
  interactive_markers::InteractiveMarkerServer* interactive_marker_server_;
  interactive_markers::MenuHandler menu_handler;

  void setButtonInteractiveMarker();
  void processButtonFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void resetInteractiveMarkers();

  visualization_msgs::InteractiveMarker makeInteractiveMarker(const std::string& type_name);
  visualization_msgs::Marker makeMarker(bool transparent);

  void normalizeQuaternion(geometry_msgs::Quaternion& orientation);
  geometry_msgs::Pose toPoseMessage();
*/
};

} // end namespace vigir_footstep_planning_rviz_plugin

#endif // FOOT_VISUAL_H
