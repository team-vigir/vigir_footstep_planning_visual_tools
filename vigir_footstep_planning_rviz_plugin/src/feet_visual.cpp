// ------------------------------
// ACHTUNG!!!!
// Ogre::Quaternion Konstruktor: Quaternion(W, x, y, z)
//
#include <algorithm>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <vigir_footstep_planning_rviz_plugin/feet_visual.h>
#include <vigir_footstep_planning_rviz_plugin/step_visual.h>
#include <interactive_markers/interactive_marker_server.h>

using namespace interactive_markers;

namespace vigir_footstep_planning_rviz_plugin
{

FeetVisual::FeetVisual( Ogre::SceneManager* scene_manager,
                        Ogre::SceneNode* parent_node,
                        FeetType type):
  scene_manager_(scene_manager)
  , feet_type(type)
{

  frame_node_ = parent_node->createChildSceneNode();
  feet_visuals_.resize(2);
  feet_visuals_[LEFT].reset(new StepVisual(scene_manager, frame_node_, FootMsg::LEFT));
  feet_visuals_[RIGHT].reset(new StepVisual(scene_manager, frame_node_, FootMsg::RIGHT));

  setFeetPositioning();
}

FeetVisual::~FeetVisual()
{
}


void FeetVisual::setFeetPositioning()
{
  float seperation;
  float frame_x;
  float frame_y;
  float frame_z;
  ros::NodeHandle nh;
  if (nh.getParam("foot/separation", seperation)
      && nh.getParam("foot/left/foot_frame/x", frame_x)
      && nh.getParam("foot/left/foot_frame/y", frame_y)
      && nh.getParam("foot/left/foot_frame/z", frame_z)
      )
  {
    pos_left.x = frame_x;
    pos_left.y = seperation/2 - frame_y;
    pos_left.z = frame_z;
    if(   nh.getParam("foot/right/foot_frame/x", frame_x)
       && nh.getParam("foot/right/foot_frame/y", frame_y)
       && nh.getParam("foot/right/foot_frame/z", frame_z)
       )
    {
      pos_right.x = frame_x;
      pos_right.y = -seperation/2 - frame_y;
      pos_right.z = frame_z;
    }
  }
  else
    ROS_INFO("Could not retrieve feet positioning from /johnny5/footstep_planning/foot/");
}


// CREATE VISUAL:
void FeetVisual::createFeetAt(const Ogre::Vector3& position, const Ogre::Quaternion& orientation)
{
  position_ = Ogre::Vector3(position.x, position.y, position.z);
  orientation_ = Ogre::Quaternion(orientation.w, orientation.x, orientation.y, orientation.z);

  Ogre::Vector3 current_pos_left = pos_left;
  Ogre::Vector3 current_pos_right = pos_right;
  computePositioning(current_pos_left, orientation);
  computePositioning(current_pos_right, orientation);

  feet_visuals_[LEFT]->createVisualAt(position + current_pos_left, orientation);
  feet_visuals_[RIGHT]->createVisualAt(position + current_pos_right, orientation);

  feet_visuals_[LEFT]->setColor(1.0f,0.0f,0.0f,1.0f);
  feet_visuals_[RIGHT]->setColor(0.0f,1.0f,0.0f,1.0f);

}

void FeetVisual::createByMessage(const vigir_footstep_planning_msgs::Feet msg)
{
  feet_visuals_[RIGHT]->createVisualAt( msg.right.pose.position , msg.right.pose.orientation);
  feet_visuals_[LEFT]->createVisualAt( msg.left.pose.position, msg.left.pose.orientation );
}

// Position and orientation are passed through to the SceneNode.
void FeetVisual::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
}

void FeetVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
}

void FeetVisual::setVisible(bool visible)
{
  frame_node_->setVisible(visible);
}

/*
void FeetVisual::updatePostion(const Ogre::Vector3& position)
{
  visuals_[LEFT]
}*/


//  - helper functions: compute offset compared to center point of the feet with given orientation
void FeetVisual::computePositioning(Ogre::Vector3& position, const Ogre::Quaternion& orientation)
{
  float w = orientation.w, x =orientation.x, y=orientation.y, z=orientation.z;

  float x_rot = position.x*(w*w+x*x-y*y-z*z) + position.y*(2*x*y - 2*w*z) + position.z*(2*x*z +2*w*y);
  float y_rot = position.x*(2*x*y + 2*w*z) + position.y*(w*w-x*x+y*y-z*z) + position.z*(2*y*z - 2*w*x);
  float z_rot = position.x*(2*x*z - 2*w*y) + position.y*(2*y*z + 2*w*x) + position.z*(w*w-x*x-y*y+z*z);
  position.x = x_rot;
  position.y = y_rot;
  position.z = z_rot;
}

/*

void StepVisual::normalizeQuaternion(geometry_msgs::Quaternion& orientation)
{
  Ogre::Quaternion o(orientation.w, orientation.x, orientation.y, orientation.z);
  o.normalise();
  orientation.w = o.w;
  orientation.x = o.x;
  orientation.y = o.y;
  orientation.z = o.z;
}


Ogre::Vector3 StepVisual::getPosition()
{
  return position_;
}

Ogre::Quaternion StepVisual::getOrientation()
{
  return orientation_;
}
*/




// Interactive Markers -----------------------------------------------------------------

//void FeetVisual::initializeInteractiveMarker(InteractiveMarkerServer* marker_server)
//{
//  interactive_marker_server_ = marker_server;

//  // case 1:
//  menu_handler.setCheckState(menu_handler.insert("enable 6DOF interaction", 0 /*boost::bind(&FeetVisual::processMenuFeedback, this, _1)*/), MenuHandler::UNCHECKED);
//  // case 2:
//  menu_handler.insert("Set as start", 0/*boost::bind(&FeetVisual::processMenuFeedback, this, _1)*/);

//  setButtonInteractiveMarker();
//  interaction = true;
//}

// set interactive marker
//void FeetVisual::setButtonInteractiveMarker()
//{
//  // remove other interactive marker
//  resetInteractiveMarkers();
//  visualization_msgs::InteractiveMarker im = makeInteractiveMarker("button");
//  visualization_msgs::Marker box_marker = makeMarker(false/*true*/);

//  visualization_msgs::InteractiveMarkerControl control;
//  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
//  control.markers.push_back( box_marker);
//  control.always_visible = false;
//  im.controls.push_back(control);

//  interactive_marker_server_->insert(im, boost::bind(&FeetVisual::processButtonFeedback, this, _1));
//  interactive_marker_server_->applyChanges();
//}
/*
//void FeetVisual::setFullSixDOFInteractiveMarker()
//{
//  ROS_INFO("Hold shift to move in/out the camera plane");
//  eraseInteractiveMarkers();
//  InteractiveMarkerMsg im = makeInteractiveMarker("6DOF_withMove3D");

//  addSixDOFControl(im);

//  // Move 3D with box marker
//  InteractiveMarkerControlMsg control;
//  control.name = "move_3d";
//  MarkerMsg marker = makeMarker(transparent false);
//  control.markers.push_back(marker);
//  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
//  control.always_visible = false;
//  im.controls.push_back(control);

//  interactive_marker_server_->insert(im, boost::bind(&StepVisual::processInteractionFeedback, this, _1));
//  setCheckState(FULLSIXDOF);
//  menu_handler.apply( *interactive_marker_server_, im.name );
//  interactive_marker_server_->applyChanges();
//}
//void FeetVisual::setSixDOFInteractiveMarker()
//{
//  // Move on 2d plane with box marker
//  // Move 6DOF with default controls
//  eraseInteractiveMarkers();
//  InteractiveMarkerMsg im = makeInteractiveMarker("6DOF");

//  addSixDOFControl(im);

//  addPlaneControl(im, with_rotation false);

//  interactive_marker_server_->insert(im, boost::bind(&StepVisual::processInteractionFeedback, this, _1));
//  setCheckState(SIXDOF);
//  menu_handler.apply( *interactive_marker_server_, im.name );
//  interactive_marker_server_->applyChanges();
//}
//void FeetVisual::setPlaneInteractiveMarker()
//{
//  eraseInteractiveMarkers();
//  visualization_msgs::InteractiveMarker im = makeInteractiveMarker("plane");

//  addPlaneControl(im, with_rotation true);

//  interactive_marker_server_->insert(im, boost::bind(&StepVisual::processInteractionFeedback, this, _1));
//  setCheckState(PLANE);
//  menu_handler.apply( *interactive_marker_server_, im.name );
//  interactive_marker_server_->applyChanges();
//}
//*/
// process feedback
//void FeetVisual::processButtonFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
//{
//  ROS_ERROR("Button Feedback");
//  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK)
//  {
//    // activate interaction
// /*   switch(interaction_mode_)
//    {
//    case PLANE:
//      setPlaneInteractiveMarker();
//      break;
//    case SIXDOF:
//      setSixDOFInteractiveMarker();
//      break;
//    case FULLSIXDOF:
//      setFullSixDOFInteractiveMarker();
//      break;
//    default:
//      ROS_WARN("Unknown interaction mode, defaulting to plane interaction.");
//      setPlaneInteractiveMarker();
//      break;
//    }*/
//  }
//}
/*
//void FeetVisual::processInteractionFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
//{
//  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
//  {
//    this->setPosition(Ogre::Vector3( feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z) - origin_);
//    this->setOrientation(Ogre::Quaternion(feedback->pose.orientation.w, feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z ));

//    current_step.foot.pose.position.x = feedback->pose.position.x;
//    current_step.foot.pose.position.y = feedback->pose.position.y;
//    current_step.foot.pose.position.z = feedback->pose.position.z-0.0275;
//    current_step.foot.pose.orientation = feedback->pose.orientation;

//    vigir_footstep_planning_msgs::EditStep edit;
//    edit.header.stamp = ros::Time::now();
//    edit.header.frame_id = current_step.header.frame_id;
//    edit.step = current_step;
//    edit.plan_mode = vigir_footstep_planning_msgs::EditStep::EDIT_MODE_FULL;

//    Q_EMIT(stepEdited(edit));
//  }
//  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
//  {
//    if(snap_to_valid)
//      Q_EMIT(updateStepsPos());
//    else
//      Q_EMIT(footDropped());
//  }
//}*/
/*
//void FeetVisual::processMenuFeedback(const FeedbackConstPtr &feedback)
//{
//  switch (feedback->menu_entry_id)
//  {
//  case 2: // Disable Interaction
//    Q_EMIT(selected(false));
//    setButtonInteractiveMarker();
//    break;
//  case 3: // Delete
//    removeStep();
//    break;
//  case 4: // Select as next start
//    Q_EMIT(cutStepPlanHere(index));
//    break;
//  case 5: // Select as goal for replan
//    Q_EMIT(replanToHere(index));
//    break;
//  case 6: // Snap to Valid position
//    MenuHandler::CheckState state;
//    menu_handler.getCheckState(feedback->menu_entry_id, state);
//    if(state == MenuHandler::CHECKED)
//    {
//      menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::UNCHECKED);
//      snap_to_valid = false;
//    }
//    else
//    {
//      menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::CHECKED);
//      snap_to_valid = true;
//    }
//    menu_handler.apply(*interactive_marker_server_, current_im_name);
//    interactive_marker_server_->applyChanges();
//    break;
//  default:
//    ROS_WARN("Menu Entry not found");
//  }
//}*/
/*
//void FeetVisual::processSetInteractionMode(const FeedbackConstPtr &feedback)
//{
//  MenuHandler::EntryHandle handle = feedback->menu_entry_id;
//  MenuHandler::CheckState state;
//  menu_handler.getCheckState(handle,state);

//  if(state == MenuHandler::CHECKED)
//    return; // nothing changes

//  if(handle == mode_begin)
//  {
//    Q_EMIT(selected(false));
//    setButtonInteractiveMarker();
//  }
//  else
//    Q_EMIT(selected(true));

//  if(handle == mode_begin  + PLANE)
//    setPlaneInteractiveMarker();
//  if(handle == mode_begin + SIXDOF)
//    setSixDOFInteractiveMarker();
//  if(handle == mode_begin + FULLSIXDOF)
//    setFullSixDOFInteractiveMarker();
//}*/

// helper functions to add controls & marker
/*
//void FeetVisual::addPlaneControl(InteractiveMarkerMsg& im, bool with_rotation)
//{
//  visualization_msgs::InteractiveMarkerControl plane_control;
//  plane_control.orientation.w = 1;
//  plane_control.orientation.x = 0;
//  plane_control.orientation.y = 1;
//  plane_control.orientation.z = 0;
//  normalizeQuaternion(plane_control.orientation);
//  // Rotate
//  if(with_rotation)
//  {
//    plane_control.name = "rotate_xy";
//    plane_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
//    im.controls.push_back(plane_control);
//  }

//  // Move Plane with box marker
//  visualization_msgs::Marker box_marker = makeMarker(false);
//  plane_control.markers.push_back( box_marker);
//  plane_control.name = "move_xy";
//  plane_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
//  plane_control.always_visible = true;
//  im.controls.push_back(plane_control);
//}*/
/*
//void FeetVisual::addSixDOFControl(InteractiveMarkerMsg& im)
//{
//  InteractiveMarkerControlMsg control;
//  // x direction
//  control.orientation.w = 1;
//  control.orientation.x = 1;
//  control.orientation.y = 0;
//  control.orientation.z = 0;
//  normalizeQuaternion(control.orientation);
//  control.name = "rotate_x";
//  control.interaction_mode = InteractiveMarkerControlMsg::ROTATE_AXIS;
//  im.controls.push_back(control);
//  control.name = "move_x";
//  control.interaction_mode = InteractiveMarkerControlMsg::MOVE_AXIS;
//  im.controls.push_back(control);

//  // y direction
//  control.orientation.w = 1;
//  control.orientation.x = 0;
//  control.orientation.y = 0;
//  control.orientation.z = 1;
//  normalizeQuaternion(control.orientation);
//  control.name = "rotate_y";
//  control.interaction_mode = InteractiveMarkerControlMsg::ROTATE_AXIS;
//  im.controls.push_back(control);
//  control.name = "move_y";
//  control.interaction_mode = InteractiveMarkerControlMsg::MOVE_AXIS;
//  im.controls.push_back(control);

//  // z direction
//  control.orientation.w = 1;
//  control.orientation.x = 0;
//  control.orientation.y = 1;
//  control.orientation.z = 0;
//  normalizeQuaternion(control.orientation);
//  control.name = "rotate_z";
//  control.interaction_mode = InteractiveMarkerControlMsg::ROTATE_AXIS;
//  im.controls.push_back(control);
//  control.name = "move_z";
//  control.interaction_mode = InteractiveMarkerControlMsg::MOVE_AXIS;
//  im.controls.push_back(control);
//}*/
//visualization_msgs::InteractiveMarker FeetVisual::makeInteractiveMarker(const std::string& type_name)
//{
//  visualization_msgs::InteractiveMarker im;
//  im.header.frame_id = current_feet.header.frame_id;
//  im.header.stamp = ros::Time::now();
//  im.name = type_name;
//  im.pose = toPoseMessage();

//  ros::NodeHandle nh;
//  float size_x;
//  if(nh.getParam("foot/size/x", size_x))
//    im.scale = size_x*2.2;
//  else
//    im.scale = 0.35;
//  return im;
//}
//visualization_msgs::Marker FeetVisual::makeMarker(bool transparent)
//{
//  visualization_msgs::Marker marker;

//  ros::NodeHandle nh;
//  float size_x, size_y, size_z;
//  if(  !(nh.getParam("foot/size/x", size_x)
//       && nh.getParam("foot/size/y", size_y)
//       && nh.getParam("foot/size/z", size_z)))
//  {
//    size_x = 0.3;
//    size_y = 0.3;
//    size_z = 0.3;
//    marker.type = visualization_msgs::Marker::SPHERE;
//  }
//  {
//    marker.type = visualization_msgs::Marker::SPHERE;
//  }

//  marker.scale.x = size_x*2.2;
//  marker.scale.y = size_x*2.2;
//  marker.scale.z = size_x*2.2;
//  marker.color.r = 0;
//  marker.color.g = 0;
//  marker.color.b = 1;
//  marker.color.a = transparent? 0 : 0.1;
//  current_marker = &marker;
//  return marker;
//}

// menu handling:
//void FeetVisual::setCheckState(int checked_index)
//{
//  menu_handler.setCheckState(mode_begin, MenuHandler::UNCHECKED);
//  menu_handler.setCheckState(mode_begin + PLANE, MenuHandler::UNCHECKED);
//  menu_handler.setCheckState(mode_begin + SIXDOF, MenuHandler::UNCHECKED);
//  menu_handler.setCheckState(mode_begin + FULLSIXDOF, MenuHandler::UNCHECKED);

//  // set Checked:
//  menu_handler.setCheckState(mode_begin + checked_index, MenuHandler::CHECKED);
//}

//// helper functions:
///*
//void FeetVisual::disableInteractiveMarker()
//{
//  eraseInteractiveMarkers();
//  interactive_marker_server_->applyChanges();
//}
//void FeetVisual::setInteractionMode(InteractionMode interaction_mode)
//{
//  interaction_mode_ = interaction_mode;
//}
//*/
//void FeetVisual::resetInteractiveMarkers()
//{
//  interactive_marker_server_->erase("button");
//  interactive_marker_server_->erase("plane");
//  interactive_marker_server_->erase("6DOF");
//  interactive_marker_server_->erase("6DOF_withMove3D");
//}
///*
//void FeetVisual::updatePoseInteractiveMarkers()
//{
//  if(interaction) // update position of interactive marker by making new interactive marker
//  {
//    // check current interaction:
//    MenuHandler::CheckState state;
//    menu_handler.getCheckState(mode_begin, state);
//    if(state == MenuHandler::CHECKED)
//    {
//      setButtonInteractiveMarker();
//      return;
//    }
//    menu_handler.getCheckState(mode_begin + PLANE, state);
//    if(state == MenuHandler::CHECKED)
//    {
//      setPlaneInteractiveMarker();
//      return;
//    }
//    menu_handler.getCheckState(mode_begin + SIXDOF, state);
//    if(state == MenuHandler::CHECKED)
//    {
//      setSixDOFInteractiveMarker();
//      return;
//    }
//    menu_handler.getCheckState(mode_begin + FULLSIXDOF, state);
//    if(state == MenuHandler::CHECKED)
//    {
//      setFullSixDOFInteractiveMarker();
//      return;
//    }
//  }
//}*/

//geometry_msgs::Pose FeetVisual::toPoseMessage()
//{
//  geometry_msgs::Pose pose;
//  pose.position.x=position_.x;
//  pose.position.y=position_.y;
//  pose.position.z=position_.z;
//  pose.orientation.x = orientation_.x;
//  pose.orientation.y = orientation_.y;
//  pose.orientation.z = orientation_.z;
//  pose.orientation.w = orientation_.w;
//  normalizeQuaternion(pose.orientation);
//  return pose;
//}

//void FeetVisual::normalizeQuaternion(geometry_msgs::Quaternion& orientation)
//{
//  Ogre::Quaternion o(orientation.w, orientation.x, orientation.y, orientation.z);
//  o.normalise();
//  orientation.w = o.w;
//  orientation.x = o.x;
//  orientation.y = o.y;
//  orientation.z = o.z;
//}

} // end namespace vigir_footstep_planning_rviz_plugin



