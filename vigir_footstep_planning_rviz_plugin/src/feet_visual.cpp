#include <vigir_footstep_planning_rviz_plugin/feet_visual.h>
#include <vigir_footstep_planning_rviz_plugin/step_visual.h>

#include <vigir_footstep_planning_rviz_plugin/common/interactive_marker_functions.h>
#include <vigir_footstep_planning_rviz_plugin/common/ogre_visualization_msgs_functions.h>

#include <interactive_markers/interactive_marker_server.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

using namespace interactive_markers;

namespace vigir_footstep_planning_rviz_plugin
{

FeetVisual::FeetVisual( Ogre::SceneManager* scene_manager,
                        Ogre::SceneNode* parent_node,
                        FeetType type):
  scene_manager_(scene_manager)
, feet_type(type)
, position_(0.f,0.f,0.f)
, orientation_(1.f,0.f,0.f,0.f)
, frame_id("")
, pos_left(0.f,0.f,0.f)
, pos_right(0.f,0.f,0.f)
, interactive_marker_server_(0)
, interaction_enabled(false)
, im_scale(1.f)
, interaction_mode(PLANE)
{
  frame_node_ = parent_node->createChildSceneNode();
  feet_visuals_.resize(2);
  feet_visuals_[LEFT].reset(new StepVisual(scene_manager, frame_node_, FootMsg::LEFT));
  feet_visuals_[RIGHT].reset(new StepVisual(scene_manager, frame_node_, FootMsg::RIGHT));
  setFeetPositioning();
}

FeetVisual::~FeetVisual()
{
  deleteInteractiveMarker();
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode( frame_node_ );
}


void FeetVisual::setFeetPositioning()
{
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
    pos_left.x = -frame_x;
    pos_left.y = seperation/2-frame_y;
    pos_left.z = -frame_z;
    if(   nh.getParam("foot/right/foot_frame/x", frame_x)
          && nh.getParam("foot/right/foot_frame/y", frame_y)
          && nh.getParam("foot/right/foot_frame/z", frame_z)
          )
    {
      pos_right.x = -frame_x;
      pos_right.y = -seperation/2-frame_y;
      pos_right.z = -frame_z;
    }
  }
  else
    ROS_INFO("Could not retrieve feet positioning from /johnny5/footstep_planning/foot/");

  im_scale = pos_left.y - pos_right.y;
}


// CREATE VISUAL:
void FeetVisual::createFeetAt(const Ogre::Vector3& position, const Ogre::Quaternion& orientation)
{
  position_ = position;
  orientation_ = orientation;

  Ogre::Vector3 transformed_pos_left = pos_left;
  Ogre::Vector3 transformed_pos_right = pos_right;
  computePositioning(transformed_pos_left, orientation);
  computePositioning(transformed_pos_right, orientation);

  feet_visuals_[LEFT]->createVisualAt(position + transformed_pos_left, orientation);
  feet_visuals_[RIGHT]->createVisualAt(position + transformed_pos_right, orientation);
}

void FeetVisual::createByMessage(const vigir_footstep_planning_msgs::Feet msg)
{
  feet_msg = msg;
  frame_id = msg.header.frame_id;

  feet_visuals_[RIGHT]->createByFootMsg(msg.right);
  feet_visuals_[LEFT]->createByFootMsg(msg.left);

  this->setPlannerPose(msg);
}

void FeetVisual::setRobotPose(const Ogre::Vector3& position, const Ogre::Quaternion& orientation)
{
  // planner pose
  position_ = position;
  orientation_ = orientation;
  //todo
  Ogre::Vector3 transformed_pos_left = pos_left;
  Ogre::Vector3 transformed_pos_right = pos_right;
  computePositioning(transformed_pos_left, orientation);
  computePositioning(transformed_pos_right, orientation);

  feet_visuals_[LEFT]->setPosition(position + transformed_pos_left);
  feet_visuals_[LEFT]->setOrientation(orientation);
  feet_visuals_[RIGHT]->setPosition(position + transformed_pos_right);
  feet_visuals_[RIGHT]->setOrientation(orientation);



  geometry_msgs::Pose pose;
  getPoseMsg(pose, position+transformed_pos_left, orientation);
  feet_msg.left.pose = pose;
  getPoseMsg(pose, position + transformed_pos_right, orientation);
  feet_msg.right.pose = pose;
}

void FeetVisual::setPlannerPose(const FeetMsg& msg)
{
  feet_msg = msg;

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<vigir_footstep_planning_msgs::TransformFeetPosesService>("transform_feet_poses");
  vigir_footstep_planning_msgs::TransformFeetPosesService srv;
  srv.request.feet = msg;
  srv.request.target_frame.data = "planner";
  if(client.call(srv))
  {
    Ogre::Vector3 left = getOgreVector(srv.response.feet.left.pose.position);
    Ogre::Vector3 right = getOgreVector(srv.response.feet.right.pose.position);
    position_ = 0.5*(left + right);
    orientation_ = getOgreQuaternion(srv.response.feet.left.pose.orientation);
  }
  else
    ROS_ERROR("Failed to call service transform_feet_poses");
}


void FeetVisual::updateFeetMsg(const vigir_footstep_planning_msgs::Feet msg, Ogre::Vector3 frame_position, Ogre::Quaternion frame_orientation)
{
  if(feet_visuals_[LEFT])
  {
    feet_visuals_[LEFT]->updateFootMsg(msg.left);
  }
  else
    feet_visuals_[LEFT]->createByFootMsg(msg.left);

  if(feet_visuals_[RIGHT])
  {
    feet_visuals_[RIGHT]->updateFootMsg(msg.right);
  }
  else
    feet_visuals_[RIGHT]->createByFootMsg(msg.right);


  this->setFrameOrientation(frame_orientation);
  this->setFramePosition(frame_position);

  feet_msg = msg;

  setPlannerPose(msg);
  resetInteractiveMarker();
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

  if(!visible)
    deleteInteractiveMarker();
  else
    resetInteractiveMarker();
}


// ---- Interactive Marker Functions:
void FeetVisual::initializeInteractiveMarker(InteractiveMarkerServer* marker_server)
{
  interactive_marker_server_ = marker_server;

  // case 1:
  menu_handler.setCheckState(menu_handler.insert("Enable 6DOF Interaction", boost::bind(&FeetVisual::processMenuFeedback, this, _1)), MenuHandler::UNCHECKED);
  // case 2:
  menu_handler.insert("Disable Interaction", boost::bind(&FeetVisual::processMenuFeedback, this, _1));

  setButtonInteractiveMarker();
}

void FeetVisual::setButtonInteractiveMarker()
{
  if(!interactive_marker_server_)
    return;

  interaction_enabled = false;

  visualization_msgs::InteractiveMarker im = getInteractiveMarker();

  addButtonControl(im, makeMarker());
  interactive_marker_server_->insert(im, boost::bind(&FeetVisual::processButtonFeedback, this, _1));
  menu_handler.apply( *interactive_marker_server_, im.name );
  interactive_marker_server_->applyChanges();
}

void FeetVisual::setSixDOFInteractiveMarker()
{
  interaction_enabled = true;

  visualization_msgs::InteractiveMarker im = getInteractiveMarker();

  addSixDOFControl(im);
  addPlaneControl(im, makeMarker(), false);

  interactive_marker_server_->insert(im, boost::bind(&FeetVisual::processInteractionFeedback, this, _1));
  menu_handler.apply( *interactive_marker_server_, im.name );
  interactive_marker_server_->applyChanges();
}

void FeetVisual::setPlaneInteractiveMarker()
{
  interaction_enabled = true;

  visualization_msgs::InteractiveMarker im = getInteractiveMarker();
  addPlaneControl(im, makeMarker(), true);
  interactive_marker_server_->insert(im, boost::bind(&FeetVisual::processInteractionFeedback, this, _1));
  menu_handler.apply(*interactive_marker_server_, im.name);
  interactive_marker_server_->applyChanges();
}

void FeetVisual::processInteractionFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  Ogre::Vector3 position(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
  Ogre::Quaternion orientation(feedback->pose.orientation.w, feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z);

  if(feedback->event_type == InteractiveMarkerFeedbackMsg::POSE_UPDATE)
  {
    this->setRobotPose(position, orientation);
  }
  if(feedback->event_type == InteractiveMarkerFeedbackMsg::MOUSE_UP)
  {
    Q_EMIT(feetPoseChanged(position, orientation));
  }
}

void FeetVisual::processButtonFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK)
  {
    switch (interaction_mode)
    {
    case PLANE:
      setPlaneInteractiveMarker();
      break;
    case SIXDOF:
      setSixDOFInteractiveMarker();
      break;
    case FULLSIXDOF:
      setSixDOFInteractiveMarker();
      break;
    default:
      setButtonInteractiveMarker();
      break;
    }
  }
}

void FeetVisual::processMenuFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  MenuHandler::EntryHandle handle = feedback->menu_entry_id;
  MenuHandler::CheckState state;
  menu_handler.getCheckState(handle, state);

  switch (handle)
  {
  case 1: // 6 DOF interaction
    if(state == MenuHandler::UNCHECKED)
    {
      menu_handler.setCheckState(handle, MenuHandler::CHECKED);
      interaction_mode = SIXDOF;
      setSixDOFInteractiveMarker();
    }
    else
    {
      menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
      interaction_mode = PLANE;
      if(interaction_enabled)
        setPlaneInteractiveMarker();
      else
        setButtonInteractiveMarker();
    }
    break;
  case 2: // disable interactive marker
    interaction_enabled = false;
    setButtonInteractiveMarker();
    break;
  default:
    ROS_WARN("Menu Entry not found");
  }
}

void FeetVisual::resetInteractiveMarker()
{
  if(interactive_marker_server_)
  {
    if(interaction_enabled)
    {
      switch(interaction_mode)
      {
      case PLANE:
        setPlaneInteractiveMarker();
        break;
      case SIXDOF:
        setSixDOFInteractiveMarker();
        break;
      case FULLSIXDOF:
        setSixDOFInteractiveMarker();
        break;
      default:
        setButtonInteractiveMarker();
        break;
      }
    }
    else
      setButtonInteractiveMarker();
  }
}

void FeetVisual::deleteInteractiveMarker()
{
  if(interactive_marker_server_)
  {
    if(feet_type == GOAL)
      interactive_marker_server_->erase("goal_feet_im"); // erase old marker if exists
    if(feet_type == START)
      interactive_marker_server_->erase("start_feet_im"); // erase old marker if exists
    interactive_marker_server_->applyChanges();
  }
}

visualization_msgs::Marker FeetVisual::makeMarker()
{
  std_msgs::ColorRGBA color;
  color.r = (feet_type == GOAL) ? 1 : 0;
  color.g = (feet_type == START) ? 1 : 0;
  color.b = 0;
  color.a = 0.4;
  visualization_msgs::Marker marker = makeSphereMarker(im_scale, color);
  return marker;
}

visualization_msgs::InteractiveMarker FeetVisual::getInteractiveMarker()
{
  deleteInteractiveMarker(); // remove old interactive marker
  geometry_msgs::Pose planner_pose;
  getPoseMsg(planner_pose, position_, orientation_);
  visualization_msgs::InteractiveMarker im = makeInteractiveMarker(feet_type == GOAL ? "goal_feet_im" : "start_feet_im",
                                                                   frame_id, planner_pose, im_scale);
  return im;
}


//  - helper function: compute offset compared to center point of the feet with given orientation
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

} // end namespace vigir_footstep_planning_rviz_plugin



