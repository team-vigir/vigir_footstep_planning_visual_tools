// ------------------------------
// ACHTUNG!!!!
// Ogre::Quaternion Konstruktor: Quaternion(W, x, y, z)
//
#include <algorithm>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreStringConverter.h>
#include <rviz/geometry.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/mesh_shape.h>
#include <rviz/ogre_helpers/stl_loader.h>
#include <rviz/ogre_helpers/movable_text.h>

#include <vigir_footstep_planning_rviz_plugin/foot_visual.h>

#include <interactive_markers/interactive_marker_server.h>

using namespace interactive_markers;

namespace vigir_footstep_planning_rviz_plugin
{

StepVisual::StepVisual( Ogre::SceneManager* scene_manager,
                        Ogre::SceneNode* parent_node,
                        unsigned int which_foot,
                        std::string frame_id,
                        int index):
  scene_manager_(scene_manager)
  , foot_index(which_foot)
  , index(index)
  , text_visible(false)
  , interaction(false)
  , frame_id_(frame_id)
  , interaction_mode_(PLANE)
  , cost(0)
  , risk(0)
  , interactive_marker_server_(0)
  , mesh_resource("")
  , origin_(0.0f,0.0f,0.0f)
  , current_im_name("")
  , snap_to_valid(false)
{
  frame_node_ = parent_node->createChildSceneNode();
  foot_.reset(new rviz::MeshShape(scene_manager_, frame_node_));
  getParameters();
  foot_->setScale(scale_);
  if(index >= 0) //start and goal have index -1
  {
    createIndexText();
  }
  if(!(which_foot == FootMsg::LEFT || which_foot == FootMsg::RIGHT))
  {
    ROS_ERROR("Invalid index for foot_index, must be FootMsg::LEFT or ::RIGHT");
  }
}

StepVisual::~StepVisual()
{
  if(interaction)
  {
    eraseInteractiveMarkers();
    interactive_marker_server_->applyChanges();
  }
  // Destroy the frame node since we don't need it anymore.
 scene_manager_->destroySceneNode( frame_node_ );
 delete stl_loader_;
 if(index>=0)
   delete text_;
}

// CREATE VISUAL:
void StepVisual::createVisualAt(const Ogre::Vector3& position, const Ogre::Quaternion& orientation)
{
  if(index>=0)
  {
    text_node_->setPosition(position+origin_);
  }
  createFootMesh(mesh_resource);

  if(foot_index == FootMsg::LEFT)
  {
    foot_->setColor(1.0 , 0.0 , 0.0 , 0.9);
  }
  if(foot_index == FootMsg::RIGHT)
  {
    foot_->setColor(0.0 , 1.0 , 0.0 , 0.9);
  }
  foot_->setPosition(position+origin_);
  foot_->setOrientation(orientation);
}

void StepVisual::getParameters()
{
  // get parameters
  ros::NodeHandle nh;
  std::string mesh_dir;
  if (!(nh.getParam("meshes_path", mesh_dir)))
    ROS_ERROR("Could not retrieve mesh directory");

  std::string foot_mesh;
  if(foot_index == FootMsg::LEFT)
  {
    if(!(nh.getParam("foot_mesh/left", foot_mesh)))
      ROS_ERROR("Could not retrieve mesh resource file");
  }
  else
  {
    if(!(nh.getParam("foot_mesh/right", foot_mesh)))
      ROS_ERROR("Could not retrieve mesh resource file");
  }

  mesh_resource = mesh_dir + foot_mesh;


  float x, y, z;
  if(nh.getParam("foot_mesh_properties/origin/x", x)
       && nh.getParam("foot_mesh_properties/origin/y", y)
       && nh.getParam("foot_mesh_properties/origin/z", z))
  {
    origin_.x = x;
    origin_.y = y;
    origin_.z = z; //= Ogre::Vector3(x, y, z);
  }
  else
    ROS_ERROR("Could not retrieve origin of foot mesh");

  if(nh.getParam("foot_mesh_properties/scale/x", x)
       && nh.getParam("foot_mesh_properties/scale/y", y)
       && nh.getParam("foot_mesh_properties/scale/z", z))
  {
    scale_.x = x;
    scale_.y = y;
    scale_.z = z; //= Ogre::Vector3(x,y,z);
  }
  else
    ROS_ERROR("Could not retrieve scale of foot mesh");

}

void StepVisual::createVisualAt(const geometry_msgs::Point& foot_position, const geometry_msgs::Quaternion& foot_orientation)
{
  Ogre::Vector3	p(foot_position.x, foot_position.y, foot_position.z);
  Ogre::Quaternion o(foot_orientation.w, foot_orientation.x, foot_orientation.y, foot_orientation.z);

  createVisualAt(p, o);
}

void StepVisual::createByMessage(const vigir_footstep_planning_msgs::Step msg)
{
  current_step = msg;
  cost = msg.cost;
  risk = msg.risk;
  createVisualAt(msg.foot.pose.position, msg.foot.pose.orientation);
}


// Position and orientation are passed through to the SceneNode.
void StepVisual::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
}

void StepVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
}

void StepVisual::setPosition( const Ogre::Vector3& position)
{
  if(index>=0)
  {
    text_node_->setPosition(position+origin_);
  }
  foot_->setPosition(position+origin_);
  if(interaction) //update pose of active interactive marker
  {
    //todo
  }
}

void StepVisual::setOrientation( const Ogre::Quaternion& orientation)
{

  foot_->setOrientation(orientation);
  if(interaction) //update pose of active interactive marker
  {
    //todo
  }
}

void StepVisual::setColor( float r, float g, float b, float a )
{
  foot_->setColor( r, g, b, a );
}

void StepVisual::createFootMesh(const std::string& path)
{
  stl_loader_ = new ogre_tools::STLLoader();
  if(stl_loader_->load(path))
  {
    foot_->beginTriangles();
    for (unsigned i=0; i < stl_loader_->triangles_.size(); i++)
    {
      ogre_tools::STLLoader::Triangle & t = stl_loader_->triangles_[i];
      foot_->addVertex(t.vertices_[0],t.normal_);
      foot_->addVertex(t.vertices_[1],t.normal_);
      foot_->addVertex(t.vertices_[2],t.normal_);
    }
    foot_->endTriangles();
  }
  else
    ROS_ERROR("Could not load mesh at %s", path.c_str());
}

void StepVisual::setVisible(bool visible)
{
  frame_node_->setVisible(visible);
  if(index>=0)
    text_node_->setVisible(text_visible && visible);
  if(interaction)
    visible ? setButtonInteractiveMarker() : disableInteractiveMarker();
}

void StepVisual::processInteractionFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
  {
    foot_->setPosition(Ogre::Vector3( feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z));
    foot_->setOrientation(Ogre::Quaternion(feedback->pose.orientation.w, feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z ));
    if(index>=0)
      text_node_->setPosition(Ogre::Vector3( feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z ));

    current_step.foot.pose.position.x = feedback->pose.position.x;
    current_step.foot.pose.position.y = feedback->pose.position.y;
    current_step.foot.pose.position.z = feedback->pose.position.z-0.0275;
    current_step.foot.pose.orientation = feedback->pose.orientation;

    vigir_footstep_planning_msgs::EditStep edit;
    edit.header.stamp = ros::Time::now();
    edit.header.frame_id = current_step.header.frame_id;
    edit.step = current_step;
    edit.plan_mode = vigir_footstep_planning_msgs::EditStep::EDIT_MODE_FULL;

    Q_EMIT(stepEdited(edit));
  }
  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
  {
    if(snap_to_valid)
      Q_EMIT(updateStepsPos());
    else
      Q_EMIT(footDropped());
  }
}

void StepVisual::processButtonFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK)
  {
    // activate interaction
    switch(interaction_mode_)
    {
    case PLANE:
      setPlaneInteractiveMarker();
      break;
    case SIXDOF:
      setSixDOFInteractiveMarker();
      break;
    case FULLSIXDOF:
      setFullSixDOFInteractiveMarker();
      break;
    default:
      ROS_WARN("Unknown interaction mode, defaulting to plane interaction.");
      setPlaneInteractiveMarker();
      break;
    }

  }
}

void StepVisual::processMenuFeedback(const FeedbackConstPtr &feedback)
{
  switch (feedback->menu_entry_id)
  {
  case 2: // Disable Interaction
    setButtonInteractiveMarker();
    break;
  case 3: // Delete
    removeStep();
    break;
  case 4: // Select as next start
    Q_EMIT(cutStepPlanHere(index));
    break;
  case 5: // Select as goal for replan
    Q_EMIT(replanToHere(index));
    break;
  case 6: // Snap to Valid position
    ROS_ERROR("Test");
    MenuHandler::CheckState state;
    menu_handler.getCheckState(feedback->menu_entry_id, state);
    if(state == MenuHandler::CHECKED)
    {
      menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::UNCHECKED);
      snap_to_valid = false;
      ROS_ERROR("false");
    }
    else
    {
      menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::CHECKED);
      snap_to_valid = true;
      ROS_ERROR("true");
    }
    menu_handler.apply(*interactive_marker_server_, current_im_name);
    interactive_marker_server_->applyChanges();
    break;
  default:
    ROS_WARN("Menu Entry not found");
  }
}

void StepVisual::processSetInteractionMode(const FeedbackConstPtr &feedback)
{
  MenuHandler::EntryHandle handle = feedback->menu_entry_id;
  MenuHandler::CheckState state;
  menu_handler.getCheckState(handle,state);

  if(state == MenuHandler::CHECKED)
    return; // nothing changes

  if(handle == mode_begin)
    setButtonInteractiveMarker();
  if(handle == mode_begin  + PLANE)
    setPlaneInteractiveMarker();
  if(handle == mode_begin + SIXDOF)
    setSixDOFInteractiveMarker();
  if(handle == mode_begin + FULLSIXDOF)
    setFullSixDOFInteractiveMarker();
}


void StepVisual::removeStep()
{
  vigir_footstep_planning_msgs::EditStep edit;
  edit.header.stamp = ros::Time::now();
  edit.header.frame_id = current_step.header.frame_id;
  edit.step = current_step;
  edit.plan_mode = vigir_footstep_planning_msgs::EditStep::EDIT_MODE_REMOVE;

  Q_EMIT(stepEdited(edit));
}

void StepVisual::setValid(bool valid)
{
  if(!valid)
  {
    foot_->setColor(0.5 , 0.5 , 0.5 , 0.9);
  }
  else
  {
    if(foot_index == FootMsg::LEFT)
    {
      foot_->setColor(1.0 , 0.0 , 0.0 , 0.9);
    }
    if(foot_index == FootMsg::RIGHT)
    {
      foot_->setColor(0.0 , 1.0 , 0.0 , 0.9);
    }
  }
}



void StepVisual::initializeInteractiveMarker(InteractiveMarkerServer* marker_server)
{
  interactive_marker_server_ = marker_server;

  MenuHandler::EntryHandle interaction_sub = menu_handler.insert( "Interaction"); // case 1
  menu_handler.insert( "Disable Interaction",  boost::bind(&StepVisual::processMenuFeedback, this, _1)); // case 2
  menu_handler.insert("Delete", boost::bind(&StepVisual::processMenuFeedback, this, _1)); // case 3
  menu_handler.insert("Select as next start", boost::bind(&StepVisual::processMenuFeedback, this, _1)); // case 4
  menu_handler.insert("Replan to here", boost::bind(&StepVisual::processMenuFeedback, this, _1)); // case 5
  menu_handler.setCheckState(menu_handler.insert("Snap to valid position (if available)", boost::bind(&StepVisual::processMenuFeedback, this, _1)), MenuHandler::CHECKED); // case 6
  snap_to_valid = true;
  // Interaction Sub menu:
  mode_begin = menu_handler.insert(interaction_sub, "None", boost::bind(&StepVisual::processSetInteractionMode, this, _1));
  menu_handler.insert(interaction_sub, "2D Plane", boost::bind(&StepVisual::processSetInteractionMode, this, _1));
  menu_handler.insert(interaction_sub, "6 DOF", boost::bind(&StepVisual::processSetInteractionMode, this, _1));
  menu_handler.insert(interaction_sub, "Full 6 DOF", boost::bind(&StepVisual::processSetInteractionMode, this, _1));

  setButtonInteractiveMarker();
  interaction = true;
}

void StepVisual::setCheckState(int checked_index)
{
  menu_handler.setCheckState(mode_begin, MenuHandler::UNCHECKED);
  menu_handler.setCheckState(mode_begin + PLANE, MenuHandler::UNCHECKED);
  menu_handler.setCheckState(mode_begin + SIXDOF, MenuHandler::UNCHECKED);
  menu_handler.setCheckState(mode_begin + FULLSIXDOF, MenuHandler::UNCHECKED);

  // set Checked:
  menu_handler.setCheckState(mode_begin + checked_index, MenuHandler::CHECKED);
}

void StepVisual::displayIndex(bool visible)
{
  if(index>=0)
  {
    text_node_->setVisible(visible);
    text_visible = visible;
  }
}

visualization_msgs::Marker StepVisual::makeMarker(bool transparent)
{
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = 0.3; //TODO: Bounding box des fußes
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.r = foot_index == FootMsg::LEFT ? 1 : 0;
  marker.color.g = foot_index == FootMsg::RIGHT ? 1 : 0;
  marker.color.b = 0;
  marker.color.a = transparent? 0 : 0.1;
  return marker;
}

visualization_msgs::InteractiveMarker StepVisual::makeInteractiveMarker(const std::string& type_name)
{
  visualization_msgs::InteractiveMarker im;
  im.header.frame_id = frame_id_;
  im.header.stamp = ros::Time::now();
  im.name = type_name + QString::number(index).toStdString();
  current_im_name = im.name;
  im.pose = toPoseMessage();
  im.scale = 0.35;
  return im;
}

void StepVisual::setButtonInteractiveMarker()
{
  // remove other interactive marker
  eraseInteractiveMarkers();

  visualization_msgs::InteractiveMarker im = makeInteractiveMarker("button");
  visualization_msgs::Marker box_marker = makeMarker(false/*true*/);

  visualization_msgs::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.markers.push_back( box_marker);
  control.always_visible = false;

  im.controls.push_back(control);
  interactive_marker_server_->insert(im, boost::bind(&StepVisual::processButtonFeedback, this, _1));
  setCheckState(0); // mode=none
  menu_handler.apply( *interactive_marker_server_, im.name );
  interactive_marker_server_->applyChanges();
}

void StepVisual::setPlaneInteractiveMarker()
{
  eraseInteractiveMarkers();
  visualization_msgs::InteractiveMarker im = makeInteractiveMarker("plane");

  addPlaneControl(im, /*with_rotation*/true);

  interactive_marker_server_->insert(im, boost::bind(&StepVisual::processInteractionFeedback, this, _1));
  setCheckState(PLANE);
  menu_handler.apply( *interactive_marker_server_, im.name );
  interactive_marker_server_->applyChanges();
}


// Move on 2d plane with box marker
// Move 6DOF with default controls
void StepVisual::setSixDOFInteractiveMarker()
{
  eraseInteractiveMarkers();
  InteractiveMarkerMsg im = makeInteractiveMarker("6DOF");

  addSixDOFControl(im);

  addPlaneControl(im, /*with_rotation*/ false);

  interactive_marker_server_->insert(im, boost::bind(&StepVisual::processInteractionFeedback, this, _1));
  setCheckState(SIXDOF);
  menu_handler.apply( *interactive_marker_server_, im.name );
  interactive_marker_server_->applyChanges();
}

void StepVisual::setFullSixDOFInteractiveMarker()
{
  ROS_INFO("Hold shift to move in/out the camera plane");
  eraseInteractiveMarkers();
  InteractiveMarkerMsg im = makeInteractiveMarker("6DOF_withMove3D");

  addSixDOFControl(im);

  // Move 3D with box marker
  InteractiveMarkerControlMsg control;
  control.name = "move_3d";
  MarkerMsg marker = makeMarker(/*transparent*/ false);
  control.markers.push_back(marker);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
  control.always_visible = false;
  im.controls.push_back(control);

  interactive_marker_server_->insert(im, boost::bind(&StepVisual::processInteractionFeedback, this, _1));
  setCheckState(FULLSIXDOF);
  menu_handler.apply( *interactive_marker_server_, im.name );
  interactive_marker_server_->applyChanges();
}

void StepVisual::addSixDOFControl(InteractiveMarkerMsg& im)
{
  InteractiveMarkerControlMsg control;
  // x direction
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  normalizeQuaternion(control.orientation);
  control.name = "rotate_x";
  control.interaction_mode = InteractiveMarkerControlMsg::ROTATE_AXIS;
  im.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControlMsg::MOVE_AXIS;
  im.controls.push_back(control);

  // y direction
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  normalizeQuaternion(control.orientation);
  control.name = "rotate_y";
  control.interaction_mode = InteractiveMarkerControlMsg::ROTATE_AXIS;
  im.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControlMsg::MOVE_AXIS;
  im.controls.push_back(control);

  // z direction
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  normalizeQuaternion(control.orientation);
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControlMsg::ROTATE_AXIS;
  im.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControlMsg::MOVE_AXIS;
  im.controls.push_back(control);
}

void StepVisual::addPlaneControl(InteractiveMarkerMsg& im, bool with_rotation)
{
  visualization_msgs::InteractiveMarkerControl plane_control;
  plane_control.orientation.w = 1;
  plane_control.orientation.x = 0;
  plane_control.orientation.y = 1;
  plane_control.orientation.z = 0;
  normalizeQuaternion(plane_control.orientation);
  // Rotate
  if(with_rotation)
  {
    plane_control.name = "rotate_xy";
    plane_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    im.controls.push_back(plane_control);
  }

  // Move Plane with box marker
  visualization_msgs::Marker box_marker = makeMarker(false);
  plane_control.markers.push_back( box_marker);
  plane_control.name = "move_xy";
  plane_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  plane_control.always_visible = true;
  im.controls.push_back(plane_control);
}

void StepVisual::disableInteractiveMarker()
{
  eraseInteractiveMarkers();
  interactive_marker_server_->applyChanges();
}

void StepVisual::createIndexText()
{
  text_node_ = frame_node_->createChildSceneNode();
  text_= new rviz::MovableText(Ogre::StringConverter::toString(index));
  text_->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_CENTER);
  text_->setCharacterHeight(0.1);
  text_node_->attachObject(text_);
  text_node_->setVisible(false);
}

geometry_msgs::Pose StepVisual::toPoseMessage()
{
  geometry_msgs::Pose pose;
  Ogre::Vector3 pos = foot_->getPosition();
  pose.position.x=pos.x;
  pose.position.y=pos.y;
  pose.position.z=pos.z;
  Ogre::Quaternion orient = foot_->getOrientation();
  pose.orientation.x = orient.x;
  pose.orientation.y = orient.y;
  pose.orientation.z = orient.z;
  pose.orientation.w = orient.w;
  normalizeQuaternion(pose.orientation);
  return pose;
}

void StepVisual::setInteractionMode(InteractionMode interaction_mode)
{
  interaction_mode_ = interaction_mode;
}

void StepVisual::eraseInteractiveMarkers()
{
  interactive_marker_server_->erase("button"+QString::number(index).toStdString());
  interactive_marker_server_->erase("plane"+ QString::number(index).toStdString());
  interactive_marker_server_->erase("6DOF"+QString::number(index).toStdString());
  interactive_marker_server_->erase("6DOF_withMove3D"+QString::number(index).toStdString());
}

void StepVisual::visualizeCost(float max)
{
  // ratio = [0,1] is 0 when cost=max and 1 wenn cost=0.
  float ratio = std::min((max-cost)/max, 1.0f);
  if(ratio < 0)
    ratio=0;

  foot_->setColor(1-ratio, 0.0, ratio, 0.9);
}

void StepVisual::normalizeQuaternion(geometry_msgs::Quaternion& orientation)
{
  Ogre::Quaternion o(orientation.w, orientation.x, orientation.y, orientation.z);
  o.normalise();
  orientation.w = o.w;
  orientation.x = o.x;
  orientation.y = o.y;
  orientation.z = o.z;
}

void StepVisual::updateStep(const vigir_footstep_planning_msgs::Step updated_msg)
{
  this->setPosition(Ogre::Vector3(updated_msg.foot.pose.position.x,
                                  updated_msg.foot.pose.position.y,
                                  updated_msg.foot.pose.position.z));

  this->setOrientation(Ogre::Quaternion(updated_msg.foot.pose.orientation.w,
                                        updated_msg.foot.pose.orientation.x,
                                        updated_msg.foot.pose.orientation.y,
                                        updated_msg.foot.pose.orientation.z));
  current_step = updated_msg;
  cost = updated_msg.cost;
  risk = updated_msg.risk;
}



} // end namespace vigir_footstep_planning_rviz_plugin



