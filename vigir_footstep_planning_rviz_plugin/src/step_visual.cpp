// ------------------------------
// ACHTUNG!!!!
// Ogre::Quaternion Konstruktor: Quaternion(W, x, y, z)
//
#include <vigir_footstep_planning_rviz_plugin/step_visual.h>
#include <vigir_footstep_planning_rviz_plugin/common/ogre_visualization_msgs_functions.h>
#include <vigir_footstep_planning_rviz_plugin/common/interactive_marker_functions.h>

#include <interactive_markers/interactive_marker_server.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <rviz/ogre_helpers/mesh_shape.h>
#include <rviz/ogre_helpers/stl_loader.h>
#include <rviz/ogre_helpers/movable_text.h>


using namespace interactive_markers;

namespace vigir_footstep_planning_rviz_plugin
{

StepVisual::StepVisual( Ogre::SceneManager* scene_manager,
                        Ogre::SceneNode* parent_node,
                        unsigned int which_foot,
                        std::string frame_id/*= ""*/,
                        int index/*= -1*/):
    scene_manager_(scene_manager)
  , text_node_(0)
  , foot_index(which_foot)
  , index(index)
  , cost(0)
  , risk(0)
  , interactive_marker_server_(0)
  , frame_id_(frame_id)
  , interaction_mode_(PLANE)
  , snap_to_valid(true)
  , display_index(false)
  , im_scale(0.35)
  , mesh_origin_(0.0f,0.0f,0.0f) // origin of frame
  , visualize_valid(false)
{
  // Frame node of the scene in which the foot is placed
  frame_node_ = parent_node->createChildSceneNode();
  // Frame Node of the foot, handles the right positioning of foot
  foot_node_ = frame_node_->createChildSceneNode();
  setFootProperties();

  foot_.reset(new rviz::MeshShape(scene_manager_, foot_node_));
  foot_->setScale(scale_);

  if(index >= 0) //start and goal have index -1
    createIndexText();
}

StepVisual::~StepVisual()
{
  if(interactive_marker_server_)
  {
    deleteInteractiveMarker();
  }
  // Destroy the frame node since we don't need it anymore.
 scene_manager_->destroySceneNode( frame_node_ );
}

void StepVisual::setFootProperties()
{
  ros::NodeHandle nh;
  float x, y, z;
  if(nh.getParam("urdf_properties/mesh_origin/x", x)
       && nh.getParam("urdf_properties/mesh_origin/y", y)
       && nh.getParam("urdf_properties/mesh_origin/z", z))
  {
    mesh_origin_ = Ogre::Vector3(x, y, z);
  }
  else
    ROS_ERROR("Could not retrieve origin of foot mesh");

  if(nh.getParam("urdf_properties/mesh_scale/x", x)
       && nh.getParam("urdf_properties/mesh_scale/y", y)
       && nh.getParam("urdf_properties/mesh_scale/z", z))
  {
    scale_ = Ogre::Vector3(x,y,z);
  }
  else
    ROS_ERROR("Could not retrieve scale of foot mesh");

  if(!nh.getParam("foot/size/x", im_scale))
    im_scale = 0.35;
  text_position_ = Ogre::Vector3::ZERO;

  if(!nh.getParam("foot/size/z", z))
     text_position_.z = 0.05;
  else
    text_position_.z = z + 0.05;
}

// CREATE VISUAL:
void StepVisual::createByMessage(const vigir_footstep_planning_msgs::Step& msg)
{
  current_step = msg;
  cost = msg.cost;
  risk = msg.risk;
  createVisualAt(getOgreVector(msg.foot.pose.position), getOgreQuaternion(msg.foot.pose.orientation));
  visualizeValid(msg.valid && !msg.colliding);
}

void StepVisual::createByFootMsg(const vigir_footstep_planning_msgs::Foot& msg)
{
  createVisualAt(getOgreVector(msg.pose.position), getOgreQuaternion(msg.pose.orientation));
}

void StepVisual::createVisualAt(const Ogre::Vector3& position, const Ogre::Quaternion& orientation)
{
  createFootMesh();

  if(foot_index == FootMsg::LEFT)
  {
    foot_->setColor(1.0 , 0.0 , 0.0 , 1.0);
  }
  if(foot_index == FootMsg::RIGHT)
  {
    foot_->setColor(0.0 , 1.0 , 0.0 , 1.0);
  }

  setPosition(position);
  setOrientation(orientation);
}

void StepVisual::createFootMesh()
{
  // 1) Get Mesh resource:
  ros::NodeHandle nh;
  std::string mesh_dir = "";
  if (!(nh.getParam("meshes_path", mesh_dir)))
    ROS_ERROR("Could not retrieve mesh directory");

  std::string foot_mesh = "";
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

  // 2) Load Mesh
  ogre_tools::STLLoader* stl_loader = new ogre_tools::STLLoader();
  if(stl_loader->load(mesh_dir + foot_mesh))
  {
    foot_->beginTriangles();

    for (unsigned i=0; i < stl_loader->triangles_.size(); i++)
    {
      ogre_tools::STLLoader::Triangle & t = stl_loader->triangles_[i];
      foot_->addVertex(t.vertices_[0],t.normal_);
      foot_->addVertex(t.vertices_[1],t.normal_);
      foot_->addVertex(t.vertices_[2],t.normal_);
    }
    foot_->endTriangles();
  // 3) Set position of the foot in foot_frame_node
    foot_->setPosition(mesh_origin_); //position of mesh relative to robot frame
    foot_->setOrientation(Ogre::Quaternion(1,0,0,0));
  }
  else
    ROS_ERROR("Could not load mesh at %s", (mesh_dir + foot_mesh).c_str());

  delete stl_loader;
}

void StepVisual::setVisible(bool visible)
{
  frame_node_->setVisible(visible);
  if(index>=0)
    text_node_->setVisible(display_index && visible);
  if(interactive_marker_server_)
    visible ? setButtonInteractiveMarker() : deleteInteractiveMarker();
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
    text_node_->setPosition(position+text_position_);
  }
  foot_node_->setPosition(position);
}
/*
void StepVisual::setPose(const geometry_msgs::Pose& pose)
{
  Ogre::Quaternion current_orientation = getOgreQuaternion(pose.orientation);
  Ogre::Matrix3 rotMat;
  current_orientation.ToRotationMatrix(rotMat);
  Ogre::Vector3 current_frame_origin = rotMat*frame_origin_;
  setPosition(getOgreVector(pose.position));
  setOrientation(current_orientation);
  current_frame_origin += getOgreVector(pose.position);
  //ROS_ERROR("position msg: %f, %f, %f", current_frame_origin.x, current_frame_origin.y, current_frame_origin.z);

}
*/
void StepVisual::setOrientation( const Ogre::Quaternion& orientation)
{
  foot_node_->setOrientation(orientation);
}

Ogre::Vector3 StepVisual::getPosition()
{
  return foot_node_->getPosition();
}

Ogre::Quaternion StepVisual::getOrientation()
{
  return foot_node_->getOrientation();
}

void StepVisual::setColor( float r, float g, float b, float a )
{
  foot_->setColor( r, g, b, a );
}

void StepVisual::displayIndex(bool visible)
{
  if(index>=0)
  {
    text_node_->setVisible(visible);
    display_index = visible;
  }
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

// called when stepPlanUpdated() was emitted
void StepVisual::updateStepMsg(const vigir_footstep_planning_msgs::Step updated_msg)
{
  setPosition(getOgreVector(updated_msg.foot.pose.position));
  setOrientation(getOgreQuaternion(updated_msg.foot.pose.orientation));

  current_step = updated_msg;
  cost = updated_msg.cost;
  risk = updated_msg.risk;
  visualizeValid(!updated_msg.colliding && updated_msg.valid);
  if(interactive_marker_server_)
    resetInteractiveMarker();
}

// called from FeetVisual
void StepVisual::updateFootMsg(const vigir_footstep_planning_msgs::Foot updated_msg)
{
  setPosition(getOgreVector(updated_msg.pose.position));
  setOrientation(getOgreQuaternion(updated_msg.pose.orientation));
  if(interactive_marker_server_)
    resetInteractiveMarker();
}

// position changed through editing of step property:
void StepVisual::editedPose(Ogre::Vector3 new_position, Ogre::Quaternion new_orientation)
{
  setPosition(new_position);
  setOrientation(new_orientation);
  resetInteractiveMarker();

  getPoseMsg(current_step.foot.pose, new_position, new_orientation);

  vigir_footstep_planning_msgs::EditStep edit;
  edit.header.stamp = ros::Time::now();
  edit.header.frame_id = current_step.header.frame_id;
  edit.step = current_step;
  edit.plan_mode = vigir_footstep_planning_msgs::EditStep::EDIT_MODE_FULL;

  Q_EMIT(stepChanged(edit));
}

void StepVisual::visualizeValid(bool valid)
{
  if(!valid && visualize_valid)
  {
    foot_->setColor(0.5 , 0.5 , 0.5 , 1);
  }
  else
  {
    if(foot_index == FootMsg::LEFT)
    {
      foot_->setColor(1.0 , 0.0 , 0.0 , 1);
    }
    if(foot_index == FootMsg::RIGHT)
    {
      foot_->setColor(0.0 , 1.0 , 0.0 , 1);
    }
  }
}

void StepVisual::visualizeCost(float max)
{
  // ratio = [0,1] is 0 when cost=max and 1 wenn cost=0.
  float ratio = std::min((max-cost)/max, 1.0f);
  if(ratio < 0)
    ratio=0;

  foot_->setColor(1-ratio, 0.0, ratio, 1);
}

void StepVisual::setVisualizeValid(bool visualize)
{
  this->visualize_valid = visualize;
  visualizeValid(current_step.valid && !current_step.colliding);
}
// Interactive Markers -----------------------------------------------------------------
void StepVisual::initializeInteractiveMarker(InteractiveMarkerServer* marker_server)
{
  interactive_marker_server_ = marker_server;

  MenuHandler::EntryHandle interaction_sub = menu_handler.insert( "Interaction"); // case 1
  menu_handler.insert( "Disable Interaction",  boost::bind(&StepVisual::processMenuFeedback, this, _1)); // case 2
  menu_handler.setVisible(menu_handler.insert("Delete", boost::bind(&StepVisual::processMenuFeedback, this, _1)), false); // case 3
  menu_handler.insert("Set as last step", boost::bind(&StepVisual::processMenuFeedback, this, _1)); // case 4
  menu_handler.insert("Replan from here", boost::bind(&StepVisual::processMenuFeedback, this, _1)); // case 5
  menu_handler.insert("Replan to here", boost::bind(&StepVisual::processMenuFeedback, this, _1)); // case 6
  menu_handler.setCheckState(menu_handler.insert("Update 3D when moved", boost::bind(&StepVisual::processMenuFeedback, this, _1)),
                             (snap_to_valid ? MenuHandler::CHECKED : MenuHandler::UNCHECKED)); // case 7
  // Interaction Sub menu:
  mode_begin = menu_handler.insert(interaction_sub, "None", boost::bind(&StepVisual::processSetInteractionMode, this, _1));
  menu_handler.insert(interaction_sub, "2D Plane", boost::bind(&StepVisual::processSetInteractionMode, this, _1));
  menu_handler.insert(interaction_sub, "6 DOF", boost::bind(&StepVisual::processSetInteractionMode, this, _1));
  menu_handler.insert(interaction_sub, "Full 6 DOF", boost::bind(&StepVisual::processSetInteractionMode, this, _1));

  setButtonInteractiveMarker();
}

// set interactive marker
void StepVisual::setButtonInteractiveMarker()
{
  if(!interactive_marker_server_)
    return;

  // remove other interactive marker of foot
  deleteInteractiveMarker();

  visualization_msgs::InteractiveMarker im = makeInteractiveMarker("button"+ QString::number(index).toStdString(),
                                                                       frame_id_, current_step.foot.pose, im_scale);

  addButtonControl(im, makeMarker());

  interactive_marker_server_->insert(im, boost::bind(&StepVisual::processButtonFeedback, this, _1));

  setCheckState(0); // mode=none
  menu_handler.apply( *interactive_marker_server_, im.name );
  interactive_marker_server_->applyChanges();
}

void StepVisual::setPlaneInteractiveMarker()
{
  deleteInteractiveMarker();

  visualization_msgs::InteractiveMarker im = makeInteractiveMarker("plane"+ QString::number(index).toStdString(),
                                                                       frame_id_, current_step.foot.pose, im_scale);

  addPlaneControl(im, makeMarker(), /*with_rotation*/true);

  interactive_marker_server_->insert(im, boost::bind(&StepVisual::processInteractionFeedback, this, _1));

  setCheckState(PLANE);

  menu_handler.apply( *interactive_marker_server_, im.name );
  interactive_marker_server_->applyChanges();
}
void StepVisual::setSixDOFInteractiveMarker()
{
  // Move on 2d plane with box marker
  // Move 6DOF with default controls
  deleteInteractiveMarker();

  InteractiveMarkerMsg im = makeInteractiveMarker("6DOF"+ QString::number(index).toStdString(),
                                                  frame_id_, current_step.foot.pose, im_scale);

  addSixDOFControl(im);
  addPlaneControl(im, makeMarker(), /*with_rotation*/false);

  interactive_marker_server_->insert(im, boost::bind(&StepVisual::processInteractionFeedback, this, _1));
  setCheckState(SIXDOF);
  menu_handler.apply( *interactive_marker_server_, im.name );
  interactive_marker_server_->applyChanges();
}
void StepVisual::setFullSixDOFInteractiveMarker()
{
  ROS_INFO("Hold shift to move in/out the camera plane");
  deleteInteractiveMarker();

  InteractiveMarkerMsg im = makeInteractiveMarker("6DOF_withMove3D" + QString::number(index).toStdString(),
                                                  frame_id_, current_step.foot.pose, im_scale);

  addSixDOFControl(im);
  addMoveSixDOFControl(im, makeMarker());

  interactive_marker_server_->insert(im, boost::bind(&StepVisual::processInteractionFeedback, this, _1));
  setCheckState(FULLSIXDOF);
  menu_handler.apply( *interactive_marker_server_, im.name );
  interactive_marker_server_->applyChanges();
}

// process feedback
void StepVisual::processButtonFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK)
  {
    // activate interaction
    Q_EMIT(selected(true));
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
void StepVisual::processInteractionFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
  {
    setPosition(getOgreVector(feedback->pose.position));
    setOrientation(getOgreQuaternion(feedback->pose.orientation));

    current_step.foot.pose.position = feedback->pose.position;
    current_step.foot.pose.orientation = feedback->pose.orientation;

    vigir_footstep_planning_msgs::EditStep edit;
    edit.header.stamp = ros::Time::now();
    edit.header.frame_id = current_step.header.frame_id;
    edit.step = current_step;
    edit.plan_mode = vigir_footstep_planning_msgs::EditStep::EDIT_MODE_FULL;

    Q_EMIT(stepChanged(edit));
  }
  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
  {
    if(snap_to_valid)
      Q_EMIT(updateStepsPos());
    else
      Q_EMIT(footDropped());
  }
}
void StepVisual::processMenuFeedback(const FeedbackConstPtr &feedback)
{
  switch (feedback->menu_entry_id)
  {
  case 2: // Disable Interaction
    Q_EMIT(selected(false));
    setButtonInteractiveMarker();
    break;
  case 3: // Delete
//    removeStep();
    break;
  case 4: // set as last step
    Q_EMIT(endStepPlanHere(index));
    break;
  case 5: // replan from here
    Q_EMIT(cutStepPlanHere(index));
    break;
  case 6: // replan to here
    Q_EMIT(replanToHere(index));
    break;
  case 7: // snap to Valid position
    MenuHandler::CheckState state;
    menu_handler.getCheckState(feedback->menu_entry_id, state);
    if(state == MenuHandler::CHECKED)
    {
      menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::UNCHECKED);
      snap_to_valid = false;
    }
    else
    {
      menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::CHECKED);
      snap_to_valid = true;
    }
    interactive_marker_server_->applyChanges();
    break;
  default:
    ROS_ERROR("Menu Entry not found");
  }
}
void StepVisual::processSetInteractionMode(const FeedbackConstPtr &feedback)
{
  MenuHandler::EntryHandle handle = feedback->menu_entry_id;
  MenuHandler::CheckState state;
  menu_handler.getCheckState(handle,state);

  if(state == MenuHandler::CHECKED)
    return; // nothing changes (one entry has to be checked, it is not unchecked by clicking on it)

  if(handle == mode_begin)
  {
    Q_EMIT(selected(false));
    setButtonInteractiveMarker();
  }
  else
    Q_EMIT(selected(true));

  if(handle == mode_begin  + PLANE)
    setPlaneInteractiveMarker();
  if(handle == mode_begin + SIXDOF)
    setSixDOFInteractiveMarker();
  if(handle == mode_begin + FULLSIXDOF)
    setFullSixDOFInteractiveMarker();
}

// helper functions to add controls & marker
visualization_msgs::Marker StepVisual::makeMarker()
{
  std_msgs::ColorRGBA color;
  color.r = foot_index == FootMsg::LEFT ? 1 : 0;
  color.g = foot_index == FootMsg::RIGHT ? 1 : 0;
  color.b = 0;
  color.a = 0.1;
  visualization_msgs::Marker marker = makeSphereMarker(im_scale, color);
  return marker;
}

// sub menu check state handling:
void StepVisual::setCheckState(int checked_index)
{
  menu_handler.setCheckState(mode_begin, MenuHandler::UNCHECKED);
  menu_handler.setCheckState(mode_begin + PLANE, MenuHandler::UNCHECKED);
  menu_handler.setCheckState(mode_begin + SIXDOF, MenuHandler::UNCHECKED);
  menu_handler.setCheckState(mode_begin + FULLSIXDOF, MenuHandler::UNCHECKED);

  // set Checked:
  menu_handler.setCheckState(mode_begin + checked_index, MenuHandler::CHECKED);
}
// set default interaction, which is activated when clicked on button
void StepVisual::setInteractionMode(InteractionMode interaction_mode)
{
  interaction_mode_ = interaction_mode;
}
void StepVisual::deleteInteractiveMarker()
{
  interactive_marker_server_->erase("button"+QString::number(index).toStdString());
  interactive_marker_server_->erase("plane"+ QString::number(index).toStdString());
  interactive_marker_server_->erase("6DOF"+QString::number(index).toStdString());
  interactive_marker_server_->erase("6DOF_withMove3D"+QString::number(index).toStdString());
  interactive_marker_server_->applyChanges();
}
// update position of interactive marker by making new interactive marker
void StepVisual::resetInteractiveMarker()
{
  if(interactive_marker_server_)
  {
    // check current interaction:
    MenuHandler::CheckState state;
    menu_handler.getCheckState(mode_begin, state);
    if(state == MenuHandler::CHECKED)
    {
      setButtonInteractiveMarker();
      return;
    }
    menu_handler.getCheckState(mode_begin + PLANE, state);
    if(state == MenuHandler::CHECKED)
    {
      setPlaneInteractiveMarker();
      return;
    }
    menu_handler.getCheckState(mode_begin + SIXDOF, state);
    if(state == MenuHandler::CHECKED)
    {
      setSixDOFInteractiveMarker();
      return;
    }
    menu_handler.getCheckState(mode_begin + FULLSIXDOF, state);
    if(state == MenuHandler::CHECKED)
    {
      setFullSixDOFInteractiveMarker();
      return;
    }
  }
}




} // end namespace vigir_footstep_planning_rviz_plugin



