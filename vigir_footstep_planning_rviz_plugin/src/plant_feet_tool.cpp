#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>


#include <rviz/viewport_mouse_event.h>

#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <vigir_footstep_planning_rviz_plugin/plant_feet_tool.h>

#include <interactive_markers/interactive_marker_server.h>

using namespace interactive_markers;

namespace vigir_footstep_planning_rviz_plugin
{

PlantFeetTool::PlantFeetTool()
  : moving_feet_node_( NULL )
  , current_feet_property_( NULL )
  , mode(BOTH)
  , interactive_marker_server_(NULL)
  , interaction3D(false)
{
}

// The destructor for a Tool subclass is
// only called when the tool is removed from the toolbar with the "-"
// button.
PlantFeetTool::~PlantFeetTool()
{
}

void PlantFeetTool::setInteractiveMarkerServer(interactive_markers::InteractiveMarkerServer* server)
{
  interactive_marker_server_ = server;
  // case 1:
  menu_handler.setCheckState(menu_handler.insert("enable 6DOF interaction", boost::bind(&PlantFeetTool::processMenuFeedback, this, _1)), MenuHandler::UNCHECKED);
  // case 2:
  menu_handler.insert("Set as start", boost::bind(&PlantFeetTool::processMenuFeedback, this, _1));
}


// onInitialize() is called by the superclass after scene_manager_ and
// context_ are set.  It should be called only once per instantiation.
// This is where most one-time initialization work should be done.
// onInitialize() is called during initial instantiation of the tool
// object.  At this point the tool has not been activated yet, so any
// scene objects created should be invisible or disconnected from the
// scene at this point.
void PlantFeetTool::onInitialize()
{
  goal_visuals_.rset_capacity(2); // Todo: for way points higher capacity
  display_dir_ = true;
  // Werte die mit Parametern ersetzt werden müssen:
  Ogre::Vector3 posLeft(-0.04,0.093,0); //Todo: wie setzt sich die 0.04 zusammen?
  Ogre::Vector3 posRight(-0.04,-0.093,0);
  Ogre::Vector3 scale(0.001,0.001,0.001);
  // -----------------------------------------------
  moving_feet_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  moving_right_ = new StepVisual(scene_manager_,moving_feet_node_, vigir_footstep_planning_msgs::Foot::RIGHT);
  moving_right_->createVisualAt(posRight, Ogre::Quaternion(1,0,0,0));
  moving_left_ = new StepVisual(scene_manager_, moving_feet_node_, vigir_footstep_planning_msgs::Foot::LEFT);
  moving_left_->createVisualAt(posLeft, Ogre::Quaternion(1,0,0,0));

  moving_right_->setColor(0.0f,1.0f,0.0f,1.0f);
  moving_left_->setColor(1.0f,0.0f,0.0f,1.0f);

  show_dir_ = new rviz::Arrow(scene_manager_, moving_feet_node_,0.2f,0.05f,0.15f,0.1f);
  show_dir_->setPosition(Ogre::Vector3(0.0f,0.0f,0.0f));
  show_dir_->setDirection(Ogre::Vector3(1.0f,0.0f,0.0f));
  show_dir_->setColor(0.8f,0.8f,0.8f,0.5f);

  moving_feet_node_->setVisible( false );
}


// activate() is called when the tool is started by the user, either
// by clicking on its button in the toolbar or by pressing its hotkey.
void PlantFeetTool::activate()
{
  if(interactive_marker_server_ && mode == BOTH)
  {
    interactive_marker_server_->erase("feet_im");
    interactive_marker_server_->applyChanges();
    interaction3D = false;
  }
  if( moving_feet_node_ )
  {
    updateMovingFeetVisibility();
    current_feet_property_ = new rviz::VectorProperty( "Goal");
    current_feet_property_->setReadOnly( true );
  }
}

// update moving feet visibility according to mode
void PlantFeetTool::updateMovingFeetVisibility()
{
  if( moving_feet_node_ )
  {
    moving_feet_node_->setVisible( true );
    switch(mode)
    {
    case LEFT:
      moving_right_->setVisible(false);
      moving_left_->setVisible(true);
      moving_left_->setPosition(Ogre::Vector3(0.0f,0.0f,0.0f));
      break;
    case RIGHT:
      moving_left_->setVisible(false);
      moving_right_->setVisible(true);
      moving_right_->setPosition(Ogre::Vector3(0.0f,0.0f,0.0f));
      break;
    case BOTH:
      Ogre::Vector3 posLeft(-0.04,0.093,0); //Todo: wie setzt sich die 0.04 zusammen?
      Ogre::Vector3 posRight(-0.04,-0.093,0);
      moving_left_->setVisible(true);
      moving_right_->setVisible(true);
      moving_left_->setPosition(posLeft);
      moving_right_->setPosition(posRight);
      break;
    }
  }
}


// deactivate() is called when the tool is being turned off because
// another tool has been chosen.
void PlantFeetTool::deactivate()
{
  if( moving_feet_node_ )
  {
    moving_feet_node_->setVisible( false );
  }
  delete current_feet_property_;
  current_feet_property_ = NULL;
  mode = BOTH;
}

int PlantFeetTool::processMouseEvent( rviz::ViewportMouseEvent& event )
{
  if( !moving_feet_node_  || interaction3D )
  {
    return Render;
  }

   Ogre::Vector3 intersection;
   Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
   if( rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                          ground_plane,
                                          event.x, event.y, intersection ))
   {
     moving_feet_node_->setPosition( intersection );
     current_feet_property_->setVector( intersection );

     if( event.leftDown() )
     {
       Q_EMIT(feetDropped(intersection, moving_feet_node_->getOrientation(), mode));
       current_feet_property_ = NULL; // Drop the reference so that deactivate() won't remove it.
       mode = BOTH; //back to default mode (for feet interaction)
       return Render | Finished;
     }
  }
  else
  {
    moving_feet_node_->setVisible( false );
  }
  return Render;
}

int PlantFeetTool::processKeyEvent(QKeyEvent *event, rviz::RenderPanel *panel)
{
  if(event->key()==Qt::Key_A)
  {
    moving_feet_node_->rotate(Ogre::Vector3(0,0,1), Ogre::Radian(0.2));
  }
  if(event->key() == Qt::Key_D)
  {
     moving_feet_node_->rotate(Ogre::Vector3(0,0,1), Ogre::Radian(-0.2));
  }

  if(event->key()==Qt::Key_X)
  {
    moving_feet_node_->setVisible( false );
  }
  return Render;
}

void PlantFeetTool::addGoalFeet(vigir_footstep_planning_msgs::Feet goal)
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if ( !context_->getFrameManager()->getTransform( goal.header.frame_id,
                                                   goal.header.stamp,
                                                   position, orientation))
  {
    ROS_INFO( "Error transforming from frame '%s' to frame '%s'",
               goal.header.frame_id.c_str(), context_->getFrameManager()->getFixedFrame().c_str());
    return;
  }
  addGoalFeet(goal, position, orientation);
}

void PlantFeetTool::addGoalFeet(vigir_footstep_planning_msgs::Feet goal, Ogre::Vector3 frame_position, Ogre::Quaternion frame_orientation)
{
  goal_visuals_.clear();
  boost::shared_ptr<StepVisual> right_visual;
  right_visual.reset(new StepVisual(scene_manager_, scene_manager_->getRootSceneNode(), vigir_footstep_planning_msgs::Foot::RIGHT ));
  right_visual->createVisualAt( goal.right.pose.position , goal.right.pose.orientation);
  right_visual->setFramePosition(frame_position );
  right_visual->setFrameOrientation(frame_orientation );
  right_visual->setVisible(true);
  goal_visuals_.push_back(right_visual);

  boost::shared_ptr<StepVisual> left_visual;
  left_visual.reset(new StepVisual( scene_manager_,  scene_manager_->getRootSceneNode(), vigir_footstep_planning_msgs::Foot::LEFT ));
  left_visual->createVisualAt( goal.left.pose.position, goal.left.pose.orientation );
  left_visual->setFramePosition( frame_position );
  left_visual->setFrameOrientation( frame_orientation );
  left_visual->setVisible(true);
  goal_visuals_.push_back(left_visual);
  // make interactive marker

  if(interactive_marker_server_)
  {
    if(interaction3D)
    {
      enableSixDOFInteraction();
    }
    else
    {
      //enableButtonInteraction();
      enablePlaneInteraction();
    }
  }
}

void PlantFeetTool::processButtonFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK)
  {
    goal_visuals_.clear();
    Q_EMIT(activateTool(true));
  }
}

void PlantFeetTool::processMenuFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
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
      interaction3D = true;
      enableSixDOFInteraction();
    }
    else
    {
      interaction3D = false;
      enableButtonInteraction();
      menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
    }
    break;
  case 2: // select as start
    Q_EMIT(newStartPose(moving_feet_node_->getPosition(), moving_feet_node_->getOrientation()));
    break;
  default:
    ROS_WARN("Menu Entry not found");
  }
}

void PlantFeetTool::setMode(PlantFeetMode mode)
{
  this->mode = mode;
}

/*
void PlantFeetTool::save( rviz::Config config ) const
{
config.mapSetValue( "Class", getClassId() );
}

void PlantFeetTool::load( const rviz::Config& config )
{

}
*/

void PlantFeetTool::reset()
{
  goal_visuals_.clear();
  if(interactive_marker_server_)
  {
    interactive_marker_server_->erase("feet_im");
    interactive_marker_server_->applyChanges();
  }
  if(moving_feet_node_)
    moving_feet_node_->setVisible(false);

}

MarkerMsg PlantFeetTool::makeMarker()
{
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::SPHERE;
  box_marker.scale.x = 0.45;
  box_marker.scale.y = 0.5;
  box_marker.scale.z = 0.2;
  box_marker.color.r = 1;
  box_marker.color.g = 1;
  box_marker.color.b = 1;
  box_marker.color.a = 0;
  return box_marker;
}

InteractiveMarkerMsg PlantFeetTool::makeInteractiveMarker()
{
  interactive_marker_server_->erase("feet_im"); // erase old marker if exists
  visualization_msgs::InteractiveMarker im;
  im.header.frame_id = context_->getFrameManager()->getFixedFrame();
  im.header.stamp = ros::Time::now();
  im.name = "feet_im";
  Ogre::Vector3 pos = moving_feet_node_->getPosition();
  im.pose.position.x = pos.x;
  im.pose.position.y = pos.y;
  im.pose.position.z = pos.z;
  Ogre::Quaternion orient = moving_feet_node_->getOrientation();
  im.pose.orientation.w = orient.w;
  im.pose.orientation.x = orient.x;
  im.pose.orientation.y = orient.y;
  im.pose.orientation.z = orient.z;
  im.scale = 0.6;
  return im;
}

void PlantFeetTool::addSixDOFControl(InteractiveMarkerMsg& im)
{
  InteractiveMarkerControlMsg control;
  // x direction
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
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
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControlMsg::ROTATE_AXIS;
  im.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControlMsg::MOVE_AXIS;
  im.controls.push_back(control);
}

void PlantFeetTool::addMoveControl(InteractiveMarkerMsg& im)
{
  // Move 3D with box marker
  InteractiveMarkerControlMsg control;
  control.name = "move_3d";
  MarkerMsg marker = makeMarker();
  control.markers.push_back(marker);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
  control.always_visible = true;
  im.controls.push_back(control);
}

void PlantFeetTool::addPlaneControl(InteractiveMarkerMsg& im)
{
  visualization_msgs::InteractiveMarkerControl plane_control;
  plane_control.orientation.w = 1;
  plane_control.orientation.x = 0;
  plane_control.orientation.y = 1;
  plane_control.orientation.z = 0;
  // Rotate
  plane_control.name = "rotate_xy";
  plane_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  im.controls.push_back(plane_control);


  // Move Plane with box marker
  visualization_msgs::Marker box_marker = makeMarker();
  plane_control.markers.push_back( box_marker);
  plane_control.name = "move_xy";
  plane_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  plane_control.always_visible = true;
  im.controls.push_back(plane_control);
}

void PlantFeetTool::processInteractionFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  Ogre::Vector3 position(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
  Ogre::Quaternion orientation(feedback->pose.orientation.w, feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z);

  if(feedback->event_type == InteractiveMarkerFeedbackMsg::MOUSE_DOWN)
  {
    updateMovingFeetVisibility();
    current_feet_property_ = new rviz::VectorProperty( "Goal");
    current_feet_property_->setReadOnly( true );
  }
  if(feedback->event_type == InteractiveMarkerFeedbackMsg::POSE_UPDATE)
  {
    moving_feet_node_->setPosition( position);
    moving_feet_node_->setOrientation( orientation );
   // current_feet_property_->setVector( position ); //todo
  }
  if(feedback->event_type == InteractiveMarkerFeedbackMsg::MOUSE_UP)
  {
    //place feet
    moving_feet_node_->setVisible(false);
    Q_EMIT(feetDropped(position, orientation, BOTH));
  }

}
void PlantFeetTool::enableButtonInteraction()
{
  InteractiveMarkerMsg im = makeInteractiveMarker();

  MarkerMsg marker = makeMarker();

  InteractiveMarkerControlMsg control;
  control.interaction_mode = InteractiveMarkerControlMsg::BUTTON;
  control.markers.push_back( marker);
  control.always_visible = true;

  im.controls.push_back(control);

  interactive_marker_server_->insert(im, boost::bind(&PlantFeetTool::processButtonFeedback, this, _1));
  menu_handler.apply( *interactive_marker_server_, im.name );
  interactive_marker_server_->applyChanges();
}

void PlantFeetTool::enablePlaneInteraction()
{
  InteractiveMarkerMsg im = makeInteractiveMarker();
  addPlaneControl(im);
  interactive_marker_server_->insert(im, boost::bind(&PlantFeetTool::processInteractionFeedback, this, _1));
  menu_handler.apply(*interactive_marker_server_, im.name);
  interactive_marker_server_->applyChanges();
}

void PlantFeetTool::enableSixDOFInteraction()
{
  InteractiveMarkerMsg im = makeInteractiveMarker();
  addMoveControl(im);
  addSixDOFControl(im);

  interactive_marker_server_->insert(im, boost::bind(&PlantFeetTool::processInteractionFeedback, this, _1));
  menu_handler.apply( *interactive_marker_server_, im.name );
  interactive_marker_server_->applyChanges();
}



} // end namespace rviz_plugin_tutorials

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning_rviz_plugin::PlantFeetTool,rviz::Tool )
// END_TUTORIAL
