#include <vigir_footstep_planning_rviz_plugin/plant_feet_tool.h>
#include <vigir_footstep_planning_rviz_plugin/feet_visual.h>
#include <vigir_footstep_planning_rviz_plugin/common/ogre_visualization_msgs_functions.h>
#include <geometry_msgs/PoseStamped.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/geometry.h>
#include <rviz/display_context.h>


namespace vigir_footstep_planning_rviz_plugin
{

PlantFeetTool::PlantFeetTool()
  : moving_feet_node_( 0 )
  , mode(GOAL_FEET)
  , moving_feet_(0)
  , generate_feet_ac("generate_feet_pose", true)
{
  robot_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/robot_pose",1);
}

PlantFeetTool::~PlantFeetTool()
{
  scene_manager_->destroySceneNode(moving_feet_node_);
}


void PlantFeetTool::onInitialize()
{

  generate_feet_ac.waitForServer(ros::Duration(1,0));

  moving_feet_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  moving_feet_ = new FeetVisual(scene_manager_, moving_feet_node_, GOAL);
  moving_feet_->createFeetAt(Ogre::Vector3(0,0,0), Ogre::Quaternion(1,0,0,0));
  moving_feet_node_->setVisible( false );
}


void PlantFeetTool::activate()
{
  if( moving_feet_node_ )
  {
    moving_feet_node_->setVisible( true );
  }
}

void PlantFeetTool::deactivate()
{
  if( moving_feet_node_ )
  {
    moving_feet_node_->setVisible( false );
  }
}

int PlantFeetTool::processMouseEvent( rviz::ViewportMouseEvent& event )
{
  if( !moving_feet_node_ )
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
    if( event.leftDown() )
    {
      setValidFeet(intersection, moving_feet_node_->getOrientation(), context_->getFixedFrame().toStdString(), this->mode == START_FEET ? START : GOAL);
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
  return Render;
}

void PlantFeetTool::setMode(PlantFeetMode mode)
{
  this->mode = mode;
}

void PlantFeetTool::setValidFeet(Ogre::Vector3 position, Ogre::Quaternion orientation, std::string frame_id, FeetType type)
{
  geometry_msgs::Pose pose;
  getPoseMsg(pose, position, orientation);

  if(generate_feet_ac.isServerConnected())
  {
    vigir_footstep_planning_msgs::GenerateFeetPoseGoal goal;
    goal.request.header.stamp = ros::Time::now();
    goal.request.header.frame_id = frame_id;
    goal.request.pose = pose;
    goal.request.flags = vigir_footstep_planning_msgs::FeetPoseRequest::FLAG_3D;
    if(type==GOAL)
      generate_feet_ac.sendGoal(goal,
                                boost::bind(&PlantFeetTool::setValidGoalCallback, this, _1, _2),
                                GenerateFeetPoseActionClient::SimpleActiveCallback(),
                                GenerateFeetPoseActionClient::SimpleFeedbackCallback());
    if(type==START)
      generate_feet_ac.sendGoal(goal,
                                boost::bind(&PlantFeetTool::setValidStartCallback, this, _1, _2),
                                GenerateFeetPoseActionClient::SimpleActiveCallback(),
                                GenerateFeetPoseActionClient::SimpleFeedbackCallback());
  }
}

void PlantFeetTool::setValidGoalCallback(const actionlib::SimpleClientGoalState& state, const GenerateFeetPoseResult& result)
{
  if(result->status.error == ErrorStatusMsg::NO_ERROR)
  {
    Q_EMIT(newGoalPose(result->feet));
  }
}

void PlantFeetTool::setValidStartCallback(const actionlib::SimpleClientGoalState& state, const GenerateFeetPoseResult& result)
{
  if(result->status.error == ErrorStatusMsg::NO_ERROR && state.isDone())
  {
    Q_EMIT(newStartPose(result->feet));

    double shift_z, shift_y, shift_x;
    if(!nh.getParam("foot/right/foot_frame/z", shift_z)
       || !nh.getParam("foot/right/foot_frame/x", shift_x))
    {
      shift_z=0;
      shift_x=0;
    }
    Ogre::Matrix3 rotMat;
    getOgreQuaternion(result->feet.left.pose.orientation).ToRotationMatrix(rotMat);
    Ogre::Vector3 shift = rotMat*Ogre::Vector3(shift_x,0,shift_z);
    Ogre::Vector3 mid_position = 0.5*(getOgreVector(result->feet.left.pose.position) + getOgreVector(result->feet.right.pose.position)) + shift;

    setRobotPose(mid_position, getOgreQuaternion(result->feet.left.pose.orientation), result->feet.header.frame_id);
  }
}

void PlantFeetTool::setRobotPose(const Ogre::Vector3& position, const Ogre::Quaternion& orientation, const std::string& frame_id)
{
  geometry_msgs::PoseStamped robot_pose;
  robot_pose.header.stamp = ros::Time::now();
  robot_pose.header.frame_id = frame_id;

  getPoseMsg(robot_pose.pose, position, orientation);

  if(ros::ok())
  {
    robot_pose_publisher.publish(robot_pose);
  }
}

} // end namespace vigir_footstep_planning_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning_rviz_plugin::PlantFeetTool,rviz::Tool )
