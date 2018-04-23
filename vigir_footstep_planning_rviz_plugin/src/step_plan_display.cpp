#include <vigir_footstep_planning_rviz_plugin/step_plan_display.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OgreVector3.h>
#include <tf/transform_listener.h>

#include <rviz/display_context.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>

#include <rviz/frame_manager.h>
#include <rviz/tool_manager.h>
#include <rviz/default_plugin/tools/interaction_tool.h>
#include <rviz/default_plugin/interactive_marker_display.h>
#include <rviz/display_group.h>

#include <vigir_footstep_planning_rviz_plugin/foot_visual.h>
#include <vigir_footstep_planning_rviz_plugin/footstep_planning_panel.h>
#include <vigir_footstep_planning_rviz_plugin/step_plan_helper.h>

#include <interactive_markers/interactive_marker_server.h>

#include <QObject>

namespace vigir_footstep_planning_rviz_plugin
{
StepPlanDisplay::StepPlanDisplay()
  : step_plan_helper_(new StepPlanHelper())
  , displayFeedback(false)
  , interaction_mode_(PLANE)
  , last_step_index(0)
{

  qRegisterMetaType<vigir_footstep_planning_msgs::Step>("vigir_footstep_planning_msgs::Step");

  frame_id_property_ = new rviz::StringProperty( "Frame ID", "",
                                               "Frame ID",
                                                this, SLOT( updateFrameID() ));
  ac_connected_ = new rviz::StatusProperty("Action Client",
                                           "Action Client of type [vigir_footstep_planning_msgs::StepPlanRequestAction] could not connect to Server.",
                                           (rviz::StatusProperty::Level) 2, //rviz::StatusProperty::Level::Error,
                                           this);
  display_index_ = new rviz::BoolProperty("Display Indices",
                                          false,
                                          "Display Indices of Step Plan",
                                          this, SLOT(displayIndex()));
  display_feedback_ = new rviz::BoolProperty("Display Feedback",
                                          false,
                                          "Displays current solution for step plan, while computing final step plan",
                                          this, SLOT(setDisplayFeedback()));
  display_start_ = new rviz::BoolProperty("Display Start",
                                          false,
                                          "Display current robot pose",
                                          this, SLOT(setStartVisible()));
  visualize_valid_ = new rviz::BoolProperty("Visualize invalid steps",
                                            true,
                                            "Marks invalid steps gray",
                                            0, 0); // todo!
  visualize_cost_ = new rviz::BoolProperty("Visualize step cost",
                                           false,
                                           "Visualizes step cost, where red indicates high and blue low cost.",
                                           this, SLOT(visualizeStepCost()));
}

void StepPlanDisplay::onInitialize()
{
  setIcon(QIcon("./vigir/vigir_footstep_planning/vigir_footstep_planning_visual_tools/vigir_footstep_planning_rviz_plugin/media/bothFeet.png"));
  step_visuals_.rset_capacity(100);
  start_visuals_.rset_capacity(2);
  panel_= new FootstepPlanningPanel();
  tool_manager_ = context_->getToolManager();

  feet_tool_ = static_cast<PlantFeetTool*>(tool_manager_->addTool("footstep_planning_rviz_plugin/PlantFeet"));
  step_plan_helper_->setFixedFrame(fixed_frame_);
  interact_tool_ = tool_manager_->addTool("rviz/Interact");

  interactive_marker_server_ = new interactive_markers::InteractiveMarkerServer("foot_marker");
  feet_tool_->setInteractiveMarkerServer(interactive_marker_server_);
  rviz::DisplayGroup* display_group = context_->getRootDisplayGroup();
  interactive_marker_display_ = (rviz::InteractiveMarkerDisplay*) display_group->createDisplay("rviz/InteractiveMarkers");
  interactive_marker_display_->initialize(context_);
  interactive_marker_display_->subProp("Update Topic")->setValue("/foot_marker/update");
  display_group->addChild(interactive_marker_display_);

  interactive_marker_display_->setName("Step Plan Interaction");
  bool connected_edit_step = step_plan_helper_->checkConnection();
  interactive_marker_display_->setEnabled(connected_edit_step);

  this->setAssociatedWidget (panel_);
  // on step plan created
  connect( panel_, SIGNAL( createdStepPlan( vigir_footstep_planning_msgs::StepPlan ) ), this, SLOT( displayStepPlan( vigir_footstep_planning_msgs::StepPlan ) ));
  connect( panel_ , SIGNAL( createdStepPlan( vigir_footstep_planning_msgs::StepPlan ) ), step_plan_helper_, SLOT( setCurrentStepPlan( vigir_footstep_planning_msgs::StepPlan ) ));
  connect( step_plan_helper_, SIGNAL( createdStepPlan( vigir_footstep_planning_msgs::StepPlan ) ), this, SLOT( displayStepPlan( vigir_footstep_planning_msgs::StepPlan ) ));
  connect( step_plan_helper_, SIGNAL(createdStepPlan(vigir_footstep_planning_msgs::StepPlan)), panel_, SLOT( setStepPlan(vigir_footstep_planning_msgs::StepPlan)));

  connect (step_plan_helper_, SIGNAL(stepValidUpdate(unsigned int, bool )), this, SLOT(setStepValid(unsigned int, bool )));
  // Feet Tool, Set Goal
  connect(panel_,  SIGNAL( feetToolActivated(bool)), this, SLOT(activateFeetTool(bool)) );
  connect(panel_,  SIGNAL( feetToolActivated(PlantFeetMode, bool)), this, SLOT(activateFeetTool(PlantFeetMode, bool)) );
  connect(panel_, SIGNAL(goalFeetAnswer(vigir_footstep_planning_msgs::Feet)), feet_tool_, SLOT(addGoalFeet(vigir_footstep_planning_msgs::Feet)));
  connect( feet_tool_, SIGNAL(feetDropped(Ogre::Vector3, Ogre::Quaternion, PlantFeetMode)),
           this, SLOT(handleFeetPose(Ogre::Vector3, Ogre::Quaternion, PlantFeetMode)));
  connect(feet_tool_, SIGNAL(activateTool(bool)), this, SLOT(activateFeetTool(bool)));
  connect(feet_tool_, SIGNAL(newStartPose(Ogre::Vector3, Ogre::Quaternion)), step_plan_helper_, SLOT(setRobotPose(Ogre::Vector3, Ogre::Quaternion)));
  // Get Feedback
  connect(this, SIGNAL(feedbackRequestedChanged(bool)), panel_, SLOT(setFeedbackRequested(bool)));
  connect(panel_, SIGNAL(sendPlanningFeedback(vigir_footstep_planning_msgs::PlanningFeedback)), this, SLOT(visualizeFeedback(vigir_footstep_planning_msgs::PlanningFeedback)));
  // Display Options
  connect( panel_, SIGNAL(displayRangeChanged(int,int)), this, SLOT(updateDisplay(int, int)));
  // Clear Scene
  connect(panel_, SIGNAL(clearScene()), this, SLOT(reset()));
//  connect(panel_, SIGNAL(clearScene()), feet_tool_, SLOT(reset()));
  connect(panel_, SIGNAL(undo()), step_plan_helper_, SLOT(setPreviousStepPlan()));
  connect(panel_, SIGNAL(acceptModifiedStepPlan()), step_plan_helper_, SLOT(acceptModifiedStepPlan()));
  connect(this, SIGNAL(requestStartPose()), panel_, SLOT(startPoseRequested()));
  connect(panel_, SIGNAL(startFeetAnswer(vigir_footstep_planning_msgs::Feet)), this, SLOT(addStartFeet(vigir_footstep_planning_msgs::Feet)));
  connect(panel_, SIGNAL(interactionModeChanged(int)), this, SLOT(setInteractionMode(int)));

  connect(panel_, SIGNAL(executeRequested()), step_plan_helper_, SLOT(executeStepPlan()));

  //Solve duplicate FootTool Problem:
  connect(tool_manager_, SIGNAL(toolAdded(Tool*)), this, SLOT(checkTool(Tool*)));
}

void StepPlanDisplay::update(float wall_dt, float ros_dt)
{
  updateACConnected();
}

void StepPlanDisplay::fixedFrameChanged()
{
  step_plan_helper_->setFixedFrame(fixed_frame_);
}


StepPlanDisplay::~StepPlanDisplay()
{
  delete interactive_marker_server_;
  delete panel_;
  delete step_plan_helper_;
}

void StepPlanDisplay::updateFrameID()
{
  panel_->setFrameID(frame_id_property_->getString());
}

void StepPlanDisplay::displayIndex()
{
  for (unsigned int i = 0; i < step_visuals_.size(); ++i)
  {
    step_visuals_[i]->displayIndex(display_index_->getBool());
  }
}

// Clear the visuals by deleting their objects.
void StepPlanDisplay::reset()
{
  step_visuals_.clear();
  start_visuals_.clear();
}


void StepPlanDisplay::displayStepPlan(const vigir_footstep_planning_msgs::StepPlan& step_plan)
{
  last_step_index = step_plan.steps.size();

  Ogre::Quaternion orientation;
  Ogre::Vector3 position;

  if(transformToFixedFrame(position, orientation, step_plan.header))
  {
    displayFeedback = false;
    step_visuals_.clear();
    addStartFeet(step_plan.start, position, orientation);
    displaySteps(step_plan.steps, position, orientation);
    setStartVisible();
    feet_tool_->reset(); // dont show goal
  }
}

void StepPlanDisplay::activateFeetTool(bool active)
{
  if ( active )
  {
    tool_manager_->setCurrentTool(feet_tool_);
  }
  else
  {
   tool_manager_->setCurrentTool(interact_tool_);
  }
}

void StepPlanDisplay::activateFeetTool(PlantFeetMode mode, bool active)
{
  feet_tool_->setMode(mode);
  activateFeetTool(active);
}

void StepPlanDisplay::addStartFeet(vigir_footstep_planning_msgs::Feet start)
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (! transformToFixedFrame(position, orientation, start.header))
    return; // failed

  addStartFeet(start, position, orientation);
}

void StepPlanDisplay::addStartFeet(const vigir_footstep_planning_msgs::Feet& start, const Ogre::Vector3& frame_position, const Ogre::Quaternion& frame_orientation)
{
  start_visuals_.clear();
  boost::shared_ptr<StepVisual> right_visual;
  right_visual.reset(new StepVisual(scene_manager_, scene_node_, vigir_footstep_planning_msgs::Foot::RIGHT, frame_id_property_->getString().toStdString()));
  right_visual->createVisualAt( start.right.pose.position , start.right.pose.orientation);
  right_visual->setFramePosition(frame_position );
  right_visual->setFrameOrientation(frame_orientation );
  start_visuals_.push_back(right_visual);

  boost::shared_ptr<StepVisual> left_visual;
  left_visual.reset(new StepVisual( scene_manager_, scene_node_, vigir_footstep_planning_msgs::Foot::LEFT, frame_id_property_->getString().toStdString() ));
  left_visual->createVisualAt( start.left.pose.position, start.left.pose.orientation );
  left_visual->setFramePosition( frame_position );
  left_visual->setFrameOrientation( frame_orientation );
  start_visuals_.push_back(left_visual);

  setStartVisible();
}

void StepPlanDisplay::setStartVisible()
{
  if(start_visuals_.size() == 2){
    start_visuals_[0]->setVisible(display_start_->getBool());
    start_visuals_[1]->setVisible(display_start_->getBool());
  }
  else
    Q_EMIT(requestStartPose());
}

void StepPlanDisplay::updateDisplay(int from, int to)
{
  for(int i=0; i < from; i++ )
  {
    step_visuals_[i]->setVisible(false);
  }
  for (int i=from; i < step_visuals_.size(); i++)
  {
    step_visuals_[i]->setVisible(true);
  }
  for(int i=to+1; i < step_visuals_.size(); i++ )
  {
    step_visuals_[i]->setVisible(false);
  }
}

void StepPlanDisplay::displaySteps(const std::vector<vigir_footstep_planning_msgs::Step>& steps, const Ogre::Vector3& frame_position, const Ogre::Quaternion& frame_orientation)
{
  bool displIndex = display_index_->getBool();  
  for (unsigned i = 0; i< steps.size(); i++)
  {
    boost::shared_ptr<StepVisual> visual;
    visual.reset(new StepVisual( scene_manager_, scene_node_, steps[i].foot.foot_index , steps[i].header.frame_id, steps[i].step_index));
    visual->createByMessage( steps[i] );
    visual->setFramePosition( frame_position );
    visual->setFrameOrientation( frame_orientation );
    visual->displayIndex(displIndex);

    //Interactive Marker:
    if(!displayFeedback)
    {
      visual->initializeInteractiveMarker(interactive_marker_server_);
      visual->setInteractionMode(interaction_mode_);
      connect(visual.get(), SIGNAL(stepEdited(vigir_footstep_planning_msgs::EditStep)), step_plan_helper_, SLOT(editStep(vigir_footstep_planning_msgs::EditStep)));
      connect(panel_, SIGNAL(clearIM()), visual.get(), SLOT(setButtonInteractiveMarker()));
      connect(visual.get(), SIGNAL(cutStepPlanHere(int)), this, SLOT(visualizeCut(int)));
      connect(visual.get(), SIGNAL(cutStepPlanHere(int)),panel_, SLOT(setLastStep(int)) );
      connect(visual.get(), SIGNAL(replanToHere(int)), panel_, SLOT(replanToIndex(int)));
      connect(visual.get(), SIGNAL(replanToHere(int)), this, SLOT(visualizeReplan(int)));
    }
    step_visuals_.push_back(visual);
  }
}



void StepPlanDisplay::visualizeFeedback(vigir_footstep_planning_msgs::PlanningFeedback feedback)
{
  if(display_feedback_->getBool())
  {
    displayFeedback = true;
    displayStepPlan(feedback.current_step_plan);
  }
}

void StepPlanDisplay::setDisplayFeedback()
{
  Q_EMIT(feedbackRequestedChanged(display_feedback_->getBool()));
}

void StepPlanDisplay::updateACConnected()
{
  bool connected_step_plan = panel_->checkConnection();
  bool connected_edit_step = step_plan_helper_->checkConnection();

  if(connected_step_plan && connected_edit_step)
  {
    ac_connected_->setLevel((rviz::StatusProperty::Level) 0); // todo: rviz::StatusProperty::Level::OK
    ac_connected_->setValue("Action Client is connected to Server.");
  }
  else if(connected_step_plan) // only stepPlanRequest possible
  {
    ac_connected_->setLevel((rviz::StatusProperty::Level) 1); // todo: rviz::StatusProperty::Level::WARN
    ac_connected_->setValue("Action Client for StepPlanRequest Action is connected. EditStep Action not!");
  }
  else
  {
    ac_connected_->setLevel((rviz::StatusProperty::Level) 2); // todo: rviz::StatusProperty::Level::ERROR
    ac_connected_->setValue("Action Client for StepPlanRequest is not connected!");
  }
}

// called after constructor
void StepPlanDisplay::load( const rviz::Config& config )
{
  rviz::Display::load(config);
  panel_->load(config);
}

void StepPlanDisplay::save( rviz::Config config ) const
{
  rviz::Display::save(config);
  panel_->save(config);
}

// Prevent Problem when saved and started:
void StepPlanDisplay::checkTool(rviz::Tool* tool)
{
//  ROS_INFO("%s", tool->getName().toStdString().c_str());
  if(tool->getName()=="Plant Feet")
  {

    feet_tool_=static_cast<PlantFeetTool*>(tool);
    feet_tool_->setInteractiveMarkerServer(interactive_marker_server_);
    connect( feet_tool_, SIGNAL(feetDropped(Ogre::Vector3, Ogre::Quaternion, PlantFeetMode)),
             this, SLOT(handleFeetPose(Ogre::Vector3, Ogre::Quaternion, PlantFeetMode)));
    connect(panel_, SIGNAL(goalFeetAnswer(vigir_footstep_planning_msgs::Feet)), feet_tool_, SLOT(addGoalFeet(vigir_footstep_planning_msgs::Feet)));
    connect(feet_tool_, SIGNAL(activateTool(bool)), this, SLOT(activateFeetTool(bool)));
    connect(feet_tool_, SIGNAL(newStartPose(Ogre::Vector3, Ogre::Quaternion)), step_plan_helper_, SLOT(setRobotPose(Ogre::Vector3, Ogre::Quaternion)));
    connect(panel_, SIGNAL(clearScene()), feet_tool_, SLOT(reset()));
  }
  if(tool->getName() == "Interact")
  {
    interact_tool_ = tool;
  }
}

void StepPlanDisplay::setStepValid(unsigned int index, bool valid)
{
  //if(visualize_valid_->getBool()) todo
    step_visuals_[index]->setValid(valid);
}

void StepPlanDisplay::visualizeCut(int new_end)
{
  last_step_index = new_end;
  for(int i = 0; i <= new_end; i++)
  {
    step_visuals_[i]->setValid(true);
  }

  for(int i = new_end+1; i < step_visuals_.size(); ++i)
  {
    step_visuals_[i]->setValid(false);
  }
}

void StepPlanDisplay::visualizeReplan(int end)
{
  for (int i = end+1; i < step_visuals_.size(); ++i)
  {
    step_visuals_[i]->setValid(true);
  }
  if(last_step_index > end) // replan from index 0 to end
  {
    for(int i = 0; i <= end; ++i)
    {
      step_visuals_[i]->setValid(false);
    }
  }
}


bool StepPlanDisplay::transformToFixedFrame(Ogre::Vector3& position, Ogre::Quaternion& orientation, const std_msgs::Header& header)
{
  if ( !context_->getFrameManager()->getTransform( header.frame_id,
                                                   header.stamp,
                                                   position, orientation))
  {
    ROS_INFO( "Error transforming from frame '%s' to frame '%s'",
               header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    return false;
  }
  return true;
}

void StepPlanDisplay::handleFeetPose(Ogre::Vector3 position, Ogre::Quaternion orientation, PlantFeetMode mode)
{
  panel_->releasePlaceFeet();
  boost::shared_ptr<StepVisual> visual;
  switch(mode)
  {
  case BOTH:
    panel_->handleGoalPose(position, orientation);
    return;
  case LEFT:
    visual.reset(new StepVisual( scene_manager_, scene_node_,vigir_footstep_planning_msgs::Foot::LEFT,frame_id_property_->getString().toStdString(), step_visuals_.size()));
    step_plan_helper_->addStep(frame_id_property_->getString().toStdString(), position, orientation,vigir_footstep_planning_msgs::Foot::LEFT, step_visuals_.size());
    break;
  case RIGHT:
    visual.reset(new StepVisual( scene_manager_, scene_node_, vigir_footstep_planning_msgs::Foot::RIGHT, frame_id_property_->getString().toStdString(), step_visuals_.size()));
    step_plan_helper_->addStep(frame_id_property_->getString().toStdString(), position, orientation,vigir_footstep_planning_msgs::Foot::RIGHT, step_visuals_.size());
    break;
  }

  if(mode == LEFT || mode == RIGHT)
  {
    visual->displayIndex( display_index_->getBool());
    visual->createVisualAt(position, orientation);
    visual->initializeInteractiveMarker(interactive_marker_server_);
    connect(visual.get(), SIGNAL(stepEdited(vigir_footstep_planning_msgs::EditStep)), step_plan_helper_, SLOT(editStep(vigir_footstep_planning_msgs::EditStep)));
    connect(panel_, SIGNAL(clearIM()), visual.get(), SLOT(setButtonInteractiveMarker()));

    // connect(visual.get(), SIGNAL(cutStepPlanHere(int)), this, SLOT(visualizeCut(int)));
  //  connect(visual.get(), SIGNAL(cutStepPlanHere(int)),panel_, SLOT(setLastStep(int)) );
    step_visuals_.push_back(visual);

  }


}
void StepPlanDisplay::setInteractionMode(int interaction_mode)
{
  if(interaction_mode <= 3 || interaction_mode >= 1){
    interaction_mode_ = static_cast<InteractionMode>(interaction_mode);
    for (unsigned int i = 0; i < step_visuals_.size(); ++i)
    {
      step_visuals_[i]->setInteractionMode(interaction_mode_);
    }
  }
  else
    ROS_WARN("Invalid interaction mode.");
}

void StepPlanDisplay::visualizeStepCost()
{
  if(visualize_cost_->getBool())
  {
    for(unsigned int i = 0; i < step_visuals_.size(); ++i)
    {
      step_visuals_[i]->visualizeCost(0.5);
    }
  }
}
}
 // end namespace vigir_footstep_planning_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning_rviz_plugin::StepPlanDisplay,rviz::Display )
