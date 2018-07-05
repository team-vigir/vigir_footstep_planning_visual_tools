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
#include <rviz/properties/vector_property.h>
#include <rviz/properties/quaternion_property.h>

#include <vigir_footstep_planning_rviz_plugin/step_visual.h>
#include <vigir_footstep_planning_rviz_plugin/footstep_planning_panel.h>
#include <vigir_footstep_planning_rviz_plugin/step_plan_helper.h>
#include <vigir_footstep_planning_rviz_plugin/feet_visual.h>

#include <interactive_markers/interactive_marker_server.h>

#include <QObject>

namespace vigir_footstep_planning_rviz_plugin
{
StepPlanDisplay::StepPlanDisplay()
  : displayFeedback(false)
  , interaction_mode_(PLANE)
  , last_step_index(0)
  , index_feet_tool(-1)
  , step_plan_helper_(0)
{
  qRegisterMetaType<vigir_footstep_planning_msgs::Step>("vigir_footstep_planning_msgs::Step");

  initializeDisplayProperties();
}

StepPlanDisplay::~StepPlanDisplay()
{
  if(index_feet_tool >-1)
  {
    tool_manager_->removeTool(index_feet_tool); // plant feet
  }
  step_visuals_.clear();
  feedback_visuals_.clear();
  delete interactive_marker_server_;
  delete panel_;
  if(step_plan_helper_)
    delete step_plan_helper_;
}

void StepPlanDisplay::onInitialize()
{
  // set icon:
  std::string icons_path;
  if(nh.getParam("icons_path", icons_path))
  setIcon(QIcon(QString::fromStdString(icons_path + "bothFeet.png")));
  step_visuals_.rset_capacity(100);
  feedback_visuals_.rset_capacity(100);

  panel_= new FootstepPlanningPanel();
  this->setAssociatedWidget (panel_);


  step_plan_helper_= new StepPlanHelper();

  // Status Prompt Connections:
  /*
  connect( step_plan_helper_, SIGNAL(actionClientConnected(QString, bool)), panel_->ui_->messageDisplay, SLOT( displayConnection(QString, bool)));
  connect( step_plan_helper_, SIGNAL(displayError(QString)), panel_->ui_->messageDisplay, SLOT( displayError(QString)));
  connect( step_plan_helper_, SIGNAL(displayInfo(QString)), panel_->ui_->messageDisplay, SLOT( displayMessage(QString)));
  connect( step_plan_helper_, SIGNAL(displaySuccess(QString)), panel_->ui_->messageDisplay, SLOT( displaySuccess(QString)));
*/
  // progress bar
  connect( step_plan_helper_, SIGNAL(executionStarted(int)), panel_, SLOT(initializeExecutionProgressbar(int)));
  connect( step_plan_helper_, SIGNAL(executionFeedback(int)), panel_, SLOT(updateProgressBar(int)));


  // ---------------------------

  step_plan_helper_->connectToActionServer();

  tool_manager_ = context_->getToolManager();

  index_feet_tool = tool_manager_->numTools();
  feet_tool_ = static_cast<PlantFeetTool*>(tool_manager_->addTool("footstep_planning_rviz_plugin/PlantFeet"));
  interact_tool_ = tool_manager_->addTool("rviz/Interact");

  interactive_marker_server_ = new interactive_markers::InteractiveMarkerServer("foot_marker");
  if(feet_tool_)
    feet_tool_->setInteractiveMarkerServer(interactive_marker_server_);

  rviz::DisplayGroup* display_group = context_->getRootDisplayGroup();
 // interactive_marker_display_ = (rviz::InteractiveMarkerDisplay*) display_group->createDisplay("rviz/InteractiveMarkers");
 // interactive_marker_display_->initialize(context_);
 // interactive_marker_display_->subProp("Update Topic")->setValue("foot_marker/update");
 // display_group->addChild(interactive_marker_display_);
 // interactive_marker_display_->setName("Step Plan Interaction");
  //bool connected_edit_step = step_plan_helper_ ? step_plan_helper_->checkConnection() : false;
 // interactive_marker_display_->setEnabled(connected_edit_step);

  makeConnections();

  //Solve duplicate FootTool Problem:
  connect(tool_manager_, SIGNAL(toolAdded(Tool*)), this, SLOT(checkTool(Tool*)));
}

void StepPlanDisplay::initializeDisplayProperties()
{
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
  update_step_plan_positions_ = new rviz::BoolProperty("Update 3D",
                                                      false,
                                                      "Updates 3D position when a new step plan is created or changed.",
                                                      this, SLOT(setUpdateStepPlan()));
  visualize_valid_ = new rviz::BoolProperty("Visualize invalid steps",
                                            true,
                                            "Marks invalid steps gray",
                                            0, 0); // todo!
  visualize_cost_ = new rviz::BoolProperty("Visualize step cost",
                                           false,
                                           "Visualizes step cost, where red indicates high and blue low cost.",
                                           this, SLOT(visualizeStepCost()));
  property_container_ = new rviz::Property("Properties",
                                        QVariant(),
                                        "List of properties of current step plan",
                                        this);
  goal_property_container_ = new rviz::BoolProperty("Goal Feet",
                                        false,
                                        "List of properties of current step plan",
                                        this);
  start_property_container_ = new rviz::BoolProperty("Start Feet",
                                           false,
                                           "List of properties of current step plan",
                                           this, SLOT(setStartVisible()));
}

void StepPlanDisplay::makeFeetToolConnections()
{
  if(step_plan_helper_)
    connect(feet_tool_, SIGNAL(newStartPose(Ogre::Vector3, Ogre::Quaternion)), step_plan_helper_, SLOT(setRobotPose(Ogre::Vector3, Ogre::Quaternion)));
  connect( feet_tool_, SIGNAL(feetDropped(Ogre::Vector3, Ogre::Quaternion, PlantFeetMode)),
           this, SLOT(handleFeetPose(Ogre::Vector3, Ogre::Quaternion, PlantFeetMode)));
  connect(panel_, SIGNAL(goalFeetAnswer(vigir_footstep_planning_msgs::Feet)), feet_tool_, SLOT(addGoalFeet(vigir_footstep_planning_msgs::Feet)));
  connect(panel_, SIGNAL(goalFeetAnswer(vigir_footstep_planning_msgs::Feet)), this, SLOT(addGoalFeetProperties(vigir_footstep_planning_msgs::Feet)));
  connect(feet_tool_, SIGNAL(activateTool(bool)), this, SLOT(activateFeetTool(bool)));
  connect(panel_, SIGNAL(clearAll()), feet_tool_, SLOT(reset()));
}

// todo: delete?
void StepPlanDisplay::disconnectFeetTool()
{
  disconnect( feet_tool_, SIGNAL(feetDropped(Ogre::Vector3, Ogre::Quaternion, PlantFeetMode)),
           this, SLOT(handleFeetPose(Ogre::Vector3, Ogre::Quaternion, PlantFeetMode)));
  disconnect(panel_, SIGNAL(goalFeetAnswer(vigir_footstep_planning_msgs::Feet)), feet_tool_, SLOT(addGoalFeet(vigir_footstep_planning_msgs::Feet)));
  disconnect(feet_tool_, SIGNAL(activateTool(bool)), this, SLOT(activateFeetTool(bool)));
  if(step_plan_helper_)
    disconnect(feet_tool_, SIGNAL(newStartPose(Ogre::Vector3, Ogre::Quaternion)), step_plan_helper_, SLOT(setRobotPose(Ogre::Vector3, Ogre::Quaternion)));
  disconnect(panel_, SIGNAL(clearAll()), feet_tool_, SLOT(reset()));
}

void StepPlanDisplay::makeConnections()
{
  // on step plan created
  connect( panel_, SIGNAL( createdStepPlan( vigir_footstep_planning_msgs::StepPlan ) ), this, SLOT( displayStepPlan( vigir_footstep_planning_msgs::StepPlan ) ));
  // Feet Tool, Set Goal
  connect(panel_,  SIGNAL( feetToolActivated(bool)), this, SLOT(activateFeetTool(bool)) );
  connect(panel_,  SIGNAL( feetToolActivated(PlantFeetMode, bool)), this, SLOT(activateFeetTool(PlantFeetMode, bool)) );

  // Get Feedback
  connect(this, SIGNAL(feedbackRequestedChanged(bool)), panel_, SLOT(setFeedbackRequested(bool)));
  connect(panel_, SIGNAL(sendPlanningFeedback(vigir_footstep_planning_msgs::PlanningFeedback)), this, SLOT(visualizeFeedback(vigir_footstep_planning_msgs::PlanningFeedback)));

  // Display Options
  connect( panel_, SIGNAL(displayRangeChanged(int,int)), this, SLOT(updateDisplay(int, int)));
  connect(panel_, SIGNAL(interactionModeChanged(int)), this, SLOT(setInteractionMode(int)));

  // Clear
  connect(panel_, SIGNAL(clearStepPlan()), step_plan_helper_, SLOT(resetStepPlan()));
  connect(panel_, SIGNAL(clearStepPlan()), this, SLOT(resetStepPlan()));

  // start pose
  connect(this, SIGNAL(requestStartPose()), panel_, SLOT(startPoseRequested()));
  connect(panel_, SIGNAL(startFeetAnswer(vigir_footstep_planning_msgs::Feet)), this, SLOT(addStartFeet(vigir_footstep_planning_msgs::Feet)));
  connect(panel_, SIGNAL(startFeetAnswer(vigir_footstep_planning_msgs::Feet)), this, SLOT(addStartFeetProperties(vigir_footstep_planning_msgs::Feet)));
  makeFeetToolConnections();

  if(step_plan_helper_)
    makeStepPlanHelperConnections();

}

void StepPlanDisplay::makeStepVisualConnections(const StepVisual* visual)
{
  if(step_plan_helper_)
  {
    connect(visual, SIGNAL(stepEdited(vigir_footstep_planning_msgs::EditStep)), step_plan_helper_, SLOT(editStep(vigir_footstep_planning_msgs::EditStep)));
    connect(visual, SIGNAL(updateStepsPos()), step_plan_helper_, SLOT(updateStepPlanPositions()));
    connect(visual, SIGNAL(footDropped()), step_plan_helper_, SLOT(acceptModifiedStepPlan()));
    connect(visual, SIGNAL(endStepPlanHere(int)), step_plan_helper_, SLOT(trimStepPlan(int)));
  }

  connect(panel_, SIGNAL(clearIM()), visual, SLOT(setButtonInteractiveMarker()));
  connect(visual, SIGNAL(cutStepPlanHere(int)), this, SLOT(visualizeCut(int)));
  connect(visual, SIGNAL(cutStepPlanHere(int)),panel_, SLOT(setLastStep(int)) );
  connect(visual, SIGNAL(replanToHere(int)), panel_, SLOT(replanToIndex(int)));
  connect(visual, SIGNAL(replanToHere(int)), this, SLOT(visualizeReplan(int)));

}

void StepPlanDisplay::makeStepPlanHelperConnections()
{
  connect( panel_ , SIGNAL( createdStepPlan( vigir_footstep_planning_msgs::StepPlan ) ), step_plan_helper_, SLOT( setCurrentStepPlan( vigir_footstep_planning_msgs::StepPlan ) ));
  connect( panel_ , SIGNAL( createdSequence(vigir_footstep_planning_msgs::StepPlan)), step_plan_helper_, SLOT( handleSequence(vigir_footstep_planning_msgs::StepPlan)));
  connect( step_plan_helper_, SIGNAL( createdStepPlan( vigir_footstep_planning_msgs::StepPlan ) ), this, SLOT( displayStepPlan( vigir_footstep_planning_msgs::StepPlan ) ));
  connect( step_plan_helper_, SIGNAL(createdStepPlan(vigir_footstep_planning_msgs::StepPlan)), panel_, SLOT( setStepPlan(vigir_footstep_planning_msgs::StepPlan)));

  connect( step_plan_helper_, SIGNAL(updatedStepPlan(vigir_footstep_planning_msgs::StepPlan)), panel_, SLOT( setStepPlan(vigir_footstep_planning_msgs::StepPlan)));
  connect( step_plan_helper_, SIGNAL(updatedStepPlan(vigir_footstep_planning_msgs::StepPlan)), this, SLOT(updateStepVisuals(vigir_footstep_planning_msgs::StepPlan)));

  // step_plan_helper functions
  connect(panel_, SIGNAL(undo()), step_plan_helper_, SLOT(setPreviousStepPlan()));
  connect (step_plan_helper_, SIGNAL(stepValidUpdate(unsigned int, bool )), this, SLOT(setStepValid(unsigned int, bool )));

  // execute
  connect(panel_, SIGNAL(executeRequested()), step_plan_helper_, SLOT(executeStepPlan()));
}

void StepPlanDisplay::update(float wall_dt, float ros_dt)
{
  updateACConnected();
}

void StepPlanDisplay::updateFrameID()
{
  panel_->setFrameID(frame_id_property_->getString());
  if(step_plan_helper_)
    step_plan_helper_->setFrameID(frame_id_property_->getString());
}

void StepPlanDisplay::displayIndex()
{
  for (unsigned int i = 0; i < step_visuals_.size(); ++i)
  {
    step_visuals_[i]->displayIndex(display_index_->getBool());
  }
}

// Clear the visuals by deleting their objects.
void StepPlanDisplay::resetStepPlan()
{
  property_container_->removeChildren();
  step_visuals_.clear();
}

void StepPlanDisplay::displayStepPlan(const vigir_footstep_planning_msgs::StepPlan& step_plan)
{
  last_step_index = step_plan.steps.size();

  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if(transformToFixedFrame(position, orientation, step_plan.header))
  {
    displayFeedback = false;
    feedback_visuals_.clear();
    step_visuals_.clear();
    addStartFeet(step_plan.start, position, orientation);
    displaySteps(step_plan.steps, position, orientation);
    setStartVisible();
    if(visualize_cost_->getBool())
      visualizeStepCost();
    if(feet_tool_)
      feet_tool_->reset(); // dont show goal
  }
}

void StepPlanDisplay::displaySteps(const std::vector<vigir_footstep_planning_msgs::Step>& steps, const Ogre::Vector3& frame_position, const Ogre::Quaternion& frame_orientation)
{
  property_container_->removeChildren();

  bool displIndex = display_index_->getBool();
  for (unsigned i = 0; i< steps.size(); i++)
  {
    StepProperty* step_property = new StepProperty(steps[i], property_container_);
    boost::shared_ptr<StepVisual> visual;
    visual.reset(new StepVisual( scene_manager_, scene_node_, steps[i].foot.foot_index, steps[i].header.frame_id, steps[i].step_index));

    visual->createByMessage( steps[i]);
    visual->setFramePosition( frame_position );
    visual->setFrameOrientation( frame_orientation );
    visual->displayIndex(displIndex);

    //Interactive Marker:
    if(!displayFeedback && i != 0)
    {
      if(i != 0)
      {
        visual->initializeInteractiveMarker(interactive_marker_server_);
        visual->setInteractionMode(interaction_mode_);
      }

      makeStepVisualConnections(visual.get());
      connect(visual.get(), SIGNAL(stepEdited(vigir_footstep_planning_msgs::EditStep)), step_property, SLOT(updateStep(vigir_footstep_planning_msgs::EditStep)));
      connect(visual.get(), SIGNAL(selected(bool)), step_property, SLOT(setExpanded(bool)));
      connect(step_property, SIGNAL(stepPoseChanged(Ogre::Vector3, Ogre::Quaternion)), visual.get(), SLOT(changePose(Ogre::Vector3, Ogre::Quaternion)));

    }
    if(!displayFeedback)
      step_visuals_.push_back(visual);

    if(displayFeedback)
      feedback_visuals_.push_back(visual);

  }
//  ROS_ERROR("Display Steps finished");

}

void StepPlanDisplay::activateFeetTool(bool active)
{
  if ( active )
  {
    if(feet_tool_)
      tool_manager_->setCurrentTool(feet_tool_);
  }
  else
  {
   tool_manager_->setCurrentTool(interact_tool_);
  }
}

void StepPlanDisplay::activateFeetTool(PlantFeetMode mode, bool active)
{
  if(feet_tool_)
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
  start_visuals_.reset(new FeetVisual(scene_manager_, scene_node_, START));
  start_visuals_->createByMessage(start);
  start_visuals_->setFramePosition(frame_position);
  start_visuals_->setFrameOrientation(frame_orientation);
  start_visuals_->setVisible(start_property_container_->getBool());


  /*
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

  start_visuals_[0]->setVisible(start_property_container_->getBool());
  start_visuals_[1]->setVisible(start_property_container_->getBool());
  */
}

void StepPlanDisplay::setStartVisible()
{
  Q_EMIT(requestStartPose());
}

void StepPlanDisplay::updateDisplay(int from, int to)
{
  if( to >= step_visuals_.size())
    return;

  for(int i=0; i < from; i++ )
  {
    step_visuals_[i]->setVisible(false);
    StepProperty* property_i = static_cast<StepProperty*>(property_container_->childAt(i));
    if(property_i)
    {
      property_i->setHidden(true);
    }
  }
  for (int i=from; i < step_visuals_.size(); i++)
  {
    step_visuals_[i]->setVisible(true);
    StepProperty* property_i = static_cast<StepProperty*>(property_container_->childAt(i));
    if(property_i)
    {
      property_i->setHidden(false);
    }

  }
  for(int i=to+1; i < step_visuals_.size(); i++ )
  {
    step_visuals_[i]->setVisible(false);
    Property* property_i = property_container_->childAt(i);
    if(property_i != NULL)
    {
      property_i->setHidden(true);
    }
  }
}

void StepPlanDisplay::visualizeFeedback(vigir_footstep_planning_msgs::PlanningFeedback feedback)
{
  if(display_feedback_->getBool())
  {
    displayFeedback = true;
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;

    if(transformToFixedFrame(position, orientation, feedback.current_step_plan.header))
    {
      feedback_visuals_.clear();
      displaySteps(feedback.current_step_plan.steps, position, orientation);
    }
  }
}

void StepPlanDisplay::setDisplayFeedback()
{
  Q_EMIT(feedbackRequestedChanged(display_feedback_->getBool()));
}

void StepPlanDisplay::updateACConnected()
{
  bool connected_step_plan = panel_->checkConnection();
  bool connected_edit_step = step_plan_helper_ ? step_plan_helper_->checkConnection() : false;


  if(connected_step_plan && connected_edit_step)
  {
    ac_connected_->setLevel((rviz::StatusProperty::Level) rviz::StatusLevel::Ok);
    ac_connected_->setValue("Action Clients are connected to Server.");
  }
  else if(connected_step_plan) // only stepPlanRequest possible
  {
    ac_connected_->setLevel((rviz::StatusProperty::Level) rviz::StatusLevel::Warn);
    ac_connected_->setValue("EditStep Action Client is not connected to Server.");
  }
  else
  {
    ac_connected_->setLevel((rviz::StatusProperty::Level) rviz::StatusLevel::Error);
    ac_connected_->setValue("StepPlanRequest Action Client is not connected to Server!");
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
 // ROS_INFO("%s", tool->getName().toStdString().c_str());
  if(tool->getName()=="Plant Feet")
  {
    feet_tool_ = static_cast<PlantFeetTool*>(tool);
    feet_tool_->setInteractiveMarkerServer(interactive_marker_server_);
    makeFeetToolConnections();
  }
  if(tool->getName() == "Interact")
  {
    interact_tool_ = tool;
  }
}

void StepPlanDisplay::setStepValid(unsigned int index, bool valid)
{
  if(!visualize_cost_->getBool())
  {
    step_visuals_[index]->setValid(valid);
    StepProperty* step_property = static_cast<StepProperty*>(property_container_->childAt(index));
    step_property->setValid(valid);
  }
//  if(step_properties_.size() == step_visuals_.size())
//    step_properties_[index]->setValid(valid);
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
  case LEFT_FOOT:
    visual.reset(new StepVisual( scene_manager_, scene_node_,vigir_footstep_planning_msgs::Foot::LEFT, frame_id_property_->getString().toStdString(), step_visuals_.size()));
    if( step_plan_helper_)
      step_plan_helper_->addStep(frame_id_property_->getString().toStdString(), position, orientation,vigir_footstep_planning_msgs::Foot::LEFT, step_visuals_.size());
    break;
  case RIGHT_FOOT:
    visual.reset(new StepVisual( scene_manager_, scene_node_, vigir_footstep_planning_msgs::Foot::RIGHT, frame_id_property_->getString().toStdString(), step_visuals_.size()));
    if( step_plan_helper_)
      step_plan_helper_->addStep(frame_id_property_->getString().toStdString(), position, orientation,vigir_footstep_planning_msgs::Foot::RIGHT, step_visuals_.size());
    break;
  }

  if(mode == LEFT_FOOT || mode == RIGHT_FOOT)
  {
    visual->displayIndex( display_index_->getBool());
    visual->createVisualAt(position, orientation);
    visual->initializeInteractiveMarker(interactive_marker_server_);
    makeStepVisualConnections(visual.get());
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
      // use parameter: default step cost: const_step_cost_estimator: step_cost: 0.1
      step_visuals_[i]->visualizeCost(0.3);
    }
  }
  else
    step_plan_helper_->checkSteps();
}

void StepPlanDisplay::setUpdateStepPlan()
{
  step_plan_helper_->setUpdateStepPlanPositions(update_step_plan_positions_->getBool());
}

void StepPlanDisplay::updateStepVisuals(vigir_footstep_planning_msgs::StepPlan updated_step_plan)
{
  if(step_visuals_.size() != updated_step_plan.steps.size())
  {
    ROS_ERROR("Cannot update step plan because of size mismatch");
  }
  for (unsigned int i = 0; i < updated_step_plan.steps.size(); ++i)
  {
    step_visuals_[i]->updateStep(updated_step_plan.steps[i]);
    StepProperty* property_i = static_cast<StepProperty*>(property_container_->childAt(i));
    property_i->updateStep(updated_step_plan.steps[i]);
//    if(step_properties_.size() == step_visuals_.size())
//      step_properties_[i]->updateStep(updated_step_plan.steps[i]);
  }
}

void StepPlanDisplay::addGoalFeetProperties(vigir_footstep_planning_msgs::Feet goal_feet)
{
  if(goal_properties.size() == 2)
  {
    updateFootProperty(goal_properties[0], goal_feet.left);
    updateFootProperty(goal_properties[1], goal_feet.right);

    Ogre::Vector3 pos = Ogre::Vector3((goal_feet.left.pose.position.x + goal_feet.right.pose.position.x) / 2,
                                      (goal_feet.left.pose.position.y + goal_feet.right.pose.position.y) / 2,
                                      (goal_feet.left.pose.position.z + goal_feet.right.pose.position.z) / 2);
    Ogre::Quaternion orient = Ogre::Quaternion(goal_feet.left.pose.orientation.w,
                                               goal_feet.left.pose.orientation.x,
                                               goal_feet.left.pose.orientation.y,
                                               goal_feet.left.pose.orientation.z);

    goal_position_property_->setVector(pos);
    goal_orientation_property_->setQuaternion(orient);

  }
  else
  {
    goal_property_container_->removeChildren();

    Ogre::Vector3 pos = Ogre::Vector3((goal_feet.left.pose.position.x + goal_feet.right.pose.position.x) / 2,
                                      (goal_feet.left.pose.position.y + goal_feet.right.pose.position.y) / 2,
                                      (goal_feet.left.pose.position.z + goal_feet.right.pose.position.z) / 2);
    Ogre::Quaternion orient = Ogre::Quaternion(goal_feet.left.pose.orientation.w,
                                               goal_feet.left.pose.orientation.x,
                                               goal_feet.left.pose.orientation.y,
                                               goal_feet.left.pose.orientation.z);
    goal_position_property_ = new rviz::VectorProperty("Position",
                                                   pos,
                                                   "Middlepoint of goal feet",
                                                   goal_property_container_,
                                                       SLOT(goalPoseUpdated()),
                                                       this);
    goal_orientation_property_ = new rviz::QuaternionProperty("Orientation",
                                                              orient,
                                                              "Orientation of goal feet.",
                                                              goal_property_container_,
                                                              SLOT(goalPoseUpdated()),
                                                              this);
    goal_properties.clear();
    // left foot:
    rviz::Property* left_container = new Property("Left",
                                                  QVariant(),
                                                  "Left foot of goal configuration.",
                                                  goal_property_container_);
    addFootProperty(goal_feet.left, left_container);
    goal_properties.push_back(left_container);
    // Right Foot:
    rviz::Property* right_container = new Property("Right",
                                                  QVariant(),
                                                  "Right foot of goal configuration.",
                                                  goal_property_container_);
    addFootProperty(goal_feet.right, right_container);
    goal_properties.push_back(right_container);
  }

}

void StepPlanDisplay::addStartFeetProperties(vigir_footstep_planning_msgs::Feet start_feet)
{

  if(start_properties.size() == 2)
  {
    updateFootProperty(start_properties[0], start_feet.left);
    updateFootProperty(start_properties[1], start_feet.right);


    Ogre::Vector3 pos = Ogre::Vector3((start_feet.left.pose.position.x + start_feet.right.pose.position.x) / 2,
                                      (start_feet.left.pose.position.y + start_feet.right.pose.position.y) / 2,
                                      (start_feet.left.pose.position.z + start_feet.right.pose.position.z) / 2 );
    Ogre::Quaternion orient = Ogre::Quaternion(start_feet.left.pose.orientation.w,
                                               start_feet.left.pose.orientation.x,
                                               start_feet.left.pose.orientation.y,
                                               start_feet.left.pose.orientation.z);

    start_position_property_->setVector(pos);
    start_orientation_property_->setQuaternion(orient);
  }
  else
  {
    start_property_container_->removeChildren();

    Ogre::Vector3 pos = Ogre::Vector3((start_feet.left.pose.position.x + start_feet.right.pose.position.x) / 2,
                                      (start_feet.left.pose.position.y + start_feet.right.pose.position.y) / 2,
                                      (start_feet.left.pose.position.z + start_feet.right.pose.position.z) / 2);
    Ogre::Quaternion orient = Ogre::Quaternion(start_feet.left.pose.orientation.w,
                                               start_feet.left.pose.orientation.x,
                                               start_feet.left.pose.orientation.y,
                                               start_feet.left.pose.orientation.z);
    start_position_property_ = new rviz::VectorProperty("Position",
                                                   pos,
                                                   "Middlepoint of goal feet",
                                                   start_property_container_,
                                                        SLOT(startPoseUpdated()), this);
    start_orientation_property_ = new rviz::QuaternionProperty("Orientation",
                                                              orient,
                                                              "Orientation of goal feet.",
                                                              start_property_container_,
                                                               SLOT(startPoseUpdated()), this);

    start_properties.clear();
    // left foot:
    rviz::Property* left_container = new Property("Left",
                                                  QVariant(),
                                                  "Left foot of start configuration.",
                                                  start_property_container_);
    addFootProperty(start_feet.left, left_container);
    start_properties.push_back(left_container);
    // Right Foot:
    rviz::Property* right_container = new Property("Right",
                                                  QVariant(),
                                                  "Right foot of start configuration.",
                                                  start_property_container_);
    addFootProperty(start_feet.right, right_container);
    start_properties.push_back(right_container);
  }
}

void StepPlanDisplay::addFootProperty(vigir_footstep_planning_msgs::Foot foot, rviz::Property* parent)
{
  rviz::VectorProperty* position = new rviz::VectorProperty("Position",
                                       Ogre::Vector3(foot.pose.position.x, foot.pose.position.y, foot.pose.position.z),
                                       "Current position",
                                       parent);
  rviz::QuaternionProperty* orientation = new rviz::QuaternionProperty("Orientation",
                                                                        Ogre::Quaternion(foot.pose.orientation.w, foot.pose.orientation.x, foot.pose.orientation.y, foot.pose.orientation.z),
                                                                        "Current orientation",
                                                                         parent);
  position->setReadOnly(true);
  orientation->setReadOnly(true);
}

void StepPlanDisplay::updateFootProperty(rviz::Property* parent, vigir_footstep_planning_msgs::Foot foot)
{

  rviz::VectorProperty* position = static_cast<rviz::VectorProperty*> (parent->childAt(0));
  rviz::QuaternionProperty* orientation = static_cast<rviz::QuaternionProperty*> (parent->childAt(1));

  position->setVector(Ogre::Vector3(foot.pose.position.x, foot.pose.position.y, foot.pose.position.z));
  orientation->setQuaternion(Ogre::Quaternion(foot.pose.orientation.w, foot.pose.orientation.x,
                                              foot.pose.orientation.y, foot.pose.orientation.z));
}

void StepPlanDisplay::goalPoseUpdated()
{
  panel_->handleGoalPose(goal_position_property_->getVector(), goal_orientation_property_->getQuaternion());
}

void StepPlanDisplay::startPoseUpdated()
{
  step_plan_helper_->setRobotPose(start_position_property_->getVector(), start_orientation_property_->getQuaternion());
}


}
 // end namespace vigir_footstep_planning_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning_rviz_plugin::StepPlanDisplay,rviz::Display )
