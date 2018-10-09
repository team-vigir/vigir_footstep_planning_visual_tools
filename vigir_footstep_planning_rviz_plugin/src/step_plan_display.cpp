#include <vigir_footstep_planning_rviz_plugin/step_plan_display.h>

#include <vigir_footstep_planning_rviz_plugin/step_visual.h>
#include <vigir_footstep_planning_rviz_plugin/step_property.h>
#include <vigir_footstep_planning_rviz_plugin/footstep_planning_panel.h>
#include <vigir_footstep_planning_rviz_plugin/step_plan_helper.h>
#include <vigir_footstep_planning_rviz_plugin/feet_visual.h>
#include <vigir_footstep_planning_rviz_plugin/place_feet_tool.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <tf/transform_listener.h>

#include <rviz/display_context.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/quaternion_property.h>

#include <rviz/frame_manager.h>
#include <rviz/tool_manager.h>
#include <rviz/default_plugin/tools/interaction_tool.h>



#include <interactive_markers/interactive_marker_server.h>


namespace vigir_footstep_planning_rviz_plugin
{
StepPlanDisplay::StepPlanDisplay()
  : displayFeedback(false)
  , interaction_mode_(PLANE)
  , last_step_index(0)
  , step_plan_helper_(0)
  , feet_tool_(0)
  , interact_tool_(0)
  , start_feet_(0)
  , goal_feet_(0)
{
  qRegisterMetaType<vigir_footstep_planning_msgs::Step>("vigir_footstep_planning_msgs::Step");

  initializeDisplayProperties();
}

StepPlanDisplay::~StepPlanDisplay()
{
  step_visuals_.clear();
  feedback_visuals_.clear();
  goal_properties.clear();
  start_properties.clear();
  delete im_server_feet;
  delete im_server_steps;
  delete panel_;
  delete goal_feet_;
  delete start_feet_;
  if(step_plan_helper_)
    delete step_plan_helper_;
}

void StepPlanDisplay::onInitialize()
{
  // set icon:
  std::string icons_path;
  if(nh.getParam("icons_path", icons_path))
    setIcon(QIcon(QString::fromStdString(icons_path + "stepPlan.png")));
  step_visuals_.rset_capacity(100);
  feedback_visuals_.rset_capacity(100);

  // Add Panel:
  panel_= new FootstepPlanningPanel();
  this->setAssociatedWidget (panel_);
  makePanelConnections(); // panel - display connections

  // Add Step Plan Helper:
  step_plan_helper_= new StepPlanHelper();
  makeStepPlanHelperConnections(); // step_plan_helper - diplay / panel connections

  // Interactive Marker Servers
  im_server_steps = new interactive_markers::InteractiveMarkerServer("step_marker");
  im_server_feet = new interactive_markers::InteractiveMarkerServer("feet_marker");


  // set interaction and feet tool when added in rviz:
  tool_manager_ = context_->getToolManager();
  connect(tool_manager_, &rviz::ToolManager::toolAdded, this, &StepPlanDisplay::checkTool);
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
  rviz::Property* visualization_properties_ = new rviz::Property("Display Options",
                                                                 QVariant(),
                                                                 "Step Plan Display Options",
                                                                 this);
  display_index_ = new rviz::BoolProperty("Display Indices",
                                          false,
                                          "Display Indices of Step Plan",
                                          visualization_properties_, SLOT(displayIndex()), this);
  display_feedback_ = new rviz::BoolProperty("Display Feedback",
                                             false,
                                             "Displays current solution for step plan, while computing final step plan",
                                             visualization_properties_, SLOT(setDisplayFeedback()), this);
  update_step_plan_positions_ = new rviz::BoolProperty("Update 3D",
                                                       false,
                                                       "Updates 3D position when a new step plan is created or changed.",
                                                       this, SLOT(setUpdateStepPlan()), this);
  visualize_valid_ = new rviz::BoolProperty("Visualize invalid steps",
                                            true,
                                            "Marks invalid steps gray",
                                            visualization_properties_,
                                            SLOT(updateVisualizeValid()), this);
  visualize_cost_ = new rviz::BoolProperty("Visualize Step Cost",
                                           false,
                                           "Visualizes step cost, where red indicates high and blue low cost.",
                                           visualization_properties_, SLOT(visualizeStepCost()), this);

  // Step Properties  ---------------------------------------------------
  step_properties_container_ = new rviz::Property("Step Properties",
                                           QVariant(),
                                           "List of properties of current step plan",
                                           this);

  // Feet Properties ----------------------------------------------------
  goal_property_container_ = new rviz::BoolProperty("Goal Feet",
                                                    false,
                                                    "Properties of current goal pose.",
                                                    this, SLOT(setGoalVisible()));
  goal_position_property_ = new rviz::VectorProperty("Position",
                                                     Ogre::Vector3::ZERO,
                                                     "Middlepoint of goal feet on the ground",
                                                     goal_property_container_,
                                                     SLOT(goalPoseUpdated()),
                                                     this);
  goal_orientation_property_ = new rviz::QuaternionProperty("Orientation",
                                                            Ogre::Quaternion::IDENTITY,
                                                            "Orientation of goal feet",
                                                            goal_property_container_,
                                                            SLOT(goalPoseUpdated()),
                                                            this);
  start_property_container_ = new rviz::BoolProperty("Start Feet",
                                                     false,
                                                     "Properties of current starting pose",
                                                     this, SLOT(setStartVisible()));
  start_position_property_ = new rviz::VectorProperty("Position",
                                                     Ogre::Vector3::ZERO,
                                                     "Middlepoint of start feet on the ground",
                                                     start_property_container_,
                                                     SLOT(startPoseUpdated()),
                                                     this);
  start_orientation_property_ = new rviz::QuaternionProperty("Orientation",
                                                            Ogre::Quaternion::IDENTITY,
                                                            "Orientation of start feet",
                                                            start_property_container_,
                                                            SLOT(startPoseUpdated()),
                                                            this);
}

void StepPlanDisplay::makeFeetToolConnections()
{
  connect(feet_tool_, &PlaceFeetTool::newStartPose, this, &StepPlanDisplay::addStartFeetMsg);
  connect(feet_tool_, &PlaceFeetTool::newGoalPose, this, &StepPlanDisplay::addGoalFeet);
  // pass goal pose to request handler
  connect(feet_tool_, &PlaceFeetTool::newGoalPose, panel_, &FootstepPlanningPanel::updateGoalPose);
}

void StepPlanDisplay::makePanelConnections()
{
  // on step plan created
  connect( panel_, &FootstepPlanningPanel::createdStepPlan, this, &StepPlanDisplay::displayStepPlan);
  // Feet Tool, Set Goal
  connect(panel_, &FootstepPlanningPanel::feetToolActivated, this, &StepPlanDisplay::activateFeetTool);

  // Get Feedback
  connect(this, &StepPlanDisplay::feedbackRequestedChanged, panel_, &FootstepPlanningPanel::setFeedbackRequested);
  connect(panel_, &FootstepPlanningPanel::sendPlanningFeedback, this, &StepPlanDisplay::visualizeFeedback);

  // Display Options
  connect(panel_, &FootstepPlanningPanel::displayRangeChanged, this, &StepPlanDisplay::updateDisplay);
  connect(panel_, &FootstepPlanningPanel::interactionModeChanged, this, &StepPlanDisplay::setInteractionMode);

  // Clear
  connect(panel_, &FootstepPlanningPanel::clearStepPlan, this, &StepPlanDisplay::resetStepPlan);
  connect(panel_, &FootstepPlanningPanel::clearAll, this, &StepPlanDisplay::resetGoal);

  // start pose
  connect(this, &StepPlanDisplay::requestStartPose, panel_, &FootstepPlanningPanel::startPoseRequested);
  connect(panel_, &FootstepPlanningPanel::startFeetAnswer, this, &StepPlanDisplay::addStartFeetMsg);

  // save
  connect(panel_, &FootstepPlanningPanel::changed, this, [=](){this->setShouldBeSaved(true);});
}

void StepPlanDisplay::makeStepPlanHelperConnections()
{
  // set step plan:
  connect( panel_ , &FootstepPlanningPanel::createdStepPlan, step_plan_helper_, &StepPlanHelper::setCurrentStepPlan);
  connect( panel_ , &FootstepPlanningPanel::createdSequence, step_plan_helper_, &StepPlanHelper::handleSequence);
  connect( step_plan_helper_, &StepPlanHelper::createdStepPlan, this, &StepPlanDisplay::displayStepPlan);
  connect( step_plan_helper_, &StepPlanHelper::createdStepPlan, panel_, &FootstepPlanningPanel::setStepPlan);
  connect( step_plan_helper_, &StepPlanHelper::createdStepPlan, panel_, &FootstepPlanningPanel::updateFromTo);
  // step plan updated
  connect( step_plan_helper_, &StepPlanHelper::updatedStepPlan, panel_, &FootstepPlanningPanel::setStepPlan);
  connect( step_plan_helper_, &StepPlanHelper::updatedStepPlan, this, &StepPlanDisplay::updateStepVisuals);
  connect (step_plan_helper_, &StepPlanHelper::stepValidUpdate, this, &StepPlanDisplay::setStepValid); //visualize validity while manipulating

  // undo
  connect(panel_, &FootstepPlanningPanel::undo, step_plan_helper_, &StepPlanHelper::setPreviousStepPlan);
  connect(panel_, &FootstepPlanningPanel::undoSequence, step_plan_helper_, &StepPlanHelper::setPreviousSequence);
  // clear
  connect(panel_, &FootstepPlanningPanel::clearStepPlan, step_plan_helper_, &StepPlanHelper::resetStepPlan);

  // Abort
  connect(panel_, &FootstepPlanningPanel::abort, step_plan_helper_, &StepPlanHelper::abortExecution);
  // execute
  connect(panel_, &FootstepPlanningPanel::executeRequested, step_plan_helper_, &StepPlanHelper::executeStepPlan);
  connect(step_plan_helper_, &StepPlanHelper::executionStarted, panel_, &FootstepPlanningPanel::initializeExecutionProgressbar);
  connect(step_plan_helper_, &StepPlanHelper::executionFeedback, panel_, &FootstepPlanningPanel::updateProgressBar);
  connect(step_plan_helper_, &StepPlanHelper::executionFinished, panel_, &FootstepPlanningPanel::updateProgressBarExecutionState);
}

void StepPlanDisplay::makeStepVisualConnections(const StepVisual* visual)
{
  if(step_plan_helper_)
  {
    connect(visual, &StepVisual::stepChanged, step_plan_helper_, &StepPlanHelper::editStep);
    connect(visual, &StepVisual::updateStepsPos, step_plan_helper_, &StepPlanHelper::updateStepPlanPositions);
    connect(visual, &StepVisual::footDropped, step_plan_helper_, &StepPlanHelper::acceptModifiedStepPlan);
    connect(visual, &StepVisual::endStepPlanHere, step_plan_helper_, &StepPlanHelper::trimStepPlan);
  }

  connect(panel_, &FootstepPlanningPanel::clearIM, visual, &StepVisual::setButtonInteractiveMarker);
  connect(visual, &StepVisual::cutStepPlanHere, this, &StepPlanDisplay::visualizeCut);
  connect(visual, &StepVisual::cutStepPlanHere,panel_, &FootstepPlanningPanel::setLastStep);
  connect(visual, &StepVisual::replanToHere, panel_, &FootstepPlanningPanel::replanToIndex);
  connect(visual, &StepVisual::replanToHere, this, &StepPlanDisplay::visualizeReplan);

}

void StepPlanDisplay::update(float wall_dt, float ros_dt)
{
  updateACConnected();
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
void StepPlanDisplay::resetStepPlan()
{
  step_properties_container_->removeChildren();
  step_visuals_.clear();
  feedback_visuals_.clear();
}

void StepPlanDisplay::displayStepPlan(const vigir_footstep_planning_msgs::StepPlan& step_plan)
{
  last_step_index = step_plan.steps.size();
  feedback_visuals_.clear();
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if(transformToFixedFrame(position, orientation, step_plan.header))
  {
    displayFeedback = false;
    step_visuals_.clear();
    addStartFeet(step_plan.start, position, orientation);
    displaySteps(step_plan.steps, position, orientation);
    setStartVisible();
    if(visualize_cost_->getBool())
      visualizeStepCost();
  }
}

void StepPlanDisplay::displaySteps(const std::vector<vigir_footstep_planning_msgs::Step>& steps, const Ogre::Vector3& frame_position, const Ogre::Quaternion& frame_orientation)
{
  if(!displayFeedback)
    step_properties_container_->removeChildren();

  bool displIndex = display_index_->getBool();
  for (unsigned i = 0; i< steps.size(); i++)
  {
    boost::shared_ptr<StepVisual> visual;
    visual.reset(new StepVisual( scene_manager_, scene_node_, steps[i].foot.foot_index, steps[i].header.frame_id, steps[i].step_index));
    visual->createByMessage( steps[i]);
    visual->setFramePosition( frame_position );
    visual->setFrameOrientation( frame_orientation );
    visual->setVisualizeValid(visualize_valid_->getBool());
    //Interactive Marker:
    if(!displayFeedback)
    {
      visual->displayIndex(displIndex);

      StepProperty* step_property = new StepProperty(steps[i], step_properties_container_);

      if(i != 0)
      {
        visual->initializeInteractiveMarker(im_server_steps);
        visual->setInteractionMode(interaction_mode_);
      }

      makeStepVisualConnections(visual.get());
      connect(visual.get(), &StepVisual::stepChanged, step_property, static_cast<void (StepProperty::*)(EditStepMsg)>(&StepProperty::updateStep));
      connect(visual.get(), &StepVisual::selected, step_property, &StepProperty::setExpanded);
      connect(step_property, &StepProperty::stepPoseChanged, visual.get(), &StepVisual::editedPose);

    }
    if(!displayFeedback)
      step_visuals_.push_back(visual);

    if(displayFeedback)
      feedback_visuals_.push_back(visual);
  }
}

void StepPlanDisplay::activateFeetTool(PlaceFeetMode mode, bool active)
{
  if(!feet_tool_)
  {
    ROS_ERROR("Please add Place Feet Tool");
    return;
  }

  if ( active )
  {
    feet_tool_->setMode(mode);
    tool_manager_->setCurrentTool(feet_tool_);
  }
  else
  {
    feet_tool_->deactivate();

    if(interact_tool_)
      tool_manager_->setCurrentTool(interact_tool_);
  }
}

void StepPlanDisplay::addStartFeetMsg(vigir_footstep_planning_msgs::Feet start)
{
  panel_->releasePlaceFeet();
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (! transformToFixedFrame(position, orientation, start.header))
    return; // failed
  addStartFeet(start, position, orientation);
}

void StepPlanDisplay::addStartFeet(const vigir_footstep_planning_msgs::Feet& start, const Ogre::Vector3& frame_position, const Ogre::Quaternion& frame_orientation)
{
  if(! start_feet_)
  {
    start_feet_ = new FeetVisual(scene_manager_, scene_node_, START);
    start_feet_->createByMessage(start);
    start_feet_->setFramePosition(frame_position);
    start_feet_->setFrameOrientation(frame_orientation);
    if(!step_plan_helper_->executeStepPlanActive())
    {
      start_feet_->initializeInteractiveMarker(im_server_feet);
      connect(start_feet_, &FeetVisual::feetPoseChanged, this, &StepPlanDisplay::setStartFeetPlannerFrameProperty);
      connect(panel_, &FootstepPlanningPanel::clearIM, start_feet_, &FeetVisual::setButtonInteractiveMarker);
    }
  }
  else
    start_feet_->updateFeetMsg(start, frame_position, frame_orientation);

  start_feet_->setVisible(start_property_container_->getBool());
  addStartFeetRobotFrameProperties(start);
}

void StepPlanDisplay::addGoalFeet(vigir_footstep_planning_msgs::Feet goal)
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (! transformToFixedFrame(position, orientation, goal.header))
    return; //failed

  if(!goal_feet_)
  {
    goal_feet_ = new FeetVisual(scene_manager_, scene_node_, GOAL);
    goal_feet_->createByMessage(goal);
    goal_feet_->setFramePosition(position);
    goal_feet_->setFrameOrientation(orientation);
    goal_feet_->initializeInteractiveMarker(im_server_feet);
    connect(goal_feet_, &FeetVisual::feetPoseChanged, this, &StepPlanDisplay::setGoalFeetPlannerFrameProperty);
    connect(panel_, &FootstepPlanningPanel::clearIM, goal_feet_, &FeetVisual::setButtonInteractiveMarker);
  }
  else
    goal_feet_->updateFeetMsg(goal, position, orientation);

  goal_feet_->setVisible(goal_property_container_->getBool());

  addGoalFeetRobotFrameProperties(goal);
}

void StepPlanDisplay::setStartVisible()
{
  Q_EMIT(requestStartPose());
}

void StepPlanDisplay::setGoalVisible()
{
  if(goal_feet_)
    goal_feet_->setVisible(goal_property_container_->getBool());
}

void StepPlanDisplay::updateDisplay(int from, int to)
{
  if( to >= step_visuals_.size())
    return;

  for(int i=0; i < from; i++ )
  {
    step_visuals_[i]->setVisible(false);
    StepProperty* property_i = static_cast<StepProperty*>(step_properties_container_->childAt(i));
    if(property_i)
    {
      property_i->setHidden(true);
    }
  }
  for (int i=from; i < step_visuals_.size(); i++)
  {
    step_visuals_[i]->setVisible(true);
    StepProperty* property_i = static_cast<StepProperty*>(step_properties_container_->childAt(i));
    if(property_i)
    {
      property_i->setHidden(false);
    }

  }
  for(int i=to+1; i < step_visuals_.size(); i++ )
  {
    step_visuals_[i]->setVisible(false);
    Property* property_i = step_properties_container_->childAt(i);
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
  bool connected_edit_step = step_plan_helper_ ? step_plan_helper_->checkConnectionEditStep() : false;


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

// Add feet tool and interact tool when added to rviz
void StepPlanDisplay::checkTool(rviz::Tool* tool)
{
  // ROS_INFO("%s", tool->getName().toStdString().c_str());
  if(tool->getName()=="Place Feet")
  {
    feet_tool_ = static_cast<PlaceFeetTool*>(tool);
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
    step_visuals_[index]->visualizeValid(valid);
    StepProperty* step_property = static_cast<StepProperty*>(step_properties_container_->childAt(index));
    step_property->setValid(valid);
  }
}

void StepPlanDisplay::visualizeCut(int new_end)
{
  last_step_index = new_end;
  for(int i = 0; i <= new_end; i++)
  {
    step_visuals_[i]->visualizeValid(true);
  }

  for(int i = new_end+1; i < step_visuals_.size(); ++i)
  {
    step_visuals_[i]->visualizeValid(false);
  }
}

void StepPlanDisplay::visualizeReplan(int end)
{
  for (int i = end+1; i < step_visuals_.size(); ++i)
  {
    step_visuals_[i]->visualizeValid(true);
  }
  if(last_step_index > end) // replan from index 0 to end
  {
    for(int i = 0; i <= end; ++i)
    {
      step_visuals_[i]->visualizeValid(false);
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

void StepPlanDisplay::setInteractionMode(int interaction_mode)
{
  if(interaction_mode <= 3 || interaction_mode >= 1){
    interaction_mode_ = static_cast<InteractionMode>(interaction_mode);
    for (unsigned int i = 0; i < step_visuals_.size(); ++i)
    {
      step_visuals_[i]->setInteractionMode(interaction_mode_);
    }
  }
}

void StepPlanDisplay::visualizeStepCost()
{
  if(visualize_cost_->getBool())
  {
    for(unsigned int i = 0; i < step_visuals_.size(); ++i)
    {
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
    step_visuals_[i]->updateStepMsg(updated_step_plan.steps[i]);
    StepProperty* property_i = static_cast<StepProperty*>(step_properties_container_->childAt(i));
    property_i->updateStep(updated_step_plan.steps[i]);
  }
}

void StepPlanDisplay::addGoalFeetRobotFrameProperties(vigir_footstep_planning_msgs::Feet goal_feet)
{
  if(goal_properties.size() == 0)
  {
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
  else
  {
    updateFootProperty(goal_properties[0], goal_feet.left);
    updateFootProperty(goal_properties[1], goal_feet.right);
  }
}

void StepPlanDisplay::setGoalFeetPlannerFrameProperty(Ogre::Vector3 position, Ogre::Quaternion orientation)
{

  if(goal_position_property_ && goal_orientation_property_)
  {
    goal_position_property_->setVector(position);
    goal_orientation_property_->setQuaternion(orientation);
  }
  else
  {
    goal_position_property_ = new rviz::VectorProperty("Position",
                                                       position,
                                                       "Middlepoint of goal feet on the ground.",
                                                       goal_property_container_,
                                                       SLOT(goalPoseUpdated()),
                                                       this);
    goal_orientation_property_ = new rviz::QuaternionProperty("Orientation",
                                                              orientation,
                                                              "Orientation of goal feet.",
                                                              goal_property_container_,
                                                              SLOT(goalPoseUpdated()),
                                                              this);
    goalPoseUpdated();
  }
}

void StepPlanDisplay::setStartFeetPlannerFrameProperty(Ogre::Vector3 position, Ogre::Quaternion orientation)
{
  if(start_position_property_ && start_orientation_property_)
  {
    start_position_property_->setVector(position);
    start_orientation_property_->setQuaternion(orientation);
  }
  else
  {
    start_position_property_ = new rviz::VectorProperty("Position",
                                                       position,
                                                       "Middlepoint of start feet on the ground.",
                                                       start_property_container_,
                                                       SLOT(startPoseUpdated()),
                                                       this);
    start_orientation_property_ = new rviz::QuaternionProperty("Orientation",
                                                              orientation,
                                                              "Orientation of start feet.",
                                                              start_property_container_,
                                                              SLOT(startPoseUpdated()),
                                                              this);
    startPoseUpdated();
  }
}

void StepPlanDisplay::addStartFeetRobotFrameProperties(vigir_footstep_planning_msgs::Feet start_feet)
{
  if(start_properties.size() == 0)
  {
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
  else
  {
    updateFootProperty(start_properties[0], start_feet.left);
    updateFootProperty(start_properties[1], start_feet.right);
  }
}

void StepPlanDisplay::addFootProperty(vigir_footstep_planning_msgs::Foot foot, rviz::Property* parent)
{
  rviz::VectorProperty* position = new rviz::VectorProperty("Position",
                                                            Ogre::Vector3(foot.pose.position.x, foot.pose.position.y, foot.pose.position.z),
                                                            "Current position (of the ancle)",
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
  if(feet_tool_)
    feet_tool_->setValidFeet(goal_position_property_->getVector(), goal_orientation_property_->getQuaternion(), frame_id_property_->getString().toStdString(), GOAL);
  else
    ROS_ERROR("Please add Place Feet Tool");
}

void StepPlanDisplay::startPoseUpdated()
{
  if(feet_tool_)
    feet_tool_->setValidFeet(start_position_property_->getVector(), start_orientation_property_->getQuaternion(), frame_id_property_->getString().toStdString(), START);
  else
    ROS_ERROR("Please add Place Feet Tool");
}

void StepPlanDisplay::resetGoal()
{
  if(goal_feet_)
  {
    delete goal_feet_;
    goal_feet_ = 0;
    goal_properties.clear();
    goal_property_container_->removeChildren(2,2); //remove last two children
  }
}

void StepPlanDisplay::updateVisualizeValid()
{
  for(int i=0; i < step_visuals_.size(); ++i)
  {
    step_visuals_[i]->setVisualizeValid(visualize_valid_->getBool());
  }
}
}
// end namespace vigir_footstep_planning_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning_rviz_plugin::StepPlanDisplay,rviz::Display )
