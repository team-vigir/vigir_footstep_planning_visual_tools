#ifndef STEP_PLAN_HELPER_H
#define STEP_PLAN_HELPER_H

#ifndef Q_MOC_RUN

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>
#include <actionlib/client/simple_action_client.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <QObject>

#endif

typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::EditStepAction> EditStepActionClient;
typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::ExecuteStepPlanAction> ExecuteStepPlanActionClient;
typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::UpdateStepPlanAction> UpdateStepPlanActionClient;
typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::UpdateFootAction> UpdateFootActionClient;
typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::SetStepPlanAction> SetStepPlanActionClient;
typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::GetStepPlanAction> GetStepPlanActionClient;


typedef vigir_footstep_planning_msgs::ErrorStatus ErrorStatusMsg;
typedef vigir_footstep_planning_msgs::StepPlan StepPlanMsg;
typedef vigir_footstep_planning_msgs::Step StepMsg;

namespace vigir_footstep_planning_rviz_plugin
{
/* modify step plan
 * update step plan
 * modify steps in step plan
 *
 * check state of step plan
 * check state of steps
 *
 * set current robot pose by publishing /robot_pose topic
*/
class StepPlanHelper : public QObject
{
  Q_OBJECT
public:
  StepPlanHelper(QObject *parent = 0);
  virtual ~StepPlanHelper();




  // Acion Server Connection
  bool checkConnection();
  void connectToActionServer();

  // add step at the end of current step plan
  void addStep(const std::string& frame_id, const Ogre::Vector3& position, const Ogre::Quaternion& orientation, unsigned int which_foot, unsigned int step_index);

  // checks if steps of current step plan are valid, emits stepValidUpdate
  void checkSteps();
  // sets if positions should be updated when a new step plan is set
  void setUpdateStepPlanPositions(bool update);

public Q_SLOTS:
  // called with edited step, sends goal to edit_step action client, result is handled in editStepCallback
  void editStep(vigir_footstep_planning_msgs::EditStep edit_step);
  // called when executing is requested, sends goal to execute_step_plan action client, result is handled in executStepPlanCallback
  void executeStepPlan();
  // update current step plan, update positions if requested
  void setCurrentStepPlan(vigir_footstep_planning_msgs::StepPlan step_plan);
  // sends /robot_pose to update the current robot pose (= starting position)
  void setRobotPose(Ogre::Vector3 position, Ogre::Quaternion orientation);
  // Frame ID is updated
  void setFrameID(QString frameID);
  // invoked from clicking undo button set to previous step plan in list previous_step_plans
  void setPreviousStepPlan();
  // called after interaction with foot, when foot is dropped, to update current step plan
  void acceptModifiedStepPlan();
  // The step plan positions will be updated to fit the current terrain
  void updateStepPlanPositions();
  // Called to update the cost after step plan has been modified [TODO]
  void updateStepPlanCost(vigir_footstep_planning_msgs::StepPlan step_plan);
  // Step Plan is reset to empty message
  void resetStepPlan();
  // Step Plan will be cut at last_index, last_index is the index of the step which will be last
  void trimStepPlan(int last_index);
  // Handle a generated sequence by updating cost
  void handleSequence(vigir_footstep_planning_msgs::StepPlan sequence);

Q_SIGNALS:
  // emitted from checkSteps() to update step visuals to display their validity
  void stepValidUpdate(unsigned int step_index, bool valid);
  // if a step plan has been modified such that the number of step have changed (trimStepPlan, deleted step, set previous)
  void createdStepPlan(vigir_footstep_planning_msgs::StepPlan step_plan);
  // if a step plan has been modified without changing the number of steps the positions etc will only be updated (update functions, setPrevious, acceptModified)
  void updatedStepPlan(vigir_footstep_planning_msgs::StepPlan step_plan);
  // signal to communicate that execution has started, this will invoke initialization of the progress bar
  void executionStarted(int nofSteps);
  // signal with execution feedback emitting last finished step, this is displayed in the progress bar
  void executionFeedback(int last_performed);
  // signals to pass information to be displayed in output [TODO]
  void actionClientConnected(QString name, bool connected);
  void displayError(QString message);
  void displayInfo(QString message);
  void displaySuccess(QString message);


private:
  // Action Client Callbacks:
  void editStepCallback(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::EditStepResultConstPtr& result);
  void executeStepPlanCallback(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::ExecuteStepPlanResultConstPtr& result);
  void executeStepPlanFeedbackCallback(const vigir_footstep_planning_msgs::ExecuteStepPlanFeedbackConstPtr& feedback);
  void updateStepPlanCallback(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::UpdateStepPlanResultConstPtr& result);
  // called to start update_step_plan action client with the wanted update mode (called by updateStepPlanPositions, updateStepPlanCost)
  void updateStepPlan(vigir_footstep_planning_msgs::UpdateMode update_mode, vigir_footstep_planning_msgs::StepPlan step_plan);
  // combine 2 step plans after a step has been deleted [TODO]
  void combineStepPlans(std::vector<StepPlanMsg>& step_plans);
  // returns true if errors were found false if not
  bool checkForErrors(ErrorStatusMsg error_status);

  //stores the last step plans, previous_step_plans[size()-1] = current_step_plan
  std::vector<vigir_footstep_planning_msgs::StepPlan> previous_step_plans;
  // the currently displayed step plan
  StepPlanMsg current_step_plan;

  ros::NodeHandle nh;
  ros::Publisher robot_pose_publisher;
  // Action Clients:
  EditStepActionClient edit_step_ac;
  ExecuteStepPlanActionClient execute_step_plan_ac;
  UpdateStepPlanActionClient update_step_plan_ac;
  UpdateFootActionClient update_foot_ac;
//  SetStepPlanActionClient set_step_plan_ac;
 // GetStepPlanActionClient get_step_plan_ac;

  std::string frame_id;

// For Execution Status:
  int execution_state;
  int last_performed_step;
// ------
  bool update_positions;

  // Keep track of action client connection
  bool edit_step_connected;
  bool execute_connected;
  bool update_plan_connected;
  bool update_foot_connected;
};


} // end namespace vigir_footstep_planning_rviz_plugin

#endif // STEP_PLAN_HELPER_H
