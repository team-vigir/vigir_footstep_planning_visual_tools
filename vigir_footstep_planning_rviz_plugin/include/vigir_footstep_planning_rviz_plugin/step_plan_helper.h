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
  bool checkConnection();
  void addStep(const std::string& frame_id, const Ogre::Vector3& position, const Ogre::Quaternion& orientation, unsigned int which_foot, unsigned int step_index);
  void checkSteps();

public Q_SLOTS:
  void editStep(vigir_footstep_planning_msgs::EditStep edit_step);
  void executeStepPlan();
  void setCurrentStepPlan(vigir_footstep_planning_msgs::StepPlan step_plan);
  void updateFoot(vigir_footstep_planning_msgs::Foot foot, unsigned int step_index);
  void setRobotPose(Ogre::Vector3 position, Ogre::Quaternion orientation);
  void setFixedFrame(QString fixed_frame);
  void setPreviousStepPlan();
  void acceptModifiedStepPlan();
Q_SIGNALS:
  void stepValidUpdate(unsigned int step_index, bool valid);
  void createdStepPlan(vigir_footstep_planning_msgs::StepPlan step_plan);

private:
  void connectToActionServer();
  void editStepCallback(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::EditStepResultConstPtr& result);
  void executeStepPlanCallback(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::ExecuteStepPlanResultConstPtr& result);
  void updateStepPlanCallback(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::UpdateStepPlanResultConstPtr& result);
  void combineStepPlans(std::vector<StepPlanMsg>& step_plans);
  bool checkForErrors(ErrorStatusMsg error_status);
  //previous_step_plans[size()] = current_step_plan
  std::vector<vigir_footstep_planning_msgs::StepPlan> previous_step_plans;
  StepPlanMsg current_step_plan;

  ros::NodeHandle nh;
  ros::Publisher robot_pose_publisher;
  // Action Clients:
  EditStepActionClient edit_step_ac;
  ExecuteStepPlanActionClient execute_step_plan_ac;
  UpdateStepPlanActionClient update_step_plan_ac;
//  SetStepPlanActionClient set_step_plan_ac;
 // GetStepPlanActionClient get_step_plan_ac;


  std::string fixed_frame_;

  bool step_edited; //true when current_step_plan is different to back() of previous_step_plans

};


} // end namespace vigir_footstep_planning_rviz_plugin

#endif // STEP_PLAN_HELPER_H
