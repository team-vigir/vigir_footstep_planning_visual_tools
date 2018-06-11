#ifndef REQUEST_HANDLER_BASE_H
#define REQUEST_HANDLER_BASE_H

#ifndef Q_MOC_RUN

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>
#include <QObject>

#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#endif

typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::StepPlanRequestAction> StepPlanRequestActionClient;
typedef vigir_footstep_planning_msgs::StepPlanRequestResultConstPtr RequestResult;

typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::GenerateFeetPoseAction> GenerateFeetPoseActionClient;
typedef vigir_footstep_planning_msgs::GenerateFeetPoseResultConstPtr GenerateFeetPoseResult;


typedef vigir_footstep_planning_msgs::ErrorStatus ErrorStatusMsg;
typedef vigir_footstep_planning_msgs::StepPlan StepPlanMsg;
typedef vigir_footstep_planning_msgs::StepPlanRequest RequestMsg;
typedef vigir_footstep_planning_msgs::Feet FeetMsg;
typedef vigir_footstep_planning_msgs::Foot FootMsg;

namespace vigir_footstep_planning_rviz_plugin
{
// Base class for pattern and planning request handlers
// both pattern and planning request handlers can:
//  - send a request for step plan and emit result step plan
//  - ask for feedback and emit it
//  - set general properties of request message
//  - connect to action server and emit when connection has changed
//  - ask for current pose of start feet and emit it
//  - add step plan to existing one by:
//      -> using current goal as start for computing next step plan
//      -> appending next step plan to current and emitting resulting step plan
//  - set current goal either to last step or to chosen step
class RequestHandlerBase : public QObject
{
  Q_OBJECT
public:
  RequestHandlerBase(QObject *parent = 0);
  virtual ~RequestHandlerBase();

  RequestMsg* request_;
  StepPlanRequestActionClient ac;
  GenerateFeetPoseActionClient generate_feet_ac;
  vigir_footstep_planning_msgs::StepPlan current_step_plan;

  void initialize();

  bool checkConnection();
  void sendRequest();
  void cancelGoals();
  void requestStartPose();
  void setCurrentStepPlan(const vigir_footstep_planning_msgs::StepPlan& step_plan);
  void setHeaderStamp();
  void setPlanningMode(int planning_mode);
  void computeShift(float& x_shift, float& y_shift, float& z_shift, const geometry_msgs::Quaternion &orientation);
  int last_step_index;
  int replan_goal_index;

  virtual void appendStepPlan(StepPlanMsg add);

public Q_SLOTS:
  void connectToActionServer();
  void setFrameID(QString frame_id);
  void setFeedbackRequested(bool requested);
  void addStepPlan();
  void setCurrentGoal(int last_index);
  //start_foot_selection = RequestMsg::RIGHT or RequestMsg::LEFT
  void setStartFoot(int start_foot_selection);
  void setParameterSet(QString parameter_set_name);

Q_SIGNALS:
  void createdStepPlan(vigir_footstep_planning_msgs::StepPlan s);
  void receivedPlanningFeedback(vigir_footstep_planning_msgs::PlanningFeedback feedback);
  void startFeetAnswer(vigir_footstep_planning_msgs::Feet start);
  void actionClientConnected(QString name, bool connected);
  void displayError(QString message);
  void displayInfo(QString message);
  void displaySuccess(QString message);

private:
  bool feedback_requested_;

  void addStepPlanCallback(const actionlib::SimpleClientGoalState& state, const RequestResult& result);
  void setCurrentGoalCallback(const actionlib::SimpleClientGoalState& state, const GenerateFeetPoseResult& result);
  void startPoseCallback(const actionlib::SimpleClientGoalState& state, const GenerateFeetPoseResult& result);
  void resultCallback(const actionlib::SimpleClientGoalState& state, const RequestResult& result);
  void feedbackCallback(const vigir_footstep_planning_msgs::StepPlanRequestFeedbackConstPtr& feedback);

};

} // end namespace vigir_footstep_planning_rviz_plugin

#endif // REQUEST_HANDLER_H
