#ifndef REQUEST_HANDLER_H
#define REQUEST_HANDLER_H

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>
#include <QObject>

#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::StepPlanRequestAction> StepPlanRequestActionClient;

namespace vigir_footstep_planning_rviz_plugin
{

class RequestHandler : public QObject
{
  Q_OBJECT
public:
  RequestHandler(QObject *parent = 0);
  virtual ~RequestHandler();
  // send Request Methods:
  void sendRequest();
  void resultCallback(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::StepPlanRequestResultConstPtr& result);
  void feedbackCallback(const vigir_footstep_planning_msgs::StepPlanRequestFeedbackConstPtr& feedback);
  void sendPatternRequest(int patternMode);
  void sendSimplePatternRequest(int patternMode, int no_steps, double step_dist, int turn_angle, double dz);
  void sendPlanningRequest();

  // set Methods:
  void setHeader();
  void setPlanningMode(int planning_mode);
  void setPatternMode(int mode);

  //get Methods
  vigir_footstep_planning_msgs::StepPlan getResultStepPlan();

Q_SIGNALS:
  void createdStepPlan(vigir_footstep_planning_msgs::StepPlan s);
  void receivedPlanningFeedback(vigir_footstep_planning_msgs::PlanningFeedback feedback);

private:
  vigir_footstep_planning_msgs::StepPlanRequest* request_;
  bool ready_to_send_;
  StepPlanRequestActionClient ac;

  vigir_footstep_planning_msgs::StepPlanRequestResultConstPtr result_;
  bool feedback_requested_;

public Q_SLOTS:
  //FrameID:
  void setFrameID(QString frame_id);
  void setGoal(Ogre::Vector3 position, Ogre::Quaternion orientation);
  void setFeedbackRequested(bool requested);

private Q_SLOTS:
  //set Pattern Parameters:
  void setNoSteps(int no_steps);
  void setStepDistance(double step_dist);
  void setSideStep(double step_side);
  void setTurnAngle(int turn_angle);
  void setdz(double dz);

  //more Pattern Parameters
  void setStartStepIndex(int start_index);
  void setRoll(int roll);
  void setPitch(int pitch);
  void setStartFoot(int start_foot);

  //checkboxes:
  void setClosingStep(int state);
  void setExtraSeperation(int state);
  void setUseTerrainModel(int state);
  void setOverride3D(int state);

};


} // end namespace vigir_footstep_planning_rviz_plugin

#endif // REQUEST_HANDLER_H
