#ifndef PLANNING_REQUEST_HANDLER_H
#define PLANNING_REQUEST_HANDLER_H

#include <vigir_footstep_planning_rviz_plugin/request_handler_base.h>

typedef vigir_footstep_planning_msgs::FeetPoseRequest FeetPoseRequestMsg;
typedef vigir_footstep_planning_msgs::StepPlan StepPlanMsg;
typedef vigir_footstep_planning_msgs::Step StepMsg;

namespace vigir_footstep_planning_rviz_plugin
{
// Class to handle planning requests
// Additionaly to the funcitonality of the base class planning request handler can:
//  - receive a request to request the correct goal pose for a given position and emit the resulting current goal pose
//  - send a planning request
class PlanningRequestHandler : public RequestHandlerBase
{
  Q_OBJECT
public:
  PlanningRequestHandler(QObject *parent = 0);
  virtual ~PlanningRequestHandler();

  void sendPlanningRequest(bool append);
  void setGoal(Ogre::Vector3 position, Ogre::Quaternion orientation);
  void replanToIndex(int index);
  void appendStepPlan(StepPlanMsg add) override;

private:
  void setReplanGoal(int index);
  void setReplanGoalCallback(const actionlib::SimpleClientGoalState& state, const GenerateFeetPoseResult& result);

  void goalPoseCallback(const actionlib::SimpleClientGoalState& state, const GenerateFeetPoseResult& result);

private Q_SLOTS:
  void setStartStepIndex(int start_step);
  void setMaxPlanningTime(double t);
  void setMaxNofSteps(int noSteps);
  void setMaxPathLengthRatio(double ratio);


Q_SIGNALS:
  void goalFeetAnswer(vigir_footstep_planning_msgs::Feet goal);

};


} // end namespace vigir_footstep_planning_rviz_plugin

#endif // PLANNING_REQUEST_HANDLER_H
