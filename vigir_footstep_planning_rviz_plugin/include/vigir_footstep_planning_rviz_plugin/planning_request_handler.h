#ifndef PLANNING_REQUEST_HANDLER_H
#define PLANNING_REQUEST_HANDLER_H

#include <vigir_footstep_planning_rviz_plugin/request_handler_base.h>


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
 // void setGoal(Ogre::Vector3 position, Ogre::Quaternion orientation);
  void setGoal(vigir_footstep_planning_msgs::Feet goal_feet);
  void replanToIndex(int index);

public Q_SLOTS:
  void setStartStepIndex(int start_step);
  void setMaxPlanningTime(double t);
  void setMaxNofSteps(int noSteps);
  void setMaxPathLengthRatio(double ratio);
  void setActivateOnPlaceFeet(bool activate); //start step plan generation when feet are placed
  void setAppend(bool appending);

protected:
  void appendStepPlan(StepPlanMsg add) override;

private:
  void replanToIndexCallback(const actionlib::SimpleClientGoalState& state, const GenerateFeetPoseResult& result);

  void goalPoseCallback(const actionlib::SimpleClientGoalState& state, const GenerateFeetPoseResult& result);

  bool activate_on_place_feet;
  bool append;
};


} // end namespace vigir_footstep_planning_rviz_plugin

#endif // PLANNING_REQUEST_HANDLER_H
