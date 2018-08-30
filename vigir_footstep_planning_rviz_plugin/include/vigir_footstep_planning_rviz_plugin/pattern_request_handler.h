#ifndef PATTERN_REQUEST_HANDLER_H
#define PATTERN_REQUEST_HANDLER_H

#include <vigir_footstep_planning_rviz_plugin/request_handler_base.h>

namespace vigir_footstep_planning_rviz_plugin
{
// Class to handle pattern planning requests
// Additionaly to the funcitonality of the base class pattern request handler can:
//  - send pattern request to a given pattern mode
//  - set parameters for pattern planning
//  - set current goal to position of last step (by using base request handlers functions)
//     (which is done always when a step plan is created)
class PatternRequestHandler : public RequestHandlerBase
{
  Q_OBJECT
public:
  PatternRequestHandler(QObject *parent = 0);
  virtual ~PatternRequestHandler();
  void sendPatternRequest(int patternMode, bool append);

public Q_SLOTS:
  //set Pattern Parameters:
  void setNoSteps(int no_steps);
  void setStepDistance(double step_dist);
  void setSideStep(double step_side);
  void setTurnAngle(int turn_angle);
  void setdz(double dz);

  //more Pattern Parameters
  void setStartStepIndex(int start_step);
  void setRoll(int roll);
  void setPitch(int pitch);

  //checkboxes:
  void setClosingStep(int state);
  void setExtraSeperation(int state);
  void setUseTerrainModel(int state);
  void setOverride3D(int state);

protected:
  void appendStepPlan(StepPlanMsg add) override;

private:
  void setPatternMode(int mode);
};
} // end namespace vigir_footstep_planning_rviz_plugin

#endif // PATTERN_REQUEST_HANDLER_H
