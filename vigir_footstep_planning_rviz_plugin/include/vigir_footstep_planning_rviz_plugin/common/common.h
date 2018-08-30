#ifndef COMMON_H
#define COMMON_H

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>
typedef vigir_footstep_planning_msgs::Foot FootMsg;
typedef vigir_footstep_planning_msgs::Feet FeetMsg;
typedef vigir_footstep_planning_msgs::Step StepMsg;
typedef vigir_footstep_planning_msgs::StepPlan StepPlanMsg;
typedef vigir_footstep_planning_msgs::StepPlanRequest RequestMsg;
typedef vigir_footstep_planning_msgs::FeetPoseRequest FeetPoseRequestMsg;
typedef vigir_footstep_planning_msgs::ErrorStatus ErrorStatusMsg;

namespace vigir_footstep_planning_rviz_plugin
{
enum InteractionMode{PLANE = 1, SIXDOF = 2,  FULLSIXDOF = 3};

enum FeetType{GOAL = 0, START = 1};

enum PlantFeetMode{GOAL_FEET = 0, START_FEET = 1};

enum FootIndex{LEFT = 0, RIGHT = 1};

}
#endif

