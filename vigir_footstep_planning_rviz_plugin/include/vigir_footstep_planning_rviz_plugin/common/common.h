//=================================================================================================
// Copyright (c) 2018, Stephanie Ferreira, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef COMMON_H
#define COMMON_H

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>
typedef vigir_footstep_planning_msgs::Foot FootMsg;
typedef vigir_footstep_planning_msgs::Feet FeetMsg;
typedef vigir_footstep_planning_msgs::Step StepMsg;
typedef vigir_footstep_planning_msgs::EditStep EditStepMsg;
typedef vigir_footstep_planning_msgs::StepPlan StepPlanMsg;
typedef vigir_footstep_planning_msgs::StepPlanRequest RequestMsg;
typedef vigir_footstep_planning_msgs::FeetPoseRequest FeetPoseRequestMsg;
typedef vigir_footstep_planning_msgs::ErrorStatus ErrorStatusMsg;

namespace vigir_footstep_planning_rviz_plugin
{
enum InteractionMode{PLANE = 1, SIXDOF = 2,  FULLSIXDOF = 3};

enum FeetType{GOAL = 0, START = 1};

enum PlaceFeetMode{GOAL_FEET = 0, START_FEET = 1};

enum FootIndex{LEFT = 0, RIGHT = 1};

}
#endif

