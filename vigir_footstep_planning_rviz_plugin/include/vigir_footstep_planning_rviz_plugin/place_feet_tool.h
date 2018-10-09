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


#ifndef PLACE_FEET_TOOL_H
#define PLACE_FEET_TOOL_H

#ifndef Q_MOC_RUN

#include <rviz/tool.h>
#include <QKeyEvent>
#include <actionlib/client/simple_action_client.h>
#include <vigir_footstep_planning_rviz_plugin/common/common.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>

#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ViewportMouseEvent;
class RenderPanel;
}

typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::GenerateFeetPoseAction> GenerateFeetPoseActionClient;
typedef vigir_footstep_planning_msgs::GenerateFeetPoseResultConstPtr GenerateFeetPoseResult;

namespace vigir_footstep_planning_rviz_plugin
{

class FeetVisual;

// - Tool handling first placing of a pair of feet (either goal (mode=GOAL_FEET) or start (mode=START_FEET)
// - Generating of valid feet message for given position & orientation
class PlaceFeetTool: public rviz::Tool
{
  Q_OBJECT
public:// A tool to place a pair of feet representing the goal pose of a step plan
  PlaceFeetTool();
  ~PlaceFeetTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );
  virtual int processKeyEvent (QKeyEvent *event, rviz::RenderPanel *panel);

  void setMode(PlaceFeetMode mode);

public Q_SLOTS:
  void setValidFeet(Ogre::Vector3 position, Ogre::Quaternion orientation, std::string frame_id, FeetType type);

Q_SIGNALS:
  void newStartPose(vigir_footstep_planning_msgs::Feet goal_feet);
  void newGoalPose(vigir_footstep_planning_msgs::Feet start_feet);

private:
  void setValidGoalCallback(const actionlib::SimpleClientGoalState& state, const GenerateFeetPoseResult& result);
  void setValidStartCallback(const actionlib::SimpleClientGoalState& state, const GenerateFeetPoseResult& result);
  void setRobotPose(const Ogre::Vector3& position, const Ogre::Quaternion& orientation, const std::string& frame_id);

  ros::NodeHandle nh;
  ros::Publisher robot_pose_publisher;
  GenerateFeetPoseActionClient generate_feet_ac;

  Ogre::SceneNode* moving_feet_node_;
  FeetVisual* moving_feet_;

  PlaceFeetMode mode;
};

} // end namespace vigir_footstep_planning_rviz_plugin

#endif // PLACE_FEET_TOOL_H
