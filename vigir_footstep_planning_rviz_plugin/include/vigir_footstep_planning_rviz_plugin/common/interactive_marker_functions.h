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

#ifndef INTERACTIVE_MARKER_FUNCTIONS_H
#define INTERACTIVE_MARKER_FUNCTIONS_H

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <interactive_markers/menu_handler.h>


// interactive marker messages
typedef visualization_msgs::Marker MarkerMsg;
typedef visualization_msgs::InteractiveMarker InteractiveMarkerMsg;
typedef visualization_msgs::InteractiveMarkerControl InteractiveMarkerControlMsg;
typedef visualization_msgs::InteractiveMarkerFeedback InteractiveMarkerFeedbackMsg;
typedef visualization_msgs::InteractiveMarkerFeedbackConstPtr FeedbackConstPtr;

inline void normaliseQuaternion(geometry_msgs::Quaternion& orientation)
{
  Ogre::Quaternion o(orientation.w, orientation.x, orientation.y, orientation.z);
  o.normalise();
  orientation.w = o.w;
  orientation.x = o.x;
  orientation.y = o.y;
  orientation.z = o.z;
}

inline MarkerMsg makeSphereMarker(float radius, std_msgs::ColorRGBA color)
{
  MarkerMsg marker;

  marker.type = MarkerMsg::SPHERE;
  marker.scale.x = radius;
  marker.scale.y = radius;
  marker.scale.z = radius;

  marker.color = color;

  return marker;
}

inline InteractiveMarkerMsg makeInteractiveMarker(const std::string& name,
                                                            const std::string& frame_id,
                                                            const geometry_msgs::Pose& pose,
                                                            float scale)
{
  visualization_msgs::InteractiveMarker im;
  im.header.frame_id = frame_id;
  im.header.stamp = ros::Time::now();
  im.name = name;
  im.pose = pose;
  im.scale = scale;
  return im;
}

inline void addButtonControl(InteractiveMarkerMsg& im, const MarkerMsg& marker)
{
  InteractiveMarkerControlMsg control;
  control.interaction_mode = InteractiveMarkerControlMsg::BUTTON;
  control.markers.push_back( marker);
  control.always_visible = false;
  im.controls.push_back(control);
}

inline void addPlaneControl(InteractiveMarkerMsg& im, const MarkerMsg& marker, bool with_rotation)
{
  InteractiveMarkerControlMsg plane_control;
  plane_control.orientation.w = 1;
  plane_control.orientation.x = 0;
  plane_control.orientation.y = 1;
  plane_control.orientation.z = 0;
  normaliseQuaternion(plane_control.orientation);
  // Rotate
  if(with_rotation)
  {
    plane_control.name = "rotate_xy";
    plane_control.interaction_mode = InteractiveMarkerControlMsg::ROTATE_AXIS;
    im.controls.push_back(plane_control);
  }

  // Move Plane with box marker
  plane_control.markers.push_back(marker);
  plane_control.name = "move_xy";
  plane_control.interaction_mode = InteractiveMarkerControlMsg::MOVE_PLANE;
  plane_control.always_visible = false;

  im.controls.push_back(plane_control);
}

inline void addSixDOFControl(InteractiveMarkerMsg& im)
{
  InteractiveMarkerControlMsg control;
  // x direction
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  normaliseQuaternion(control.orientation);
  control.name = "rotate_x";
  control.interaction_mode = InteractiveMarkerControlMsg::ROTATE_AXIS;
  im.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControlMsg::MOVE_AXIS;
  im.controls.push_back(control);

  // y direction
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  normaliseQuaternion(control.orientation);
  control.name = "rotate_y";
  control.interaction_mode = InteractiveMarkerControlMsg::ROTATE_AXIS;
  im.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControlMsg::MOVE_AXIS;
  im.controls.push_back(control);

  // z direction
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  normaliseQuaternion(control.orientation);
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControlMsg::ROTATE_AXIS;
  im.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControlMsg::MOVE_AXIS;
  im.controls.push_back(control);
}

inline void addMoveSixDOFControl(InteractiveMarkerMsg& im, const MarkerMsg& marker)
{
  // Move 3D with box marker
  InteractiveMarkerControlMsg control;
  control.name = "move_3d";
  control.markers.push_back(marker);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
  control.always_visible = false;
  im.controls.push_back(control);
}
#endif
