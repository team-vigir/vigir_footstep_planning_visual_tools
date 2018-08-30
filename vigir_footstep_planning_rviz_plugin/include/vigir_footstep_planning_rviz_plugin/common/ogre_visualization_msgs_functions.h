#ifndef OGRE_VISUALIZATION_MSGS_FUNCTIONS_H
#define OGRE_VISUALIZATION_MSGS_FUNCTIONS_H

#include <geometry_msgs/Pose.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>


inline Ogre::Vector3 getOgreVector(const geometry_msgs::Point& position)
{
  return Ogre::Vector3(position.x, position.y, position.z);
}

inline Ogre::Quaternion getOgreQuaternion(const geometry_msgs::Quaternion& orientation)
{
  return Ogre::Quaternion(orientation.w, orientation.x, orientation.y, orientation.z);
}

inline geometry_msgs::Point getPositionMsg(const Ogre::Vector3& position)
{
  geometry_msgs::Point p;
  p.x = position.x;
  p.y = position.y;
  p.z = position.z;
  return p;
}

inline geometry_msgs::Quaternion getQuaternionMsg(const Ogre::Quaternion& orientation)
{
  geometry_msgs::Quaternion o;
  o.w = orientation.w;
  o.x = orientation.x;
  o.y = orientation.y;
  o.z = orientation.z;
  return o;
}

inline void getOgrePose(const geometry_msgs::Pose& pose_in, Ogre::Vector3& position_out, Ogre::Quaternion& orientation_out)
{
  position_out = getOgreVector(pose_in.position);
  orientation_out = getOgreQuaternion(pose_in.orientation);
}

inline void getPoseMsg(geometry_msgs::Pose& pose_out, const Ogre::Vector3& position_in, const Ogre::Quaternion& orientation_in)
{
  pose_out.position = getPositionMsg(position_in);
  pose_out.orientation = getQuaternionMsg(orientation_in);
}
#endif
