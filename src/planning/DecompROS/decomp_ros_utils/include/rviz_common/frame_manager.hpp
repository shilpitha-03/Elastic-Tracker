// vendored frame_manager.hpp from ign-rviz-common for Humble
#ifndef RVIZ_COMMON_FRAME_MANAGER_HPP_
#define RVIZ_COMMON_FRAME_MANAGER_HPP_

#include <string>
#include <map>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

namespace rviz_common
{

/// A minimal stub of FrameManager so your plugin compiles
class FrameManager
{
public:
  FrameManager() = default;
  ~FrameManager() = default;

  /// Normally transforms stamped data into Ogre scene frames.
  /// Here we provide no‑ops so code that calls these will link.
  bool getTransform(const std::string & /*target_frame*/,
                    const std::string & /*source_frame*/,
                    Ogre::Vector3 & position,
                    Ogre::Quaternion & orientation) const
  {
    position = Ogre::Vector3::ZERO;
    orientation = Ogre::Quaternion::IDENTITY;
    return true;
  }
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON_FRAME_MANAGER_HPP_

