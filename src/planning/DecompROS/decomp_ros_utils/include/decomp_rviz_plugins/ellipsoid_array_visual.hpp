#ifndef DECOMP_RVIZ_PLUGINS_ELLIPSOID_ARRAY_VISUAL_HPP_
#define DECOMP_RVIZ_PLUGINS_ELLIPSOID_ARRAY_VISUAL_HPP_

#include <memory>
#include <vector>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <rviz_rendering/objects/shape.hpp>

#include <decomp_ros_msgs/msg/ellipsoid_array.hpp>
#include <decomp_geometry/ellipsoid.h>
#include <Eigen/Eigenvalues>

namespace decomp_rviz_plugins
{

class EllipsoidArrayVisual
{
public:
  EllipsoidArrayVisual(
    Ogre::SceneManager * scene_manager,
    Ogre::SceneNode    * parent_node);
  ~EllipsoidArrayVisual();

  /// Feed in the newest message
  void setMessage(decomp_ros_msgs::msg::EllipsoidArray::ConstSharedPtr msg);

  /// Control the frame node’s pose
  void setFramePosition(const Ogre::Vector3 & position);
  void setFrameOrientation(const Ogre::Quaternion & orientation);

  /// Change color & alpha of all ellipsoids
  void setColor(float r, float g, float b, float a);

private:
  std::vector<std::unique_ptr<rviz_rendering::Shape>> objs_;
  Ogre::SceneNode   * frame_node_;
  Ogre::SceneManager* scene_manager_;
};

}  // namespace decomp_rviz_plugins

#endif  // DECOMP_RVIZ_PLUGINS_ELLIPSOID_ARRAY_VISUAL_HPP_
