#ifndef BOUND_VISUAL_HPP
#define BOUND_VISUAL_HPP

// Ogre and RViz 2‐style billboard line
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <rviz_rendering/objects/billboard_line.hpp>

#include <decomp_basis/data_type.h>  // your existing dependency

namespace decomp_rviz_plugins
{

class BoundVisual
{
public:
  BoundVisual(Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node);
  ~BoundVisual();

  /// \brief set the faces of the polytope (each face is a vector of Vec3f)
  void setMessage(const vec_E<vec_Vec3f> & bds);

  /// \brief control the scene‐node frame pose
  void setFramePosition(const Ogre::Vector3 & position);
  void setFrameOrientation(const Ogre::Quaternion & orientation);

  /// \brief color & scale of all lines
  void setColor(float r, float g, float b, float a);
  void setScale(float s);

private:
  /// each face has its own billboard‐line
  std::vector<std::unique_ptr<rviz_rendering::BillboardLine>> objs_;
  Ogre::SceneNode * frame_node_;
  Ogre::SceneManager * scene_manager_;
};

}  // namespace decomp_rviz_plugins

#endif  // BOUND_VISUAL_HPP
