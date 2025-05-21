// #ifndef DECOMP_RVIZ_PLUGINS_MESH_VISUAL_HPP_
// #define DECOMP_RVIZ_PLUGINS_MESH_VISUAL_HPP_

// #include <memory>
// #include <vector>

// #include <OgreSceneManager.h>
// #include <OgreSceneNode.h>

// #include "rviz_rendering/objects/mesh_shape.hpp"
// #include <decomp_geometry/polyhedron.h>
// #include <Eigen/Eigenvalues>

// namespace decomp_rviz_plugins
// {

// class MeshVisual
// {
// public:
//   MeshVisual(
//     Ogre::SceneManager * scene_manager,
//     Ogre::SceneNode    * parent_node);

//   virtual ~MeshVisual();

//   /// Provide a list of faces (each face is a vector of Vec3f)
//   void setMessage(const vec_E<vec_Vec3f> & bds);

//   /// Control the scene‐node frame pose
//   void setFramePosition(const Ogre::Vector3 & position);
//   void setFrameOrientation(const Ogre::Quaternion & orientation);

//   /// Set uniform color & alpha
//   void setColor(float r, float g, float b, float a);

// private:
//   std::unique_ptr<rviz_rendering::objects::MeshShape> obj_;
//   Ogre::SceneNode   * frame_node_;
//   Ogre::SceneManager* scene_manager_;
// };

// }  // namespace decomp_rviz_plugins

// #endif  // DECOMP_RVIZ_PLUGINS_MESH_VISUAL_HPP_















#ifndef DECOMP_RVIZ_PLUGINS_MESH_VISUAL_HPP_
#define DECOMP_RVIZ_PLUGINS_MESH_VISUAL_HPP_

#include <memory>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include "rviz_rendering/objects/mesh_shape.hpp"   // vendored

namespace decomp_rviz_plugins
{

class MeshVisual
{
public:
  MeshVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  ~MeshVisual();

  void setMessage(const vec_E<vec_Vec3f>& bds);
  void setFramePosition(const Ogre::Vector3& pos);
  void setFrameOrientation(const Ogre::Quaternion& ori);
  void setColor(float r, float g, float b, float a);

private:
  Ogre::SceneManager* scene_manager_;
  Ogre::SceneNode* frame_node_;
  std::unique_ptr<rviz_rendering::objects::MeshShape> obj_;  // fixed
};

}  // namespace decomp_rviz_plugins

#endif  // DECOMP_RVIZ_PLUGINS_MESH_VISUAL_HPP_
