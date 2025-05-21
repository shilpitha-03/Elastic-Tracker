// include/rviz_rendering/objects/mesh_shape.hpp

#ifndef RVIZ_RENDERING_OBJECTS_MESH_SHAPE_HPP_
#define RVIZ_RENDERING_OBJECTS_MESH_SHAPE_HPP_

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreManualObject.h>
#include <memory>

namespace rviz_rendering
{
namespace objects
{

/// A helper class to build dynamic triangle meshes in Ogre.
class MeshShape
{
public:
  MeshShape(Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node = nullptr);
  ~MeshShape();

  void beginTriangles();
  void addVertex(const Ogre::Vector3 & position, const Ogre::Vector3 & normal);
  void addTriangle(unsigned int a, unsigned int b, unsigned int c);
  void endTriangles();
  void clear();

private:
  std::unique_ptr<Ogre::ManualObject> manual_object_;
  Ogre::SceneNode * scene_node_;
  unsigned int current_vertex_index_;
};

}  // namespace objects
}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING_OBJECTS_MESH_SHAPE_HPP_
