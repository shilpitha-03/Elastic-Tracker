// src/objects/mesh_shape.cpp

#include "rviz_rendering/objects/mesh_shape.hpp"

namespace rviz_rendering
{
namespace objects
{

MeshShape::MeshShape(Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node)
: current_vertex_index_(0)
{
  manual_object_.reset(new Ogre::ManualObject());
  scene_node_ = (parent_node)
    ? parent_node->createChildSceneNode()
    : scene_manager->getRootSceneNode()->createChildSceneNode();
  scene_node_->attachObject(manual_object_.get());
}

MeshShape::~MeshShape()
{
  // Ogre will clean up the ManualObject when the scene node is destroyed
}

void MeshShape::beginTriangles()
{
  manual_object_->clear();
  manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
  current_vertex_index_ = 0;
}

void MeshShape::addVertex(const Ogre::Vector3 & position, const Ogre::Vector3 & normal)
{
  manual_object_->position(position);
  manual_object_->normal(normal);
  ++current_vertex_index_;
}

void MeshShape::addTriangle(unsigned int a, unsigned int b, unsigned int c)
{
  manual_object_->triangle(a, b, c);
}

void MeshShape::endTriangles()
{
  manual_object_->end();
}

void MeshShape::clear()
{
  manual_object_->clear();
}

}  // namespace objects
}  // namespace rviz_rendering
