#include "decomp_rviz_plugins/mesh_visual.hpp"

#include <cmath>  // for std::isnan

namespace decomp_rviz_plugins
{

MeshVisual::MeshVisual(
  Ogre::SceneManager * scene_manager,
  Ogre::SceneNode    * parent_node)
: scene_manager_(scene_manager)
{
  frame_node_ = parent_node->createChildSceneNode();
  obj_.reset(new rviz_rendering::objects::MeshShape(scene_manager_, frame_node_));
}

MeshVisual::~MeshVisual()
{
  scene_manager_->destroySceneNode(frame_node_);
}

void MeshVisual::setMessage(const vec_E<vec_Vec3f> & bds)
{
  obj_->clear();

  if (bds.empty()) {
    return;
  }

  obj_->beginTriangles();
  int vertex_index = 0;

  for (auto const & vs : bds) {
    if (vs.size() < 3) {
      continue;
    }
    // compute normal from first three vertices
    Vec3f p0 = vs[0], p1 = vs[1], p2 = vs[2];
    Vec3f n  = (p2 - p0).cross(p1 - p0).normalized();
    if (std::isnan(n(0))) {
      n = Vec3f(0, 0, -1);
    }
    Ogre::Vector3 normal(
      static_cast<float>(n(0)),
      static_cast<float>(n(1)),
      static_cast<float>(n(2)));

    int start_index = vertex_index;
    for (size_t i = 0; i < vs.size(); ++i, ++vertex_index) {
      // extract each vertex coordinate from Eigen::Vector3d
      const Vec3f & pt = vs[i];
      Ogre::Vector3 v(
        static_cast<float>(pt(0)),
        static_cast<float>(pt(1)),
        static_cast<float>(pt(2)));
      obj_->addVertex(v, normal);

      // after first two verts, form triangle (start, prev, current)
      if (i >= 2) {
        obj_->addTriangle(start_index, vertex_index - 1, vertex_index);
      }
    }
  }

  obj_->endTriangles();
}

void MeshVisual::setFramePosition(const Ogre::Vector3 & pos)
{
  frame_node_->setPosition(pos);
}

void MeshVisual::setFrameOrientation(const Ogre::Quaternion & ori)
{
  frame_node_->setOrientation(ori);
}

void MeshVisual::setColor(float r, float g, float b, float a)
{
  obj_->setColor(r, g, b, a);
}

}  // namespace decomp_rviz_plugins
