#include "decomp_rviz_plugins/bound_visual.hpp"

namespace decomp_rviz_plugins
{

BoundVisual::BoundVisual(
    Ogre::SceneManager * scene_manager,
    Ogre::SceneNode * parent_node)
: scene_manager_(scene_manager)
{
  frame_node_ = parent_node->createChildSceneNode();
}

BoundVisual::~BoundVisual()
{
  scene_manager_->destroySceneNode(frame_node_);
}

void BoundVisual::setMessage(const vec_E<vec_Vec3f> & bds)
{
  objs_.clear();
  if (bds.empty()) {
    return;
  }

  // allocate one BillboardLine per face
  objs_.resize(bds.size());
  for (size_t i = 0; i < bds.size(); ++i) {
    objs_[i].reset(
      new rviz_rendering::BillboardLine(scene_manager_, frame_node_));
  }

  // draw each face as a closed loop
  for (size_t fi = 0; fi < bds.size(); ++fi) {
    const auto & vs = bds[fi];
    auto & line = *objs_[fi];
    for (size_t i = 0; i <= vs.size(); ++i) {
      size_t idx = (i < vs.size() ? i : 0);
      const auto & v = vs[idx];
      if (!std::isnan(v(0))) {
        line.addPoint(Ogre::Vector3(v(0), v(1), v(2)));
      }
    }
  }
}

void BoundVisual::setFramePosition(const Ogre::Vector3 & position)
{
  frame_node_->setPosition(position);
}

void BoundVisual::setFrameOrientation(const Ogre::Quaternion & orientation)
{
  frame_node_->setOrientation(orientation);
}

void BoundVisual::setColor(float r, float g, float b, float a)
{
  for (auto & ptr : objs_) {
    ptr->setColor(r, g, b, a);
  }
}

void BoundVisual::setScale(float s)
{
  for (auto & ptr : objs_) {
    ptr->setLineWidth(s);
  }
}

}  // namespace decomp_rviz_plugins
