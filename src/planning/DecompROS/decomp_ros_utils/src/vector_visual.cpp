#include "decomp_rviz_plugins/vector_visual.hpp"

namespace decomp_rviz_plugins
{

VectorVisual::VectorVisual(Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node)
: scene_manager_(scene_manager)
{
  frame_node_ = parent_node->createChildSceneNode();
}

VectorVisual::~VectorVisual()
{
  scene_manager_->destroySceneNode(frame_node_);
}

void VectorVisual::setMessage(const vec_E_Pair & vs)
{
  objs_.clear();
  vs_ = vs;
  if (vs.empty()) return;

  objs_.reserve(vs.size());
  for (size_t i = 0; i < vs.size(); ++i) {
    auto arrow = std::make_unique<rviz_rendering::Arrow>(scene_manager_, frame_node_);
    objs_.push_back(std::move(arrow));
  }

  for (size_t i = 0; i < vs.size(); ++i) {
    const auto & v = vs[i];
    Vec3f origin = v.first;
    Vec3f dir = v.second.normalized();
    float length = v.second.norm() * s_;

    objs_[i]->setPosition(Ogre::Vector3(origin.x(), origin.y(), origin.z()));
    objs_[i]->setDirection(Ogre::Vector3(dir.x(), dir.y(), dir.z()));
    objs_[i]->setScale(Ogre::Vector3(s_, s_, s_));

    float shaft_length = 0.7f * length;
    float head_length = 0.3f * length;
    float shaft_radius = length / 8.0f;
    float head_radius  = length / 5.0f;
    objs_[i]->set(shaft_length, shaft_radius, head_length, head_radius);
  }
}

void VectorVisual::setFramePosition(const Ogre::Vector3 & position)
{
  frame_node_->setPosition(position);
}

void VectorVisual::setFrameOrientation(const Ogre::Quaternion & orientation)
{
  frame_node_->setOrientation(orientation);
}

void VectorVisual::setColor(float r, float g, float b, float a)
{
  for (auto & arrow : objs_) {
    arrow->setColor(r, g, b, a);
  }
}

void VectorVisual::setScale(float s)
{
  s_ = s;
  for (size_t i = 0; i < objs_.size(); ++i) {
    float length = vs_[i].second.norm() * s_;
    float shaft_length = 0.7f * length;
    float head_length = 0.3f * length;
    float shaft_radius = length / 8.0f;
    float head_radius  = length / 5.0f;
    objs_[i]->set(shaft_length, shaft_radius, head_length, head_radius);
  }
}

} // namespace decomp_rviz_plugins
