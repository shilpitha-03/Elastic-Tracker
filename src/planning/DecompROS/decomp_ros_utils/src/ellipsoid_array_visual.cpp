#include "decomp_rviz_plugins/ellipsoid_array_visual.hpp"

namespace decomp_rviz_plugins
{

EllipsoidArrayVisual::EllipsoidArrayVisual(
    Ogre::SceneManager * scene_manager,
    Ogre::SceneNode    * parent_node)
: scene_manager_(scene_manager)
{
  frame_node_ = parent_node->createChildSceneNode();
}

EllipsoidArrayVisual::~EllipsoidArrayVisual()
{
  scene_manager_->destroySceneNode(frame_node_);
}

void EllipsoidArrayVisual::setMessage(
  decomp_ros_msgs::msg::EllipsoidArray::ConstSharedPtr msg)
{
  objs_.clear();
  if (msg->ellipsoids.empty()) {
    return;
  }

  // Validate no NaNs
  for (auto const & e : msg->ellipsoids) {
    if (std::isnan(e.d[0]) || std::isnan(e.d[1]) || std::isnan(e.d[2])) {
      return;
    }
    for (int k = 0; k < 9; ++k) {
      if (std::isnan(e.e[k])) {
        return;
      }
    }
  }

  // Create one sphere per ellipsoid
  objs_.resize(msg->ellipsoids.size());
  for (auto & shape : objs_) {
    shape.reset(new rviz_rendering::Shape(
      rviz_rendering::Shape::Type::Sphere, scene_manager_, frame_node_));
  }

  // Configure each sphere
  for (size_t i = 0; i < msg->ellipsoids.size(); ++i) {
    auto const & m = msg->ellipsoids[i];

    // build the 3×3 matrix E
    Mat3f E;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        E(r,c) = m.e[3*r + c];
      }
    }
    Eigen::SelfAdjointEigenSolver<Mat3f> es(E);

    // scale = 2×eigenvalues
    Ogre::Vector3 scale(
      2*es.eigenvalues()[0],
      2*es.eigenvalues()[1],
      2*es.eigenvalues()[2]);
    objs_[i]->setScale(scale);

    // position = d vector
    Ogre::Vector3 pos(m.d[0], m.d[1], m.d[2]);
    objs_[i]->setPosition(pos);

    // orientation from eigenvectors
    Quatf q(es.eigenvectors().determinant() * es.eigenvectors());
    Ogre::Quaternion ori(q.w(), q.x(), q.y(), q.z());
    objs_[i]->setOrientation(ori);
  }
}

void EllipsoidArrayVisual::setFramePosition(const Ogre::Vector3 & pos)
{
  frame_node_->setPosition(pos);
}

void EllipsoidArrayVisual::setFrameOrientation(const Ogre::Quaternion & ori)
{
  frame_node_->setOrientation(ori);
}

void EllipsoidArrayVisual::setColor(float r, float g, float b, float a)
{
  for (auto & shape : objs_) {
    shape->setColor(r,g,b,a);
  }
}

}  // namespace decomp_rviz_plugins
