#ifndef DECOMP_RVIZ_PLUGINS_VECTOR_VISUAL_HPP_
#define DECOMP_RVIZ_PLUGINS_VECTOR_VISUAL_HPP_

#include <vector>
#include <memory>
#include <utility>

#include <Eigen/Core>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz_rendering/objects/arrow.hpp>

using Vec3f = Eigen::Vector3f;
using PairVec = std::pair<Vec3f, Vec3f>;
using vec_E_Pair = std::vector<PairVec>;

namespace decomp_rviz_plugins
{

/**
 * \brief Visualizes a set of vectors as arrows in RViz2
 */
class VectorVisual
{
public:
  VectorVisual(Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node);
  ~VectorVisual();

  /// 
  void setMessage(const vec_E_Pair & vs);
  void setFramePosition(const Ogre::Vector3 & position);
  void setFrameOrientation(const Ogre::Quaternion & orientation);

  void setColor(float r, float g, float b, float a);
  void setScale(float s);

private:
  std::vector<std::unique_ptr<rviz_rendering::Arrow>> objs_;
  Ogre::SceneNode * frame_node_;
  Ogre::SceneManager * scene_manager_;
  float s_ = 1.0f;
  vec_E_Pair vs_;
};

} // namespace decomp_rviz_plugins
#endif // DECOMP_RVIZ_PLUGINS_VECTOR_VISUAL_HPP_