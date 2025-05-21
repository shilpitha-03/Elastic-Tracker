#ifndef DECOMP_RVIZ_PLUGINS_POLYHEDRON_ARRAY_DISPLAY_HPP_
#define DECOMP_RVIZ_PLUGINS_POLYHEDRON_ARRAY_DISPLAY_HPP_

#include <memory>
#include <vector>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/enum_property.hpp>

#include <decomp_ros_msgs/msg/polyhedron_array.hpp>

namespace Ogre { class Vector3; class Quaternion; }

namespace decomp_rviz_plugins
{

/**
 * \brief Visualization of a PolyhedronArray message in RViz2
 */
class PolyhedronArrayDisplay
  : public rviz_common::MessageFilterDisplay<decomp_ros_msgs::msg::PolyhedronArray>
{
  Q_OBJECT
public:
  PolyhedronArrayDisplay();
  ~PolyhedronArrayDisplay() override;

protected:
  void onInitialize() override;
  void reset() override;

private Q_SLOTS:
  void updateMeshColorAndAlpha();
  void updateBoundColorAndAlpha();
  void updateVsColorAndAlpha();
  void updateState();
  void updateScale();
  void updateVsScale();

private:
  void processMessage(decomp_ros_msgs::msg::PolyhedronArray::ConstSharedPtr msg) override;
  void visualizeMessage(int state);
  void visualizeMesh();
  void visualizeBound();
  void visualizeVs();

  // visuals
  std::shared_ptr<MeshVisual>   visual_mesh_;
  std::shared_ptr<BoundVisual>  visual_bound_;
  std::shared_ptr<VectorVisual> visual_vector_;

  // properties
  rviz_common::properties::ColorProperty * mesh_color_property_;
  rviz_common::properties::ColorProperty * bound_color_property_;
  rviz_common::properties::ColorProperty * vs_color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::FloatProperty * scale_property_;
  rviz_common::properties::FloatProperty * vs_scale_property_;
  rviz_common::properties::EnumProperty  * state_property_;

  // current pose of the frame
  Ogre::Vector3   position_;
  Ogre::Quaternion orientation_;

  // cached geometry
  vec_E<vec_Vec3f> vertices_;
  vec_E<std::pair<Vec3f, Vec3f>> vs_;
};

}  // namespace decomp_rviz_plugins

#endif  // DECOMP_RVIZ_PLUGINS_POLYHEDRON_ARRAY_DISPLAY_HPP_