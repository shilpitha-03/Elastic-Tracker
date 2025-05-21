#include "decomp_rviz_plugins/polyhedron_array_display.hpp"
#include "decomp_rviz_plugins/mesh_visual.hpp"
#include "decomp_rviz_plugins/bound_visual.hpp"
#include "decomp_rviz_plugins/vector_visual.hpp"
#include <rviz_common/frame_manager.hpp>

namespace decomp_rviz_plugins
{

PolyhedronArrayDisplay::PolyhedronArrayDisplay()
{
  mesh_color_property_ = new rviz_common::properties::ColorProperty(
    "MeshColor", QColor(0,170,255), "Mesh color.",
    this, SLOT(updateMeshColorAndAlpha()));

  bound_color_property_ = new rviz_common::properties::ColorProperty(
    "BoundColor", QColor(255,0,0), "Bound color.",
    this, SLOT(updateBoundColorAndAlpha()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 0.2f,
    "0 is fully transparent, 1.0 is fully opaque, only affect mesh",
    this, SLOT(updateMeshColorAndAlpha()));

  scale_property_ = new rviz_common::properties::FloatProperty(
    "Scale", 0.1f, "bound scale.",
    this, SLOT(updateScale()));

  vs_scale_property_ = new rviz_common::properties::FloatProperty(
    "VsScale", 1.0f, "Vs scale.",
    this, SLOT(updateVsScale()));

  vs_color_property_ = new rviz_common::properties::ColorProperty(
    "VsColor", QColor(0,255,0), "Vs color.",
    this, SLOT(updateVsColorAndAlpha()));

  state_property_ = new rviz_common::properties::EnumProperty(
    "State", "Mesh",
    "Select visualization state: Mesh, Bound, Both, Vs",
    this, SLOT(updateState()));
  state_property_->addOption("Mesh", 0);
  state_property_->addOption("Bound", 1);
  state_property_->addOption("Both", 2);
  state_property_->addOption("Vs", 3);
}

PolyhedronArrayDisplay::~PolyhedronArrayDisplay() = default;

void PolyhedronArrayDisplay::onInitialize()
{
  MessageFilterDisplay::onInitialize();
}

void PolyhedronArrayDisplay::reset()
{
  MessageFilterDisplay::reset();
  visual_mesh_.reset();
  visual_bound_.reset();
  visual_vector_.reset();
}

void PolyhedronArrayDisplay::processMessage(
  decomp_ros_msgs::msg::PolyhedronArray::ConstSharedPtr msg)
{
  // transform into fixed frame
  if (!context_->getFrameManager()->getTransform(
        msg->header.frame_id, msg->header.stamp,
        position_, orientation_))
  {
    return;
  }

  vertices_.clear();
  vs_.clear();

  auto polys = DecompROS::ros_to_polyhedron_array(*msg);
  for (auto const & poly : polys) {
    auto bds = cal_vertices(poly);
    vertices_.insert(vertices_.end(), bds.begin(), bds.end());
    auto normals = poly.cal_normals();
    vs_.insert(vs_.end(), normals.begin(), normals.end());
  }

  int state = state_property_->getOptionInt();
  visualizeMessage(state);
}

void PolyhedronArrayDisplay::visualizeMesh()
{
  visual_mesh_ = std::make_shared<MeshVisual>(
    context_->getSceneManager(), scene_node_);
  visual_mesh_->setMessage(vertices_);
  visual_mesh_->setFramePosition(position_);
  visual_mesh_->setFrameOrientation(orientation_);
  float alpha = alpha_property_->getFloat();
  auto c = mesh_color_property_->getOgreColor();
  visual_mesh_->setColor(c.r, c.g, c.b, alpha);
}

void PolyhedronArrayDisplay::visualizeBound()
{
  visual_bound_ = std::make_shared<BoundVisual>(
    context_->getSceneManager(), scene_node_);
  visual_bound_->setMessage(vertices_);
  visual_bound_->setFramePosition(position_);
  visual_bound_->setFrameOrientation(orientation_);
  auto c = bound_color_property_->getOgreColor();
  visual_bound_->setColor(c.r, c.g, c.b, 1.0f);
  visual_bound_->setScale(scale_property_->getFloat());
}

void PolyhedronArrayDisplay::visualizeVs()
{
  visual_vector_ = std::make_shared<VectorVisual>(
    context_->getSceneManager(), scene_node_);
  visual_vector_->setMessage(vs_);
  visual_vector_->setFramePosition(position_);
  visual_vector_->setFrameOrientation(orientation_);
  auto c = vs_color_property_->getOgreColor();
  visual_vector_->setColor(c.r, c.g, c.b, 1.0f);
  visual_vector_->setScale(vs_scale_property_->getFloat());
}

void PolyhedronArrayDisplay::visualizeMessage(int state)
{
  switch(state) {
    case 0: visual_bound_.reset(); visual_vector_.reset(); visualizeMesh(); break;
    case 1: visual_mesh_.reset(); visual_vector_.reset(); visualizeBound(); break;
    case 2: visual_vector_.reset(); visualizeMesh(); visualizeBound(); break;
    case 3: visualizeVs(); break;
  }
}

void PolyhedronArrayDisplay::updateMeshColorAndAlpha()
{
  if (visual_mesh_) {
    float a = alpha_property_->getFloat();
    auto c = mesh_color_property_->getOgreColor();
    visual_mesh_->setColor(c.r, c.g, c.b, a);
  }
}

void PolyhedronArrayDisplay::updateBoundColorAndAlpha()
{
  if (visual_bound_) {
    auto c = bound_color_property_->getOgreColor();
    visual_bound_->setColor(c.r, c.g, c.b, 1.0f);
  }
}

void PolyhedronArrayDisplay::updateVsColorAndAlpha()
{
  if (visual_vector_) {
    auto c = vs_color_property_->getOgreColor();
    visual_vector_->setColor(c.r, c.g, c.b, 1.0f);
  }
}

void PolyhedronArrayDisplay::updateState()
{
  int s = state_property_->getOptionInt();
  visualizeMessage(s);
}

void PolyhedronArrayDisplay::updateScale()
{
  if (visual_bound_) visual_bound_->setScale(scale_property_->getFloat());
}

void PolyhedronArrayDisplay::updateVsScale()
{
  if (visual_vector_) visual_vector_->setScale(vs_scale_property_->getFloat());
}

} // namespace decomp_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  decomp_rviz_plugins::PolyhedronArrayDisplay,
  rviz_common::Display)
