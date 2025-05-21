#include "decomp_rviz_plugins/ellipsoid_array_display.hpp"

#include "decomp_rviz_plugins/ellipsoid_array_visual.hpp"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <rviz_common/properties/property.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_common/frame_manager.hpp>

namespace decomp_rviz_plugins
{

EllipsoidArrayDisplay::EllipsoidArrayDisplay()
{
  // Create two properties under this display: Color and Alpha
  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(204, 51, 204),
    "Color to draw the ellipsoids",
    this,
    SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 0.5f,
    "0 is fully transparent, 1.0 is fully opaque",
    this,
    SLOT(updateColorAndAlpha()));
}

EllipsoidArrayDisplay::~EllipsoidArrayDisplay() = default;

void EllipsoidArrayDisplay::onInitialize()
{
  // Let base class set up ROS subscription, scene node, etc.
  MessageFilterDisplay::onInitialize();

  // Create our visual helper
  visual_.reset(
    new EllipsoidArrayVisual(context_->getSceneManager(), scene_node_));
}

void EllipsoidArrayDisplay::reset()
{
  MessageFilterDisplay::reset();
  visual_.reset(
    new EllipsoidArrayVisual(context_->getSceneManager(), scene_node_));
}

void EllipsoidArrayDisplay::updateColorAndAlpha()
{
  // pull values from properties
  Ogre::ColourValue color = color_property_->getOgreColor();
  float alpha = alpha_property_->getFloat();

  visual_->setColor(color.r, color.g, color.b, alpha);
}

void EllipsoidArrayDisplay::processMessage(decomp_ros_msgs::msg::EllipsoidArray::ConstSharedPtr msg)
{
  // make sure color/alpha are applied
  updateColorAndAlpha();

  // pass the raw vector<msg::Ellipsoid> into the visual
  visual_->setMessage(msg->ellipsoids);
}

}  // namespace decomp_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
// tell pluginlib about this class.  Must match package.xml export.
PLUGINLIB_EXPORT_CLASS(decomp_rviz_plugins::EllipsoidArrayDisplay, rviz_common::Display)
