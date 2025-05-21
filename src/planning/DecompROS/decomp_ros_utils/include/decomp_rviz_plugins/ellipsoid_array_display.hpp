#ifndef DECOMP_RVIZ_PLUGINS_ELLIPSOID_ARRAY_DISPLAY_HPP_
#define DECOMP_RVIZ_PLUGINS_ELLIPSOID_ARRAY_DISPLAY_HPP_

#include <memory>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>

#include <decomp_ros_msgs/msg/ellipsoid_array.hpp>

namespace Ogre { class SceneManager; class SceneNode; }

namespace decomp_rviz_plugins
{

/// Forward‐declare the visual class that does the Ogre work
class EllipsoidArrayVisual;

/// Display of a decomp_ros_msgs::msg::EllipsoidArray in RViz2
class EllipsoidArrayDisplay
  : public rviz_common::MessageFilterDisplay<decomp_ros_msgs::msg::EllipsoidArray>
{
  Q_OBJECT

public:
  EllipsoidArrayDisplay();
  ~EllipsoidArrayDisplay() override;

protected:
  /** @brief Called once when the display is created. */
  void onInitialize() override;

  /** @brief Clear the visual on reset */
  void reset() override;

private Q_SLOTS:
  /** @brief Invoked when user changes color or alpha in the property panel */
  void updateColorAndAlpha();

private:
  /** @brief Message callback: hand the data to the visual. */
  void processMessage(decomp_ros_msgs::msg::EllipsoidArray::ConstSharedPtr msg) override;

  std::shared_ptr<EllipsoidArrayVisual> visual_;

  rviz_common::properties::ColorProperty * color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
};

}  // namespace decomp_rviz_plugins

#endif  // DECOMP_RVIZ_PLUGINS_ELLIPSOID_ARRAY_DISPLAY_HPP_
