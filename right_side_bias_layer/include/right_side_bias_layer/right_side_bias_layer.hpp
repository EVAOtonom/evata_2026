#ifndef RIGHT_SIDE_BIAS_LAYER__RIGHT_SIDE_BIAS_LAYER_HPP_
#define RIGHT_SIDE_BIAS_LAYER__RIGHT_SIDE_BIAS_LAYER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace right_side_bias_layer
{

class RightSideBiasLayer : public nav2_costmap_2d::Layer
{
public:
  RightSideBiasLayer() = default;
  ~RightSideBiasLayer() override = default;

  void onInitialize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override;
  bool isClearable() override;

private:
  struct SignedDistanceResult
  {
    double signed_distance{0.0};
    double distance{0.0};
    bool valid{false};
  };

  bool loadCenterlineCsv(const std::string & path);
  SignedDistanceResult signedDistanceToCenterline(double x, double y) const;
  unsigned char calculateBiasCost(double signed_distance) const;

  std::vector<geometry_msgs::msg::Point> centerline_;
  std::string csv_path_;

  bool enabled_{true};
  bool invert_side_{false};
  double influence_distance_{4.0};
  double preferred_right_offset_{0.9};
  double right_sigma_{0.75};
  double center_cost_{70.0};
  double left_gain_{170.0};
  double left_sigma_{1.2};
  int minimum_cost_{0};
  int maximum_cost_{252};

  double line_min_x_{0.0};
  double line_min_y_{0.0};
  double line_max_x_{0.0};
  double line_max_y_{0.0};
};

}  // namespace right_side_bias_layer

#endif  // RIGHT_SIDE_BIAS_LAYER__RIGHT_SIDE_BIAS_LAYER_HPP_
