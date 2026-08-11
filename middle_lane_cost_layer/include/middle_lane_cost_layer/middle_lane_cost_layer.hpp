#ifndef MIDDLE_LANE_COST_LAYER__MIDDLE_LANE_COST_LAYER_HPP_
#define MIDDLE_LANE_COST_LAYER__MIDDLE_LANE_COST_LAYER_HPP_

#include <string>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"

namespace middle_lane_cost_layer
{

class MiddleLaneCostLayer : public nav2_costmap_2d::Layer
{
public:
  MiddleLaneCostLayer() = default;
  ~MiddleLaneCostLayer() override = default;

  void onInitialize() override;

  void updateBounds(
    double robot_x,
    double robot_y,
    double robot_yaw,
    double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j) override;

  void reset() override;
  bool isClearable() override;

private:
  struct Point2D
  {
    int id{0};
    double x{0.0};
    double y{0.0};
  };

  struct Segment2D
  {
    Point2D a;
    Point2D b;
    double min_x{0.0};
    double max_x{0.0};
    double min_y{0.0};
    double max_y{0.0};
  };

  bool loadCsv(const std::string & path);

  static std::vector<std::string> splitCsvLine(const std::string & line);
  static std::string trim(const std::string & value);
  static double squaredDistanceToSegment(
    double x,
    double y,
    const Segment2D & segment);

  bool enabled_{true};
  std::string csv_path_;

  // Segmentin iki yanındaki kalınlık. Toplam yüksek-cost genişliği = 2 * lane_half_width_.
  double lane_half_width_{0.35};

  // 0-252 arası tutulur. 253/254 obstacle sınıflarıyla karışmasın diye 252 üst sınırdır.
  int lane_cost_{230};

  std::vector<Segment2D> segments_;

  double all_min_x_{0.0};
  double all_max_x_{0.0};
  double all_min_y_{0.0};
  double all_max_y_{0.0};
};

}  // namespace middle_lane_cost_layer

#endif  // MIDDLE_LANE_COST_LAYER__MIDDLE_LANE_COST_LAYER_HPP_
