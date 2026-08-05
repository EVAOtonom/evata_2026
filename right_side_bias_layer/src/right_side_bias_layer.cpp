#include "right_side_bias_layer/right_side_bias_layer.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace right_side_bias_layer
{

void RightSideBiasLayer::onInitialize()
{
  auto node = node_.lock();

  if (!node) {
    throw std::runtime_error(
            "RightSideBiasLayer: lifecycle node is unavailable");
  }

  node->declare_parameter(name_ + ".enabled", true);
  node->declare_parameter(name_ + ".csv_path", std::string(""));
  node->declare_parameter(name_ + ".invert_side", false);

  node->declare_parameter(name_ + ".influence_distance", 3.0);
  node->declare_parameter(name_ + ".preferred_right_offset", 1.5);

  node->declare_parameter(name_ + ".right_sigma", 0.75);
  node->declare_parameter(name_ + ".center_cost", 70.0);

  node->declare_parameter(name_ + ".left_gain", 170.0);
  node->declare_parameter(name_ + ".left_sigma", 1.2);

  node->declare_parameter(name_ + ".minimum_cost", 0);
  node->declare_parameter(name_ + ".maximum_cost", 230);

  node->get_parameter(
    name_ + ".enabled",
    enabled_);

  node->get_parameter(
    name_ + ".csv_path",
    csv_path_);

  node->get_parameter(
    name_ + ".invert_side",
    invert_side_);

  node->get_parameter(
    name_ + ".influence_distance",
    influence_distance_);

  node->get_parameter(
    name_ + ".preferred_right_offset",
    preferred_right_offset_);

  node->get_parameter(
    name_ + ".right_sigma",
    right_sigma_);

  node->get_parameter(
    name_ + ".center_cost",
    center_cost_);

  node->get_parameter(
    name_ + ".left_gain",
    left_gain_);

  node->get_parameter(
    name_ + ".left_sigma",
    left_sigma_);

  node->get_parameter(
    name_ + ".minimum_cost",
    minimum_cost_);

  node->get_parameter(
    name_ + ".maximum_cost",
    maximum_cost_);

  influence_distance_ =
    std::max(0.1, influence_distance_);

  preferred_right_offset_ =
    std::max(0.0, preferred_right_offset_);

  right_sigma_ =
    std::max(0.05, right_sigma_);

  left_sigma_ =
    std::max(0.05, left_sigma_);

  minimum_cost_ =
    std::clamp(
    minimum_cost_,
    0,
    252);

  maximum_cost_ =
    std::clamp(
    maximum_cost_,
    minimum_cost_,
    252);

  if (csv_path_.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[%s] csv_path is empty. Layer disabled.",
      name_.c_str());

    enabled_ = false;
    current_ = true;
    return;
  }

  if (!loadCenterlineCsv(csv_path_)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[%s] could not load centerline CSV: %s. Layer disabled.",
      name_.c_str(),
      csv_path_.c_str());

    enabled_ = false;
    current_ = true;
    return;
  }

  current_ = true;

  RCLCPP_INFO(
    node->get_logger(),
    "[%s] loaded %zu centerline points from %s",
    name_.c_str(),
    centerline_.size(),
    csv_path_.c_str());
}

bool RightSideBiasLayer::loadCenterlineCsv(
  const std::string & path)
{
  std::ifstream input(path);

  if (!input.is_open()) {
    return false;
  }

  std::vector<geometry_msgs::msg::Point> loaded_points;

  std::string line;

  while (std::getline(input, line)) {
    if (line.empty()) {
      continue;
    }

    if (line.front() == '#') {
      continue;
    }

    std::stringstream stream(line);

    std::string x_text;
    std::string y_text;

    if (!std::getline(stream, x_text, ',')) {
      continue;
    }

    if (!std::getline(stream, y_text)) {
      continue;
    }

    try {
      geometry_msgs::msg::Point point;

      point.x = std::stod(x_text);
      point.y = std::stod(y_text);
      point.z = 0.0;

      loaded_points.push_back(point);
    } catch (const std::exception &) {
      /*
       * CSV header veya bozuk satırlar atlanır.
       *
       * Örnek header:
       *
       * x,y
       */
      continue;
    }
  }

  if (loaded_points.size() < 2) {
    return false;
  }

  centerline_ = std::move(loaded_points);

  line_min_x_ = centerline_.front().x;
  line_max_x_ = centerline_.front().x;

  line_min_y_ = centerline_.front().y;
  line_max_y_ = centerline_.front().y;

  for (const auto & point : centerline_) {
    line_min_x_ =
      std::min(
      line_min_x_,
      point.x);

    line_max_x_ =
      std::max(
      line_max_x_,
      point.x);

    line_min_y_ =
      std::min(
      line_min_y_,
      point.y);

    line_max_y_ =
      std::max(
      line_max_y_,
      point.y);
  }

  return true;
}

void RightSideBiasLayer::updateBounds(
  double,
  double,
  double,
  double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  if (!enabled_) {
    return;
  }

  if (centerline_.size() < 2) {
    return;
  }

  /*
   * Katmanın etkilediği alan her costmap güncellemesinde
   * tekrar update sınırına dahil edilir.
   *
   * Böylece:
   *
   * - master costmap temizlenirse
   * - RViz display yeniden başlarsa
   * - costmap tekrar publish edilirse
   *
   * bias katmanı yeniden hesaplanabilir.
   */
  *min_x =
    std::min(
    *min_x,
    line_min_x_ - influence_distance_);

  *min_y =
    std::min(
    *min_y,
    line_min_y_ - influence_distance_);

  *max_x =
    std::max(
    *max_x,
    line_max_x_ + influence_distance_);

  *max_y =
    std::max(
    *max_y,
    line_max_y_ + influence_distance_);
}

RightSideBiasLayer::SignedDistanceResult
RightSideBiasLayer::signedDistanceToCenterline(
  double x,
  double y) const
{
  SignedDistanceResult result;

  double minimum_squared_distance =
    std::numeric_limits<double>::infinity();

  for (std::size_t index = 0;
    index + 1 < centerline_.size();
    ++index)
  {
    const auto & point_a =
      centerline_[index];

    const auto & point_b =
      centerline_[index + 1];

    const double segment_x =
      point_b.x - point_a.x;

    const double segment_y =
      point_b.y - point_a.y;

    const double segment_length_squared =
      segment_x * segment_x +
      segment_y * segment_y;

    if (segment_length_squared < 1e-12) {
      continue;
    }

    const double point_relative_x =
      x - point_a.x;

    const double point_relative_y =
      y - point_a.y;

    const double projection =
      std::clamp(
      (
        point_relative_x * segment_x +
        point_relative_y * segment_y
      ) / segment_length_squared,
      0.0,
      1.0);

    const double nearest_x =
      point_a.x +
      projection * segment_x;

    const double nearest_y =
      point_a.y +
      projection * segment_y;

    const double difference_x =
      x - nearest_x;

    const double difference_y =
      y - nearest_y;

    const double squared_distance =
      difference_x * difference_x +
      difference_y * difference_y;

    if (squared_distance >= minimum_squared_distance) {
      continue;
    }

    minimum_squared_distance =
      squared_distance;

    result.distance =
      std::sqrt(squared_distance);

    /*
     * Noktalar sürüş yönünde sıralı olduğunda:
     *
     * cross > 0 : merkez çizginin solu
     * cross < 0 : merkez çizginin sağı
     */
    const double cross_product =
      segment_x * (y - point_a.y) -
      segment_y * (x - point_a.x);

    if (cross_product >= 0.0) {
      result.signed_distance =
        result.distance;
    } else {
      result.signed_distance =
        -result.distance;
    }

    if (invert_side_) {
      result.signed_distance *= -1.0;
    }

    result.valid = true;
  }

  return result;
}

unsigned char RightSideBiasLayer::calculateBiasCost(
  double signed_distance) const
{
  double cost = 0.0;

  if (signed_distance <= 0.0) {
    /*
     * Sağ taraf.
     *
     * preferred_right_offset mesafesinde minimum cost oluşur.
     *
     * Örneğin:
     *
     * preferred_right_offset = 1.5
     *
     * merkez çizginin 1.5 metre sağı tercih edilir.
     */
    const double right_distance =
      -signed_distance;

    const double offset_error =
      right_distance -
      preferred_right_offset_;

    const double exponent =
      -(
      offset_error *
      offset_error
      ) /
      (
      2.0 *
      right_sigma_ *
      right_sigma_
      );

    const double gaussian_rise =
      1.0 -
      std::exp(exponent);

    cost =
      static_cast<double>(minimum_cost_) +
      (
      center_cost_ -
      static_cast<double>(minimum_cost_)
      ) *
      gaussian_rise;
  } else {
    /*
     * Sol taraf ve karşı şerit.
     *
     * Merkez çizgide center_cost ile başlar.
     * Sola gittikçe left_gain kadar yükselir.
     */
    const double exponent =
      -(
      signed_distance *
      signed_distance
      ) /
      (
      2.0 *
      left_sigma_ *
      left_sigma_
      );

    const double gaussian_rise =
      1.0 -
      std::exp(exponent);

    cost =
      center_cost_ +
      left_gain_ *
      gaussian_rise;
  }

  const int rounded_cost =
    static_cast<int>(
    std::lround(cost));

  const int clamped_cost =
    std::clamp(
    rounded_cost,
    minimum_cost_,
    maximum_cost_);

  return static_cast<unsigned char>(
    clamped_cost);
}

void RightSideBiasLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i,
  int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  if (centerline_.size() < 2) {
    return;
  }

  const int size_x =
    static_cast<int>(
    master_grid.getSizeInCellsX());

  const int size_y =
    static_cast<int>(
    master_grid.getSizeInCellsY());

  const int bounded_min_i =
    std::clamp(
    min_i,
    0,
    size_x);

  const int bounded_min_j =
    std::clamp(
    min_j,
    0,
    size_y);

  const int bounded_max_i =
    std::clamp(
    max_i,
    0,
    size_x);

  const int bounded_max_j =
    std::clamp(
    max_j,
    0,
    size_y);

  for (int map_y = bounded_min_j;
    map_y < bounded_max_j;
    ++map_y)
  {
    for (int map_x = bounded_min_i;
      map_x < bounded_max_i;
      ++map_x)
    {
      double world_x = 0.0;
      double world_y = 0.0;

      master_grid.mapToWorld(
        static_cast<unsigned int>(map_x),
        static_cast<unsigned int>(map_y),
        world_x,
        world_y);

      const SignedDistanceResult distance_result =
        signedDistanceToCenterline(
        world_x,
        world_y);

      if (!distance_result.valid) {
        continue;
      }

      if (
        distance_result.distance >
        influence_distance_)
      {
        continue;
      }

      const unsigned int cell_x =
        static_cast<unsigned int>(map_x);

      const unsigned int cell_y =
        static_cast<unsigned int>(map_y);

      const unsigned char old_cost =
        master_grid.getCost(
        cell_x,
        cell_y);

      /*
       * Unknown alanı bias layer ile traversable yapma.
       */
      if (
        old_cost ==
        nav2_costmap_2d::NO_INFORMATION)
      {
        continue;
      }

      /*
       * Lethal ve inscribed obstacle hücrelerini koru.
       */
      if (
        old_cost >=
        nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        continue;
      }

      const unsigned char bias_cost =
        calculateBiasCost(
        distance_result.signed_distance);

      /*
       * Obstacle ve inflation maliyetlerini ezmemek için
       * maksimum maliyet kullanılır.
       */
      const unsigned char merged_cost =
        std::max(
        old_cost,
        bias_cost);

      master_grid.setCost(
        cell_x,
        cell_y,
        merged_cost);
    }
  }

  current_ = true;
}

void RightSideBiasLayer::reset()
{
  /*
   * Katman doğrudan clear edilmiyor.
   *
   * updateBounds() her costmap döngüsünde etki alanını
   * yeniden işaretlediği için layer tekrar çizilebilir.
   */
  current_ = false;
}

bool RightSideBiasLayer::isClearable()
{
  /*
   * RViz / Nav2 reset sırasında custom layer'ın reset akışına
   * dahil edilmesini engeller.
   *
   * Master grid temizlense bile updateBounds() alanı tekrar
   * güncelleyecektir.
   */
  return false;
}

}  // namespace right_side_bias_layer

PLUGINLIB_EXPORT_CLASS(
  right_side_bias_layer::RightSideBiasLayer,
  nav2_costmap_2d::Layer)
