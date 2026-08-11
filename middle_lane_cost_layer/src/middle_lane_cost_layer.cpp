#include "middle_lane_cost_layer/middle_lane_cost_layer.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace middle_lane_cost_layer
{

void MiddleLaneCostLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("MiddleLaneCostLayer: lifecycle node is unavailable");
  }

  // Nav2 Layer helper'ı parametreleri <plugin_name>.<param> olarak declare eder.
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("csv_path", rclcpp::ParameterValue(std::string("")));
  declareParameter("lane_half_width", rclcpp::ParameterValue(0.35));
  declareParameter("lane_cost", rclcpp::ParameterValue(230));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".csv_path", csv_path_);
  node->get_parameter(name_ + ".lane_half_width", lane_half_width_);
  node->get_parameter(name_ + ".lane_cost", lane_cost_);

  lane_half_width_ = std::max(0.01, lane_half_width_);
  lane_cost_ = std::clamp(lane_cost_, 0, 252);

  if (!enabled_) {
    RCLCPP_INFO(
      node->get_logger(),
      "[%s] disabled by parameter.",
      name_.c_str());
    current_ = true;
    return;
  }

  if (csv_path_.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[%s] csv_path is empty. Layer disabled.",
      name_.c_str());
    enabled_ = false;
    current_ = true;
    return;
  }

  if (!loadCsv(csv_path_)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[%s] failed to load CSV: %s. Layer disabled.",
      name_.c_str(),
      csv_path_.c_str());
    enabled_ = false;
    current_ = true;
    return;
  }

  current_ = true;

  RCLCPP_INFO(
    node->get_logger(),
    "[%s] loaded %zu independent middle-lane segments from %s",
    name_.c_str(),
    segments_.size(),
    csv_path_.c_str());

  RCLCPP_INFO(
    node->get_logger(),
    "[%s] pairing mode: (1-2), (3-4), (5-6), ... ; lane_cost=%d, total_width=%.2f m",
    name_.c_str(),
    lane_cost_,
    2.0 * lane_half_width_);
}

std::string MiddleLaneCostLayer::trim(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }

  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::vector<std::string> MiddleLaneCostLayer::splitCsvLine(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;

  while (std::getline(stream, field, ',')) {
    fields.push_back(trim(field));
  }

  return fields;
}

bool MiddleLaneCostLayer::loadCsv(const std::string & path)
{
  auto node = node_.lock();

  std::ifstream input(path);
  if (!input.is_open()) {
    return false;
  }

  std::vector<Point2D> points;
  std::string line;

  int id_column = -1;
  int x_column = -1;
  int y_column = -1;
  bool columns_initialized = false;
  int fallback_id = 1;

  while (std::getline(input, line)) {
    line = trim(line);
    if (line.empty() || line.front() == '#') {
      continue;
    }

    const auto fields = splitCsvLine(line);
    if (fields.empty()) {
      continue;
    }

    if (!columns_initialized) {
      for (std::size_t i = 0; i < fields.size(); ++i) {
        if (fields[i] == "point_id") {
          id_column = static_cast<int>(i);
        } else if (fields[i] == "x") {
          x_column = static_cast<int>(i);
        } else if (fields[i] == "y") {
          y_column = static_cast<int>(i);
        }
      }

      // Header bulunduysa header satırını atla.
      if (x_column >= 0 && y_column >= 0) {
        columns_initialized = true;
        continue;
      }

      // Header yoksa x,y formatını da destekle.
      id_column = -1;
      x_column = 0;
      y_column = 1;
      columns_initialized = true;
    }

    if (
      x_column < 0 || y_column < 0 ||
      static_cast<std::size_t>(x_column) >= fields.size() ||
      static_cast<std::size_t>(y_column) >= fields.size())
    {
      continue;
    }

    try {
      Point2D point;
      point.x = std::stod(fields[static_cast<std::size_t>(x_column)]);
      point.y = std::stod(fields[static_cast<std::size_t>(y_column)]);

      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        continue;
      }

      if (
        id_column >= 0 &&
        static_cast<std::size_t>(id_column) < fields.size())
      {
        point.id = std::stoi(fields[static_cast<std::size_t>(id_column)]);
      } else {
        point.id = fallback_id;
      }

      ++fallback_id;
      points.push_back(point);
    } catch (const std::exception &) {
      continue;
    }
  }

  if (points.size() < 2) {
    return false;
  }

  // point_id varsa CSV satır sırasından bağımsız olarak 1..N sıralamasını garanti et.
  std::sort(
    points.begin(), points.end(),
    [](const Point2D & lhs, const Point2D & rhs) {
      return lhs.id < rhs.id;
    });

  if (points.size() % 2 != 0) {
    if (node) {
      RCLCPP_WARN(
        node->get_logger(),
        "[%s] odd number of points (%zu). Last unmatched point is ignored.",
        name_.c_str(),
        points.size());
    }
    points.pop_back();
  }

  segments_.clear();
  segments_.reserve(points.size() / 2);

  for (std::size_t i = 0; i + 1 < points.size(); i += 2) {
    Segment2D segment;
    segment.a = points[i];
    segment.b = points[i + 1];

    const double dx = segment.b.x - segment.a.x;
    const double dy = segment.b.y - segment.a.y;

    if (dx * dx + dy * dy < 1e-12) {
      if (node) {
        RCLCPP_WARN(
          node->get_logger(),
          "[%s] degenerate segment (%d-%d) ignored.",
          name_.c_str(),
          segment.a.id,
          segment.b.id);
      }
      continue;
    }

    segment.min_x = std::min(segment.a.x, segment.b.x);
    segment.max_x = std::max(segment.a.x, segment.b.x);
    segment.min_y = std::min(segment.a.y, segment.b.y);
    segment.max_y = std::max(segment.a.y, segment.b.y);

    segments_.push_back(segment);
  }

  if (segments_.empty()) {
    return false;
  }

  all_min_x_ = std::numeric_limits<double>::infinity();
  all_min_y_ = std::numeric_limits<double>::infinity();
  all_max_x_ = -std::numeric_limits<double>::infinity();
  all_max_y_ = -std::numeric_limits<double>::infinity();

  for (const auto & segment : segments_) {
    all_min_x_ = std::min(all_min_x_, segment.min_x);
    all_max_x_ = std::max(all_max_x_, segment.max_x);
    all_min_y_ = std::min(all_min_y_, segment.min_y);
    all_max_y_ = std::max(all_max_y_, segment.max_y);
  }

  return true;
}

double MiddleLaneCostLayer::squaredDistanceToSegment(
  double x,
  double y,
  const Segment2D & segment)
{
  const double dx = segment.b.x - segment.a.x;
  const double dy = segment.b.y - segment.a.y;
  const double length_squared = dx * dx + dy * dy;

  if (length_squared < 1e-12) {
    const double px = x - segment.a.x;
    const double py = y - segment.a.y;
    return px * px + py * py;
  }

  const double t = std::clamp(
    ((x - segment.a.x) * dx + (y - segment.a.y) * dy) / length_squared,
    0.0,
    1.0);

  const double nearest_x = segment.a.x + t * dx;
  const double nearest_y = segment.a.y + t * dy;

  const double ex = x - nearest_x;
  const double ey = y - nearest_y;

  return ex * ex + ey * ey;
}

void MiddleLaneCostLayer::updateBounds(
  double,
  double,
  double,
  double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  if (!enabled_ || segments_.empty()) {
    return;
  }

  *min_x = std::min(*min_x, all_min_x_ - lane_half_width_);
  *min_y = std::min(*min_y, all_min_y_ - lane_half_width_);
  *max_x = std::max(*max_x, all_max_x_ + lane_half_width_);
  *max_y = std::max(*max_y, all_max_y_ + lane_half_width_);
}

void MiddleLaneCostLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i,
  int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_ || segments_.empty()) {
    return;
  }

  const int size_x = static_cast<int>(master_grid.getSizeInCellsX());
  const int size_y = static_cast<int>(master_grid.getSizeInCellsY());

  const int start_i = std::clamp(min_i, 0, size_x);
  const int start_j = std::clamp(min_j, 0, size_y);
  const int end_i = std::clamp(max_i, 0, size_x);
  const int end_j = std::clamp(max_j, 0, size_y);

  const double max_distance_squared = lane_half_width_ * lane_half_width_;
  const unsigned char target_cost = static_cast<unsigned char>(lane_cost_);

  for (int j = start_j; j < end_j; ++j) {
    for (int i = start_i; i < end_i; ++i) {
      double world_x = 0.0;
      double world_y = 0.0;

      master_grid.mapToWorld(
        static_cast<unsigned int>(i),
        static_cast<unsigned int>(j),
        world_x,
        world_y);

      bool on_middle_lane = false;

      for (const auto & segment : segments_) {
        // Hızlı bounding-box elemesi.
        if (
          world_x < segment.min_x - lane_half_width_ ||
          world_x > segment.max_x + lane_half_width_ ||
          world_y < segment.min_y - lane_half_width_ ||
          world_y > segment.max_y + lane_half_width_)
        {
          continue;
        }

        if (squaredDistanceToSegment(world_x, world_y, segment) <= max_distance_squared) {
          on_middle_lane = true;
          break;
        }
      }

      if (!on_middle_lane) {
        continue;
      }

      const auto cell_x = static_cast<unsigned int>(i);
      const auto cell_y = static_cast<unsigned int>(j);
      const unsigned char old_cost = master_grid.getCost(cell_x, cell_y);

      // Unknown alanı bilinen alana çevirmiyoruz.
      if (old_cost == nav2_costmap_2d::NO_INFORMATION) {
        continue;
      }

      // Var olan obstacle / inflation maliyetlerini azaltma; yalnızca yükselt.
      master_grid.setCost(cell_x, cell_y, std::max(old_cost, target_cost));
    }
  }

  current_ = true;
}

void MiddleLaneCostLayer::reset()
{
  current_ = false;
}

bool MiddleLaneCostLayer::isClearable()
{
  return false;
}

}  // namespace middle_lane_cost_layer

PLUGINLIB_EXPORT_CLASS(
  middle_lane_cost_layer::MiddleLaneCostLayer,
  nav2_costmap_2d::Layer)
