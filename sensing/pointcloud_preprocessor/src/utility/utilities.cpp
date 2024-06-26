// Copyright 2022 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pointcloud_preprocessor/utility/utilities.hpp"

#include <autoware_point_types/types.hpp>

namespace pointcloud_preprocessor::utils
{
void to_cgal_polygon(const geometry_msgs::msg::Polygon & polygon_in, PolygonCgal & polygon_out)
{
  if (polygon_in.points.size() < 3) {
    throw std::length_error("Polygon vertex count should be larger than 2.");
  }

  const auto & vertices_in = polygon_in.points;
  polygon_out.resize(vertices_in.size());
  std::transform(
    polygon_in.points.cbegin(), polygon_in.points.cend(), polygon_out.begin(),
    [](const geometry_msgs::msg::Point32 & p_in) { return PointCgal(p_in.x, p_in.y); });
}

void to_cgal_polygon(const lanelet::BasicPolygon2d & polygon_in, PolygonCgal & polygon_out)
{
  if (polygon_in.size() < 3) {
    throw std::length_error("Polygon vertex count should be larger than 2.");
  }

  const auto & vertices_in = polygon_in;
  polygon_out.resize(vertices_in.size());
  std::transform(
    polygon_in.cbegin(), polygon_in.cend(), polygon_out.begin(),
    [](const Eigen::Matrix<double, 2, 1> & p_in) { return PointCgal(p_in.x(), p_in.y()); });
}

void remove_polygon_cgal_from_cloud(
  const sensor_msgs::msg::PointCloud2 & cloud_in, const PolygonCgal & polyline_polygon,
  sensor_msgs::msg::PointCloud2 & cloud_out, const std::optional<float> & max_z)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_output;

  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_in, "x"), iter_y(cloud_in, "y"),
       iter_z(cloud_in, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const bool within_max_z = max_z ? *iter_z <= *max_z : true;
    const bool within_polygon = CGAL::bounded_side_2(
                                  polyline_polygon.begin(), polyline_polygon.end(),
                                  PointCgal(*iter_x, *iter_y), K()) == CGAL::ON_BOUNDED_SIDE;
    // remove points within the polygon and max_z
    if (!(within_max_z && within_polygon)) {
      pcl::PointXYZ p;
      p.x = *iter_x;
      p.y = *iter_y;
      p.z = *iter_z;
      pcl_output.emplace_back(p);
    }
  }

  pcl::toROSMsg(pcl_output, cloud_out);
  cloud_out.header = cloud_in.header;
}

void remove_polygon_cgal_from_cloud(
  const pcl::PointCloud<pcl::PointXYZ> & cloud_in, const PolygonCgal & polyline_polygon,
  pcl::PointCloud<pcl::PointXYZ> & cloud_out, const std::optional<float> & max_z)
{
  cloud_out.clear();
  cloud_out.header = cloud_in.header;

  for (const auto & p : cloud_in) {
    const bool within_max_z = max_z ? p.z <= *max_z : true;
    const bool within_polygon = CGAL::bounded_side_2(
                                  polyline_polygon.begin(), polyline_polygon.end(),
                                  PointCgal(p.x, p.y), K()) == CGAL::ON_BOUNDED_SIDE;
    // remove points within the polygon and max_z
    if (!(within_max_z && within_polygon)) {
      cloud_out.emplace_back(p);
    }
  }
}

void remove_polygon_cgal_from_cloud(
  const sensor_msgs::msg::PointCloud2 & cloud_in,
  const std::vector<PolygonCgal> & polyline_polygons, sensor_msgs::msg::PointCloud2 & cloud_out,
  const std::optional<float> & max_z)
{
  if (polyline_polygons.empty()) {
    cloud_out = cloud_in;
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_in, "x"), iter_y(cloud_in, "y"),
       iter_z(cloud_in, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const bool within_max_z = max_z ? *iter_z <= *max_z : true;
    const pcl::PointXYZ p(*iter_x, *iter_y, *iter_z);
    const bool within_polygon = point_within_cgal_polys(p, polyline_polygons);
    // remove points within the polygon and max_z
    if (within_max_z && within_polygon) {
      continue;
    }
    filtered_cloud.emplace_back(p);
  }

  pcl::toROSMsg(filtered_cloud, cloud_out);
  cloud_out.header = cloud_in.header;
}

void remove_polygon_cgal_from_cloud(
  const pcl::PointCloud<pcl::PointXYZ> & cloud_in,
  const std::vector<PolygonCgal> & polyline_polygons, pcl::PointCloud<pcl::PointXYZ> & cloud_out,
  const std::optional<float> & max_z)
{
  if (polyline_polygons.empty()) {
    cloud_out = cloud_in;
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
  for (const auto & p : cloud_in) {
    const bool within_max_z = max_z ? p.z <= *max_z : true;
    const bool within_polygon = point_within_cgal_polys(p, polyline_polygons);
    // remove points within the polygon and max_z
    if (within_max_z && within_polygon) {
      continue;
    }
    filtered_cloud.emplace_back(p);
  }

  cloud_out = filtered_cloud;
  cloud_out.header = cloud_in.header;
}

bool point_within_cgal_polys(
  const pcl::PointXYZ & point, const std::vector<PolygonCgal> & polyline_polygons)
{
  for (const auto & polygon : polyline_polygons) {
    if (
      CGAL::bounded_side_2(polygon.cbegin(), polygon.cend(), PointCgal(point.x, point.y), K()) ==
      CGAL::ON_BOUNDED_SIDE) {
      return true;
    }
  }

  return false;
}

bool is_data_layout_compatible_with_PointXYZI(const sensor_msgs::msg::PointCloud2 & input)
{
  using autoware_point_types::PointIndex;
  using autoware_point_types::PointXYZI;
  if (input.fields.size() < 4) {
    return false;
  }
  bool same_layout = true;
  const auto & field_x = input.fields.at(static_cast<size_t>(PointIndex::X));
  same_layout &= field_x.name == "x";
  same_layout &= field_x.offset == offsetof(PointXYZI, x);
  same_layout &= field_x.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_x.count == 1;
  const auto & field_y = input.fields.at(static_cast<size_t>(PointIndex::Y));
  same_layout &= field_y.name == "y";
  same_layout &= field_y.offset == offsetof(PointXYZI, y);
  same_layout &= field_y.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_y.count == 1;
  const auto & field_z = input.fields.at(static_cast<size_t>(PointIndex::Z));
  same_layout &= field_z.name == "z";
  same_layout &= field_z.offset == offsetof(PointXYZI, z);
  same_layout &= field_z.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_z.count == 1;
  const auto & field_intensity = input.fields.at(static_cast<size_t>(PointIndex::Intensity));
  same_layout &= field_intensity.name == "intensity";
  same_layout &= field_intensity.offset == offsetof(PointXYZI, intensity);
  same_layout &= field_intensity.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_intensity.count == 1;
  return same_layout;
}

bool is_data_layout_compatible_with_PointXYZIRADRT(const sensor_msgs::msg::PointCloud2 & input)
{
  using autoware_point_types::PointIndex;
  using autoware_point_types::PointXYZIRADRT;
  if (input.fields.size() < 9) {
    return false;
  }
  bool same_layout = true;
  const auto & field_x = input.fields.at(static_cast<size_t>(PointIndex::X));
  same_layout &= field_x.name == "x";
  same_layout &= field_x.offset == offsetof(PointXYZIRADRT, x);
  same_layout &= field_x.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_x.count == 1;
  const auto & field_y = input.fields.at(static_cast<size_t>(PointIndex::Y));
  same_layout &= field_y.name == "y";
  same_layout &= field_y.offset == offsetof(PointXYZIRADRT, y);
  same_layout &= field_y.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_y.count == 1;
  const auto & field_z = input.fields.at(static_cast<size_t>(PointIndex::Z));
  same_layout &= field_z.name == "z";
  same_layout &= field_z.offset == offsetof(PointXYZIRADRT, z);
  same_layout &= field_z.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_z.count == 1;
  const auto & field_intensity = input.fields.at(static_cast<size_t>(PointIndex::Intensity));
  same_layout &= field_intensity.name == "intensity";
  same_layout &= field_intensity.offset == offsetof(PointXYZIRADRT, intensity);
  same_layout &= field_intensity.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_intensity.count == 1;
  const auto & field_ring = input.fields.at(static_cast<size_t>(PointIndex::Ring));
  same_layout &= field_ring.name == "ring";
  same_layout &= field_ring.offset == offsetof(PointXYZIRADRT, ring);
  same_layout &= field_ring.datatype == sensor_msgs::msg::PointField::UINT16;
  same_layout &= field_ring.count == 1;
  const auto & field_azimuth = input.fields.at(static_cast<size_t>(PointIndex::Azimuth));
  same_layout &= field_azimuth.name == "azimuth";
  same_layout &= field_azimuth.offset == offsetof(PointXYZIRADRT, azimuth);
  same_layout &= field_azimuth.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_azimuth.count == 1;
  const auto & field_distance = input.fields.at(static_cast<size_t>(PointIndex::Distance));
  same_layout &= field_distance.name == "distance";
  same_layout &= field_distance.offset == offsetof(PointXYZIRADRT, distance);
  same_layout &= field_distance.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_distance.count == 1;
  const auto & field_return_type = input.fields.at(static_cast<size_t>(PointIndex::ReturnType));
  same_layout &= field_return_type.name == "return_type";
  same_layout &= field_return_type.offset == offsetof(PointXYZIRADRT, return_type);
  same_layout &= field_return_type.datatype == sensor_msgs::msg::PointField::UINT8;
  same_layout &= field_return_type.count == 1;
  const auto & field_time_stamp = input.fields.at(static_cast<size_t>(PointIndex::TimeStamp));
  same_layout &= field_time_stamp.name == "time_stamp";
  same_layout &= field_time_stamp.offset == offsetof(PointXYZIRADRT, time_stamp);
  same_layout &= field_time_stamp.datatype == sensor_msgs::msg::PointField::FLOAT64;
  same_layout &= field_time_stamp.count == 1;
  return same_layout;
}

}  // namespace pointcloud_preprocessor::utils
