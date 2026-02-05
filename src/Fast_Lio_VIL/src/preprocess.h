// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum LID_TYPE
{
  AVIA = 1,
  VELO16,
  OUST64,
  MID360
};  //{1, 2, 3}
enum TIME_UNIT
{
  SEC = 0,
  MS = 1,
  US = 2,
  NS = 3
};


namespace velodyne_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(float, time, time)(uint16_t, ring,
                                                                                                        ring))

namespace ouster_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;
  uint16_t reflectivity;
  uint8_t ring;
  uint16_t ambient;
  uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

namespace livox_ros
{
typedef struct {
  float x;            /**< X axis, Unit:m */
  float y;            /**< Y axis, Unit:m */
  float z;            /**< Z axis, Unit:m */
  float reflectivity; /**< Reflectivity   */
  uint8_t tag;        /**< Livox point tag   */
  uint8_t line;       /**< Laser line id     */
} LivoxPointXyzrtl;

typedef struct {
  float x;            /**< X axis, Unit:m */
  float y;            /**< Y axis, Unit:m */
  float z;            /**< Z axis, Unit:m */
  float intensity;    /**< Intensity   */
  uint8_t tag;        /**< Livox point tag   */
  uint8_t line;       /**< Laser line id     */
} LivoxPointXyzitl;
}
POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::LivoxPointXyzrtl,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, reflectivity, reflectivity)
    (uint8_t, tag, tag)
    (uint8_t, line, line)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::LivoxPointXyzitl,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint8_t, tag, tag)
    (uint8_t, line, line)
)

class Preprocess
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();
  
  void process(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void process(const sensor_msgs::msg::PointCloud2::UniquePtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void set(int lid_type, double bld, int pfilt_num);

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudXYZI pl_full, pl_surf;
  PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
  float time_unit_scale;
  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit;
  double blind;
  bool given_offset_time;

private:
  void avia_handler(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg);
  void oust64_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
  void velodyne_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
  void mid360_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
  void default_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
  
  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
