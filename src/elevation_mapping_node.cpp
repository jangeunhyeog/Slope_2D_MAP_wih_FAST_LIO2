#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <map>
#include <set>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

using namespace std::chrono_literals;

class ElevationMappingNode : public rclcpp::Node
{
public:
    ElevationMappingNode() : Node("elevation_mapping_node")
    {
        // Parameters
        this->declare_parameter<double>("grid_resolution", 0.1);
        this->declare_parameter<double>("robot_height", 2.0);
        this->declare_parameter<double>("obstacle_height_thresh", 0.3);
        this->declare_parameter<double>("max_slope_degree", 30.0);
        this->declare_parameter<int>("min_valid_points", 3); // Noise filter
        this->declare_parameter<double>("sensor_height", 0.0); // Sensor height from ground
        this->declare_parameter<double>("ceiling_cutoff", 2.0); // Relative height to ignore as ceiling
        this->declare_parameter<std::string>("input_topic", "/cloud_registered");
        this->declare_parameter<std::string>("map_frame_id", "camera_init");
        this->declare_parameter<std::string>("robot_frame_id", "body"); 

        this->get_parameter("grid_resolution", grid_resolution_);
        this->get_parameter("robot_height", robot_height_);
        this->get_parameter("obstacle_height_thresh", obstacle_height_thresh_);
        this->get_parameter("max_slope_degree", max_slope_degree_);
        this->get_parameter("min_valid_points", min_valid_points_);
        this->get_parameter("sensor_height", sensor_height_);
        this->get_parameter("ceiling_cutoff", ceiling_cutoff_);
        
        std::string input_topic;
        this->get_parameter("input_topic", input_topic);
        this->get_parameter("map_frame_id", map_frame_id_);
        this->get_parameter("robot_frame_id", robot_frame_id_);

        // Height resolution for vertical discretization (e.g., 5cm)
        vertical_resolution_ = 0.05; 

        // Initialize bounds
        min_grid_x_ = std::numeric_limits<int>::max();
        max_grid_x_ = std::numeric_limits<int>::min();
        min_grid_y_ = std::numeric_limits<int>::max();
        max_grid_y_ = std::numeric_limits<int>::min();

        // TF Buffer & Listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribers & Publishers
        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, 10, std::bind(&ElevationMappingNode::cloudCallback, this, std::placeholders::_1));
        
        pub_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);
        pub_elevation_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/elevation_points", 1);
        
        // Timer for map publication (1 Hz)
        timer_ = this->create_wall_timer(
            1s, std::bind(&ElevationMappingNode::publishMap, this));

        RCLCPP_INFO(this->get_logger(), "Elevation Mapping Node Started (With Ceiling Filter).");
        RCLCPP_INFO(this->get_logger(), "Grid: %.2f, MinPts: %d, Slope: %.1f, SensorH: %.2f, Ceiling: %.1f", 
                    grid_resolution_, min_valid_points_, max_slope_degree_, sensor_height_, ceiling_cutoff_);
    }

private:
    struct GridCell {
        std::map<int, int> z_histogram; // z_idx -> hit_count
    };

    struct ProcessedCell {
        double ground_z;
        bool is_obstacle;
        bool has_data;
    };

    std::map<std::pair<int, int>, GridCell> elevation_map_;

    double grid_resolution_;
    double robot_height_;
    double obstacle_height_thresh_;
    double max_slope_degree_;
    double vertical_resolution_;
    int min_valid_points_;
    double sensor_height_;
    double ceiling_cutoff_;
    
    // Track active map area
    int min_grid_x_, max_grid_x_;
    int min_grid_y_, max_grid_y_;

    std::string map_frame_id_;
    std::string robot_frame_id_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_elevation_points_;
    rclcpp::TimerBase::SharedPtr timer_;

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        for (const auto& pt : cloud.points)
        {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
                continue;

            int grid_x = static_cast<int>(std::floor(pt.x / grid_resolution_));
            int grid_y = static_cast<int>(std::floor(pt.y / grid_resolution_));
            int z_idx = static_cast<int>(std::floor(pt.z / vertical_resolution_));

            // Accumulate hit count per Z-level
            elevation_map_[{grid_x, grid_y}].z_histogram[z_idx]++;

            if (grid_x < min_grid_x_) min_grid_x_ = grid_x;
            if (grid_x > max_grid_x_) max_grid_x_ = grid_x;
            if (grid_y < min_grid_y_) min_grid_y_ = grid_y;
            if (grid_y > max_grid_y_) max_grid_y_ = grid_y;
        }
    }

    void publishMap()
    {
        if (elevation_map_.empty()) return;

        // Step 0: Get Robot Position Mechanism
        double robot_x = 0.0, robot_y = 0.0, robot_z = 0.0;
        bool has_transform = false;
        try {
            // Need the latest transform
            if (tf_buffer_->canTransform(map_frame_id_, robot_frame_id_, tf2::TimePointZero)) {
                 geometry_msgs::msg::TransformStamped transform = 
                    tf_buffer_->lookupTransform(map_frame_id_, robot_frame_id_, tf2::TimePointZero);
                 robot_x = transform.transform.translation.x;
                 robot_y = transform.transform.translation.y;
                 robot_z = transform.transform.translation.z;
                 has_transform = true;
            }
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "TF Error: %s", ex.what());
        }

        nav_msgs::msg::OccupancyGrid map_msg;
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = map_frame_id_;
        
        map_msg.info.resolution = grid_resolution_;
        
        int width = max_grid_x_ - min_grid_x_ + 1;
        int height = max_grid_y_ - min_grid_y_ + 1;

        if (width * height > 100000000) { 
             RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Map is too large to publish: %dx%d", width, height);
             return;
        }

        map_msg.info.width = width;
        map_msg.info.height = height;
        map_msg.info.origin.position.x = min_grid_x_ * grid_resolution_;
        map_msg.info.origin.position.y = min_grid_y_ * grid_resolution_;
        map_msg.info.origin.position.z = 0.0;
        map_msg.info.origin.orientation.w = 1.0;

        map_msg.data.resize(width * height, -1);

        int gap_threshold_idx = static_cast<int>(robot_height_ / vertical_resolution_);
        int obs_threshold_idx = static_cast<int>(obstacle_height_thresh_ / vertical_resolution_);
        
        std::vector<ProcessedCell> processed_grid(width * height, {0.0, false, false});

        // Loop 1: Filter Noise & Identify Ground/Obstacles
        for (auto const& [key, cell] : elevation_map_)
        {
            int dx = key.first - min_grid_x_;
            int dy = key.second - min_grid_y_;
            if (dx < 0 || dx >= width || dy < 0 || dy >= height) continue;
            int idx = dy * width + dx;

            // --- Noise Filter & Ceiling Filter ---
            std::vector<int> valid_z_indices;
            for (auto const& [z, count] : cell.z_histogram) {
                if (count >= min_valid_points_) {
                    // Check ceiling threshold if transform is available
                    if (has_transform) {
                        double z_m = z * vertical_resolution_;
                        if (z_m > (robot_z + ceiling_cutoff_)) {
                            continue; // Ignore this point, it's too high (Ceiling)
                        }
                    }
                    valid_z_indices.push_back(z);
                }
            }
            if (valid_z_indices.empty()) continue; 
            
            // --- Ground Identification ---
            // Smallest valid z is ground
            int ground_z_idx = valid_z_indices[0]; 
            processed_grid[idx].ground_z = ground_z_idx * vertical_resolution_;
            processed_grid[idx].has_data = true;

            // --- Ceiling & Obstacle Check ---
            // We still do this to detect obstacles ON the ground (boxes, walls)
            // effective_max_z calculated relative to the identified ground
            int prev_z_idx = ground_z_idx;
            int effective_max_z_idx = ground_z_idx;

            for (size_t i = 1; i < valid_z_indices.size(); ++i) {
                int current_z_idx = valid_z_indices[i];
                if (current_z_idx - prev_z_idx > gap_threshold_idx) {
                    // Gap detected -> This is ceiling (relative to this ground), ignore higher points
                    break;
                }
                effective_max_z_idx = current_z_idx;
                prev_z_idx = current_z_idx;
            }

            if ((effective_max_z_idx - ground_z_idx) > obs_threshold_idx) {
                processed_grid[idx].is_obstacle = true;
            }
        }

        // Loop 2: Robot Footprint Correction
        // Force the area under the robot to be traversable
        if (has_transform) {
            int rx = static_cast<int>(std::floor(robot_x / grid_resolution_));
            int ry = static_cast<int>(std::floor(robot_y / grid_resolution_));
            
            int radius = 3; // e.g. 3 cells radius ~ 30cm radius 
            for (int dy = -radius; dy <= radius; dy++) {
                for (int dx = -radius; dx <= radius; dx++) {
                   int img_x = rx + dx - min_grid_x_;
                   int img_y = ry + dy - min_grid_y_;
                   
                   if (img_x >= 0 && img_x < width && img_y >= 0 && img_y < height) {
                       int idx = img_y * width + img_x;
                       
                       // Check if within circular radius
                       if (dx*dx + dy*dy <= radius*radius) {
                            // Clear obstacle flag
                            processed_grid[idx].is_obstacle = false;
                            
                            // If no data was present, we inject fake ground data at robot's Z
                            // so logic below treats it as valid traversable space
                            if (!processed_grid[idx].has_data) {
                                processed_grid[idx].has_data = true;
                                // Correct ground height: Robot Z - Sensor Height
                                processed_grid[idx].ground_z = robot_z - sensor_height_;
                            }
                       }
                   }
                }
            }
        }

        // Initialize PointCloud for visualization
        pcl::PointCloud<pcl::PointXYZRGB> elevation_pcl;

        // Loop 3: Gradient Check & Final Map Population
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int idx = y * width + x;
                const auto& current_cell = processed_grid[idx];

                if (!current_cell.has_data) {
                    map_msg.data[idx] = -1; 
                    continue;
                }

                // Add to PointCloud (Center of cell)
                pcl::PointXYZRGB pt;
                pt.x = min_grid_x_ * grid_resolution_ + x * grid_resolution_ + grid_resolution_/2.0;
                pt.y = min_grid_y_ * grid_resolution_ + y * grid_resolution_ + grid_resolution_/2.0;
                pt.z = current_cell.ground_z;

                bool is_obstacle_final = false;

                if (current_cell.is_obstacle) {
                    map_msg.data[idx] = 100;
                    is_obstacle_final = true;
                } else {
                    // --- Plane Fitting (PCA) for Gradient ---
                    // Gather 8 neighbors + center
                    Eigen::MatrixXd points(3, 9); 
                    int count = 0;
                    
                    // Add center
                    points.col(count++) = Eigen::Vector3d(x * grid_resolution_, y * grid_resolution_, current_cell.ground_z);

                    // Add neighbors
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            if (dx == 0 && dy == 0) continue;
                            int nx = x + dx;
                            int ny = y + dy;
                            if (nx >=0 && nx < width && ny >=0 && ny < height) {
                                int n_idx = ny * width + nx;
                                // Only consider valid data points for plane fitting
                                if (processed_grid[n_idx].has_data) {
                                    // If neighbor is obstacle, it acts as a high point, contributing to slope
                                    points.col(count++) = Eigen::Vector3d(nx * grid_resolution_, ny * grid_resolution_, processed_grid[n_idx].ground_z);
                                }
                            }
                        }
                    }

                    if (count < 3) {
                        // Not enough points to fit plane -> assume flat
                        map_msg.data[idx] = 0; 
                    } else {
                        // Calculate Normal
                        Eigen::MatrixXd pts = points.leftCols(count);
                        Eigen::Vector3d centroid = pts.rowwise().mean();
                        Eigen::MatrixXd centered = pts.colwise() - centroid;
                        Eigen::Matrix3d cov = centered * centered.transpose();
                        
                        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
                        // The eigenvector corresponding to the SMALLEST eigenvalue is the normal
                        Eigen::Vector3d normal = solver.eigenvectors().col(0); 
        
                        // Angle with Z-axis (0,0,1)
                        // dot(n, z) = n.z()
                        double angle = std::acos(std::abs(normal.z())); 
                        double slope_deg = angle * 180.0 / M_PI;
        
                        if (slope_deg > max_slope_degree_) {
                            map_msg.data[idx] = 100; // Too steep
                            is_obstacle_final = true;
                        } else {
                            map_msg.data[idx] = 0;
                        }
                    }
                }

                if (is_obstacle_final) {
                    pt.r = 255; pt.g = 0; pt.b = 0; // Red for Obstacle
                } else {
                    pt.r = 0; pt.g = 255; pt.b = 0; // Green for Ground
                }
                elevation_pcl.push_back(pt);
            }
        }
        
        pub_map_->publish(map_msg);

        // Publish PointCloud
        if (!elevation_pcl.empty()) {
            sensor_msgs::msg::PointCloud2 pcl_msg;
            pcl::toROSMsg(elevation_pcl, pcl_msg);
            pcl_msg.header = map_msg.header;
            pub_elevation_points_->publish(pcl_msg);
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ElevationMappingNode>());
    rclcpp::shutdown();
    return 0;
}
