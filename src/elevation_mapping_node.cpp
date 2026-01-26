#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
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
        this->declare_parameter<double>("max_slope_degree", 30.0); // New parameter
        this->declare_parameter<std::string>("input_topic", "/cloud_registered");
        this->declare_parameter<std::string>("map_frame_id", "camera_init");

        this->get_parameter("grid_resolution", grid_resolution_);
        this->get_parameter("robot_height", robot_height_);
        this->get_parameter("obstacle_height_thresh", obstacle_height_thresh_);
        this->get_parameter("max_slope_degree", max_slope_degree_); // New parameter
        std::string input_topic;
        this->get_parameter("input_topic", input_topic);
        this->get_parameter("map_frame_id", map_frame_id_);

        // Height resolution for vertical discretization (e.g., 5cm)
        vertical_resolution_ = 0.05; 

        // Initialize bounds
        min_grid_x_ = std::numeric_limits<int>::max();
        max_grid_x_ = std::numeric_limits<int>::min();
        min_grid_y_ = std::numeric_limits<int>::max();
        max_grid_y_ = std::numeric_limits<int>::min();

        // Subscribers & Publishers
        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, 10, std::bind(&ElevationMappingNode::cloudCallback, this, std::placeholders::_1));
        
        pub_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);
        
        // Timer for map publication (1 Hz)
        timer_ = this->create_wall_timer(
            1s, std::bind(&ElevationMappingNode::publishMap, this));

        RCLCPP_INFO(this->get_logger(), "Elevation Mapping Node Started (Traversability Logic).");
        RCLCPP_INFO(this->get_logger(), "Grid Res: %.2f, Obstacle Thresh: %.2f, Max Slope: %.1f deg", 
                    grid_resolution_, obstacle_height_thresh_, max_slope_degree_);
    }

private:
    struct GridCell {
        std::set<int> z_indices; 
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
    
    // Track active map area
    int min_grid_x_, max_grid_x_;
    int min_grid_y_, max_grid_y_;

    std::string map_frame_id_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
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

            elevation_map_[{grid_x, grid_y}].z_indices.insert(z_idx);

            if (grid_x < min_grid_x_) min_grid_x_ = grid_x;
            if (grid_x > max_grid_x_) max_grid_x_ = grid_x;
            if (grid_y < min_grid_y_) min_grid_y_ = grid_y;
            if (grid_y > max_grid_y_) max_grid_y_ = grid_y;
        }
    }

    void publishMap()
    {
        if (elevation_map_.empty()) return;

        nav_msgs::msg::OccupancyGrid map_msg;
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = map_frame_id_;
        
        map_msg.info.resolution = grid_resolution_;
        
        int width = max_grid_x_ - min_grid_x_ + 1;
        int height = max_grid_y_ - min_grid_y_ + 1;

        if (width * height > 100000000) { 
             RCLCPP_WARN(this->get_logger(), "Map too large: %dx%d", width, height);
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
        double tan_max_slope = std::tan(max_slope_degree_ * M_PI / 180.0);

        // First Pass: Calculate Ground & Obstacle Status per cell
        // We use a temporary map because we need neighbors' ground_z for slope check
        // Using a map is slow, but simpler. For speed, we could use a 2D vector relative to min_grid.
        
        std::vector<ProcessedCell> processed_grid(width * height, {0.0, false, false});

        for (auto const& [key, cell] : elevation_map_)
        {
            if (cell.z_indices.empty()) continue;

            int dx = key.first - min_grid_x_;
            int dy = key.second - min_grid_y_;
            if (dx < 0 || dx >= width || dy < 0 || dy >= height) continue;
            int idx = dy * width + dx;

            // 1. Identify Ground
            auto it = cell.z_indices.begin();
            int ground_z_idx = *it;
            processed_grid[idx].ground_z = ground_z_idx * vertical_resolution_;
            processed_grid[idx].has_data = true;

            // 2. Filter Ceiling (Find effective max z)
            int prev_z_idx = ground_z_idx;
            int effective_max_z_idx = ground_z_idx;

            for (++it; it != cell.z_indices.end(); ++it)
            {
                int current_z_idx = *it;
                if (current_z_idx - prev_z_idx > gap_threshold_idx) {
                    // Gap detected -> Ceiling. Stop.
                    break;
                }
                effective_max_z_idx = current_z_idx;
                prev_z_idx = current_z_idx;
            }

            // 3. Obstacle Check (Relative Height)
            if ((effective_max_z_idx - ground_z_idx) > obs_threshold_idx)
            {
                processed_grid[idx].is_obstacle = true;
            }
        }

        // Second Pass: Fill Occupancy Grid with Slope Check
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int idx = y * width + x;
                const auto& current_cell = processed_grid[idx];

                if (!current_cell.has_data) {
                    map_msg.data[idx] = -1; // Unknown
                    continue;
                }

                if (current_cell.is_obstacle) {
                    map_msg.data[idx] = 100; // Wall/Obstacle
                    continue;
                }

                // 4. Slope Check
                // Check neighbors (4-connectivity)
                double max_height_diff = 0.0;
                int valid_neighbors = 0;
                int neighbors[4][2] = {{0,1}, {0,-1}, {1,0}, {-1,0}};

                for (auto& n : neighbors) {
                    int nx = x + n[0];
                    int ny = y + n[1];
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        int n_idx = ny * width + nx;
                        if (processed_grid[n_idx].has_data) {
                            double diff = std::abs(current_cell.ground_z - processed_grid[n_idx].ground_z);
                            if (diff > max_height_diff) max_height_diff = diff;
                            valid_neighbors++;
                        }
                    }
                }

                // Calculate slope: tan(theta) = diff / resolution
                // We check if diff > resolution * tan(max_slope)
                // Note: This is an approximation. Ideally sqrt(dx^2 + dy^2).
                if (max_height_diff > (grid_resolution_ * tan_max_slope)) {
                     map_msg.data[idx] = 100; // Too steep -> Treated as obstacle
                } else {
                     map_msg.data[idx] = 0; // Free and Traversable
                }
            }
        }

        pub_map_->publish(map_msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ElevationMappingNode>());
    rclcpp::shutdown();
    return 0;
}
