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
        this->declare_parameter<double>("grid_resolution", 0.2);
        this->declare_parameter<double>("robot_height", 2.0);
        this->declare_parameter<double>("obstacle_height_thresh", 0.3);
        this->declare_parameter<double>("max_slope_degree", 30.0);
        this->declare_parameter<int>("min_valid_points", 4); // Noise filter
        this->declare_parameter<double>("sensor_height", 0.0); // Sensor height from ground
        this->declare_parameter<double>("ceiling_cutoff", 2.0); // Relative height to ignore as ceiling
        this->declare_parameter<double>("local_map_size", 30.0); // Local map size in meters
        this->declare_parameter<double>("active_map_size", 30.0); // Active calculation area

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
        this->get_parameter("local_map_size", local_map_size_);
        this->get_parameter("active_map_size", active_map_size_);
        
        std::string input_topic;
        this->get_parameter("input_topic", input_topic);
        this->get_parameter("map_frame_id", map_frame_id_);
        this->get_parameter("robot_frame_id", robot_frame_id_);

        // Height resolution for vertical discretization (e.g., 5cm)
        vertical_resolution_ = 0.05; 

        // Initialize bounds (Global Bounds Tracker)
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
        pub_total_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/total_map", 1);
        pub_elevation_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/elevation_points", 1);
        
        // Timer for map publication (1 Hz)
        timer_ = this->create_wall_timer(
            1s, std::bind(&ElevationMappingNode::publishMap, this));

        // Timer for total map publication (0.2 Hz = 5s)
        total_map_timer_ = this->create_wall_timer(
            5s, std::bind(&ElevationMappingNode::publishTotalMap, this));
            
        // Timer for memory pruning (0.5 Hz = 2s)
        prune_timer_ = this->create_wall_timer(
            2s, std::bind(&ElevationMappingNode::pruneMap, this));

        RCLCPP_INFO(this->get_logger(), "Elevation Mapping Node Started (With Ceiling Filter).");
        RCLCPP_INFO(this->get_logger(), "Grid: %.2f, LocalMap: %.1fm, MinPts: %d, Slope: %.1f", 
                    grid_resolution_, local_map_size_, min_valid_points_, max_slope_degree_);
    }

private:
    struct GridCell {
        std::map<int, int> z_histogram; // Heavy (Kept only for active area)
        bool visited = false; // New: Marks if this cell has been processed in Local Map
        
        // Cache: Save last calculation results from generateGrid
        float last_h = 0.0f;
        bool last_is_obs = false;
        bool has_calculated = false; 
    };
    
    struct StaticCell {
        float height;
        bool is_obstacle;
    };

    struct ProcessedCell {
        double ground_z;
        bool is_obstacle;
        bool has_data;
    };

    std::map<std::pair<int, int>, GridCell> elevation_map_; // Active Map (Heavy)
    std::map<std::pair<int, int>, StaticCell> static_map_;  // Frozen Map (Light)

    double grid_resolution_;
    double robot_height_;
    double obstacle_height_thresh_;
    double max_slope_degree_;
    double vertical_resolution_;
    int min_valid_points_;
    double sensor_height_;
    double ceiling_cutoff_;
    double local_map_size_;
    double active_map_size_;
    
    // Track active map area
    int min_grid_x_, max_grid_x_;
    int min_grid_y_, max_grid_y_;

    std::string map_frame_id_;
    std::string robot_frame_id_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_total_map_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_elevation_points_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr total_map_timer_;
    rclcpp::TimerBase::SharedPtr prune_timer_;

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
            
            // Note: We allow active map updates on top of history. 
            // History is preserved and updated explicitly by generateGrid -> static_map.

            if (grid_x < min_grid_x_) min_grid_x_ = grid_x;
            if (grid_x > max_grid_x_) max_grid_x_ = grid_x;
            if (grid_y < min_grid_y_) min_grid_y_ = grid_y;
            if (grid_y > max_grid_y_) max_grid_y_ = grid_y;
        }
    }
    
    void pruneMap()
    {
        // 1. Get Robot Position
        double robot_x = 0.0, robot_y = 0.0;
        try {
           if (tf_buffer_->canTransform(map_frame_id_, robot_frame_id_, tf2::TimePointZero)) {
                auto tf = tf_buffer_->lookupTransform(map_frame_id_, robot_frame_id_, tf2::TimePointZero);
                robot_x = tf.transform.translation.x;
                robot_y = tf.transform.translation.y;
           } else {
               return; // No TF, can't determine active area
           }
        } catch (...) { return; }

        int center_x = static_cast<int>(std::floor(robot_x / grid_resolution_));
        int center_y = static_cast<int>(std::floor(robot_y / grid_resolution_));
        int active_rad = static_cast<int>((active_map_size_ / 2.0) / grid_resolution_);
        
        // Bounds defining Active Area
        int min_ax = center_x - active_rad;
        int max_ax = center_x + active_rad;
        int min_ay = center_y - active_rad;
        int max_ay = center_y + active_rad;

        // Iterate Active Map and identify out-of-bounds cells
        auto it = elevation_map_.begin();
        while (it != elevation_map_.end()) {
            int gx = it->first.first;
            int gy = it->first.second;
            
            if (gx < min_ax || gx > max_ax || gy < min_ay || gy > max_ay) {
                // Remove cells outside the active area.
                // Note: Persistent data is already saved to static_map_ during Local Map generation.
                it = elevation_map_.erase(it);
            } else {
                ++it;
            }
        }
    }



    void publishTotalMap()
    {
         if (static_map_.empty()) return; // Only publish history
         if (pub_total_map_->get_subscription_count() == 0) return; 

         // Calculate bounds of Static Map
         int min_x = std::numeric_limits<int>::max();
         int max_x = std::numeric_limits<int>::min();
         int min_y = std::numeric_limits<int>::max();
         int max_y = std::numeric_limits<int>::min();

         for (const auto& kv : static_map_) {
             if (kv.first.first < min_x) min_x = kv.first.first;
             if (kv.first.first > max_x) max_x = kv.first.first;
             if (kv.first.second < min_y) min_y = kv.first.second;
             if (kv.first.second > max_y) max_y = kv.first.second;
         }

         int width = max_x - min_x + 1;
         int height = max_y - min_y + 1;
         
         if (width * height > 200000000) { 
              RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Total Map too large: %dx%d", width, height);
              return;
         }
         
         nav_msgs::msg::OccupancyGrid map_msg;
         map_msg.header.stamp = this->now();
         map_msg.header.frame_id = map_frame_id_;
         map_msg.info.resolution = grid_resolution_;
         map_msg.info.width = width;
         map_msg.info.height = height;
         map_msg.info.origin.position.x = min_x * grid_resolution_;
         map_msg.info.origin.position.y = min_y * grid_resolution_;
         map_msg.info.origin.position.z = 0.0;
         map_msg.info.origin.orientation.w = 1.0;
         map_msg.data.resize(width * height, -1);

         // Fill from Static Map ONLY
         for (const auto& kv : static_map_) {
             int x = kv.first.first - min_x;
             int y = kv.first.second - min_y;
             if (x >= 0 && x < width && y >= 0 && y < height) {
                 int idx = y * width + x;
                 if (kv.second.is_obstacle) map_msg.data[idx] = 100;
                 else map_msg.data[idx] = 0;
             }
         }
         
         pub_total_map_->publish(map_msg);
    }

    void publishMap()
    {
        if (elevation_map_.empty() && static_map_.empty()) return;

        // Step 0: Get Robot Position
        double robot_x = 0.0, robot_y = 0.0, robot_z = 0.0;
        bool has_transform = false;
        try {
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

        // --- Calculate Local Window Bounds ---
        int local_radius_cells = static_cast<int>((local_map_size_ / 2.0) / grid_resolution_);
        int center_grid_x = static_cast<int>(std::floor(robot_x / grid_resolution_));
        int center_grid_y = static_cast<int>(std::floor(robot_y / grid_resolution_));

        int window_min_x = center_grid_x - local_radius_cells;
        int window_max_x = center_grid_x + local_radius_cells;
        int window_min_y = center_grid_y - local_radius_cells;
        int window_max_y = center_grid_y + local_radius_cells;

        pcl::PointCloud<pcl::PointXYZRGB> elevation_pcl;
        
        // IMPORTANT: For Local Map, mark cells as VISITED
        auto map_msg = generateGrid(window_min_x, window_max_x, window_min_y, window_max_y, 
                                    true, &elevation_pcl, has_transform, robot_x, robot_y, robot_z, true);

        if (map_msg.info.width > 0) {
            pub_map_->publish(map_msg);
        }

        if (!elevation_pcl.empty()) {
            sensor_msgs::msg::PointCloud2 pcl_msg;
            pcl::toROSMsg(elevation_pcl, pcl_msg);
            pcl_msg.header = map_msg.header;
            pub_elevation_points_->publish(pcl_msg);
        }
    }



    // --- Helper Functions ---

    bool computeCellProperties(const GridCell& cell, float& out_height, bool& out_is_obs)
    {
        int gap_thresh = static_cast<int>(robot_height_ / vertical_resolution_);
        int obs_thresh = static_cast<int>(obstacle_height_thresh_ / vertical_resolution_);

        std::vector<int> valid_z;
        for (auto const& [z, count] : cell.z_histogram) {
            if (count >= min_valid_points_) valid_z.push_back(z);
        }
        
        if (valid_z.empty()) return false;

        int g_idx = valid_z[0];
        int prev_z = g_idx;
        int max_z = g_idx;
        for (size_t i = 1; i < valid_z.size(); ++i) {
            int current_z = valid_z[i];
            if (current_z - prev_z > gap_thresh) break;
            max_z = current_z;
            prev_z = current_z;
        }
        
        out_height = g_idx * vertical_resolution_;
        out_is_obs = ((max_z - g_idx) > obs_thresh);
        return true;
    }

    nav_msgs::msg::OccupancyGrid generateGrid(int min_x, int max_x, int min_y, int max_y, 
                                              bool apply_dilation, 
                                              pcl::PointCloud<pcl::PointXYZRGB>* out_pcl = nullptr,
                                              bool has_robot_pose = false, double rx = 0, double ry = 0, double rz = 0,
                                              bool mark_as_visited = false)
    {
        nav_msgs::msg::OccupancyGrid map_msg;
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = map_frame_id_;
        
        int width = max_x - min_x + 1;
        int height = max_y - min_y + 1;

        if (width <= 0 || height <= 0) return map_msg;

        map_msg.info.resolution = grid_resolution_;
        map_msg.info.width = width;
        map_msg.info.height = height;
        map_msg.info.origin.position.x = min_x * grid_resolution_;
        map_msg.info.origin.position.y = min_y * grid_resolution_;
        map_msg.info.origin.position.z = 0.0;
        map_msg.info.origin.orientation.w = 1.0;
        map_msg.data.resize(width * height, -1);

        std::vector<ProcessedCell> processed_grid(width * height, {0.0, false, false});
        
        int gap_thresh = static_cast<int>(robot_height_ / vertical_resolution_);
        int obs_thresh = static_cast<int>(obstacle_height_thresh_ / vertical_resolution_);

        // 1. Process Height Data
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int grid_x = min_x + x;
                int grid_y = min_y + y;
                
                auto it = elevation_map_.find({grid_x, grid_y});
                if (it != elevation_map_.end()) {
                    auto& cell = it->second;
                    int idx = y * width + x;

                    // Optimization: Skip unvisited cells if not generating Local Map
                    if (!cell.visited && !mark_as_visited) {
                        continue; 
                    }

                    // Only mark as visited if explicitly requested (Local Map only)
                    if (mark_as_visited) {
                        cell.visited = true;
                    }

                    // --- Noise Filter & Ceiling Filter ---
                    std::vector<int> valid_z_indices;
                    for (auto const& [z, count] : cell.z_histogram) {
                        if (count >= min_valid_points_) {
                        // Check ceiling threshold if transform is available
                            if (has_robot_pose) {
                                double z_m = z * vertical_resolution_;
                                if (z_m > (rz + ceiling_cutoff_)) {
                                    continue; // Ignore this point, it's too high (Ceiling)
                                }
                            }
                            valid_z_indices.push_back(z);
                        }
                    }
                    if (valid_z_indices.empty()) continue; 
                    
                    // --- Ground Identification ---
                    int ground_z_idx = valid_z_indices[0]; 
                    processed_grid[idx].ground_z = ground_z_idx * vertical_resolution_;
                    processed_grid[idx].has_data = true;

                    // --- Ceiling & Obstacle Check ---
                    int prev_z_idx = ground_z_idx;
                    int effective_max_z_idx = ground_z_idx;

                    for (size_t i = 1; i < valid_z_indices.size(); ++i) {
                        int current_z_idx = valid_z_indices[i];
                        if (current_z_idx - prev_z_idx > gap_thresh) {
                            break;
                        }
                        effective_max_z_idx = current_z_idx;
                        prev_z_idx = current_z_idx;
                    }

                    if ((effective_max_z_idx - ground_z_idx) > obs_thresh) {
                        processed_grid[idx].is_obstacle = true;
                    }
                    continue; // Done with this cell
                }
                
                // [NOTE] Static Map is only used for Total Map generation, not here.
            }
        }

        // Loop 2: Robot Footprint Correction
        // Force the area under the robot to be traversable
        if (has_robot_pose) {
            int grid_rx = static_cast<int>(std::floor(rx / grid_resolution_));
            int grid_ry = static_cast<int>(std::floor(ry / grid_resolution_));
            
            int radius = 3; 
            for (int dy = -radius; dy <= radius; dy++) {
                for (int dx = -radius; dx <= radius; dx++) {
                   int global_x = grid_rx + dx;
                   int global_y = grid_ry + dy;
                   
                   // Convert to window coords
                   int img_x = global_x - min_x;
                   int img_y = global_y - min_y;

                   if (img_x >= 0 && img_x < width && img_y >= 0 && img_y < height) {
                       int idx = img_y * width + img_x;
                       
                       if (dx*dx + dy*dy <= radius*radius) {
                            processed_grid[idx].is_obstacle = false;
                            
                            if (!processed_grid[idx].has_data) {
                                processed_grid[idx].has_data = true;
                                processed_grid[idx].ground_z = rz - sensor_height_;
                            }
                       }
                   }
                }
            }
        }

        // Initialize PointCloud for visualization
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
                pt.x = min_x * grid_resolution_ + x * grid_resolution_ + grid_resolution_/2.0;
                pt.y = min_y * grid_resolution_ + y * grid_resolution_ + grid_resolution_/2.0;
                pt.z = current_cell.ground_z;

                bool is_obstacle_final = false;

                if (current_cell.is_obstacle) {
                    map_msg.data[idx] = 100;
                    is_obstacle_final = true;
                } else {
                    // --- Plane Fitting (PCA) for Gradient ---
                    Eigen::MatrixXd points(3, 9); 
                    int count = 0;
                    
                    points.col(count++) = Eigen::Vector3d(x * grid_resolution_, y * grid_resolution_, current_cell.ground_z);

                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            if (dx == 0 && dy == 0) continue;
                            int nx = x + dx;
                            int ny = y + dy;
                            if (nx >=0 && nx < width && ny >=0 && ny < height) {
                                int n_idx = ny * width + nx;
                                if (processed_grid[n_idx].has_data) {
                                    points.col(count++) = Eigen::Vector3d(nx * grid_resolution_, ny * grid_resolution_, processed_grid[n_idx].ground_z);
                                }
                            }
                        }
                    }

                    if (count < 3) {
                        map_msg.data[idx] = 0; 
                    } else {
                        Eigen::MatrixXd pts = points.leftCols(count);
                        Eigen::Vector3d centroid = pts.rowwise().mean();
                        Eigen::MatrixXd centered = pts.colwise() - centroid;
                        Eigen::Matrix3d cov = centered * centered.transpose();
                        
                        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
                        Eigen::Vector3d normal = solver.eigenvectors().col(0); 
        
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
                
                // CACHE RESULTS: Write back to Elevation Map for Pruning
                int grid_x = min_x + x;
                int grid_y = min_y + y;
                auto it = elevation_map_.find({grid_x, grid_y});
                if (it != elevation_map_.end()) {
                    it->second.last_h = current_cell.ground_z;
                    it->second.last_is_obs = is_obstacle_final;
                    it->second.has_calculated = true;
                }
                
                // Accumulate to Total Map (History) if this is a Local Map update
                if (mark_as_visited) {
                     static_map_[{grid_x, grid_y}] = StaticCell{static_cast<float>(current_cell.ground_z), is_obstacle_final};
                }

                if (out_pcl) {
                    out_pcl->push_back(pt);
                }
            }
        }
        
        // --- Hole Filling (Dilation) Pass ---
        std::vector<int8_t> dilated_data = map_msg.data; 
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int idx = y * width + x;
                
                if (map_msg.data[idx] == -1) { // Only fill UNKNOWN space
                    // Check neighbors for FREE SPACE
                    bool neighbor_is_free = false;
                    int dxs[] = {0, 0, -1, 1};
                    int dys[] = {-1, 1, 0, 0};
                    
                    for(int i=0; i<4; ++i) {
                         int nx = x + dxs[i];
                         int ny = y + dys[i];
                         if(nx >=0 && nx < width && ny>=0 && ny < height) {
                             int n_idx = ny*width + nx;
                             if (map_msg.data[n_idx] == 0) { // If neighbor is Free (Traversable)
                                  neighbor_is_free = true;
                                  break;
                             }
                         }
                    }
                    
                    if (neighbor_is_free) {
                        dilated_data[idx] = 0; // Mark as Free
                        // Dilation doesn't necessarily mean we have height data, so we don't save to static_map from here effectively.
                    }
                }
            }
        }
        map_msg.data = dilated_data;

        return map_msg;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ElevationMappingNode>());
    rclcpp::shutdown();
    return 0;
}
