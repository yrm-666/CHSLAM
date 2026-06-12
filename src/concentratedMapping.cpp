#include "common.h"
#include "mapDatabase.h"
#include "systemMonitor.h"
#include "robustOptimizer.h"
#include "co_lrio/srv/save_files.hpp"
#include <rclcpp/serialization.hpp>
#include <filesystem>


namespace co_lrio
{
class ConcentratedMapping : public rclcpp::Node
{
private:
    /* parameter */
    ConcentratedMappingParams params;

    /* ros2 */
    // subscriber
    rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr sub_optimization_request;
    std::unordered_map<int, rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr> sub_imu_odometry;

    // publisher
    rclcpp::Publisher<co_lrio::msg::LoopClosure>::SharedPtr pub_loop_closure;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_loop_closure_constraints;
    std::unordered_map<int, rclcpp::Publisher<co_lrio::msg::OptimizationResponse>::SharedPtr> pub_optimization_response;
    std::unordered_map<int, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> pub_global_path;
    std::unordered_map<int, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pub_global_map;
    std::unordered_map<int, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pub_global_near_map;

    // service
    rclcpp::Service<co_lrio::srv::SaveFiles>::SharedPtr save_trajectories_service;

    // tf2
    std::unique_ptr<tf2_ros::TransformBroadcaster> world_2_odom_broadcaster;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::map<int8_t, gtsam::Pose3> trans_to_pub;
    std::map<int8_t, gtsam::Pose3> trans;

    // loop throttling state (intra / inter tracked independently, per robot):
    // streak = consecutive loops counted; anchor = pose of the most recent counted
    // loop; suppressed = category disabled until the robot leaves the anchor by
    // params.last_loop_dist_ meters
    std::mutex lock_on_loop_throttle;
    std::map<int8_t, int> intra_loop_streak, inter_loop_streak;
    std::map<int8_t, gtsam::Pose3> intra_loop_anchor, inter_loop_anchor;
    std::map<int8_t, bool> intra_loop_suppressed, inter_loop_suppressed;

    /* database */
    std::unique_ptr<MapDatabase> map_database;
    
    // robotic swarm information
    std::deque<int8_t> pub_map_for_robots;
    std::unordered_map<int, std::string> prefix;
    std::unordered_map<int, nav_msgs::msg::Path> global_path;
    std::unordered_map<int, deque<nav_msgs::msg::Odometry>> imu_odometry_queue;
    std::unordered_map<int, co_lrio::msg::OptimizationResponse> optimization_response_msg;
    std::unordered_map<int, std::unordered_map<int, pair<gtsam::Symbol, float>>> pairwise_distance;

    /* loop closure */
    int inter_robot_loop_ptr;

    bool update_path_flag;

    unique_ptr<ScanDescriptor> scan_descriptor;

    map<gtsam::Symbol, gtsam::Symbol> loop_indexes;
    map<gtsam::Symbol, gtsam::Symbol> loop_indexes_copy;

    /* global robust optimizer */
    unique_ptr<RobustOptimizer> optimizer;
    unique_ptr<OutlierRejection> outlier_reject;
    gtsam::Values global_optimized_keyposes_copy;

    // mutex
    std::mutex lock_on_optmizer;
    std::mutex lock_on_gtpath;
    std::mutex lock_on_imu;

    // global map visualization
    pcl::KdTreeFLANN<PointPose3D>::Ptr kdtree_history_keyposes;
    pcl::VoxelGrid<PointPose3D> downsample_filter_for_global_map;

    std::unique_ptr<systemMonitor> system_monitor;

public:
    ConcentratedMapping(const rclcpp::NodeOptions& options) : Node("ConcentratedMapping", options)
    {
        /* get parameter */
        // simulator mode
        this->declare_parameter("simulator_mode", false);
        this->get_parameter("simulator_mode", params.simulator_mode_);

        // save file directory
        this->declare_parameter("save_directory", "/root/cslam_ws/src/Co-LRIO/co_lrio_output");
        this->get_parameter("save_directory", params.save_directory_);
        

        // set logger level
        this->declare_parameter("loop_log_level", 20);
        this->get_parameter("loop_log_level", params.loop_log_level_);
        this->declare_parameter("loop_log_mini_level", 20);
        this->get_parameter("loop_log_mini_level", params.loop_log_mini_level_);
        this->declare_parameter("optimization_log_level", 20);
        this->get_parameter("optimization_log_level", params.optimization_log_level_);
        this->declare_parameter("optimization_log_mini_level", 20);
        this->get_parameter("optimization_log_mini_level", params.optimization_log_mini_level_);
        this->declare_parameter("uwb_log_level", 20);
        this->get_parameter("uwb_log_level", params.uwb_log_level_);
        this->declare_parameter("pcm_log_level", 20);
        this->get_parameter("pcm_log_level", params.pcm_log_level_);
        this->declare_parameter("map_log_mini_level", 20);
        this->get_parameter("map_log_mini_level", params.map_log_level_);
        auto return_value1 = rcutils_logging_set_logger_level("loop_log", params.loop_log_level_);
        auto return_value2 = rcutils_logging_set_logger_level("loop_log_mini", params.loop_log_mini_level_);
        auto return_value3 = rcutils_logging_set_logger_level("optimization_log", params.optimization_log_level_);
        auto return_value4 = rcutils_logging_set_logger_level("optimization_log_mini", params.optimization_log_mini_level_);
        auto return_value5 = rcutils_logging_set_logger_level("uwb_log", params.uwb_log_level_);
        auto return_value6 = rcutils_logging_set_logger_level("pcm_log", params.pcm_log_level_);
        auto return_value7 = rcutils_logging_set_logger_level("map_log", params.map_log_level_);

        // frames
        this->declare_parameter("odometry_frame", "/odom");
        this->get_parameter("odometry_frame", params.odometry_frame_);
        this->declare_parameter("world_frame", "/world");
        this->get_parameter("world_frame", params.world_frame_);

        // Optional manual robot initial poses in the shared world frame:
        // [x, y, z, roll, pitch, yaw]
        std::vector<double> manual_init_robot_0;
        std::vector<double> manual_init_robot_1;
        std::vector<double> manual_init_robot_2;
        this->declare_parameter("manual_init_robot_0", std::vector<double>{});
        this->get_parameter("manual_init_robot_0", manual_init_robot_0);
        this->declare_parameter("manual_init_robot_1", std::vector<double>{});
        this->get_parameter("manual_init_robot_1", manual_init_robot_1);
        this->declare_parameter("manual_init_robot_2", std::vector<double>{});
        this->get_parameter("manual_init_robot_2", manual_init_robot_2);

        // LiDAR setting
        string sensor_type;
        this->declare_parameter("sensor", "");
        this->get_parameter("sensor", sensor_type);
        if (sensor_type == "velodyne")
        {
            params.sensor_type_ = LiDARType::VELODYNE;
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger(""), "Invalid sensor type (must be either 'velodyne' or 'ouster'): %s ", sensor_type.c_str());
            rclcpp::shutdown();
        }
        this->declare_parameter("n_scan", 16);
        this->get_parameter("n_scan", params.n_scan_);
        RCLCPP_INFO(rclcpp::get_logger(""), "\033[1;32msensors: %s with lines %d.\033[0m", sensor_type.c_str(), params.n_scan_);

        // loop detection setting
        this->declare_parameter("enable_loop", true);
        this->get_parameter("enable_loop", params.enable_loop_);
        this->declare_parameter("inter_robot_max_distance", 0.0f);
        this->get_parameter("inter_robot_max_distance", params.inter_robot_max_distance_);
        this->declare_parameter("loop_closure_max_pose_distance", 50.0f);
        this->get_parameter("loop_closure_max_pose_distance", params.loop_closure_max_pose_distance_);
        this->declare_parameter("loop_streak_threshold", 5);
        this->get_parameter("loop_streak_threshold", params.loop_streak_threshold_);
        this->declare_parameter("last_loop_dist", 30.0f);
        this->get_parameter("last_loop_dist", params.last_loop_dist_);
        string descriptor_type;
        this->declare_parameter("descriptor_type", "");
        this->get_parameter("descriptor_type", descriptor_type);
        if (descriptor_type == "ScanContext")
        {
            params.descriptor_type_ = DescriptorType::ScanContext;
        }
        else if (descriptor_type == "LidarIris")
        {
            params.descriptor_type_ = DescriptorType::LidarIris;
        }
        else if (descriptor_type == "MSOLDescriptor")
        {
            params.descriptor_type_ = DescriptorType::MultiSector;
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger(""), "Invalid descriptor type: %s", descriptor_type.c_str());
            rclcpp::shutdown();
        }
        RCLCPP_INFO(rclcpp::get_logger(""), "\033[1;32mdescriptors: %s.\033[0m", descriptor_type.c_str());
        this->declare_parameter("distance_threshold", 0.25f);
        this->get_parameter("distance_threshold", params.distance_threshold_);
        this->declare_parameter("candidates_num", 20);
        this->get_parameter("candidates_num", params.candidates_num_);
        this->declare_parameter("max_radius", 80.0f);
        this->get_parameter("max_radius", params.max_radius_);
        this->declare_parameter("exclude_recent_num", 200);
        this->get_parameter("exclude_recent_num", params.exclude_recent_num_);
        this->declare_parameter("map_leaf_size", 0.2f);
        this->get_parameter("map_leaf_size", params.map_leaf_size_);

        // pcm configuration
        this->declare_parameter("loop_num_threshold", 2);
        this->get_parameter("loop_num_threshold", params.loop_num_threshold_);
        this->declare_parameter("pcm_threshold", 5.348f);
        this->get_parameter("pcm_threshold", params.pcm_threshold_);
        this->declare_parameter("use_pcm", true);
        this->get_parameter("use_pcm", params.use_pcm_);

        // uwb configuration
        this->declare_parameter("ranging_outlier_threshold", 0.3f);
        this->get_parameter("ranging_outlier_threshold", params.ranging_outlier_threshold_);
        this->declare_parameter("enable_ranging_outlier_threshold", false);
        this->get_parameter("enable_ranging_outlier_threshold", params.enable_ranging_outlier_threshold_);
        this->declare_parameter("minimum_ranging", 0.2f);
        this->get_parameter("minimum_ranging", params.minimum_ranging_);
        this->declare_parameter("enable_ranging", false);
        this->get_parameter("enable_ranging", params.enable_ranging_);
        this->declare_parameter("uwb_distance_thereshold_for_loop", 0.01);
        this->get_parameter("uwb_distance_thereshold_for_loop", params.uwb_distance_thereshold_for_loop_);

        // optimization setting
        this->declare_parameter("enable_risam", true);
        this->get_parameter("enable_risam", params.enable_risam_);
        this->declare_parameter("use_global_near_map", false);
        this->get_parameter("use_global_near_map", params.use_global_near_map_);
        this->declare_parameter("global_near_map", 20.0f);
        this->get_parameter("global_near_map", params.global_near_map_);

        // gps setting
        this->declare_parameter("use_gnss", false);
        this->get_parameter("use_gnss", params.use_gnss_);
        this->declare_parameter("use_rtk", false);
        this->get_parameter("use_rtk", params.use_rtk_);
        this->declare_parameter("gps_cov_threshold", 0.1);
        this->get_parameter("gps_cov_threshold", params.gps_cov_threshold_);
        this->declare_parameter("use_gps_elevation", false);
        this->get_parameter("use_gps_elevation", params.use_gps_elevation_);

        // ground plane factor
        this->declare_parameter("enable_ground_plane", false);
        this->get_parameter("enable_ground_plane", params.enable_ground_plane_);
        this->declare_parameter("ground_plane_z", 0.0);
        this->get_parameter("ground_plane_z", params.ground_plane_z_);
        this->declare_parameter("ground_plane_noise_z", 0.1f);
        this->get_parameter("ground_plane_noise_z", params.ground_plane_noise_z_);
        this->declare_parameter("ground_plane_noise_roll", 0.01f);
        this->get_parameter("ground_plane_noise_roll", params.ground_plane_noise_roll_);
        this->declare_parameter("ground_plane_noise_pitch", 0.01f);
        this->get_parameter("ground_plane_noise_pitch", params.ground_plane_noise_pitch_);
        this->declare_parameter("ground_plane_factor_interval", 20);
        this->get_parameter("ground_plane_factor_interval", params.ground_plane_factor_interval_);

        // cpu setting
        this->declare_parameter("pub_tf_interval", 0.01f);
        this->get_parameter("pub_tf_interval", params.pub_tf_interval_);
        this->declare_parameter("loop_detection_interval", 0.1f);
        this->get_parameter("loop_detection_interval", params.loop_detection_interval_);
        this->declare_parameter("pub_global_map_interval", 20.0f);
        this->get_parameter("pub_global_map_interval", params.pub_global_map_interval_);
        this->declare_parameter("global_map_visualization_radius", 200.0f);
        this->get_parameter("global_map_visualization_radius", params.global_map_visualization_radius_);
        this->declare_parameter("parallel_cpu_core", 8);
        this->get_parameter("parallel_cpu_core", params.parallel_cpu_core_);

        /* ros2 */
        // subscriber
        std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> optimization_request_callback =
            std::bind(&co_lrio::ConcentratedMapping::optimizationInfoHandler, this, std::placeholders::_1);
        auto sub_optimization_request_options = rclcpp::SubscriptionOptions();
        sub_optimization_request_options.topic_stats_options.state = rclcpp::TopicStatisticsState::Disable;
        sub_optimization_request_options.topic_stats_options.publish_period = std::chrono::seconds(1);
        sub_optimization_request_options.topic_stats_options.publish_topic = "/statistics";
        auto callback_group_sub_1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        sub_optimization_request_options.callback_group = callback_group_sub_1;
        sub_optimization_request = this->create_subscription<co_lrio::msg::OptimizationRequest>(
			"/co_lrio/optimization_request", qos_reliable, optimization_request_callback, sub_optimization_request_options);

        // publisher
        pub_loop_closure = this->create_publisher<co_lrio::msg::LoopClosure>(
            "/co_lrio/loop_closure", qos_reliable);
        pub_loop_closure_constraints = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/co_lrio/loop_closure_constraints", 1);

        // service
        std::function<void(std::shared_ptr<co_lrio::srv::SaveFiles::Request>, std::shared_ptr<co_lrio::srv::SaveFiles::Response>)> service_func = 
            std::bind(&ConcentratedMapping::saveTrajectoriesService, this, std::placeholders::_1, std::placeholders::_2);
        save_trajectories_service = this->create_service<co_lrio::srv::SaveFiles>("/save_files", service_func);

        // tf2
        world_2_odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        /* database */
        map_database = std::unique_ptr<MapDatabase>(new MapDatabase(params.parallel_cpu_core_, params.global_near_map_));

        // robotic swarm information
        pub_map_for_robots.clear();
        prefix.clear();
        global_path.clear();
        sub_imu_odometry.clear();
        pub_global_path.clear();
        pub_global_map.clear();
        optimization_response_msg.clear();
        pairwise_distance.clear();
        pub_optimization_response.clear();
        
        /* loop closure */
        inter_robot_loop_ptr = 0;
        loop_indexes.clear();
        loop_indexes_copy.clear();
        update_path_flag = false;

        if (params.descriptor_type_ == DescriptorType::ScanContext)
        {
            scan_descriptor = unique_ptr<ScanDescriptor>(new ScanContextDescriptor(
                20, 60, params.candidates_num_, params.distance_threshold_, params.max_radius_,
                params.exclude_recent_num_, params.save_directory_));
        }
        else if (params.descriptor_type_ == DescriptorType::LidarIris)
        {
            scan_descriptor = unique_ptr<ScanDescriptor>(new lidarIrisDescriptor(
                80, 360, params.n_scan_, params.distance_threshold_, params.exclude_recent_num_,
                2, params.candidates_num_, 4, 18, 1.6f, 0.75f, params.save_directory_));
        }
        else if (params.descriptor_type_ == DescriptorType::MultiSector)
        {
            // New MultiSectorDescriptor signature:
            // MultiSectorDescriptor(int sector_num, float max_range, int min_points,
            //                       int candidates_num, float distance_threshold,
            //                       int exclude_recent, std::string directory)
            scan_descriptor = unique_ptr<ScanDescriptor>(new MultiSectorDescriptor(
                20,                          // ring_num
                24,                          // sector_num
                params.candidates_num_,      // candidates_num
                params.distance_threshold_,  // distance_threshold
                params.max_radius_,          // max_radius
                params.exclude_recent_num_,  // exclude_recent_num
                params.save_directory_       // directory
            ));
        }

        /* global robust optimizer */
        optimizer = unique_ptr<RobustOptimizer>(new RobustOptimizer(true, params.enable_risam_, SolverType::GaussNewton,
            ThresholdType::Cost, 0.8, params.loop_num_threshold_, params.use_pcm_, params.pcm_threshold_,
            params.save_directory_, params.enable_ranging_, params.minimum_ranging_,
            params.enable_ranging_outlier_threshold_, params.ranging_outlier_threshold_,
            params.gps_cov_threshold_, params.use_gps_elevation_,
            params.enable_ground_plane_, params.ground_plane_z_,
            params.ground_plane_noise_z_, params.ground_plane_noise_roll_,
            params.ground_plane_noise_pitch_, params.ground_plane_factor_interval_));

        auto register_manual_pose = [this](RobustOptimizer* optimizer_ptr, const int8_t robot_id, const std::vector<double>& pose_vec)
        {
            if (pose_vec.empty())
            {
                return;
            }
            if (pose_vec.size() != 6)
            {
                RCLCPP_WARN(
                    rclcpp::get_logger(""),
                    "manual_init_robot_%d must have 6 values [x, y, z, roll, pitch, yaw], got %zu.",
                    robot_id,
                    pose_vec.size());
                return;
            }

            gtsam::Pose3 pose(
                gtsam::Rot3::RzRyRx(pose_vec[3], pose_vec[4], pose_vec[5]),
                gtsam::Point3(pose_vec[0], pose_vec[1], pose_vec[2]));
            optimizer_ptr->setManualInitialPose(robot_id, pose);
            RCLCPP_INFO(
                rclcpp::get_logger("optimization_log_mini"),
                "\033[0;36mregister manual prior for robot %d: [%.3f %.3f %.3f %.3f %.3f %.3f]\033[0m",
                robot_id,
                pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3], pose_vec[4], pose_vec[5]);
        };
        register_manual_pose(optimizer.get(), 0, manual_init_robot_0);
        register_manual_pose(optimizer.get(), 1, manual_init_robot_1);
        register_manual_pose(optimizer.get(), 2, manual_init_robot_2);

        // global map visualization
        downsample_filter_for_global_map.setLeafSize(params.map_leaf_size_, params.map_leaf_size_, params.map_leaf_size_);
        kdtree_history_keyposes.reset(new pcl::KdTreeFLANN<PointPose3D>());

        system_monitor = std::unique_ptr<systemMonitor>(new systemMonitor(this, 10));
    }

    ~ConcentratedMapping()
    {

    }

    // owner gate on the verified-loop timeline (optimizationInfoHandler): handles
    // release + streak reset, driven by the poses of loops that actually passed
    // verification — these arrive ordered along the trajectory, so the state stays
    // consistent regardless of verification-queue latency.
    // returns true while the category is suppressed at query_pose.
    bool loopThrottled(
        const bool& is_intra,
        const int8_t& robot,
        const gtsam::Pose3& query_pose)
    {
        std::lock_guard<std::mutex> lock(lock_on_loop_throttle);

        auto& streak = is_intra ? intra_loop_streak : inter_loop_streak;
        auto& anchor = is_intra ? intra_loop_anchor : inter_loop_anchor;
        auto& suppressed = is_intra ? intra_loop_suppressed : inter_loop_suppressed;

        if (!suppressed[robot])
        {
            return false;
        }

        if (anchor[robot].range(query_pose) >= params.last_loop_dist_)
        {
            suppressed[robot] = false;
            streak[robot] = 0;
            RCLCPP_INFO(rclcpp::get_logger("factor_log"),
                "[LoopThrottle] robot %d %s loops re-enabled (moved %.0fm from anchor)",
                (int)robot, is_intra ? "intra" : "inter", (double)params.last_loop_dist_);
            return false;
        }

        return true;
    }

    // read-only check used by the detection side (performInterLoopClosure) to
    // skip sending candidates while their category is suppressed near the anchor.
    // Deliberately mutates no state — the detection frontier runs ahead of the
    // verified-loop timeline and must neither release the suppression nor
    // advance the streak.
    bool loopSuppressed(
        const bool& is_intra,
        const int8_t& robot,
        const gtsam::Pose3& loop_pose)
    {
        std::lock_guard<std::mutex> lock(lock_on_loop_throttle);

        const auto& anchor = is_intra ? intra_loop_anchor : inter_loop_anchor;
        auto& suppressed = is_intra ? intra_loop_suppressed : inter_loop_suppressed;

        if (!suppressed[robot] || !anchor.count(robot))
        {
            return false;
        }

        return anchor.at(robot).range(loop_pose) < params.last_loop_dist_;
    }

    // counts an accepted loop of the given category; a spatial gap larger than
    // last_loop_dist between consecutive loops breaks the streak; reaching
    // loop_streak_threshold turns the suppression on
    void countLoop(
        const bool& is_intra,
        const int8_t& robot,
        const gtsam::Pose3& loop_pose)
    {
        std::lock_guard<std::mutex> lock(lock_on_loop_throttle);

        auto& streak = is_intra ? intra_loop_streak : inter_loop_streak;
        auto& anchor = is_intra ? intra_loop_anchor : inter_loop_anchor;
        auto& suppressed = is_intra ? intra_loop_suppressed : inter_loop_suppressed;

        if (streak[robot] > 0 && anchor[robot].range(loop_pose) > params.last_loop_dist_)
        {
            streak[robot] = 0;
        }

        streak[robot]++;
        anchor[robot] = loop_pose;

        if (streak[robot] >= params.loop_streak_threshold_)
        {
            suppressed[robot] = true;
            RCLCPP_INFO(rclcpp::get_logger("factor_log"),
                "[LoopThrottle] robot %d %s loops reached %d, suppressed within %.0fm",
                (int)robot, is_intra ? "intra" : "inter", streak[robot],
                (double)params.last_loop_dist_);
        }
    }

    void performInterLoopClosure()  //执行一次回环检测
    {
        // early return
        if (inter_robot_loop_ptr >= scan_descriptor->getSize())     //越界保护  scan_descriptor->getSize()为描述子的索引大小
        {
            return;
        }

        // periodic status log — every 100 frames print descriptor pool size
        if (inter_robot_loop_ptr % 100 == 0)
        {
            // RCLCPP_INFO(rclcpp::get_logger("loop_log"),
            //     "[LoopDetect] ptr=%d  descriptor_pool=%d",
            //     inter_robot_loop_ptr, scan_descriptor->getSize());
        }

        // detect inter-robot and intra-robot loop closure
        auto robot_key = scan_descriptor->getIndex(inter_robot_loop_ptr);
        auto t_detect_s = std::chrono::high_resolution_clock::now();
        auto candidates = scan_descriptor->detectLoopClosure(robot_key.first, robot_key.second);
        auto ms_detect = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - t_detect_s).count() / 1e3f;
        RCLCPP_DEBUG(rclcpp::get_logger("loop_log"), "performInterLoopClosure %d/%d", inter_robot_loop_ptr, scan_descriptor->getSize());
        // RCLCPP_WARN(rclcpp::get_logger("loop_log_mini"),
        //     "[Timing][LoopDetect] query=[%d-%d] candidates=%zu detect=%.1fms enable_loop=%d",
        //     robot_key.first, robot_key.second, candidates.size(), ms_detect, params.enable_loop_ ? 1 : 0);
        (void)ms_detect;
        inter_robot_loop_ptr++;

        // no loop closure candidate
        if (candidates.empty())
        {
            return;
        }

        for (const auto& candidate : candidates)
        {
            if (!params.enable_loop_ && robot_key.first!=get<0>(candidate)) // 如果外回环检测被禁用，并且候选项是异机器人回环，则跳过该候选项
            {
                // RCLCPP_INFO(rclcpp::get_logger("loop_log_mini"),
                //     "[LoopDetect] skip inter-robot candidate [%d-%d]->[%d-%d], enable_loop=false",
                //     robot_key.first, robot_key.second, get<0>(candidate), get<1>(candidate));
                continue;
            }

            // distance pre-filter for intra-robot candidates: both keys live in the
            // same robot's (backend-optimized) frame, so reject before the candidate
            // is ever sent to the front-end queue instead of after GICP dequeues it
            if (params.loop_closure_max_pose_distance_ > 0.0f && robot_key.first == get<0>(candidate))
            {
                const auto p0 = map_database->poseAt(robot_key.first, robot_key.second).translation();
                const auto p1 = map_database->poseAt(get<0>(candidate), get<1>(candidate)).translation();
                const double dx = p0.x()-p1.x(), dy = p0.y()-p1.y(), dz = p0.z()-p1.z();
                const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

                if (dist > params.loop_closure_max_pose_distance_)
                {
                    continue;
                }
            }

            // distance pre-filter for inter-robot candidates
            if (params.inter_robot_max_distance_ > 0.0f && robot_key.first != get<0>(candidate))
            {
                const auto r0 = robot_key.first;
                const auto k0 = robot_key.second;
                const auto r1 = get<0>(candidate);
                const auto k1 = get<1>(candidate);

                if (trans_to_pub.count(r0) && trans_to_pub.count(r1))
                {
                    const auto p0 = trans_to_pub.at(r0).compose(map_database->poseAt(r0, k0)).translation();
                    const auto p1 = trans_to_pub.at(r1).compose(map_database->poseAt(r1, k1)).translation();
                    const double dx = p0.x()-p1.x(), dy = p0.y()-p1.y(), dz = p0.z()-p1.z();
                    const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

                    if (dist > params.inter_robot_max_distance_)
                    {
                        // RCLCPP_INFO(rclcpp::get_logger("loop_log_mini"),
                        //     "[LoopFilter] skip inter-robot [%d-%d]->[%d-%d] world_dist=%.1fm > %.0fm",
                        //     r0, k0, r1, k1, dist, (double)params.inter_robot_max_distance_);
                        continue;
                    }
                }
            }

            // loop throttling (read-only here): skip candidates of a category that
            // is currently suppressed near its anchor. All state transitions live
            // on the verified-loop timeline in optimizationInfoHandler — counting
            // candidate sends would let failed verifications (e.g. low-overlap
            // startup candidates) consume the whole streak budget without a single
            // loop ever entering the graph
            const bool candidate_is_intra = (robot_key.first == get<0>(candidate));
            if (loopSuppressed(candidate_is_intra, robot_key.first,
                    map_database->poseAt(robot_key.first, robot_key.second)))
            {
                continue;
            }

            // verify loop closure candidate
            sendLoopClosureStageOne(robot_key.first, robot_key.second, get<0>(candidate), get<1>(candidate), get<2>(candidate));
            // RCLCPP_INFO(rclcpp::get_logger("loop_log"), "\033[1;33mLoop:[%d-%d] [%d-%d] initYaw:%d.\033[0m",
            //     robot_key.first, robot_key.second, get<0>(candidate), get<1>(candidate), get<2>(candidate));
        }
    }

    void sendLoopClosureStageOne( 
        const int8_t& robot0,
        const int& key0,
        const int8_t& robot1,
        const int& key1,
        const int& yaw_diff)
    {
        auto loop_closure_msg = std::make_unique<co_lrio::msg::LoopClosure>();

        loop_closure_msg->robot0 = robot0;  
        loop_closure_msg->key0 = key0;
        loop_closure_msg->robot1 = robot1;
        loop_closure_msg->key1 = key1;
        loop_closure_msg->yaw_diff = yaw_diff;    //yaw_diff为两个关键帧之间的yaw差值
        loop_closure_msg->noise = 999.0;

        static rclcpp::SerializedMessage serialized_msg;
        static rclcpp::Serialization<co_lrio::msg::LoopClosure> serializer;
        serializer.serialize_message(loop_closure_msg.get(), &serialized_msg);
        pub_loop_closure->publish(serialized_msg);
    }

    void loopClosureDetectionThread()
    {
        rclcpp::Rate rate(1.0/params.loop_detection_interval_);

        while (rclcpp::ok())
        {
            performInterLoopClosure();

            rate.sleep();
        }
    }

    void allocateMemoryAndInitialization(    //给某个机器人初始化/准备好后续要用的各种容器和发布对象
        const int8_t& input_robot)
    {
        // robot prefix
        string prefix_base = "robot_";
        prefix_base += to_string(int(input_robot));
        prefix.emplace(input_robot, prefix_base);

        // subscriber
        std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> imu_odometry_callback = 
            std::bind(&ConcentratedMapping::imuOdometryHandler, this, std::placeholders::_1, input_robot);
        auto sub_imu_odometry_tmp = this->create_subscription<nav_msgs::msg::Odometry>(
            prefix.at(input_robot) + "/co_lrio/imu_odometry", qos_best_effort, imu_odometry_callback);
        sub_imu_odometry.emplace(input_robot, sub_imu_odometry_tmp);

        // publisher
        auto pub_global_path_tmp = this->create_publisher<nav_msgs::msg::Path>(
            prefix.at(input_robot) + "/co_lrio/global_path", 1);
        pub_global_path.emplace(input_robot, pub_global_path_tmp);
        auto pub_global_map_tmp = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            prefix.at(input_robot) + "/co_lrio/global_map", 1);
        pub_global_map.emplace(input_robot, pub_global_map_tmp);
        auto pub_global_near_map_tmp = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            prefix.at(input_robot) + "/co_lrio/global_near_map", 1);
        pub_global_near_map.emplace(input_robot, pub_global_near_map_tmp);
        auto pub_optimization_response_tmp = this->create_publisher<co_lrio::msg::OptimizationResponse>(
            prefix.at(input_robot) + "/co_lrio/optimization_response", qos_reliable);
        pub_optimization_response.emplace(input_robot, pub_optimization_response_tmp);

        // global path
        nav_msgs::msg::Path global_path_base;
        global_path.emplace(make_pair(input_robot, global_path_base));

        // relative transformation
        auto relative_trans = gtsam::Pose3();
        trans_to_pub.emplace(make_pair(input_robot, relative_trans));
        trans.emplace(make_pair(input_robot, relative_trans));

        // optimization response
        co_lrio::msg::OptimizationResponse optimization_response_msg_base;
        optimization_response_msg.emplace(input_robot, optimization_response_msg_base);
    }

    void updateGlobalPath(
        const int8_t& robot_id,
        const gtsam::Pose3& pose_in)
    {
        static geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = pose_in.translation().x();
        pose_stamped.pose.position.y = pose_in.translation().y();
        pose_stamped.pose.position.z = pose_in.translation().z();
        pose_stamped.pose.orientation.x = pose_in.rotation().toQuaternion().x();
        pose_stamped.pose.orientation.y = pose_in.rotation().toQuaternion().y();
        pose_stamped.pose.orientation.z = pose_in.rotation().toQuaternion().z();
        pose_stamped.pose.orientation.w = pose_in.rotation().toQuaternion().w();
        global_path.at(robot_id).poses.push_back(pose_stamped);
    }

    void publishOptimizationResponse(   //把一次优化结果封装成 OptimizationResponse 消息并发布给对应机器人。
        const int8_t& robot,
        const gtsam::Symbol& symbol,
        const co_lrio::msg::OptimizationRequest& msg,
        const gtsam::Pose3& pose,
        const bool& flag) 
    {
        auto optimization_response_msg_ptr = std::make_unique<co_lrio::msg::OptimizationResponse>();

        auto optimized_keypose_msg = gtsamPoseToOdometryMsg(pose);
        optimized_keypose_msg.header.stamp = msg.header.stamp;
        optimized_keypose_msg.header.frame_id = prefix.at(robot) + params.odometry_frame_;
        optimized_keypose_msg.child_frame_id = prefix.at(robot) + "/odom_mapping";
        optimization_response_msg.at(robot).robot_id = robot;
        optimization_response_msg.at(robot).index_to = symbol;
        optimization_response_msg.at(robot).pose_to = optimized_keypose_msg;
        optimization_response_msg.at(robot).update_keyposes = flag;
        *optimization_response_msg_ptr = optimization_response_msg.at(robot);   

        static rclcpp::SerializedMessage serialized_msg;
        static rclcpp::Serialization<co_lrio::msg::OptimizationResponse> serializer;
        serializer.serialize_message(optimization_response_msg_ptr.get(), &serialized_msg);
        pub_optimization_response.at(robot)->publish(serialized_msg);
    }

    void correctPoses(
        const int& robot_id,
        const double& timestamp,
        const gtsam::Values& input_optimized_keyposes)
    {
        bool update_flag = find(optimizer->getConnectedRobot().begin(),optimizer->getConnectedRobot().end(),robot_id) != optimizer->getConnectedRobot().end();
        // clear path  
        //判断这次优化是否涉及到"连通的机器人集合"。如果是，后面会重建这些机器人的 global_path。
        if (update_flag)   
        {
            for (const auto& robot : optimizer->getConnectedRobot())
            {
                global_path.at(robot).poses.clear();   //清空每个相关机器人的global_path

                auto prior_symbol = gtsam::Symbol(robot + 'a', 0);
                if (input_optimized_keyposes.exists(prior_symbol))    // prior_symbol（每个机器人第 0 帧的先验）更新 trans_to_pub 和 trans。trans_to_pub 是机器人相对于 world 的位姿，trans 是机器人相对于前一个机器人（world）的位姿。
                {
                    auto relative_trans = gtsam::Pose3().between(input_optimized_keyposes.at<gtsam::Pose3>(prior_symbol));
                    trans_to_pub.at(robot) = relative_trans;  
                    relative_trans = input_optimized_keyposes.at<gtsam::Pose3>(prior_symbol).between(gtsam::Pose3());
                    trans.at(robot) = relative_trans;
                }
            }
        }

        auto t_correct_s = std::chrono::high_resolution_clock::now();

        // update poses  遍历所有优化后的变量，把位姿写回地图数据库
        pcl::PointCloud<PointPose6D>::Ptr keyposes_to_update(new pcl::PointCloud<PointPose6D>());
        for (const auto& key_value : input_optimized_keyposes)
        {
            auto symbol = (gtsam::Symbol)key_value.key;
            if (symbol.chr() < 'a') // ignore imu pose and non-Pose3 variables (e.g. ground plane 'G')
            {
                continue;
            }
            auto pose = input_optimized_keyposes.at<gtsam::Pose3>(symbol);
            auto robot = symbol.chr() - 'a';
            auto index = (int)symbol.index();

            map_database->updatePose(robot, index, pose);

            // update global path  如果这次优化涉及到连通的机器人集合，则更新它们的 global_path。
            if (update_flag)
            {
                updateGlobalPath(robot, pose);
            }

            //为当前机器人打包一份"优化后关键帧位姿集合（点云形式）"放进响应消息
            if (robot == robot_id)
            {
                static PointPose6D p;
                auto local_pose = trans.at(robot).compose(pose);
                p.x = local_pose.translation().x();
                p.y = local_pose.translation().y();
                p.z = local_pose.translation().z();
                p.roll = local_pose.rotation().roll();
                p.pitch = local_pose.rotation().pitch();
                p.yaw  = local_pose.rotation().yaw();
                p.intensity = symbol.index();
                keyposes_to_update->push_back(p);  //把优化后的位姿添加到 keyposes_to_update 中，后续会发布给机器人。
            }
        }

        auto t_update_end = std::chrono::high_resolution_clock::now();

        // publish path 如果这次优化涉及到连通的机器人集合，则发布它们的 global_path。
        if (update_flag)
        {
            for (const auto& robot : optimizer->getConnectedRobot())   //遍历所有连通的机器人，发布它们的 global_path。
            {
                // publish path
                if (pub_global_path.at(robot)->get_subscription_count() != 0 && !global_path.at(robot).poses.empty())
                {
                    global_path.at(robot).header.stamp = this->get_clock()->now();    //用当前节点的ROS2时钟
                    global_path.at(robot).header.frame_id = params.world_frame_;
                    pub_global_path.at(robot)->publish(global_path.at(robot));
                }
            }
        }

        auto t_publish_end = std::chrono::high_resolution_clock::now();
        RCLCPP_WARN(rclcpp::get_logger("optimization_log"),
            "[Timing][correctPoses] robot=%d  poses=%d  db_update=%.1fms  path_pub=%.1fms",
            robot_id, (int)input_optimized_keyposes.size(),
            float(std::chrono::duration_cast<std::chrono::microseconds>(t_update_end - t_correct_s).count())/1e3,
            float(std::chrono::duration_cast<std::chrono::microseconds>(t_publish_end - t_update_end).count())/1e3);

        // buffer  把优化后的位姿封装成 keyposes 消息
        pcl::toROSMsg(*keyposes_to_update, optimization_response_msg.at(robot_id).keyposes);
    }

    void optimizationInfoHandler(const std::shared_ptr<rclcpp::SerializedMessage> serialized_msg)
    {
        lock_guard<std::mutex> lock(lock_on_optmizer);

        static co_lrio::msg::OptimizationRequest msg;
        static auto serializer = rclcpp::Serialization<co_lrio::msg::OptimizationRequest>();
        serializer.deserialize_message(serialized_msg.get(), &msg);

        auto robot_id = msg.robot_id;
        if (prefix.find(robot_id) == prefix.end())  //如果机器人每出现过，创建容器并初始化相关信息
        {
            allocateMemoryAndInitialization(robot_id);
            optimizer->addRobot(robot_id); //将机器人加入因子图 
        }
        
        system_monitor->addReceviedMsg(serialized_msg->size(), true);

        auto t0 = chrono::high_resolution_clock::now();
        SwarmFrame sf(msg, imu_odometry_queue, trans_to_pub); // 把优化请求消息、IMU里程计队列和相对变换传给 SwarmFrame，后者会根据这些信息构建出一个"包含当前优化所需全部信息的对象 sf"。
        auto t1 = chrono::high_resolution_clock::now();

        // loop throttling: verified loop frames of a suppressed category are dropped
        // before entering the factor graph; accepted ones update the streak counter.
        // frame classification mirrors saveFactor: same robot + non-consecutive
        // indices (excluding the 0/0 prior) = intra loop; different robots = inter loop
        {
            const bool same_robot = (sf.index_from.chr() == sf.index_to.chr());
            const bool is_intra_loop = same_robot
                && !(sf.index_to.index() == 0 && sf.index_from.index() == 0)
                && (sf.index_to.index() != sf.index_from.index() + 1);
            const bool is_inter_loop = !same_robot;

            if (is_intra_loop || is_inter_loop)
            {
                const int8_t loop_robot = sf.index_from.chr() - 'a';
                const auto loop_pose = map_database->poseAt(loop_robot, sf.index_from.index());

                // the throttle state machine lives entirely on this verified-loop
                // timeline: arrivals are ordered along the trajectory, so counting,
                // suppression and release stay consistent regardless of how long
                // the front-end verification queue delays them. Failed verifications
                // never reach this point and therefore never consume the budget.
                if (loopThrottled(is_intra_loop, loop_robot, loop_pose))
                {
                    RCLCPP_INFO(rclcpp::get_logger("factor_log"),
                        "[LoopThrottle] drop %s loop %c%d -> %c%d (suppressed)",
                        is_intra_loop ? "intra" : "inter",
                        sf.index_from.chr(), (int)sf.index_from.index(),
                        sf.index_to.chr(), (int)sf.index_to.index());
                    return;
                }

                countLoop(is_intra_loop, loop_robot, loop_pose);
            }
        }

        auto start_optimize_time = chrono::high_resolution_clock::now();
        optimizer->optimize(sf, loop_indexes);
        auto end_optimize_time = chrono::high_resolution_clock::now();
        auto optimize_duration = chrono::duration_cast<chrono::microseconds>(end_optimize_time - start_optimize_time).count();
        system_monitor->addOptimizeTime(float(optimize_duration)/1e3);
        auto t2 = chrono::high_resolution_clock::now();

        if (sf.do_optimize)
        {
            // RCLCPP_INFO(rclcpp::get_logger("optimization_log"), "\033[0;36mcorrectPoses\033[0m");
            correctPoses(robot_id, sf.timestamp, optimizer->getLatestValues(robot_id));  //修正位姿
        }
        auto t3 = chrono::high_resolution_clock::now();

        // RCLCPP_INFO(rclcpp::get_logger("loop_log"),
        //     "[DescSave] robot=%d key=%d isOdom=%d desc_size=%zu pool=%d",
        //     (int)sf.robot_id, sf.robot_key, (int)sf.isOdom,
        //     sf.descriptor.size(), scan_descriptor->getSize());
        if (sf.isOdom)
        {
            // save map and pose
            map_database->savePoseAndMap(sf);
            // make global descriptor
            scan_descriptor->saveDescriptorAndKey(sf.descriptor, sf.robot_id, sf.robot_key);
            auto t4 = chrono::high_resolution_clock::now();
            auto ms_swarm  = chrono::duration_cast<chrono::microseconds>(t1 - t0).count() / 1e3f;
            auto ms_optim  = chrono::duration_cast<chrono::microseconds>(t2 - t1).count() / 1e3f;
            auto ms_correct= chrono::duration_cast<chrono::microseconds>(t3 - t2).count() / 1e3f;
            auto ms_save   = chrono::duration_cast<chrono::microseconds>(t4 - t3).count() / 1e3f;
            auto ms_total  = chrono::duration_cast<chrono::microseconds>(t4 - t0).count() / 1e3f;
            // RCLCPP_WARN(rclcpp::get_logger("optimization_log"),
            //     "[Timing] key=%d  swarm=%.1fms  optim=%.1fms  correct=%.1fms  save=%.1fms  TOTAL=%.1fms",
            //     sf.robot_key, ms_swarm, ms_optim, ms_correct, ms_save, ms_total);
            (void)ms_swarm; (void)ms_optim; (void)ms_correct; (void)ms_save; (void)ms_total;
            // check distance for loop
            if (!sf.other_ids.empty() && params.enable_loop_ && params.enable_ranging_)
            {
                // add distance constraint
                std::unordered_map<int, pair<gtsam::Symbol, float>> new_pairwise_distance;
                for (auto i = 0; i < sf.other_ids.size(); i++)
                {
                    // store distance
                    new_pairwise_distance.emplace(sf.other_ids[i], make_pair(sf.index_to, sf.distances[i]));

                    // find pairwise distance
                    auto other_id = sf.other_ids[i];

                    double dis = sf.distances[i];

                    if (pairwise_distance.find(other_id) == pairwise_distance.end() ||
                        pairwise_distance.at(other_id).find(sf.robot_id) == pairwise_distance.at(other_id).end())
                    {
                        // RCLCPP_INFO(rclcpp::get_logger("optimization_log"), "\033[0;36mdon't find pairwise_distance!\033[0m");
                        continue;
                    }

                    auto symbol_other = gtsam::Symbol(pairwise_distance.at(other_id).at(sf.robot_id).first);
                
                    if (dis < params.uwb_distance_thereshold_for_loop_)
                    {
                        sendLoopClosureStageOne(sf.robot_id, sf.index_to.index(), symbol_other.chr()-'a', symbol_other.index(), 0);
                    }
                }

                if (pairwise_distance.find(sf.robot_id) == pairwise_distance.end())
                {
                    pairwise_distance.emplace(make_pair(sf.robot_id, new_pairwise_distance));
                }
                else
                {
                    pairwise_distance.at(sf.robot_id) = new_pairwise_distance;
                }
            }
            
            // response message
            // RCLCPP_INFO(rclcpp::get_logger("optimization_log"), "\033[0;36m<optimization> publish Response (Key %d) %d\033[0m", sf.robot_id, sf.robot_key);
            publishOptimizationResponse(robot_id, sf.index_to, msg, trans.at(robot_id).compose(optimizer->getLatestValue(robot_id)), sf.update_keyposes);
        }
        else
        {
            for (const auto& robot : optimizer->getConnectedRobot())
            {
                pub_map_for_robots.emplace_back(robot);
            }
        }
    }

    void imuOdometryHandler(
        const std::shared_ptr<rclcpp::SerializedMessage> serialized_msg,
        const int8_t robot_id)
    {
        lock_guard<std::mutex> lock(lock_on_imu);

        static nav_msgs::msg::Odometry msg;
        static auto serializer = rclcpp::Serialization<nav_msgs::msg::Odometry>();
        serializer.deserialize_message(serialized_msg.get(), &msg);

        if(imu_odometry_queue.find(robot_id) == imu_odometry_queue.end())
        {
            deque<nav_msgs::msg::Odometry> odom_queue;
            odom_queue.emplace_back(msg);
            imu_odometry_queue.emplace(make_pair(robot_id, odom_queue));
        }
        else
        {
            imu_odometry_queue.at(robot_id).emplace_back(msg);
        }

        system_monitor->addReceviedMsg(serialized_msg->size());
    }

    // void updateNearMapThread()
    // {
    //     if (!params.use_global_near_map_)
    //     {
    //         return;
    //     }
        
    //     rclcpp::Rate rate(10);

    //     while (rclcpp::ok())
    //     {
    //         rate.sleep();

    //         for (const auto& robot : pub_map_for_robots)
    //         {
    //             // get near map
    //             // auto global_near_map = map_database->updateAndGetNearGlobalMap(robot, optimizer->getConnectedRobot());

    //             // publish near map
    //             sensor_msgs::msg::PointCloud2 global_near_map_msg;
    //             pcl::toROSMsg(*global_near_map.first, global_near_map_msg);
    //             global_near_map_msg.header.stamp = this->get_clock()->now();
    //             global_near_map_msg.header.frame_id = params.world_frame_;
    //             pub_global_near_map.at(robot)->publish(global_near_map_msg);

    //             pub_map_for_robots.pop_front();
    //         }
    //     }
    // }

    void publishLoopClosureConstraint()
    {
        if (loop_indexes_copy.empty())
        {
            return;
        }

        // loop nodes   配置可视化的参数
        int index = 1;
        visualization_msgs::msg::Marker nodes;
        nodes.header.frame_id = "world";
        nodes.header.stamp = this->get_clock()->now();
        nodes.action = visualization_msgs::msg::Marker::ADD;
        nodes.type = visualization_msgs::msg::Marker::SPHERE;
        nodes.ns = "loop_nodes";
        nodes.id = 0;
        nodes.lifetime = rclcpp::Duration(0, 0);
        nodes.pose.position.x = 0.0f;
        nodes.pose.position.y = 0.0f;
        nodes.pose.position.z = 0.0f;
        nodes.pose.orientation.w = 1.0f;
        nodes.pose.orientation.x = 0.0f;
        nodes.pose.orientation.y = 0.0f;
        nodes.pose.orientation.z = 0.0f;
        nodes.scale.x = 0.3f; nodes.scale.y = 0.3f; nodes.scale.z = 0.3f; 
        nodes.color.r = 0.9f; nodes.color.g = 0.9f; nodes.color.b = 0.3f;
        nodes.color.a = 1.0f;
        nodes.frame_locked = true;

        // loop edges  
        visualization_msgs::msg::Marker constraints;
        constraints.header.frame_id = "world";
        constraints.header.stamp = this->get_clock()->now();
        constraints.action = visualization_msgs::msg::Marker::ADD;
        constraints.type = visualization_msgs::msg::Marker::LINE_LIST;
        constraints.lifetime = rclcpp::Duration(0, 0);
        constraints.ns = "loop_constraints";
        constraints.id = 1;
        constraints.pose.position.x = 0.0f;
        constraints.pose.position.y = 0.0f;
        constraints.pose.position.z = 0.0f;
        constraints.pose.orientation.w = 1.0f;
        constraints.pose.orientation.x = 0.0f;
        constraints.pose.orientation.y = 0.0f;
        constraints.pose.orientation.z = 0.0f;
        constraints.scale.x = 0.2f;
        constraints.color.r = 0.9f; constraints.color.g = 0.9f; constraints.color.b = 0.3f;
        constraints.color.a = 1.0f;
        constraints.frame_locked = true;

        // iterate on all accepted loop closures
        for (const auto& loop : loop_indexes_copy)
        {
            auto index0 = loop.first;
            auto index1 = loop.second;

            if (!global_optimized_keyposes_copy.exists(index0) || !global_optimized_keyposes_copy.exists(index1))
            {
                continue;
            }

            geometry_msgs::msg::Point p;
            auto pose0 = global_optimized_keyposes_copy.at<gtsam::Pose3>(index0);
            p.x = pose0.translation().x();
            p.y = pose0.translation().y();
            p.z = pose0.translation().z();
            nodes.points.push_back(p);
            constraints.points.push_back(p);
            auto pose1 = global_optimized_keyposes_copy.at<gtsam::Pose3>(index1);
            p.x = pose1.translation().x();
            p.y = pose1.translation().y();
            p.z = pose1.translation().z();
            nodes.points.push_back(p);
            constraints.points.push_back(p);
        }

        // publish loop closure markers
        visualization_msgs::msg::MarkerArray markers_array;
        markers_array.markers.push_back(nodes);
        markers_array.markers.push_back(constraints);
        pub_loop_closure_constraints->publish(markers_array);
    }

    void publishGlobalMap()
    {
        // extract trajectory of each robot
        map<int8_t, pcl::PointCloud<PointPose3D>::Ptr> poses_3d_clouds;
        for (const auto& key_value : global_optimized_keyposes_copy)
        {
            auto symbol = (gtsam::Symbol)key_value.key;
            if (symbol.chr() < 'a') // skip imu pose and non-Pose3 variables (e.g. ground plane 'G')
            {
                continue;
            }
            auto pose = global_optimized_keyposes_copy.at<gtsam::Pose3>(symbol);
            auto index = symbol.index();
            auto robot = symbol.chr() - 'a';

            PointPose3D pose_3d;
            pose_3d.x = pose.translation().x();
            pose_3d.y = pose.translation().y();
            pose_3d.z = pose.translation().z();
            pose_3d.intensity = symbol.index();
            if (poses_3d_clouds.find(robot) == poses_3d_clouds.end())
            {
                pcl::PointCloud<PointPose3D>::Ptr new_poses_3d_cloud(new pcl::PointCloud<PointPose3D>());
                new_poses_3d_cloud->push_back(pose_3d);
                poses_3d_clouds.emplace(make_pair(robot, new_poses_3d_cloud));
            }
            else
            {
                poses_3d_clouds.at(robot)->push_back(pose_3d);
            }
        }

        // global map of each robot
        for (const auto& robot : optimizer->getConnectedRobot())
        {
            // early return
            if (pub_global_map.at(robot)->get_subscription_count() == 0)
            {
                continue;
            }

            // find the closest history key frame
            vector<int> indices;
            vector<float> distances;
            kdtree_history_keyposes->setInputCloud(poses_3d_clouds.at(robot));
            kdtree_history_keyposes->radiusSearch(poses_3d_clouds.at(robot)->back(),
                params.global_map_visualization_radius_, indices, distances, 0);

            // extract visualized key frames
            pcl::PointCloud<PointPose3D>::Ptr global_map_keyframes(new pcl::PointCloud<PointPose3D>());
            pcl::PointCloud<PointPose3D>::Ptr global_map_keyframes_ds(new pcl::PointCloud<PointPose3D>());
            for (const auto& indice : indices)
            {
                auto index = poses_3d_clouds.at(robot)->points[indice].intensity;
                auto symbol = gtsam::Symbol('a' + robot, index);
                *global_map_keyframes += *map_database->cloudAt(robot, index, 0);
            }

            // downsample visualized points
            downsample_filter_for_global_map.setInputCloud(global_map_keyframes);
            downsample_filter_for_global_map.filter(*global_map_keyframes_ds);

            // publish global map
            sensor_msgs::msg::PointCloud2 global_map_msg;
            pcl::toROSMsg(*global_map_keyframes_ds, global_map_msg);
            global_map_msg.header.stamp = this->get_clock()->now();
            global_map_msg.header.frame_id = params.world_frame_;
            pub_global_map.at(robot)->publish(global_map_msg);
        }
    }

    void visualizationThread()
    {
        rclcpp::Rate rate(1.0/params.pub_global_map_interval_); // update global map per 60s

        while (true)
        {
            rate.sleep();

            lock_on_optmizer.lock();
            loop_indexes_copy = loop_indexes;
            global_optimized_keyposes_copy = optimizer->getLatestValues();
            lock_on_optmizer.unlock();

            // loop closure visualization
            publishLoopClosureConstraint();

            // global map visualization
            publishGlobalMap();
        }
    }

    void publishTfThread()
    {
        if (params.simulator_mode_)
        {
            return;
        }

        rclcpp::Rate rate(1.0/params.pub_tf_interval_);

        while (true)
        {
            rate.sleep();

            for (const auto& trans : trans_to_pub)
            {
                tf2::Quaternion quat_tf;
                tf2::Stamped<tf2::Transform> trans_lidar_2_base;
                auto robot = trans.first;
                auto relative_trans = trans.second;
                try
                {
                    tf2::fromMsg(tf_buffer->lookupTransform(
                        // "robot_" + to_string(robot) + "/odom", "robot_" + to_string(robot) + "/base_link", rclcpp::Time(0)), trans_lidar_2_base);
                        "robot_" + to_string(robot) + params.odometry_frame_,
                        "robot_" + to_string(robot) + "/base_link",
                        rclcpp::Time(0)), trans_lidar_2_base);
                }
                catch (tf2::TransformException ex)
                {
                    RCLCPP_ERROR(rclcpp::get_logger(""), "%s", ex.what());
                }
        
                quat_tf.setRPY(relative_trans.rotation().roll(), relative_trans.rotation().pitch(), relative_trans.rotation().yaw());
                tf2::Transform t_world_to_odom = tf2::Transform(quat_tf, tf2::Vector3(relative_trans.translation().x(), relative_trans.translation().y(), relative_trans.translation().z()));
                tf2::Stamped<tf2::Transform> temp_trans(t_world_to_odom, trans_lidar_2_base.stamp_, params.world_frame_);
                // tf2::Stamped<tf2::Transform> temp_trans(
                //     t_world_to_odom,
                //     tf2_ros::fromMsg(this->get_clock()->now()),
                //     params.world_frame_);
                geometry_msgs::msg::TransformStamped trans_world_to_odom;
                tf2::convert(temp_trans, trans_world_to_odom);
                trans_world_to_odom.child_frame_id = "robot_" + to_string(robot) + params.odometry_frame_;
                world_2_odom_broadcaster->sendTransform(trans_world_to_odom);
            }

            if (trans_to_pub.empty())
            {
                tf2::Quaternion quat_tf;
                auto robot = 0;
                auto relative_trans = gtsam::Pose3();
                quat_tf.setRPY(relative_trans.rotation().roll(), relative_trans.rotation().pitch(), relative_trans.rotation().yaw());
                tf2::Transform t_world_to_odom = tf2::Transform(quat_tf, tf2::Vector3(relative_trans.translation().x(), relative_trans.translation().y(), relative_trans.translation().z()));
                tf2::Stamped<tf2::Transform> temp_trans(t_world_to_odom, tf2_ros::fromMsg(this->get_clock()->now()), params.world_frame_);
                geometry_msgs::msg::TransformStamped trans_world_to_odom;
                tf2::convert(temp_trans, trans_world_to_odom);
                trans_world_to_odom.child_frame_id = "robot_" + to_string(robot) + params.odometry_frame_;
                world_2_odom_broadcaster->sendTransform(trans_world_to_odom);
            }
        }
    }

    void saveTrajectoriesService(
        const std::shared_ptr<co_lrio::srv::SaveFiles::Request> req,
        std::shared_ptr<co_lrio::srv::SaveFiles::Response> res)
    {
        res->success = false;
        // directory
        RCLCPP_INFO(rclcpp::get_logger(""), "Saving trajectories to files...");
        std::string directory;
        if(req->destination.empty())
        {
            directory = params.save_directory_;
        }
        else
        {
            directory = std::getenv("HOME") + req->destination;
            // const char* home_env = std::getenv("HOME");
            // if (home_env) {
            //     directory = std::string(home_env) + req->destination;
            // } else {
            //     directory = "/root" + req->destination;
            // }
        }
        RCLCPP_INFO(rclcpp::get_logger(""), "Save directory: %s", directory.c_str());
        std::error_code ec;
        std::filesystem::create_directories(directory, ec);
        if (ec)
        {
            RCLCPP_ERROR(rclcpp::get_logger(""), "Failed to create save directory '%s': %s",
                directory.c_str(), ec.message().c_str());
            return;
        }

        // extract trajectory and map of each robot
        RCLCPP_INFO(rclcpp::get_logger(""), "Extracting trajectories");
        std::map<int8_t, pcl::PointCloud<PointPose6D>::Ptr> poses_6d;
        std::map<int8_t, pcl::PointCloud<PointPose3D>::Ptr> global_maps;
        auto latest_values = optimizer->getLatestValues();
        for (const auto& key_value : latest_values)
        {
            gtsam::Symbol symbol = key_value.key;
            // ignore imu pose and non-Pose3 variables (e.g. ground plane 'G')
            if (symbol.chr() >= 0 && symbol.chr() < 'a')
            {
                continue;
            }
            gtsam::Pose3 pose = latest_values.at<gtsam::Pose3>(symbol);
            int index = symbol.index();
            int8_t robot = symbol.chr() - 'a';

            PointPose6D pose_6d;
            pose_6d.x = pose.translation().x();
            pose_6d.y = pose.translation().y();
            pose_6d.z = pose.translation().z();
            pose_6d.roll = pose.rotation().toQuaternion().x();
            pose_6d.pitch = pose.rotation().toQuaternion().y();
            pose_6d.yaw = pose.rotation().toQuaternion().z();
            pose_6d.intensity = pose.rotation().toQuaternion().w();
            pose_6d.time = index;

            if (poses_6d.find(robot) == poses_6d.end())
            {
                pcl::PointCloud<PointPose6D>::Ptr pose_6f_tmp(new pcl::PointCloud<PointPose6D>());
                pose_6f_tmp->push_back(pose_6d);
                poses_6d.emplace(make_pair(robot, pose_6f_tmp));
            }
            else
            {
                poses_6d.at(robot)->push_back(pose_6d);
            }

            if (global_maps.find(robot) == global_maps.end())
            {
                pcl::PointCloud<PointPose3D>::Ptr global_map(new pcl::PointCloud<PointPose3D>());
                *global_map += *map_database->cloudAt(robot, index, 0);
                global_maps.emplace(make_pair(robot, global_map));
            }
            else
            {
                *global_maps.at(robot) += *map_database->cloudAt(robot, index, 0);
            }
        }

        // save trajectories as .tum file
        RCLCPP_INFO(rclcpp::get_logger(""), "Saving trajectories");
        for (const auto& poses_6d_info : poses_6d)
        {
            std::ofstream tum_file;
            tum_file.open(directory + "/" + prefix.at(poses_6d_info.first) + "_tum_trajectory.txt");
            if (!tum_file.is_open())
            {
                RCLCPP_ERROR(rclcpp::get_logger(""), "Failed to open trajectory file for robot %d", poses_6d_info.first);
                return;
            }
            tum_file.setf(ios::fixed);
            tum_file.precision(10);

            for (const auto& pose : poses_6d_info.second->points)
            {
                tum_file << map_database->timestampAt(poses_6d_info.first, (int)pose.time) << " "
                    << pose.x << " " << pose.y << " " << pose.z << " "
                    << pose.roll << " " << pose.pitch << " " << pose.yaw << " " << pose.intensity << std::endl;
            }

            tum_file.close();
        }

        // save pose graph
        RCLCPP_INFO(rclcpp::get_logger(""), "Saving pose graph");
        auto latest_graph = optimizer->getLatestGraph();
        latest_graph.saveGraph(directory + "/optimized_pose_graph.dot", latest_values);

        auto initial_values = optimizer->getInitialValues();
        auto initial_graph = optimizer->getInitialGraph();
        initial_graph.saveGraph(directory + "/initial_pose_graph.dot", initial_values);
        
        // save global maps pcd
        RCLCPP_INFO(rclcpp::get_logger(""), "Saving maps");
        for (const auto& global_maps_info : global_maps)
        {
            pcl::PointCloud<PointPose3D>::Ptr input_map(new pcl::PointCloud<PointPose3D>());
            *input_map = *global_maps.at(global_maps_info.first);
            static std::vector<int> valid_indices;
            pcl::removeNaNFromPointCloud(*input_map, *input_map, valid_indices);
            if (input_map->empty())
            {
                RCLCPP_WARN(rclcpp::get_logger(""), "Robot %d map is empty after filtering, skip saving pcd", global_maps_info.first);
                continue;
            }

            pcl::PointCloud<PointPose3D>::Ptr globalMapDS(new pcl::PointCloud<PointPose3D>());
            float leaf = 0.2f;
            bool filtered_ok = false;
            for (int retry = 0; retry < 4; ++retry)
            {
                try
                {
                    pcl::VoxelGrid<PointPose3D> downSizeFilterPcdMap;
                    downSizeFilterPcdMap.setLeafSize(leaf, leaf, leaf);
                    downSizeFilterPcdMap.setInputCloud(input_map);
                    downSizeFilterPcdMap.filter(*globalMapDS);
                    filtered_ok = true;
                    break;
                }
                catch (const std::exception& e)
                {
                    RCLCPP_WARN(rclcpp::get_logger(""), "VoxelGrid failed (leaf=%.3f): %s. Retrying with larger leaf.",
                        leaf, e.what());
                    leaf *= 2.0f;
                }
            }
            if (!filtered_ok || globalMapDS->empty())
            {
                RCLCPP_WARN(rclcpp::get_logger(""), "Robot %d downsample failed/empty, save raw map instead", global_maps_info.first);
                globalMapDS = input_map;
            }

            const auto map_path = directory + "/" + prefix.at(global_maps_info.first) + "_map.pcd";
            const int ret = pcl::io::savePCDFileBinary(map_path, *globalMapDS);
            if (ret != 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger(""), "Failed to save map file: %s (ret=%d)", map_path.c_str(), ret);
                return;
            }
        }

        RCLCPP_INFO(rclcpp::get_logger(""), "Saving trajectories and maps to files completed.");
        res->success = true;
    }
};
}  
RCLCPP_COMPONENTS_REGISTER_NODE(co_lrio::ConcentratedMapping)

int main(
    int argc,
    char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.enable_topic_statistics(false);
    options.use_intra_process_comms(false);
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::executor::ExecutorArgs(), 2, true);

    auto CM = std::make_shared<co_lrio::ConcentratedMapping>(options);
    exec.add_node(CM);

    thread loop_closure_detection_thread(&co_lrio::ConcentratedMapping::loopClosureDetectionThread, CM); 
    thread visualization_thread(&co_lrio::ConcentratedMapping::visualizationThread, CM);
    thread tf_thread(&co_lrio::ConcentratedMapping::publishTfThread, CM);
    // thread update_map_thread(&co_lrio::ConcentratedMapping::updateNearMapThread, CM);
    loop_closure_detection_thread.detach();
    visualization_thread.detach();
    tf_thread.detach();
    // update_map_thread.detach();

    RCLCPP_INFO(rclcpp::get_logger(""), "\033[1;32mNode %s %s Started.\033[0m", CM->get_namespace(), CM->get_name());
    
    exec.spin();

    rclcpp::shutdown();

    return 0;
}

//如果我禁用uwb，什么时候触发跨机器人回环检测？
//
