#ifndef _MULTI_SECTOR_DESCRIPTOR_H_
#define _MULTI_SECTOR_DESCRIPTOR_H_

#include "descriptorBasis.h"
#include "seu_hfx.hpp"
// #include "robustOptimizer.h"
#include <unordered_map>
#include <mutex>

class MultiSectorDescriptor : public ScanDescriptor
{
public:
    MultiSectorDescriptor(
        int ring_num = 20,
        int sector_num = 24,
        int candidates_num = 6,
        float distance_threshold = 1000.0f,
        float max_radius = 50.0f,
        int exclude_recent_num = 100,
        std::string directory = "") ;

    ~MultiSectorDescriptor();

    // User-side API (compatible with ScanContextDescriptor)
    std::vector<float> makeDescriptor(const pcl::PointCloud<pcl::PointXYZI>::Ptr scan) override;

    void saveDescriptorAndKey(const std::vector<float> descriptor,
                              const int8_t& robot,
                              const int& index) override;

    std::vector<std::tuple<int8_t, int, int>> detectLoopClosure(
        const int8_t& cur_robot,
        const int& cur_ptr) override;

    std::pair<int8_t, int> getIndex(const int& key) override;

    int getSize(const int8_t& id = -1) override;

private:
    // helpers
    Eigen::MatrixXf cldToEigen(const CLDType& cld) const;
    CLDType eigenToCLD(const Eigen::MatrixXf& mat) const;

    // parameters
    int pc_ring_num_; // will be set to rz_dim when first descriptor saved
    int pc_sector_num_;
    float distance_threshold_;
    float pc_max_radius_;
    int exclude_recent_num_;
    int candidates_num_;

    // data containers (mirror ScanContextDescriptor layout for easy swap)
    std::unordered_map<int8_t, std::vector<Eigen::MatrixXf>> ms_features; // ringkey matrices per robot
    std::unordered_map<int8_t, Eigen::MatrixXf> ms_ringkey;
    std::unordered_map<int8_t, std::vector<int>> local_to_global_maps;
    std::vector<std::pair<int8_t, int>> ms_indexes; // global index -> (robot, index)

    // CLD storage keyed by global index
    std::unordered_map<int, CLDType> cld_db_;

    // kd tree
    Nabo::NNSearchF* kdTree = NULL;

    // other
    std::vector<int8_t> all_robots;
    std::string save_directory_;
    std::ofstream ms_file;
    std::mutex mutex_;
};

#endif
#ifndef _MULTI_SECTOR_DESCRIPTOR_H_
#define _MULTI_SECTOR_DESCRIPTOR_H_

#include "descriptorBasis.h"
#include <unordered_map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

typedef std::vector<std::vector<int>> CLDType;

class MultiSectorDescriptor : public ScanDescriptor
{
public:
    MultiSectorDescriptor(
        int sector_num = 24,
        float max_range = 50.0f,
        int min_points = 3,
        int candidates_num = 6,
        float distance_threshold = 0.14f,
        int exclude_recent = 100,
        std::string directory = "/root/cslam_ws/src/Co-LRIO/co_lrio_output");

    ~MultiSectorDescriptor();

    // User API (match ScanDescriptor)
    std::vector<float> makeDescriptor(const pcl::PointCloud<pcl::PointXYZI>::Ptr scan) override;
    void saveDescriptorAndKey(const std::vector<float> descriptor, const int8_t& robot, const int& index) override;
    std::vector<std::tuple<int8_t, int, int>> detectLoopClosure(const int8_t& cur_robot, const int& cur_ptr) override;
    std::pair<int8_t, int> getIndex(const int& key) override;
    int getSize(const int8_t& id = -1) override;

private:
    // CLD utilities (simple local implementations)
    CLDType generate_cld_my(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                            int num_sectors, float max_range, int min_points_per_sector);

    std::vector<float> complete_similarity(const CLDType& map_cld,
                                           const CLDType& query_cld,
                                           const std::vector<int>& newtable,
                                           int& query_size,
                                           bool& flag_rot,
                                           int& offset_sector,
                                           int& zero_len);

    std::vector<uint16_t> create_lookup_table();

private:
    int sector_num_;
    float max_range_;
    int min_points_;
    int candidates_num_;
    float distance_threshold_;
    int exclude_recent_num_;

    // storage
    std::unordered_map<int8_t, std::vector<CLDType>> cld_store_; // robot -> list of CLDs
    std::unordered_map<int8_t, Eigen::MatrixXf> cld_sector_key_; // robot -> sector summary per column
    std::unordered_map<int8_t, std::vector<int>> local_to_global_maps_;
    std::vector<std::pair<int8_t,int>> descriptor_indexes_; // global index -> (robot, original index)
    std::vector<int8_t> all_robots_;

    // kd-tree
    Nabo::NNSearchF* kdTree = NULL;

    // lookup table for similarity
    std::vector<int> lookup_table_int_; // small int-based table

    // file
    std::string save_directory_;
    std::ofstream sc_file_;
};

#endif
