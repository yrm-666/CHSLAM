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
    // parameters
    int pc_sector_num_;
    float pc_max_radius_;
    float distance_threshold_;
    int exclude_recent_num_;
    int candidates_num_;

    // per-frame record: mirrors loop_closure_detector.hpp FrameDescriptor
    // (flat vector, no ring-key matrix, no conservativeResize overhead)
    struct MSFrame {
        int8_t  robot_id;
        int     robot_key;
        CLDType cld;
    };

    // flat global database (cache-friendly, O(1) append)
    std::vector<MSFrame> database_;
    // per-robot list of global indices, in insertion order
    std::unordered_map<int8_t, std::vector<int>> local_to_global_maps;
    std::vector<std::pair<int8_t, int>> ms_indexes; // global index -> (robot, robot_key)
    std::vector<int8_t> all_robots;

    std::string save_directory_;
    std::ofstream ms_file;
    std::mutex mutex_;
};

#endif
