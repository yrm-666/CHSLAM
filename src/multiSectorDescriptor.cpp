#include "multiSectorDescriptor.h"

#include <algorithm>
#include <numeric>
#include <omp.h>

MultiSectorDescriptor::MultiSectorDescriptor(
    int   ring_num,
    int   sector_num,
    int   candidates_num,
    float distance_threshold,
    float max_radius,
    int   exclude_recent_num,
    std::string directory)
    : pc_sector_num_(sector_num),
      pc_max_radius_(max_radius),
      distance_threshold_(distance_threshold),
      exclude_recent_num_(exclude_recent_num),
      candidates_num_(candidates_num),
      save_directory_(directory)
{
    if (!save_directory_.empty()) {
        ms_file.open(save_directory_ + "/msLoop.txt");
        ms_file.setf(std::ios::fixed);
        ms_file.precision(10);
    }
}

MultiSectorDescriptor::~MultiSectorDescriptor()
{
    if (ms_file.is_open()) ms_file.close();
}

// ---------------------------------------------------------------------------
//  makeDescriptor
//  Calls generate_cld_my (seu_hfx.hpp) — same as loop_closure_detector.hpp.
//  Flattens CLDType to float vector with sector-major layout:
//    vec[s * rz_dim + r] = cld[s][r]
//  saveDescriptorAndKey uses the same layout to reconstruct CLDType.
// ---------------------------------------------------------------------------
std::vector<float> MultiSectorDescriptor::makeDescriptor(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr scan)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_xyz->reserve(scan->points.size());
    for (const auto& p : scan->points)
        cloud_xyz->push_back(pcl::PointXYZ(p.x, p.y, p.z));

    // generate_cld_my: inline free function in seu_hfx.hpp
    CLDType cld = generate_cld_my(cloud_xyz, pc_sector_num_, pc_max_radius_, 3);
    if (cld.empty() || cld[0].empty()) return {};

    const int rz_dim = static_cast<int>(cld[0].size());
    std::vector<float> vec;
    vec.reserve(pc_sector_num_ * rz_dim);
    for (int s = 0; s < pc_sector_num_; ++s)
        for (int r = 0; r < rz_dim; ++r)
            vec.push_back(static_cast<float>(cld[s][r]));
    return vec;
}

// ---------------------------------------------------------------------------
//  saveDescriptorAndKey
//  Reconstructs CLDType from the float vector (sector-major layout, matching
//  makeDescriptor above) and appends to the flat database_.
//  O(1) append — no ring-key matrix, no conservativeResize.
// ---------------------------------------------------------------------------
void MultiSectorDescriptor::saveDescriptorAndKey(
    const std::vector<float> descriptor,
    const int8_t& robot,
    const int& index)
{
    if (descriptor.empty() || pc_sector_num_ <= 0) return;

    const int total  = static_cast<int>(descriptor.size());
    const int rz_dim = total / pc_sector_num_;
    if (rz_dim * pc_sector_num_ != total) return;

    CLDType cld(pc_sector_num_, std::vector<int>(rz_dim));
    for (int s = 0; s < pc_sector_num_; ++s)
        for (int r = 0; r < rz_dim; ++r)
            cld[s][r] = static_cast<int>(descriptor[s * rz_dim + r]);

    std::lock_guard<std::mutex> lock(mutex_);

    const int global_idx = static_cast<int>(database_.size());
    database_.push_back({robot, index, std::move(cld)});

    if (local_to_global_maps.find(robot) == local_to_global_maps.end()) {
        local_to_global_maps.emplace(robot, std::vector<int>{});
        all_robots.push_back(robot);
    }
    local_to_global_maps.at(robot).push_back(global_idx);
    ms_indexes.emplace_back(robot, index);

    if (ms_file.is_open())
        ms_file << static_cast<int>(robot) << " " << index << " " << global_idx << "\n";
}

// ---------------------------------------------------------------------------
//  detectLoopClosure
//  Stage-1 of loop_closure_detector.hpp: full O(N) CLD similarity search
//  across all robots with OMP parallelism, returns top-K candidates.
//  No KNN pre-filter — ensures inter-robot candidates are not missed.
//  GICP + Overlap verification is done in the main node (calculateTransformation).
// ---------------------------------------------------------------------------
std::vector<std::tuple<int8_t, int, int>> MultiSectorDescriptor::detectLoopClosure(
    const int8_t& cur_robot,
    const int& cur_ptr)
{
    std::vector<std::tuple<int8_t, int, int>> result;
    std::lock_guard<std::mutex> lock(mutex_);

    // locate query frame in the database
    int query_global = -1;
    if (local_to_global_maps.count(cur_robot)) {
        for (int gi : local_to_global_maps.at(cur_robot)) {
            if (database_[gi].robot_key == cur_ptr) {
                query_global = gi;
                break;
            }
        }
    }
    if (query_global < 0) return result;
    const CLDType& query_cld = database_[query_global].cld;

    // build search list: same robot excludes recent frames, other robots all
    struct SearchEntry { int global_idx; int8_t robot; int robot_key; };
    std::vector<SearchEntry> entries;
    entries.reserve(database_.size());
    for (const auto& robot : all_robots) {
        const auto& glist = local_to_global_maps.at(robot);
        const int total   = static_cast<int>(glist.size());
        const int limit   = (robot == cur_robot) ? (total - exclude_recent_num_) : total;
        for (int li = 0; li < limit; ++li) {
            const int gi = glist[li];
            if (gi == query_global) continue;
            entries.push_back({gi, robot, database_[gi].robot_key});
        }
    }
    if (entries.empty()) return result;

    // parallel CLD similarity search — same as loop_closure_detector.hpp Stage-1
    struct Candidate { int8_t robot; int robot_key; float score; int rot; };
    std::vector<Candidate> candidates;

    #pragma omp parallel
    {
        std::vector<Candidate> local;
        #pragma omp for schedule(dynamic, 8) nowait
        for (int i = 0; i < static_cast<int>(entries.size()); ++i) {
            const auto& e = entries[i];
            int qsize = 0, off = 0, zlen = 0;
            bool frot = false;
            const std::vector<int> newtable = {0, -1, 3};  // same as loop_closure_detector.hpp

            auto sim = complete_similarity(
                database_[e.global_idx].cld, query_cld,
                newtable, qsize, frot, off, zlen);
            if (sim.empty()) continue;

            const auto  it  = std::max_element(sim.begin(), sim.end());
            const float val = *it;
            const int   idx = static_cast<int>(std::distance(sim.begin(), it));
            const int   rot = frot ? (idx - off) : idx;
            local.push_back({e.robot, e.robot_key, val, rot});
        }
        #pragma omp critical
        { candidates.insert(candidates.end(), local.begin(), local.end()); }
    }

    // printf("[LCD] query=robot_%d_frame_%d | db=%zu | search=%zu\n",
    //        static_cast<int>(cur_robot), cur_ptr,
    //        database_.size(), entries.size());

    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) { return a.score > b.score; });

    const int cap = std::min(candidates_num_, static_cast<int>(candidates.size()));
    for (int i = 0; i < cap; ++i)
        result.emplace_back(candidates[i].robot, candidates[i].robot_key, candidates[i].rot);

    return result;
}

std::pair<int8_t, int> MultiSectorDescriptor::getIndex(const int& key)
{
    std::lock_guard<std::mutex> lock(mutex_);
    return ms_indexes.at(key);
}

int MultiSectorDescriptor::getSize(const int8_t& id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (id == -1) return static_cast<int>(database_.size());
    if (!local_to_global_maps.count(id)) return 0;
    return static_cast<int>(local_to_global_maps.at(id).size());
}
