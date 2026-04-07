#include "multiSectorDescriptor.h"

#include <algorithm>
#include <numeric>
#include <omp.h>

MultiSectorDescriptor::MultiSectorDescriptor(
    int ring_num,
    int sector_num,
    int candidates_num,
    float distance_threshold,
    float max_radius,
    int exclude_recent_num,
    std::string directory)
    : pc_ring_num_(0), // will be set on first saved descriptor (rz_dim)
      pc_sector_num_(sector_num),
      distance_threshold_(distance_threshold),
      pc_max_radius_(max_radius),
      exclude_recent_num_(exclude_recent_num),
      candidates_num_(candidates_num),
      save_directory_(directory)
{
    if (!save_directory_.empty()) {
        std::string p = std::getenv("HOME") + save_directory_ + "/msLoop.txt";
        ms_file.open(p);
        ms_file.setf(std::ios::fixed);
        ms_file.precision(10);
    }
}

MultiSectorDescriptor::~MultiSectorDescriptor()
{
    if (ms_file.is_open()) ms_file.close();
}


//把多扇区描述子 CLDType（类型为 std::vector<std::vector<int>>，外层按扇区索引、内层按 rz/bin 索引）转换成一个 Eigen::MatrixXf 矩阵
Eigen::MatrixXf MultiSectorDescriptor::cldToEigen(const CLDType& cld) const
{
    if (cld.empty()) return Eigen::MatrixXf();
    int sectors = static_cast<int>(cld.size());
    int rz_dim = static_cast<int>(cld[0].size());
    Eigen::MatrixXf M = Eigen::MatrixXf::Zero(rz_dim, sectors);
    for (int s = 0; s < sectors; ++s) {
        for (int r = 0; r < rz_dim; ++r) {
            M(r, s) = static_cast<float>(cld[s][r]);
        }
    }
    return M;
}


CLDType MultiSectorDescriptor::eigenToCLD(const Eigen::MatrixXf& mat) const
{
    CLDType out;
    if (mat.size() == 0) return out;
    int rz_dim = mat.rows();
    int sectors = mat.cols();
    out.resize(sectors);
    for (int s = 0; s < sectors; ++s) {
        out[s].resize(rz_dim);
        for (int r = 0; r < rz_dim; ++r) {
            out[s][r] = static_cast<int>(std::round(mat(r, s)));
        }
    }
    return out;
}

std::vector<float> MultiSectorDescriptor::makeDescriptor(const pcl::PointCloud<pcl::PointXYZI>::Ptr scan)
{
    // convert to pcl::PointXYZ cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_xyz->reserve(scan->points.size());
    for (const auto& p : scan->points) {
        cloud_xyz->push_back(pcl::PointXYZ(p.x, p.y, p.z));
    }

    CLDType cld = generate_cld_my(cloud_xyz, pc_sector_num_, pc_max_radius_, 3);
    Eigen::MatrixXf mat = cldToEigen(cld); // rows = rz_dim, cols = sector_num

    std::vector<float> vec(mat.data(), mat.data() + mat.size());
    return vec;
}

void MultiSectorDescriptor::saveDescriptorAndKey(const std::vector<float> descriptor,
                                                const int8_t& robot,
                                                const int& index)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (descriptor.empty()) return;

    if (pc_sector_num_ <= 0) return;

    int total = static_cast<int>(descriptor.size());
    int rz_dim = total / pc_sector_num_;
    if (rz_dim * pc_sector_num_ != total) {
        // malformed descriptor length
        return;
    }

    // set ring dim on first save
    if (pc_ring_num_ == 0) pc_ring_num_ = rz_dim;

    Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>> mat(descriptor.data(), rz_dim, pc_sector_num_);

    // compute ringkey (row-wise mean) 行键
    Eigen::MatrixXf ringkey(rz_dim, 1);
    for (int r = 0; r < rz_dim; ++r) {
        ringkey(r, 0) = mat.row(r).mean();
    }

    // store
    if (ms_features.find(robot) == ms_features.end()) {   //如果不是同一个机器人,则新建一个vector存储这个机器人的描述子
        vector<Eigen::MatrixXf> v; 
        v.emplace_back(mat);  
        ms_features.emplace(robot, v);

        Eigen::MatrixXf rk; 
        rk.conservativeResize(rz_dim, 1); 
        rk.block(0,0,rz_dim,1) = ringkey;
        ms_ringkey.emplace(robot, rk);

        vector<int> local; local.emplace_back(ms_indexes.size());
        local_to_global_maps.emplace(robot, local);
        ms_indexes.emplace_back(make_pair(robot, index));
        all_robots.emplace_back(robot);
    } else {  //如果是同一个机器人，则直接在原来的vector上添加描述子
        ms_features.at(robot).emplace_back(mat);
        int cols = ms_features.at(robot).size();
        ms_ringkey.at(robot).conservativeResize(rz_dim, cols);
        ms_ringkey.at(robot).block(0, cols-1, rz_dim, 1) = ringkey;
        local_to_global_maps.at(robot).emplace_back(ms_indexes.size());
        ms_indexes.emplace_back(make_pair(robot, index));
    }

    // save CLD to database keyed by global index
    int global_idx = static_cast<int>(ms_indexes.size()) - 1;
    CLDType cld = eigenToCLD(mat);
    cld_db_.emplace(global_idx, cld);

    if (ms_file.is_open()) {
        ms_file << (int)robot << " " << index << " " << global_idx << std::endl;
    }
}

std::vector<std::tuple<int8_t, int, int>> MultiSectorDescriptor::detectLoopClosure(
    const int8_t& cur_robot,
    const int& cur_ptr)
{
    std::vector<std::tuple<int8_t, int, int>> result;
    std::lock_guard<std::mutex> lock(mutex_);

    //===========================================================
    // 1) 拼接各机器人历史 ringkey（对当前机器人排除最近若干帧）
    //===========================================================
    Eigen::MatrixXf new_ms_ringkey;
    std::vector<int> new_local_to_global_maps;
    std::vector<Eigen::MatrixXf> new_ms_features;

    for (const auto& robot : all_robots) {
        if (robot == cur_robot) {
            int cur_col = new_ms_ringkey.cols();
            int add_row = static_cast<int>(local_to_global_maps.at(cur_robot).size()) - exclude_recent_num_;
            if (add_row > 0) {
                new_ms_ringkey.conservativeResize(pc_ring_num_, cur_col + add_row);
                new_ms_ringkey.block(0, cur_col, pc_ring_num_, add_row) =
                    ms_ringkey.at(cur_robot).block(0, 0, pc_ring_num_, add_row);
                new_local_to_global_maps.insert(new_local_to_global_maps.end(),
                                                local_to_global_maps.at(cur_robot).begin(),
                                                local_to_global_maps.at(cur_robot).end() - exclude_recent_num_);
                new_ms_features.insert(new_ms_features.end(),
                                       ms_features.at(cur_robot).begin(),
                                       ms_features.at(cur_robot).end() - exclude_recent_num_);
            }
        } else {  // 如果是机器人间查询,则添加全部历史数据 new_ms_ringkey是每帧的列向量，每列是行均值, new_local_to_global_maps是每一帧的全局索引, new_ms_features是每帧的描述子矩阵
            int cur_col = new_ms_ringkey.cols();
            int add_row = static_cast<int>(local_to_global_maps.at(robot).size());
            if (add_row > 0) {
                new_ms_ringkey.conservativeResize(pc_ring_num_, cur_col + add_row);
                new_ms_ringkey.block(0, cur_col, pc_ring_num_, add_row) =
                    ms_ringkey.at(robot).block(0, 0, pc_ring_num_, add_row);
                new_local_to_global_maps.insert(new_local_to_global_maps.end(),
                                                local_to_global_maps.at(robot).begin(),
                                                local_to_global_maps.at(robot).end());
                new_ms_features.insert(new_ms_features.end(),
                                       ms_features.at(robot).begin(),
                                       ms_features.at(robot).end());
            }
        }
    }

    if (new_local_to_global_maps.size() <= static_cast<size_t>(candidates_num_)) return result;

    //===========================================================
    // 2) 用 KD‑tree 对 query ringkey 做粗筛得到若干 candidate索引 
    //===========================================================

    // kd-tree coarse search
    kdTree = Nabo::NNSearchF::createKDTreeLinearHeap(new_ms_ringkey, pc_ring_num_);

    // query ringkey for cur_robot@cur_ptr
    if (ms_ringkey.find(cur_robot) == ms_ringkey.end()) return result; 
    if (cur_ptr >= ms_ringkey.at(cur_robot).cols()) return result;
    Eigen::MatrixXf query_rk = ms_ringkey.at(cur_robot).col(cur_ptr);  //当前机器人的当前帧的行键 

    Eigen::VectorXi indices(candidates_num_); 
    Eigen::VectorXf distance(candidates_num_); 
    kdTree->knn(query_rk, indices, distance, candidates_num_);  

    // reconstruct query CLD
    int query_global = -1;
    for (int gi = 0; gi < (int)ms_indexes.size(); ++gi) {
        if (ms_indexes[gi].first == cur_robot && ms_indexes[gi].second == cur_ptr) {
            query_global = gi;  //当前机器人的当前帧对应的全局索引  即确定全局表中哪一项对应当前请求的 (robot, index)。
            break;
        }
    }
    if (query_global == -1) return result;
    CLDType query_cld = cld_db_.at(query_global);  //从全局数据库中取出 query 的 CLD 描述子矩阵

    //=============================================================
    // 3) 对每个 candidate 恢复对应全局索引，从 cld_db_ 中取 CLD，用 complete_similarity 精排
    //=============================================================
    std::map<int8_t, float> best_score_same;  //同机器人最佳得分
    std::map<int8_t, int> best_global_same;  //同机器人最佳全局索引 
    std::map<int8_t, int> yaw_same;  //同机器人最佳旋转偏移 

    std::map<int8_t, float> best_score_other; //异机器人最佳得分 
    std::map<int8_t, int> best_global_other; //异机器人最佳全局索引 
    std::map<int8_t, int> yaw_other; //异机器人最佳旋转偏移 

    std::vector<int> indices_vec(indices.data(), indices.data() + indices.size()); // 将 Eigen::VectorXi 转换为 std::vector<int> 以便后续处理

    struct CandRes { 
    bool valid=false; 
    int8_t robot=0; 
    int global_idx=-1; 
    float score=0.f; 
    int rot=0; 
    }; 

    std::vector<CandRes> cres(indices_vec.size());

    // 为KD候选列表中的每个索引并行计算CLD相似度，并记录每个候选的机器人ID、全局索引、相似度得分和旋转偏移
    #pragma omp parallel for schedule(dynamic) 
    for (int ii = 0; ii < static_cast<int>(indices_vec.size()); ++ii) { 
        int idx = indices_vec[ii]; // 
        if (idx < 0) continue;
        if (idx >= static_cast<int>(new_local_to_global_maps.size())) continue;
        int global_idx = new_local_to_global_maps[idx]; // 从局部索引恢复全局索引
        auto itdb = cld_db_.find(global_idx); //
        if (itdb == cld_db_.end()) continue;  
        CLDType candidate_cld = itdb->second;  //描述子矩阵 

        int qsize = 0, off = 0, zlen = 0;
        bool frot = false;
        std::vector<int> newtable = {0, -1, 3};
        std::vector<float> sim = complete_similarity(candidate_cld, query_cld, newtable, qsize, frot, off, zlen);
        if (sim.empty()) continue; // 相似度计算失败 
        auto it = std::max_element(sim.begin(), sim.end());  
        float val = *it;
        int idx_best = static_cast<int>(std::distance(sim.begin(), it));
        int rot = frot ? (idx_best - off) : idx_best; 

        // read robot id (safe read)
        auto p = ms_indexes.at(global_idx);
        int8_t robot = p.first;

        cres[ii].valid = true;  //标记该候选项有效 
        cres[ii].robot = robot; //记录候选项所属机器人ID
        cres[ii].global_idx = global_idx; //记录候选项对应的全局索引 
        cres[ii].score = val; //记录候选项的相似度得分 
        cres[ii].rot = rot; //记录候选项的旋转偏移 
    }

    //=============================================================
    // 4) 对每个 robot 保留最优候选，返回 (robot, per_robot_index, yaw)
    //=============================================================

    // 遍历所有候选结果，分别记录同机器人和异机器人的最佳得分、全局索引和旋转偏移 
    for (const auto &c : cres) {
        if (!c.valid) continue; 
        if (c.robot == cur_robot) {  //同机器人候选项
            if (best_score_same.find(c.robot) == best_score_same.end() || c.score > best_score_same[c.robot]) {
                best_score_same[c.robot] = c.score;
                best_global_same[c.robot] = c.global_idx; 
                yaw_same[c.robot] = c.rot;
            }
        } else {
            if (best_score_other.find(c.robot) == best_score_other.end() || c.score > best_score_other[c.robot]) {
                best_score_other[c.robot] = c.score;
                best_global_other[c.robot] = c.global_idx;
                yaw_other[c.robot] = c.rot;
            }
        }
    }

    size_t cap = static_cast<size_t>(std::max(1, candidates_num_));  //确保至少返回一个候选项（如果有的话）

    // add same-robot candidate first (usually at most one entry for cur_robot)
    //如果同机器人中有候选项且结果数量未达上限，则优先添加同机器人得分最高的候选项到结果列表中
    if (best_score_same.find(cur_robot) != best_score_same.end() && result.size() < cap) {
        int gidx = best_global_same[cur_robot];
        auto pr = ms_indexes.at(gidx);
        result.emplace_back(std::make_tuple(pr.first, pr.second, yaw_same[cur_robot]));
    }

    // 如果结果数量未达上限，则按得分从高到低添加异机器人候选项，直到达到上限 
    std::vector<std::pair<int8_t, float>> other_list;
    other_list.reserve(best_score_other.size());
    for (const auto &kv : best_score_other) {
        other_list.emplace_back(kv.first, kv.second);
    }
    std::sort(other_list.begin(), other_list.end(), [](const std::pair<int8_t,float>& a, const std::pair<int8_t,float>& b){
        return a.second > b.second;
    });

    for (const auto &p : other_list) {
        if (result.size() >= cap) break;
        int8_t robot = p.first;
        int gidx = best_global_other[robot];
        auto pr = ms_indexes.at(gidx);
        result.emplace_back(std::make_tuple(pr.first, pr.second, yaw_other[robot]));
    }

    return result;
}

std::pair<int8_t, int> MultiSectorDescriptor::getIndex(const int& key)
{
    return ms_indexes.at(key);
}

int MultiSectorDescriptor::getSize(const int8_t& id)
{
    if (id == -1) return static_cast<int>(ms_indexes.size());
    return static_cast<int>(local_to_global_maps.at(id).size());
}
