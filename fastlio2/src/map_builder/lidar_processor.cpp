#include "lidar_processor.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>

namespace
{
double percentile(std::vector<double> values, double fraction)
{
    if (values.empty())
        return std::numeric_limits<double>::quiet_NaN();
    const size_t index = std::min(values.size() - 1,
                                  static_cast<size_t>(fraction * static_cast<double>(values.size() - 1)));
    std::nth_element(values.begin(), values.begin() + index, values.end());
    return values[index];
}

V3D rpyDegrees(const M3D &rotation)
{
    return rotation.eulerAngles(2, 1, 0).reverse() * (180.0 / M_PI);
}

void appendVector(std::vector<double> &out, const V3D &value)
{
    out.push_back(value.x());
    out.push_back(value.y());
    out.push_back(value.z());
}
}

const std::string &LidarProcessor::iterationDiagnosticsSchema()
{
    static const std::string schema =
        "scan,iteration,input,neighbors_ok,neighbor_distance_ok,plane_ok,effective,"
        "reject_missing_neighbors,reject_far_neighbors,reject_plane,reject_score,"
        "neighbor_m_mean,neighbor_m_p90,neighbor_m_max,residual_signed_mean,"
        "residual_abs_mean,residual_rms,residual_abs_p50,residual_abs_p90,"
        "residual_abs_p99,residual_abs_max,score_mean,score_min,normal_horizontal,"
        "normal_vertical,normal_oblique,normal_eigen_min,normal_eigen_mid,"
        "normal_eigen_max,information_eigen_min,information_eigen_max,"
        "information_condition,input_range_0_2,input_range_2_5,input_range_5_10,"
        "input_range_10_plus,input_body_below_0_5,input_body_middle,input_body_above_0_5,"
        "input_body_z_min,input_body_z_p10,input_body_z_p50,input_body_z_p90,input_body_z_max,"
        "effective_range_0_2,effective_range_2_5,effective_range_5_10,effective_range_10_plus,"
        "effective_body_below_0_5,effective_body_middle,effective_body_above_0_5,"
        "effective_body_z_min,effective_body_z_p10,effective_body_z_p50,effective_body_z_p90,"
        "effective_body_z_max,state_x,state_y,state_z,"
        "state_roll_deg,state_pitch_deg,state_yaw_deg,state_vx,state_vy,state_vz";
    return schema;
}

const std::string &LidarProcessor::scanDiagnosticsSchema()
{
    static const std::string schema =
        "scan,stamp,input,downsampled,map_size_before,map_valid_before,map_size_after,"
        "map_valid_after,map_boxes_removed,map_points_requested,map_points_inserted,"
        "iterations,final_valid,pre_x,pre_y,pre_z,post_x,post_y,post_z,dx,dy,dz,"
        "pre_roll_deg,pre_pitch_deg,pre_yaw_deg,post_roll_deg,post_pitch_deg,"
        "post_yaw_deg,droll_deg,dpitch_deg,dyaw_deg,pre_vx,pre_vy,pre_vz,post_vx,"
        "post_vy,post_vz,dvx,dvy,dvz,cov_roll,cov_pitch,cov_yaw,cov_x,cov_y,cov_z,"
        "cov_vx,cov_vy,cov_vz";
    return schema;
}

LidarProcessor::LidarProcessor(Config &config, std::shared_ptr<IESKF> kf) : m_config(config), m_kf(kf)
{
    m_ikdtree = std::make_shared<KD_TREE<PointType>>();
    m_ikdtree->set_downsample_param(m_config.map_resolution);
    m_cloud_down_lidar.reset(new CloudType);
    m_cloud_down_world.reset(new CloudType(10000, 1));
    m_norm_vec.reset(new CloudType(10000, 1));
    m_effect_cloud_lidar.reset(new CloudType(10000, 1));
    m_effect_norm_vec.reset(new CloudType(10000, 1));
    m_nearest_points.resize(10000);
    m_point_selected_flag.resize(10000, false);
    m_diagnostic_correspondences.reset(new CloudType);

    if (m_config.scan_resolution > 0.0)
    {
        m_scan_filter.setLeafSize(m_config.scan_resolution, m_config.scan_resolution, m_config.scan_resolution);
    }

    m_kf->setLossFunction([&](State &s, SharedState &d)
                          { updateLossFunc(s, d); });
    m_kf->setStopFunction([&](const V21D &delta) -> bool
                          { V3D rot_delta = delta.block<3, 1>(0, 0);
                            V3D t_delta = delta.block<3, 1>(3, 0);
                            return (rot_delta.norm() * 57.3 < 0.01) && (t_delta.norm() * 100 < 0.015); });
}

void LidarProcessor::trimCloudMap()
{
    m_local_map.cub_to_rm.clear();
    m_map_boxes_removed = 0;
    const State &state = m_kf->x();
    Eigen::Vector3d pos_lidar = state.t_wi + state.r_wi * state.t_il;

    if (!m_local_map.initialed)
    {
        for (int i = 0; i < 3; i++)
        {
            m_local_map.local_map_corner.vertex_min[i] = pos_lidar[i] - m_config.cube_len / 2.0;
            m_local_map.local_map_corner.vertex_max[i] = pos_lidar[i] + m_config.cube_len / 2.0;
        }
        m_local_map.initialed = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    double det_thresh = m_config.move_thresh * m_config.det_range;
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_lidar(i) - m_local_map.local_map_corner.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_lidar(i) - m_local_map.local_map_corner.vertex_max[i]);

        if (dist_to_map_edge[i][0] <= det_thresh || dist_to_map_edge[i][1] <= det_thresh)
            need_move = true;
    }
    if (!need_move)
        return;
    BoxPointType new_corner, temp_corner;
    new_corner = m_local_map.local_map_corner;
    float mov_dist = std::max((m_config.cube_len - 2.0 * m_config.move_thresh * m_config.det_range) * 0.5 * 0.9, double(m_config.det_range * (m_config.move_thresh - 1)));

    for (int i = 0; i < 3; i++)
    {
        temp_corner = m_local_map.local_map_corner;
        if (dist_to_map_edge[i][0] <= det_thresh)
        {
            new_corner.vertex_max[i] -= mov_dist;
            new_corner.vertex_min[i] -= mov_dist;
            temp_corner.vertex_min[i] = m_local_map.local_map_corner.vertex_max[i] - mov_dist;
            m_local_map.cub_to_rm.push_back(temp_corner);
        }
        else if (dist_to_map_edge[i][1] <= det_thresh)
        {
            new_corner.vertex_max[i] += mov_dist;
            new_corner.vertex_min[i] += mov_dist;
            temp_corner.vertex_max[i] = m_local_map.local_map_corner.vertex_min[i] + mov_dist;
            m_local_map.cub_to_rm.push_back(temp_corner);
        }
    }
    m_local_map.local_map_corner = new_corner;

    PointVec points_history;
    m_ikdtree->acquire_removed_points(points_history);

    // 删除局部地图之外的点云
    if (m_local_map.cub_to_rm.size() > 0)
        m_ikdtree->Delete_Point_Boxes(m_local_map.cub_to_rm);
    m_map_boxes_removed = static_cast<int>(m_local_map.cub_to_rm.size());
    return;
}

void LidarProcessor::incrCloudMap()
{
    m_map_points_requested = 0;
    m_map_points_inserted = 0;
    if (m_cloud_down_lidar->empty())
        return;
    const State &state = m_kf->x();
    int size = m_cloud_down_lidar->size();
    PointVec point_to_add;
    PointVec point_no_need_downsample;
    for (int i = 0; i < size; i++)
    {
        const PointType &p = m_cloud_down_lidar->points[i];
        Eigen::Vector3d point(p.x, p.y, p.z);
        point = state.r_wi * (state.r_il * point + state.t_il) + state.t_wi;
        m_cloud_down_world->points[i].x = point(0);
        m_cloud_down_world->points[i].y = point(1);
        m_cloud_down_world->points[i].z = point(2);
        m_cloud_down_world->points[i].intensity = m_cloud_down_lidar->points[i].intensity;
        // 如果该点附近没有近邻点则需要添加到地图中
        if (m_nearest_points[i].empty())
        {
            point_to_add.push_back(m_cloud_down_world->points[i]);
            continue;
        }

        const PointVec &points_near = m_nearest_points[i];
        bool need_add = true;
        PointType downsample_result, mid_point;
        mid_point.x = std::floor(m_cloud_down_world->points[i].x / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;
        mid_point.y = std::floor(m_cloud_down_world->points[i].y / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;
        mid_point.z = std::floor(m_cloud_down_world->points[i].z / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;

        // 如果该点所在的voxel没有点，则直接加入地图，且不需要降采样
        if (fabs(points_near[0].x - mid_point.x) > 0.5 * m_config.map_resolution && fabs(points_near[0].y - mid_point.y) > 0.5 * m_config.map_resolution && fabs(points_near[0].z - mid_point.z) > 0.5 * m_config.map_resolution)
        {
            point_no_need_downsample.push_back(m_cloud_down_world->points[i]);
            continue;
        }
        float dist = sq_dist(m_cloud_down_world->points[i], mid_point);

        for (int readd_i = 0; readd_i < m_config.near_search_num; readd_i++)
        {
            // 如果该点的近邻点较少，则需要加入到地图中
            if (points_near.size() < static_cast<size_t>(m_config.near_search_num))
                break;
            // 如果该点的近邻点距离voxel中心点的距离比该点距离voxel中心点更近，则不需要加入该点
            if (sq_dist(points_near[readd_i], mid_point) < dist)
            {
                need_add = false;
                break;
            }
        }
        if (need_add)
            point_to_add.push_back(m_cloud_down_world->points[i]);
    }
    m_map_points_requested = static_cast<int>(point_to_add.size() + point_no_need_downsample.size());
    m_map_points_inserted = m_ikdtree->Add_Points(point_to_add, true);
    m_map_points_inserted += m_ikdtree->Add_Points(point_no_need_downsample, false);
}

void LidarProcessor::initCloudMap(PointVec &point_vec)
{
    m_ikdtree->Build(point_vec);
}

void LidarProcessor::process(SyncPackage &package)
{
    // m_kf->setLossFunction([&](State &s, SharedState &d)
    //                       { updateLossFunc(s, d); });
    // m_kf->setStopFunction([&](const V21D &delta) -> bool
    //                       { V3D rot_delta = delta.block<3, 1>(0, 0);
    //                         V3D t_delta = delta.block<3, 1>(3, 0);
    //                         return (rot_delta.norm() * 57.3 < 0.01) && (t_delta.norm() * 100 < 0.015); });
    ++m_scan_sequence;
    m_current_iteration = 0;
    m_iteration_diagnostics.clear();
    m_diagnostic_correspondences->clear();
    const State state_before = m_kf->x();
    const int map_size_before = m_ikdtree->size();
    const int map_valid_before = m_ikdtree->validnum();

    if (m_config.scan_resolution > 0.0)
    {
        m_scan_filter.setInputCloud(package.cloud);
        m_scan_filter.filter(*m_cloud_down_lidar);
    }
    else
    {
        pcl::copyPointCloud(*package.cloud, *m_cloud_down_lidar);
    }
    trimCloudMap();
    m_kf->update();
    incrCloudMap();

    if (!m_config.diagnostics_enabled)
        return;

    const State &state_after = m_kf->x();
    const V3D position_delta = state_after.t_wi - state_before.t_wi;
    const V3D velocity_delta = state_after.v - state_before.v;
    const V3D pre_rpy = rpyDegrees(state_before.r_wi);
    const V3D post_rpy = rpyDegrees(state_after.r_wi);
    const V3D rotation_delta = Sophus::SO3d(state_before.r_wi.transpose() * state_after.r_wi).log() * (180.0 / M_PI);
    const M21D &covariance = m_kf->P();
    std::vector<double> values;
    values.reserve(49);
    values.push_back(static_cast<double>(m_scan_sequence));
    values.push_back(package.cloud_end_time);
    values.push_back(static_cast<double>(package.cloud->size()));
    values.push_back(static_cast<double>(m_cloud_down_lidar->size()));
    values.push_back(map_size_before);
    values.push_back(map_valid_before);
    values.push_back(m_ikdtree->size());
    values.push_back(m_ikdtree->validnum());
    values.push_back(m_map_boxes_removed);
    values.push_back(m_map_points_requested);
    values.push_back(m_map_points_inserted);
    values.push_back(static_cast<double>(m_iteration_diagnostics.size()));
    values.push_back(m_iteration_diagnostics.empty() ? 0.0 :
                     (m_iteration_diagnostics.back().values[6] > 0.0 ? 1.0 : 0.0));
    appendVector(values, state_before.t_wi);
    appendVector(values, state_after.t_wi);
    appendVector(values, position_delta);
    appendVector(values, pre_rpy);
    appendVector(values, post_rpy);
    appendVector(values, rotation_delta);
    appendVector(values, state_before.v);
    appendVector(values, state_after.v);
    appendVector(values, velocity_delta);
    for (int index : {0, 1, 2, 3, 4, 5, 12, 13, 14})
        values.push_back(covariance(index, index));
    m_scan_diagnostics.values = std::move(values);
}

void LidarProcessor::updateLossFunc(State &state, SharedState &share_data)
{
    int size = m_cloud_down_lidar->size();
    if (m_rejection_reason.size() < static_cast<size_t>(size))
    {
        m_rejection_reason.resize(size);
        m_kth_neighbor_distance.resize(size);
        m_plane_residual.resize(size);
        m_selection_score.resize(size);
    }
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < size; i++)
    {
        m_rejection_reason[i] = 1;
        m_kth_neighbor_distance[i] = std::numeric_limits<double>::quiet_NaN();
        m_plane_residual[i] = std::numeric_limits<double>::quiet_NaN();
        m_selection_score[i] = std::numeric_limits<double>::quiet_NaN();
        PointType &point_body = m_cloud_down_lidar->points[i];
        PointType &point_world = m_cloud_down_world->points[i];
        Eigen::Vector3d point_body_vec(point_body.x, point_body.y, point_body.z);
        Eigen::Vector3d point_world_vec = state.r_wi * (state.r_il * point_body_vec + state.t_il) + state.t_wi;
        point_world.x = point_world_vec(0);
        point_world.y = point_world_vec(1);
        point_world.z = point_world_vec(2);
        point_world.intensity = point_body.intensity;
        std::vector<float> point_sq_dist(m_config.near_search_num);
        auto &points_near = m_nearest_points[i];
        m_ikdtree->Nearest_Search(point_world, m_config.near_search_num, points_near, point_sq_dist);
        if (points_near.size() < static_cast<size_t>(m_config.near_search_num))
        {
            m_point_selected_flag[i] = false;
            continue;
        }
        m_kth_neighbor_distance[i] = std::sqrt(std::max(0.0f, point_sq_dist[m_config.near_search_num - 1]));
        if (point_sq_dist[m_config.near_search_num - 1] > 5)
        {
            m_rejection_reason[i] = 2;
            m_point_selected_flag[i] = false;
            continue;
        }
        m_rejection_reason[i] = 3;
        m_point_selected_flag[i] = true;
        if (!m_point_selected_flag[i])
            continue;

        Eigen::Vector4d pabcd;
        m_point_selected_flag[i] = false;
        if (esti_plane(points_near, 0.1, pabcd))
        {
            m_rejection_reason[i] = 4;
            double pd2 = pabcd(0) * point_world_vec(0) + pabcd(1) * point_world_vec(1) + pabcd(2) * point_world_vec(2) + pabcd(3);
            double s = 1 - 0.9 * std::fabs(pd2) / std::sqrt(point_body_vec.norm());
            m_plane_residual[i] = pd2;
            m_selection_score[i] = s;
            m_norm_vec->points[i].x = pabcd(0);
            m_norm_vec->points[i].y = pabcd(1);
            m_norm_vec->points[i].z = pabcd(2);
            m_norm_vec->points[i].intensity = pd2;
            if (s > 0.9)
            {
                m_rejection_reason[i] = 0;
                m_point_selected_flag[i] = true;
            }
        }
    }

    int effect_feat_num = 0;
    m_diagnostic_correspondences->clear();
    for (int i = 0; i < size; i++)
    {
        PointType diagnostic_point = m_cloud_down_world->points[i];
        diagnostic_point.curvature = static_cast<float>(m_rejection_reason[i]);
        diagnostic_point.intensity = std::isfinite(m_plane_residual[i]) ?
            static_cast<float>(m_plane_residual[i]) : std::numeric_limits<float>::quiet_NaN();
        if (m_rejection_reason[i] == 0 || m_rejection_reason[i] == 4)
        {
            diagnostic_point.normal_x = m_norm_vec->points[i].x;
            diagnostic_point.normal_y = m_norm_vec->points[i].y;
            diagnostic_point.normal_z = m_norm_vec->points[i].z;
        }
        else
        {
            diagnostic_point.normal_x = 0.0f;
            diagnostic_point.normal_y = 0.0f;
            diagnostic_point.normal_z = 0.0f;
        }
        m_diagnostic_correspondences->push_back(diagnostic_point);

        if (!m_point_selected_flag[i])
            continue;
        m_effect_cloud_lidar->points[effect_feat_num] = m_cloud_down_lidar->points[i];
        m_effect_norm_vec->points[effect_feat_num] = m_norm_vec->points[i];
        effect_feat_num++;
    }
    if (effect_feat_num < 1)
    {
        share_data.valid = false;
        std::cerr << "NO Effective Points!" << std::endl;
    }
    else
        share_data.valid = true;
    share_data.H.setZero();
    share_data.b.setZero();
    Eigen::Matrix<double, 1, 12> J;
    for (int i = 0; i < effect_feat_num; i++)
    {
        J.setZero();
        const PointType &laser_p = m_effect_cloud_lidar->points[i];
        const PointType &norm_p = m_effect_norm_vec->points[i];
        Eigen::Vector3d laser_p_vec(laser_p.x, laser_p.y, laser_p.z);
        Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);
        Eigen::Matrix<double, 1, 3> B = -norm_vec.transpose() * state.r_wi * Sophus::SO3d::hat(state.r_il * laser_p_vec + state.t_wi);
        J.block<1, 3>(0, 0) = B;
        J.block<1, 3>(0, 3) = norm_vec.transpose();
        if (m_config.esti_il)
        {
            Eigen::Matrix<double, 1, 3> C = -norm_vec.transpose() * state.r_wi * state.r_il * Sophus::SO3d::hat(laser_p_vec);
            Eigen::Matrix<double, 1, 3> D = norm_vec.transpose() * state.r_wi;
            J.block<1, 3>(0, 6) = C;
            J.block<1, 3>(0, 9) = D;
        }
        share_data.H += J.transpose() * m_config.lidar_cov_inv * J;
        share_data.b += J.transpose() * m_config.lidar_cov_inv * norm_p.intensity;
    }

    if (m_config.diagnostics_enabled)
    {
        int neighbors_ok = 0, distance_ok = 0, plane_ok = 0;
        int reject_missing = 0, reject_far = 0, reject_plane = 0, reject_score = 0;
        int horizontal = 0, vertical = 0, oblique = 0;
        int input_range_0_2 = 0, input_range_2_5 = 0, input_range_5_10 = 0, input_range_10_plus = 0;
        int effective_range_0_2 = 0, effective_range_2_5 = 0, effective_range_5_10 = 0, effective_range_10_plus = 0;
        int input_below = 0, input_middle = 0, input_above = 0;
        int effective_below = 0, effective_middle = 0, effective_above = 0;
        std::vector<double> neighbor_distances, abs_residuals, signed_residuals, scores;
        std::vector<double> input_body_z, effective_body_z;
        M3D normal_information = M3D::Zero();
        for (int i = 0; i < size; ++i)
        {
            const uint8_t reason = m_rejection_reason[i];
            const PointType &point = m_cloud_down_lidar->points[i];
            const double range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            input_body_z.push_back(point.z);
            if (range < 2.0) ++input_range_0_2;
            else if (range < 5.0) ++input_range_2_5;
            else if (range < 10.0) ++input_range_5_10;
            else ++input_range_10_plus;
            if (point.z < -0.5) ++input_below;
            else if (point.z > 0.5) ++input_above;
            else ++input_middle;
            if (reason != 1) ++neighbors_ok;
            if (reason != 1 && reason != 2) ++distance_ok;
            if (reason == 0 || reason == 4) ++plane_ok;
            if (reason == 1) ++reject_missing;
            else if (reason == 2) ++reject_far;
            else if (reason == 3) ++reject_plane;
            else if (reason == 4) ++reject_score;
            if (std::isfinite(m_kth_neighbor_distance[i]))
                neighbor_distances.push_back(m_kth_neighbor_distance[i]);
            if (reason != 0)
                continue;
            const double residual = m_plane_residual[i];
            signed_residuals.push_back(residual);
            abs_residuals.push_back(std::fabs(residual));
            scores.push_back(m_selection_score[i]);
            const V3D normal(m_norm_vec->points[i].x, m_norm_vec->points[i].y, m_norm_vec->points[i].z);
            normal_information += normal * normal.transpose();
            const double abs_nz = std::fabs(normal.z());
            if (abs_nz > 0.85) ++horizontal;
            else if (abs_nz < 0.25) ++vertical;
            else ++oblique;
            effective_body_z.push_back(point.z);
            if (range < 2.0) ++effective_range_0_2;
            else if (range < 5.0) ++effective_range_2_5;
            else if (range < 10.0) ++effective_range_5_10;
            else ++effective_range_10_plus;
            if (point.z < -0.5) ++effective_below;
            else if (point.z > 0.5) ++effective_above;
            else ++effective_middle;
        }
        const double signed_mean = signed_residuals.empty() ? std::numeric_limits<double>::quiet_NaN() :
            std::accumulate(signed_residuals.begin(), signed_residuals.end(), 0.0) / signed_residuals.size();
        const double abs_mean = abs_residuals.empty() ? std::numeric_limits<double>::quiet_NaN() :
            std::accumulate(abs_residuals.begin(), abs_residuals.end(), 0.0) / abs_residuals.size();
        double rms = 0.0;
        for (double residual : signed_residuals) rms += residual * residual;
        rms = signed_residuals.empty() ? std::numeric_limits<double>::quiet_NaN() : std::sqrt(rms / signed_residuals.size());
        const double neighbor_mean = neighbor_distances.empty() ? std::numeric_limits<double>::quiet_NaN() :
            std::accumulate(neighbor_distances.begin(), neighbor_distances.end(), 0.0) / neighbor_distances.size();
        const double score_mean = scores.empty() ? std::numeric_limits<double>::quiet_NaN() :
            std::accumulate(scores.begin(), scores.end(), 0.0) / scores.size();
        const double score_min = scores.empty() ? std::numeric_limits<double>::quiet_NaN() :
            *std::min_element(scores.begin(), scores.end());
        Eigen::SelfAdjointEigenSolver<M3D> normal_solver(normal_information);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> information_solver(share_data.H.block<6, 6>(0, 0));
        const auto normal_eigen = normal_solver.eigenvalues();
        const auto information_eigen = information_solver.eigenvalues();
        const double information_min = information_eigen.minCoeff();
        const double information_max = information_eigen.maxCoeff();
        const double information_condition = information_min > 1e-12 ? information_max / information_min :
            std::numeric_limits<double>::infinity();
        const V3D rpy = rpyDegrees(state.r_wi);
        IterationDiagnostics diagnostics;
        auto &v = diagnostics.values;
        v = {static_cast<double>(m_scan_sequence), static_cast<double>(m_current_iteration++),
             static_cast<double>(size), static_cast<double>(neighbors_ok), static_cast<double>(distance_ok),
             static_cast<double>(plane_ok), static_cast<double>(effect_feat_num), static_cast<double>(reject_missing),
             static_cast<double>(reject_far), static_cast<double>(reject_plane), static_cast<double>(reject_score),
             neighbor_mean, percentile(neighbor_distances, 0.90),
             neighbor_distances.empty() ? std::numeric_limits<double>::quiet_NaN() : *std::max_element(neighbor_distances.begin(), neighbor_distances.end()),
             signed_mean, abs_mean, rms, percentile(abs_residuals, 0.50), percentile(abs_residuals, 0.90),
             percentile(abs_residuals, 0.99),
             abs_residuals.empty() ? std::numeric_limits<double>::quiet_NaN() : *std::max_element(abs_residuals.begin(), abs_residuals.end()),
             score_mean, score_min, static_cast<double>(horizontal), static_cast<double>(vertical),
             static_cast<double>(oblique), normal_eigen(0), normal_eigen(1), normal_eigen(2),
             information_min, information_max, information_condition,
             static_cast<double>(input_range_0_2), static_cast<double>(input_range_2_5),
             static_cast<double>(input_range_5_10), static_cast<double>(input_range_10_plus),
             static_cast<double>(input_below), static_cast<double>(input_middle), static_cast<double>(input_above),
             input_body_z.empty() ? std::numeric_limits<double>::quiet_NaN() : *std::min_element(input_body_z.begin(), input_body_z.end()),
             percentile(input_body_z, 0.10), percentile(input_body_z, 0.50), percentile(input_body_z, 0.90),
             input_body_z.empty() ? std::numeric_limits<double>::quiet_NaN() : *std::max_element(input_body_z.begin(), input_body_z.end()),
             static_cast<double>(effective_range_0_2), static_cast<double>(effective_range_2_5),
             static_cast<double>(effective_range_5_10), static_cast<double>(effective_range_10_plus),
             static_cast<double>(effective_below), static_cast<double>(effective_middle), static_cast<double>(effective_above),
             effective_body_z.empty() ? std::numeric_limits<double>::quiet_NaN() : *std::min_element(effective_body_z.begin(), effective_body_z.end()),
             percentile(effective_body_z, 0.10), percentile(effective_body_z, 0.50), percentile(effective_body_z, 0.90),
             effective_body_z.empty() ? std::numeric_limits<double>::quiet_NaN() : *std::max_element(effective_body_z.begin(), effective_body_z.end()),
             state.t_wi.x(), state.t_wi.y(), state.t_wi.z(), rpy.x(), rpy.y(), rpy.z(),
             state.v.x(), state.v.y(), state.v.z()};
        m_iteration_diagnostics.push_back(std::move(diagnostics));
    }
}

CloudType::Ptr LidarProcessor::transformCloud(CloudType::Ptr inp, const M3D &r, const V3D &t)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = r.cast<float>();
    transform.block<3, 1>(0, 3) = t.cast<float>();
    CloudType::Ptr ret(new CloudType);
    pcl::transformPointCloud(*inp, *ret, transform);
    return ret;
}
