#pragma once
#include "commons.h"
#include "ieskf.h"
#include "ikd_Tree.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <cstdint>
#include <string>

struct IterationDiagnostics
{
    std::vector<double> values;
};

struct ScanDiagnostics
{
    std::vector<double> values;
};

struct LocalMap
{
    bool initialed = false;
    BoxPointType local_map_corner;
    Vec<BoxPointType> cub_to_rm;
};

class LidarProcessor
{
public:
    LidarProcessor(Config &config, std::shared_ptr<IESKF> kf);

    void trimCloudMap();

    void incrCloudMap();

    void initCloudMap(PointVec &point_vec);

    void process(SyncPackage &package);

    void updateLossFunc(State &state, SharedState &share_data);

    static CloudType::Ptr transformCloud(CloudType::Ptr inp, const M3D &r, const V3D &t);
    M3D r_wl() { return m_kf->x().r_wi * m_kf->x().r_il; }
    V3D t_wl() { return m_kf->x().t_wi + m_kf->x().r_wi * m_kf->x().t_il; }

    const ScanDiagnostics &scanDiagnostics() const { return m_scan_diagnostics; }
    const std::vector<IterationDiagnostics> &iterationDiagnostics() const { return m_iteration_diagnostics; }
    CloudType::Ptr diagnosticCorrespondences() const { return m_diagnostic_correspondences; }
    bool diagnosticsEnabled() const { return m_config.diagnostics_enabled; }

    static const std::string &scanDiagnosticsSchema();
    static const std::string &iterationDiagnosticsSchema();

private:
    Config m_config;
    LocalMap m_local_map;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<KD_TREE<PointType>> m_ikdtree;
    CloudType::Ptr m_cloud_lidar;
    CloudType::Ptr m_cloud_down_lidar;
    CloudType::Ptr m_cloud_down_world;
    std::vector<bool> m_point_selected_flag;
    CloudType::Ptr m_norm_vec;
    CloudType::Ptr m_effect_cloud_lidar;
    CloudType::Ptr m_effect_norm_vec;
    std::vector<PointVec> m_nearest_points;
    pcl::VoxelGrid<PointType> m_scan_filter;

    uint64_t m_scan_sequence = 0;
    size_t m_current_iteration = 0;
    ScanDiagnostics m_scan_diagnostics;
    std::vector<IterationDiagnostics> m_iteration_diagnostics;
    CloudType::Ptr m_diagnostic_correspondences;
    std::vector<uint8_t> m_rejection_reason;
    std::vector<double> m_kth_neighbor_distance;
    std::vector<double> m_plane_residual;
    std::vector<double> m_selection_score;
    int m_map_boxes_removed = 0;
    int m_map_points_requested = 0;
    int m_map_points_inserted = 0;
};
