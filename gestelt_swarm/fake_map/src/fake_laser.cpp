/**
 * @file fake_laser.cpp
 * @author ZheJiang University 
 * @brief Originally from ZJU FAST Lab, with very minor modifications by JohnTGZ
 * @version 0.1
 * @date 2024-05-14
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <fake_map/fake_laser.h>

/**
 * @brief
 * Given a line and a plane, calculating for the intersection and the distance.
 * line_dir is required to be a normalized vector.
 * @param intersection the intersecting point.
 * @param line_p A point in the line.
 * @param line_dir The line direction vector.
 * @param plane_p A point in the plane.
 * @param plane_normal The plane normal vector.
 * @return double
 * The distance between the query point and the intersection.
 * A negtive value means the line direction vector points away from the plane.
 */
inline double FakeLaser::line_intersect_plane(Eigen::Vector3d &intersection,
    const Eigen::Vector3d &line_p, const Eigen::Vector3d &line_dir,
    const Eigen::Vector3d &plane_p, const Eigen::Vector3d &plane_normal)
{
    double d = (plane_p - line_p).dot(plane_normal) / line_dir.dot(plane_normal);
    intersection = line_p + d * line_dir;
    return d;
}

/**
 * @brief
 * filter the points not in range
 * @param idx
 * @param pt
 * @param laser_t
 * @param laser_R must be normalized in each column
 * @return true
 * @return false
 */
inline bool FakeLaser::pt_to_idx(Eigen::Vector2i &idx,
    const Eigen::Vector3d &pt,
    const Eigen::Vector3d &laser_t, const Eigen::Matrix3d &laser_R)
{
    Eigen::Vector3d inter_p;
    double dis_pt_to_laser_plane = line_intersect_plane(inter_p, pt, laser_R.col(2), laser_t, laser_R.col(2));
    double dis_laser_to_inter_p = (inter_p - laser_t).norm();
    double vtc_rad = std::atan2((pt - laser_t).dot(laser_R.col(2)), dis_laser_to_inter_p);
    if (std::fabs(vtc_rad) >= _half_vtc_resolution_and_half_range)
        return false;

    double x_in_roll_pitch_plane = (inter_p - laser_t).dot(laser_R.col(0));
    double y_in_roll_pitch_plane = (inter_p - laser_t).dot(laser_R.col(1));
    double hrz_rad = std::atan2(y_in_roll_pitch_plane, x_in_roll_pitch_plane);
    if (std::fabs(hrz_rad) >= _half_hrz_range)
        return false;

    vtc_rad += _half_vtc_resolution_and_half_range;
    int vtc_idx = std::floor(vtc_rad / _vtc_resolution_rad);
    if (vtc_idx >= _vtc_laser_line_num)
        vtc_idx = 0;

    hrz_rad += M_PI + _hrz_resolution_rad / 2.0;
    int hrz_idx = std::floor(hrz_rad / _hrz_resolution_rad);
    if (hrz_idx >= _hrz_laser_line_num)
        hrz_idx = 0;

    idx << hrz_idx, vtc_idx;
    return true;
}

/**
 * @brief
 *
 * @param x pixel x value
 * @param y pixel y value
 * @param dis distance
 * @param pt in laser coordinate.
 */
inline void FakeLaser::idx_to_pt(int x, int y, double dis, Eigen::Vector3d &pt)
{
    double vtc_rad = y * _vtc_resolution_rad - _vtc_laser_range_rad / 2.0;
    double hrz_rad = x * _hrz_resolution_rad - M_PI;
    pt[2] = sin(vtc_rad) * dis; // z_in_laser_coor
    double xy_square_in_laser_coor = cos(vtc_rad) * dis;
    pt[0] = cos(hrz_rad) * xy_square_in_laser_coor; // x_in_laser_coor
    pt[1] = sin(hrz_rad) * xy_square_in_laser_coor; // y_in_laser_coor
}

void FakeLaser::set_parameters(
    double resolution,
    double sensing_range,
    const pcl::PointCloud<pcl::PointXYZ>& global_map,
    double vtc_laser_range_dgr,
    double hrz_laser_range_dgr,
    double vtc_laser_line_num,
    double hrz_laser_line_num)
{
    _hrz_laser_line_num = hrz_laser_line_num;
    _vtc_laser_line_num = vtc_laser_line_num;

    _vtc_laser_range_rad = vtc_laser_range_dgr / 180.0 * M_PI;
    _vtc_resolution_rad = _vtc_laser_range_rad / (double)(_vtc_laser_line_num - 1);
    _half_vtc_resolution_and_half_range = (_vtc_laser_range_rad + _vtc_resolution_rad) / 2.0;

    _half_hrz_range = hrz_laser_range_dgr / 180.0 * M_PI / 2.0;
    _hrz_resolution_rad = 2 * M_PI / (double)_hrz_laser_line_num;
    
    _sensing_range = sensing_range;
    _resolution = resolution;

    idx_map = Eigen::MatrixXi::Constant(_hrz_laser_line_num, _vtc_laser_line_num, -1);
    dis_map = Eigen::MatrixXd::Constant(_hrz_laser_line_num, _vtc_laser_line_num, 9999.0);
    pcl::copyPointCloud(global_map, _global_map);
    kdtree_local_map.setInputCloud(_global_map.makeShared());
}

void FakeLaser::render_sensed_points(
    Eigen::Vector3d trans, Eigen::Matrix3d rot,
    pcl::PointCloud<pcl::PointXYZ> &local_map)
{
    std::vector<int> point_idx_radius_search;
    std::vector<float> point_radius_squared_distance;

    pcl::PointXYZ searchPoint(
        trans.x(), trans.y(), trans.z());
    kdtree_local_map.radiusSearch(
        searchPoint, _sensing_range, point_idx_radius_search, 
        point_radius_squared_distance);

    Eigen::Affine3d world_to_camera = 
        Eigen::Affine3d::Identity();
    world_to_camera.translation() = trans;
    world_to_camera.linear() = rot;

    idx_map.setConstant(-1);
    dis_map.setConstant(9999.0);

    pcl::PointXYZ pt;
    for (size_t i = 0; i < point_idx_radius_search.size(); ++i)
    {
        pt = _global_map.points[point_idx_radius_search[i]];

        Eigen::Vector2i idx;
        bool in_range = pt_to_idx(
            idx, Eigen::Vector3d(pt.x, pt.y, pt.z), 
            trans, rot);
        if (!in_range)
            continue;
        
        Eigen::Vector3d pt_vec = Eigen::Vector3d(
            pt.x - trans.x(),
            pt.y - trans.y(),
            pt.z - trans.z());
        double dis_curr_pt = pt_vec.norm();

        double vtc_rad = idx[1] * _vtc_resolution_rad - _vtc_laser_range_rad / 2.0;
        double dis_to_z_axis = dis_curr_pt * std::cos(vtc_rad);
        double mesh_len_hrz = dis_to_z_axis * _hrz_resolution_rad;
        double mesh_len_vtc = dis_to_z_axis * _vtc_resolution_rad;
        int hrz_occ_grid_num = std::min((int)std::floor(_resolution / mesh_len_hrz), _hrz_laser_line_num);
        int vtc_occ_grid_num = std::min((int)std::floor(_resolution / mesh_len_vtc), _vtc_laser_line_num);
        // ROS_INFO_STREAM("hrz_occ_grid_num " << hrz_occ_grid_num / 2 << ", vtc_occ_grid_num " << vtc_occ_grid_num / 2);
        int tmp1 = hrz_occ_grid_num, tmp2 = vtc_occ_grid_num;
        for (int d_hrz_idx = -tmp1; d_hrz_idx <= tmp1; ++d_hrz_idx)
            for (int d_vtc_idx = -tmp2; d_vtc_idx <= tmp2; ++d_vtc_idx)
            {
                // it's a ring in hrz coordiante
                int hrz_idx = (idx[0] + d_hrz_idx + _hrz_laser_line_num) % _hrz_laser_line_num; 
                int vtc_idx = idx[1] + d_vtc_idx;
                if (vtc_idx >= _vtc_laser_line_num)
                    continue;

                if (vtc_idx < 0)
                    continue;

                if (dis_curr_pt < dis_map(hrz_idx, vtc_idx))
                {
                    idx_map(hrz_idx, vtc_idx) = i;
                    dis_map(hrz_idx, vtc_idx) = dis_curr_pt;
                }
            }
    }

    local_map.points.clear();
    for (int x = 0; x < _hrz_laser_line_num; ++x)
        for (int y = 0; y < _vtc_laser_line_num; ++y)
        {
            // use laser line pts
            Eigen::Vector3d p;
            if (idx_map(x, y) != -1)
            {
                idx_to_pt(x, y, dis_map(x, y), p);
                Eigen::Affine3d camera_to_point = 
                    Eigen::Affine3d::Identity();
                camera_to_point.translation() = p;

                Eigen::Vector3d world_point = 
                    (world_to_camera * camera_to_point).translation();

                local_map.points.emplace_back(
                    world_point.x(), world_point.y(), world_point.z());
            }
        }

    local_map.width = local_map.points.size();
    local_map.height = 1;
    local_map.is_dense = true;

}
