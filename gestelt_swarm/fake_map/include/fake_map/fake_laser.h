#ifndef FAKE_LASER_H
#define FAKE_LASER_H

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <cmath>
#include <math.h>

class FakeLaser
{
    private:
    
        Eigen::MatrixXi idx_map;
        Eigen::MatrixXd dis_map;

        pcl::PointCloud<pcl::PointXYZ> _global_map;

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_local_map;

        int _hrz_laser_line_num;
        int _vtc_laser_line_num;
        double _resolution;
        double _sensing_range;
        double _hrz_resolution_rad;
        double _vtc_resolution_rad;
        double _vtc_laser_range_rad;
        double _half_vtc_resolution_and_half_range;
        double _half_hrz_range = M_PI;

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
        inline double line_intersect_plane(Eigen::Vector3d &intersection,
            const Eigen::Vector3d &line_p, const Eigen::Vector3d &line_dir,
            const Eigen::Vector3d &plane_p, const Eigen::Vector3d &plane_normal);

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
        inline bool pt_to_idx(Eigen::Vector2i &idx,
            const Eigen::Vector3d &pt,
            const Eigen::Vector3d &laser_t, 
            const Eigen::Matrix3d &laser_R);

        /**
         * @brief
         *
         * @param x pixel x value
         * @param y pixel y value
         * @param dis distance
         * @param pt in laser coordinate.
         */
        inline void idx_to_pt(
            int x, int y, 
            double dis, Eigen::Vector3d &pt);

    public:

        FakeLaser(){};
        ~FakeLaser(){};

        void set_parameters(double resolution,
            double sensing_range,
            const pcl::PointCloud<pcl::PointXYZ>& global_map,
            double vtc_laser_range_dgr,
            double hrz_laser_range_dgr,
            double vtc_laser_line_num,
            double hrz_laser_line_num);

        void render_sensed_points(
            Eigen::Vector3d trans, Eigen::Matrix3d rot,
            pcl::PointCloud<pcl::PointXYZ> &local_map);
};

#endif //FAKE_LASER_H