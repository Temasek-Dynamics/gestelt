#include <sfc_generation/spherical_sfc.h>

SphericalSFC::SphericalSFC(std::shared_ptr<GridMap> grid_map, const SphericalSFCParams& sfc_params):
    grid_map_(grid_map), sfc_params_(sfc_params)
{}   


void SphericalSFC::addVizPublishers(ros::Publisher& p_cand_viz_pub, 
    ros::Publisher& dist_viz_pub, ros::Publisher& sfc_spherical_viz_pub,
    ros::Publisher&  sfc_waypoints_viz_pub)
{
    p_cand_viz_pub_ = p_cand_viz_pub;
    dist_viz_pub_ = dist_viz_pub;
    sfc_spherical_viz_pub_ = sfc_spherical_viz_pub;
    sfc_waypoints_viz_pub_ = sfc_waypoints_viz_pub;
}

bool SphericalSFC::generateSFC(const std::vector<Eigen::Vector3d> &path)
{
    Sphere B_cur; // current sphere being considered

    size_t path_idx_cur = 0; // Current index of guide path

    // Initialize largest sphere at initial position
    if (!generateFreeSphere(path[path_idx_cur], B_cur)){
        return false;
    }

    sfc_spheres_.push_back(B_cur);
    
    itr_ = 0;
    while (itr_ < sfc_params_.max_itr)
    {
        if (!getForwardPointOnPath(path, path_idx_cur, B_cur)){
            return false;
        }
        
        /* For debugging */
        // generateFreeSphere(path[path_idx_cur], B_cur);
        // sfc_spheres_.push_back(B_cur);
        /* end */

        if (!BatchSample(path[path_idx_cur], B_cur)){
            return false;
        }
        
        sfc_spheres_.push_back(B_cur);
        if (B_cur.contains(path.back())){ // If current sphere contains the goal
            std::cout << "SFC corridor to goal generated!" << std::endl;
            break;
        }

        itr_++;
    }

    publishVizSphericalSFC(sfc_spheres_, sfc_spherical_viz_pub_, "world");

    if (!sfc_spheres_.back().contains(path.back())){ 
        std::cout << "Final safe flight corridor does not contain the goal" << std::endl;

        if (itr_ > sfc_params_.max_itr){
            std::cout << "[SphericalSFC] Maximum iterations " << sfc_params_.max_itr 
                    << " exceeded. Consumed " << itr_ << " iterations." << std::endl;
        }

        return false;
    }

    std::vector<Eigen::Vector3d> traj_waypoints = initializeWaypointsAndTimeAllocation(sfc_spheres_);
    publishVizPiecewiseTrajectory(traj_waypoints, sfc_waypoints_viz_pub_);

    return true;
}   

bool SphericalSFC::generateFreeSphere(const Eigen::Vector3d& point, Sphere& B)
{
    Eigen::Vector3d occ_nearest;
    double radius;

    if (grid_map_->getNearestOccupiedCell(point, occ_nearest, radius)){
        B.center = point;
        B.setRadius(radius);
        // std::cout << "point: " << point << std::endl;
        // std::cout << "Occ_nearest: " << occ_nearest << std::endl;
        // std::cout << "Sphere radius: " << sphere.radius << std::endl;

        return true;
    }
    return false;
        
}

bool SphericalSFC::getForwardPointOnPath(
    const std::vector<Eigen::Vector3d> &path, size_t& start_idx, const Sphere& B)
{
    for (size_t i = start_idx; i < path.size(); i++){
        // Iterate forward through the guide path to find a point outside the sphere 
        if (!B.contains(path[i])){
            std::cout << "Found point outside sphere: " << path[start_idx] << std::endl; 
            start_idx = i;
            return true;
        }
    }
    
    std::cout << "Did not find point outside sphere: " << std::endl; 
    return false;
}

bool SphericalSFC::BatchSample(const Eigen::Vector3d& p_guide, Sphere& B_cur)
{
    B_cand_pq_ = std::priority_queue<std::shared_ptr<Sphere>, std::vector<std::shared_ptr<Sphere>>, Sphere::CompareScorePtr>();

    /* Debugging */
    std::vector<Eigen::Vector3d> p_cand_vec;

    // Calculate orientation and standard deviation of normal sampling distribution
    Eigen::Vector3d dir_vec = (p_guide - B_cur.center);
    std::cout << "Dir vec: " << dir_vec << std::endl;
    // TODO: Should we take the standard dev of the normalized or unnormalized vector?
    // Get standard deviation along direction vector
    double stddev_x = sfc_params_.mult_stddev_x * dir_vec.norm();
    Eigen::Vector3d stddev{stddev_x, 2*stddev_x, 2*stddev_x};

    // Calculate orientation of distribution ellipsoid
    Eigen::Matrix<double, 3, 3> ellipse_rot_mat = rotationAlign(Eigen::Vector3d::UnitX(), dir_vec.normalized());
    Eigen::Quaterniond ellipse_orientation(ellipse_rot_mat);

    // Set up seed for sampler
    uint64_t seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    sampler_.setSeed(seed);
    sampler_.setParams(p_guide, stddev);
    
    // Sample the points first then rotate them
    for (int k = 0; k < sfc_params_.max_sample_points; k++){
        /* Debugging */
        p_cand_vec.push_back(sampler_.sample());
    }

    transformPoints(p_cand_vec, p_guide, ellipse_rot_mat);

    for (auto& p_cand: p_cand_vec){
        std::shared_ptr<Sphere> B_cand = std::make_shared<Sphere>();

        // Generate candidate sphere, calculate score and add to priority queue
        generateFreeSphere(p_cand, *B_cand);
        B_cand->score = computeCandSphereScore(*B_cand, B_cur);

        B_cand_pq_.push(B_cand);
    }

    // Publish candidate points and 3d distribution visualization
    publishVizPoints(p_cand_vec, p_cand_viz_pub_); 
    dist_viz_pub_.publish( createVizEllipsoid(p_guide, stddev, ellipse_orientation, "world", itr_));

    if (!B_cand_pq_.empty()){
        std::cout << "Assigned next candidate sphere with volume " << (*B_cand_pq_.top()).getVolume() << 
            " and intersecting volume " << getIntersectingVolume(*B_cand_pq_.top(), B_cur ) << std::endl;
        B_cur = *B_cand_pq_.top();
        return true;
    } 

    return false;
}

void SphericalSFC::transformPoints(std::vector<Eigen::Vector3d>& pts, Eigen::Vector3d origin, const Eigen::Matrix<double, 3, 3>& rot_mat)
{
    for (auto& pt : pts){
        pt = (rot_mat * (pt - origin)) + origin;
    }
}

double SphericalSFC::computeCandSphereScore(Sphere& B_cand, Sphere& B_prev)
{   
    double V_cand = B_cand.getVolume();
    double V_intersect = getIntersectingVolume(B_cand, B_prev);

    return sfc_params_.W_cand_vol * V_cand + sfc_params_.W_intersect_vol * V_intersect;
}

double SphericalSFC::getIntersectingVolume(Sphere& B_a, Sphere& B_b)
{
    
    double d = (B_a.center - B_b.center).norm();
    if (d >= (B_a.radius + B_b.radius)){ // Non-intersection
        return -999999;
    }
    double vol_intersect = (1/12) * M_PI * (4* B_a.radius + d) * (2 * B_a.radius - d) * (2 * B_a.radius - d);

    return vol_intersect;
}

Eigen::Vector3d SphericalSFC::getIntersectionCenter(const Sphere& B_a, const Sphere& B_b)
{   
    Eigen::Vector3d dir_vec = B_b.center - B_a.center; 
    double d_centroids = (dir_vec).norm(); // scalar distance between spheres B_a and B_b 
    // Get distance to intersection along direction vector dir_vec from center of B_a to center of B_b
    double d_intersect = (pow(B_a.radius,2) + pow(d_centroids,2) - pow(B_b.radius,2) ) / (2*d_centroids);

    Eigen::Vector3d pt_intersect = B_a.center + (d_intersect/d_centroids) * dir_vec;

    return pt_intersect;
}

// TODO: Time allocation not implemented yet
std::vector<Eigen::Vector3d> SphericalSFC::initializeWaypointsAndTimeAllocation(const std::vector<SphericalSFC::Sphere>& sfc_spheres)
{
    std::vector<Eigen::Vector3d> traj_waypoints;

    traj_waypoints.push_back(sfc_spheres[0].center);

    for(int i = 1; i < sfc_spheres.size(); i++){
        // If the center of the previous sphere is not inside current sphere,
        // We add their intersection into the waypoint
        if (!sfc_spheres[i].contains(sfc_spheres[i-1].center)){
            traj_waypoints.push_back(getIntersectionCenter(sfc_spheres[i-1], sfc_spheres[i]) );
        }
        traj_waypoints.push_back(sfc_spheres[i].center);
    }
    // traj_waypoints.push_back(sfc_spheres.back().center);

    return traj_waypoints;
}

// Get rotation that aligns z to d
Eigen::Matrix<double, 3, 3> SphericalSFC::rotationAlign(const Eigen::Vector3d & z, const Eigen::Vector3d & d)
{
    // TODO: IF either vector is a unit vector, possibility to speed this up

    // The code below has been taken from https://iquilezles.org/articles/noacos/, where it explains
    // how to more efficiently compute a rotation matrix aligning vector z to d.
    const Eigen::Vector3d v = z.cross(d);
    const double c = z.dot(d);
    const double k = 1.0f/(1.0f+c);

    Eigen::Matrix<double, 3, 3> rot_mat; 
    rot_mat <<  v(0)*v(0)*k + c,     v(1)*v(0)*k - v(2),    v(2)*v(0)*k + v(1),
                v(0)*v(1)*k + v(2),   v(1)*v(1)*k + c,      v(2)*v(1)*k - v(0),
                v(0)*v(2)*k - v(1),   v(1)*v(2)*k + v(0),    v(2)*v(2)*k + c ;

    return rot_mat;

}

void SphericalSFC::publishVizPiecewiseTrajectory(const std::vector<Eigen::Vector3d>& pts, ros::Publisher& publisher, const std::string& frame_id)
{
    visualization_msgs::Marker sphere_list, path_line_strip;
    Eigen::Vector3d wp_color = Eigen::Vector3d{1.0, 0.0, 0.0};
    double wp_alpha = 0.7;
    double wp_radius = 0.2;

    Eigen::Vector3d line_color = Eigen::Vector3d{1.0, 0.0, 0.0};
    double line_alpha = 0.7;
    double line_scale = 0.2;

    // sphere_list.action = visualization_msgs::Marker::DELETEALL;
    // path_line_strip.action = visualization_msgs::Marker::DELETEALL;
    // publisher.publish(sphere_list);
    // publisher.publish(path_line_strip);

    /* Path waypoint spheres */
    sphere_list.header.frame_id = frame_id;
    sphere_list.header.stamp = ros::Time::now();
    sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;
    sphere_list.action = visualization_msgs::Marker::ADD;
    sphere_list.ns = "spherical_sfc_pts"; 
    sphere_list.id = 1; 
    sphere_list.pose.orientation.w = 1.0;

    sphere_list.color.r = wp_color(0);
    sphere_list.color.g = wp_color(1);
    sphere_list.color.b = wp_color(2);
    sphere_list.color.a = wp_alpha;

    sphere_list.scale.x = wp_radius;
    sphere_list.scale.y = wp_radius;
    sphere_list.scale.z = wp_radius;
    
    /* Path line strips */
    path_line_strip.header.frame_id = frame_id;
    path_line_strip.header.stamp = ros::Time::now();
    path_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    path_line_strip.action = visualization_msgs::Marker::ADD;
    path_line_strip.ns = "spherical_sfc_line"; 
    path_line_strip.id = 2;
    path_line_strip.pose.orientation.w = 1.0;

    path_line_strip.color.r = line_color(0);
    path_line_strip.color.g = line_color(1);
    path_line_strip.color.b = line_color(2);
    path_line_strip.color.a = line_alpha;

    path_line_strip.scale.x = line_scale;

    geometry_msgs::Point pt;
    for (int i = 0; i < pts.size(); i++){
        pt.x = pts[i](0);
        pt.y = pts[i](1);
        pt.z = pts[i](2);

        sphere_list.points.push_back(pt);
        path_line_strip.points.push_back(pt);
    }

    publisher.publish(sphere_list);
    publisher.publish(path_line_strip);
}

void SphericalSFC::publishVizPoints(const std::vector<Eigen::Vector3d>& pts, ros::Publisher& publisher, Eigen::Vector3d color, double radius, const std::string& frame_id)
{
  visualization_msgs::Marker sphere_list;
  double alpha = 0.7;

  sphere_list.header.frame_id = frame_id;
  sphere_list.header.stamp = ros::Time::now();
  sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;
  sphere_list.action = visualization_msgs::Marker::ADD;
  sphere_list.ns = "spherical_sfc_pts"; 
  sphere_list.id = 1; 
  sphere_list.pose.orientation.w = 1.0;

  sphere_list.color.r = color(0);
  sphere_list.color.g = color(1);
  sphere_list.color.b = color(2);
  sphere_list.color.a = alpha;

  sphere_list.scale.x = radius;
  sphere_list.scale.y = radius;
  sphere_list.scale.z = radius;

  geometry_msgs::Point pt;
  for (int i = 0; i < pts.size(); i++){
    pt.x = pts[i](0);
    pt.y = pts[i](1);
    pt.z = pts[i](2);

    sphere_list.points.push_back(pt);
  }

  publisher.publish(sphere_list);
}

visualization_msgs::Marker SphericalSFC::createVizEllipsoid(const Eigen::Vector3d& center, const Eigen::Vector3d& stddev, const Eigen::Quaterniond& orientation, const std::string& frame_id, const int& id)
{
    visualization_msgs::Marker sphere;

    sphere.header.frame_id = frame_id;
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    sphere.pose.orientation.x = orientation.x();
    sphere.pose.orientation.y = orientation.y();
    sphere.pose.orientation.z = orientation.z();
    sphere.pose.orientation.w = orientation.w();

    sphere.color.r = 1.0;
    sphere.color.g = 1.0;
    sphere.color.b = 0.0;
    sphere.color.a = 0.5;
    sphere.scale.x = stddev(0);
    sphere.scale.y = stddev(1);
    sphere.scale.z = stddev(2);

    sphere.pose.position.x = center(0);
    sphere.pose.position.y = center(1);
    sphere.pose.position.z = center(2);

    return sphere;
}