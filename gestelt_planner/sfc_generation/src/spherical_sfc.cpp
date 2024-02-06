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
    double get_fwd_pt_durs{0};
    double batch_sample_durs{0};

    auto a = std::chrono::high_resolution_clock::now();

    Sphere B_cur; // current sphere being considered

    size_t path_idx_cur = 0; // Current index of guide path

    // Initialize largest sphere at initial position
    if (!generateFreeSphere(path[0], B_cur)){
        std::cout << "Failed to generate free sphere centered on start point" << std::endl;
        return false;
    }

    sfc_spheres_.push_back(B_cur);
    
    auto b = std::chrono::high_resolution_clock::now();

    itr_ = 0;
    while (itr_ < sfc_params_.max_itr)
    {
        auto get_fwd_pt = std::chrono::high_resolution_clock::now();

        if (!getForwardPointOnPath(path, path_idx_cur, B_cur)){
            std::cout << "getForwardPointOnPath: Failed to get forward point on the path" << std::endl;
            return false;
        }

        auto get_fwd_pt_end = std::chrono::high_resolution_clock::now();

        auto batch_sample = std::chrono::high_resolution_clock::now();

        if (!BatchSample(path[path_idx_cur], B_cur)){
            std::cout << "Batch sample failed" << std::endl;
            return false;
        }

        auto batch_sample_end = std::chrono::high_resolution_clock::now();
        
        sfc_spheres_.push_back(B_cur);

        if (B_cur.contains(path.back())){ // If current sphere contains the goal
            break;
        }

        get_fwd_pt_durs += std::chrono::duration_cast<std::chrono::duration<double>>(
            get_fwd_pt_end - get_fwd_pt).count();

        batch_sample_durs += std::chrono::duration_cast<std::chrono::duration<double>>(
            batch_sample_end - batch_sample).count();

        itr_++;
    }

    auto c = std::chrono::high_resolution_clock::now();

    publishVizSphericalSFC(sfc_spheres_, sfc_spherical_viz_pub_, "world");

    computeSFCTrajectory(sfc_spheres_, path.back(), sfc_traj_);
    publishVizPiecewiseTrajectory(sfc_traj_.waypoints, sfc_waypoints_viz_pub_);

    auto d = std::chrono::high_resolution_clock::now();

    if (!sfc_spheres_.back().contains(path.back())){ 
        std::cout << "[SphericalSFC] Final safe flight corridor does not contain the goal" << std::endl;

        if (itr_ > sfc_params_.max_itr){
            std::cout << "[SphericalSFC] Maximum iterations " << sfc_params_.max_itr 
                    << " exceeded. Consumed " << itr_ << " iterations." << std::endl;
        }

        return false;
    }

    auto e = std::chrono::high_resolution_clock::now();

    auto total_loop_dur = std::chrono::duration_cast<std::chrono::duration<double>>(
        e - a).count();

    auto preloop_dur = std::chrono::duration_cast<std::chrono::duration<double>>(
        b - a).count();

    auto loop_dur = std::chrono::duration_cast<std::chrono::duration<double>>(
        c - b).count();

    auto pub_dur = std::chrono::duration_cast<std::chrono::duration<double>>(
        d - c).count();

    std::cout << "Spherical SFC runtimes [ms]: " << std::endl;

    std::cout << "  Total dur: " << total_loop_dur *1000   << std::endl;
    std::cout << "  Preloop dur: " << preloop_dur *1000    << ", pct:" << preloop_dur / total_loop_dur * 100 << "%" << std::endl;
    std::cout << "  loop dur: " << loop_dur  *1000         << ", pct:" << loop_dur / total_loop_dur * 100<< "%" << std::endl;
    std::cout << "      get_fwd_pt_durs: " << get_fwd_pt_durs  *1000         << ", pct:" << get_fwd_pt_durs / total_loop_dur * 100<< "%" << std::endl;
    std::cout << "      batch_sample_durs: " << batch_sample_durs  *1000     << ", pct:" << batch_sample_durs / total_loop_dur * 100<< "%" << std::endl;

    std::cout << "  publish dur: " << pub_dur  *1000       << "s, pct:" << pub_dur / total_loop_dur * 100<< "%" << std::endl;

    return true;
}   

bool SphericalSFC::generateFreeSphere(const Eigen::Vector3d& point, Sphere& B)
{
    Eigen::Vector3d occ_nearest;
    double radius;

    if (grid_map_->getNearestOccupiedCell(point, occ_nearest, radius)){
        B.center = point;
        B.setRadius(radius);
        return true;
    }
    return false;
}

bool SphericalSFC::getForwardPointOnPath(
    const std::vector<Eigen::Vector3d> &path, size_t& start_idx, const Sphere& B_prev)
{
    for (size_t i = start_idx; i < path.size(); i++){
        // Iterate forward through the guide path to find a point outside the sphere 
        if (!B_prev.contains(path[i])){
            // std::cout << "Found point outside sphere: " << path[start_idx] << std::endl; 
            start_idx = i;
            return true;
        }
    }
    
    // std::cout << "Did not find point outside sphere: " << std::endl; 
    return false;
}

bool SphericalSFC::BatchSample(const Eigen::Vector3d& p_guide, Sphere& B_cur)
{
    auto a = std::chrono::high_resolution_clock::now();

    // Priority queue of candidate spheres sorted by highest score first
    std::priority_queue<std::shared_ptr<Sphere>, std::vector<std::shared_ptr<Sphere>>, Sphere::CompareScorePtr> B_cand_pq = 
        std::priority_queue<std::shared_ptr<Sphere>, std::vector<std::shared_ptr<Sphere>>, Sphere::CompareScorePtr>();

    std::vector<Eigen::Vector3d> p_cand_vec; // vector of candidate points

    // Calculate orientation and standard deviation of normal sampling distribution
    Eigen::Vector3d dir_vec = (p_guide - B_cur.center); // Direction vector from previous sphere to p_guide
    // std::cout << "Dir vec: " << dir_vec << std::endl;
    double stddev_x = sfc_params_.mult_stddev_x * dir_vec.norm(); // Get standard deviation along direction vector
    double stddev_y = sfc_params_.mult_stddev_y * dir_vec.norm(); // Get standard deviation along direction vector
    double stddev_z = sfc_params_.mult_stddev_z * dir_vec.norm(); // Get standard deviation along direction vector
    Eigen::Vector3d stddev{stddev_x, stddev_y, stddev_z};

    // Calculate orientation of distribution ellipsoid
    Eigen::Matrix<double, 3, 3> ellipse_rot_mat = rotationAlign(Eigen::Vector3d::UnitX(), dir_vec.normalized());
    Eigen::Quaterniond ellipse_orientation(ellipse_rot_mat);

    // Set up seed for sampler
    uint64_t seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    sampler_.setSeed(seed);
    sampler_.setParams(p_guide, stddev);
    
    auto b = std::chrono::high_resolution_clock::now();

    // Sample the points first then rotate them
    for (int k = 0; k < sfc_params_.max_sample_points; k++){
        /* Debugging */
        p_cand_vec.push_back(sampler_.sample());
    }

    auto c = std::chrono::high_resolution_clock::now();

    // Transform set of sampled points to have its mean at p_guide and rotated along dir_vec
    // TODO: Use matrix parallelization for transformations
    transformPoints(p_cand_vec, p_guide, ellipse_rot_mat);

    auto d = std::chrono::high_resolution_clock::now();

    double d2_dur{0};
    double d3_dur{0};

    for (auto& p_cand: p_cand_vec){
        auto d1 = std::chrono::high_resolution_clock::now();

        std::shared_ptr<Sphere> B_cand = std::make_shared<Sphere>();

        // Generate candidate sphere, calculate score and add to priority queue
        generateFreeSphere(p_cand, *B_cand);

        auto d2 = std::chrono::high_resolution_clock::now();

        B_cand->score = computeCandSphereScore(*B_cand, B_cur); 

        if (B_cand->score > 0){ // If score is positive, add sphere to priority queue
            B_cand_pq.push(B_cand); 
        }

        auto d3 = std::chrono::high_resolution_clock::now();
        d2_dur += std::chrono::duration_cast<std::chrono::duration<double>>(
            d2 - d1).count() * 1000.0;
        d3_dur += std::chrono::duration_cast<std::chrono::duration<double>>(
            d3 - d2).count() * 1000.0;
    }

    auto e = std::chrono::high_resolution_clock::now();

    // Publish candidate points and 3d distribution visualization
    if (sfc_params_.debug_viz){
        publishVizPoints(p_cand_vec, p_cand_viz_pub_); 
        dist_viz_pub_.publish( createVizEllipsoid(p_guide, stddev, ellipse_orientation, "world", itr_));
    }

    auto f = std::chrono::high_resolution_clock::now();

    if (B_cand_pq.empty()){
        std::cout << "Unable to generate next candidate sphere"<< std::endl;
        return false;
    }

    // std::cout << "Assigned next candidate sphere of radius " << B_cand_pq.top()->radius 
    //             << ", with volume " << B_cand_pq.top()->getVolume() 
    //             << ", intersecting volume " << getIntersectingVolume(*B_cand_pq.top(), B_cur ) 
    //             << ", Center " << B_cand_pq.top()->center
    //             << std::endl;

    B_cur = Sphere(B_cand_pq.top());

    double chpt_b = std::chrono::duration_cast<std::chrono::duration<double>>(
        b - a).count() * 1000.0;
    double chpt_c = std::chrono::duration_cast<std::chrono::duration<double>>(
        c - b).count() * 1000.0;
    double chpt_d = std::chrono::duration_cast<std::chrono::duration<double>>(
        d - c).count() * 1000.0;
    double chpt_e = std::chrono::duration_cast<std::chrono::duration<double>>(
        e - d).count() * 1000.0;
    double chpt_f = std::chrono::duration_cast<std::chrono::duration<double>>(
        f - e).count() * 1000.0;

    double total_dur = chpt_b + chpt_c + chpt_d + chpt_e + chpt_f;
    std::cout << "==========" << std::endl;

    std::cout << "          b: " << chpt_b *1000  << ", pct:" << chpt_b / total_dur * 100 << "%" << std::endl;
    std::cout << "          c: " << chpt_c *1000  << ", pct:" << chpt_c / total_dur * 100 << "%" << std::endl;
    std::cout << "          d: " << chpt_d *1000  << ", pct:" << chpt_d / total_dur * 100 << "%" << std::endl;
    std::cout << "              d2: " << d2_dur *1000  << ", pct:" << d2_dur / total_dur * 100 << "%" << std::endl;
    std::cout << "              d3: " << d3_dur *1000  << ", pct:" << d3_dur / total_dur * 100 << "%" << std::endl;
    std::cout << "          e: " << chpt_e *1000  << ", pct:" << chpt_e / total_dur * 100 << "%" << std::endl;
    std::cout << "          f: " << chpt_f *1000  << ", pct:" << chpt_f / total_dur * 100 << "%" << std::endl;
    std::cout << "==========" << std::endl;

    return true;
}

void SphericalSFC::transformPoints(std::vector<Eigen::Vector3d>& pts, Eigen::Vector3d origin, const Eigen::Matrix<double, 3, 3>& rot_mat)
{
    for (auto& pt : pts){
        pt = (rot_mat * (pt - origin)) + origin;
    }
}

double SphericalSFC::computeCandSphereScore(Sphere& B_cand, Sphere& B_prev)
{      
    if (B_cand.getVolume() > sfc_params_.max_sphere_vol) // candidate sphere's volume has exceeded the maximum allowed
    {
        return -1.0;
    }

    double dist_btw_centers_sq = (B_cand.center - B_prev.center).squaredNorm();

    if (B_prev.contains(B_cand.center)){ // candidate sphere's center inside previous sphere
        if (dist_btw_centers_sq + B_cand.radius <= B_prev.radius){ // If candidate sphere is fully contained within previous sphere
            return -1.0;
        }
    }

    // Obtain volume of candidate sphere
    double V_cand = B_cand.getVolume();
    // if candidate sphere is below minimum sphere volume, discard.
    if (V_cand < sfc_params_.min_sphere_vol){ 
        return -1.0;
    }

    // Obtain volume of intersection between candidate and previous sphere 
    double V_intersect = getIntersectingVolume(B_cand, B_prev);
    // IF intersection volume is zero, discard.
    if (V_intersect < sfc_params_.min_sphere_intersection_vol){
        return -1.0;
    }

    return sfc_params_.W_cand_vol * V_cand + sfc_params_.W_intersect_vol * V_intersect;
}

double SphericalSFC::getIntersectingVolume(Sphere& B_a, Sphere& B_b)
{
    double d = (B_a.center - B_b.center).norm(); // distance between center of spheres

    // Check for non-intersection
    if (d >= (B_a.radius + B_b.radius)){ 
        return -1;
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

void SphericalSFC::computeSFCTrajectory(const std::vector<SphericalSFC::Sphere>& sfc_spheres, const Eigen::Vector3d& goal_pos, SphericalSFC::SFCTrajectory& sfc_traj)
{
    /* 1: Retrieve waypoints from the intersections between the spheres */
    std::vector<Eigen::Vector3d> traj_waypoints(sfc_spheres.size()+1);
 
    traj_waypoints[0] = sfc_spheres[0].center; // Add start state
    for (int i = 1; i < sfc_spheres.size(); i++){
        // TODO: Add check for intersection between 2 spheres?

        // Add intersection between 2 spheres
        traj_waypoints[i] = getIntersectionCenter(sfc_spheres[i-1], sfc_spheres[i]) ;
    }
    traj_waypoints.back() = goal_pos; // Add goal state

    /* 2: Get time allocation using constant velocity time allocation */
    std::vector<double> segs_time_durations(traj_waypoints.size()-1); // Vector of time durations for each segment {t_1, t_2, ..., t_M}
    // TODO: Use trapezoidal time allocation

    for (int i = 0 ; i < traj_waypoints.size()-1; i++){
        // Using constant velocity time allocation
        segs_time_durations[i] = (traj_waypoints[i+1] - traj_waypoints[i]).norm() / sfc_params_.avg_vel ;
    }

    sfc_traj.spheres = sfc_spheres;
    sfc_traj.waypoints = traj_waypoints;
    sfc_traj.segs_t_dur = segs_time_durations;
}

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

void SphericalSFC::publishVizPiecewiseTrajectory(
    const std::vector<Eigen::Vector3d>& pts, ros::Publisher& publisher, 
    const std::string& frame_id)
{
    if (!sfc_params_.debug_viz){
        return;
    }

    visualization_msgs::Marker sphere_list, path_line_strip;
    Eigen::Vector3d wp_color = Eigen::Vector3d{0.0, 0.0, 1.0};
    double wp_alpha = 0.3;
    double wp_radius = 0.15;

    Eigen::Vector3d line_color = Eigen::Vector3d{0.0, 0.0, 1.0};
    double line_alpha = 0.3;
    double line_scale = 0.05;

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

void SphericalSFC::publishVizPoints(
    const std::vector<Eigen::Vector3d>& pts, ros::Publisher& publisher, 
    Eigen::Vector3d color, double radius, const std::string& frame_id)
{
    if (!sfc_params_.debug_viz){
        return; 
    }
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

void SphericalSFC::publishVizSphericalSFC(  
    const std::vector<SphericalSFC::Sphere>& sfc_spheres, 
    ros::Publisher& publisher, const std::string& frame_id) 
{
    if (!sfc_params_.debug_viz){
        return;
    }

    for (int i = 0; i < sfc_spheres.size(); i++){
        publisher.publish(createVizSphere(sfc_spheres[i].center, sfc_spheres[i].getDiameter(), frame_id, i));
    }
}

visualization_msgs::Marker SphericalSFC::createVizSphere( 
    const Eigen::Vector3d& center, const double& diameter, 
    const std::string& frame_id, const int& id)
{
    visualization_msgs::Marker sphere;

    sphere.header.frame_id = frame_id;
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    sphere.pose.orientation.w = 1.0;

    sphere.color.r = 0.5;
    sphere.color.g = 0.5;
    sphere.color.b = 0.5;
    sphere.color.a = 0.3;
    sphere.scale.x = diameter;
    sphere.scale.y = diameter;
    sphere.scale.z = diameter;

    sphere.pose.position.x = center(0);
    sphere.pose.position.y = center(1);
    sphere.pose.position.z = center(2);

    return sphere;
  }

visualization_msgs::Marker SphericalSFC::createVizEllipsoid(
    const Eigen::Vector3d& center, const Eigen::Vector3d& stddev, 
    const Eigen::Quaterniond& orientation, const std::string& frame_id, const int& id)
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