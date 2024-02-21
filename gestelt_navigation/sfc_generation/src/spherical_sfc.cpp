#include <sfc_generation/spherical_sfc.h>

SphericalSFC::SphericalSFC(std::shared_ptr<GridMap> grid_map, const SphericalSFCParams& sfc_params):
    grid_map_(grid_map), sfc_params_(sfc_params)
{}   

void SphericalSFC::addVizPublishers(
    ros::Publisher& p_cand_viz_pub, 
    ros::Publisher& dist_viz_pub, 
    ros::Publisher& sfc_spherical_viz_pub,
    ros::Publisher&  sfc_waypoints_viz_pub,
    ros::Publisher& samp_dir_vec_pub)
{
    p_cand_viz_pub_ = p_cand_viz_pub;
    dist_viz_pub_ = dist_viz_pub;
    samp_dir_vec_pub_ = samp_dir_vec_pub;
    sfc_spherical_viz_pub_ = sfc_spherical_viz_pub;
    sfc_waypoints_viz_pub_ = sfc_waypoints_viz_pub;
}

void SphericalSFC::reset()
{
    // Clear planning data
    sfc_spheres_.clear();
    sfc_traj_.clear();

    // Clear visualizations
    p_cand_vec_hist_.clear();
    sampling_dist_hist_.markers.clear();
    samp_dir_vec_hist_.markers.clear();

    clearVisualizations();
}   

void SphericalSFC::clearVisualizations()
{
    visualization_msgs::MarkerArray markerarray;

    visualization_msgs::Marker marker;
    marker.id = 0; 
    marker.action = visualization_msgs::Marker::DELETEALL;
    
    visualization_msgs::Marker marker_arr_m;
    marker_arr_m.action = visualization_msgs::Marker::DELETEALL;
    marker_arr_m.id = 0;
    marker_arr_m.ns = "sfc_samp_dist";
    markerarray.markers.push_back(marker_arr_m);
    marker_arr_m.ns = "sfc_spheres";
    markerarray.markers.push_back(marker_arr_m);
    marker_arr_m.ns = "sfc_samp_dir_vec";
    markerarray.markers.push_back(marker_arr_m);

    marker.ns = "sfc_cand_pts"; 
    p_cand_viz_pub_.publish(marker);

    dist_viz_pub_.publish(markerarray);

    sfc_spherical_viz_pub_.publish(markerarray); 

    marker.ns = "sfc_trajectory_waypoints"; 
    sfc_waypoints_viz_pub_.publish(marker); 
    marker.ns = "sfc_trajectory_line"; 
    sfc_waypoints_viz_pub_.publish(marker); 

    samp_dir_vec_pub_.publish(markerarray); 
}

bool SphericalSFC::generateSFC(const std::vector<Eigen::Vector3d> &path)
{
    reset();

    double get_fwd_pt_durs{0};
    double batch_sample_durs{0};

    auto a = std::chrono::high_resolution_clock::now();

    Sphere B_starting; // starting sphere
    Sphere B_cur; // current sphere being considered

    size_t path_idx_cur = 0; // Current index of guide path

    // Initialize largest sphere at initial position
    if (!generateFreeSphere(path[0], B_cur)){
        std::cout << "Failed to generate free sphere centered on start point" << std::endl;
        return false;
    }

    // Intialize largest sphere along guide path containing start point
    for (size_t i = 0; i < path.size(); i++){
        // Generate free sphere and check if it contains start point
        //      IF NOT, then stop and pick previous sphere
        if (!generateFreeSphere(path[i], B_cur)){
            break;
        }
        // Check if new sphere contains start point
        if (!B_cur.contains(path[0])){
            break;
        }
        path_idx_cur = i;
        B_starting = B_cur;
    }
    
    // Add first sphere
    sfc_spheres_.push_back(B_starting);
    
    auto b = std::chrono::high_resolution_clock::now();

    itr_ = 0;
    while (itr_ < sfc_params_.max_itr)
    {
        auto get_fwd_pt = std::chrono::high_resolution_clock::now();

        if (!getForwardPointOnPath(path, path_idx_cur, B_cur)){
            std::cout << "getForwardPointOnPath: Failed to get forward point on the path" << std::endl;
            break;
        }

        auto get_fwd_pt_end = std::chrono::high_resolution_clock::now();

        auto batch_sample = std::chrono::high_resolution_clock::now();

        if (!BatchSample(path[path_idx_cur], B_cur)){
            std::cout << "Batch sample failed" << std::endl;
            break;
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

    // Publish candidate points and 3d distribution visualization
    if (sfc_params_.debug_viz){

        publishVizPoints(p_cand_vec_hist_, p_cand_viz_pub_); 
        dist_viz_pub_.publish(sampling_dist_hist_);
        samp_dir_vec_pub_.publish(samp_dir_vec_hist_);

        publishVizSphericalSFC(sfc_spheres_, sfc_spherical_viz_pub_, "world");

        computeSFCTrajectory(sfc_spheres_, path[0], path.back(), sfc_traj_);
        publishVizPiecewiseTrajectory(sfc_traj_.waypoints, sfc_waypoints_viz_pub_);
    }

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
    
    if (sfc_params_.debug_viz){
        // std::cout << "Spherical SFC runtimes [ms]: " << std::endl;

        // std::cout << "  Total dur: " << total_loop_dur *1000   << std::endl;
        // std::cout << "  Preloop dur: " << preloop_dur *1000    << ", pct:" << preloop_dur / total_loop_dur * 100 << "%" << std::endl;
        // std::cout << "  loop dur: " << loop_dur  *1000         << ", pct:" << loop_dur / total_loop_dur * 100<< "%" << std::endl;
        // std::cout << "      get_fwd_pt_durs: " << get_fwd_pt_durs  *1000         << ", pct:" << get_fwd_pt_durs / total_loop_dur * 100<< "%" << std::endl;
        // std::cout << "      batch_sample_durs: " << batch_sample_durs  *1000     << ", pct:" << batch_sample_durs / total_loop_dur * 100<< "%" << std::endl;

        // std::cout << "  publish dur: " << pub_dur  *1000       << "s, pct:" << pub_dur / total_loop_dur * 100<< "%" << std::endl;
    }

    return true;
}   

bool SphericalSFC::generateFreeSphere(const Eigen::Vector3d& point, Sphere& B)
{
    Eigen::Vector3d occ_nearest;
    double dist_to_nearest_obs;

    if (grid_map_->getNearestOccupiedCell(point, occ_nearest, dist_to_nearest_obs)){
        B.center = point;
        if (dist_to_nearest_obs <= sfc_params_.spherical_buffer){
            return false;
        }
        B.setRadius(dist_to_nearest_obs - sfc_params_.spherical_buffer);
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

bool SphericalSFC::BatchSample(const Eigen::Vector3d& pt_guide, Sphere& B_cur)
{
    auto a = std::chrono::high_resolution_clock::now();

    // Priority queue of candidate spheres sorted by highest score first
    std::priority_queue<std::shared_ptr<Sphere>, std::vector<std::shared_ptr<Sphere>>, Sphere::CompareScorePtr> B_cand_pq = 
        std::priority_queue<std::shared_ptr<Sphere>, std::vector<std::shared_ptr<Sphere>>, Sphere::CompareScorePtr>();

    std::vector<Eigen::Vector3d> p_cand_vec; // vector of candidate points

    // Calculate orientation and standard deviation of normal sampling distribution
    Eigen::Vector3d dir_vec = (pt_guide - B_cur.center); // Direction vector from previous sphere to pt_guide
    // std::cout << "Dir vec: " << dir_vec << std::endl;
    double stddev_x = sfc_params_.mult_stddev_x * dir_vec.norm(); // Get standard deviation along direction vector
    double stddev_y = sfc_params_.mult_stddev_y * dir_vec.norm(); // Get standard deviation along direction vector
    double stddev_z = sfc_params_.mult_stddev_z * dir_vec.norm(); // Get standard deviation along direction vector
    Eigen::Vector3d stddev_3d{stddev_x, stddev_y, stddev_z};

    // Calculate orientation of distribution ellipsoid
    Eigen::Matrix<double, 3, 3> ellipse_rot_mat = rotationAlign(Eigen::Vector3d::UnitX(), dir_vec.normalized());
    Eigen::Quaterniond ellipse_orientation(ellipse_rot_mat);

    // Set up seed for sampler
    uint64_t seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    sampler_.setParams(pt_guide, stddev_3d, seed);
    
    auto b = std::chrono::high_resolution_clock::now();

    // Sample the points first
    p_cand_vec = sampler_.sample(sfc_params_.max_sample_points);

    auto c = std::chrono::high_resolution_clock::now();

    // Transform set of sampled points to have its mean at pt_guide and rotated along dir_vec
    // TODO: Use matrix parallelization for transformations
    transformPoints(p_cand_vec, pt_guide, ellipse_rot_mat);

    auto d = std::chrono::high_resolution_clock::now();

    double d2_dur{0};
    double d3_dur{0};

    for (auto& p_cand: p_cand_vec){
        // Generate candidate sphere, calculate score and add to priority queue

        auto d1 = std::chrono::high_resolution_clock::now();

        std::shared_ptr<Sphere> B_cand = std::make_shared<Sphere>();
        
        if (!generateFreeSphere(p_cand, *B_cand)){
            continue;
        }

        // Condition for a valid candidate sphere
        // 1) Cand sphere contains guide point
        // 2) Cand sphere is within minimum and maximum volume bounds 
        // 3) Cand sphere is not completely contained within previous sphere

        // 1) Sphere must contain guide point
        if (!B_cand->contains(pt_guide)){
            continue;
        }

        // 2) Sphere is bounded by a minimum and maximum volume 
        if (B_cand->getVolume() < sfc_params_.min_sphere_vol || B_cand->getVolume() > sfc_params_.max_sphere_vol) 
        {
            // If candidate sphere's volume is below minimum or maximum volume
            continue;
        }

        // 3) Cand sphere is not completely contained within previous sphere
        if (B_cur.contains(B_cand->center)){ // candidate sphere's center inside previous sphere
            if ((B_cand->center - B_cur.center).norm() + B_cand->radius <= B_cur.radius){ // If candidate sphere is fully contained within previous sphere
                continue;
            }
        }

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

    double total_dur = chpt_b + chpt_c + chpt_d + chpt_e;
    if (sfc_params_.debug_viz){
    
        // std::cout << "==========" << std::endl;

        // std::cout << "          b: " << chpt_b *1000  << ", pct:" << chpt_b / total_dur * 100 << "%" << std::endl;
        // std::cout << "          c: " << chpt_c *1000  << ", pct:" << chpt_c / total_dur * 100 << "%" << std::endl;
        // std::cout << "          d: " << chpt_d *1000  << ", pct:" << chpt_d / total_dur * 100 << "%" << std::endl;
        // std::cout << "              d2: " << d2_dur *1000  << ", pct:" << d2_dur / total_dur * 100 << "%" << std::endl;
        // std::cout << "              d3: " << d3_dur *1000  << ", pct:" << d3_dur / total_dur * 100 << "%" << std::endl;
        // std::cout << "          e: " << chpt_e *1000  << ", pct:" << chpt_e / total_dur * 100 << "%" << std::endl;
        // std::cout << "==========" << std::endl;
    }

    p_cand_vec_hist_.insert(p_cand_vec_hist_.end(), p_cand_vec.begin(), p_cand_vec.end());
    sampling_dist_hist_.markers.push_back(createVizEllipsoid(pt_guide, stddev_3d, ellipse_orientation, "world", itr_));
    samp_dir_vec_hist_.markers.push_back(createArrow(pt_guide, dir_vec, "world", itr_));

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
    return sfc_params_.W_cand_vol * B_cand.getVolume() + sfc_params_.W_intersect_vol * getIntersectingVolume(B_cand, B_prev);
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
    double d_intersect = (B_a.radius*B_a.radius + d_centroids*d_centroids - B_b.radius*B_b.radius ) / (2*d_centroids);

    Eigen::Vector3d pt_intersect = B_a.center + (d_intersect/d_centroids) * dir_vec;

    return pt_intersect;
}

void SphericalSFC::computeSFCTrajectory(
    const std::vector<SphericalSFC::Sphere>& sfc_spheres, 
    const Eigen::Vector3d& start_pos, 
    const Eigen::Vector3d& goal_pos, 
    SphericalSFC::SFCTrajectory& sfc_traj)
{
    /* 1: Retrieve waypoints from the intersections between the spheres */
    std::vector<Eigen::Vector3d> traj_waypoints(sfc_spheres.size()+1);
 
    traj_waypoints[0] = start_pos; // Add start state
    for (size_t i = 1; i < sfc_spheres.size(); i++){
        // TODO: Add check for intersection between 2 spheres?

        // Add intersection between 2 spheres
        traj_waypoints[i] = getIntersectionCenter(sfc_spheres[i-1], sfc_spheres[i]) ;
    }
    traj_waypoints.back() = goal_pos; // Add goal state

    /* 2: Get time allocation  */
    std::vector<double> segs_time_durations(traj_waypoints.size()-1); // Vector of time durations for each segment {t_1, t_2, ..., t_M}

    // We can either use triangle or trapezoidal time profile.
    // We use a trapezoidal time profile

    for (size_t i = 0 ; i < traj_waypoints.size()-1; i++){
        double traj_dist = (traj_waypoints[i+1] - traj_waypoints[i]).norm();
        
        // For trapezoidal time profile: t_s, t_c and t_d is the time taken to 
        //  accelerate, travel at maximum velocity, and decelerate respectively.
        double t_s, t_c, t_d;
        t_c = ( (traj_dist / sfc_params_.max_vel) - (sfc_params_.max_vel / sfc_params_.max_acc) );
        if (t_c <= 0){ // Follow triangle profile
            t_s = sqrt(traj_dist/(2*sfc_params_.max_acc));
            t_d = t_s;
            t_c = 0.0;
        }
        else{
            t_s = sfc_params_.max_vel / sfc_params_.max_acc;
            t_d = t_s;
        }
        segs_time_durations[i] = t_s + t_c + t_d;
    }

    // for (size_t i = 0 ; i < traj_waypoints.size()-1; i++){
    //     // Using constant velocity time allocation
    //     segs_time_durations[i] = (traj_waypoints[i+1] - traj_waypoints[i]).norm() / sfc_params_.avg_vel ;
    // }

    sfc_traj.spheres = sfc_spheres;
    sfc_traj.waypoints = traj_waypoints;
    sfc_traj.segs_t_dur = segs_time_durations;
}

Eigen::Matrix<double, 3, 3> SphericalSFC::rotationAlign(
    const Eigen::Vector3d & z, const Eigen::Vector3d & d)
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

    visualization_msgs::Marker sphere_list, path_line_strip;
    Eigen::Vector3d wp_color = Eigen::Vector3d{0.0, 0.0, 1.0};
    double wp_alpha = 0.75;
    double wp_radius = 0.15;

    Eigen::Vector3d line_color = Eigen::Vector3d{0.0, 0.0, 1.0};
    double line_alpha = 0.75;
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
    sphere_list.ns = "sfc_trajectory_waypoints"; 
    sphere_list.id = 0; 
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
    path_line_strip.ns = "sfc_trajectory_line"; 
    path_line_strip.id = 0;
    path_line_strip.pose.orientation.w = 1.0;

    path_line_strip.color.r = line_color(0);
    path_line_strip.color.g = line_color(1);
    path_line_strip.color.b = line_color(2);
    path_line_strip.color.a = line_alpha;

    path_line_strip.scale.x = line_scale;

    geometry_msgs::Point pt;
    for (size_t i = 0; i < pts.size(); i++){
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
    visualization_msgs::Marker sphere_list;
    double alpha = 0.7;

    sphere_list.header.frame_id = frame_id;
    sphere_list.header.stamp = ros::Time::now();
    sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;
    sphere_list.action = visualization_msgs::Marker::ADD;
    sphere_list.ns = "sfc_cand_pts"; 
    sphere_list.id = 0; 
    sphere_list.pose.orientation.w = 1.0;

    sphere_list.color.r = color(0);
    sphere_list.color.g = color(1);
    sphere_list.color.b = color(2);
    sphere_list.color.a = alpha;

    sphere_list.scale.x = radius;
    sphere_list.scale.y = radius;
    sphere_list.scale.z = radius;

    geometry_msgs::Point pt;
    for (size_t i = 0; i < pts.size(); i++){
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
    visualization_msgs::MarkerArray sfc_spheres_marker_arr;

    for (size_t i = 0; i < sfc_spheres.size(); i++){
        sfc_spheres_marker_arr.markers.push_back(createVizSphere(sfc_spheres[i].center, sfc_spheres[i].getDiameter(), frame_id, i));
    }
    publisher.publish(sfc_spheres_marker_arr);
}

visualization_msgs::Marker SphericalSFC::createArrow(
    const Eigen::Vector3d& pt_guide, const Eigen::Vector3d& dir_vec, 
    const std::string& frame_id, const int& id)
{
    visualization_msgs::Marker arrow;

    arrow.ns = "sfc_samp_dir_vec";
    arrow.header.frame_id = frame_id;
    arrow.header.stamp = ros::Time::now();
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;

    double scale = 0.1;

    arrow.color.r = 1.0;
    arrow.color.g = 1.0;
    arrow.color.b = 1.0;
    arrow.color.a = 0.7;
    arrow.scale.x = scale;
    arrow.scale.y = 2 * scale;
    arrow.scale.z = 2 * scale;

    geometry_msgs::Point start, end;
    start.x = pt_guide(0);
    start.y = pt_guide(1);
    start.z = pt_guide(2);
    end.x = pt_guide(0) + dir_vec(0);
    end.y = pt_guide(1) + dir_vec(1);
    end.z = pt_guide(2) + dir_vec(2);

    arrow.points.clear();
    arrow.points.push_back(start);
    arrow.points.push_back(end);
    arrow.id = id;

    return arrow;
}

visualization_msgs::Marker SphericalSFC::createVizSphere( 
    const Eigen::Vector3d& center, const double& diameter, 
    const std::string& frame_id, const int& id)
{
    visualization_msgs::Marker sphere;

    sphere.ns = "sfc_spheres";
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
    const Eigen::Vector3d& center, const Eigen::Vector3d& stddev_3d, 
    const Eigen::Quaterniond& orientation, const std::string& frame_id, const int& id)
{
    visualization_msgs::Marker sphere;

    sphere.ns = "sfc_samp_dist";
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
    sphere.scale.x = stddev_3d(0);
    sphere.scale.y = stddev_3d(1);
    sphere.scale.z = stddev_3d(2);

    sphere.pose.position.x = center(0);
    sphere.pose.position.y = center(1);
    sphere.pose.position.z = center(2);

    return sphere;
}