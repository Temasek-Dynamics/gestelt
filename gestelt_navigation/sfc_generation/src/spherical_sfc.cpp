#include <sfc_generation/spherical_sfc.h>

SphericalSFC::SphericalSFC(std::shared_ptr<GridMap> grid_map, const SphericalSFCParams& sfc_params):
    grid_map_(grid_map), sfc_params_(sfc_params)
{}   

void SphericalSFC::addPublishers(
    std::unordered_map<std::string, ros::Publisher >& publisher_map
    )
{
    p_cand_viz_pub_ =           publisher_map["sfc_cand_points"];   // Marker
    dist_viz_pub_ =             publisher_map["sfc_dist"];          // MarkerArray
    sfc_spherical_viz_pub_ =    publisher_map["sfc_spherical"];     // MarkerArray
    sfc_waypoints_viz_pub_ =    publisher_map["sfc_waypoints"];     // Marker
    samp_dir_vec_pub_ =         publisher_map["sfc_samp_dir_vec"];  // MarkerArray
    intxn_spheres_pub_ =        publisher_map["sfc_intxn_spheres"]; // MarkerArray
}

void SphericalSFC::reset()
{
    // Clear planning data
    itr_ = 0;
    sfc_spheres_.clear();
    sfc_traj_.reset();
    front_end_path_.clear();

    // Clear visualizations
    p_cand_vec_hist_.clear();
    sampling_dist_hist_.markers.clear();
    samp_dir_vec_hist_.markers.clear();

    sfc_sampled_spheres_.clear();
    samp_dir_vec_.clear();
    guide_points_vec_.clear();
    
    front_end_path_.clear();

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

bool SphericalSFC::generateSFC( const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &path,
                                const bool& enable_rhc_plan,
                                const double& rhc_dist,
                                const Eigen::Vector3d& start_pos,
                                const double& req_plan_time)
{
    // IF nearing the end of goal, disable RHC Planning
    bool do_rhc_planning = enable_rhc_plan && sfc_traj_.spheres.size() > 3;

    if (do_rhc_planning){ // enable receding horizon corridor planning
        // Prune trajectory segments that have been traversed
        // PRUNE FROM 0 TO NUM_SEGS_TRAVERSED
        int num_segs_traversed = sfc_traj_.getNumSegmentsTraversed(req_plan_time);
        sfc_traj_.prune(0, num_segs_traversed);

        // Keep segments up to maximum waypoint index and prune the rest
        // PRUNE FROM MAX_WP_IDX TO END
        int max_wp_idx = sfc_traj_.getMaxWaypointWithinDist(start_pos, rhc_dist);
        sfc_traj_.keep(max_wp_idx);
    }
    else { // Plan from scratch
        reset(); //reset all data structures
    }

    /* Initialize all data structures */

    front_end_path_ = path; // front_end path
    guide_path_kdtree_ = 
        std::make_unique<KDTreeVectorOfVectorsAdaptor<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>, double>>(
            3, path, 10); //kd tree used to query nearest point on the front-end path

    SSFC::Sphere B_cur; // current sphere being considered
    size_t path_idx_cur = 0; // Current index of guide path
    bool planning_success = true;
    bool skip_main_loop = false;

    if (do_rhc_planning) { // enable receding horizon corridor planning
        // std::vector<double> query_pt;
        // if (sfc_traj_.waypoints.empty()){
        //     query_pt = {sfc_traj_.waypoints.back()(0), 
        //                 sfc_traj_.waypoints.back()(1), 
        //                 sfc_traj_.waypoints.back()(2)};
        // }
        // else {
        //     query_pt = {start_pos(0), 
        //                 start_pos(1), 
        //                 start_pos(2)};
        // }

        // Get nearest point from last sfc waypoint to front end path
        std::vector<double> query_pt = {sfc_traj_.waypoints.back()(0), 
                                        sfc_traj_.waypoints.back()(1), 
                                        sfc_traj_.waypoints.back()(2)};
        std::vector<size_t> out_indices(1);
        std::vector<double> out_distances_sq(1);

        guide_path_kdtree_->query(&query_pt[0], 1, &out_indices[0], &out_distances_sq[0]);

        path_idx_cur = out_indices[0];
        
        // Assign spheres from receding horizon corridor
        sfc_spheres_ = sfc_traj_.spheres;
        B_cur = sfc_spheres_.back();

        // Skip main loop if goal is in starting sphere
        if (sfc_spheres_.back().contains(path.back())){
            skip_main_loop = true; 
        }
    }
    else { // Plan from scratch
        SSFC::Sphere B_start; // starting sphere
        
        // Initialize largest sphere at initial position
        if (!generateFreeSphere(path[0], B_cur)){
            std::cout << "[SSFC] Failed to generate free sphere centered on start point" << std::endl;
            return false;
        }

        // Initialize largest sphere along guide path containing start point
        for (size_t i = 0; i < path.size(); i++){
            // Generate free sphere and check if it contains start point
            //      IF NOT, then stop and pick previous sphere
            if (!generateFreeSphere(path[i], B_cur)){
                break;
            }
            // Check if new sphere contains start point
            if (!B_cur.contains(path[0])){
                B_cur = B_start;
                break;
            }
            path_idx_cur = i;
            B_start = B_cur;
        }
        
        // Add first sphere
        sfc_spheres_.push_back(B_start);
        // Skip main loop if goal is in starting sphere
        if (sfc_spheres_.back().contains(path.back())){
            skip_main_loop = true; 
        }
    }

    /* Main loop */

    itr_ = 0;
    while (itr_ < sfc_params_.max_itr && !skip_main_loop)
    {
        if (!getForwardPointOnPath(path, path_idx_cur, B_cur)){
            std::cout << "getForwardPointOnPath: Failed to get forward point on the path" << std::endl;
            planning_success = false;
            break;
        }

        if (!BatchSample(path[path_idx_cur], B_cur)){
            std::cout << "[SSFC] Batch sample failed" << std::endl;
            planning_success = false;
            break;
        }
        
        sfc_spheres_.push_back(B_cur);

        if (B_cur.contains(path.back())){ // If current sphere contains the goal
            break;
        }

        itr_++;
    }

    if (!sfc_spheres_.back().contains(path.back())){ 
        std::cout << "[SSFC] Final safe flight corridor does not contain the goal" << std::endl;

        if (itr_ > sfc_params_.max_itr){
            std::cout << "[SSFC] Maximum iterations " << sfc_params_.max_itr 
                    << " exceeded. Consumed " << itr_ << " iterations." << std::endl;
        }

        planning_success = false;
    }
    
    if (planning_success){
        postProcessSpheres(sfc_spheres_);
        constructSFCTrajectory(sfc_spheres_, path[0], path.back(), sfc_traj_);
    }

    // Publish candidate points and 3d distribution visualization
    if (sfc_params_.debug_viz){

        publishVizPoints(p_cand_vec_hist_, p_cand_viz_pub_); 
        dist_viz_pub_.publish(sampling_dist_hist_);
        samp_dir_vec_pub_.publish(samp_dir_vec_hist_);

        publishVizSphericalSFC(sfc_spheres_, sfc_spherical_viz_pub_, "world");

        publishVizIntxnSpheres(sfc_traj_.intxn_spheres, intxn_spheres_pub_, "world");

        publishVizPiecewiseTrajectory(sfc_traj_.waypoints, sfc_waypoints_viz_pub_);
    }

    return planning_success;
}   

bool SphericalSFC::generateFreeSphere(const Eigen::Vector3d& center, SSFC::Sphere& B)
{
    Eigen::Vector3d obs_pos;
    double R; // R: Radius of largest possible free sphere from center, i.e. distance to nearest obstacle
    double bfr = sfc_params_.spherical_buffer; // b: buffer

    if (grid_map_->getNearestOccupiedCell(center, obs_pos, R)){
        if (R <= bfr){
            return false;
        }
        B.center = center;
        B.setRadius(R - bfr);

        return true;
    }
    return false;
}

bool SphericalSFC::getForwardPointOnPath(
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &path, size_t& start_idx, const SSFC::Sphere& B_prev)
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

bool SphericalSFC::BatchSample(const Eigen::Vector3d& pt_guide, SSFC::Sphere& B_cur)
{

    // Priority queue of candidate spheres sorted by highest score first
    std::priority_queue<std::shared_ptr<SSFC::Sphere>, std::vector<std::shared_ptr<SSFC::Sphere>>, SSFC::Sphere::CompareScorePtr> B_cand_pq = 
        std::priority_queue<std::shared_ptr<SSFC::Sphere>, std::vector<std::shared_ptr<SSFC::Sphere>>, SSFC::Sphere::CompareScorePtr>();

    std::vector<Eigen::Vector3d> p_cand_vec; // vector of candidate points

    // Calculate orientation and standard deviation of normal sampling distribution
    Eigen::Vector3d dir_vec = (pt_guide - B_cur.center); // Direction vector from previous sphere to pt_guide
    Eigen::Vector3d dir_vec_unit = dir_vec.normalized();

    Eigen::Vector3d samp_dir_vec = dir_vec;
    Eigen::Vector3d samp_mean = pt_guide;

    double stddev_x = sfc_params_.mult_stddev_x * samp_dir_vec.norm(); // Get standard deviation along direction vector
    double stddev_y = sfc_params_.mult_stddev_y * samp_dir_vec.norm(); // Get standard deviation along direction vector
    double stddev_z = sfc_params_.mult_stddev_z * samp_dir_vec.norm(); // Get standard deviation along direction vector
    Eigen::Vector3d stddev_3d{stddev_x, stddev_y, stddev_z};

    // Calculate orientation of distribution ellipsoid
    Eigen::Matrix<double, 3, 3> ellipse_rot_mat = rotationAlign(Eigen::Vector3d::UnitX(), dir_vec_unit);
    Eigen::Quaterniond ellipse_orientation(ellipse_rot_mat);

    // Set up seed for sampler
    uint64_t seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    sampler_.setParams(samp_mean, stddev_3d, seed);

    // Sample the points first
    p_cand_vec = sampler_.sample(sfc_params_.max_sample_points);

    // Transform set of sampled points to have its mean at pt_guide and rotated along dir_vec
    // TODO: Use matrix parallelization for transformations
    transformPoints(p_cand_vec, samp_mean, ellipse_rot_mat);


    std::vector<SSFC::Sphere> sampled_spheres; // Vector of all sampled spheres used for debugging

    for (auto& p_cand: p_cand_vec){

        // Generate candidate sphere, calculate score and add to priority queue

        std::shared_ptr<SSFC::Sphere> B_cand = std::make_shared<SSFC::Sphere>();
        
        if (!generateFreeSphere(p_cand, *B_cand)){
            continue;
        }

        sampled_spheres.push_back(*B_cand); // For debugging

        // Condition for a valid candidate sphere
        // 1) Cand sphere contains guide point
        // 2) Cand sphere is within minimum and maximum volume bounds 
        // 3) Cand sphere is not completely contained within previous sphere
        // 4) Cand sphere is intersecting the previous sphere

        // 1) SSFC::Sphere must contain guide point
        // if (!B_cand->contains(pt_guide)){
        //     continue;
        // }

        // 2) SSFC::Sphere is bounded by a minimum and maximum volume 
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

        // 4) Cand sphere is intersecting previous sphere
        if(getIntersectingVolume(*B_cand, B_cur) < 0.0001){
            continue;
        }


        B_cand->score = computeCandSphereScore(*B_cand, B_cur); 

        if (B_cand->score > 0) { // If score is positive, add sphere to priority queue
            B_cand_pq.push(B_cand); 
        }

    }


    p_cand_vec_hist_.insert(p_cand_vec_hist_.end(), p_cand_vec.begin(), p_cand_vec.end());
    sampling_dist_hist_.markers.push_back(createVizEllipsoid(samp_mean, stddev_3d, ellipse_orientation, "world", itr_));
    samp_dir_vec_hist_.markers.push_back(createArrow(samp_mean, samp_dir_vec, "world", itr_));

    // For debugging purposes
    sfc_sampled_spheres_.push_back(sampled_spheres);
    samp_dir_vec_.push_back(samp_dir_vec);
    guide_points_vec_.push_back(samp_mean);

    if (B_cand_pq.empty()){
        // std::cout << "[SSFC] Unable to generate next candidate sphere, B_cand_pq empty!" << std::endl;
        return false;
    }

    B_cur = SSFC::Sphere(B_cand_pq.top());

    // std::cout << "Assigned next candidate sphere of radius " << B_cand_pq.top()->radius 
    //             << ", with volume " << B_cand_pq.top()->getVolume() 
    //             << ", intersecting volume " << getIntersectingVolume(*B_cand_pq.top(), B_cur ) 
    //             << ", Center " << B_cand_pq.top()->center
    //             << std::endl;

    return true;
}

void SphericalSFC::transformPoints(std::vector<Eigen::Vector3d>& pts, Eigen::Vector3d origin, const Eigen::Matrix<double, 3, 3>& rot_mat)
{
    for (auto& pt : pts){
        pt = (rot_mat * (pt - origin)) + origin;
    }
}

double SphericalSFC::computeCandSphereScore(SSFC::Sphere& B_cand, SSFC::Sphere& B_prev)
{      
    std::vector<double> query_pt = {B_cand.center(0), B_cand.center(1), B_cand.center(2)};
    
    const size_t        num_closest = 1;
    std::vector<size_t> out_indices(num_closest);
    std::vector<double> out_distances_sq(num_closest);

    // Progress measures how much progress the candidate sphere has made along the path
    double progress = 0.0;
    if (sfc_params_.W_progress > 0){
        guide_path_kdtree_->query(&query_pt[0], num_closest, &out_indices[0], &out_distances_sq[0]);
        progress = double(out_indices[0]) / front_end_path_.size();
    }

    return sfc_params_.W_cand_vol * B_cand.getVolume() 
        + sfc_params_.W_intersect_vol * getIntersectingVolume(B_cand, B_prev) 
        + sfc_params_.W_progress * progress;
}

double SphericalSFC::getIntersectingVolume(SSFC::Sphere& B_a, SSFC::Sphere& B_b)
{
    if (!isIntersect(B_a, B_b)){ 
        return -1;
    }

    double d = (B_a.center - B_b.center).norm(); // distance between center of spheres
    double h = (B_a.radius - (d - B_b.radius))/2; // Height of spherical cap
    double a = sqrt( B_a.radius*B_a.radius - (d - B_b.radius +h )*(d - B_b.radius +h )); // Radius of spherical cap

    double vol_intersect = 2.0 * (1.0/6.0) * (M_PI * h) * (3*a*a + h*h) ;

    return vol_intersect;
}

bool SphericalSFC::isIntersect(const SSFC::Sphere& B_a, const SSFC::Sphere& B_b)
{
    // Check for non-intersection
    return (B_a.center - B_b.center).norm() < (B_a.radius + B_b.radius);
}

Eigen::Vector3d SphericalSFC::getIntersectionCenter(const SSFC::Sphere& B_a, const SSFC::Sphere& B_b)
{   
    Eigen::Vector3d dir_vec = B_b.center - B_a.center; 
    double r_a = B_a.radius, r_b = B_b.radius;

    double d_ctrd = (dir_vec).norm(); // scalar distance between spheres B_a and B_b 
    // Get distance to intersection along direction vector dir_vec from center of B_a to center of B_b
    double h = (r_a + r_b - d_ctrd)/2;
    double d_intxn = h + d_ctrd - r_b;

    Eigen::Vector3d pt_intxn = B_a.center + (d_intxn / d_ctrd) * dir_vec;

    return pt_intxn;
}

double SphericalSFC::getIntersectionRadius(const SSFC::Sphere& B_a, const SSFC::Sphere& B_b)
{   
    double r_a = B_a.radius, r_b = B_b.radius;

    double d_ctrd = (B_b.center - B_a.center).norm(); // scalar distance between spheres B_a and B_b 
    double h = (r_a + r_b - d_ctrd)/2;
    double intxn_radius = sqrt(r_a * r_a  - (d_ctrd - r_b + h) * (d_ctrd - r_b + h) );

    return intxn_radius;
}

void SphericalSFC::postProcessSpheres(std::vector<SSFC::Sphere>& sfc_spheres)
{
    // Check for overlap between spheres and remove them
    // For example 
    //      for 0, 1, 2. check if 0 intersects 2, if so then remove 1,
    //      then move on to check 0, 2, 3., and so on until no intersection is detected, 
    //      then move on to check index 2

    std::vector<SSFC::Sphere> sfc_spheres_proc; // post-processed

    for (size_t i = 0; i < sfc_spheres.size(); i++) // First sphere to compare from
    {
        sfc_spheres_proc.push_back(sfc_spheres[i]); 

        size_t skip = 0; // Number of next spheres to skip (i.e. remove from post processed sphere)

        for (size_t j= i + 2; j < sfc_spheres.size(); j++) // Second sphere to compare against 
        {
            if (isIntersect(sfc_spheres[i], sfc_spheres[j])){
                // We can skip the next sphere
                skip++;
            }
            else {
                // Sphere j does not intersect sphere i,
                // So we have no more spheres in between them to skip
                break;
            }
        }
        i += skip;
    }

    sfc_spheres = sfc_spheres_proc;
}

void SphericalSFC::constructSFCTrajectory(
    const std::vector<SSFC::Sphere>& sfc_spheres, 
    const Eigen::Vector3d& start_pos, 
    const Eigen::Vector3d& goal_pos, 
    SSFC::SFCTrajectory& sfc_traj)
{
    size_t num_segs = sfc_spheres.size(); // Number of segments
    size_t num_wps = sfc_spheres.size() + 1; // Number of waypoints

    /* 1: Retrieve waypoints which are the center of the intersections between the spheres */
    sfc_traj.waypoints.clear();
    sfc_traj.waypoints.resize(num_wps);
 
    sfc_traj.waypoints[0] = start_pos; // Add start state
    for (size_t i = 1; i < num_segs; i++){
        // Add intersection between 2 spheres
        sfc_traj.waypoints[i] = getIntersectionCenter(sfc_spheres[i-1], sfc_spheres[i]) ;
    }
    sfc_traj.waypoints.back() = goal_pos; // Add goal state

    /* 2: Get time allocation  */
    sfc_traj.segs_t_dur.clear();
    sfc_traj.segs_t_dur.resize(num_segs); // Vector of time durations for each segment {t_1, t_2, ..., t_M}

    if (sfc_params_.time_allocation_type == 0) // Max velocity speed time allocation
    {
        // We either use either triangle or trapezoidal time profile.
        for (size_t i = 0 ; i < num_segs; i++){
            double traj_dist = (sfc_traj.waypoints[i+1] - sfc_traj.waypoints[i]).norm();
            sfc_traj.segs_t_dur[i] = traj_dist / sfc_params_.max_vel;
        }
    }
    else if (sfc_params_.time_allocation_type == 1){ // Trapezoidal time allocation
        // d_triangle: distance travelled when reaching maximum velocity from rest with maximum acceleration.
        double d_triangle = 0.5*(sfc_params_.max_vel * sfc_params_.max_vel) / sfc_params_.max_acc;

        // We either use either triangle or trapezoidal time profile.
        for (size_t i = 0 ; i < num_segs; i++){
            double traj_dist = (sfc_traj.waypoints[i+1] - sfc_traj.waypoints[i]).norm();
            
            // For trapezoidal time profile: t_s, t_c and t_d is the time taken to 
            //  accelerate, travel at maximum velocity, and decelerate respectively.
            double t_s, t_c, t_d;

            if (2*d_triangle >= traj_dist){ // Follow triangle profile, because distance is too short to reach maximum velocity
                t_s = sqrt(traj_dist/(sfc_params_.max_acc));  //Acceleration phase
                t_c = 0.0;       // Constant maximum velocity phase
                t_d = t_s;      //Deceleration phase
            }
            else{ // Follow trapezoidal profile
                t_s = sfc_params_.max_vel / sfc_params_.max_acc; //Acceleration phase
                t_c = (traj_dist - 2*d_triangle)/sfc_params_.max_vel;  // Constant maximum velocity phase
                t_d = t_s; //Deceleration phase
            }
            sfc_traj.segs_t_dur[i] = t_s + t_c + t_d;
        }
    }
    else { // TODO: S-curve

    }

    /* 3: Get distance and vector to center of spherical cap (the intersection between 2 spheres) */
    sfc_traj.intxn_plane_vec.clear();
    sfc_traj.intxn_plane_dist.clear();
    sfc_traj.intxn_circle_radius.clear();
    sfc_traj.intxn_spheres.clear();

    sfc_traj.intxn_plane_vec.resize(num_segs-1);
    sfc_traj.intxn_plane_dist.resize(num_segs-1);
    sfc_traj.intxn_circle_radius.resize(num_segs-1);
    sfc_traj.intxn_spheres.resize(num_segs-1);

    for (size_t i = 0; i < num_segs-1; i++){
        // if (i == 0){
        //     std::cout << getIntersectionRadius(sfc_spheres[i], sfc_spheres[i+1]) << std::endl;
        // }
        sfc_traj.intxn_circle_radius[i] = getIntersectionRadius(sfc_spheres[i], sfc_spheres[i+1]);

        auto vec_to_intxn_plane = getIntersectionCenter(sfc_spheres[i], sfc_spheres[i+1]) - sfc_spheres[i].center;
        // Get vector from center of sphere to the center of the spherical cap (intersection between sphere i and i+1)
        // i.e. normal to intersection plane
        sfc_traj.intxn_plane_vec[i] = vec_to_intxn_plane.normalized() * sfc_traj.intxn_circle_radius[i];

        // Get distance from sphere center to intersection center
        sfc_traj.intxn_plane_dist[i] = vec_to_intxn_plane.norm();

        // If intersection spheres are within a buffer distance of the obstacle, 
        // Reduce the radius of the intersecting spheres to fulfill the buffer
        Eigen::Vector3d obs_pos;
        double dist_to_obs; // R: Radius of largest possible free sphere from center,
        if (grid_map_->getNearestOccupiedCell(sfc_traj.waypoints[i+1], obs_pos, dist_to_obs)){
            if (dist_to_obs <= sfc_params_.spherical_buffer){
                // Distance to obstacle is smaller than spherical buffer!
                throw std::runtime_error("SFC Intersection sphere is too close to obstacle!");
            }

            if (dist_to_obs < sfc_traj.intxn_circle_radius[i] + sfc_params_.spherical_buffer){
                sfc_traj.intxn_circle_radius[i] = dist_to_obs - sfc_params_.spherical_buffer;
            }

            if (sfc_traj.intxn_circle_radius[i] <= 0)
            {
                throw std::runtime_error("SFC Intersection sphere has negative radius!");
            }
        }

        // Construct intersection spheres for visualization
        sfc_traj.intxn_spheres[i] = SSFC::Sphere(sfc_traj.waypoints[i+1], sfc_traj.intxn_circle_radius[i]);
    }


    /* 4: Assign spheres */
    sfc_traj.spheres = sfc_spheres;
}

Eigen::Matrix<double, 3, 3> SphericalSFC::rotationAlign(
    const Eigen::Vector3d & z, const Eigen::Vector3d & d)
{
    // TODO: IF either vector is a unit vector, possibility to speed this up
    // The code below has been taken from https://iquilezles.org/articles/noacos/, where it explains
    // how to more efficiently compute a rotation matrix aligning vector z to d.

    // v: axis of rotation
    const Eigen::Vector3d v = z.cross(d);
    double c = z.dot(d);
    
    double k;
    // Detect singularity
    if ( (1.0+c) < 0.000001){
        k = 0.0;
    }
    else {
        k = 1.0f/(1.0f+c);
    }

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
    double wp_alpha = 0.5;
    double wp_radius = 0.075;

    Eigen::Vector3d line_color = Eigen::Vector3d{0.0, 0.0, 1.0};
    double line_alpha = 0.5;
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

void SphericalSFC::publishVizIntxnSpheres(
    const std::vector<SSFC::Sphere>& sfc_spheres, 
    ros::Publisher& publisher, const std::string& frame_id)  
{
    visualization_msgs::MarkerArray sfc_spheres_marker_arr;

    for (size_t i = 0; i < sfc_spheres.size(); i++){
        sfc_spheres_marker_arr.markers.push_back(
            createVizSphere(sfc_spheres[i].center, sfc_spheres[i].getDiameter(), 
                            Eigen::Vector4d{0.0, 0.0, 1.0, 0.4},
                            frame_id, i, "intxn_spheres"));
    }
    publisher.publish(sfc_spheres_marker_arr);
}

void SphericalSFC::publishVizSphericalSFC(  
    const std::vector<SSFC::Sphere>& sfc_spheres, 
    ros::Publisher& publisher, const std::string& frame_id)  
{
    visualization_msgs::MarkerArray sfc_spheres_marker_arr;

    for (size_t i = 0; i < sfc_spheres.size(); i++){
        sfc_spheres_marker_arr.markers.push_back(
            createVizSphere(sfc_spheres[i].center, sfc_spheres[i].getDiameter(), 
                            Eigen::Vector4d{0.5, 0.5, 0.5, 0.3},
                            frame_id, i, "sfc_spheres"));
    }
    publisher.publish(sfc_spheres_marker_arr);
}

visualization_msgs::Marker SphericalSFC::createArrow(
    const Eigen::Vector3d& start_pt, const Eigen::Vector3d& dir_vec, 
    const std::string& frame_id, const int& id)
{
    visualization_msgs::Marker arrow;

    arrow.ns = "sfc_samp_dir_vec";
    arrow.header.frame_id = frame_id;
    arrow.header.stamp = ros::Time::now();
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;

    double scale = 0.1;

    arrow.color.r = 0.0;
    arrow.color.g = 1.0;
    arrow.color.b = 1.0;
    arrow.color.a = 0.9;
    arrow.scale.x = scale;
    arrow.scale.y = 2 * scale;
    arrow.scale.z = 2 * scale;

    geometry_msgs::Point start, end;
    start.x = start_pt(0);
    start.y = start_pt(1);
    start.z = start_pt(2);
    end.x = start_pt(0) + dir_vec(0);
    end.y = start_pt(1) + dir_vec(1);
    end.z = start_pt(2) + dir_vec(2);

    arrow.points.clear();
    arrow.points.push_back(start);
    arrow.points.push_back(end);
    arrow.id = id;

    return arrow;
}

visualization_msgs::Marker SphericalSFC::createVizSphere( 
    const Eigen::Vector3d& center, const double& diameter, 
    const Eigen::Vector4d& color,
    const std::string& frame_id, const int& id, const std::string& ns)
{
    visualization_msgs::Marker sphere;

    sphere.ns = ns; 
    sphere.header.frame_id = frame_id;
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    sphere.pose.orientation.w = 1.0;

    

    sphere.color.r = color(0);
    sphere.color.g = color(1);
    sphere.color.b = color(2);
    sphere.color.a = color(3);
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