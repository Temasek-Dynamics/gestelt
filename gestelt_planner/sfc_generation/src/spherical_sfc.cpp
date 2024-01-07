#include <sfc_generation/spherical_sfc.h>

SphericalSFC::SphericalSFC(std::shared_ptr<GridMap> grid_map, const int& max_itr):
    grid_map_(grid_map), max_itr_(max_itr)
{
    max_sample_points_ = 10000;
    weight_cand_vol_ = 0.5;
    weight_intersect_vol_ = 0.5;
}   

void SphericalSFC::addVizPublishers(ros::Publisher& p_cand_viz_pub, 
    ros::Publisher& dist_viz_pub, ros::Publisher& sfc_spherical_viz_pub)
{
    p_cand_viz_pub_ = p_cand_viz_pub;
    dist_viz_pub_ = dist_viz_pub;
    sfc_spherical_viz_pub_ = sfc_spherical_viz_pub;
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
    while (itr_ < max_itr_)
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
        // if B_cur.contains(path.back()){ // If current sphere contains the goal
        //     break;
        // }

        itr_++;
    }

    publishVizSphericalSFC(sfc_spheres_, "world", sfc_spherical_viz_pub_);

    // if !(sfc_spheres_.back().contains(path.back())){ 
    //     std::cout << "Final safe flight corridor does not contain the goal" << std::endl;
    //     return false;
    // }

    // if (itr_ > max_itr_){
    //     std::cout << "[SphericalSFC] Maximum iterations " << max_itr_ << " exceeded. Consumed " << itr << " iterations." << std::endl;
    //     return false;
    // }

    // waypointAndTimeInitialization(sfc_spheres_);

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
    B_cand_pq_ = std::priority_queue<std::shared_ptr<Sphere>, std::vector<std::shared_ptr<Sphere>>, Sphere::CompareCostPtr>();

    /* Debugging */
    std::vector<Eigen::Vector3d> p_cand_vec;

    // Calculate orientation and standard deviation of normal sampling distribution
    Eigen::Vector3d dir_vec = B_cur.center - p_guide;

    // Normalize the dir_vec 
    Eigen::Vector3d dir_vec_norm = dir_vec.normalized();

    // TODO: Should we take the standard dev of the normalized or unnormalized vector?
    // Get standard deviation along direction vector
    double stddev_x = 0.5 * dir_vec.norm();
    Eigen::Vector3d stddev{stddev_x, 0.5*stddev_x, 0.5*stddev_x};

    // Calculate orientation of distribution ellipsoid
    Eigen::Matrix<double, 3, 3> ellipse_rot_mat = rotationAlign(Eigen::Vector3d::UnitX(), dir_vec_norm);
    Eigen::Quaterniond ellipse_orientation(ellipse_rot_mat);

    // Set up seed for sampler
    uint64_t seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    sampler_.setSeed(seed);
    sampler_.setParams(p_guide, stddev);
    
    // Sample the points first then rotate them
    for (int k = 0; k < max_sample_points_; k++){
        /* Debugging */
        p_cand_vec.push_back(sampler_.sample());
    }

    for (auto& p_cand: p_cand_vec){
        std::shared_ptr<Sphere> B_cand = std::make_shared<Sphere>();
        // TODO: Transform point 
        // p_cand = ellipse_rot_mat * p_cand;

        // Generate candidate sphere, calculate score and add to priority queue
        generateFreeSphere(p_cand, *B_cand);
        B_cand->score = computeCandSphereScore(*B_cand, B_cur);

        B_cand_pq_.push(B_cand);
    }

    // Publish candidate points and 3d distribution visualization
    publishVizPoints(p_cand_vec, "world", p_cand_viz_pub_); 
    dist_viz_pub_.publish( createVizEllipsoid(p_guide, stddev, ellipse_orientation, "world", itr_));

    if (!B_cand_pq_.empty()){
        B_cur = *B_cand_pq_.top();
        return true;
    } 

    return false;
}

double SphericalSFC::computeCandSphereScore(Sphere& B_cand, Sphere& B_prev)
{   
    double V_cand = B_cand.getVolume();
    double V_intersect = getIntersectingVolume(B_cand, B_prev);

    return weight_cand_vol_ * V_cand + weight_intersect_vol_ * V_intersect;
}

double SphericalSFC::getIntersectingVolume(Sphere& B_a, Sphere& B_b)
{
    return 0.0;
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

void SphericalSFC::publishVizPoints(const std::vector<Eigen::Vector3d>& pts, const std::string& frame_id, ros::Publisher& publisher)
{
  visualization_msgs::Marker sphere_list;
  double radius = 0.025;
  double alpha = 0.5;

  sphere_list.header.frame_id = frame_id;
  sphere_list.header.stamp = ros::Time::now();
  sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;
  sphere_list.action = visualization_msgs::Marker::ADD;
  sphere_list.ns = "spherical_sfc_cand_pts"; 
  sphere_list.id = 1; 
  sphere_list.pose.orientation.w = 1.0;

  sphere_list.color.r = 1.0;
  sphere_list.color.g = 1.0;
  sphere_list.color.b = 1.0;
  sphere_list.color.a = alpha;

  sphere_list.scale.x = radius;
  sphere_list.scale.y = radius;
  sphere_list.scale.z = radius;

  geometry_msgs::Point pt;
  for (int i = 1; i < pts.size() - 1; i++){
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