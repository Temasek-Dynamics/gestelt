#include <sfc_generation/spherical_sfc.h>

SphericalSFC::SphericalSFC(std::shared_ptr<GridMap> grid_map):
    grid_map_(grid_map)
{}   

bool SphericalSFC::generateSFC(const std::vector<Eigen::Vector3d> &path)
{
    Sphere B_cur; // current sphere being considered
    // Initialize largest sphere at initial position
    B_cur = generateOneSphere(path[0]);

    sfc_spheres_.push_back(B_cur);

    while (true)
    {
        p_h = getForwardPointOnPath(path, B_cur);
        B_cur = BatchSample(p_h, B_cur);
        
        sfc_spheres.push_back(B_cur);
        if B_cur.contains(path.back()){
            break;
        }
    }

    waypointAndTimeInitialization(sfc_spheres);

    return true;
}   

Sphere SphericalSFC::generateOneSphere(const Eigen::Vector3d& point)
{

}

Sphere SphericalSFC::getForwardPointOnPath(const std::vector<Eigen::Vector3d> &path, const Sphere& sphere)
{
    
}

Sphere SphericalSFC::BatchSample(const Eigen::Vector3d& point, const Sphere& sphere)
{
    
}

std::vector<Sphere> const SphericalSFC::getSFCWaypoints(){
    return sfc_spheres_;
}
