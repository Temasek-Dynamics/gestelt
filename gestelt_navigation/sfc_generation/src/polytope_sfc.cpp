#include <sfc_generation/polytope_sfc.h>

PolytopeSFC::PolytopeSFC(std::shared_ptr<GridMap> map, const PolytopeSFCParams& sfc_params):
    map_(map), params_(sfc_params)
{}   

// Need to use eigen aligned allocator
// TODO: need to postprocess path before feeding to generateSFC
// TODO: Change poly_const_vec to poly_constr_vec
// CHANGE voxel_size to resolution

bool PolytopeSFC::generateSFC(const std::vector<Eigen::Vector3d> &path_3d)
{
    bool planning_success = true;

    std::vector<std::vector<double>> path;
    for (auto& pt3d : path_3d) {
        std::vector<double> pt_vec{pt3d(0), pt3d(1), pt3d(2)};
        path.push_back(pt_vec);
    }

    std::shared_ptr<JPS::VoxelMapUtil> map_util = ::std::make_shared<JPS::VoxelMapUtil>();

    map_->updateLocalMap();
    // Origin of local map should be set relative to UAV current position (i.e. doesn't change unless we change the local map size)
    map_util->setMap(map_->getLocalOrigin(), 
                    map_->getLocalMapNumVoxels(), 
                    map_->getData(),
                    map_->getRes());

    // define new polyhedra for visualization
    std::vector<std::vector<double>> poly_seeds_new;

    // define new contraints
    std::vector<LinearConstraint3D> poly_const_vec_new;

    // define vector for visualization of the polyhedra
    // vec_E<T> = std::vector<T, Eigen::aligned_allocator<T>>;
    std::vector<Polyhedron3D> poly_vec_new;

    // keep the polyhedra that were used in the previous optimization for the
    // trajectory generation for feasibility guarantees
    if (poly_const_vec_.size() > 0) {
        // insert poly, constraints and seed
        for (int i = 0; i < int(poly_used_idx_.size()); i++) {
            if (poly_used_idx_[i]) {
                poly_vec_new.push_back(poly_vec_[i]);
                poly_const_vec_new.push_back(poly_const_vec_[i]);
                poly_seeds_new.push_back(poly_seeds_[i]);
            }
        }
    }

    // Copy the global path and add the current state to it
    std::deque<std::vector<double>> path_curr(  path.begin(),
                                                path.end());
    // path_curr.push_front({path[0][0], path[0][1], path[0][2]});
    
    // then find segment outside the polyhedra in our poly_const_vec_new by
    // walking on the path starting from the current position
    int n_poly = poly_const_vec_new.size();

    /* generate new poly */
    // start by sampling the path until we reach a point that is outside the
    // polyhedra that already exist and that is different that the seeds of
    // these polyhedra
    size_t path_idx = 1;
    Eigen::Vector3d curr_pt(path_curr[0][0], path_curr[0][1], path_curr[0][2]); // start from start

    double voxel_size = map_util->getRes();
    Eigen::Vector3d origin = map_util->getOrigin();
    Eigen::Vector3i dim = map_util->getDim();

    /* generate new polyhedra using the global path until poly_hor_ */ 
    while (n_poly < poly_hor_) { // While num poly < max poly to consider
        Eigen::Vector3d next_pt(path_curr[path_idx][0], path_curr[path_idx][1], path_curr[path_idx][2]);
        Eigen::Vector3d diff = (next_pt - curr_pt);
        double dist_to_next_pt = diff.norm();   // Euclidean distance to next point
        double samp_dist = voxel_size / 10;     // Sampling distance

        if (dist_to_next_pt > samp_dist) {
            // sample the next point along the line between the current point and the next point
            curr_pt = curr_pt + samp_dist * diff / dist_to_next_pt;
        } else {
            // set current point to the next path point
            curr_pt = next_pt;

            // increment path index and check if the next point is the last point
            path_idx++;
            if ( path_idx == path_curr.size()) {
                // we reached the final point
                break;
            }
        }

        /* check if point is in at least one poly */
        bool inside_at_least_one_poly = false;
        for (int i = 0; i < int(poly_const_vec_new.size()); i++) { // Itr through all polytopes
            if (poly_const_vec_new[i].inside(Eigen::Vector3d(curr_pt(0), curr_pt(1), curr_pt(2)))) {
                inside_at_least_one_poly = true;
                break;
            }
        }
        // if the point is in at least one poly, continue to the next sampled point on the path
        if (inside_at_least_one_poly) {
            continue;
        }

        /* check if current point is not a previous seed */
        Eigen::Vector3i seed(int((curr_pt[0] - origin[0]) / voxel_size),
                int((curr_pt[1] - origin[1]) / voxel_size),
                int((curr_pt[2] - origin[2]) / voxel_size));

        bool is_previous_seed = false;
        for (int i = 0; i < int(poly_seeds_new.size()); i++) {
            Eigen::Vector3d seed_world{
                seed[0] * voxel_size + voxel_size / 2 + origin[0],
                seed[1] * voxel_size + voxel_size / 2 + origin[1],
                seed[2] * voxel_size + voxel_size / 2 + origin[2]
            }; // Seed in world origin
            if (seed_world(0) == poly_seeds_new[i][0] &&
                seed_world(1) == poly_seeds_new[i][1] &&
                seed_world(2) == poly_seeds_new[i][2]) {
                is_previous_seed = true;
                break;
            }
        }
        // if the seed was already used for one of the polyhedra, continue to
        // the next sampled point on the path
        if (is_previous_seed) {
            continue;
        }

        /* generate polyhedron */
        // bool use_cvx_new = toumieh_cvx_decomp_;    
        // // first check if the seed is constrained from a given direction
        // if (    (map_util->isOccupied(Eigen::Vector3i(seed[0] - 1, seed[1], seed[2])) 
        //             && map_util->isOccupied(Eigen::Vector3i(seed[0] + 1, seed[1], seed[2]))) // +- x
        //     ||  (map_util->isOccupied(Eigen::Vector3i(seed[0], seed[1] - 1, seed[2])) 
        //             && map_util->isOccupied(Eigen::Vector3i(seed[0], seed[1] + 1, seed[2]))) // +- y
        //     ||  (map_util->isOccupied(Eigen::Vector3i(seed[0], seed[1], seed[2] - 1)) 
        //             && map_util->isOccupied(Eigen::Vector3i(seed[0], seed[1], seed[2] + 1)))) // +- z
        // {
        //     cvx_decomp_type_ = true;
        // }

        // first push seed into the new poly_seeds list
        Eigen::Vector3d seed_world{
            seed[0] * voxel_size + voxel_size / 2 + origin[0],
            seed[1] * voxel_size + voxel_size / 2 + origin[1],
            seed[2] * voxel_size + voxel_size / 2 + origin[2]
        };
        poly_seeds_new.push_back({seed_world(0), seed_world(1), seed_world(2)});

        // Use point as seed for polyhedra
        // Check if use voxel grid based method or liu's method
        Polyhedron3D poly_new;
        std::vector<int8_t> grid_data = map_->getData();
        switch (cvx_decomp_type_){
            case CVXDecompType::LIU :
                // Do nothing 
                break;
            case CVXDecompType::TOUMIEH_OLD : 
                // use original method
                poly_new = convex_decomp_lib::GetPolyOcta3D(seed, grid_data, dim,
                                                            n_it_decomp_, voxel_size,
                                                            -(n_poly + 1), origin);
                break;
            case CVXDecompType::TOUMIEH_NEW :
                // if use new method
                poly_new = convex_decomp_lib::GetPolyOcta3DNew(
                    seed, grid_data, dim, n_it_decomp_, voxel_size, -(n_poly + 1),
                    origin);
                break;
        }

        // convert polyhedron to linear constraints and add it to the poly constraints vector; 
        // first gets Vec3f point and second gets Vec3f normals defining hyperplanes
        auto poly_inf = poly_new.cal_normals(); // Pair of (point on plane, normal)
        int num_planes = poly_inf.size();       // Number of planes
        // Constraint: A_poly * x - b_poly <= 0
        MatDNf<3> A_poly(num_planes, 3);        
        VecDf b_poly(num_planes);               
        for (int i = 0; i < num_planes; i++) { // For each plane
            A_poly.row(i) = poly_inf[i].second;                     // normal
            b_poly(i) = poly_inf[i].first.dot(poly_inf[i].second);  // point.dot(normal)
        }
        poly_vec_new.push_back(poly_new);
        poly_const_vec_new.push_back(LinearConstraint3D(A_poly, b_poly));

        // increment the number of polyhedra
        n_poly++;
    } // end "while (n_poly < poly_hor_)"

    // save polyhedra and seeds
    poly_vec_ = poly_vec_new;
    poly_const_vec_ = poly_const_vec_new;
    poly_seeds_ = poly_seeds_new;

    return planning_success;
}   
