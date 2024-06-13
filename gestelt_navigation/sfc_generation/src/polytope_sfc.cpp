#include <sfc_generation/polytope_sfc.h>

PolytopeSFC::PolytopeSFC(std::shared_ptr<GridMap> map, const PolytopeSFCParams& sfc_params):
    map_(map), params_(sfc_params)
{}   

void PolytopeSFC::addPublishers(
    std::unordered_map<std::string, ros::Publisher> &publisher_map)
{
    poly_sfc_pub_ = publisher_map["sfc/poly"];
    ellipsoid_arr_pub_ = publisher_map["sfc/ellipsoid"];
}

void PolytopeSFC::reset()
{
    poly_vec_hyp_.clear();
    poly_vec_vtx_.clear();
    poly_vec_.clear();
    poly_seeds_.clear();
    poly_constr_vec_.clear();
}

bool PolytopeSFC::generateSFCVoxelOld(
    std::vector<Polyhedron3D, Eigen::aligned_allocator<Polyhedron3D>>& poly_vec, 
    std::vector<LinearConstraint3D>& poly_constr_vec, 
    std::vector<::std::vector<double>>& poly_seeds, 
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &path_3d,
    std::shared_ptr<GridMap> map)
{
    std::vector<std::vector<double>> path;
    for (auto& pt3d : path_3d) {
        std::vector<double> pt_vec{pt3d(0), pt3d(1), pt3d(2)};
        path.push_back(pt_vec);
    }

    std::shared_ptr<JPS::VoxelMapUtil> map_util = ::std::make_shared<JPS::VoxelMapUtil>();

    // map->updateLocalMap();
    // Origin of local map should be set relative to UAV current position (i.e. doesn't change unless we change the local map size)
    map_util->setMap(   map->getLocalOrigin(), 
                        map->getLocalMapNumVoxels(), 
                        map->getData(),
                        map->getRes());

    // define new polyhedra for visualization
    std::vector<std::vector<double>> poly_seeds_new;

    // define new contraints
    std::vector<LinearConstraint3D> poly_constr_vec_new;

    // define vector for visualization of the polyhedra
    // vec_E<T> = std::vector<T, Eigen::aligned_allocator<T>>;
    std::vector<Polyhedron3D, Eigen::aligned_allocator<Polyhedron3D>> poly_vec_new;

    // keep the polyhedra that were used in the previous optimization for the
    // trajectory generation for feasibility guarantees
    if (!poly_constr_vec_.empty()) {
        // insert poly, constraints and seed
        for (int i = 0; i < int(poly_used_idx_.size()); i++) {
            if (poly_used_idx_[i]) {
                poly_vec_new.push_back(poly_vec_[i]);
                poly_constr_vec_new.push_back(poly_constr_vec_[i]);
                poly_seeds_new.push_back(poly_seeds_[i]);
            }
        }
    }

    // Copy the global path and add the current state to it
    std::deque<std::vector<double>> path_curr( path.begin(), path.end());
    // path_curr.push_front({path[0][0], path[0][1], path[0][2]});
    
    // then find segment outside the polyhedra in our poly_constr_vec_new by
    // walking on the path starting from the current position
    int n_poly = poly_constr_vec_new.size();

    /* generate new poly */
    // start by sampling the path until we reach a point that is outside the
    // polyhedra that already exist and that is different that the seeds of
    // these polyhedra
    size_t path_idx = 1;
    Eigen::Vector3d curr_pt(path_curr[0][0], path_curr[0][1], path_curr[0][2]); // start from start

    double voxel_size = map_util->getRes();
    Eigen::Vector3d origin = map_util->getOrigin();
    Eigen::Vector3i dim = map_util->getDim();

    /* generate new polyhedra using the global path until the number of polygons exceed poly_hor */ 
    while (n_poly < params_.poly_hor) { // While num poly < max poly to consider
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
        for (int i = 0; i < int(poly_constr_vec_new.size()); i++) { // Itr through all polytopes
            if (poly_constr_vec_new[i].inside(Eigen::Vector3d(curr_pt(0), curr_pt(1), curr_pt(2)))) {
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
        // // first check if the seed is constrained from a given direction
        // if (    (map_util->isOccupied(Eigen::Vector3i(seed[0] - 1, seed[1], seed[2])) 
        //             && map_util->isOccupied(Eigen::Vector3i(seed[0] + 1, seed[1], seed[2]))) // +- x
        //     ||  (map_util->isOccupied(Eigen::Vector3i(seed[0], seed[1] - 1, seed[2])) 
        //             && map_util->isOccupied(Eigen::Vector3i(seed[0], seed[1] + 1, seed[2]))) // +- y
        //     ||  (map_util->isOccupied(Eigen::Vector3i(seed[0], seed[1], seed[2] - 1)) 
        //             && map_util->isOccupied(Eigen::Vector3i(seed[0], seed[1], seed[2] + 1)))) // +- z
        // {
        //     cvx_decomp_type = CVXDecompType::TOUMIEH_NEW;
        // }

        // first push seed into the new poly_seeds list
        Eigen::Vector3d seed_world{
            seed[0] * voxel_size + voxel_size / 2 + origin[0],
            seed[1] * voxel_size + voxel_size / 2 + origin[1],
            seed[2] * voxel_size + voxel_size / 2 + origin[2]
        };
        poly_seeds_new.push_back({seed_world(0), seed_world(1), seed_world(2)});

        // Use point as seed for polyhedra
        Polyhedron3D poly_new;
        std::vector<int8_t> grid_data = map->getData();
        poly_new = convex_decomp_lib::GetPolyOcta3D(
            seed, grid_data, dim,
            params_.n_it_decomp, voxel_size,
            -(n_poly + 1), origin);

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
        poly_constr_vec_new.push_back(LinearConstraint3D(A_poly, b_poly));

        // increment the number of polyhedra
        n_poly++;
    } // end "while (n_poly < params_.poly_hor)"

    // save polyhedra and seeds
    poly_vec = poly_vec_new;
    poly_constr_vec = poly_constr_vec_new;
    poly_seeds = poly_seeds_new;

    return true;
}

bool PolytopeSFC::generateSFCVoxelNew(
    std::vector<Polyhedron3D, Eigen::aligned_allocator<Polyhedron3D>>& poly_vec, 
    std::vector<LinearConstraint3D>& poly_constr_vec, 
    std::vector<::std::vector<double>>& poly_seeds, 
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &path_3d,
    std::shared_ptr<GridMap> map)
{
    std::vector<std::vector<double>> path;
    for (auto& pt3d : path_3d) {
        std::vector<double> pt_vec{pt3d(0), pt3d(1), pt3d(2)};
        path.push_back(pt_vec);
    }

    std::shared_ptr<JPS::VoxelMapUtil> map_util = ::std::make_shared<JPS::VoxelMapUtil>();

    // map->updateLocalMap();
    // Origin of local map should be set relative to UAV current position (i.e. doesn't change unless we change the local map size)
    map_util->setMap(   map->getLocalOrigin(), 
                        map->getLocalMapNumVoxels(), 
                        map->getData(),
                        map->getRes());

    // define new polyhedra for visualization
    std::vector<std::vector<double>> poly_seeds_new;

    // define new contraints
    std::vector<LinearConstraint3D> poly_constr_vec_new;

    // define vector for visualization of the polyhedra
    // vec_E<T> = std::vector<T, Eigen::aligned_allocator<T>>;
    std::vector<Polyhedron3D, Eigen::aligned_allocator<Polyhedron3D>> poly_vec_new;

    // keep the polyhedra that were used in the previous optimization for the
    // trajectory generation for feasibility guarantees
    if (!poly_constr_vec_.empty()) {
        // insert poly, constraints and seed
        for (int i = 0; i < int(poly_used_idx_.size()); i++) {
            if (poly_used_idx_[i]) {
                poly_vec_new.push_back(poly_vec_[i]);
                poly_constr_vec_new.push_back(poly_constr_vec_[i]);
                poly_seeds_new.push_back(poly_seeds_[i]);
            }
        }
    }

    // Copy the global path and add the current state to it
    std::deque<std::vector<double>> path_curr( path.begin(), path.end());
    // path_curr.push_front({path[0][0], path[0][1], path[0][2]});
    
    // then find segment outside the polyhedra in our poly_constr_vec_new by
    // walking on the path starting from the current position
    int n_poly = poly_constr_vec_new.size();

    /* generate new poly */
    // start by sampling the path until we reach a point that is outside the
    // polyhedra that already exist and that is different that the seeds of
    // these polyhedra
    size_t path_idx = 1;
    Eigen::Vector3d curr_pt(path_curr[0][0], path_curr[0][1], path_curr[0][2]); // start from start

    double voxel_size = map_util->getRes();
    Eigen::Vector3d origin = map_util->getOrigin();
    Eigen::Vector3i dim = map_util->getDim();

    /* generate new polyhedra using the global path until the number of polygons exceed poly_hor */ 
    while (n_poly < params_.poly_hor) { // While num poly < max poly to consider
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
        for (int i = 0; i < int(poly_constr_vec_new.size()); i++) { // Itr through all polytopes
            if (poly_constr_vec_new[i].inside(Eigen::Vector3d(curr_pt(0), curr_pt(1), curr_pt(2)))) {
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
        // // first check if the seed is constrained from a given direction
        // if (    (map_util->isOccupied(Eigen::Vector3i(seed[0] - 1, seed[1], seed[2])) 
        //             && map_util->isOccupied(Eigen::Vector3i(seed[0] + 1, seed[1], seed[2]))) // +- x
        //     ||  (map_util->isOccupied(Eigen::Vector3i(seed[0], seed[1] - 1, seed[2])) 
        //             && map_util->isOccupied(Eigen::Vector3i(seed[0], seed[1] + 1, seed[2]))) // +- y
        //     ||  (map_util->isOccupied(Eigen::Vector3i(seed[0], seed[1], seed[2] - 1)) 
        //             && map_util->isOccupied(Eigen::Vector3i(seed[0], seed[1], seed[2] + 1)))) // +- z
        // {
        //     cvx_decomp_type = CVXDecompType::TOUMIEH_NEW;
        // }

        // first push seed into the new poly_seeds list
        Eigen::Vector3d seed_world{
            seed[0] * voxel_size + voxel_size / 2 + origin[0],
            seed[1] * voxel_size + voxel_size / 2 + origin[1],
            seed[2] * voxel_size + voxel_size / 2 + origin[2]
        };
        poly_seeds_new.push_back({seed_world(0), seed_world(1), seed_world(2)});

        // Use point as seed for polyhedra
        Polyhedron3D poly_new;
        std::vector<int8_t> grid_data = map->getData();

        poly_new = convex_decomp_lib::GetPolyOcta3DNew(
            seed, grid_data, dim, 
            params_.n_it_decomp, voxel_size, 
            -(n_poly + 1), origin);

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
        poly_constr_vec_new.push_back(LinearConstraint3D(A_poly, b_poly));

        // increment the number of polyhedra
        n_poly++;
    } // end "while (n_poly < params_.poly_hor)"

    // save polyhedra and seeds
    poly_vec_ = poly_vec_new;
    poly_constr_vec_ = poly_constr_vec_new;
    poly_seeds_ = poly_seeds_new;
    
    return true;
}

bool PolytopeSFC::generateSFCLiu(   std::vector<Polyhedron3D, Eigen::aligned_allocator<Polyhedron3D>>& poly_vec,
                                    std::vector<LinearConstraint3D>& poly_constr_vec, 
                                    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &path_3d, 
                                    std::shared_ptr<GridMap> map)
{
    EllipsoidDecomp3D ellip_decomp_util_; // Decomposition util for Liu's method
    //Using ellipsoid decomposition
    ellip_decomp_util_.set_obs(map->getObsPtsVec());
    ellip_decomp_util_.set_local_bbox(Vec3f(params_.bbox_x, params_.bbox_y, params_.bbox_z)); 
    ellip_decomp_util_.dilate(path_3d); //Set max iteration number of 10, do fix the path

    //Publish visualization msgs
    decomp_ros_msgs::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(ellip_decomp_util_.get_ellipsoids());
    es_msg.header.frame_id = params_.world_frame;
    ellipsoid_arr_pub_.publish(es_msg);

    poly_vec = ellip_decomp_util_.get_polyhedrons();

    std::vector<LinearConstraint3D> poly_constr_vec_new;

    // Construct poly_constr_vec
    for (const auto& poly: poly_vec)
    {
        int num_planes = poly.vs_.size();
        // Constraint: A_poly * x - b_poly <= 0
        MatDNf<3> A_poly(num_planes, 3);        
        VecDf b_poly(num_planes);               

        for (int i = 0; i < num_planes; i++) { // For each plane
            A_poly.row(i) = poly.vs_[i].n_;                     // normal
            b_poly(i) = poly.vs_[i].p_.dot(poly.vs_[i].n_);  // point.dot(normal)
        }
        poly_constr_vec_new.push_back(LinearConstraint3D(A_poly, b_poly));

    }

    poly_constr_vec = poly_constr_vec_new;

    return true;
}

bool PolytopeSFC::generateSFC(  const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &path_3d,
                                const bool& enable_rhc_plan,                // Not used
                                const double& rhc_dist,                     // Not used
                                const Eigen::Vector3d& start_pos,           // Not used
                                const double& req_plan_time)                // Not used
{
    bool sfc_generation_success = false;
    switch (params_.cvx_decomp_type){
        case CVXDecompType::LIU :
            // Do nothing 
            sfc_generation_success = generateSFCLiu(poly_vec_, poly_constr_vec_, path_3d, map_);
            break;
        case CVXDecompType::TOUMIEH_OLD : 
            // use original method
            sfc_generation_success = generateSFCVoxelOld(poly_vec_, poly_constr_vec_, poly_seeds_, path_3d, map_);
            break;
        case CVXDecompType::TOUMIEH_NEW :
            sfc_generation_success = generateSFCVoxelNew(poly_vec_, poly_constr_vec_, poly_seeds_, path_3d, map_);
            break;
    }
    
    if (!sfc_generation_success){
        return false;
    }

    // Convert from hyperplane representation to vertex representation 
    std::vector<Eigen::MatrixX4d> poly_vec_hyp; 
    std::vector<Eigen::Matrix3Xd> poly_vec_vtx; 

    // Generate hyperplane representation f polyhedrons
    for (const auto& lin_constr: poly_constr_vec_){
        poly_vec_hyp.push_back(lin_constr.getHypMatrix());
    }

    for (size_t i = 0; i < poly_vec_hyp.size(); i++) // For each polyhedron
    {
        const Eigen::ArrayXd norms = poly_vec_hyp[i].leftCols<3>().rowwise().norm();
        poly_vec_hyp[i].array().colwise() /= norms;
    }

    if (!processCorridor(poly_vec_hyp, poly_vec_vtx))
    {
        std::cout << "[PolytopeSFC::generateSFC]: Failed to process corridor" << std::endl;
        return false;
    }
    poly_vec_hyp_ = poly_vec_hyp;
    poly_vec_vtx_ = poly_vec_vtx;

    // Publish polyhedron SFC
    publishPolyhedra(poly_vec_);

    return true;
}   


void PolytopeSFC::publishPolyhedra(
    const std::vector<Polyhedron3D, Eigen::aligned_allocator<Polyhedron3D>>& poly_vec) {
    // create polyhedra message
    decomp_ros_msgs::PolyhedronArray poly_msg = 
        DecompROS::polyhedron_array_to_ros(poly_vec);
    poly_msg.header.frame_id = params_.world_frame;

    poly_sfc_pub_.publish(poly_msg);
}