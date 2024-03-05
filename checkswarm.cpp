  class TrajContainer
  {
    GlobalTrajData global_traj; // Global trajectory from start to actual goal
    LocalTrajData local_traj; // Local trajectory from start to local target/goal
    SwarmTrajData swarm_traj;
  }

  typedef std::vector<LocalTrajData> SwarmTrajData;


  bool EGOReplanFSM::checkTrajectoryClearance(bool& obs_col, bool& swarm_col)
  {
    obs_col = false;
    swarm_col = false;

    LocalTrajData *traj = &((*swarm_minco_trajs_)[drone_id_]);
    
    if (traj->traj_id <= 0){ // Return if no local trajectory yet
      return false;
    }

    double t_cur = ros::Time::now().toSec() - traj->start_time;

    // pts_chk: Vector of constraint points. Within each piece is a vector of (timestamp, position).
    std::vector<std::vector<std::pair<double, Eigen::Vector3d>>> pts_chk = traj->pts_chk; 
    int num_segs = pts_chk.size(); // Number of segments

    // pair of (seg_idx, t_in_segment / dur_segment)
    std::pair<int, double> idx_time_ratio_pair = traj->traj.getIdxTimeRatioAtTime(t_cur);
    
    // idx_seg is starting segment index, computed by multiplying segment index and it's fraction with the number of constraints per segment
    size_t idx_seg = floor((idx_time_ratio_pair.first + idx_time_ratio_pair.second) * back_end_optimizer_->get_cps_num_perPiece_());

    if (idx_seg >= num_segs) 
    {
        // Starting segment index exceeds total number of segments
        return false;
    }

    size_t idx_pt = 0; // idx of point within constraint piece
    for (; idx_seg < num_segs; idx_seg++) // iterate through each segment
    {
      for (idx_pt = 0; idx_pt < pts_chk[idx_seg].size(); ++idx_pt) // Iterate through each point in segment
      {
        // If time of point being checked exceeds current time, check for potential collision from that particular index onwards
        if (pts_chk[idx_seg][idx_pt].first > t_cur)
        {
          goto find_ij_start;
        }
      }
    }
    
    find_ij_start:;

      for (size_t i = idx_seg; i < num_segs; ++i) // Iterate through each piece index
      {
        for (size_t j = idx_pt; j < pts_chk[i].size(); ++j) // Iterate through each point within piece index
        {
          double t = pts_chk[i][j].first;             // time
          Eigen::Vector3d pos = pts_chk[i][j].second; //position

          bool in_obs_grid = map_->getInflateOccupancy(pos); // Indicates if occupancy grid is occupied

          if (!in_obs_grid){ // If free from obstacle 
            
            for (size_t k = 0; k < (*swarm_minco_trajs_).size(); k++) // Iterate through trajectories of other agents
            {
                // Check that it's not a null pointer
                if (!(*swarm_minco_trajs_)[k]){
                    logError(str_fmt("Swarm agent %d has empty trajectory", k))
                    continue;
                }
                LocalTrajData *swarm_k_traj = &((*swarm_minco_trajs_)[k]);

                if ((swarm_k_traj->drone_id == drone_id_))
                {
                    continue;
                }

                // Skip own trajectory or if drone ID of trajectory does not match desired ID
                // Or if the trajectory of other drones are not planned yet
                if ((swarm_k_traj->drone_id != (int)k))
                {
                    logError(str_fmt("Swarm agent %d's trajectory does not match it's prescribed ID %d", k, swarm_k_traj->drone_id))
                    continue;
                }

                // Calculate time for other drone
                double t_X = t - (traj->start_time - swarm_k_traj->start_time);
                // If time t_X is valid
                if (t_X > 0 && t_X < swarm_k_traj->duration) 
                {
                    Eigen::Vector3d agent_predicted_pos = swarm_k_traj->traj.getPos(t_X);
                    double inter_agent_dist = (pos - agent_predicted_pos).norm();

                    if (inter_agent_dist < swarm_clearance_)
                    {
                    logWarn(str_fmt("Clearance between drones %d and %d is %f, too close!",
                            drone_id_, (int)id, inter_agent_dist));

                    swarm_col = ((t - t_cur) < time_to_col_threshold_);
                    return true;
                    }
                }
            }
          }
          else {
            obs_col = true;
            return true;
          }

        }
        idx_pt = 0; // reset to start of path
      }

      return EMPTY_E;
  }