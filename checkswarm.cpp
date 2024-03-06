  class TrajContainer
  {
    GlobalTrajData global_traj; // Global trajectory from start to actual goal
    LocalTrajData local_traj; // Local trajectory from start to local target/goal
    SwarmTrajData swarm_traj;
  }

  typedef std::vector<LocalTrajData> SwarmTrajData;


