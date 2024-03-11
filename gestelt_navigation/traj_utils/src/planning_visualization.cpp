#include <traj_utils/planning_visualization.h>

using std::cout;
using std::endl;
namespace ego_planner
{
  PlanningVisualization::PlanningVisualization(ros::NodeHandle &nh)
  {
    node = nh;

    nh.param("grid_map/uav_origin_frame", origin_frame_, std::string("world"));

    goal_point_pub = nh.advertise<visualization_msgs::Marker>("back_end/goal_point", 2);

    global_list_pub = nh.advertise<visualization_msgs::Marker>("global_list", 2);
    a_star_list_pub = nh.advertise<visualization_msgs::Marker>("a_star_list", 20);

    // Debugging topics
    /* Initial trajectories*/
    initial_mjo_pub_ = nh.advertise<visualization_msgs::Marker>("back_end/dbg/initial_mjo", 2);
    initial_mjo_q_pub_ = nh.advertise<visualization_msgs::Marker>("back_end/dbg/initial_mjo_q", 20);
    initial_ctrl_pts_q_pub_ = nh.advertise<visualization_msgs::Marker>("back_end/dbg/initial_ctrl_pts_q", 20);

    /* Intermediate trajectories*/
    intmd_ctrl_pts_q_pub_ = nh.advertise<visualization_msgs::Marker>("back_end/dbg/intmd_ctrl_pts_q", 20);
    intmd_ctrl_pts_xi_pub_ = nh.advertise<visualization_msgs::Marker>("back_end/dbg/intmd_ctrl_pts_xi", 20);

    /* Optimal trajectories*/
    optimal_mjo_pub_ = nh.advertise<visualization_msgs::Marker>("back_end/optimal_mjo", 2);
    optimal_ctrl_pts_q_pub_ = nh.advertise<visualization_msgs::Marker>("back_end/optimal_ctrl_pts_q", 2);
    failed_list_pub = nh.advertise<visualization_msgs::Marker>("back_end/dbg/failed_list", 2);

    /* Sphere intersection*/
    sphere_intxn_vec_pub_ = nh.advertise<visualization_msgs::MarkerArray>("sphere_intxn_vector", 2);

    // Publish (S,V) Pairs from ESDF-Free local planner
    planner_sv_pairs_pub_ = nh.advertise<visualization_msgs::MarkerArray>("planner_sv_pairs", 1000);

    /* Intermediate gradients during optimization */
    // intmd_grad0_pub = nh.advertise<visualization_msgs::MarkerArray>("grad0_dur_opt", 10);
    // intmd_grad1_pub = nh.advertise<visualization_msgs::MarkerArray>("grad1_dur_opt", 10);
    
    // grad_smoo_pub_ = nh.advertise<visualization_msgs::MarkerArray>("smoothness_grad", 10);
    // grad_feas_pub_ = nh.advertise<visualization_msgs::MarkerArray>("feasibility_grad", 10);
    // grad_swarm_pub_ = nh.advertise<visualization_msgs::MarkerArray>("swarm_grad", 10);
    grad_dist_var_pub_ = nh.advertise<visualization_msgs::MarkerArray>("dist_var_grad", 10);
    grad_agg_pos_pub_ = nh.advertise<visualization_msgs::MarkerArray>("agg_pos_grad", 10);
  }

  // // real ids used: {id, id+1000}
  void PlanningVisualization::displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                                                Eigen::Vector4d color, int id, bool show_sphere /* = true */ )
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = origin_frame_;
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 5.0;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      if (show_sphere) sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    if (show_sphere) {
      pub.publish(sphere);
    }
    pub.publish(line_strip);
  }

  // real ids used: {id, id+1}
  void PlanningVisualization::generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                                       const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = origin_frame_;
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 3;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    array.markers.push_back(sphere);
    array.markers.push_back(line_strip);
  }

  // real ids used: {1000*id ~ (arrow nums)+1000*id}
  void PlanningVisualization::generateArrowDisplayArray(
    visualization_msgs::MarkerArray &array, const vector<Eigen::Vector3d> &list, 
    double scale, const Eigen::Vector4d& color, const int& id, const std::string& ns)
  {
    visualization_msgs::Marker arrow;
    arrow.ns = ns;
    arrow.header.frame_id = origin_frame_;
    arrow.header.stamp = ros::Time::now();
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;

    // geometry_msgs::Point start, end;
    // arrow.points

    arrow.color.r = color(0);
    arrow.color.g = color(1);
    arrow.color.b = color(2);
    arrow.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    arrow.scale.x = scale;
    arrow.scale.y = 2 * scale;
    arrow.scale.z = 2 * scale;

    geometry_msgs::Point start, end;
    for (int i = 0; i < int(list.size() / 2); i++)
    {
      // arrow.color.r = color(0) / (1+i);
      // arrow.color.g = color(1) / (1+i);
      // arrow.color.b = color(2) / (1+i);

      start.x = list[2 * i](0);
      start.y = list[2 * i](1);
      start.z = list[2 * i](2);
      end.x = list[2 * i + 1](0);
      end.y = list[2 * i + 1](1);
      end.z = list[2 * i + 1](2);
      arrow.points.clear();
      arrow.points.push_back(start);
      arrow.points.push_back(end);
      arrow.id = i + id * 1000;

      array.markers.push_back(arrow);
    }
  }

  void PlanningVisualization::displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id)
  {
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = origin_frame_;
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = id;

    sphere.pose.orientation.w = 1.0;
    sphere.color.r = color(0);
    sphere.color.g = color(1);
    sphere.color.b = color(2);
    sphere.color.a = color(3);
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.pose.position.x = goal_point(0);
    sphere.pose.position.y = goal_point(1);
    sphere.pose.position.z = goal_point(2);

    goal_point_pub.publish(sphere);
  }

  void PlanningVisualization::displayGlobalPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {

    if (global_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector4d color(0, 0.5, 0.5, 1);
    displayMarkerList(global_list_pub, init_pts, scale, color, id);
  }

  void PlanningVisualization::displayIntermediateMJO_xi(
    const std::vector<Eigen::MatrixXd>& trajectories)
  {
    if (intmd_ctrl_pts_xi_pub_.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector3d color_green(0, 1, 0);
    Eigen::Vector3d color_red(1, 0, 0);

    // For each trajectory, publish it
    for (size_t i = 0; i < trajectories.size(); i++){
      
      // Determine color based on i
      // Between red and green 
      Eigen::Vector3d colorRGB = generateColor(
        i, trajectories.size(), color_green, color_red);

      double alpha = (double(i)+0.15) / double(trajectories.size());
      Eigen::Vector4d colorRGBA(colorRGB(0), colorRGB(1), colorRGB(2), alpha); 
      
      std::vector<Eigen::Vector3d> list;
      for (size_t j = 0; j < trajectories[i].cols(); j++)
      {
        Eigen::Vector3d pt = trajectories[i].col(j).transpose();
        list.push_back(pt);
      }
      displayMarkerList(intmd_ctrl_pts_xi_pub_, list, 0.03, colorRGBA, i);
    }
  }

  void PlanningVisualization::displayIntermediateMJO_q(
    const std::vector<Eigen::MatrixXd>& trajectories)
  {
    if (intmd_ctrl_pts_q_pub_.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector3d color_green(0, 1, 0);
    Eigen::Vector3d color_red(1, 0, 0);

    // For each trajectory, publish it
    for (size_t i = 0; i < trajectories.size(); i++){
      
      // Determine color based on i
      // Between red and green 
      Eigen::Vector3d colorRGB = generateColor(
        i, trajectories.size(), color_green, color_red);

      double alpha = (double(i)+0.15) / double(trajectories.size());
      Eigen::Vector4d colorRGBA(colorRGB(0), colorRGB(1), colorRGB(2), alpha); 
      
      std::vector<Eigen::Vector3d> list;
      for (size_t j = 0; j < trajectories[i].cols(); j++)
      {
        Eigen::Vector3d pt = trajectories[i].col(j).transpose();
        list.push_back(pt);
      }
      displayMarkerList(intmd_ctrl_pts_q_pub_, list, 0.03, colorRGBA, i);
    }
  }

  void PlanningVisualization::displayInitialMJO_q(Eigen::MatrixXd pts, int id)
  {
    if (initial_mjo_q_pub_.getNumSubscribers() == 0)
    {
      return;
    }
    vector<Eigen::Vector3d> list;
    for (int i = 0; i < pts.cols(); i++)
    {
      Eigen::Vector3d pt = pts.col(i).transpose();
      list.push_back(pt);
    }
    Eigen::Vector4d color(1, 0, 1, 0.5); // Purple
    displayMarkerList(initial_mjo_q_pub_, list, 0.075, color, id);
  }

  void PlanningVisualization::displayInitialMJO(
    vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {
    if (initial_mjo_pub_.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector4d color(0, 1, 0, 0.75); // Green
    displayMarkerList(initial_mjo_pub_, init_pts, scale, color, id);
  }

  void PlanningVisualization::displayInitialCtrlPts_q(Eigen::MatrixXd pts)
  {
    int id = 0;

    if (initial_ctrl_pts_q_pub_.getNumSubscribers() == 0)
    {
      return;
    }
    vector<Eigen::Vector3d> list;
    for (int i = 0; i < pts.cols(); i++)
    {
      Eigen::Vector3d pt = pts.col(i).transpose();
      list.push_back(pt);
    }
    Eigen::Vector4d color(0.8, 0.8, 0.0, 0.5); // Dirty yellow
    displayMarkerList(initial_ctrl_pts_q_pub_, list, 0.15, color, id);
  }

  void PlanningVisualization::displayOptimalMJO(Eigen::MatrixXd optimal_pts, int id)
  {
    if (optimal_mjo_pub_.getNumSubscribers() == 0)
    {
      return;
    }
    vector<Eigen::Vector3d> list;
    for (int i = 0; i < optimal_pts.cols(); i++)
    {
      Eigen::Vector3d pt = optimal_pts.col(i).transpose();
      list.push_back(pt);
    }
    Eigen::Vector4d color(1, 0, 0, 0.4);
    displayMarkerList(optimal_mjo_pub_, list, 0.1, color, id);
  }

  void PlanningVisualization::displayOptimalCtrlPts_q(Eigen::MatrixXd pts)
  {
    int id = 0;

    if (optimal_ctrl_pts_q_pub_.getNumSubscribers() == 0)
    {
      return;
    }
    vector<Eigen::Vector3d> list;
    for (int i = 0; i < pts.cols(); i++)
    {
      Eigen::Vector3d pt = pts.col(i).transpose();
      list.push_back(pt);
    }
    Eigen::Vector4d color(0, 1, 0, 1.0); // Green
    displayMarkerList(optimal_ctrl_pts_q_pub_, list, 0.15, color, id);
  }

  void PlanningVisualization::displayFailedList(Eigen::MatrixXd failed_pts, int id)
  {

    if (failed_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    vector<Eigen::Vector3d> list;
    for (int i = 0; i < failed_pts.cols(); i++)
    {
      Eigen::Vector3d pt = failed_pts.col(i).transpose();
      list.push_back(pt);
    }
    Eigen::Vector4d color(0.3, 0, 0, 1);
    displayMarkerList(failed_list_pub, list, 0.1, color, id);
  }

  void PlanningVisualization::displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id /* = Eigen::Vector4d(0.5,0.5,0,1)*/)
  {
    if (a_star_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    int i = 0;
    vector<Eigen::Vector3d> list;

    Eigen::Vector4d color = Eigen::Vector4d(0.5 + ((double)rand() / RAND_MAX / 2), 0.5 + ((double)rand() / RAND_MAX / 2), 0, 1); // make the A star pathes different every time.
    double scale = 0.05 + (double)rand() / RAND_MAX / 10;

    for (auto block : a_star_paths) // For each a_star path segment
    {
      list.clear();
      for (auto pt : block)
      {
        list.push_back(pt);
      }
      //Eigen::Vector4d color(0.5,0.5,0,1);
      displayMarkerList(a_star_list_pub, list, scale, color, id + i); // real ids used: [ id ~ id+a_star_paths.size() ]
      i++;
    }
  }

  void PlanningVisualization::displaySphereIntxnVec(
    std::vector<Eigen::Vector3d> sphere_centers, std::vector<Eigen::Vector3d> intxn_plane_vec)
  {
    visualization_msgs::MarkerArray sphereIntxnVec;

    for (size_t i = 0; i < intxn_plane_vec.size(); i++ )
    {
      sphereIntxnVec.markers.push_back(
        createArrow(sphere_centers[i], intxn_plane_vec[i], "world", "sphere_intxn_vec", i));
    }

    sphere_intxn_vec_pub_.publish(sphereIntxnVec);
  }

  visualization_msgs::Marker PlanningVisualization::createArrow(
      const Eigen::Vector3d& start_pt, const Eigen::Vector3d& dir_vec, 
      const std::string& frame_id, const std::string& ns, const int& id)
  {
    visualization_msgs::Marker arrow;

    arrow.ns = ns;
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

  void PlanningVisualization::displayArrowList(
    ros::Publisher &pub, const vector<Eigen::Vector3d> &list, 
    double scale, const Eigen::Vector4d& color, const int& id, const std::string& ns)
  {
    visualization_msgs::MarkerArray array;
    pub.publish(array); //publish to clear existing arrows

    generateArrowDisplayArray(array, list, scale, color, id, ns);

    pub.publish(array);
  }

  void PlanningVisualization::displayIntermediateGrad(
    const std::string& type, const Eigen::MatrixXd &pts, const Eigen::MatrixXd &grad)
  {
    if ( pts.cols() != grad.cols() )
    {
      ROS_ERROR("pts.cols() != grad.cols()");
      return;
    }
    std::vector<Eigen::Vector3d> arrow_pts;
    arrow_pts.reserve(pts.cols()*2);
    if ( !type.compare("swarm") )
    {
      for ( int i=0; i<pts.cols(); i++ )
      {
        arrow_pts.emplace_back(pts.col(i)); // Arrow start
        arrow_pts.emplace_back(grad.col(i)); // Arrow end
      }
    }
    else
    {
      for ( int i=0; i<pts.cols(); i++ )
      {
        arrow_pts.emplace_back(pts.col(i)); // Arrow start
        arrow_pts.emplace_back(pts.col(i)+grad.col(i)); // Arrow end
      }
    }
    
    int grad_id = 0;
    Eigen::Vector4d color;

    if ( !type.compare("smoothness") )
    {
      color = Eigen::Vector4d{1.0, 0.0, 0.0, 0.75};
      displayArrowList(grad_smoo_pub_, arrow_pts, 0.04, color, grad_id, "grad_smoothness");
    }
    else if ( !type.compare("feasiblity") )
    {
      color = Eigen::Vector4d{1.0, 0.0, 0.0, 0.75};
      displayArrowList(grad_feas_pub_, arrow_pts, 0.04, color, grad_id, "grad_feasiblity");
    }
    else if ( !type.compare("swarm") )
    {
      color = Eigen::Vector4d{1.0, 0.0, 0.0, 0.75};
      displayArrowList(grad_swarm_pub_, arrow_pts, 0.04, color, grad_id, "grad_swarm");
    }
    else if ( !type.compare("dist_variance") )
    {
      color = Eigen::Vector4d{0.0, 1.0, 0.0, 0.75};
      displayArrowList(grad_dist_var_pub_, arrow_pts, 0.025, color, grad_id, "grad_dist_variance");
    }
    else if ( !type.compare("aggregate_position") )
    {
      color = Eigen::Vector4d{1.0, 0.0, 0.0, 0.75};
      displayArrowList(grad_agg_pos_pub_, arrow_pts, 0.025, color, grad_id, "grad_dist_variance");
    }
    // else if ( !type.compare("grad0") )
    // {
    //   displayArrowList(intmd_grad0_pub, arrow_pts, 0.05, color, id);
    // }
    // else if ( !type.compare("grad1") )
    // {
    //   displayArrowList(intmd_grad1_pub, arrow_pts, 0.05, color, id);
    // }
    // else if ( !type.compare("dist") )
    // {
    //   displayArrowList(intmd_grad_static_obs_pub, arrow_pts, 0.05, color, id);
    // }
    else {
      std::cerr << "PlanningVisualization: Invalid type!" << std::endl;
    }


  }

  // void PlanningVisualization::displayIntermediateGrad(
  //   std::string type, Eigen::MatrixXd &pts, Eigen::MatrixXd &grad, int id, Eigen::Vector4d color)
  // {
  //   if ( pts.cols() != grad.cols() )
  //   {
  //     ROS_ERROR("pts.cols() != grad.cols()");
  //     return;
  //   }
  //   std::vector<Eigen::Vector3d> arrow_;
  //   arrow_.reserve(pts.cols()*2);
  //   if ( !type.compare("swarm") )
  //   {
  //     for ( int i=0; i<pts.cols(); i++ )
  //     {
  //       arrow_.emplace_back(pts.col(i)); // Arrow start
  //       arrow_.emplace_back(grad.col(i)); // Arrow end
  //     }
  //   }
  //   else
  //   {
  //     for ( int i=0; i<pts.cols(); i++ )
  //     {
  //       arrow_.emplace_back(pts.col(i)); // Arrow start
  //       arrow_.emplace_back(pts.col(i)+grad.col(i)); // Arrow end
  //     }
  //   }
    

  //   if ( !type.compare("grad0") )
  //   {
  //     displayArrowList(intmd_grad0_pub, arrow_, 0.05, color, id);
  //   }
  //   else if ( !type.compare("grad1") )
  //   {
  //     displayArrowList(intmd_grad1_pub, arrow_, 0.05, color, id);
  //   }
  //   else if ( !type.compare("dist") )
  //   {
  //     displayArrowList(intmd_grad_static_obs_pub, arrow_, 0.05, color, id);
  //   }
  //   else if ( !type.compare("smoo") )
  //   {
  //     displayArrowList(grad_smoo_pub_, arrow_, 0.05, color, id);
  //   }
  //   else if ( !type.compare("feas") )
  //   {
  //     displayArrowList(grad_feas_pub_, arrow_, 0.05, color, id);
  //   }
  //   else if ( !type.compare("swarm") )
  //   {
  //     displayArrowList(grad_swarm_pub_, arrow_, 0.02, color, id);
  //   }
  // }

  void PlanningVisualization::pubSVPairs(
    const std::vector<Eigen::Vector3d> &pts, 
    const std::vector<Eigen::Vector3d> &dir, 
    const int& id, const Eigen::Vector4d& color)
  {

    if ( pts.size() != dir.size() )
    {
      ROS_ERROR("[PlanningVisualization] pubSVPairs: pts.cols() != grad.cols()");
      return;
    }
    std::vector<Eigen::Vector3d> arrow;
    arrow.reserve(pts.size()*2);

    for ( int i=0; i < pts.size(); i++ )
    {
      arrow.emplace_back(pts[i]); // Arrow start
      arrow.emplace_back(pts[i] + dir[i]); // Arrow end
    }

    displayArrowList(planner_sv_pairs_pub_, arrow, 0.05, color, id);
  }


} // namespace ego_planner