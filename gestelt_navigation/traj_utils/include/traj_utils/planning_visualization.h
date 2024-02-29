#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <eigen3/Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <stdlib.h>

using std::vector;
namespace ego_planner
{
  class PlanningVisualization
  {
  private:
    std::string origin_frame_;

    ros::NodeHandle node;

    ros::Publisher goal_point_pub;
    ros::Publisher global_list_pub;

    /* Initial */
    ros::Publisher initial_mjo_pub_; // Publish unconstrained coordinates
    ros::Publisher initial_mjo_q_pub_; // Publish projected coordinates q
    ros::Publisher initial_mjo_xi_pub_; // Publish unconstrained coords xi
    ros::Publisher initial_ctrl_pts_pub_; // Publish control points
    ros::Publisher initial_ctrl_pts_xi_pub_; // Publish control points in xi
    ros::Publisher initial_ctrl_pts_q_pub_; // Publish control points in q

    /* Intermediate */
    ros::Publisher intmd_ctrl_pts_q_pub_; // (q coordinates) Publish intermediate control points as they are being optimized
    ros::Publisher intmd_ctrl_pts_xi_pub_; // (xi coordinates) Publish intermediate control points as they are being optimized

    /* Optimized */
    ros::Publisher optimal_mjo_q_pub_; // Publish optimal constraint points in q space
    ros::Publisher optimal_mjo_pub_;  // Publish original MJO

    /* Failed */
    ros::Publisher failed_list_pub;


    ros::Publisher a_star_list_pub;

    ros::Publisher planner_sv_pairs_pub_;

    ros::Publisher intmd_pt0_pub;
    ros::Publisher intmd_pt1_pub;
    ros::Publisher intmd_grad0_pub;
    ros::Publisher intmd_grad1_pub;
    ros::Publisher intmd_grad_smoo_pub;
    ros::Publisher intmd_grad_static_obs_pub;
    ros::Publisher intmd_grad_feas_pub;
    ros::Publisher intmd_grad_swarm_pub;

  public:
    PlanningVisualization(/* args */) {}
    ~PlanningVisualization() {}
    PlanningVisualization(ros::NodeHandle &nh);

    typedef std::shared_ptr<PlanningVisualization> Ptr;

    // Helper functions for generating display marker data structures 

    void displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id,  bool show_sphere = true);
    void generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                  const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    
    /**
     * @brief Generate an array of arrows given a list of start-end point pairs
     * 
     * @param array 
     * @param list 
     * @param scale 
     * @param color 
     * @param id 
     */
    void generateArrowDisplayArray(visualization_msgs::MarkerArray &array, 
      const vector<Eigen::Vector3d> &list, 
      double scale, const Eigen::Vector4d& color, const int& id);

    // Publisher functions 

    void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
    void displayGlobalPathList(vector<Eigen::Vector3d> global_pts, const double scale, int id);

    /* Initial MJO */
    void displayInitialMJO(vector<Eigen::Vector3d> init_pts, const double scale, int id);
    void displayInitialMJO_q(Eigen::MatrixXd pts, int id);
    void displayInitialMJO_xi(Eigen::MatrixXd pts, int id);

    void displayInitialCtrlPts(Eigen::MatrixXd pts);
    void displayInitialCtrlPts_xi(Eigen::MatrixXd pts);
    void displayInitialCtrlPts_q(Eigen::MatrixXd pts);

    /* Optimal MJO */
    void displayOptimalMJO(Eigen::MatrixXd optimal_pts, int id);
    void displayOptimalMJO_q(Eigen::MatrixXd pts);

    /**
     * @brief Display all intermediate MJO trajectory in xi coordinates
     * 
     * @param pts 
     */
    void displayIntermediateMJO_xi(const std::vector<Eigen::MatrixXd>& trajectories); 

    /**
     * @brief Display all intermediate MJO trajectory
     * 
     * @param pts 
     */
    void displayIntermediateMJO_q(const std::vector<Eigen::MatrixXd>& trajectories); 

    // void displayMultiInitPathList(vector<vector<Eigen::Vector3d>> init_trajs, const double scale);

    void displayFailedList(Eigen::MatrixXd failed_pts, int id);
    void displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id);
    void displayArrowList(
      ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale, const Eigen::Vector4d& color, const int& id);
    
    void displayIntermediatePt(std::string type, Eigen::MatrixXd &pts, int id, Eigen::Vector4d color);
    void displayIntermediateGrad(std::string type, Eigen::MatrixXd &pts, Eigen::MatrixXd &grad, int id, Eigen::Vector4d color);

    /**
     * @brief Display static obstacle gradients
     * 
     * @param pts Starting Point (x, y, z) 
     * @param grad Direction vector (x, y, z) 
     * @param id 
     * @param color 
     */
    void pubStaticObsGrad(Eigen::MatrixXd &pts, Eigen::MatrixXd &grad, int id, Eigen::Vector4d color);

    /**
     * @brief Display {s,v} pairs generated by the ESDF-Free local planner
     * 
     * @param pts Starting Point (x, y, z) 
     * @param grad Direction vector (x, y, z) 
     * @param id 
     * @param color 
     */
    void pubSVPairs(const std::vector<Eigen::Vector3d> &pts, 
      const std::vector<Eigen::Vector3d> &grad, 
      const int& id, const Eigen::Vector4d& color);


    /* Color gradient functions */
    
    //https://stackoverflow.com/questions/22607043/color-gradient-algorithm

    /**
     * @brief Returns a sRGB value in the range [0,1]
     *        for linear input in [0,1].
     * @param x linear input in [0,1]
     * @return sRGB value in the range [0,1]
     */
    Eigen::Vector3d to_sRGB_f(const Eigen::Vector3d& c){
      Eigen::Vector3d c_new;

      for (size_t i = 0; i < c.size(); i++){
        c_new(i) = c(i) <= 0.0031308 ? (12.92 * c(i)) :  (1.055 * (pow(c(i),(1.0/2.4))) - 0.055);
      }

      return c_new;
    }

    Eigen::Vector3d to_sRGB(const Eigen::Vector3d& c){
      Eigen::Vector3d c_new;

      return 255.9999 * to_sRGB_f(c);
    }

    /**
     * @brief Returns a linear value in the range [0,1] 
     *        for sRGB input in [0,255].
     * 
     * @param c sRGB input in [0,255]
     * @return linear value in the range [0,1]
     */
    Eigen::Vector3d from_sRGB(const Eigen::Vector3d& c) {  
      Eigen::Vector3d c_new;
      for (size_t i = 0; i < c.size(); i++){
        double c_i = c(i) / 255.0;
        if (c_i <= 0.04045){
          c_new(i) = c_i / 12.92;
        }
        else{
          c_new(i) = pow(((c_i + 0.055) / 1.055), 2.4);
        }
      }
      return c_new;
    }

    /**
     * @brief Linear interpolation for 2 scalars
     * 
     * @param c1 
     * @param c2 
     * @param frac 
     * @return double 
     */
    double lerp(const double& c1, 
                const double& c2, 
                const double& frac)
    {
      return c1 * (1 - frac) + c2 * frac;
    }

    /**
     * @brief Linear interpolation for 2 3-vectors given a fraction
     * 
     * @param c1 
     * @param c2 
     * @param frac 
     * @return Eigen::Vector3d 
     */
    Eigen::Vector3d lerp(const Eigen::Vector3d& c1, 
                const Eigen::Vector3d& c2, 
                const double& frac)
    {
      Eigen::Vector3d c_new;
      for (size_t i = 0; i < c1.size(); i++){
        c_new(i) = c1(i) * (1 - frac) + c2(i) * frac;
      }

      return c_new;
    }

    /**
     * @brief Generate color based on a given range, start color1, and end color2
     * 
     * @param x 
     * @param range 
     * @param color1 
     * @param color2 
     * @return Eigen::Vector3d 
     */
    Eigen::Vector3d generateColor(
      const int& x, const int& range,
      const Eigen::Vector3d& color1, const Eigen::Vector3d& color2) 
    {
      double gamma = 0.43;
      // Eigen::Vector3d color1_lin = from_sRGB(color1);
      // Eigen::Vector3d color2_lin = from_sRGB(color2);

      double bright1 = pow(color1.sum(),gamma);
      double bright2 = pow(color2.sum(),gamma);

      double frac = double(x)/double(range);

      double intensity = pow(lerp(bright1, bright2, frac), (1.0/gamma));
      
      Eigen::Vector3d color = lerp(color1, color2, frac);

      if ( color.sum() != 0) {
        for (size_t i = 0; i < color.size(); i++)
        {
          color(i) = color(i) * intensity / color.sum();
        }
      }

      // color = to_sRGB(color);

      return color;
    }

  };
} // namespace ego_planner
#endif // _PLANNING_VISUALIZATION_H_