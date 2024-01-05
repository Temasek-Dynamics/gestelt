#ifndef _SFC_BASE_H_
#define _SFC_BASE_H_

#include <Eigen/Eigen>
#include <memory>

#include <grid_map/grid_map.h>

class SFCBase
{
public:
  /**
   * @brief Generate a safe flight corridor
   * 
   * @param path 
   * @return true 
   * @return false 
   */
  virtual bool generateSFC(const std::vector<Eigen::Vector3d> &path) = 0;

protected:
  SFCBase(){};
}; // class SFCBase

#endif // _SFC_BASE_H_