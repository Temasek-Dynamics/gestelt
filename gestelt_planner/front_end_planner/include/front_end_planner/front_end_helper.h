#ifndef _FRONT_END_HELPER_H_
#define _FRONT_END_HELPER_H_

#include <Eigen/Eigen>
#include <memory>

class Waypoint
{
// Waypoint class is a LIFO queue 

public:
  // Default constructor
  Waypoint(){
  }

  // Reset
  void reset(){
    wp_queue.clear();
  }
  
  /**
   * @brief Add multiple waypoints
   * 
   * @param wp 
   * @return true 
   * @return false 
   */
  bool addMultipleWP(const std::vector<Eigen::Vector3d>& wp_vec){
    // Reverse the waypoints and add on top of the stack
    std::vector<Eigen::Vector3d> wp_vec_reversed = wp_vec;
    std::reverse(wp_vec_reversed.begin(), wp_vec_reversed.end());

    for (auto wp : wp_vec_reversed){
      wp_queue.push_back(wp);
    }

    return true;
  }

  /**
   * @brief Add a single waypoint
   * 
   * @param wp 
   * @return true 
   * @return false 
   */
  bool addWP(const Eigen::Vector3d& wp){
    wp_queue.push_back(wp);
    return true;
  }

  /* Getter methods */

  /**
   * @brief Get the next waypoint
   * 
   * @return Eigen::Vector3d 
   */
  const Eigen::Vector3d& nextWP(){
    return wp_queue.back();
  }

  /**
   * @brief Get all waypoints as a vector
   * 
   * @return const Eigen::Vector3d& 
   */
  const std::vector<Eigen::Vector3d>& getQueue(){
    return wp_queue;
  }

  /**
   * @brief Get the size of the queue
   * 
   * @return Eigen::Vector3d 
   */
  size_t size() const {
    return wp_queue.size();
  }

  /**
   * @brief Get the size of the queue
   * 
   * @return Eigen::Vector3d 
   */
  bool empty() const {
    return wp_queue.empty();
  }

  /* Setter methods */

  /**
   * @brief Pop the last waypoint
   * 
   */
  void popWP() {
    if (wp_queue.empty()){
        return;
    }
    else {
      wp_queue.pop_back();
    }
  }

private:
  std::vector<Eigen::Vector3d> wp_queue;
};

template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
  int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
  if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
  auto size = static_cast<size_t>( size_s );
  std::unique_ptr<char[]> buf( new char[ size ] );
  std::snprintf( buf.get(), size, format.c_str(), args ... );
  return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

#endif // _FRONT_END_HELPER_H_