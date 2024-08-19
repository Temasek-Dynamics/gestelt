#ifndef _DYNAMICVORONOI_H_
#define _DYNAMICVORONOI_H_

#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <queue>

#include "bucketedqueue.h"
#include <memory>

#include <math.h>
#include <iostream>

#include <Eigen/Eigen>

#include "nanoflann.hpp" // For nearest neighbors queries
#include "KDTreeVectorOfVectorsAdaptor.h" // For nearest neighbors queries

//! A DynamicVoronoi object computes and updates a distance map and Voronoi diagram.
class DynamicVoronoi {

public:
  struct DynamicVoronoiParams{
    double resolution{0.1};
    double origin_x{0.0};
    double origin_y{0.0};
    double origin_z{0.0};
  }; // struct DynamicVoronoiParams

public:
  
  DynamicVoronoi();
  ~DynamicVoronoi();

  DynamicVoronoi(const DynamicVoronoiParams& params)
  {
    sqrt2 = sqrt(2.0);
    data = NULL;
    gridMap = nullptr;
    alternativeDiagram = NULL;
    allocatedGridMap = false;

    res_ = params.resolution;
    origin_x_ = params.origin_x;
    origin_y_ = params.origin_y;
    origin_z_ = params.origin_z;

    origin_z_cm_ = (int) (params.origin_z * 100);
  }

  //! Initialization with an empty map
  void initializeEmpty(int _sizeX, int _sizeY, bool initGridMap=true);
  //! Initialization with a given binary map (false==free, true==occupied)
  void initializeMap(int _sizeX, int _sizeY, const std::vector<bool>& bool_map_1d_arr) ;

  //! add an obstacle at the specified cell coordinate
  void occupyCell(int x, int y);
  //! remove an obstacle at the specified cell coordinate
  void clearCell(int x, int y);
  //! remove old dynamic obstacles and add the new ones
  void exchangeObstacles(std::vector<INTPOINT>& newObstacles);

  //! update distance map and Voronoi diagram to reflect the changes
  void update(bool updateRealDist=true) ;
  //! prune the Voronoi diagram
  void prune() ;
  //! prune the Voronoi diagram by globally revisiting all Voronoi nodes. Takes more time but gives a more sparsely pruned Voronoi graph. You need to call this after every call to udpate()
  void updateAlternativePrunedDiagram();
  //! retrieve the alternatively pruned diagram. see updateAlternativePrunedDiagram()
  int** alternativePrunedDiagram() {
    return alternativeDiagram;
  };

  //! returns whether the specified cell is part of the alternatively pruned diagram. See updateAlternativePrunedDiagram.
  bool isVoronoiAlternative( const int& x, const int& y ) const;

  //! check if cell is a voronoi vertex (has at least 3 voronoi neighbours)
  bool isVoronoiVertex(int x, int y);
  //! returns whether the specified cell is part of the (pruned) Voronoi graph
  bool isVoronoi(const int& x, const int& y ) const;
  //! checks whether the specficied location is occupied
  bool isOccupied(int x, int y);
  //! write the current distance map and voronoi diagram as ppm file
  void visualize(const char* filename="result.ppm");

  /* Getter methods */

  // Get all voronoi vertices (voronoi cells that have at least 3 voronoi neighbours)
  std::vector<Eigen::Vector3d> getVoronoiVertices();

  //! retrieve the number of neighbors that are Voronoi cells (4-connected)
  int getNumVoronoiNeighbors(int x, int y);

  //! retrieve the number of neighbors that are Voronoi cells (4-connected)
  int getNumVoronoiNeighborsAlternative(int x, int y);

  //! returns the squared obstacle distance at the specified location
  int getSqrDistance( int x, int y ) {
    if( (x>0) && (x<sizeX) && (y>0) && (y<sizeY)){
      return data[x][y].sqdist; 

    } 
     else return -1;
  }
  //! returns the obstacle distance at the specified location
  float getDistance( int x, int y );


  //! returns the horizontal size of the workspace/map
  unsigned int getSizeX() {return sizeX;}
  //! returns the vertical size of the workspace/map
  unsigned int getSizeY() {return sizeY;}

private:  
  struct dataCell {
    float dist;
    int sqdist;
    char voronoi;   // voronoi status
    char queueing;
    int obstX;  // Position to nearest obstacle
    int obstY;  // Position to nearest obstacle
    bool needsRaise;
  };

  typedef enum {
    voronoiKeep=-4, 
    freeQueued = -3, 
    voronoiRetry=-2, 
    voronoiPrune=-1, 
    free=0, 
    occupied=1} State;
  typedef enum {fwNotQueued=1, fwQueued=2, fwProcessed=3, bwQueued=4, bwProcessed=1} QueueingState;
  typedef enum {invalidObstData = SHRT_MAX/2} ObstDataState;
  typedef enum {pruned, keep, retry} markerMatchResult;

  // methods
  inline void checkVoro(int x, int y, int nx, int ny, dataCell& c, dataCell& nc);
  void recheckVoro();
  void commitAndColorize(bool updateRealDist=true);
  inline void reviveVoroNeighbors(int &x, int &y);

  inline bool isOccupied(int &x, int &y, dataCell &c);
  inline markerMatchResult markerMatch(int x, int y);
  inline bool markerMatchAlternative(int x, int y);
  inline int getVoronoiPruneValence(int x, int y);

/* Exposed methods used to interface with external planners */
public:

  /* Mapping methods */
  void setObstacle(int x, int y);
  void removeObstacle(int x, int y);

  /* Planning methods */

  // Update KD Tree with all voronoi cells for query of nearest voronoi cell 
  void updateKDTree();

  // Get nearest vornoi cell given a position
  bool getNearestVoroCell(const INTPOINT& grid_pos, INTPOINT& nearest_voro_cell);

  // Get voronoi or voronoi bubble expansion neighbors
  void getVoroNeighbors(const INTPOINT& grid_pos, std::vector<Eigen::Vector3i>& neighbours, 
                        const std::unordered_set<IntPoint>& marked_bubble_cells, const bool& allow_wait=false) ;

  /* Checking methods */
  // If cell is in map
  bool isInMap(int x, int y);
  //! checks whether the specficied location is occupied
  bool isOccupied(const INTPOINT& grid_pos) const;
  //! checks whether the location is occupied
  bool isOccupied(const size_t& x, const size_t& y) const ;

  // Convert from position to index
  bool posToIdx(const DblPoint& map_pos, INTPOINT& grid_pos);

  // Convert from position to index
  void idxToPos(const INTPOINT& grid_pos, DblPoint& map_pos);

  double getOriginX() const {
    return origin_x_;
  }

  double getOriginY() const {
    return origin_y_;
  }

  double getOriginZ() const {
    return origin_z_;
  }

  double getSizeX() const {
    return sizeX;
  }

  double getSizeY() const {
    return sizeY;
  }

  // Top and bottom voronoi planes
  std::shared_ptr<DynamicVoronoi> top_voro_{nullptr};
  std::shared_ptr<DynamicVoronoi> bottom_voro_{nullptr};

private:
  // queues

  BucketPrioQueue<INTPOINT> open;
  std::queue<INTPOINT> pruneQueue;
  BucketPrioQueue<INTPOINT> sortedPruneQueue;

  std::vector<INTPOINT> removeList;
  std::vector<INTPOINT> addList;
  std::vector<INTPOINT> lastObstacles;

  // maps
  bool flip_y_{false};
  int sizeY;
  int sizeX;
  dataCell** data;
  std::shared_ptr<std::vector<std::vector<bool>>> gridMap; // 2d vector of booleans assigned upon initialization
  bool allocatedGridMap;

  // parameters
  int padding;
  double doubleThreshold;

  double sqrt2;

  //  dataCell** getData(){ return data; }
  int** alternativeDiagram;

  double res_{0.1}; // Resolution of the voronoi map 

  double origin_x_{0.0};  // in units of m
  double origin_y_{0.0};  // in units of m
  double origin_z_{0.0};  // in units of m

  int origin_z_cm_;  // in units of cm

  // Data structure
  bool init_kd_tree_{false};

  std::unique_ptr<KDTreeVectorOfVectorsAdaptor<std::vector<Eigen::Vector2i>, double>>   
    voro_kd_tree_; // KD Tree for guide path

  std::vector<Eigen::Vector2i> voro_cells_;

}; // end class DynamicVoronoi


#endif

