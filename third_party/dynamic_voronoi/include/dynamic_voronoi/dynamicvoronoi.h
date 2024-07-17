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
  }

  //! Initialization with an empty map
  void initializeEmpty(int _sizeX, int _sizeY, bool initGridMap=true);
  //! Initialization with a given binary map (false==free, true==occupied)
  void initializeMap(int _sizeX, int _sizeY, std::shared_ptr<std::vector<std::vector<bool>>> _gridMap) ;

  //! add an obstacle at the specified cell coordinate
  void occupyCell(int x, int y) ;
  //! remove an obstacle at the specified cell coordinate
  void clearCell(int x, int y) ;
  //! remove old dynamic obstacles and add the new ones
  void exchangeObstacles(std::vector<INTPOINT>& newObstacles) ;

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
  //! retrieve the number of neighbors that are Voronoi nodes (4-connected)
  int getNumVoronoiNeighborsAlternative(int x, int y);
  //! returns whether the specified cell is part of the alternatively pruned diagram. See updateAlternativePrunedDiagram.
  bool isVoronoiAlternative( const int& x, const int& y ) const;


  int getSqrDistance( int x, int y ) {
    //  if( (x>0) && (x<sizeX) && (y>0) && (y<sizeY)) 
    return data[x][y].sqdist; 
    //  else return -1;
  }
  //! returns the obstacle distance at the specified location
  float getDistance( int x, int y );
  //! returns whether the specified cell is part of the (pruned) Voronoi graph
  bool isVoronoi(const int& x, const int& y ) const;
  //! checks whether the specficied location is occupied
  bool isOccupied(int x, int y);
  //! write the current distance map and voronoi diagram as ppm file
  void visualize(const char* filename="result.ppm");

  //! returns the horizontal size of the workspace/map
  unsigned int getSizeX() {return sizeX;}
  //! returns the vertical size of the workspace/map
  unsigned int getSizeY() {return sizeY;}

private:  
  struct dataCell {
    float dist;
    int sqdist;
    char voronoi;
    char queueing;
    int obstX;
    int obstY;
    bool needsRaise;
  };

  typedef enum {voronoiKeep=-4, freeQueued = -3, voronoiRetry=-2, voronoiPrune=-1, free=0, occupied=1} State;
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
  // Expand a voronoi bubble with all cells within the bubble filled as free space
  // void expandVoronoiBubble(const INTPOINT& grid_pos);

  // Gets 8-connected neighbours
  // void getNeighbors(const INTPOINT& grid_pos, std::vector<INTPOINT>& neighbours);

  // Gets 8-connected neighbours in voronoi diagram
  void getVoroNeighbors(const INTPOINT& grid_pos, std::vector<INTPOINT>& neighbours,
                        const INTPOINT& goal_grid_pos);

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
    return origin_z_;
  }

  double getOriginY() const {
    return origin_z_;
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
  std::shared_ptr<std::vector<std::vector<bool>>> gridMap;
  bool allocatedGridMap;

  // parameters
  int padding;
  double doubleThreshold;

  double sqrt2;

  //  dataCell** getData(){ return data; }
  int** alternativeDiagram;

  double res_{0.1}; // Resolution of the voronoi map 

  double origin_x_{0.0};
  double origin_y_{0.0};
  double origin_z_{0.0};
};


#endif

