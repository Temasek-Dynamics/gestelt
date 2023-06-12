#include <plan_env/grid_map.h>
#include <Eigen/Eigen>
#include <limits>

using namespace Eigen;
double inf = numeric_limits<float>::infinity();

struct GridNode
{
	enum enum_state
	{
		OPENSET = 1,
		CLOSEDSET = 2,
		UNDEFINED = 3
	};

	int rounds{0}; // Distinguish every call
	enum enum_state state
	{
		UNDEFINED
	};

	Eigen::Vector3i index;

	double gScore{inf}, fScore{inf};

	GridNode * parent{NULL};
};

class PlannerBase
{
public:
  PlannerBase(){

  }

  // Adds pointer reference to gridmap
  bool addGridMap(GridMap::Ptr occ_map){
    occ_map_ = occ_map;
    init_ = true;
  }

  bool generate_plan(Vector3d start_pos, Vector3d end_pos){

    return true;
  }

  // std::vector<GridNode *> getPath() {
  // }

private: 
  bool init_;
  GridMap::Ptr occ_map_;


};