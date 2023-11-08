#include <iostream>
#include <ctime>
#include <random>
#include <math.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Eigen/Eigen>

#include <ros/ros.h>

struct Cylinder 
{
	double x;
	double y;
	double radius;
	double height;
	
	Cylinder(double x, double y, double radius, double height): 
		x(x), y(y), radius(radius), height(height){};
};

struct GoalRegion
{
	double min_x, max_x;
	double min_y, max_y;

	GoalRegion(double min_x, double max_x, double min_y, double max_y): 
		min_x(min_x), min_y(min_y), max_x(max_x), max_y(max_y){};
};

class DeterministicForest
{
	public:

		DeterministicForest(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
		{
			_nh.param<int>("polar_obs", _polar_obs_num, 1);
			_nh.param<int>("circle_obs", _circle_obs_num, 1);
			_nh.param<int>("seed", _seed, 1);

			_nh.param<std::string>("path", _path, "");

			_nh.param<double>("map/resolution", _resolution, 0.1);
			_nh.param<double>("map/min_distance", _min_dist, 1.0);
			_nh.param<double>("map/x_size", _x_size, 5.0);
			_nh.param<double>("map/y_size", _y_size, 5.0);
			_nh.param<double>("map/z_size", _z_size, 5.0);

			max_x_ = _x_size/2;
			max_y_ = _y_size/2;
			min_x_ = -_x_size/2;
			min_y_ = -_y_size/2;
		
			_nh.param<double>("obstacle/lower_rad", _w_l, 0.3);
			_nh.param<double>("obstacle/upper_rad", _w_h, 0.8);
			_nh.param<double>("obstacle/lower_hei", _h_l, 3.0);
			_nh.param<double>("obstacle/upper_hei", _h_h, 7.0);
			_nh.param<double>("obstacle/radius_l", radius_l_, 7.0);
			_nh.param<double>("obstacle/radius_h", radius_h_, 7.0);
			_nh.param<double>("obstacle/z_l", z_l_, 7.0);
			_nh.param<double>("obstacle/z_h", z_h_, 7.0);
			_nh.param<double>("obstacle/theta", theta_, 7.0);

			std::vector<Cylinder> cylinders;

			cylinders.push_back(Cylinder(1.25, 0.0, 0.15, 3.0));
			cylinders.push_back(Cylinder(0.0, -1.25, 0.15, 3.0));
			cylinders.push_back(Cylinder(-1.25, 0.0, 0.15, 3.0));
			cylinders.push_back(Cylinder(0.0, 1.25, 0.15, 3.0));

			DeterministicMapGenerate(cylinders);

			// Create regions where obstacles cannot be created.
			goal_regions_.push_back(GoalRegion(-1.1, 1.1, -1.1, 1.1));

			double min_x = 0.6;
			double min_y = 0.6;
			double max_x = 2.8;
			double max_y = 2.8;

			// Top left
			goal_regions_.push_back(GoalRegion(min_x, max_x, min_y, max_y));	
			// Top right
			goal_regions_.push_back(GoalRegion(min_x, max_x, -max_y, -min_y));
			// Bottom left
			goal_regions_.push_back(GoalRegion(-max_x, -min_x, min_y, max_y));
			// Bottom right
			goal_regions_.push_back(GoalRegion(-max_x, -min_x,  -max_y, -min_y));

			_x_l = -_x_size / 2.0;
			_x_h = +_x_size / 2.0;

			_y_l = -_y_size / 2.0;
			_y_h = +_y_size / 2.0;

			_polar_obs_num = std::min(_polar_obs_num, (int)_x_size * 10);
			_z_limit = _z_size;

			// unsigned int seed = rd();
			eng.seed(_seed);
			// RandomMapGenerate();

			// Generate floor, ceiling and walls
			generateFloor(_x_size, _y_size);
			generateCeiling(_x_size, _y_size, _z_size);

			generateYWall(max_x_, min_y_,  max_y_, 3.0);
			generateYWall(min_x_, min_y_,  max_y_, 3.0);

			generateXWall(max_y_,  min_x_,  max_x_,  3.0);
			generateXWall(min_y_,  min_x_,  max_x_, 3.0);

			cloud_map.width = cloud_map.points.size();
			cloud_map.height = 1;
			cloud_map.is_dense = true;

			ROS_WARN("Finished generating random map ");
		}

		~DeterministicForest(){}

		pcl::PointCloud<pcl::PointXYZ> get_pcl() 
		{
			return cloud_map;
		}

		std::string get_directory() 
		{
			return _path;
		}

		void addSquare(pcl::visualization::PCLVisualizer& viewer, double x_min, double y_min, double x_max, double y_max, const std::string& id)
		{
			viewer.addCube(x_min, y_min, x_max, y_max, 0.0, 0.1, 1.0, 0.5, 1.0, id);
		}

		void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
		{
			std::string id_base = "goal_region";
			
			int i = 0; 
			for (auto goal_region : goal_regions_){
				// ROS_INFO("Goal region: (%f, %f), (%f, %f)",  goal_region.min_x, goal_region.max_x, goal_region.max_y, goal_region.max_y);
				addSquare(viewer, goal_region.min_x, goal_region.max_x,  goal_region.min_y, goal_region.max_y, id_base + std::to_string(i));
				i++;
			}
		}

	private:

		void generateCyclinder(double x, double y, double radius, double height)
		{
			pcl::PointXYZ pt;

			// Convert ctr_x and ctr_y within resolution grid
			double ctr_x = floor(x / _resolution) * _resolution + _resolution / 2.0;
			double ctr_y = floor(y / _resolution) * _resolution + _resolution / 2.0;

			// Convert diameter to number of cells per meter
			int diameter = ceil((radius*2) / _resolution);

			for (int r = -diameter / 2.0; r < diameter / 2.0; r++)
			{
				for (int s = -diameter / 2.0; s < diameter / 2.0; s++)
				{
					int heiNum = ceil(height / _resolution);
					// Iterate through height cells
					for (int t = -10; t < heiNum; t++)
					{
						// Convert back to meters
						double temp_x = ctr_x + (r * _resolution) + 1e-2;
						double temp_y = ctr_y + (s * _resolution) + 1e-2;
						double temp_z = (t * _resolution) + 1e-2;
						// If current point is within the radius of the circle center, then add to point cloud
						if ((Eigen::Vector2d(temp_x, temp_y) - Eigen::Vector2d(ctr_x, ctr_y)).norm() <= radius)
						{
							pt.x = temp_x;
							pt.y = temp_y;
							pt.z = temp_z;
							cloud_map.points.push_back(pt);
						}
					}
				}
			}
		}

		/**
		 * @brief Generates a wall along y axis. 
		 * 
		 * @param x 
		 * @param y1 
		 * @param y2 
		 * @param height 
		 */
		void generateYWall(double x, double y1, double y2, double height)
		{
			pcl::PointXYZ pt;

			// Get length of wall in number of cells
			long length_num_cells = (y2 - y1)/_resolution;
			int height_num_cells = ceil(height / _resolution);

			// Convert points to be within resolution grid
			double x_grid = floor(x / _resolution) * _resolution + _resolution / 2.0;
			double y1_grid = floor(y1 / _resolution) * _resolution + _resolution / 2.0;

			pt.x = x_grid;

			// iterate through length of the wall along y axis
			for (int i = 0; i < length_num_cells; i++ ){
				pt.y = y1_grid + i * _resolution  + 1e-2;

				// iterate through height of the wall along z axis
				for (int t = 0; t < height_num_cells; t++){
					pt.z = (t * _resolution) + 1e-2;
					cloud_map.points.push_back(pt);
				}
			}
		}

		/**
		 * @brief Generates a wall along X axis. 
		 * 
		 * @param y
		 * @param x1 
		 * @param x2 
		 * @param height 
		 */
		void generateXWall(double y, double x1, double x2, double height)
		{
			pcl::PointXYZ pt;

			// Get length of wall in number of cells
			long length_num_cells = (x2 - x1)/_resolution;
			int height_num_cells = ceil(height / _resolution);

			// Convert points to be within resolution grid
			double y_grid = floor(y / _resolution) * _resolution + _resolution / 2.0;
			double x1_grid = floor(x1 / _resolution) * _resolution + _resolution / 2.0;

			pt.y = y_grid;

			// iterate through length of the wall along x axis
			for (int i = 0; i < length_num_cells; i++ ){
				pt.x = x1_grid + i * _resolution  + 1e-2;

				// iterate through height of the wall along z axis
				for (int t = 0; t < height_num_cells; t++){
					pt.z = (t * _resolution) + 1e-2;
					cloud_map.points.push_back(pt);
				}
			}
		}

		void generateFloor(double size_x, double size_y)
		{
			pcl::PointXYZ pt;
			pt.z = 0;

			long num_x_cells = floor(size_x / _resolution);
			long num_y_cells = floor(size_y / _resolution);

			for (int i = -num_x_cells/2; i < num_x_cells/2; i++){ // x-axis
				for (int j = -num_y_cells/2; j < num_y_cells/2; j++){ // y-axis
					// Points need to be in units of meters
					pt.x = i*_resolution + 1e-2;
					pt.y = j*_resolution + 1e-2;

					cloud_map.points.push_back(pt);
				}
			}
		}

		void generateCeiling(double size_x, double size_y, double size_z)
		{
			pcl::PointXYZ pt;
			pt.z = _z_size;

			long num_x_cells = floor(size_x / _resolution);
			long num_y_cells = floor(size_y / _resolution);

			for (int i = -num_x_cells/2; i < num_x_cells/2; i++){ // x-axis
				for (int j = -num_y_cells/2; j < num_y_cells/2; j++){ // y-axis
					// Points need to be in units of meters
					pt.x = i*_resolution + 1e-2;
					pt.y = j*_resolution + 1e-2;

					cloud_map.points.push_back(pt);
				}
			}
		}

		/**
		 * @brief Check if obstacle is in goal regions. IF true, then return false.
		 * 
		 */
		bool inGoalRegions(double x, double y){
			 for (auto goal_region : goal_regions_){
				if ((x > goal_region.min_x && x < goal_region.max_x)
					&& (y > goal_region.min_y && y < goal_region.max_y))
				{
					return true;
				}
			 }
			 return false;
		}

		/**
		 * @brief Generate map with pre-determined cylinders 
		 */
		void DeterministicMapGenerate(const std::vector<Cylinder>& cylinders)
		{
			for (auto cyl : cylinders){
				generateCyclinder(cyl.x, cyl.y, cyl.radius, cyl.height);
			}
		}

		/**
		 * @brief Generate map with random cylinders and circles
		 */
		void RandomMapGenerate()
		{
			int widNum;
			double radius;

			pcl::PointXYZ pt_random;

			// Vector of obstacle positions
			std::vector<Eigen::Vector2d> existing_obs_pos;

			rand_x = std::uniform_real_distribution<double>(_x_l, _x_h);
			rand_y = std::uniform_real_distribution<double>(_y_l, _y_h);
			rand_w = std::uniform_real_distribution<double>(_w_l, _w_h);
			rand_h = std::uniform_real_distribution<double>(_h_l, _h_h);

			rand_radius_ = std::uniform_real_distribution<double>(radius_l_, radius_h_);
			rand_radius2_ = std::uniform_real_distribution<double>(radius_l_, 1.2);
			rand_theta_ = std::uniform_real_distribution<double>(-theta_, theta_);
			rand_z_ = std::uniform_real_distribution<double>(z_l_, z_h_);

			int max_attempts = 999;
			// generate polar obs
			// For each obstacle generation
			for (int i = 0; i < _polar_obs_num && ros::ok(); i++)
			{	
				if (i > max_attempts){
					break;
				}

				double x, y, w, h, inf;
				x = rand_x(eng);
				y = rand_y(eng);
				w = rand_w(eng);

				inf = 1.0;

				// Check if pose is outside of goal regions
				if (inGoalRegions(x,y)){
					i--;
					goto next_loop;
				}

				for (auto pos : existing_obs_pos){ // iterate through each existing obstacle
					// Only allow currently generated positions at least a certain 
					// clearance away from the current position.
					if ((Eigen::Vector2d(x, y) - pos).norm() < _min_dist /*metres*/)
					{
						i--;
						goto next_loop;
					}
				}

				existing_obs_pos.push_back(Eigen::Vector2d(x, y));

				x = floor(x / _resolution) * _resolution + _resolution / 2.0;
				y = floor(y / _resolution) * _resolution + _resolution / 2.0;

				widNum = ceil((w * inf) / _resolution);
				radius = (w * inf) / 2;

				for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
				for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
				{
					h = rand_h(eng);
					int heiNum = ceil(h / _resolution);
					for (int t = 0; t < heiNum; t++)
					{
						double temp_x = x + (r + 0.5) * _resolution + 1e-2;
						double temp_y = y + (s + 0.5) * _resolution + 1e-2;
						double temp_z = (t + 0.5) * _resolution + 1e-2;
						if ((Eigen::Vector2d(temp_x, temp_y) - Eigen::Vector2d(x, y)).norm() <= radius)
						{
							pt_random.x = temp_x;
							pt_random.y = temp_y;
							pt_random.z = temp_z;
							cloud_map.points.push_back(pt_random);
						}
					}
				}

				next_loop:;
			}

			// generate circle obs
			for (int i = 0; i < _circle_obs_num; ++i)
			{
				double x, y, z;
				x = rand_x(eng);
				y = rand_y(eng);
				z = rand_z_(eng);

				x = floor(x / _resolution) * _resolution + _resolution / 2.0;
				y = floor(y / _resolution) * _resolution + _resolution / 2.0;
				z = floor(z / _resolution) * _resolution + _resolution / 2.0;

				Eigen::Vector3d translate(x, y, z);

				double theta = rand_theta_(eng);
				Eigen::Matrix3d rotate;
				rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
					1;

				double radius1 = rand_radius_(eng);
				double radius2 = rand_radius2_(eng);

				// draw a circle centered at (x,y,z)
				Eigen::Vector3d cpt;
				for (double angle = 0.0; angle < 6.282; angle += _resolution / 2)
				{
					cpt(0) = 0.0;
					cpt(1) = radius1 * cos(angle);
					cpt(2) = radius2 * sin(angle);

					// inflate
					Eigen::Vector3d cpt_if;
					for (int ifx = -0; ifx <= 0; ++ifx)
						for (int ify = -0; ify <= 0; ++ify)
						for (int ifz = -0; ifz <= 0; ++ifz)
						{
							cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
														ifz * _resolution);
							cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
							pt_random.x = cpt_if(0);
							pt_random.y = cpt_if(1);
							pt_random.z = cpt_if(2);
							cloud_map.push_back(pt_random);
						}
				}
			}
		}

		ros::NodeHandle _nh;

		std::random_device rd;
		std::default_random_engine eng;
		std::uniform_real_distribution<double> rand_x;
		std::uniform_real_distribution<double> rand_y;
		std::uniform_real_distribution<double> rand_w;
		std::uniform_real_distribution<double> rand_h;

		std::vector<double> _state;

		std::string _path;

		int _seed;

		std::vector<GoalRegion> goal_regions_;

		// Params
		double max_x_;
		double max_y_;
		double min_x_;
		double min_y_;

		int _polar_obs_num;
		double _x_size, _y_size, _z_size;
		double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
		double _z_limit, _sensing_range, _resolution, _pub_rate;
		double _min_dist;

		int _circle_obs_num;
		double radius_l_, radius_h_, z_l_, z_h_;
		double theta_;

		std::uniform_real_distribution<double> rand_radius_;
		std::uniform_real_distribution<double> rand_radius2_;
		std::uniform_real_distribution<double> rand_theta_;
		std::uniform_real_distribution<double> rand_z_;

		pcl::PointCloud<pcl::PointXYZ> cloud_map;

};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_generation");
	ros::NodeHandle _nh("~");

	DeterministicForest forest(_nh);

	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

	time (&rawtime);
	timeinfo = localtime(&rawtime);
	
	strftime(buffer,sizeof(buffer),"%d-%m-%Y_%H:%M:%S",timeinfo);
	std::string get_time(buffer);

	// std::string file_name = forest.get_directory() + get_time + ".pcd";
	std::string file_name = forest.get_directory() + "demo_map.pcd";

	printf("saved to path: %s\n", 
		file_name.c_str());

	pcl::PointCloud<pcl::PointXYZ> cloud_map = forest.get_pcl();

	pcl::io::savePCDFileASCII(
		file_name, forest.get_pcl());

	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");

	viewer.showCloud(cloud_map.makeShared());

    // viewer.runOnVisualizationThreadOnce(&DeterministicForest::viewerOneOff);
    viewer.runOnVisualizationThreadOnce(boost::bind(&DeterministicForest::viewerOneOff, &forest, _1));

	while (!viewer.wasStopped())
	{

	}

	ros::shutdown();

	return 0;
}