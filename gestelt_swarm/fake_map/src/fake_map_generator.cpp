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

struct ObsFreeSquare
{
	double min_x, max_x;
	double min_y, max_y;

	ObsFreeSquare(double min_x, double max_x, double min_y, double max_y): 
		min_x(min_x), min_y(min_y), max_x(max_x), max_y(max_y){};

	bool isInObsFree(const double& x, const double& y){
		return (x > this->min_x && x < this->max_x)
					&& (y > this->min_y && y < this->max_y); 
	}

};

struct ObsFreeRadial
{ // Encapsulates a circular region where obstacles cannot be placed
	double x, y, radius;

	ObsFreeRadial(const double& x, const double& y, const double radius): 
		x(x), y(y), radius(radius) {};

	bool isInObsFree(const double& x, const double& y){
		return fabs(x - this->x) < radius && fabs(y - this->y) < radius; 
	}
};

class DeterministicForest
{
	public:

		DeterministicForest(ros::NodeHandle &nodeHandle) : nh_(nodeHandle)
		{
			nh_.param<int>("polar_obs", _polar_obs_num, 1);
			nh_.param<int>("circle_obs", _circle_obs_num, 1);
			nh_.param<int>("seed", _seed, 1);

			nh_.param<std::string>("filename", filename_, "new_map");
			nh_.param<std::string>("path", file_path_, "");

			nh_.param<double>("map/resolution", _resolution, 0.05);
			nh_.param<double>("map/inter_obstacle_clearance", inter_obs_clearance_, 1.0);
			nh_.param<double>("map/x_size", x_size_, 5.0);
			nh_.param<double>("map/y_size", y_size_, 5.0);
			nh_.param<double>("map/z_size", z_size_, 5.0);

			nh_.param<bool>("map/enable_floor", enable_floor_, false);
			nh_.param<bool>("map/enable_ceiling", enable_ceiling_, false);
			nh_.param<bool>("map/enable_walls", enable_walls_, false);

			max_x_ = x_size_/2;
			max_y_ = y_size_/2;
			min_x_ = -x_size_/2;
			min_y_ = -y_size_/2;
		
			// Params for cylinders
			nh_.param<double>("obstacle/cylinder/lower_rad", _w_l, 0.3);
			nh_.param<double>("obstacle/cylinder/upper_rad", _w_h, 0.8);
			nh_.param<double>("obstacle/cylinder/lower_hei", _h_l, 3.0);
			nh_.param<double>("obstacle/cylinder/upper_hei", _h_h, 7.0);
			// Params for circles
			nh_.param<double>("obstacle/circle/radius_l", radius_l_, 7.0);
			nh_.param<double>("obstacle/circle/radius_h", radius_h_, 7.0);
			nh_.param<double>("obstacle/circle/z_l", z_l_, 7.0);
			nh_.param<double>("obstacle/circle/z_h", z_h_, 7.0);
			nh_.param<double>("obstacle/circle/theta", theta_, 7.0);

			_x_l = -x_size_ / 2.0;
			_x_h = +x_size_ / 2.0;

			_y_l = -y_size_ / 2.0;
			_y_h = +y_size_ / 2.0;

			_polar_obs_num = std::min(_polar_obs_num, (int)x_size_ * 10);

			// // Create regions where obstacles cannot be created.
			// obs_free_radial_.push_back(ObsFreeRadial(0.0, 0.0, 0.1));

			// We can either:
			// A1) Generate a test map for the vicon room
			// generateViconTest();

			// A2) Generate a antipodal map for the vicon room
			// generateViconTestAntipodal();

			// B) Generate a random forest map for benchmarking
			// unsigned int seed = rd();
			eng.seed(_seed);
			RandomMapGenerate();

			// Add floor, ceiling and walls if enabled
			if (enable_floor_){
				generateHorizontalPlane(x_size_, y_size_, 0.0);
			}
			if (enable_ceiling_){
				generateHorizontalPlane(x_size_, y_size_, z_size_);
			}
			if (enable_walls_){
				genWalls(x_size_, y_size_, z_size_);
			}

			// Generate pre-determined cylinders
			std::vector<Cylinder> cylinders;

			double a = 1.75;

			cylinders.push_back(Cylinder(a, 0.0, 0.15, 3.0));
			cylinders.push_back(Cylinder(0.0, -a, 0.15, 3.0));
			cylinders.push_back(Cylinder(-a, 0.0, 0.15, 3.0));
			cylinders.push_back(Cylinder(0.0, a, 0.15, 3.0));
			cylinders.push_back(Cylinder(0.3, 0.0, 0.15, 3.0));

			generatePredeterminedCyclinders(cylinders);

			// C) Generate a tunnel for benchmarking
			// generateRectangularTunnel(Eigen::Vector3d{0.0, 0.0, 0.0}, 10.0, 2.0, 2.0);

			// D) Generate a narrow window (1m x 1m) for benchmarking 
			// double win_side_length = 1.0;
			// double win_top_btm_height= 1.0;
			// double win_length = 1.0;
			// double win_height = 1.0;

			// // Side wall along window
			// generateWall(Eigen::Vector2d{0.0, 0.0}, Eigen::Vector2d{0.0, win_side_length}, 
			// 	0.0, 2 * win_top_btm_height + win_height);
			// generateWall(Eigen::Vector2d{0.0, win_side_length + win_length}, Eigen::Vector2d{0.0, 2*win_side_length + win_length}, 
			// 	0.0, 2 * win_top_btm_height + win_height);

			// // Top and bottom wall of window
			// generateWall(Eigen::Vector2d{0.0, win_side_length}, Eigen::Vector2d{0.0, win_side_length + win_length}, 
			// 	0.0, win_top_btm_height);
			// generateWall(Eigen::Vector2d{0.0, win_side_length}, Eigen::Vector2d{0.0, win_side_length + win_length}, 
			// 	win_top_btm_height + win_height, 2 * win_top_btm_height + win_height);

			// // Generate floor
			// generateHorizontalPlane(20.0, 20.0, 0.0);	

			// // E) Generate anti-podal map
			// double a = 2.0;
			// double b = 2.0;
			// double buffer_rad = 0.3;

			// // Add all 4 origin positions of the drones + 0.5 buffer
			// obs_free_radial_.push_back(ObsFreeRadial(a+b, a+b, b + buffer_rad));
			// obs_free_radial_.push_back(ObsFreeRadial(a+b, -a-b, b + buffer_rad));
			// obs_free_radial_.push_back(ObsFreeRadial(-a-b, -a-b, b + buffer_rad));
			// obs_free_radial_.push_back(ObsFreeRadial(-a-b, a+b, b + buffer_rad));

			// eng.seed(_seed);
			// RandomMapGenerate();
			// generateHorizontalPlane(x_size_, y_size_, 0.0); // Add floor
			// // generateHorizontalPlane(x_size_, y_size_, z_size_); // Add ceiling

			// F) Generate forest map
			// eng.seed(_seed);
			// RandomMapGenerate();
			// generateHorizontalPlane(x_size_, y_size_, 0.0); // Add floor
			// generateHorizontalPlane(x_size_, y_size_, z_size_); // Add ceiling

			cloud_map.width = cloud_map.points.size();
			cloud_map.height = 1;
			cloud_map.is_dense = true;

			ROS_WARN("Finished generating random map ");
		}

		~DeterministicForest(){}

		void generateViconTest(){
			// Create regions where obstacles cannot be created.
			obs_free_squares_.push_back(ObsFreeSquare(-1.1, 1.1, -1.1, 1.1));

			double min_x = 0.6;
			double min_y = 0.6;
			double max_x = 2.8;
			double max_y = 2.8;

			// Top left
			obs_free_squares_.push_back(ObsFreeSquare(min_x, max_x, min_y, max_y));	
			// Top right
			obs_free_squares_.push_back(ObsFreeSquare(min_x, max_x, -max_y, -min_y));
			// Bottom left
			obs_free_squares_.push_back(ObsFreeSquare(-max_x, -min_x, min_y, max_y));
			// Bottom right
			obs_free_squares_.push_back(ObsFreeSquare(-max_x, -min_x,  -max_y, -min_y));


			std::vector<Cylinder> cylinders;

			cylinders.push_back(Cylinder(1.25, 0.0, 0.15, 3.0));
			cylinders.push_back(Cylinder(0.0, -1.25, 0.15, 3.0));
			cylinders.push_back(Cylinder(-1.25, 0.0, 0.15, 3.0));
			cylinders.push_back(Cylinder(0.0, 1.25, 0.15, 3.0));

			generatePredeterminedCyclinders(cylinders);
		}

		// Generate antipodal swap map for 6 drones
		void generateViconTestAntipodal(){
			// No obstacle space
			double buffer_rad = 0.3;
			for (size_t i = 0; i < 6; i++){ 
				Eigen::Matrix3d rot_mat = Eigen::AngleAxisd((M_PI/180.0) * i * 60, Eigen::Vector3d::UnitZ()).matrix();

				Eigen::Vector3d drone_pos = rot_mat * Eigen::Vector3d{2.4, 0, 0};
				obs_free_radial_.push_back(ObsFreeRadial(drone_pos(0), drone_pos(1), buffer_rad));

				std::cout << "drone_pos: " << drone_pos.transpose() << std::endl;
			}

			std::vector<Cylinder> cylinders;

			cylinders.push_back(Cylinder(1.25, 0.0, 0.15, 3.0));
			cylinders.push_back(Cylinder(0.0, -1.25, 0.15, 3.0));
			cylinders.push_back(Cylinder(-1.25, 0.0, 0.15, 3.0));
			cylinders.push_back(Cylinder(0.0, 1.25, 0.15, 3.0));

			generatePredeterminedCyclinders(cylinders);
		}


		/**
		 * @brief Generate a rectangular tunnel 
		 * 
		 */
		void generateRectangularTunnel(
			const Eigen::Vector3d& start_pt, 
			const double& length, const double& width, const double& height){

			Eigen::Vector3d end_pt = start_pt + Eigen::Vector3d{length, width, 0.0};

			// Generate tunnel floor
			generateHorizontalPlane(
				Eigen::Vector2d{start_pt(0),start_pt(1)}, Eigen::Vector2d{end_pt(0),end_pt(1)}, 0.0);

			// Generate tunnel ceiling
			generateHorizontalPlane(
				Eigen::Vector2d{start_pt(0),start_pt(1)}, Eigen::Vector2d{end_pt(0),end_pt(1)}, height);

			/* Generate tunnel walls */
			// Right wall (keep y1 constant, and change x)
			generateWall(Eigen::Vector2d{start_pt(0),start_pt(1)}, Eigen::Vector2d{end_pt(0),start_pt(1)}, 0.0, height);

			// Left wall (keep y2 constant, and change x)
			generateWall(Eigen::Vector2d{start_pt(0),end_pt(1)}, Eigen::Vector2d{end_pt(0),end_pt(1)}, 0.0, height);

		}

		void genWalls(const double& x_size, const double& y_size, const double& z_size)
		{
			generateYWall(x_size/2, 	-y_size/2,  	y_size/2, 	z_size);
			generateYWall(-x_size/2, 	-y_size/2,  	y_size/2, 	z_size);

			generateXWall(y_size/2,  	-x_size/2,  	x_size/2,  	z_size);
			generateXWall(-y_size/2,  	-x_size/2,  	x_size/2, 	z_size);
		}

		void genCeilingFloorWalls(	const double& x_size, const double& y_size, const double& z_size, 
									const double& min_x_, const double& max_x_,
									const double& min_y_, const double& max_y_)
		{
			// Generate floor, ceiling and walls
			generateHorizontalPlane(x_size, y_size, 0.0);
			generateHorizontalPlane(x_size, y_size, z_size);

			generateYWall(max_x_, min_y_,  max_y_, z_size);
			generateYWall(min_x_, min_y_,  max_y_, z_size);

			generateXWall(max_y_,  min_x_,  max_x_,  z_size);
			generateXWall(min_y_,  min_x_,  max_x_, z_size);
		}

		pcl::PointCloud<pcl::PointXYZ> get_pcl() 
		{
			return cloud_map;
		}

		std::string get_directory() const
		{
			return file_path_;
		}


		std::string get_filename() const
		{
			return filename_;
		}
		

		void addSquareViz(pcl::visualization::PCLVisualizer& viewer, double x_min, double y_min, double x_max, double y_max, const std::string& id)
		{
			viewer.addCube(x_min, y_min, x_max, y_max, 0.0, 0.1, 1.0, 0.5, 1.0, id);
		}

		void addSphereViz(pcl::visualization::PCLVisualizer& viewer, double x, double y, double radius, const std::string& id)
		{
			// pcl::Point x, y, 0.0
			// viewer.addSphere(center, radius, 0.0, 0.1, 1.0, 0.5, 1.0, id);
		}

		void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
		{
			std::string id_base = "goal_region";
			
			int i = 0; 
			for (auto goal_region : obs_free_squares_){
				// ROS_INFO("Goal region: (%f, %f), (%f, %f)",  goal_region.min_x, goal_region.max_x, goal_region.max_y, goal_region.max_y);
				addSquareViz(viewer, goal_region.min_x, goal_region.max_x,  goal_region.min_y, goal_region.max_y, id_base + std::to_string(i));
				i++;
			}

			for (auto goal_region : obs_free_radial_){
				// ROS_INFO("Goal region: (%f, %f), (%f, %f)",  goal_region.min_x, goal_region.max_x, goal_region.max_y, goal_region.max_y);
				addSphereViz(viewer, goal_region.x, goal_region.y, 1.0, id_base + std::to_string(i));
				i++;
			}
		}

	private:

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
				if (isInObsFreeRegion(x,y)){
					i--; // retry current iteration
					goto next_loop;
				}

				for (auto pos : existing_obs_pos){ // iterate through each existing obstacle
					// Only allow currently generated positions at least a certain 
					// clearance away from the current position.
					if ((Eigen::Vector2d(x, y) - pos).norm() < inter_obs_clearance_ /*metres*/)
					{
						i--; // retry current iteration
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

		/* METHODS FOR GENERATION OF PRIMITIVES */

		/**
		 * @brief Generate a cylinder
		 * 
		 * @param x 
		 * @param y 
		 * @param radius 
		 * @param height 
		 */
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
					for (int t = 0; t < heiNum; t++)
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

		void generateWall(const Eigen::Vector2d& start_pt, const Eigen::Vector2d& end_pt, const double& start_z, const double& height)
		{
			pcl::PointXYZ pt;

			long num_z_cells =  floor((height - start_z)/ _resolution);

			int x0 = floor(start_pt(0)/_resolution);
			int y0 = floor(start_pt(1)/_resolution);

			int x1 = floor(end_pt(0)/_resolution);
			int y1 = floor(end_pt(1)/_resolution);

			int dx = abs(x1 - x0);
			int sx = x0 < x1 ? 1 : -1;
			int dy = -abs(y1 - y0);
			int sy = y0 < y1 ? 1 : -1;
			int error = dx + dy;
			
			// We need to iterate through cell space
			while (true){
				pt.x = x0 * _resolution;
				pt.y = y0 * _resolution;
				for (int k = 0; k < num_z_cells; k++){
					pt.z = start_z + k * _resolution + 1e-2;
					cloud_map.points.push_back(pt);
				}

				if (x0 == x1 && y0 == y1) {
					break;
				}
				int e2 = 2 * error;
				if (e2 >= dy){
					if (x0 == x1) {
						break;
					}
					error = error + dy;
					x0 = x0 + sx;
				}
				if (e2 <= dx){
					if (y0 == y1) {
						break;
					}
					error = error + dx;
					y0 = y0 + sy;
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

		void generateHorizontalPlane(const Eigen::Vector2d& start_pt, const Eigen::Vector2d& end_pt, const double& z_height)
		{
			pcl::PointXYZ pt;
			pt.z = z_height;

			long num_x_cells = floor((end_pt(0) - start_pt(0)) / _resolution);
			long num_y_cells = floor((end_pt(1) - start_pt(1)) / _resolution);

			for (int i = 0; i < num_x_cells; i++){ // x-axis
				// Points need to be in units of meters
				pt.x = start_pt(0) + i * _resolution + 1e-2;
				for (int j = 0; j < num_y_cells; j++){ // y-axis
					pt.y = start_pt(1) + j * _resolution + 1e-2;

					cloud_map.points.push_back(pt);
				}
			}
			
		}

		/**
		 * @brief Add a horizontal plane of predefined size and height
		 * 
		 * @param size_x 
		 * @param size_y 
		 * @param size_z 
		 */
		void generateHorizontalPlane(double size_x, double size_y, double z_height)
		{
			pcl::PointXYZ pt;
			pt.z = z_height;

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
		 * @brief Generate map with pre-determined cylinders 
		 */
		void generatePredeterminedCyclinders(const std::vector<Cylinder>& cylinders)
		{
			for (auto cyl : cylinders){
				generateCyclinder(cyl.x, cyl.y, cyl.radius, cyl.height);
			}
		}

		/**
		 * @brief Check if obstacle is in goal regions. IF true, then return false.
		 * 
		 */
		bool isInObsFreeRegion(double x, double y){
			 for (auto obs_free_region : obs_free_squares_){
				if ( obs_free_region.isInObsFree(x, y))
				{
					return true;
				}
			 }

			 for (auto obs_free_region : obs_free_radial_){
				if (obs_free_region.isInObsFree(x, y))
				{
					return true;
				}
			 }

			 return false;
		}


		ros::NodeHandle nh_;

		std::random_device rd;
		std::default_random_engine eng;
		std::uniform_real_distribution<double> rand_x;
		std::uniform_real_distribution<double> rand_y;
		std::uniform_real_distribution<double> rand_w;
		std::uniform_real_distribution<double> rand_h;

		std::vector<double> _state;

		std::string file_path_;
		std::string filename_;

		int _seed;

		std::vector<ObsFreeSquare> obs_free_squares_;
		std::vector<ObsFreeRadial> obs_free_radial_;

		// Params
		double max_x_;
		double max_y_;
		double min_x_;
		double min_y_;

		int _polar_obs_num;
		double x_size_, y_size_, z_size_;
		double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
		double _sensing_range, _resolution, _pub_rate;
		double inter_obs_clearance_; // Clearance between obstacles

		bool enable_floor_, enable_ceiling_, enable_walls_;

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
	ros::NodeHandle nh_("~");

	DeterministicForest forest(nh_);

	pcl::PointCloud<pcl::PointXYZ> cloud_map = forest.get_pcl();

	// time_t rawtime;
	// struct tm * timeinfo;
	// char buffer[80];
	// time (&rawtime);
	// timeinfo = localtime(&rawtime);
	// strftime(buffer,sizeof(buffer),"%d-%m-%Y_%H:%M:%S",timeinfo);
	// std::string get_time(buffer);
	// std::string file_name = forest.get_directory() + get_time + ".pcd";

	std::string file_path = forest.get_directory() + forest.get_filename() + ".pcd";
	pcl::io::savePCDFileASCII(file_path, cloud_map);
	printf("saved to path: %s\n",  file_path.c_str());

	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(cloud_map.makeShared());

	// viewer.runOnVisualizationThreadOnce(&DeterministicForest::viewerOneOff);
	viewer.runOnVisualizationThreadOnce(boost::bind(&DeterministicForest::viewerOneOff, &forest, _1));

	while (!viewer.wasStopped() && ros::ok())
	{
		ros::spin();
	}

	ros::shutdown();

	return 0;
}