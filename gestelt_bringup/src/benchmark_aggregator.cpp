#include <ros/ros.h>
#include <numeric>
#include <deque>

#include <trajectory_server_msgs/TimeBenchmark.h>

struct Benchmark {
  
  std::deque<std::pair<double, double>> planner_cpu_time;
  std::deque<std::pair<double, double>> planner_wall_time;
  double avg_planner_cpu_time;
  double avg_planner_wall_time;

  std::deque<std::pair<double, double>> gridmap_update_occ_cpu_time;
  std::deque<std::pair<double, double>> gridmap_update_occ_wall_time;
  double avg_gridmap_update_occ_cpu_time;
  double avg_gridmap_update_occ_wall_time;

  // Replan frequency

  // CPU Usage

};

class BenchmarkAggregator {
public:

  void init(ros::NodeHandle& nh) {
    // TODO Add ddynamic reconfigure

    nh.param("num_drones", num_drones_, 0);
    num_drones_--; // TODO: remove this after solving the num_drones issue
    
    nh.param("time_window", time_window_, 10.);
    nh.param("compute_frequency", comp_freq_, 1.);

    std::string plan_t_bench_topic;
    nh.param("plan_time_bench_topic", plan_t_bench_topic, std::string("plan_time_benchmark"));

    // Subscribers
    for (int i = 0; i < num_drones_; i++) {
      // Add time benchmark topic subscription
      std::string plan_t_bench_topic_indiv = std::string("/drone" + std::to_string(i) + "/" + plan_t_bench_topic);

      ros::Subscriber plan_time_sub = nh.subscribe<trajectory_server_msgs::TimeBenchmark>(
        plan_t_bench_topic_indiv, 5, boost::bind(&BenchmarkAggregator::planTimeBenchCB, this, _1, i));
      plan_time_bench_sub_.push_back(plan_time_sub);

      // Allocate Benchmark objects
      benchmarks_.push_back(Benchmark());
    }

    // Publishers
    // benchmark_aggregation_pub_ = nh.advertise<<trajectory_server_msgs::BenchmarkAggregation>("/benchmark_aggregation", 10);

    // Timers
    aggregate_timer_ = nh.createTimer(ros::Duration(1/comp_freq_), &BenchmarkAggregator::aggrerateTimerCB, this);

  }

  void planTimeBenchCB(const trajectory_server_msgs::TimeBenchmark::ConstPtr &msg, int drone_id)
  {
    double now = ros::Time::now().toSec();

    // Pop out any values with timestamps not within the time window
    if (!benchmarks_[drone_id].planner_cpu_time.empty()){
      if ((now - benchmarks_[drone_id].planner_cpu_time.front().second)){
        benchmarks_[drone_id].planner_cpu_time.pop_front();
        benchmarks_[drone_id].planner_wall_time.pop_front();

        benchmarks_[drone_id].gridmap_update_occ_cpu_time.pop_front();
        benchmarks_[drone_id].gridmap_update_occ_wall_time.pop_front();
      }
    }

    // Add timestamps
    benchmarks_[drone_id].planner_cpu_time.push_back(std::make_pair(msg->planner_cpu_time, now));
    benchmarks_[drone_id].planner_wall_time.push_back(std::make_pair(msg->planner_wall_time, now));

    benchmarks_[drone_id].gridmap_update_occ_cpu_time.push_back(std::make_pair(msg->gridmap_update_occ_cpu_time, now));
    benchmarks_[drone_id].gridmap_update_occ_wall_time.push_back(std::make_pair(msg->gridmap_update_occ_wall_time, now));
  }

  // Aggreate all the benchmarks
  void aggrerateTimerCB(const ros::TimerEvent &e){
    for (int i = 0; i < num_drones_; i++) {
      
      // Get averages 
      benchmarks_[i].avg_planner_cpu_time = calculate_average(benchmarks_[i].planner_cpu_time);
      benchmarks_[i].avg_planner_wall_time = calculate_average(benchmarks_[i].planner_wall_time);

      benchmarks_[i].avg_gridmap_update_occ_cpu_time = calculate_average(benchmarks_[i].gridmap_update_occ_cpu_time);
      benchmarks_[i].avg_gridmap_update_occ_wall_time = calculate_average(benchmarks_[i].gridmap_update_occ_wall_time);
      
    }

    printBenchmarks();
  }

  double calculate_average(const std::deque<std::pair<double, double>>& time_elapsed_container){
    return std::accumulate(
      time_elapsed_container.begin(), time_elapsed_container.end(), 0.,
      [] (auto &a, auto &b) { return a + b.first;}
      ) / time_elapsed_container.size();
  }

  void printBenchmarks() {
    for (int i = 0; i < num_drones_; i++) {
      ROS_INFO("=====");
      ROS_INFO("  Planner(Local) Avg Time: CPU: %f ms, Wall: %f ms", 
        benchmarks_[i].avg_planner_cpu_time, benchmarks_[i].avg_planner_wall_time);
      ROS_INFO("  Update Occ Avg Time: CPU: %f ms, Wall: %f ms", 
        benchmarks_[i].avg_gridmap_update_occ_cpu_time, benchmarks_[i].avg_gridmap_update_occ_wall_time);
      ROS_INFO("=====");
    }
  }

private: 
  /* Params */
  int num_drones_;
  double time_window_;
  double comp_freq_;

  std::vector<ros::Subscriber> plan_time_bench_sub_; // Subscribers to planner time benchmark of all drones 

  std::vector<Benchmark> benchmarks_;

  ros::Publisher benchmark_aggregation_pub_;
  ros::Timer aggregate_timer_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bench_aggregator");
  ros::NodeHandle nh("~");

  BenchmarkAggregator bench_aggregator;

  bench_aggregator.init(nh);

  ros::spin();

  return 0;
}