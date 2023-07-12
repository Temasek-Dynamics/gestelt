#include <ros/ros.h>
#include <numeric>
#include <deque>

#include "sys/types.h"
#include "sys/sysinfo.h"
#include "stdlib.h"
#include "stdio.h"

struct CPUUsage {
  unsigned long long lastTotalUser, lastTotalUserLow, lastTotalSys, lastTotalIdle;
  unsigned long long totalUser, totalUserLow, totalSys, totalIdle, total;

  double percent_used;
};

class HardwareMonitor {
public:

  void init(ros::NodeHandle& nh) {
    // TODO Add ddynamic reconfigure
    nh.param("compute_frequency", comp_freq_, 1.);
    nh.param("num_cpus", num_cpus_, 4);

    // Publishers
    // hardware_bench_pub_ = nh.advertise<<trajectory_server_msgs::BenchmarkAggregation>("/benchmark_aggregation", 10);

    // Timers
    aggregate_timer_ = nh.createTimer(ros::Duration(1/comp_freq_), &HardwareMonitor::aggrerateTimerCB, this);

    // Initialize reading of hardware values
    sysinfo(&memInfo);
    initCPUValues();
  }

  void initCPUValues(){
    cpu_cores_.resize(num_cpus_);

    FILE* file = fopen("/proc/stat", "r");

    fscanf(file, "cpu %llu %llu %llu %llu", &cpu_overall.lastTotalUser, &cpu_overall.lastTotalUserLow,
      &cpu_overall.lastTotalSys, &cpu_overall.lastTotalIdle);

    for (int i = 0; i < num_cpus_; i++){
      std::string cpu_id = "cpu" + std::to_string(i);
      fscanf(file, (cpu_id + std::string(" %llu %llu %llu %llu")).c_str(), &cpu_cores_[i].lastTotalUser, &cpu_cores_[i].lastTotalUserLow,
          &cpu_cores_[i].lastTotalSys, &cpu_cores_[i].lastTotalIdle);
    }
    fclose(file);
  }

  /* Timers */

  // Aggregate all the benchmarks
  void aggrerateTimerCB(const ros::TimerEvent &e){
    getRAMUsage(0);
    getCPUUsage(0);

    printBenchmarks();
  }

  void getRAMUsage(const int& drone_id){
    // Convert from kB to gB
    totalPhysMem = (double) memInfo.totalram / (1.0e9);
    physMemUsed = memInfo.totalram  - memInfo.freeram;
    physMemUsed = physMemUsed * (double) memInfo.mem_unit/ (1.0e9);
    physMemUsedPercentage = (double) physMemUsed / (double) totalPhysMem; 
  }

  void getCPUUsage(const int& drone_id){
    FILE* file;

    file = fopen("/proc/stat", "r");
    fscanf(file, "cpu %llu %llu %llu %llu", &cpu_overall.totalUser, &cpu_overall.totalUserLow,
      &cpu_overall.totalSys, &cpu_overall.totalIdle);
    updateCPUUsage(cpu_overall);

    for (int i = 0; i < num_cpus_; i++){
      std::string cpu_id = "cpu" + std::to_string(i);
      fscanf(file, (cpu_id + std::string(" %llu %llu %llu %llu")).c_str(), &cpu_cores_[i].totalUser, &cpu_cores_[i].totalUserLow,
          &cpu_cores_[i].totalSys, &cpu_cores_[i].totalIdle);
      updateCPUUsage(cpu_cores_[i]);
    }
  }

  void updateCPUUsage(CPUUsage& cu) {
    unsigned long long& totalUser = cu.totalUser;
    unsigned long long& totalUserLow = cu.totalUserLow;
    unsigned long long& totalSys = cu.totalSys;
    unsigned long long& totalIdle = cu.totalIdle;
    unsigned long long& total = cu.total;

    unsigned long long& lastTotalUser = cu.lastTotalUser;
    unsigned long long& lastTotalUserLow = cu.lastTotalUserLow;
    unsigned long long& lastTotalSys = cu.lastTotalSys;
    unsigned long long& lastTotalIdle = cu.lastTotalIdle;
    
    double & percent_used = cu.percent_used;

    if (totalUser < lastTotalUser || totalUserLow < lastTotalUserLow ||
        totalSys < lastTotalSys || totalIdle < lastTotalIdle){
        //Overflow detection. Just skip this value.
        percent_used = -1.0;
    }
    else{
        total = (totalUser - lastTotalUser) + (totalUserLow - lastTotalUserLow) +
            (totalSys - lastTotalSys);
        percent_used = total;
        total += (totalIdle - lastTotalIdle);
        percent_used /= total;
        percent_used *= 100;
    }

    lastTotalUser = totalUser;
    lastTotalUserLow = totalUserLow;
    lastTotalSys = totalSys;
    lastTotalIdle = totalIdle;
  }

  /* Helper methods */
  void printBenchmarks() {
    ROS_INFO("=====");
    ROS_INFO("  RAM Used: %.2f / %.2f (%.2f Pct)", 
      physMemUsed, totalPhysMem, physMemUsedPercentage);
    ROS_INFO("  CPU Usage overall: %.2f Pct", cpu_overall.percent_used);

    ROS_INFO("=====");
  }

private: 
  /* Params */
  double comp_freq_;
  int num_cpus_;

  // ros::Publisher benchmark_aggregation_pub_;
  ros::Timer aggregate_timer_;

  struct sysinfo memInfo;

  // CPU Usage
  unsigned long long lastTotalUser, lastTotalUserLow, lastTotalSys, lastTotalIdle;
  CPUUsage cpu_overall; // CPU Usage overall
  std::vector<CPUUsage> cpu_cores_; // CPU Usage by core

  // Memorary Usage (in GigaBytes)
  double totalPhysMem; // Total RAM currently available
  double physMemUsed; // Total RAM currently used
  double physMemUsedPercentage; // Percentage of total RAM currently used

  // Network stats
    // Bandwidth, latency, signal strength
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bench_aggregator");
  ros::NodeHandle nh("~");

  HardwareMonitor bench_aggregator;

  bench_aggregator.init(nh);

  ros::spin();

  return 0;
}