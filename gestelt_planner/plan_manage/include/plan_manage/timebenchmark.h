#ifndef _TIME_BENCHMARK_H_
#define _TIME_BENCHMARK_H_

#include <chrono>
#include <ctime>

#include <unordered_map>
#include <vector>

struct Stopwatch {
    bool stopwatch_running{false};
    std::clock_t cpu_time_start, cpu_time_end; // CPU ('user') Time
    std::chrono::high_resolution_clock::time_point wall_time_start, wall_time_end; // Wall ('real') time

    double cpu_time_elapsed_ms;
    double wall_time_elapsed_ms;
};

struct TimeBenchmark {
    std::unordered_map<std::string, Stopwatch> stopwatches;

    /**
     * @brief Add Stopwatches with IDs attached to them. This allows
     * us to record times for different segments of codes and retrieve
     * them by their ID later
     * 
     * @param ids 
     */
    void add_ids(std::vector<std::string> ids){
        for (auto id : ids){
            stopwatches[id] = Stopwatch(); 
        }
    }

    /**
     * @brief Start the stopwatch for a specific id
     * 
     * @param id 
     */
    void start_stopwatch(const std::string& id){
        if (!is_id_found(id)){
            return;
        }

        stopwatches[id].stopwatch_running = true;
        stopwatches[id].cpu_time_start = std::clock();
        stopwatches[id].wall_time_start = std::chrono::high_resolution_clock::now();
    }

    /**
     * @brief End the stopwatch for a specific id
     * 
     * @param id 
     */
    void stop_stopwatch(const std::string& id) {
        if (!is_id_found(id)){
            return;
        }

        if (!stopwatches[id].stopwatch_running){
            printf("Stopwatch for id %s is not running, unable to end it\n", id.c_str());
            return;
        }
        stopwatches[id].cpu_time_end = std::clock();
        stopwatches[id].wall_time_end = std::chrono::high_resolution_clock::now();
        stopwatches[id].stopwatch_running = false;

        // CPU Time in milliseconds
        stopwatches[id].cpu_time_elapsed_ms = 1000.0 * (stopwatches[id].cpu_time_end - stopwatches[id].cpu_time_start) / CLOCKS_PER_SEC;

        auto wall_time_elapsed_nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(stopwatches[id].wall_time_end - stopwatches[id].wall_time_start);
        stopwatches[id].wall_time_elapsed_ms = wall_time_elapsed_nanoseconds.count() * 1e-6;
    }

    /**
     * @brief Retrieve CPU time
     * 
     * @param id 
     */
    double get_elapsed_cpu_time(const std::string& id) {
        return stopwatches[id].cpu_time_elapsed_ms;
    }

    /**
     * @brief Retrieve Wall time
     * 
     * @param id 
     */
    double get_elapsed_wall_time(const std::string& id) {
        return stopwatches[id].wall_time_elapsed_ms;
    }
    
    /**
     * @brief Get a printable string detailing the CPU and wall time
     * 
     * @param id 
     */
    std::string get_report_string(const std::string& id){
        std::string report;

        report += "[" + id 
            + "] CPU Time: " + std::to_string(stopwatches[id].cpu_time_elapsed_ms) + "ms"
            + ", Wall Time: " + std::to_string(stopwatches[id].wall_time_elapsed_ms) + " ms";
    
        return report;
    }

    /**
     * @brief Returns true if an existing stopwatch with the specified ID is found
     * 
     * @param id 
     */
    bool is_id_found(const std::string& id){
        auto search = stopwatches.find(id);
        if (search == stopwatches.end()){
            printf("Stopwatch ID %s not found \n", id.c_str());
            return false;
        }
        return true;
    }

};

#endif // _TIME_BENCHMARK_H_