#ifndef UTILS_H
#define UTILS_H

#include <utility>
#include <vector>
#include <eigen3/Eigen/Core>
#include <chrono>
#include <iostream>
#include <unordered_map>

struct Point3
{
    float x, y, z;

    bool operator==(const Point3 &rhs) const
    {
        return x == rhs.x && y == rhs.y && z == rhs.z;
    }
};

struct BoundingBox
{
    Point3 min;
    Point3 max;
};

struct MAPFInstance
{
    int numAgents;
    BoundingBox env;

    std::vector<Point3> startLocs;
    std::vector<Point3> goalLocs;
};

struct Collision
{
    int agent1;
    int agent2;
    int t;

    std::pair<Eigen::Vector3f, Eigen::Vector3f> location;
};

struct Constraint
{
    int agentNum;
    int t;
    Eigen::Vector3f location;
};

class Timer
{
public:
    Timer(Timer &other) = delete; // Prevent copy constructor
    void operator=(const Timer &) = delete; // Prevent assignment

    static Timer* getInstance()
    {
        if(timer_ == nullptr)
        {
            timer_ = new Timer();
        }
        
        return timer_;
    }

    void start(std::string timer_name)
    {
        TimeStatistics &cur = named_timers_[timer_name];

        if (cur.running)
        {
            fprintf(stderr, "TIMER ERROR: Timer already started\n");
            return;
        }

        cur.start_time = std::chrono::high_resolution_clock::now();
        cur.running = true;
    }

    void stop(std::string timer_name)
    {
        if (!checkExists(timer_name))
        {
            fprintf(stderr, "TIMER ERROR: No such timer exists to stop\n");
            return;
        }

        TimeStatistics &cur = named_timers_[timer_name];

        auto diff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - cur.start_time);
        double dur_secs = diff.count() * 1e-6;

        cur.call_count += 1;
        cur.total_time += dur_secs;

        cur.max_time = dur_secs > cur.max_time ? dur_secs : cur.max_time;
        cur.min_time = dur_secs < cur.min_time ? dur_secs : cur.min_time;

        cur.running = false;
    }

    void stopAll(bool reset_timers = false)
    {
        for (const std::pair<std::string, TimeStatistics> &it : named_timers_)
        {
            stop(it.first);

            if (reset_timers)
                reset(it.first);
        }
    }

    void reset(std::string timer_name)
    {
        if (!checkExists(timer_name))
        {
            fprintf(stderr, "TIMER ERROR: No such timer exists to stop\n");
            return;
        }

        TimeStatistics &cur = named_timers_[timer_name];

        cur = TimeStatistics();
    }

    void printTimerStats(std::string timer_name, bool print_header = true)
    {
        if (!checkExists(timer_name))
        {
            fprintf(stderr, "TIMER ERROR: No such timer exists to print stats for\n");
            return;
        }

        TimeStatistics cur = named_timers_[timer_name];
        double average_time = cur.total_time / (uint64_t)cur.call_count;

        if (print_header)
        {
            printf("\n------------------------------------------\n");
            printf("TIMING STATS\n");
            printf("------------------------------------------\n");
        }
        printf("Timing stats for %s timer\n", timer_name.c_str());
        printf("Number of times called ->          %d\n", cur.call_count);
        printf("Total time             ->          %.3fs\n", cur.total_time);
        printf("Average time           ->          %.3fs\n", average_time);
        printf("Minimum time           ->          %.3fs\n", cur.min_time);
        printf("Maximum time           ->          %.3fs\n", cur.max_time);
        printf("\n------------------------------------------\n");
    }

    void printAllTimers(bool reset_timers = true)
    {

        // We want to print the header only on the first timer (makes output neater)
        bool print_header = true;
        for (const std::pair<std::string, TimeStatistics> &it : named_timers_)
        {
            printTimerStats(it.first, print_header);

            print_header = false;

            if (reset_timers)
                reset(it.first);
        }
    }

protected:
    Timer()
    {
        
    }

    static Timer* timer_;

private:
    typedef struct TimeStatistics
    {
        int call_count = 0;
        double min_time = std::numeric_limits<double>::max();
        double max_time = 0;
        double total_time = 0;

        bool running = false;
        std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    } TimeStatistics;

    bool checkExists(std::string timer_name)
    {
        return named_timers_.count(timer_name) == 1;
    }

    std::unordered_map<std::string, TimeStatistics> named_timers_;
};

#endif