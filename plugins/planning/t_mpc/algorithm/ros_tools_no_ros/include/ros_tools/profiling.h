#ifndef ros_tools_PROFILING_H__
#define ros_tools_PROFILING_H__

#include <string>
#include <chrono>
#include <unordered_map>
#include <mutex>
#include <fstream>

#define BENCHMARKERS RosTools::Benchmarkers::get()

namespace RosTools
{
    class Benchmarker
    {
    public:
        Benchmarker(const std::string &name);

        void start();
        double stop();
        void reset();
        void cancel();

        void print();

        double getLast() const;
        double getTotalDuration() const;

        bool isRunning() const;

    private:
        std::chrono::system_clock::time_point start_time_;

        double total_duration_ = 0.0;
        double max_duration_ = -1.0;
        double min_duration_ = 99999.0;

        double last_ = -1.0;

        int total_runs_ = 0;

        std::string name_;
        bool running_ = false;
    };

    class Benchmarkers
    {
    public:
        static Benchmarkers &get()
        {
            static Benchmarkers instance;
            return instance;
        }

        Benchmarker &getBenchmarker(std::string benchmark_name)
        {
            if (_benchmarkers.find(benchmark_name) == _benchmarkers.end())
            {
                _benchmarkers.insert(std::make_pair(benchmark_name, Benchmarker(benchmark_name)));
            }
            return _benchmarkers.at(benchmark_name); // Retrieve the publisher from the map
        }

        void print()
        {
            for (auto &bm : _benchmarkers)
            {
                bm.second.print();
            }
        }

    private:
        std::unordered_map<std::string, Benchmarker> _benchmarkers;

        std::string frame_id{"map"};

        Benchmarkers() {}                                       // Private constructor to prevent instantiation
        ~Benchmarkers() {}                                      // Private destructor to prevent deletion
        Benchmarkers(const Benchmarkers &) = delete;            // Delete copy constructor
        Benchmarkers &operator=(const Benchmarkers &) = delete; // Delete assignment operator
    };

    class Timer
    {
    public:
        // Duration in s
        Timer(const double &duration = 0.);

        void setDuration(const double &duration);
        void start();

        double currentDuration();
        bool hasFinished();

    private:
        std::chrono::system_clock::time_point start_time;
        double duration_;
    };

/********** Some Fancy timing classes for profiling (from TheCherno) ***********/
#define PROFILER 1
#if PROFILER
#define PROFILE_SCOPE(name) RosTools::InstrumentationTimer timer##__LINE__(name)
#define PROFILE_FUNCTION() PROFILE_SCOPE(__FUNCTION__)
#define PROFILE_AND_LOG(debug_enable, name) \
    if (debug_enable)                       \
    {                                       \
        ROS_INFO_STREAM(name);              \
    }                                       \
    RosTools::InstrumentationTimer timer##__LINE__(name)
#else
#define PROFILE_SCOPE(name)
#define PROFILE_FUNCTION()
#endif

    struct ProfileResult
    {
        std::string Name;
        long long Start, End;
        uint32_t ThreadID;
    };

    struct InstrumentationSession
    {
        std::string Name;
    };

    class Instrumentor
    {
    private:
        InstrumentationSession *m_CurrentSession;
        std::ofstream m_OutputStream;
        int m_ProfileCount;
        std::mutex m_lock;

    public:
        Instrumentor() : m_CurrentSession(nullptr), m_ProfileCount(0) {}

        void BeginSession(const std::string &name, const std::string &filepath = "profiler.json");
        void EndSession();

        void WriteProfile(const ProfileResult &result);

        void WriteHeader();
        void WriteFooter();

        static Instrumentor &Get()
        {
            static Instrumentor *instance = new Instrumentor();
            return *instance;
        }
    };

    class InstrumentationTimer
    {
    public:
        InstrumentationTimer(const char *name);
        ~InstrumentationTimer();

        void Stop();

    private:
        const char *m_Name;
        std::chrono::system_clock::time_point m_StartTimepoint;
        bool m_Stopped;
    };

}
#endif // ros_tools_PROFILING_H__
