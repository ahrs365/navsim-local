#include "ros_tools/profiling.h"

#include <ros_tools/logging.h>
#include <ros_tools/paths.h>

#include <algorithm>
#include <thread>

namespace RosTools
{
    Benchmarker::Benchmarker(const std::string &name)
    {
        name_ = name;
        running_ = false;
    }

    // Print results on destruct
    void Benchmarker::print()
    {
        double average_run_time = total_duration_ / ((double)total_runs_) * 1000.0;

        LOG_DIVIDER();
        LOG_VALUE("Timing of", name_);
        LOG_VALUE("Average (ms)", average_run_time);
        LOG_VALUE("Max (ms)", max_duration_ * 1000.0);
    }

    void Benchmarker::start()
    {
        running_ = true;
        start_time_ = std::chrono::system_clock::now();
    }

    void Benchmarker::cancel()
    {
        running_ = false;
    }

    double Benchmarker::stop()
    {
        if (!running_)
            return 0.0;

        auto end_time = std::chrono::system_clock::now();
        std::chrono::duration<double> current_duration = end_time - start_time_;

        if (current_duration.count() < min_duration_)
            min_duration_ = current_duration.count();

        if (current_duration.count() > max_duration_)
            max_duration_ = current_duration.count();

        total_duration_ += current_duration.count();
        total_runs_++;

        last_ = current_duration.count();
        running_ = false;
        return last_;
    }

    double Benchmarker::getLast() const { return last_; }

    double Benchmarker::getTotalDuration() const { return total_duration_; }

    void Benchmarker::reset()
    {
        total_duration_ = 0.0;
        max_duration_ = -1.0;
        min_duration_ = 99999.0;
        last_ = -1.0;
        total_runs_ = 0;
        running_ = false;
    }

    bool Benchmarker::isRunning() const { return running_; }

    Timer::Timer(const double &duration) { duration_ = duration; }

    void Timer::setDuration(const double &duration) { duration_ = duration; }
    void Timer::start() { start_time = std::chrono::system_clock::now(); }

    double Timer::currentDuration()
    {
        auto end_time = std::chrono::system_clock::now();
        std::chrono::duration<double> current_duration = end_time - start_time;

        return current_duration.count();
    }

    bool Timer::hasFinished()
    {
        auto duration = currentDuration();

        return duration >= duration_;
    }

    void Instrumentor::BeginSession(const std::string &name, const std::string &filepath)
    {
        std::string full_filepath = getPackagePath(name) + filepath;
        LOG_VALUE("Profiling Path", full_filepath);
        m_OutputStream.open(full_filepath);
        WriteHeader();
        m_CurrentSession = new InstrumentationSession{name};
    }

    void Instrumentor::EndSession()
    {
        WriteFooter();
        m_OutputStream.close();
        delete m_CurrentSession;
        m_CurrentSession = nullptr;
        m_ProfileCount = 0;
    }

    void Instrumentor::WriteProfile(const ProfileResult &result)
    {
        std::lock_guard<std::mutex> lock(m_lock);

        if (m_ProfileCount++ > 0)
            m_OutputStream << ",";

        std::string name = result.Name;
        std::replace(name.begin(), name.end(), '"', '\'');

        m_OutputStream << "{";
        m_OutputStream << "\"cat\":\"function\",";
        m_OutputStream << "\"dur\":" << (result.End - result.Start) << ',';
        m_OutputStream << "\"name\":\"" << name << "\",";
        m_OutputStream << "\"ph\":\"X\",";
        m_OutputStream << "\"pid\":0,";
        m_OutputStream << "\"tid\":" << result.ThreadID << ",";
        m_OutputStream << "\"ts\":" << result.Start;
        m_OutputStream << "}";

        m_OutputStream.flush();
    }

    void Instrumentor::WriteHeader()
    {
        m_OutputStream << "{\"otherData\": {},\"traceEvents\":[";
        m_OutputStream.flush();
    }

    void Instrumentor::WriteFooter()
    {
        m_OutputStream << "]}";
        m_OutputStream.flush();
    }

    InstrumentationTimer::InstrumentationTimer(const char *name) : m_Name(name), m_Stopped(false) { m_StartTimepoint = std::chrono::system_clock::now(); }

    InstrumentationTimer::~InstrumentationTimer()
    {
        if (!m_Stopped)
            Stop();
    }

    void InstrumentationTimer::Stop()
    {
        auto endTimepoint = std::chrono::system_clock::now();

        long long start = std::chrono::time_point_cast<std::chrono::microseconds>(m_StartTimepoint).time_since_epoch().count();
        long long end = std::chrono::time_point_cast<std::chrono::microseconds>(endTimepoint).time_since_epoch().count();

        uint32_t threadID = std::hash<std::thread::id>{}(std::this_thread::get_id());
        Instrumentor::Get().WriteProfile({m_Name, start, end, threadID});

        m_Stopped = true;
    }
}
