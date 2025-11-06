#include "ros_tools/data_saver.h"

#include <iomanip>
#include <filesystem>

namespace RosTools
{

    void DoubleDataSet::AddData(const double &value)
    {
        num_entries_++;
        data_.push_back(value);
    }

    void DoubleDataSet::SaveData(std::ofstream &file)
    {
        // Print the number of entries
        file << name_.c_str() << ": " << 1 << " " << std::to_string(num_entries_) << "\n";

        for (size_t i = 0; i < data_.size(); i++)
        {
            file << std::fixed << std::setprecision(12) << data_[i] << "\n";
        }
    }

    void DoubleDataSet::Clear()
    {
        num_entries_ = 0;
        data_.clear();
    }

    void PointDataSet::AddData(const Eigen::Vector2d &value)
    {
        num_entries_++;
        data_.push_back(value);
    }

    void PointDataSet::SaveData(std::ofstream &file)
    {
        // Print the number of entries
        file << name_.c_str() << ": " << 2 << " " << std::to_string(num_entries_) << "\n";

        for (size_t i = 0; i < data_.size(); i++)
        {
            file << std::fixed << std::setprecision(12) << data_[i](0) << " " << data_[i](1) << "\n";
        }
    }
    void PointDataSet::Clear()
    {
        num_entries_ = 0;
        data_.clear();
    }

    DataSaver::DataSaver(double size, bool add_timestamp)
    {
        datasets_.reserve(size);
        add_timestamp_ = add_timestamp;
    }

    void DataSaver::ReadDataFromFile(std::ifstream &import_file, std::map<std::string, std::vector<double>> &result)
    {
        ReadSingleDataFromFile(import_file, result);
    }

    void DataSaver::ReadDataFromFile(std::ifstream &import_file, std::map<std::string, std::vector<int>> &result)
    {
        ReadSingleDataFromFile(import_file, result);
    }

    void DataSaver::ReadDataFromFile(std::ifstream &import_file, std::map<std::string, std::vector<Eigen::Vector2d>> &result)
    {
        double x, y;
        std::string data_name;
        int data_size, num_entries;

        while (import_file >> data_name >> data_size >> num_entries)
        {
            data_name = data_name.substr(0, data_name.find(":")); // Remove the colon

            result[data_name].reserve(num_entries);
            for (int i = 0; i < num_entries; i++)
            {
                import_file >> x;
                import_file >> y;
                result[data_name].push_back(Eigen::Vector2d(x, y));
            }
        }
    }

    void DataSaver::SaveData(const std::string &file_name)
    {
        std::string path = getPackagePath("ros_tools") + "/data";
        SaveData(path, file_name);
    }

    std::string DataSaver::getFilePath(const std::string &file_path, const std::string &file_name, bool create_folder)
    {
        // Create directories if they do not exist
        std::string complete_file_path = file_path + "/" + file_name;
        std::string folder_path = complete_file_path.substr(0, complete_file_path.rfind("/"));

        if (create_folder)
        {
            if (std::filesystem::create_directories(folder_path))
                LOG_INFO("Data Saver: Creating Directory Path: " << folder_path);
        }

        std::string full_file_path;
        if (add_timestamp_)
        {
            if (!timestamp_set)
            {
                start_time_ = std::chrono::system_clock::now();
                timestamp_set = true;
            }

            auto tt = std::chrono::system_clock::to_time_t(start_time_);
            auto local_time = *localtime(&tt);

            std::string datestamp = ParseYear(local_time.tm_year) + "_";
            datestamp += ValueWithZero(local_time.tm_mon + 1) + "_";
            datestamp += ValueWithZero(local_time.tm_mday);

            std::string timestamp = "";
            timestamp += ValueWithZero(local_time.tm_hour);
            timestamp += ValueWithZero(local_time.tm_min);

            full_file_path = complete_file_path + "_" + datestamp + "-" + timestamp + ".txt";
        }
        else
        {
            full_file_path = complete_file_path + ".txt";
        }

        return full_file_path;
    }

    // Use the given path
    void DataSaver::SaveData(const std::string &file_path, const std::string &file_name)
    {
        std::string full_file_path = getFilePath(file_path, file_name, true);

        // Setup a file stream
        std::ofstream export_file;

        LOG_INFO("Data Saver: Saving collected data in " << full_file_path);

        export_file.open(full_file_path);

        for (auto &dataset : datasets_)
            dataset->SaveData(export_file);

        export_file << "-1\n";

        // Close the file
        export_file.close();
    }

    std::string DataSaver::ParseYear(int value) { return "20" + std::to_string(value).erase(0, 1); }

    std::string DataSaver::ValueWithZero(int value)
    {
        if (value < 10)
            return "0" + std::to_string(value);
        else
            return std::to_string(value);
    }

    // This is hacky
    bool DataSaver::LoadAllData(const std::string &file_path, const std::string &file_name, std::map<std::string, std::vector<double>> &result_scalar,
                                std::map<std::string, std::vector<Eigen::Vector2d>> &result_vector)
    {
        // Setup a file stream
        std::string full_file_path = file_path + "/" + file_name + ".txt";

        std::ifstream import_file(full_file_path);

        LOG_INFO("Data Saver: Loading data from " << full_file_path);

        if (!import_file.good())
        {
            LOG_WARN("Data Saver: No file with this name was found.");
            return false;
        }

        double x, y;
        std::string data_name;
        int data_size, num_entries;

        while (import_file >> data_name >> data_size >> num_entries)
        {
            data_name = data_name.substr(0, data_name.find(":")); // Remove the colon
            if (data_size == 1)
            {
                result_scalar[data_name].reserve(num_entries);
                for (int i = 0; i < num_entries; i++)
                {
                    import_file >> x;
                    result_scalar[data_name].push_back((double)x);
                }
            }
            else
            {
                result_vector[data_name].reserve(num_entries);
                for (int i = 0; i < num_entries; i++)
                {
                    import_file >> x;
                    import_file >> y;
                    result_vector[data_name].push_back(Eigen::Vector2d(x, y));
                }
            }
        }
        return true;
    }

    void DataSaver::Clear()
    {
        for (auto &dataset : datasets_)
            dataset->Clear();
    }

    void DataSaver::SetAddTimestamp(bool add_timestamp) { add_timestamp_ = add_timestamp; }
}