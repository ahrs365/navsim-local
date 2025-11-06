//
// Class for exporting data from ROS to a .txt document, to be read by another program
// Useful for plotting etc.

// Usage: Make a DataSaver class
// Then use AddData(data_name, data) to add new data occurences
// When you want to save, use SaveData()

// You can use Clear() to clear the currently saved data, i.e., when you want to save in each iteration

// Example:
/*
DataSaver data_saver_test;
data_saver_test.AddData("num", 1);
data_saver_test.AddData("num", 2);
data_saver_test.AddData("num2", 3);
data_saver_test.SaveData("export_1");
*/
// The associated matlab file LoadROSData.m converts the generated files to a matlab structure

#ifndef DATA_SAVER_H
#define DATA_SAVER_H

#include <ros_tools/logging.h>
#include <ros_tools/paths.h>

#include <Eigen/Dense>
#include <chrono>
#include <fstream>
#include <map>
#include <memory>

#include <vector>

namespace RosTools
{

  class DataSet
  {
  public:
    DataSet(const std::string &&name)
    {
      name_ = name;
      num_entries_ = 0;
    };
    DataSet(const std::string &name)
    {
      name_ = name;
      num_entries_ = 0;
    };

  protected:
    std::string name_;
    int num_entries_;

  public:
    virtual void AddData(const double &value) { (void)value; };
    virtual void AddData(const Eigen::Vector2d &value) { (void)value; };

    virtual void SaveData(std::ofstream &file) = 0;

    virtual void Clear() = 0;
  };

  class DoubleDataSet : public DataSet
  {
  public:
    DoubleDataSet(const std::string &&name) : DataSet(name) { data_.reserve(100); };

    DoubleDataSet(const std::string &name) : DataSet(name) { data_.reserve(100); };

  protected:
    // Data needs to be convertible to double, there is no easy work around
    std::vector<double> data_;

  public:
    void AddData(const double &value);
    void SaveData(std::ofstream &file);

    void Clear();
  };

  class PointDataSet : public DataSet
  {
  public:
    PointDataSet(const std::string &&name) : DataSet(name) { data_.reserve(100); };
    PointDataSet(const std::string &name) : DataSet(name) { data_.reserve(100); };

  protected:
    // Data needs to be convertible to double, there is no easy work around
    std::vector<Eigen::Vector2d> data_;

  public:
    void AddData(const Eigen::Vector2d &value);
    void SaveData(std::ofstream &file);

    void Clear();
  };

  class DataSaver
  {
  public:
    DataSaver(double size = 20, bool add_timestamp = false);

  private:
    std::vector<std::unique_ptr<DataSet>> datasets_;

    std::map<std::string, int> data_lookup_;

    bool add_timestamp_;

    bool timestamp_set = false;
    std::chrono::system_clock::time_point start_time_;

    template <typename T>
    void ReadSingleDataFromFile(std::ifstream &import_file, std::map<std::string, std::vector<T>> &result)
    {
      double x;
      std::string data_name;
      int data_size, num_entries;

      while (import_file >> data_name >> data_size >> num_entries)
      {
        data_name = data_name.substr(0, data_name.find(":")); // Remove the colon

        result[data_name].reserve(num_entries);

        for (int i = 0; i < num_entries; i++)
        {
          import_file >> x;
          result[data_name].push_back(x);
        }
      }
    }

    void ReadDataFromFile(std::ifstream &import_file, std::map<std::string, std::vector<double>> &result);

    void ReadDataFromFile(std::ifstream &import_file, std::map<std::string, std::vector<int>> &result);

    void ReadDataFromFile(std::ifstream &import_file, std::map<std::string, std::vector<Eigen::Vector2d>> &result);

  public:
    template <typename T>
    void AddData(const std::string &&data_name, const T &data_value)
    {
      auto idx_it = data_lookup_.find(data_name);
      int idx;
      if (idx_it == data_lookup_.end())
      {
        if (datasets_.size() > 1e5)
          LOG_WARN("Warning: Data saver is not saving anymore data, too much different datasets were added already "
                   "(safety to prevent allocation errors!)");

        // Create a new data set, if none exists with this name
        datasets_.emplace_back();

        // Depending on the datatype, add the data
        if (typeid(data_value) == typeid(const Eigen::Vector2d))
        {
          datasets_[datasets_.size() - 1].reset(new PointDataSet(data_name));
        }
        else
        {
          datasets_[datasets_.size() - 1].reset(new DoubleDataSet(data_name));
        }

        // Add its index to the map
        data_lookup_[data_name] = datasets_.size() - 1;

        // Add the data
        idx = datasets_.size() - 1;
      }
      else
      {
        idx = idx_it->second;
      }

      datasets_[idx]->AddData(data_value);
    }

    std::string getFilePath(const std::string &file_path, const std::string &file_name, bool create_folder = true);
    void SaveData(const std::string &file_name);
    void SaveData(const std::string &file_path, const std::string &file_name);

    std::string ParseYear(int value);

    std::string ValueWithZero(int value);

    template <class T>
    bool LoadData(const std::string &file_name, std::map<std::string, std::vector<T>> &result)
    {
      const std::string path = getPackagePath("mpc_planner") + "/data/";
      return LoadData(path, file_name, result);
    }

    template <class T>
    bool LoadData(const std::string &file_path, const std::string &file_name, std::map<std::string, std::vector<T>> &result)
    {
      std::string full_file_path = file_path + "/" + file_name + ".txt";

      // Setup a file stream
      std::ifstream import_file(full_file_path);

      LOG_INFO("Data Saver: Loading data from " << full_file_path);

      if (!import_file.good())
      {
        LOG_WARN("Data Saver: No file with this name was found.");
        return false;
      }

      ReadDataFromFile(import_file, result);

      return true;
    }

    // This is hacky
    bool LoadAllData(const std::string &file_path, const std::string &file_name, std::map<std::string, std::vector<double>> &result_scalar,
                     std::map<std::string, std::vector<Eigen::Vector2d>> &result_vector);
    void Clear();

    void SetAddTimestamp(bool add_timestamp);
  };
}

#endif