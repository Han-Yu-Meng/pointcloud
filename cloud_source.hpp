/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

// cloud_source.hpp

#pragma once

#include <atomic>
#include <boost/random.hpp>
#include <chrono>
#include <mutex>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <thread>
#include <fins/node.hpp>

using CloudXYZI = pcl::PointCloud<pcl::PointXYZI>;

class RandomCloudSource : public fins::Node {
public:
  void define() override {
    set_name("RandomCloudSource");
    set_description("Periodically generates a random point cloud.");
    set_category("Point Cloud>Source");

    register_output<0, CloudXYZI::Ptr>("cloud");

    register_parameter<int>("num_points", &RandomCloudSource::update_num_points, 1000);
    register_parameter<double>("lower_bound", &RandomCloudSource::update_lower, -10.0);
    register_parameter<double>("upper_bound", &RandomCloudSource::update_upper, 10.0);
    register_parameter<int>("interval_ms", &RandomCloudSource::update_interval, 100);
  }

  void initialize() override {
    rng_.seed(static_cast<boost::uint32_t>(std::chrono::high_resolution_clock::now().time_since_epoch().count()));
    is_running_ = false;
  }

  void run() override {
    is_running_ = true;
    worker_ = std::thread(&RandomCloudSource::loop, this);
  }

  void pause() override {
    is_running_ = false;
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  void reset() override { pause(); }

  void update_num_points(const int &n) {
    std::lock_guard<std::mutex> lock(mutex_);
    num_points_ = n > 0 ? n : 1000;
  }
  void update_lower(const double &v) {
    std::lock_guard<std::mutex> lock(mutex_);
    lower_ = v;
  }
  void update_upper(const double &v) {
    std::lock_guard<std::mutex> lock(mutex_);
    upper_ = v;
  }
  void update_interval(const int &v) {
    std::lock_guard<std::mutex> lock(mutex_);
    interval_ms_ = v > 0 ? v : 100;
  }

private:
  void loop() {
    while (is_running_) {
      int n;
      double l, u;
      int interval;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        n = num_points_;
        l = lower_;
        u = upper_;
        interval = interval_ms_;
      }

      auto cloud = std::make_shared<CloudXYZI>();
      cloud->width = n;
      cloud->height = 1;
      cloud->points.resize(n);
      cloud->is_dense = false;
      cloud->header.frame_id = "random_cloud";

      boost::random::uniform_real_distribution<float> dist(l, u);
      for (size_t i = 0; i < static_cast<size_t>(n); ++i) {
        cloud->points[i].x = dist(rng_);
        cloud->points[i].y = dist(rng_);
        cloud->points[i].z = dist(rng_);
      }

      send<0>(cloud, fins::now());
      std::this_thread::sleep_for(std::chrono::milliseconds(interval));
    }
  }

  boost::random::mt19937 rng_;
  std::thread worker_;
  std::atomic<bool> is_running_{false};
  std::mutex mutex_;

  int num_points_ = 1000;
  double lower_ = -10.0;
  double upper_ = 10.0;
  int interval_ms_ = 100;
};

class PcdFileSource : public fins::Node {
public:
  void define() override {
    set_name("PcdFileSource");
    set_description("Periodically publishes a point cloud loaded from a PCD file.");
    set_category("Point Cloud>Source");

    register_output<0, CloudXYZI::Ptr>("cloud");

    register_parameter<std::string>("file_path", &PcdFileSource::update_path, "/path/to/pointcloud.pcd");
    register_parameter<int>("interval_ms", &PcdFileSource::update_interval, 1000);
  }

  void initialize() override {
    loaded_cloud_.reset(new CloudXYZI());
    is_running_ = false;
  }

  void run() override {
    is_running_ = true;
    worker_ = std::thread(&PcdFileSource::loop, this);
  }

  void pause() override {
    is_running_ = false;
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  void reset() override {
    pause();
    std::lock_guard<std::mutex> lock(mutex_);
    loaded_cloud_->clear();
  }

  void update_path(const std::string &path) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(path, *loaded_cloud_) == -1) {
      logger->error("Couldn't read PCD file: {}", path);
    } else {
      logger->info("Loaded {} points from {}", loaded_cloud_->size(), path);
    }
  }

  void update_interval(const int &v) {
    std::lock_guard<std::mutex> lock(mutex_);
    interval_ms_ = v > 0 ? v : 1000;
  }

private:
  void loop() {
    while (is_running_) {
      int interval;
      CloudXYZI::Ptr cloud_copy;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        interval = interval_ms_;
        if (loaded_cloud_ && !loaded_cloud_->empty()) {
          cloud_copy = loaded_cloud_;
        }
      }

      if (cloud_copy && !cloud_copy->empty()) {
        // cloud_copy->header.stamp = 
        send<0>(cloud_copy, fins::now());
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(interval));
    }
  }

  CloudXYZI::Ptr loaded_cloud_;
  std::thread worker_;
  std::atomic<bool> is_running_{false};
  std::mutex mutex_;
  int interval_ms_ = 1000;
};

EXPORT_NODE(RandomCloudSource)
EXPORT_NODE(PcdFileSource)