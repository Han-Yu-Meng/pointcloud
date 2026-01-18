/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

// cloud_process.hpp

#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Filters
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

// Features
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

// Segmentation
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

// Common & Search
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/transforms.h>

#include <pcl/surface/poisson.h>
#include <fins/functional_node.hpp>

namespace cloud_process {

using fins::Function;
using fins::Input;
using fins::Output;
using fins::Parameter;

using CloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
using CloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>;
using CloudNormal = pcl::PointCloud<pcl::PointNormal>;

inline void check_cloud_empty(const CloudXYZI::Ptr &cloud_ptr) {
  if (!cloud_ptr || cloud_ptr->empty()) {
    throw std::runtime_error("Input point cloud is empty");
  }
}

static auto voxel_filter = Function("VoxelFilter",
  [](Input<CloudXYZI::Ptr> &cloud, Output<CloudXYZI::Ptr> &filtered_cloud, Parameter<float> &leaf_size) {
    check_cloud_empty(cloud);
    filtered_cloud = std::make_shared<CloudXYZI>();
    pcl::VoxelGrid<pcl::PointXYZI> vf;
    vf.setInputCloud(*cloud);
    float ls = *leaf_size > 0.0f ? *leaf_size : 0.1f;
    vf.setLeafSize(ls, ls, ls);
    vf.filter(**filtered_cloud);
    (*filtered_cloud)->header = (*cloud)->header;
  })
  .with_description("Downsample cloud using VoxelGrid")
  .with_inputs_description({"cloud"})
  .with_outputs_description({"cloud"})
  .with_parameter<float>("leaf_size", 0.1f)
  .with_category("Point Cloud>Process")
  .build();

static auto sor_filter = Function("SORFilter",
  [](Input<CloudXYZI::Ptr> &cloud, Output<CloudXYZI::Ptr> &filtered_cloud, Parameter<int> &mean_k, Parameter<double> &stddev_mul) {
    check_cloud_empty(cloud);
    filtered_cloud = std::make_shared<CloudXYZI>();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(*cloud);
    sor.setMeanK(*mean_k > 0 ? *mean_k : 50);
    sor.setStddevMulThresh(*stddev_mul > 0.0 ? *stddev_mul : 1.0);
    sor.filter(**filtered_cloud);
    (*filtered_cloud)->header = (*cloud)->header;
  })
  .with_description("Remove outliers using StatisticalOutlierRemoval")
  .with_inputs_description({"cloud"})
  .with_outputs_description({"cloud"})
  .with_parameter<int>("mean_k", 50)
  .with_parameter<double>("stddev_mul", 1.0)
  .with_category("Point Cloud>Process")
  .build();

static auto radius_filter = Function("RadiusFilter",
  [](Input<CloudXYZI::Ptr> &cloud, Output<CloudXYZI::Ptr> &filtered_cloud, Parameter<double> &radius, Parameter<int> &min_neighbors) {
    check_cloud_empty(cloud);
    filtered_cloud = std::make_shared<CloudXYZI>();
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
    outrem.setInputCloud(*cloud);
    outrem.setRadiusSearch(*radius > 0.0 ? *radius : 0.8);
    outrem.setMinNeighborsInRadius(*min_neighbors > 0 ? *min_neighbors : 2);
    outrem.filter(**filtered_cloud);
    (*filtered_cloud)->header = (*cloud)->header;
  })
  .with_description("Remove outliers using RadiusOutlierRemoval")
  .with_inputs_description({"cloud"})
  .with_outputs_description({"cloud"})
  .with_parameter<double>("radius", 0.8)
  .with_parameter<int>("min_neighbors", 2)
  .with_category("Point Cloud>Process")
  .build();

static auto random_filter = Function("RandomFilter", 
  [](Input<CloudXYZI::Ptr> &cloud, Output<CloudXYZI::Ptr> &filtered_cloud, Parameter<int> &point_num) {
    check_cloud_empty(cloud);
    filtered_cloud = std::make_shared<CloudXYZI>();
    pcl::RandomSample<pcl::PointXYZI> random_sampler;
    random_sampler.setInputCloud(*cloud);
    random_sampler.setSample(*point_num > 0 ? *point_num : 1000);
    random_sampler.filter(**filtered_cloud);
    (*filtered_cloud)->header = (*cloud)->header;
  })
  .with_description("Randomly sample N points")
  .with_inputs_description({"cloud"})
  .with_outputs_description({"cloud"})
  .with_parameter<int>("num", 1000)
  .with_category("Point Cloud>Process")
  .build();

static auto plane_segmentation = Function("PlaneSegmentation", 
  [](Input<CloudXYZI::Ptr> &cloud, Output<CloudXYZI::Ptr> &plane_cloud, Parameter<double> &distance_threshold) {
    check_cloud_empty(cloud);
    plane_cloud = std::make_shared<CloudXYZI>();
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(*distance_threshold > 0.0 ? *distance_threshold : 0.01);
    seg.setInputCloud(*cloud);
    seg.segment(inliers, coefficients);

    if (inliers.indices.empty()) {
      return;
    }

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(*cloud);
    extract.setIndices(std::make_shared<pcl::PointIndices>(inliers));
    extract.setNegative(false); // true to get non-plane points
    extract.filter(**plane_cloud);
    (*plane_cloud)->header = (*cloud)->header;
  })
  .with_description("Extract largest plane using RANSAC")
  .with_inputs_description({"cloud"})
  .with_outputs_description({"plane_cloud"})
  .with_parameter<double>("dist_thresh", 0.01)
  .with_category("Point Cloud>Process")
  .build();

static auto pass_through_filter = Function("PassThroughFilter",
  [](Input<CloudXYZI::Ptr> &cloud, Output<CloudXYZI::Ptr> &filtered_cloud, 
      Parameter<std::string> &field_name, Parameter<float> &limit_min, Parameter<float> &limit_max) {
    check_cloud_empty(cloud);
    filtered_cloud = std::make_shared<CloudXYZI>();
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(*cloud);
    pass.setFilterFieldName(*field_name);
    pass.setFilterLimits(*limit_min, *limit_max);
    pass.filter(**filtered_cloud);
    (*filtered_cloud)->header = (*cloud)->header;
  })
  .with_description("Filter points by field limits (e.g. z range)")
  .with_inputs_description({"cloud"})
  .with_outputs_description({"cloud"})
  .with_parameter<std::string>("field_name", "z")
  .with_parameter<float>("min", -100.0f)
  .with_parameter<float>("max", 100.0f)
  .with_category("Point Cloud>Process")
  .build();

static auto euclidean_cluster_extraction = Function("EuclideanClusterExtraction",
  [](Input<CloudXYZI::Ptr> &cloud, Output<CloudXYZRGB::Ptr> &colored_cloud, 
      Parameter<double> &tolerance, Parameter<int> &min_cluster_size, Parameter<int> &max_cluster_size) {
    check_cloud_empty(cloud);
    
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(*cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    
    double tol = *tolerance > 0 ? *tolerance : 0.2;
    int min_sz = *min_cluster_size > 0 ? *min_cluster_size : 20;
    int max_sz = *max_cluster_size > 0 ? *max_cluster_size : 25000;

    ec.setClusterTolerance(tol);
    ec.setMinClusterSize(min_sz);
    ec.setMaxClusterSize(max_sz);
    ec.setSearchMethod(tree);
    ec.setInputCloud(*cloud);
    ec.extract(cluster_indices);

    colored_cloud = std::make_shared<CloudXYZRGB>();
    
    // Assign random color to each cluster
    for (const auto& indices : cluster_indices) {
      uint8_t r = rand() % 256;
      uint8_t g = rand() % 256;
      uint8_t b = rand() % 256;
      for (const auto& index : indices.indices) {
        pcl::PointXYZRGB p;
        p.x = (*cloud)->points[index].x;
        p.y = (*cloud)->points[index].y;
        p.z = (*cloud)->points[index].z;
        p.r = r; p.g = g; p.b = b;
        (*colored_cloud)->push_back(p);
      }
    }
    (*colored_cloud)->width = (*colored_cloud)->size();
    (*colored_cloud)->height = 1;
    (*colored_cloud)->is_dense = true;
    (*colored_cloud)->header = (*cloud)->header;
  })
  .with_description("Cluster points based on euclidean distance")
  .with_inputs_description({"cloud"})
  .with_outputs_description({"colored_cloud"})
  .with_parameter<double>("tolerance", 0.2)
  .with_parameter<int>("min_size", 20)
  .with_parameter<int>("max_size", 25000)
  .with_category("Point Cloud>Process")
  .build();

static auto transform_cloud = Function("TransformCloud",
  [](Input<CloudXYZI::Ptr> &cloud, Output<CloudXYZI::Ptr> &transformed_cloud, 
      Parameter<float> &x, Parameter<float> &y, Parameter<float> &z, 
      Parameter<float> &roll, Parameter<float> &pitch, Parameter<float> &yaw) {
    check_cloud_empty(cloud);
    transformed_cloud = std::make_shared<CloudXYZI>();
    
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << *x, *y, *z;
    transform.rotate(Eigen::AngleAxisf(*roll, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(*pitch, Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf(*yaw, Eigen::Vector3f::UnitZ()));

    pcl::transformPointCloud(**cloud, **transformed_cloud, transform);

    (*transformed_cloud)->header = (*cloud)->header;
  })
  .with_description("Apply Rigid Body Transform (x,y,z, r,p,y)")
  .with_inputs_description({"cloud"})
  .with_outputs_description({"cloud"})
  .with_parameter<float>("x", 0.f).with_parameter<float>("y", 0.f).with_parameter<float>("z", 0.f)
  .with_parameter<float>("roll", 0.f).with_parameter<float>("pitch", 0.f).with_parameter<float>("yaw", 0.f)
  .with_category("Point Cloud>Process")
  .build();

static auto normal_estimation = Function("NormalEstimation",
  [](Input<CloudXYZI::Ptr> &cloud, Output<CloudNormal::Ptr> &normals, Parameter<double> &radius) {
    check_cloud_empty(cloud);
    normals = std::make_shared<CloudNormal>();
    
    pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::PointNormal> ne;
    ne.setInputCloud(*cloud);
    
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(*radius > 0.0 ? *radius : 0.5);
    
    ne.compute(**normals);
    
    for(size_t i=0; i<(*cloud)->size(); ++i) {
      (*normals)->points[i].x = (*cloud)->points[i].x;
      (*normals)->points[i].y = (*cloud)->points[i].y;
      (*normals)->points[i].z = (*cloud)->points[i].z;
    }

    (*normals)->header = (*cloud)->header;
  })
  .with_description("Estimate normals using OpenMP")
  .with_inputs_description({"cloud"})
  .with_outputs_description({"normals"})
  .with_parameter<double>("radius", 0.5)
  .with_category("Point Cloud>Process")
  .build();

}