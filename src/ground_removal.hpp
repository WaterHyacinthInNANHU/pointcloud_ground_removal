//
// Created by hviktortsoi on 20-5-27.
//
#include <iostream>
#include <vector>
#include <omp.h>
#include <algorithm>
#include <pcl/segmentation/impl/region_growing.hpp>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZ PointCloudType;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
typedef std::vector<std::vector<int>> ClusterIndices;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
typedef pcl::PointXYZ XYZ;


struct FilterGroundResult {
    PointCloudPtr ground;
    PointCloudPtr non_ground;
    pcl::ModelCoefficients::Ptr coef;

    FilterGroundResult(const PointCloudPtr &ground, const PointCloudPtr &nonGround,
                       const pcl::ModelCoefficients::Ptr &coef) :
            ground(ground),
            non_ground(nonGround),
            coef(coef) {}
};


/**
 * filter ground
 * @param input 输入点云
 * @param distance_th 分割阈值 值越大 地面越多 地上物体残留的地面越少
 * @return
 */
FilterGroundResult filter_ground(const PointCloudPtr &input, double distance_th, int iter_times = 200) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<XYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distance_th);
    seg.setMaxIterations(iter_times);
    seg.setInputCloud(input);

    seg.segment(*inliers, *coefficients);

    PointCloudPtr landscape(new PointCloud), ground(new PointCloud);
    // extract points
    pcl::ExtractIndices<XYZ> extract;
    extract.setInputCloud(input);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*landscape);

    extract.setNegative(false);
    extract.filter(*ground);

    return FilterGroundResult(ground, landscape, coefficients);
}

