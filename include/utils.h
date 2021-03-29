#pragma once

#include <iostream>
#include <vector>
#include <string>

#include <boost/filesystem.hpp>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>

#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>



using namespace std;

vector<boost::filesystem::path> listDir(string dir_path);

void apply_single_color_to_cloud(int r, int g, int b,pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored);


pcl::visualization::PCLVisualizer::Ptr twoCloudViewer (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2);

pcl::visualization::PCLVisualizer::Ptr twoNormalViewer (
    pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud, pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud2,int level = 50, float scale = 0.01);
