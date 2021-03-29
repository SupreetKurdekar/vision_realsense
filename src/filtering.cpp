#include <utils.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>

int
 main (int argc, char** argv)
{

  string my_path = argv[1];
  string my_storage_path = argv[2];
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Fill in the cloud data
  pcl::io::loadPCDFile<pcl::PointXYZRGB> (my_path, *cloud);
  pcl::PCDWriter writer;

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-1.2, 1.2);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  // Create the filtering object2
  pcl::PassThrough<pcl::PointXYZRGB> pass2;
  pass2.setInputCloud (cloud_filtered);
  pass2.setFilterFieldName ("y");
  pass2.setFilterLimits (-5, 0.173);
  //pass.setFilterLimitsNegative (true);
  pass2.filter (*cloud_filtered2);

    // making the visualiser
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

int v1(0);
viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
viewer->setBackgroundColor (0, 0, 0, v1);
pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (cloud);
viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud1", v1);

int v2(0);
viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
viewer->setBackgroundColor (0, 0, 0, v2);
pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2 (cloud_filtered);

    std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
    std::string now = std::to_string((long)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count());

    writer.writeASCII(my_storage_path + "/" + now + ".pcd",*cloud_filtered,8);

viewer->addPointCloud<pcl::PointXYZRGB> (cloud_filtered, rgb2, "sample cloud2", v2);


  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return (0);
}