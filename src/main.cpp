#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/shared_ptr.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/organized_fast_mesh.h>
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/common/projection_matrix.h>

#include <sstream>
#include <pcl/common/centroid.h>
#include "geometry_msgs/Vector3.h"

#include "shape_msgs/Mesh.h"
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"

#include "actionlib/server/simple_action_server.h"

#include <pcl/surface/gp3.h>
#include <sys/time.h>
#include <pcl/surface/mls.h>
#include <pcl/io/ply_io.h>
#include <boost/thread.hpp>

//#include <pcl/apps/organized_segmentation_demo.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>

#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <geometry_msgs/Pose.h>

#include <pcl/common/pca.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>


class TableDetector
{
    private:

        ros::NodeHandle node_handle;
        boost::mutex mutex;

        ros::Subscriber pointCloudSubscriber;
        ros::Publisher zPublisher;

        void pointCloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg);

    public:
        TableDetector();
};


TableDetector::TableDetector():
    node_handle("table_detector")
{

    pointCloudSubscriber =  node_handle.subscribe("/head_camera/depth_registered/points", 10,  &TableDetector::pointCloudCB, this);
    zPublisher = node_handle.advertise<geometry_msgs::Vector3>("tablePose", 10);

    ROS_INFO("table_detector_node ready\n");
}


void TableDetector::pointCloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PCLPointCloud2::Ptr pcl_pc (new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(*msg, *pcl_pc);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*pcl_pc, *cloud);

    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
        model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));


    pcl::IndicesPtr inliers (new std::vector<int>());

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    ransac.getInliers(*inliers);

    pcl::PCA<pcl::PointXYZ> pca;
    pca.setIndices(inliers);
    pca.setInputCloud(cloud);
    pca.getEigenValues();
    Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();

    geometry_msgs::Vector3 zvector;
    if(eigen_vectors(1,2) > 0)
    {
        zvector.x = -eigen_vectors(0,2);
        zvector.y = -eigen_vectors(1,2);
        zvector.z = -eigen_vectors(2,2);
    }
    else
    {
        zvector.x = eigen_vectors(0,2);
        zvector.y = eigen_vectors(1,2);
        zvector.z = eigen_vectors(2,2);
    }

    zPublisher.publish(zvector);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "table_detector_node");
  ros::NodeHandle nh;

  TableDetector node;

  //5 hz
  ros::Rate r(5);
  
  while(ros::ok())
    {
      ros::spinOnce();
      r.sleep();
    }
  

  return 0;
}
