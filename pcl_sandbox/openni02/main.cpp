// thanks http://ros-robot.blogspot.jp/2011/08/pclapi-point-cloud-library-pcl-pcl-api.html
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

void segmentate(pcl::PointCloud<pcl::PointXYZRGB>& cloud, double threshould) {
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (threshould);

    seg.setInputCloud (cloud.makeShared ());
    seg.segment (*inliers, *coefficients);

    for (size_t i = 0; i < inliers->indices.size (); ++i) {
        cloud.points[inliers->indices[i]].r = 255;
        cloud.points[inliers->indices[i]].g = 0;
        cloud.points[inliers->indices[i]].b = 0;
    }
}

void main()
{
    // viewerを作る
    pcl::visualization::CloudViewer viewer( "OpenNI Viewer" );

    // データ更新のコールバック関数
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f = 
        [&viewer](const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud) {
            if (!viewer.wasStopped()) {
                pcl::PointCloud<pcl::PointXYZRGB> segmented_cloud( *cloud );
                segmentate( segmented_cloud, 0.01 );
                viewer.showCloud( segmented_cloud.makeShared() );
            }
    };

    // OpenNIの入力を作る
    pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();
    interface->registerCallback( f );
    interface->start();
    while ( !viewer.wasStopped() ) {
        Sleep( 0 );
    }

    interface->stop();
}
