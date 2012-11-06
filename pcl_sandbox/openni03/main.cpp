// thanks http://ros-robot.blogspot.jp/2011/08/pclapi-point-cloud-library-pcl-pcl-api.html
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/extract_indices.h>

pcl::PointIndices::Ptr segmentate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, double threshould)
{
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

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    for (size_t i = 0; i < inliers->indices.size (); ++i) {
        cloud->points[inliers->indices[i]].r = 255;
        cloud->points[inliers->indices[i]].g = 0;
        cloud->points[inliers->indices[i]].b = 0;
    }

    return inliers;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr filter( pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, 
             pcl::PointIndices::Ptr inliers, bool isNegatibe )
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result =
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB>() );

    //フィルタリング
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud( cloud );
    extract.setIndices( inliers );

    // true にすると平面を除去、false にすると平面以外を除去
    extract.setNegative( isNegatibe );
    extract.filter( *result );

    return result;
}

void main()
{
    // viewerを作る
    pcl::visualization::CloudViewer viewer( "OpenNI Viewer" );

    // データ更新のコールバック関数
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f = 
        [&viewer](const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud) {
            if (!viewer.wasStopped()) {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr copy_cloud( new pcl::PointCloud<pcl::PointXYZRGB>( *cloud ) );

                auto inliers = segmentate( copy_cloud, 0.01 );
                //auto filtered = filter( copy_cloud, inliers, true );
                auto filtered = filter( copy_cloud, inliers, false );

                viewer.showCloud( filtered );
            }
    };

    // OpenNIの入力を作る
    pcl::Grabber* interface = new pcl::OpenNIGrabber();
    interface->registerCallback( f );
    interface->start();
    while ( !viewer.wasStopped() ) {
        Sleep( 0 );
    }

    interface->stop();
}
