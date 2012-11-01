#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

void main()
{
    // viewerを作る
    pcl::visualization::CloudViewer viewer( "OpenNI Viewer" );

    // データ更新のコールバック関数
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f = 
        [&viewer](const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud) {
        if (!viewer.wasStopped()) {
            viewer.showCloud( cloud );
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
