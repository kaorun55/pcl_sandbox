#include <pcl/io/openni2_grabber.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>

void keyboardEventOccurred( const pcl::visualization::KeyboardEvent &event, void* viewer_void )
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_ptr = *static_cast<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr *> (viewer_void);
	if ( event.getKeySym() == "s" && event.keyDown() )
	{
		std::cout << "Save" << std::endl;
		pcl::io::savePLYFileASCII( "cloud.ply", *point_cloud_ptr );
	}
}

void main()
{
    // viewerを作る
    pcl::visualization::CloudViewer viewer( "OpenNI Viewer" );

	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr point_cloud_ptr;

    // データ更新のコールバック関数
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = 
        [&viewer, &point_cloud_ptr](const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
        if (!viewer.wasStopped()) {
			point_cloud_ptr = cloud;
            viewer.showCloud( cloud );
        }
    };

	viewer.registerKeyboardCallback( keyboardEventOccurred, (void*)&point_cloud_ptr );

    // OpenNIの入力を作る
    pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();
    interface->registerCallback( f );
    interface->start();
    while ( !viewer.wasStopped() ) {
        Sleep( 0 );
    }

    interface->stop();
}
