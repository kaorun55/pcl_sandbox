#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

void main()
{
    // viewer�����
    pcl::visualization::CloudViewer viewer( "OpenNI Viewer" );

    // �f�[�^�X�V�̃R�[���o�b�N�֐�
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f = 
        [&viewer](const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud) {
        if (!viewer.wasStopped()) {
            viewer.showCloud( cloud );
        }
    };

    // OpenNI�̓��͂����
    pcl::Grabber* interface = new pcl::OpenNIGrabber();
    interface->registerCallback( f );
    interface->start();
    while ( !viewer.wasStopped() ) {
        Sleep( 0 );
    }

    interface->stop();
}
