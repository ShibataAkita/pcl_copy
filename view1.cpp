/*显示彩色的点云图*/
#include <pcl/io/openni2_grabber.h>

#include <pcl/visualization/cloud_viewer.h>

#include <pcl/visualization/pcl_visualizer.h>

// #include <pcl/io/io.h>

// #include <pcl/common/time.h>

// #include <pcl/features/integral_image_normal.h>

// #include <pcl/features/normal_3d.h>

// #include <pcl/ModelCoefficients.h>

// #include <pcl/segmentation/planar_region.h>

// #include <pcl/segmentation/organized_multi_plane_segmentation.h>

// #include <pcl/segmentation/organized_connected_component_segmentation.h>

// #include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZRGBA PointT;

class OpenNIOrganizedMultiPlaneSegmentation

{

  private:

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    pcl::PointCloud<PointT>::ConstPtr prev_cloud;

    boost::mutex cloud_mutex;

  public:

    OpenNIOrganizedMultiPlaneSegmentation ()

    {

    }

    ~OpenNIOrganizedMultiPlaneSegmentation ()

    {

    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer>

    cloudViewer (pcl::PointCloud<PointT>::ConstPtr cloud)

    {

      boost::shared_ptr < pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Viewer"));

      viewer->setBackgroundColor (0, 0, 0);

      viewer->addPointCloud<PointT> (cloud,"cloud");

      viewer->addCoordinateSystem (1.0, "global");
      viewer->setCameraPosition(0,0,-0.5,0,1,0);

      viewer->initCameraParameters ();

      return (viewer);

    }

    void

    cloud_cb_ (const pcl::PointCloud<PointT>::ConstPtr& cloud)

    {

      if (!viewer->wasStopped ())

      {

        cloud_mutex.lock ();

        prev_cloud = cloud;

        cloud_mutex.unlock ();

      }

    }

    void

    run ()

    {

      pcl::Grabber* interface = new pcl::io::OpenNI2Grabber ();

      boost::function<void(const pcl::PointCloud<PointT>::ConstPtr&)> f = boost::bind (&OpenNIOrganizedMultiPlaneSegmentation::cloud_cb_, this, _1);

      //make a viewer

      pcl::PointCloud<PointT>::Ptr init_cloud_ptr (new pcl::PointCloud<PointT>);

      viewer = cloudViewer (init_cloud_ptr);

      boost::signals2::connection c = interface->registerCallback (f);

      interface->start ();

      while (!viewer->wasStopped ())

      {

        viewer->spinOnce (100);

        if (prev_cloud && cloud_mutex.try_lock ())

        {

            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.15, "cloud");

          pcl::visualization::PointCloudColorHandlerRGBField<PointT> color (prev_cloud);

          if (!viewer->updatePointCloud<PointT> (prev_cloud, color,"cloud"))

            viewer->addPointCloud<PointT> (prev_cloud, color,"cloud");

          cloud_mutex.unlock ();

        }

      }

      interface->stop ();

    }

};

int

main ()

{

  OpenNIOrganizedMultiPlaneSegmentation multi_plane_app;

  multi_plane_app.run ();

  return 0;

}
