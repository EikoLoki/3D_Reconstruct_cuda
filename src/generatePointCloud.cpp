// C++ lib
#include <iostream>
#include <string>

// OpenCV lib
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL lib
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
//#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
//#include <pcl/surface/poisson.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>
//#include <pcl/console/parse.h>
//#include <boost/thread/thread.hpp>
//#include <pcl/features/normal_3d_omp.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/io/point_cloud_image_extractors.h>
#include <pcl/io/png_io.h>
// inner include
#include <config.h>

using namespace std;
using namespace cv;
//using namespace pcl;

// define point cloud type
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud; 



PointCloud::Ptr buildPointCloud(Mat rgb, Mat depth, string& pcd_path){

    PointCloud::Ptr cloud(new PointCloud);
    for (int m = 0; m < depth.rows; m++){
        for (int n=0; n < depth.cols; n++)
        {
            // get the value of depth map in (m, n)
            float d = depth.ptr<float>(m)[n];

            // if d exist then add a point to point cloud
            PointT p;
            if (d == 0){
                p.z = 0;
                p.x = (n - camera_cx_left) * p.z / camera_fx_left;
                p.y = (m - camera_cy_left) * p.z / camera_fy_left;
                p.b = 0;
                p.g = 0;
                p.r = 0;

            }
            else{
                // compute the space coordinates
                p.z = double(d) / camera_factor;
                p.x = (n - camera_cx_left) * p.z / camera_fx_left;
                p.y = (m - camera_cy_left) * p.z / camera_fy_left;

                // get colors from RGB map
                // remember in OpenCV RGB map are set in order of BGR
                p.b = rgb.ptr<uchar>(m)[n*3];
                p.g = rgb.ptr<uchar>(m)[n*3+1];
                p.r = rgb.ptr<uchar>(m)[n*3+2];

            }
            // push back p to the point cloud
            cloud->points.push_back( p );

        }
    }
    // set and save point cloud
    cloud->height = 720;
    cloud->width = 1280;

    cout <<"point cloud size = "<<cloud->points.size()<<endl;
    cloud->is_dense = false;
    pcl::io::savePCDFile( pcd_path, *cloud );

    cout<<"Point cloud saved."<<endl;
    return cloud;
}




/*
void meshUp(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, String vtk_path){

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals


    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);


    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.025);

    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (50);
    gp3.setMaximumSurfaceAngle(M_PI); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);


    pcl::io::saveVTKFile(vtk_path,triangles);
    cout << "VTK file saved:" << endl;


}
*/

pcl::visualization::PCLVisualizer::Ptr simpleVis (PointCloud::Ptr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<PointT> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return viewer;
}

int main( int argc, char** argv )
{
    /*
    string dir = argv[1];
    int num = atoi(argv[2]);

    for(int i = 1; i <= num; i++){
        char depth_name[20];
        char pcd_name[20];
        char rgb_name[20];
        char vtk_name[20];
        sprintf(depth_name,"depth%02d.ext",i);
        sprintf(pcd_name,"PointCloud%02d.pcd",i);
        sprintf(rgb_name,"left%02d.png",i);
        sprintf(vtk_name,"vtk%02d.vtk",i);
        string depth_path = dir + "/depth/" + depth_name;
        string pcd_path = dir + "/PointCloud/" + pcd_name;
        string rgb_path = dir + "/rectify/" + rgb_name;
        string vtk_path = dir + "/VTK/" + vtk_name;

        Mat rgb, depth;
        rgb = imread(rgb_path,-1);

        // rgb 8UC3
        // depth 32FC1
        FileStorage fs_depth(depth_path,FileStorage::READ);
        fs_depth["depth"] >> depth;
        fs_depth.release();

        // variable point cloud
        // smart point build a void point cloud release automatically
        PointCloud::Ptr cloud ( new PointCloud );
        cloud = buildPointCloud(rgb, depth, pcd_path);



        // clear and quit
        cloud->points.clear();
        cout << "i: "<< i << endl;


    }
    return 0;
    */


    /*
    String pcd_data = argv[1];
    pcl::PCLPointCloud2 data;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //PointCloud::Ptr cloud_rgb;
    pcl::io::loadPCDFile(pcd_data, data);
    cout << "load successfully!"<< endl;
    pcl::fromPCLPointCloud2(data, *cloud);
    cout << "transfer successfully!" << endl;
    pcl::io::savePCDFile("raw.pcd",*cloud);
    //pcl::copyPointCloud(cloud, cloud_rgb);
    meshUp(cloud, "meshup.vtk");
    return 0;*/


    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::io::loadPCDFile (argv[1], cloud);

    boost::uint16_t* depth_image;

    /*
    pcl::visualization::PCLVisualizer::Ptr viewer;
    viewer = simpleVis(cloud);

    while (!viewer->wasStopped ())
      {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      }

    */


    //works from a single camera pos
    /*
    pcl::PCLImage image;
    pcl::io::PointCloudImageExtractorFromRGBField<pcl::PointXYZRGB> pcie;
    bool suc = pcie.extract(cloud, image);

    cout << suc << endl;
    pcl::io::savePNGFile("image.png", image);
    */


    /*
    //METHOD #1: Using a Matrix4f
    //    This is the "manual" method, perfect to understand but error prone !
    //
      Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

      // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
      float theta = M_PI/4; // The angle of rotation in radians
      transform_1 (0,0) = cos (theta);
      transform_1 (0,1) = -sin(theta);
      transform_1 (1,0) = sin (theta);
      transform_1 (1,1) = cos (theta);
      //    (row, column)

      // Define a translation of 2.5 meters on the x axis.
      transform_1 (0,3) = 2.5;

      // Print the transformation
      printf ("Method #1: using a Matrix4f\n");
      std::cout << transform_1 << std::endl;

      //  METHOD #2: Using a Affine3f
      //  This method is easier and less error prone

      Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

      // Define a translation of 2.5 meters on the x axis.
      transform_2.translation() << 2.5, 0.0, 0.0;

      // The same rotation matrix as before; theta radians around Z axis
      transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

      // Print the transformation
      printf ("\nMethod #2: using an Affine3f\n");
      std::cout << transform_2.matrix() << std::endl;

      // Executing the transformation
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
      // You can either apply transform_1 or transform_2; they are the same
      pcl::transformPointCloud (*cloud, *transformed_cloud, transform_2);

      // Visualization
      printf(  "\nPoint cloud colors :  white  = original point cloud\n"
          "                        red  = transformed point cloud\n");
      pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

       // Define R,G,B colors for the point cloud
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> source_cloud_color_handler (cloud, 255, 255, 255);
      // We add the point cloud to the viewer and pass the color handler
      viewer.addPointCloud (cloud, source_cloud_color_handler, "original_cloud");

      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
      viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

      viewer.addCoordinateSystem (1.0, "cloud", 0);
      viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
      //viewer.setPosition(800, 400); // Setting visualiser window position

      while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
      }
      */

    /*
    // We now want to create a range image from the above point cloud, with a 1deg angular resolution
    float angularResolution = (float) (  0.1f * (M_PI/180.0f));  //   1.0 degree in radians
    float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
    float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel=0.00;
    float minRange = 0.0f;
    int borderSize = 1;

    pcl::RangeImage rangeImage;
    rangeImage.createFromPointCloud(*cloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                    sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

    pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
    range_image_widget.showRangeImage (rangeImage);

    while(true){
        range_image_widget.spinOnce();
        pcl_sleep(0.1);
    }

    */
    /*
    //pcl::visualization::PCLVisualizer::Ptr viewer;

    pcl::MovingLeastSquares<PointT, PointT> mls;
    mls.setInputCloud (cloud);
    mls.setSearchRadius (0.01);
    //mls.setPolynomialFit (true);
    mls.setPolynomialOrder (2);
    mls.setUpsamplingMethod (pcl::MovingLeastSquares<PointT, PointT>::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius (0.01);
    mls.setUpsamplingStepSize (0.006);
    PointCloud::Ptr cloud_smoothed (new PointCloud ());
    cout << "mls processing..." << endl;
    mls.process (*cloud_smoothed);
    cout << "mls done!" << endl;

    pcl::io::savePCDFile(argv[2], *cloud_smoothed);

    */
    /*
    viewer = simpleVis(cloud);

    while (!viewer->wasStopped ())
      {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      }
    */


    /*
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads (8);
    ne.setInputCloud (cloud_smoothed);
    ne.setRadiusSearch (0.1);
    Eigen::Vector4f centroid;
    compute3DCentroid (*cloud_smoothed, centroid);
    ne.setViewPoint (centroid[0], centroid[1], centroid[2]);
    pcl::PointCloud<Normal>::Ptr cloud_normals (new pcl::PointCloud<Normal> ());
    cout << "normal estimation processing ..." << endl;
    ne.compute (*cloud_normals);
    cout << "normal estimation done ..." << endl;

    for(size_t i = 0; i < cloud_normals->size (); ++i)
    {
      cloud_normals->points[i].normal_x *= -1;
      cloud_normals->points[i].normal_y *= -1;
      cloud_normals->points[i].normal_z *= -1;
    }

    pcl::PointCloud<PointNormal>::Ptr cloud_smoothed_normals (new pcl::PointCloud<PointNormal> ());
    concatenateFields (*cloud_smoothed, *cloud_normals, *cloud_smoothed_normals);

    pcl::Poisson<PointNormal> poisson;
    poisson.setDepth (11);
    poisson.setInputCloud (cloud_smoothed_normals);
    PolygonMesh mesh;
    cout << "poisson reconstruction processing ..." << endl;
    poisson.reconstruct (mesh);
    cout << "poisson reconstruction done ..." << endl;
    io::saveVTKFile (argv[2], mesh);
    */


    return 0;
}
