#include "visThread.h"

using namespace std;
using namespace yarp::os;

// Empty constructor
VisThread::VisThread(int period, const string &_cloudname):RateThread(period), id(_cloudname){}

// Initialize Variables
bool VisThread::threadInit()
{
    cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>); // Point cloud
    viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer("Point Cloud Viewer")); //viewer
    
    //initialize here variables
    printf("\nStarting visualizer Thread\n");

    // Flags
    initialized = false;
    addClouds = false;
    clearing = false;
    updatingCloud = false;
    displayBB = false;
    displayNormals = false;

    // Processing parameters
    minimumBB = false;
    radiusSearch = 0.03;

    return true;
}

// Module and visualization loop
void VisThread::run()
{
    if (initialized)
    {
        if (!viewer->wasStopped ())
        {
            viewer->spinOnce ();
            // Get lock on the boolean update and check if cloud was updated
            boost::mutex::scoped_lock updateLock(updateModelMutex);
            if(update)
            {
                if(updatingCloud){
                    // Clean visualizer to plot new cloud
                    viewer->removePointCloud(id);
                    viewer->removePointCloud("normals");
                    viewer->removeAllShapes();

                    // Check if the loaded file contains color information or not
                    bool colorCloud = false;
                    for  (int p = 1; p < cloud->points.size(); ++p)      {
                        uint32_t rgb = *reinterpret_cast<int*>(&cloud->points[p].rgb);
                        if (rgb != 0)
                            colorCloud = true;
                    }
                    // Add cloud color if it has not
                    if (!colorCloud)    {
                        // Define R,G,B colors for the point cloud
                        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_color_handler(cloud, 230, 20, 0); //red
                        viewer->addPointCloud (cloud, cloud_color_handler, id);
                    }else{
                        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> cloud_color_handler (cloud);
                        viewer->addPointCloud (cloud, cloud_color_handler, id);
                    }
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id);
                    updatingCloud = false;
                }

                // Compute and bounding box to display
                if (displayBB)
                {
                    plotBB(minimumBB);
                    displayBB = false;
                }

                // Compute and add normals to display
                if (displayNormals)
                {
                    plotNormals(radiusSearch);
                    displayNormals = false;
                }

                // Clear display
                if (clearing){
                    viewer->removePointCloud(id);
                    viewer->removePointCloud("normals");
                    viewer->removeAllShapes();
                    clearing = false;
                }

                update = false;
            }
            updateLock.unlock();
        }else{
            //Close viewer
            printf("Closing Visualizer\n");
            viewer->close();
            viewer->removePointCloud(id);
            this->stop();
        }

    }
}

void VisThread::threadRelease()
{
    printf("Closing Visualizer Thread\n");
}

// Unlock mutex temporarily to allow an update cycle on the visualizer
void VisThread::updateVis()
{
    // Lock mutex while update is on
    boost::mutex::scoped_lock updateLock(updateModelMutex);
    update = true;
    updateLock.unlock();

    printf("Visualizer updated\n");
}

// Clear display
void VisThread::clearVisualizer()
{
    clearing = true;
    updateVis();
}

// Display new cloud received
void VisThread::updateCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in)
{
    printf("Updating displayed cloud\n");
    if (addClouds)
        *cloud += *cloud_in; // new cloud is added to last one
    else
        *cloud = *cloud_in;  // new cloud overwrites last one

    if (!initialized)
    {
        // Set camera position and orientation
        viewer->setBackgroundColor (0.05, 0.05, 0.05, 0); // Setting background to a dark grey
        viewer->addCoordinateSystem (0.05);
        initialized = true;
    }

    updatingCloud = true;
    updateVis();
    printf("Cloud updated\n");
}

// Set flow to allow normals to be computed and added on display update.
void VisThread::addNormals(double rS)
{
    if (initialized){
        displayNormals = true;
        radiusSearch = rS;
        updateVis();
    }else{
        printf("Please load a cloud before trying to compute the normals\n");
    }
}

// Compute and display normals
void VisThread::plotNormals(double rS)
{
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius radiusSearch
    ne.setRadiusSearch (rS);

    // Compute the features
    ne.compute (*cloud_normals);

    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, cloud_normals, 30, rS, "normals");
}

// Set flow to allow bounding box to be computed and added on display update.
void VisThread::addBoundingBox(bool minBB)
{
    if (initialized){
        displayBB = true;
        minimumBB = minBB;

        updateVis();

        if (minBB)
            printf("Minimal Bounding Box added to display\n");
        else
            printf("Axis Aligned Bounding Box added to display\n");
    }else{
        printf("Please load cloud to compute Bounding box from.\n");
    }
}

// Computes and display Bounding Box
void VisThread::plotBB(bool minBB)
{
    // Create the feature extractor estimation class, and pass the input cloud to it
    pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;
    feature_extractor.setInputCloud (cloud);
    feature_extractor.compute ();

    if (minBB)          // Oriented bounding box (OBB)
    {   // Declare points and rotational matrix
        pcl::PointXYZRGB min_point_OBB;
        pcl::PointXYZRGB max_point_OBB;
        pcl::PointXYZRGB position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;

        // Compute extrema of minimum bounding box and rotational matrix wrt axis.
        feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
        Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
        Eigen::Quaternionf quat (rotational_matrix_OBB);

        // Display oriented minimum boinding box
        viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");

    }else{              // Axis-aligned bounding box (AABB)
        // Declare extrema points of AABB
        pcl::PointXYZRGB min_point_AABB;
        pcl::PointXYZRGB max_point_AABB;

        // Compute extrema of AABB
        feature_extractor.getAABB (min_point_AABB, max_point_AABB);

        // Display axis aligned minimum boinding box
        viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
    }
}

// Selects between displaying each cloud individually or accumulating them on the display.
void VisThread::accumulateClouds(bool accum)
{
    if (accum){
        addClouds = true;
        printf("Clouds plotted together now\n");
    }else{
        addClouds = false;
        printf("Clouds plotted separately now\n");
    }
}

