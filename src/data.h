#include <string>
#include <experimental/filesystem>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

namespace fs = std::experimental::filesystem;
using std::string;

const string KITTI_ES_ODOMETRY_CALIB{"data_odometry_calib/dataset/sequences"};
const string KITTI_ES_ODIMETRY_GRAY{"data_odometry_gray/dataset/sequences"};
const string KITTI_ES_ODIMETRY_LIDAR{"data_odometry_velodyne/dataset/sequences"};
const string CAM0{"image_0"};
const string CAM1{"image_1"};
const string CAM2{"image_2"};
const string CAM3{"image_3"};
const string LIDAR_ES{"velodyne"};
const string LIDAR{"velodyne_points"};
const int BUFFER_SIZE = 4096;

struct Kitti_eigen_split_ds
{
    std::vector<fs::path> cam0_files;
    std::vector<fs::path> cam1_files;
    std::vector<fs::path> lidar_files;
};

struct Kitti_ds: Kitti_eigen_split_ds
{
    std::vector<fs::path> cam2_files;
    std::vector<fs::path> cam3_files;
};

struct Kitti_eigen_split_viewers
{
    pcl::visualization::PCLVisualizer* lidar_viewer;
    pcl::visualization::ImageViewer* cam0_viewer;
    pcl::visualization::ImageViewer* cam1_viewer;
    pcl::visualization::ImageViewer* disparity_viewer;

    Kitti_eigen_split_viewers()
    {
        lidar_viewer = nullptr;
        cam0_viewer = nullptr;
        cam1_viewer = nullptr;
        disparity_viewer = nullptr;
    }

    ~Kitti_eigen_split_viewers()
    {
        if(lidar_viewer != nullptr)
            delete lidar_viewer;
        if(cam0_viewer != nullptr)
            delete cam0_viewer;
        if(cam1_viewer != nullptr)
            delete cam1_viewer; 
        if(disparity_viewer != nullptr)
            delete disparity_viewer;
    }
};

struct Kitti_viewers : Kitti_eigen_split_viewers
{
    pcl::visualization::ImageViewer* cam2_viewer;
    pcl::visualization::ImageViewer* cam3_viewer;

    Kitti_viewers()
    {
        cam2_viewer = nullptr;
        cam3_viewer = nullptr;
    }

    ~Kitti_viewers()
    {
        if(cam2_viewer != nullptr)
            delete cam2_viewer;
        if(cam3_viewer != nullptr)
            delete cam3_viewer;
    }
};

struct Kitti_eigen_item
{
    fs::path cam0_file;
    fs::path cam1_file;
    fs::path lidar_file;
};

struct Kitti_item : Kitti_eigen_item
{
    fs::path cam2_file;
    fs::path cam3_file;
};
