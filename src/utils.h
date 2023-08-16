#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#include <fstream>
#include <vector>
#include <string>
#include <thread>
//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
//OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
using std::cout;
using std::endl;


using std::string;
const string KITTI_ODOMETRY_CALIB{"data_odometry_calib/dataset/sequences"};
const string KITTI_ODIMETRY_GRAY{"data_odometry_gray/dataset/sequences"};
const string KITTI_ODIMETRY_LIDAR{"data_odometry_velodyne/dataset/sequences"};
const string CAM0{"image_0"};
const string CAM1{"image_1"};
const string LIDAR{"velodyne"};
const int BUFFER_SIZE = 4096;

struct Kitti_eigen_split_ds
{
    std::vector<fs::path> cam0_files;
    std::vector<fs::path> cam1_files;
    std::vector<fs::path> lidar_files;
};


struct Kitti_eigen_split_viewers
{
    pcl::visualization::PCLVisualizer* lidar_viewer;
    pcl::visualization::ImageViewer* cam0_viewer;
    pcl::visualization::ImageViewer* cam1_viewer;

    Kitti_eigen_split_viewers()
    {
        lidar_viewer = nullptr;
        cam0_viewer = nullptr;
        cam1_viewer = nullptr;
    }

    ~Kitti_eigen_split_viewers()
    {
        if(lidar_viewer != nullptr)
            delete lidar_viewer;
        if(cam0_viewer != nullptr)
            delete cam0_viewer;
        if(cam1_viewer != nullptr)
            delete cam1_viewer; 
    }
};

struct Kitti_eigen_item
{
    fs::path cam0_file;
    fs::path cam1_file;
    fs::path lidar_file;
};


class Kitti_eigen_split
{
private:
    
    int current_idx;
    void init();

public:
    fs::path kitti_path;
    Kitti_eigen_split_ds file_paths;
    Kitti_eigen_split_viewers viewers;
    Kitti_eigen_split(string path);
    void increase_index(){current_idx++;}
    int get_index(){return current_idx;}
  //  string get_image_folders();
   // string get_lidar_folders();
    //fs::path get_image(int index);
   // fs::path get_lidar(int index);
    Kitti_eigen_item operator[](int index);
    void visualize();
};

pcl::PointCloud<pcl::PointXYZ>::Ptr load_pcd(fs::path path);
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *);
void updateImage(Kitti_eigen_split*);
void updateLidar(Kitti_eigen_split*);


