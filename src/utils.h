#include "data.h"

#include <fstream>
#include <vector>
#include <thread>

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
    Kitti_eigen_item operator[](int index);
    void visualize();
};

pcl::PointCloud<pcl::PointXYZ>::Ptr load_pcd(fs::path path);
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *);
void updateImage(Kitti_eigen_split*);
void updateLidar(Kitti_eigen_split*);


