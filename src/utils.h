#include "data.h"

#include <fstream>
#include <vector>
#include <thread>
#include <iostream>
//OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/ximgproc.hpp"

using std::cout;
using std::endl;
using namespace cv::ximgproc;

class Kitti_eigen_split
{
private:
    int current_idx;
    void init();
    cv::Ptr<cv::StereoSGBM> sgbm;

public:
    fs::path kitti_path;
    Kitti_eigen_split_ds file_paths;
    Kitti_eigen_split_viewers viewers;
    Kitti_eigen_split(string path);
    void increase_index(){current_idx++;}
    void decrease_index(){current_idx--;}
    int get_index() const{return current_idx;}
    Kitti_eigen_item operator[](int index);
    void visualize();
    cv::Mat compute_disparity(const cv::Mat& img0, const cv::Mat& img1);
};

pcl::PointCloud<pcl::PointXYZ>::Ptr load_pcd(fs::path path);
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *);
void updateImage(Kitti_eigen_split*);
void updateLidar(Kitti_eigen_split*);


