#include "utils.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr load_pcd(fs::path path)
{
    std::fstream input(path, ios::in | ios::binary);
    input.seekg(0, ios::beg);
    pcl::PointCloud<pcl::PointXYZ>::Ptr points (new pcl::PointCloud<pcl::PointXYZ>); 
    int i;
	for (i=0; input.good() && !input.eof(); i++) {
		pcl::PointXYZ point;
        float intensity;
		input.read((char *) &point.x, 3*sizeof(float));
		input.read((char *) &intensity, sizeof(float));
		points->push_back(point);
	}
	input.close();
    return points;
}


Kitti_eigen_split::Kitti_eigen_split(string path):kitti_path(path) 
                                                 
{
    init();  
}

Kitti_eigen_item Kitti_eigen_split::operator[](int idx)
{
    Kitti_eigen_item item;
    item.cam0_file = file_paths.cam0_files[idx];
    item.cam1_file = file_paths.cam1_files[idx];
    item.lidar_file = file_paths.lidar_files[idx];
    return item;
}

void Kitti_eigen_split::init()
{
    auto img_folder = kitti_path / KITTI_ES_ODIMETRY_GRAY;
    auto lidar_folder = kitti_path / KITTI_ES_ODIMETRY_LIDAR;
    std::vector<fs::path> img_folders, lidar_folders;
    try
    {
        for (auto const& dir_entry : fs::directory_iterator{img_folder})
            img_folders.push_back(dir_entry.path());
        for(auto const& dir_entry : fs::directory_iterator{lidar_folder})
            lidar_folders.push_back(dir_entry.path());
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    if(img_folders.size() != lidar_folders.size())
        cout << "WARNING: There might be someting wrong with the folder size" << endl;
    
    for (int ii = 0; ii < img_folders.size(); ii++)
    {
        //TODO:USE MULTI THREAD TO IMPROVE THE CODE // SEQ VS PARALLEL
        //TODO: SPECIFY BUFFER SIZE
        for (auto const& dir_entry : fs::directory_iterator{img_folders[ii]/CAM0})
            file_paths.cam0_files.push_back(dir_entry.path());
        for (auto const& dir_entry : fs::directory_iterator{img_folders[ii]/CAM1})
            file_paths.cam1_files.push_back(dir_entry.path());
        for (auto const& dir_entry : fs::directory_iterator{lidar_folders[ii]/LIDAR_ES})
            file_paths.lidar_files.push_back(dir_entry.path());
    }
    
}

void Kitti_eigen_split::visualize()
{
    viewers.cam0_viewer = new pcl::visualization::ImageViewer("Image0");//viewer2.get();
    viewers.cam1_viewer = new pcl::visualization::ImageViewer("Image1");
    viewers.lidar_viewer = new pcl::visualization::PCLVisualizer("Lidar");
    //TODO: error handle for loading data
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd = load_pcd(file_paths.lidar_files[current_idx]);
    viewers.lidar_viewer->addPointCloud(pcd);
    cv::Mat img0 = cv::imread(file_paths.cam0_files[current_idx]);
    cv::Mat img1 = cv::imread(file_paths.cam1_files[current_idx]);
    cv::resize(img0, img0, cv::Size(img0.cols/2, img0.rows), cv::INTER_LINEAR);
    cv::resize(img1, img1, cv::Size(img1.cols/2, img1.rows), cv::INTER_LINEAR);
    viewers.cam0_viewer->showRGBImage(img0.data, img0.cols, img0.rows);
    viewers.cam1_viewer->showRGBImage(img1.data, img1.cols, img1.rows);

    viewers.cam0_viewer->registerKeyboardCallback(keyboardEventOccurred, static_cast<void*>(this));
    viewers.cam1_viewer->registerKeyboardCallback(keyboardEventOccurred, static_cast<void*>(this));
    viewers.lidar_viewer->registerKeyboardCallback(keyboardEventOccurred, static_cast<void*>(this));
    while (!viewers.cam0_viewer->wasStopped() && !viewers.cam1_viewer->wasStopped() && !viewers.lidar_viewer->wasStopped())
    {
        viewers.cam1_viewer->spin();
        viewers.lidar_viewer->spinOnce(100);
        viewers.cam0_viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void updateLidar(Kitti_eigen_split* kes)
{
    auto points = load_pcd(kes->file_paths.lidar_files[kes->get_index()]);
    if(!kes->viewers.lidar_viewer->wasStopped())
        kes->viewers.lidar_viewer->updatePointCloud(points); 
}

void updateImage(Kitti_eigen_split* kes)
{
    cv::Mat img0 = cv::imread(kes->file_paths.cam0_files[kes->get_index()]);
    cv::Mat img1 = cv::imread(kes->file_paths.cam1_files[kes->get_index()]);
    cv::resize(img0, img0, cv::Size(img0.cols/2, img0.rows), cv::INTER_LINEAR);
    cv::resize(img1, img1, cv::Size(img1.cols/2, img1.rows), cv::INTER_LINEAR);
    if(!kes->viewers.cam0_viewer->wasStopped())
        kes->viewers.cam0_viewer->showRGBImage(img0.data, img0.cols, img0.rows);
    if(!kes->viewers.cam1_viewer->wasStopped())
        kes->viewers.cam1_viewer->showRGBImage(img1.data, img1.cols, img1.rows);
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* ptr)
{
    Kitti_eigen_split* kes = static_cast<Kitti_eigen_split*>(ptr);
    if (event.getKeySym() == "q" && event.keyDown())
    {
        kes->viewers.cam0_viewer->close();
        kes->viewers.cam1_viewer->close();
        kes->viewers.lidar_viewer->close();
    }
    if (event.getKeyCode () == 32 && event.keyDown ())
    {
        kes->increase_index();
        updateImage(kes);
        updateLidar(kes);
        if(!kes->viewers.lidar_viewer->wasStopped())
            kes->viewers.lidar_viewer->spinOnce(100);
   }
}