#include "utils.h"

int main()
{
    Kitti_eigen_split kitti("../../../kitti_eigen_split");
    kitti.visualize();
    return 0;
}