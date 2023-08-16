#include "utils.h"

int main()
{
    Kitti_eigen_split kitti("../../../kitti");
    kitti.visualize();
    return 0;
}