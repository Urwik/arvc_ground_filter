#include<iostream>

#include "arvc_utils/arvc_viewer.hpp"
#include "arvc_utils/arvc_utils.hpp"

int main(int argc, char const *argv[])
{
    
    PointCloud::Ptr input_cloud(new PointCloud);
    input_cloud = arvc::readCloud("../examples/minkunet_predictions/00000.ply");

    arvc::viewer viewer("Cloud Viewer");
    viewer.addCloud(input_cloud);
    viewer.show();

    return 0;
}
