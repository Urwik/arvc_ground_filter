/* USAGE:
    * 0. Compile the package with catkin_make
    * 1. Set config.yaml to desired parameters: Mode, Node Length, Node Width, Sac Threshold, Voxel Size
    * 2. Go to the root dir where the clouds are located
    * 3. Run the following command:  
    * rosrun arvc_ground_filter ground_filter_node <path_to_cloud> <mode>{ratio, module, hybrid, wofine, wocoarse_ratio, wocoarse_module, wocoarse_hybrid}
    *   !! If <path_to_cloud> is not set, will apply the algorithm to every cloud found in the current directory !!
    * 
    * Name: Fran Soler Mora
    * email: f.soler@umh.es
 */

#include <ros/package.h>
#include <pcl/common/common.h>
#include <yaml-cpp/yaml.h>
#include <cstdlib> // For std::getenv
#include "arvc_ground_filter/ground_filter.hpp"
#include "arvc_utils/arvc_utils.hpp"
#include "utils.hpp"


std::vector<fs::path> get_data_paths(int argc, char **argv)
{
    std::vector<fs::path> path_vector;
    
    // COMPUTE THE ALGORITHM FOR EVERY CLOUD IN THE CURRENT FOLDER
    if (argc == 1)
    {
        fs::path current_dir = fs::current_path();

        // Save all the paths of the clouds in the current directory for the tqdm loop
        for (const auto &entry : fs::directory_iterator(current_dir))
        {
            if (entry.path().extension() == ".pcd" || entry.path().extension() == ".ply")
                path_vector.push_back(entry.path());
        }
    }
    // COMPUTE THE ALGORITHM ONLY ONE CLOUD PASSED AS ARGUMENT IN CURRENT FOLDER
    else if (argc == 2)
    {
        fs::path entry = argv[1];
        std::cout << "Processing cloud: " << entry << std::endl;

        if (entry.extension() == ".pcd" || entry.extension() == ".ply")
        {
            path_vector.push_back(entry);
        }
    }
    else
    {
        std::cout << "\tNo mode selected." << std::endl;
        std::cout << "\tUsage: ./rrss_grnd_filter <path_to_cloud> <mode>{ratio, module, hybrid, wofine, wocoarse}" << std::endl;
    }

    return path_vector;
}

// Function to get memory usage in bytes
long getMemoryUsageInBytes() {
    std::ifstream statm_file("/proc/self/status");
    std::string line;
    long memoryUsage = 0;

    while (std::getline(statm_file, line)) {
        if (line.find("VmRSS:") != std::string::npos) {  // VmRSS gives resident memory in KB
            std::istringstream iss(line);
            std::string key;
            long value;
            std::string unit;
            iss >> key >> value >> unit;  // Extract key, value, and unit (should be "kB")
            memoryUsage = value * 1024;   // Convert from kilobytes to bytes
            break;
        }
    }
    return memoryUsage;  // Return memory in bytes
}



int main(int argc, char **argv)
{
    typedef pcl::PointXYZLNormal PointIN;
    

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    std::cout << YELLOW << "Running Ground Filter Node:" << RESET << std::endl;

    // const std::string package_path = ros::package::getPath("arvc_ground_filter");
    // const fs::path CONFIG = fs::path(package_path) / "config/config.yaml";

    const fs::path CONFIG = "/home/arvc/workSpaces/arvc_ws/src/arvc_ground_filter/config/config.yaml";

    YAML::Node config = YAML::LoadFile(CONFIG.string());
    
    std::vector<fs::path> path_vector;
    path_vector = get_data_paths(argc, argv);
    
    if (config["CROP_SET"].as<int>() != 0)
    {
        path_vector.resize(config["CROP_SET"].as<int>());
    }


    std::cout << "\t Evaluating clouds: " << path_vector.size() << std::endl;

    // VARIABLES UTILITIES
    utils::Metrics global_metrics;

    int normals_time = 0;
    int metrics_time = 0;

    float coarse_ground_size_ratio = 0.0;

    string best_recall_cloud;
    float best_recall = 0;

    pcl::PointCloud<PointIN>::Ptr input_cloud_xyzln (new pcl::PointCloud<PointIN>);
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZL>);

    const int dataset_size = path_vector.size();


    // std::cout << "Num argc: " << argc << std::endl;
    // return 0;
    
    if (argc > 1)
    {
        fs::path entry = argv[1];
        input_cloud_xyzln = arvc::readPointCloud<PointIN>(entry);
        pcl::copyPointCloud(*input_cloud_xyzln, *cloud);

        GroundFilter gf(config);

        gf.cloud_id = entry.stem();
        gf.set_input_cloud(cloud);
        gf.compute();

        return 0;
    }



    auto start = std::chrono::high_resolution_clock::now();
    for (const fs::path &entry : tq::tqdm(path_vector))
    {
        std::cout << "Memory before loading cloud: " << getMemoryUsageInBytes() << " bytes" << std::endl;
        input_cloud_xyzln = arvc::readPointCloud<PointIN>(entry);
        pcl::copyPointCloud(*input_cloud_xyzln, *cloud);


        std::cout << "Memory after loading cloud: " << getMemoryUsageInBytes() << " bytes" << std::endl;

        GroundFilter gf(config);

        gf.set_input_cloud(cloud);
        gf.compute();
        
        std::cout << "Memory after filtering: " << getMemoryUsageInBytes() << " bytes" << std::endl;


        normals_time += gf.normals_time;
        metrics_time += gf.metrics_time;

        if (config["EN_METRIC"].as<bool>())
        {
            global_metrics.tp += gf.cm.tp;
            global_metrics.tn += gf.cm.tn;
            global_metrics.fp += gf.cm.fp;
            global_metrics.fn += gf.cm.fn;
        }
    }

    global_metrics.plotMetrics();

    std::cout << "Memory after all computations: " << getMemoryUsageInBytes() << " bytes" << std::endl;


    // PRINT COMPUTATION TIME
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    int avg_time = (int)floor((duration.count() - metrics_time) / dataset_size);
    std::cout << "Average Computation Time: " << avg_time << " ms" << std::endl;

    return 0;
}

    // coarse_ground_size_ratio += gf.coarse_ground_idx->size() / cloud->size();

    /*
    // GET IMPORTANT CLOUDS
    float recall_hybrid = gf.metricas.values.recall;
    gf.computeWOfine();
    float recall_WOfine = gf.metricas.values.recall;

    if (recall_hybrid > recall_WOfine)
    {
        cout << GREEN << gf.path.stem() << RESET << endl;
    } 

    if (gf.metricas.values.recall > best_recall)
    {
        best_recall = gf.metricas.values.recall;
        best_recall_cloud = entry.stem();
    }
    */

    // float avg_coarse_ground_size_ratio = coarse_ground_size_ratio / dataset_size;

    // std::cout << "Coarse ground size ratio: " << avg_coarse_ground_size_ratio << std::endl;

    // std::cout << BLUE << "Best recall: " << best_recall_cloud << RESET << endl;
    // std::cout << YELLOW << "Code end!!" << RESET << std::endl;
