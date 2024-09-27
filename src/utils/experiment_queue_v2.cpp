/* USAGE:
    * 1. Set config.yaml to desired parameters: Mode, Node Length, Node Width, Sac Threshold, Voxel Size
    * 2. Go to the root dir where the clouds are located
    * 3. Run the following command:  
    * rosrun arvc_ground_filter ground_filter_node <path_to_cloud> <mode>{ratio, module, hybrid, wofine, wocoarse_ratio, wocoarse_module, wocoarse_hybrid}
    *   !! If <path_to_cloud> is not set, will apply the algorithm to every cloud found in the current directory !!
    * 
    * Name: Fran Soler Mora
    * email: f.soler@umh.es
 */

#include "arvc_ground_filter/ground_filter.hpp"
#include <yaml-cpp/yaml.h>
#include <cstdlib> // For std::getenv
#include <pcl/common/common.h>
#include "arvc_utils/arvc_utils.hpp"
#include "utils.hpp"
#include "csv_utils.hpp"

typedef pcl::PointXYZLNormal PointIN;

std::vector<fs::path> get_data_paths(fs::path root_dir)
{
    std::vector<fs::path> path_vector;

    root_dir = root_dir / "ply_xyzln";

    for (const auto &entry : fs::directory_iterator(root_dir))
    {
        if (entry.path().extension() == ".ply")
            path_vector.push_back(entry.path());
    }
    return path_vector;
}


void experiment(YAML::Node config, std::vector<fs::path> path_vector){

    std::string set = path_vector[0].parent_path().parent_path().filename().string();
    std::cout << "EXP MODE:  " << config["MODE"].as<string>() << " -- DATASET: " << set << std::endl;
    // VARIABLES UTILITIES
    utils::Metrics global_metrics;
    csv_data data;  

    int normals_time = 0;
    int metrics_time = 0;
    int dataset_size = path_vector.size();
    int ground_size = 0;
    int truss_size = 0;

    float coarse_ground_size_ratio = 0.0;

    string set_id = path_vector[0].parent_path().parent_path().filename().string();
    string best_recall_cloud;
    float best_recall = 0;

    pcl::PointCloud<PointIN>::Ptr input_cloud_xyzln (new pcl::PointCloud<PointIN>);
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZL>);

    // tqdm loop
    auto start = std::chrono::high_resolution_clock::now();
    for (const fs::path &entry : tq::tqdm(path_vector))
    {
        input_cloud_xyzln = arvc::readPointCloud<PointIN>(entry);
        pcl::copyPointCloud(*input_cloud_xyzln, *cloud);

        GroundFilter gf(config);

        gf.set_input_cloud(cloud);
        gf.compute();
        
        normals_time += gf.normals_time;
        metrics_time += gf.metrics_time;
        ground_size += gf.gt_ground_idx->size();
        truss_size += gf.gt_truss_idx->size();

        if (config["EN_METRIC"].as<bool>())
        {
            global_metrics.tp += gf.cm.tp;
            global_metrics.tn += gf.cm.tn;
            global_metrics.fp += gf.cm.fp;
            global_metrics.fn += gf.cm.fn;
        }
    }

    std::cout << "SAVING INFORMATION TO CSV STRUCT" << std::endl;
    data.experiment_id = "experiment_id";
    data.set_id = set_id;
    data.set_size = path_vector.size();
    data.precision = global_metrics.precision();
    data.recall = global_metrics.recall();
    data.f1_score = global_metrics.f1_score();
    data.accuracy = global_metrics.accuracy();
    data.miou = global_metrics.miou();
    data.tp = global_metrics.tp;
    data.tn = global_metrics.tn;
    data.fp = global_metrics.fp;
    data.fn = global_metrics.fn;
    data.ground_size = int (ground_size / dataset_size);
    data.truss_size = int (truss_size / dataset_size);

    data.mode               = config["MODE"].as<std::string>();
    data.node_length        = config["NODE_LENGTH"].as<float>();
    data.node_width         = config["NODE_WIDTH"].as<float>();
    data.sac_threshold      = config["SAC_THRESHOLD"].as<float>();
    data.voxel_size         = config["VOXEL_SIZE"].as<float>();
    data.crop_set           = config["CROP_SET"].as<int>();

    data.density_first      = config["DENSITY"]["first"].as<bool>();
    data.density_enable     = config["DENSITY"]["enable"].as<bool>();
    data.density_radius     = config["DENSITY"]["radius"].as<float>();
    data.density_threshold  = config["DENSITY"]["threshold"].as<int>();

    data.euclid_enable      = config["EUCLID"]["enable"].as<bool>();
    data.euclid_radius      = config["EUCLID"]["radius"].as<float>();
    data.euclid_min_size    = config["EUCLID"]["min_size"].as<int>();



    global_metrics.plotMetrics();

    // PRINT COMPUTATION TIME
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    int avg_time = (int)floor((duration.count() - metrics_time) / dataset_size);
    std::cout << "Average Computation Time: " << avg_time << " ms" << endl;

    data.exec_time = avg_time;
    writeToCSV(fs::path("/home/arvc/workSpaces/arvc_ws/src/arvc_ground_filter/results"), data);
}




int main(int argc, char **argv)
{
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    std::cout << YELLOW << "Running Ground Filter Node:" << RESET << std::endl;

    const char *homeDir = std::getenv("HOME");
    const string HOME = homeDir;
    const fs::path CONFIG = HOME + "/workSpaces/arvc_ws/src/arvc_ground_filter/config/config.yaml";
    const fs::path ROOT_DIR = HOME + "/datasets/complex_structure";

    const std::vector<std::string> DATASETS = {"orto", "crossed", "00", "01", "02", "03"};
    const std::vector<std::string> MODES = {"ratio", "module", "hybrid", "wofine", "wocoarse_ratio", "wocoarse_module", "wocoarse_hybrid"};

    
    YAML::Node config = YAML::LoadFile(CONFIG.string());

    std::vector<fs::path> path_vector;
    for (const string &dataset : DATASETS)
    {
        path_vector = get_data_paths(ROOT_DIR / dataset);

        if (config["CROP_SET"].as<int>() != 0)
        {
            path_vector.resize(config["CROP_SET"].as<int>());
        }

        for (const string &mode : MODES)
        {
            config["MODE"] = mode;
            experiment(config, path_vector);
        }
    }


    return 0;
}


