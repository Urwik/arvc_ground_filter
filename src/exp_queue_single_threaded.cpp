#include "arvc_ground_filter/ground_filter.hpp"
#include "custom_logger.hpp"
#include "csv_utils.hpp"
#include "arvc_utils/arvc_metrics.hpp"
#include <yaml-cpp/yaml.h>
#include <cstdlib> // For std::getenv
#include <ctime>
#include <chrono>
#include <sstream>

// #include <thread>
// #include <mutex>

// std::mutex mtx;

// struct exp_config{
//     fs::path set_path;
//     fs::path clouds_dir;
//     fs::path output_dir;
//     string experiment_id;
//     int MODO;
//     float NODE_LENGTH;
//     float NODE_WIDTH;
//     float SAC_THRESHOLD;
//     float VOXEL_SIZE;
//     int CROP_SET;

//     bool DENSITY_FIRST;
//     bool EN_DENSITY;
//     float DENSITY_THRESHOLD;
//     float DENSITY_RADIUS;

//     bool EN_EUCLIDEAN_CLUSTERING;
//     float CLUSTER_RADIUS;
//     int CLUSTER_MIN_SIZE;
//     bool SAVE_SEGMENTED_CLOUD;
//     bool EN_VISUALIZATION;
//     bool EN_METRICS;

// };

void experiment(YAML::Node config, fs::path clouds_dir, fs::path output_dir){

    MODE modo = parse_MODE(config["MODE"].as<std::string>());

    std::time_t now_c = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&now_c), "%F %T");
    std::string dateTimeStr = oss.str();

    // VARIABLES UTILITIES
    arvc::Metrics experiment_metrics;
    int normals_time = 0, metrics_time = 0, dataset_size = 0;

    // Save all the paths of the clouds in the current directory for the tqdm loop
    std::vector<fs::path> path_vector;
    for (const auto &entry : fs::directory_iterator(clouds_dir))
    {
        if (entry.path().extension() == ".pcd" || entry.path().extension() == ".ply")
            path_vector.push_back(entry.path());
    }
    std::sort(path_vector.begin(), path_vector.end());


    if (config["CROP_SET"].as<int>() != 0) {
        path_vector.resize(config["CROP_SET"].as<int>());
    }

    dataset_size = path_vector.size();

    int num_ground_idx = 0;
    int num_truss_idx = 0;

    // for (const fs::path &entry : tq::tqdm(path_vector))
    auto start = std::chrono::high_resolution_clock::now();
    int progress_count = 0;
    
    for (const fs::path &entry : path_vector)
    {
        std::cout << "Progress: " << progress_count << " / " << dataset_size << std::endl;
        pcl::PointCloud<pcl::PointXYZL>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZL>);
        pcl::PointCloud<pcl::PointXYZLNormal>::Ptr in_cloud_xyzln (new pcl::PointCloud<pcl::PointXYZLNormal>);

        in_cloud_xyzln = readPointCloud<pcl::PointXYZLNormal>(entry);
        pcl::copyPointCloud(*in_cloud_xyzln, *input_cloud);

        
        GroundFilter gf;

        // Debug and Visualization Parameters
        gf.cons.enable = config["EN_DEBUG"].as<bool>();
        gf.cons.enable_vis = config["EN_VISUAL"].as<bool>();
        gf.enable_metrics = true;

        // Ground Filter Parameters
        gf.set_mode(modo);
        gf.set_node_length(config["NODE_LENGTH"].as<float>());
        gf.set_node_width(config["NODE_WIDTH"].as<float>());
        gf.set_sac_threshold(config["SAC_THRESHOLD"].as<float>());
        gf.set_voxel_size(config["VOXEL_SIZE"].as<float>());

        // Density Filter Parameters
        gf.enable_density_filter    = true;
        gf.density_first            = false; // Default false
        gf.density_radius           = config["DENSITY"]["radius"].as<float>();
        gf.density_threshold        = config["DENSITY"]["threshold"].as<float>();

        gf.enable_euclidean_clustering  = false;


        gf.set_input_cloud(input_cloud);
        gf.compute();


        if (config["SAVE_SEGMENTED_CLOUD"].as<bool>()) {
            pcl::PointCloud<pcl::PointXYZL>::Ptr segmented_cloud (new pcl::PointCloud<pcl::PointXYZL>);
            pcl::copyPointCloud(*input_cloud, *segmented_cloud);

            for (size_t i = 0; i < input_cloud->points.size(); i++) {

                if (std::find(gf.truss_idx->begin(), gf.truss_idx->end(), i) != gf.truss_idx->end())
                    segmented_cloud->points[i].label = 1;
                else
                    segmented_cloud->points[i].label = 0;
            }


            fs::path save_path = output_dir / (entry.stem().string() + ".ply");
            savePointCloud<pcl::PointXYZL>(segmented_cloud, save_path);
        }


        num_ground_idx += gf.gt_ground_idx->size();
        num_truss_idx += gf.gt_truss_idx->size();

        normals_time += gf.normals_time;
        metrics_time += gf.metrics_time;

        experiment_metrics.accuracy.push_back(gf.metricas.values.accuracy);
        experiment_metrics.precision.push_back(gf.metricas.values.precision);
        experiment_metrics.recall.push_back(gf.metricas.values.recall);
        experiment_metrics.f1_score.push_back(gf.metricas.values.f1_score);
        experiment_metrics.tp.push_back(gf.metricas.values.tp);
        experiment_metrics.tn.push_back(gf.metricas.values.tn);
        experiment_metrics.fp.push_back(gf.metricas.values.fp);
        experiment_metrics.fn.push_back(gf.metricas.values.fn);
        progress_count++;
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    std::string set_id = clouds_dir.parent_path().filename().string();

    int exp_tp = std::accumulate(experiment_metrics.tp.begin(), experiment_metrics.tp.end(), 0);
    int exp_fp = std::accumulate(experiment_metrics.fp.begin(), experiment_metrics.fp.end(), 0);
    int exp_fn = std::accumulate(experiment_metrics.fn.begin(), experiment_metrics.fn.end(), 0);
    int exp_tn = std::accumulate(experiment_metrics.tn.begin(), experiment_metrics.tn.end(), 0);

    float precision = exp_tp / (exp_tp + exp_fp);;
    float recall = exp_tp / (exp_tp + exp_fn);;
    float f1_score = 2 * (precision * recall) / (precision + recall);

    csv_data data;
    data.experiment_id = dateTimeStr;
    data.mode = parse_MODE(modo);
    data.set = stoi(set_id);
    data.set_size = dataset_size;
    data.precision = precision;
    data.recall = recall;
    data.f1_score = f1_score;
    data.tp = arvc::Metrics::getMean<int>(experiment_metrics.tp);
    data.tn = arvc::Metrics::getMean<int>(experiment_metrics.tn);
    data.fp = arvc::Metrics::getMean<int>(experiment_metrics.fp);
    data.fn = arvc::Metrics::getMean<int>(experiment_metrics.fn);
    data.exec_time = (int) floor((duration.count() -   metrics_time) / dataset_size);
    data.ground_size = (int)round(num_ground_idx/dataset_size);
    data.truss_size = (int)round(num_truss_idx/dataset_size);

    data.density_threshold = config["DENSITY"]["threshold"].as<float>();
    data.density_first = config["DENSITY"]["first"].as<bool>();

    data.euclidean_threshold = config["EUCLID"]["min_size"].as<int>();
    data.voxel_size = config["VOXEL_SIZE"].as<float>();
    data.sac_threshold = config["SAC_THRESHOLD"].as<float>();
    data.node_length = config["NODE_LENGTH"].as<float>();
    data.node_width = config["NODE_WIDTH"].as<float>();
    
    writeToCSV(output_dir, data);
}


int main(int argc, char **argv)
{
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    std::cout << YELLOW << "Running your code..." << RESET << std::endl;
    

    const char* HOME = std::getenv("HOME");
    const string HOME_PATH = HOME;

    // Configure experiment
    const fs::path DATA_DIR = "/media/arvc/data/datasets/sncs_test/v1";
    const fs::path OUTPUT_DIR = HOME_PATH + "/workspaces/arvc_ws/src/arvc_ground_filter/results_sncs_revision_single_threaded/";
    
    YAML::Node config = YAML::LoadFile(HOME_PATH + "/workspaces/arvc_ws/src/arvc_ground_filter/config/config.yaml");

    int NUM_OF_MODES = 7;

    // PARA CADA SET
    for (fs::path set_path : fs::directory_iterator(DATA_DIR))
    {
        // Only the set 00 and 05 for test proposes
        if (set_path.filename().string() != "00" && set_path.filename().string() != "05") continue;

        if (!fs::is_directory(set_path))
            continue;

        const std::string SET_ID = set_path.filename().string();
        fs::path clouds_dir = set_path / "ply_xyzln";

        // PARA CADA MODO
        for (int i = 0; i < NUM_OF_MODES; i++)
        {
            fs::path output_dir = OUTPUT_DIR / SET_ID / config["MODE"].as<std::string>();

            if (i != 2) continue; // Only hybrid mode (2) for test proposes
            
            config["MODE"] = i;

            if (SET_ID == "00" || SET_ID == "03")
            {
                config["NODE_LENGTH"] = 2.0f * 1.0;
                config["NODE_WIDTH"] = 0.15f * 1.0;
            }
            else if (SET_ID == "01" || SET_ID == "04")
            {
                config["NODE_LENGTH"] = 1.5f * 1.0;
                config["NODE_WIDTH"] = 0.10f * 1.0;
            }
            else if (SET_ID == "02" || SET_ID == "05")
            {
                config["NODE_LENGTH"] = 1.0f * 1.0;
                config["NODE_WIDTH"] = 0.05f * 1.0;
            }

            experiment(config, clouds_dir, output_dir);
        }

    }

    return 0;
}
