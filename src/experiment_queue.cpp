#include "arvc_ground_filter/ground_filter.hpp"
#include "custom_logger.hpp"
#include "csv_utils.hpp"
#include "arvc_utils/arvc_metrics.hpp"
#include <yaml-cpp/yaml.h>
#include <cstdlib> // For std::getenv
#include <ctime>
#include <chrono>
#include <sstream>

#include <thread>
#include <mutex>

std::mutex mtx;

struct exp_config{
    fs::path set_path;
    fs::path clouds_dir;
    fs::path output_dir;
    string experiment_id;
    int MODO;
    float NODE_LENGTH;
    float NODE_WIDTH;
    float SAC_THRESHOLD;
    float VOXEL_SIZE;
    int CROP_SET;

    bool EN_DENSITY;
    float DENSITY_THRESHOLD;
    float DENSITY_RADIUS;

    bool EN_EUCLIDEAN_CLUSTERING;
    float CLUSTER_RADIUS;
    int CLUSTER_MIN_SIZE;

};

void experiment(exp_config _config){

    MODE modo = static_cast<MODE>(_config.MODO);
    

    std::time_t now_c = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&now_c), "%F %T");
    std::string strDateTime = oss.str();



    const float NODE_LENGTH     = _config.NODE_LENGTH;
    const float NODE_WIDTH      = _config.NODE_WIDTH;
    const float SAC_THRESHOLD   = _config.SAC_THRESHOLD;

    const int CROP_SET          = _config.CROP_SET;

    // VARIABLES UTILITIES
    arvc::Metrics global_metrics;
    int normals_time = 0, metrics_time = 0, dataset_size = 0;

    // Save all the paths of the clouds in the current directory for the tqdm loop
    std::vector<fs::path> path_vector;
    for (const auto &entry : fs::directory_iterator(_config.clouds_dir))
    {
        if (entry.path().extension() == ".pcd" || entry.path().extension() == ".ply")
            path_vector.push_back(entry.path());
    }


    if (CROP_SET != 0) {
        path_vector.resize(CROP_SET);
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

        gf.cons.enable = false;
        gf.cons.enable_vis = false;
        gf.enable_metrics = true;

        gf.set_mode(modo);
        gf.set_node_length(NODE_LENGTH);
        gf.set_node_width(NODE_WIDTH);
        gf.set_sac_threshold(SAC_THRESHOLD);
        gf.set_voxel_size(_config.VOXEL_SIZE);

        gf.density_first            = false;
        gf.enable_density_filter    = _config.EN_DENSITY;
        gf.density_radius           = _config.DENSITY_RADIUS;
        gf.density_threshold        = _config.DENSITY_THRESHOLD;

        gf.enable_euclidean_clustering  = _config.EN_EUCLIDEAN_CLUSTERING;
        gf.cluster_radius               = _config.CLUSTER_RADIUS;
        gf.cluster_min_size             = _config.CLUSTER_MIN_SIZE;

        gf.set_input_cloud(input_cloud);
        gf.compute();

        num_ground_idx += gf.gt_ground_idx->size();
        num_truss_idx += gf.gt_truss_idx->size();

        normals_time += gf.normals_time;
        metrics_time += gf.metrics_time;

        global_metrics.accuracy.push_back(gf.metricas.values.accuracy);
        global_metrics.precision.push_back(gf.metricas.values.precision);
        global_metrics.recall.push_back(gf.metricas.values.recall);
        global_metrics.f1_score.push_back(gf.metricas.values.f1_score);
        global_metrics.tp.push_back(gf.metricas.values.tp);
        global_metrics.tn.push_back(gf.metricas.values.tn);
        global_metrics.fp.push_back(gf.metricas.values.fp);
        global_metrics.fn.push_back(gf.metricas.values.fn);
        progress_count++;
    }


    // PLOT METRICS
    // global_metrics.plotMetrics();

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    mtx.lock();
    Logger myLog(_config.set_path.string() + "/log.txt");
    myLog.log("------------------------------------------------------------\n");
    myLog.log("Date and Time: " + strDateTime);
    myLog.log("Experiment ID: " + _config.experiment_id);
    myLog.log("Current directory: " + _config.set_path.string());
    myLog.log("Crop Set: " + to_string(CROP_SET));
    myLog.log("Mode: " + parse_MODE(modo));
    myLog.log("Node Length: " + to_string(_config.NODE_LENGTH));
    myLog.log("Node Width: " + to_string(_config.NODE_WIDTH));
    myLog.log("SAC Threshold: " + to_string(_config.SAC_THRESHOLD));
    myLog.log("Voxel Size: " + to_string(_config.VOXEL_SIZE));
    myLog.log("Density Filter: " + to_string(_config.EN_DENSITY));
    myLog.log("Density Threshold: " + to_string(_config.DENSITY_THRESHOLD));
    myLog.log("Density Radius: " + to_string(_config.DENSITY_RADIUS));
    myLog.log("Euclidean Clustering: " + to_string(_config.EN_EUCLIDEAN_CLUSTERING));
    myLog.log("Cluster Radius: " + to_string(_config.CLUSTER_RADIUS));
    myLog.log("Cluster Min Size: " + to_string(_config.CLUSTER_MIN_SIZE));
    myLog.log(global_metrics.getString());
    myLog.log("Average Computation Time: " + to_string(duration.count() / dataset_size) + " ms");
    mtx.unlock();


    csv_data data;
    data.experiment_id = _config.experiment_id;
    data.mode = parse_MODE(modo);
    data.set = stoi(_config.set_path.filename().string());
    data.set_size = dataset_size;
    data.precision = arvc::Metrics::getMean<float>(global_metrics.precision);
    data.recall = arvc::Metrics::getMean<float>(global_metrics.recall);
    data.f1_score = arvc::Metrics::getMean<float>(global_metrics.f1_score);
    data.tp = arvc::Metrics::getMean<int>(global_metrics.tp);
    data.tn = arvc::Metrics::getMean<int>(global_metrics.tn);
    data.fp = arvc::Metrics::getMean<int>(global_metrics.fp);
    data.fn = arvc::Metrics::getMean<int>(global_metrics.fn);
    data.exec_time = (int) floor((duration.count() -   metrics_time) / dataset_size);
    data.ground_size = (int)round(num_ground_idx/dataset_size);
    data.truss_size = (int)round(num_truss_idx/dataset_size);
    data.density_threshold = _config.DENSITY_THRESHOLD;
    data.euclidean_threshold = _config.CLUSTER_MIN_SIZE;
    data.voxel_size = _config.VOXEL_SIZE;
    data.sac_threshold = _config.SAC_THRESHOLD;
    data.node_length = _config.NODE_LENGTH;
    data.node_width = _config.NODE_WIDTH;
    
    mtx.lock();
    writeToCSV(_config.output_dir, data);
    mtx.unlock();
}


int main(int argc, char **argv)
{
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    std::cout << YELLOW << "Running your code..." << RESET << std::endl;

    const char* homeDir = std::getenv("HOME");
    const string homePath = homeDir;

    YAML::Node config = YAML::LoadFile(homePath + "/workspaces/arvc_ws/src/arvc_ground_filter/config/config.yaml");
    exp_config e_config;

    fs::path dataset_dir = "/media/arvc/data/datasets/sncs_test/v1";

    e_config.MODO                    = static_cast<int>(parse_MODE(config["MODE"].as<string>()));
    e_config.NODE_LENGTH             = config["NODE_LENGTH"].as<float>();
    e_config.NODE_WIDTH              = config["NODE_WIDTH"].as<float>();
    e_config.SAC_THRESHOLD           = config["SAC_THRESHOLD"].as<float>();
    e_config.VOXEL_SIZE              = config["VOXEL_SIZE"].as<float>();
    e_config.CROP_SET                = config["CROP_SET"].as<int>();
    e_config.EN_DENSITY              = config["DENSITY"]["enable"].as<bool>();
    e_config.DENSITY_RADIUS          = config["DENSITY"]["radius"].as<float>();
    e_config.DENSITY_THRESHOLD       = config["DENSITY"]["threshold"].as<int>();
    e_config.EN_EUCLIDEAN_CLUSTERING = config["EUCLID"]["enable"].as<bool>();
    e_config.CLUSTER_RADIUS          = config["EUCLID"]["radius"].as<float>();
    e_config.CLUSTER_MIN_SIZE        = config["EUCLID"]["min_size"].as<int>();

    int NUM_OF_MODES = 7;

    std::vector<std::thread> threads;

    int thread_count = 0;
    // PARA CADA SET
    for (fs::path set_path : fs::directory_iterator(dataset_dir))
    {

        if (!fs::is_directory(set_path))
            continue;

        std::string set_id = set_path.filename().string();
        fs::path clouds_dir = set_path / "ply_xyzln";

        e_config.set_path = set_path;
        e_config.clouds_dir = clouds_dir;
        e_config.output_dir = homePath + "/workspaces/arvc_ws/src/arvc_ground_filter/results_sncs/" + set_id;

        // PARA CADA MODO
        for (int i = 0; i < NUM_OF_MODES; i++)
        {
            e_config.MODO = i;

            if (set_id == "00" || set_id == "03")
            {
                e_config.NODE_LENGTH = 2.0f * 1.0;
                e_config.NODE_WIDTH = 0.15f * 1.0;
            }
            else if (set_id == "01" || set_id == "04")
            {
                e_config.NODE_LENGTH = 1.5f * 1.0;
                e_config.NODE_WIDTH = 0.10f * 1.0;
            }
            else if (set_id == "02" || set_id == "05")
            {
                e_config.NODE_LENGTH = 1.0f * 1.0;
                e_config.NODE_WIDTH = 0.05f * 1.0;
            }

            auto now = std::chrono::system_clock::now();
            std::time_t now_c = std::chrono::system_clock::to_time_t(now);

            // Format time as hour, minute, and second
            std::ostringstream oss;
            oss << std::put_time(std::localtime(&now_c), "%H%M%S");

            // Set experiment id
            e_config.experiment_id = oss.str() + "_" + to_string(thread_count);
            threads.push_back(std::thread(experiment, e_config));

            thread_count++;
        }

    }

    for (std::thread& t : threads)
    {
        if (t.joinable())
            t.join();
    }
    
    return 0;
}
