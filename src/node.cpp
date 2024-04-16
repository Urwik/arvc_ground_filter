#include "arvc_ground_filter/ground_filter.hpp"
#include <yaml-cpp/yaml.h>
#include <cstdlib> // For std::getenv


int main(int argc, char **argv)
{
    std::cout << YELLOW << "Running your code..." << RESET << std::endl;
    auto start = std::chrono::high_resolution_clock::now();

    const char* homeDir = std::getenv("HOME");
    const string homePath = homeDir;

    YAML::Node config = YAML::LoadFile(homePath + "/workSpaces/arvc_ws/src/arvc_ground_filter/config/config.yaml");

    fs::path cur_dir = fs::current_path();

    // CONFIGURATION PARAMS
    const bool EN_DEBUG  = config["EN_DEBUG"].as<bool>();
    const bool EN_VISUAL = config["EN_VISUAL"].as<bool>();
    const bool EN_METRIC = config["EN_METRIC"].as<bool>();


    const bool DEN_FIRST = config["DENSITY"]["first"].as<bool>();
    const bool EN_DENSITY_FILTER    = config["DENSITY"]["enable"].as<bool>();
    const float DENSITY_RADIUS      = config["DENSITY"]["radius"].as<float>();
    const float DENSITY_THRESHOLD   = config["DENSITY"]["threshold"].as<int>();

    const bool EN_EUCLIDEAN     = config["EUCLID"]["enable"].as<bool>();
    const float EUCLID_RADIUS   = config["EUCLID"]["radius"].as<float>();
    const int EUCLID_MIN_SIZE   = config["EUCLID"]["min_size"].as<int>();

    const MODE modo           = parse_MODE(config["MODE"].as<string>());
    const float NODE_LENGTH   = config["NODE_LENGTH"].as<float>();
    const float NODE_WIDTH    = config["NODE_WIDTH"].as<float>();
    const float SAC_THRESHOLD = config["SAC_THRESHOLD"].as<float>();
    const float VOXEL_SIZE    = config["VOXEL_SIZE"].as<float>(); // 0.05
    const int CROP_SET = config["CROP_SET"].as<int>();

    // VARIABLES UTILITIES
    arvc::Metrics global_metrics;
    
    int normals_time = 0;
    int metrics_time = 0;
    int dataset_size = 0;

    float coarse_ground_size_ratio = 0.0;

    string best_recall_cloud;
    float best_recall = 0;

    // COMPUTE THE ALGORITHM FOR EVERY CLOUD IN THE CURRENT FOLDER
    if (argc == 1) {
        fs::path current_dir = fs::current_path();
        
        // Save all the paths of the clouds in the current directory for the tqdm loop
        std::vector<fs::path> path_vector;
        for (const auto &entry : fs::directory_iterator(current_dir))
        {
            if (entry.path().extension() == ".pcd" || entry.path().extension() == ".ply")
                path_vector.push_back(entry.path());
        }
        dataset_size = path_vector.size();

        if (CROP_SET != 0) {
            path_vector.resize(CROP_SET);
        }

        // tqdm loop
        for (const fs::path &entry : tq::tqdm(path_vector))
        // for(const fs::path &entry : path_vector)
        {
            pcl::PointCloud<pcl::PointXYZL>::Ptr input_cloud = read_cloud(entry);
            
            GroundFilter gf;
            
            gf.cons.enable = EN_DEBUG;
            gf.cons.enable_vis = EN_VISUAL;
            gf.enable_metrics = EN_METRIC;

            gf.set_node_length(NODE_LENGTH);
            gf.set_node_width(NODE_WIDTH);
            gf.set_sac_threshold(SAC_THRESHOLD);
            gf.set_voxel_size(VOXEL_SIZE);

            gf.density_first = DEN_FIRST;
            gf.enable_density_filter = EN_DENSITY_FILTER;
            gf.density_radius = DENSITY_RADIUS;
            gf.density_threshold = DENSITY_THRESHOLD;

            gf.enable_euclidean_clustering = EN_EUCLIDEAN;
            gf.cluster_radius = EUCLID_RADIUS;
            gf.cluster_min_size = EUCLID_MIN_SIZE;

            gf.set_mode(modo);
            gf.set_input_cloud(input_cloud);
            gf.compute();
        
            std::cout << entry.stem() <<": " << "Coarse ground size: " << gf.coarse_ground_idx->size() << std::endl;
            coarse_ground_size_ratio += gf.coarse_ground_idx->size() / input_cloud->size();

            // GET IMPORTANT CLOUDS
/*             float recall_hybrid = gf.metricas.values.recall;
            gf.computeWOfine();
            float recall_WOfine = gf.metricas.values.recall;

            if (recall_hybrid > recall_WOfine)
            {
                cout << GREEN << gf.path.stem() << RESET << endl;
            } */

            normals_time += gf.normals_time;
            metrics_time += gf.metrics_time;
            if (gf.metricas.values.recall > best_recall)
            {
                best_recall = gf.metricas.values.recall;
                best_recall_cloud = entry.stem();
            }

            if (EN_METRIC)
            {
                global_metrics.accuracy.push_back(gf.metricas.values.accuracy);
                global_metrics.precision.push_back(gf.metricas.values.precision);
                global_metrics.recall.push_back(gf.metricas.values.recall);
                global_metrics.f1_score.push_back(gf.metricas.values.f1_score);
                global_metrics.tp.push_back(gf.metricas.values.tp);
                global_metrics.tn.push_back(gf.metricas.values.tn);
                global_metrics.fp.push_back(gf.metricas.values.fp);
                global_metrics.fn.push_back(gf.metricas.values.fn);
            }
        }
    }

    // COMPUTE THE ALGORITHM ONLY ONE CLOUD PASSED AS ARGUMENT IN CURRENT FOLDER
    else if (argc == 2)
    {
        fs::path entry = argv[1];
        std::cout << "Processing cloud: " << entry << std::endl;
        pcl::PointCloud<pcl::PointXYZL>::Ptr input_cloud = read_cloud(entry);

        GroundFilter gf;
        
        gf.cons.enable = EN_DEBUG;
        gf.cons.enable_vis = EN_VISUAL;
        gf.enable_metrics = EN_METRIC;

        gf.set_node_length(NODE_LENGTH);
        gf.set_node_width(NODE_WIDTH);
        gf.set_sac_threshold(SAC_THRESHOLD);
        gf.set_voxel_size(VOXEL_SIZE);

        gf.density_first = DEN_FIRST;
        gf.enable_density_filter = EN_DENSITY_FILTER;
        gf.density_radius = DENSITY_RADIUS;
        gf.density_threshold = DENSITY_THRESHOLD;

        gf.enable_euclidean_clustering = EN_EUCLIDEAN;
        gf.cluster_radius = EUCLID_RADIUS;
        gf.cluster_min_size = EUCLID_MIN_SIZE;

        gf.set_mode(modo);
        gf.set_input_cloud(input_cloud);
        gf.compute();

        coarse_ground_size_ratio += gf.coarse_ground_idx->size() / input_cloud->size();

        normals_time += gf.normals_time;
        metrics_time += gf.metrics_time;

        if (EN_METRIC)
        {
            global_metrics.accuracy.push_back(gf.metricas.values.accuracy);
            global_metrics.precision.push_back(gf.metricas.values.precision);
            global_metrics.recall.push_back(gf.metricas.values.recall);
            global_metrics.f1_score.push_back(gf.metricas.values.f1_score);
            global_metrics.tp.push_back(gf.metricas.values.tp);
            global_metrics.tn.push_back(gf.metricas.values.tn);
            global_metrics.fp.push_back(gf.metricas.values.fp);
            global_metrics.fn.push_back(gf.metricas.values.fn);
        }
    }

    else
    {
        std::cout << "\tNo mode selected." << std::endl;
        std::cout << "\tUsage: ./rrss_grnd_filter <path_to_cloud> <mode>{ratio, module, hybrid, wofine, wocoarse}" << std::endl;
        return 1;
    }

    // PLOT METRICS

    if (EN_METRIC) {
        global_metrics.plotMetrics();
    }

    // PRINT COMPUTATION TIME
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    int avg_time = (int) floor((duration.count() - metrics_time) / dataset_size);
    float avg_coarse_ground_size_ratio = coarse_ground_size_ratio / dataset_size;

    std::cout << "Coarse ground size ratio: " << avg_coarse_ground_size_ratio << std::endl;


    std::cout << "Num of Evaluated clouds: " << dataset_size << std::endl;
    std::cout << "Average Computation Time: " << avg_time << " ms" << endl;

    std::cout << BLUE << "Best recall: " << best_recall_cloud << RESET << endl;
    std::cout << YELLOW << "Code end!!" << RESET << std::endl;
    return 0;
}
