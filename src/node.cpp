#include "arvc_ground_filter/ground_filter.hpp"
#include <yaml-cpp/yaml.h>

int main(int argc, char **argv)
{
    std::cout << YELLOW << "Running your code..." << RESET << std::endl;
    auto start = std::chrono::high_resolution_clock::now();

    YAML::Node config = YAML::LoadFile("/home/arvc/workSpaces/arvc_ws/src/arvc_ground_filter/config/config.yaml");

    // CONFIGURATION PARAMS
    const bool EN_DEBUG  = config["EN_DEBUG"].as<bool>();
    const bool EN_VISUAL = config["EN_VISUAL"].as<bool>();
    const bool EN_METRIC = config["EN_METRIC"].as<bool>();

    const MODE modo = parse_MODE(config["MODE"].as<string>());
    const float NODE_LENGTH   = config["NODE_LENGTH"].as<float>();
    const float NODE_WIDTH    = config["NODE_WIDTH"].as<float>();
    const float SAC_THRESHOLD = config["SAC_THRESHOLD"].as<float>();
    const float VOXEL_SIZE    = config["VOXEL_SIZE"].as<float>(); // 0.05
    const bool EN_DENSITY_FILTER = config["EN_DENSITY_FILTER"].as<bool>();
    const int CROP_SET = config["CROP_SET"].as<int>();


    // VARIABLES UTILITIES
    arvc::Metrics global_metrics;

    vector<float> by_module_f1;
    vector<float> by_hybrid_f1;
    
    int normals_time = 0, metrics_time = 0, dataset_size = 0;

    // COMPUTE THE ALGORITHM FOR EVERY CLOUD IN THE CURRENT FOLDER
    if (argc == 1)
    {

        fs::path current_dir = fs::current_path();

        // Save all the paths of the clouds in the current directory for the tqdm loop
        std::vector<fs::path> path_vector;
        for (const auto &entry : fs::directory_iterator(current_dir))
        {
            if (entry.path().extension() == ".pcd" || entry.path().extension() == ".ply")
                path_vector.push_back(entry.path());
        }
        dataset_size = path_vector.size();

        if (CROP_SET < dataset_size)
            path_vector.resize(CROP_SET);

        // tqdm loop
        for (const fs::path &entry : tq::tqdm(path_vector))
        // for(const fs::path &entry : path_vector)
        {
            pcl::PointCloud<pcl::PointXYZL>::Ptr input_cloud = read_cloud(entry);
            
            GroundFilter gf;
            gf.set_mode(modo);
            gf.set_node_length(NODE_LENGTH);
            gf.set_node_width(NODE_WIDTH);
            gf.set_sac_threshold(SAC_THRESHOLD);

            gf.cons.enable = EN_DEBUG;
            gf.cons.enable_vis = EN_VISUAL;
            gf.enable_metrics = EN_METRIC;
            gf.enable_density_filter = EN_DENSITY_FILTER;
            gf.set_voxel_size(VOXEL_SIZE);

            gf.set_input_cloud(input_cloud);
            gf.compute();

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
    else if (argc == 3)
    {
        fs::path entry = argv[1];
        pcl::PointCloud<pcl::PointXYZL>::Ptr input_cloud = read_cloud(entry);

        MODE modo = parse_MODE(argv[2]);
        
        GroundFilter gf;
        gf.set_mode(modo);
        gf.set_node_length(NODE_LENGTH);
        gf.set_node_width(NODE_WIDTH);
        gf.set_sac_threshold(SAC_THRESHOLD);

        gf.cons.enable = EN_DEBUG;
        gf.cons.enable_vis = EN_VISUAL;
        gf.enable_metrics = EN_METRIC;
        gf.enable_density_filter = EN_DENSITY_FILTER;
        gf.set_voxel_size(VOXEL_SIZE);
        
        gf.set_mode(modo);
        gf.set_input_cloud(input_cloud);
        gf.compute();


        gf.cons.enable_vis  = EN_VISUAL;
        gf.cons.enable      = EN_DEBUG;
        gf.enable_metrics   = EN_METRIC;

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

    if (EN_METRIC)
        global_metrics.plotMetrics();

    // PRINT COMPUTATION TIME
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    std::cout << YELLOW << "Code end!!" << RESET << std::endl;

    std::cout << "Total Computation Time: " << duration.count() << " ms" << std::endl;
    std::cout << "Computation time without normal estimation: " << duration.count() - normals_time << " ms" << std::endl;
    std::cout << "Evaluated clouds: " << dataset_size << std::endl;
    cout << "Average Computation Time: " << duration.count() / dataset_size << " ms" << endl;
    cout << "Average Computation Time Without normal estimation and metrics: " << (duration.count() - normals_time - metrics_time) / dataset_size << " ms" << endl;

    return 0;
}