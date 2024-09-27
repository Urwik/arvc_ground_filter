#pragma once

#include <iostream>
#include <fstream>
#include <istream>
#include <vector>
#include <string>
#include <filesystem>

namespace fs = std::filesystem;


    std::string csv_header = "EXPERIMENT_ID,MODE,SET,NODE_LENGTH,NODE_WIDTH,SAC_THRESHOLD,VOXEL_SIZE,CROP_SET,SET_SIZE,PRECISION,RECALL,F1_SCORE,ACCURACY,MIOU,TP,TN,FP,FN,EXEC_TIME,GROUND_SIZE,TRUSS_SIZE,ENABLE_DENSITY,DENSITY_FIRST,DENSITY_RADIUS,DENSITY_THRESHOLD,ENABLE_EUCLID,EUCLID_RADIUS,EUCLID_MIN_SIZE";

struct csv_data
{
    string experiment_id;
    string set_id;
    int set_size;
    float precision, recall, f1_score, accuracy, miou;
    int tp, tn, fp, fn;
    int exec_time;
    int ground_size;
    int truss_size;
    
    std::string mode;
    float node_length;
    float node_width;
    float sac_threshold;
    float voxel_size;
    int crop_set;
    
    bool density_first;
    bool density_enable;
    float density_radius;
    int density_threshold;
    
    bool euclid_enable;
    float euclid_radius;
    int euclid_min_size;
};



void writeToCSV(const fs::path& dir_path, const csv_data& data) {
    
    if (!fs::exists(dir_path)) {
        fs::create_directories(dir_path);
    }

    fs::path csv_path = dir_path / "test.csv";
    
    
    std::ifstream inFile(csv_path);
    bool isEmpty = inFile.peek() == std::ifstream::traits_type::eof();
    inFile.close();


    std::ofstream csv_file; 
    csv_file.open(csv_path, std::ios::app | std::ios::out);

    // Check if the file is open
    if (!csv_file.is_open()) {
        std::cerr << "Error opening file " << csv_path << std::endl;
        return;
    }

    else if (isEmpty) {
        csv_file << csv_header << "\n";
    }

    csv_file << data.experiment_id << ",";
    csv_file << data.mode << ",";
    csv_file << data.set_id << ",";

    csv_file << data.node_length << ",";
    csv_file << data.node_width << ",";
    csv_file << data.sac_threshold << ",";
    csv_file << data.voxel_size << ",";
    csv_file << data.crop_set << ",";

    csv_file << data.set_size << ",";
    csv_file << data.precision << ",";
    csv_file << data.recall << ",";
    csv_file << data.f1_score << ",";
    csv_file << data.accuracy << ",";
    csv_file << data.miou << ",";
    csv_file << data.tp << ",";
    csv_file << data.tn << ",";
    csv_file << data.fp << ",";
    csv_file << data.fn << ",";
    csv_file << data.exec_time << ",";
    csv_file << data.ground_size << ",";
    csv_file << data.truss_size << ",";

    csv_file << data.density_enable << ",";
    csv_file << data.density_first << ",";
    csv_file << data.density_radius << ",";
    csv_file << data.density_threshold << ",";

    csv_file << data.euclid_enable << ",";
    csv_file << data.euclid_radius << ",";
    csv_file << data.euclid_min_size << "\n";

    csv_file.close();
}