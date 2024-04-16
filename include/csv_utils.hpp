#pragma once

#include <iostream>
#include <fstream>
#include <istream>
#include <vector>
#include <string>
#include <filesystem>

namespace fs = std::filesystem;


std::string csv_header = "experiment_id,mode,set,set_size,precision,recall,f1_score,tp,tn,fp,fn,exec_time,ground_size,truss_size,density_threshold,euclidean_threshold,voxel_size,sac_threshold,node_length,node_width";

struct csv_data
{
    string experiment_id;
    string mode;
    int set;
    int set_size;
    float precision, recall, f1_score;
    int tp, tn, fp, fn;
    int exec_time;
    int ground_size;
    int truss_size;
    float density_threshold;
    float euclidean_threshold;
    float voxel_size;
    float sac_threshold;
    float node_length;
    float node_width;
};



void writeToCSV(const fs::path& dir_path, const csv_data& data) {
    
    if (!fs::exists(dir_path)) {
        fs::create_directories(dir_path);
    }

    fs::path csv_path = dir_path / "output.csv";
    
    
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

    csv_file << data.experiment_id << ","
         << data.mode << ","
         << data.set << ","
         << data.set_size << ","
         << data.precision << ","
         << data.recall << ","
         << data.f1_score << ","
         << data.tp << ","
         << data.tn << ","
         << data.fp << ","
         << data.fn << ","
         << data.exec_time << ","
         << data.ground_size << ","
         << data.truss_size << ","
         << data.density_threshold << ","
         << data.euclidean_threshold << ","
         << data.voxel_size << ","
         << data.sac_threshold << ","
         << data.node_length << ","
         << data.node_width << "\n";
        


    csv_file.close();
}