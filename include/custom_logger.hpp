#pragma once

#include <iostream>
#include <fstream>
#include <string>

class Logger {
private:
    std::ofstream logfile;

public:
    Logger(const std::string& filename) {
        logfile.open(filename, std::ios::app | std::ios::out);
        
        if (!logfile.is_open()) {
            std::cerr << "Error: Unable to open log file " << filename << std::endl;
        }
    }

    ~Logger() {
        if (logfile.is_open()) {
            logfile.close();
        }
    }

    void log(const std::string& message) {
        if (logfile.is_open()) {
            logfile << message << std::endl;
        }
    }
};