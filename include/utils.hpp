#pragma once

#include <iostream>
#include <filesystem>
#include <algorithm>
#include <vector>
#include <unordered_set>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

#include "arvc_utils/arvc_utils.hpp"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace fs = std::filesystem;

namespace utils {

    struct GtIndices
    {
        pcl::IndicesPtr truss;
        pcl::IndicesPtr ground;

        GtIndices() {
            truss = pcl::IndicesPtr(new std::vector<int>);
            ground = pcl::IndicesPtr(new std::vector<int>);
        }
    };

    struct ConfusionMatrix
    {
        int tp;
        int tn;
        int fp;
        int fn;
    };


    pcl::PointCloud<pcl::PointXYZL>::Ptr read_cloud(fs::path _path)
    {

        pcl::PCDReader reader;
        pcl::PointCloud<pcl::PointXYZL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZL>);

        reader.read(_path.string(), *cloud);
        return cloud;
    }

    GtIndices get_ground_truth_indices(pcl::PointCloud<pcl::PointXYZL>::Ptr &_cloud)
    {

        GtIndices indices;

        for (size_t i = 0; i < _cloud->points.size(); i++)
        {

            if (_cloud->points[i].label > 0)
            {
                indices.truss->push_back(i);
            }
            else
                indices.ground->push_back(i);
        }
        return indices;
    }


    std::vector<int>
    vector_difference(std::vector<int> v1, std::vector<int> v2)
    {
        std::vector<int> difference;

        // FIRST METHOD - TIME CONSUMING
        // for (int i = 0; i < v1.size(); i++)
        // {
        //     for (int j = 0; j < v2.size(); j++)
        //     {
        //         if (v1[i] != v2[j])
        //         {
        //             difference.push_back(v1[i]);
        //         }
        //     }
        // }

        // SECOND METHOD
        std::unordered_set<int> set_v2(v2.begin(), v2.end());

        for (const int& elem : v1) {
            if (set_v2.find(elem) == set_v2.end()) {
                difference.push_back(elem);
        }
    }

        return difference;
    }

    std::vector<int>
    vector_intersection(std::vector<int> v1, std::vector<int> v2)
    {
        std::vector<int> intersection;

        // FIRST METHOD - TIME CONSUMING
        // for (int i = 0; i < v1.size(); i++)
        // {
        //     for (int j = 0; j < v2.size(); j++)
        //     {
        //         if (v1[i] == v2[j])
        //         {
        //             intersection.push_back(v1[i]);
        //         }
        //     }
        // }

        // SECOND METHOD
        std::unordered_set<int> set_v1(v1.begin(), v1.end());

        for (const int& elem : v2)
        {
            if (set_v1.find(elem) != set_v1.end())
            {
                intersection.push_back(elem);
                set_v1.erase(elem); // Optional: to avoid duplicates in the intersection
            }
        }

        return intersection;
    }


    ConfusionMatrix
    compute_conf_matrix(pcl::IndicesPtr &gt_truss_idx, pcl::IndicesPtr &gt_ground_idx, pcl::IndicesPtr &truss_idx, pcl::IndicesPtr &ground_idx)
    {
        ConfusionMatrix cm;

        cm.tp = vector_intersection(*truss_idx, *gt_truss_idx).size();
        cm.tn = vector_intersection(*ground_idx, *gt_ground_idx).size();
        cm.fp = vector_difference(*truss_idx, *gt_truss_idx).size();
        cm.fn = vector_difference(*ground_idx, *gt_ground_idx).size();

        return cm;
    }


    class Metrics
    {
        public:
        int tp;
        int tn;
        int fp;
        int fn;

        Metrics() {
            this->tp = 0;
            this->tn = 0;
            this->fp = 0;
            this->fn = 0;
        };

        float precision() {
            return (float)this->tp / (float)(this->tp + this->fp);
        }

        float recall() {
            return (float)this->tp / (float)(this->tp + this->fn);
        }

        float f1_score() {
            return 2 * (this->precision() * this->recall()) / (this->precision() + this->recall());
        }

        float accuracy() {
            return (float)(this->tp + this->tn) / (float)(this->tp + this->fp + this->fn + this->tn);
        }

        float miou() {
            return (float)this->tp / (float)(this->tp + this->fp + this->fn);
        }

        void plotMetrics(){
            std::cout << "Metrics: " << std::endl;
            std::cout << "\tPrecision: "  << this->precision() << std::endl;
            std::cout << "\tRecall: "     << this->recall() << std::endl;
            std::cout << "\tF1 Score: "   << this->f1_score() << std::endl;
            std::cout << "\tAccuracy: "   << this->accuracy() << std::endl;
            std::cout << "\tMIoU: "       << this->miou() << std::endl;
        }
    };

}