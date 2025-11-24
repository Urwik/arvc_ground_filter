#include "arvc_ground_filter/ground_filter.hpp"


using namespace std;

GroundFilter::GroundFilter() {

  this->cloud_in = PointCloud::Ptr (new PointCloud);
  this->cloud_out = pcl::PointCloud<pcl::PointXYZL>::Ptr (new pcl::PointCloud<pcl::PointXYZL>);
  this->cloud_in_labeled = pcl::PointCloud<pcl::PointXYZL>::Ptr (new pcl::PointCloud<pcl::PointXYZL>);

  this->coarse_ground_idx = pcl::IndicesPtr (new pcl::Indices);
  this->coarse_truss_idx = pcl::IndicesPtr (new pcl::Indices);

  this->truss_idx = pcl::IndicesPtr (new pcl::Indices);
  this->ground_idx = pcl::IndicesPtr (new pcl::Indices);
  this->gt_truss_idx = pcl::IndicesPtr (new pcl::Indices);
  this->gt_ground_idx = pcl::IndicesPtr (new pcl::Indices);
  this->low_density_idx = pcl::IndicesPtr (new pcl::Indices);
  this->wrong_idx = pcl::IndicesPtr (new pcl::Indices);
  this->tp_idx = pcl::IndicesPtr (new pcl::Indices);
  this->fp_idx = pcl::IndicesPtr (new pcl::Indices);
  this->fn_idx = pcl::IndicesPtr (new pcl::Indices);
  this->tn_idx = pcl::IndicesPtr (new pcl::Indices);

  this->cons.enable_vis = false;
  this->enable_metrics = false;
  this->normals_time = 0;
  this->metrics_time = 0;
  this->ratio_threshold = 0.3f;
  this->module_threshold = 1000.0f;
  this->sac_threshold = 0.5f;
  this->mode = MODE::HYBRID;
  this->cloud_id = "NO_ID";
}


GroundFilter::~GroundFilter() {
}


void GroundFilter::set_input_cloud(pcl::PointCloud<pcl::PointXYZL>::Ptr &_cloud){

  // this->cloud_in_labeled = _cloud;
  
  // if(this->density_first) {
  //   this->enable_density_filter = false;
  //   this->density_filter();
  // }
  // gt_indices tmp_gt =  getGroundTruthIndices(this->cloud_in_labeled);

  gt_indices tmp_gt =  getGroundTruthIndices(_cloud);
  this->gt_truss_idx = tmp_gt.truss;
  this->gt_ground_idx = tmp_gt.ground;

  // this->ground_data_ratio = (float)this->gt_ground_idx->size() / ((float)this->gt_ground_idx->size() + (float)this->gt_truss_idx->size());

  pcl::copyPointCloud(*_cloud, *this->cloud_in);
}

void GroundFilter::set_mode(MODE _mode){
  this->mode = _mode;
}

void GroundFilter::set_node_length(float _length){
  this->node_length = _length;
}

void GroundFilter::set_node_width(float _width){
  this->node_width = _width;
}

void GroundFilter::set_sac_threshold(float _threshold){
  this->sac_threshold = _threshold;
}

void GroundFilter::set_voxel_size(float _voxel_size){
  this->voxel_size = _voxel_size;
}

int GroundFilter::compute()
{
  if(this->enable_density_filter && this->density_first) {
    *this->cloud_in = *arvc::density_filter_cloud( this->cloud_in, this->density_radius, this->density_threshold, false);
  }

  switch (this->mode)
  {
  case MODE::RATIO:
    this->ratio_threshold = this->node_width / this->sac_threshold;
    this->coarse_segmentation();
    this->fine_segmentation();
    this->update_segmentation();

    if(this->enable_density_filter and this->enable_euclidean_clustering)
      std::cout << "ERROR: Density filter and Euclidean clustering cannot be enabled at the same time" << std::endl;
    else if(this->enable_density_filter && !this->density_first)
      this->density_filter();
    else if(this->enable_euclidean_clustering)
        this->euclidean_clustering();
    break;
  
  case MODE::MODULE:
    this->module_threshold = this->sac_threshold;
    this->coarse_segmentation();
    this->fine_segmentation();
    this->update_segmentation();
    if(this->enable_density_filter and this->enable_euclidean_clustering)
      std::cout << "ERROR: Density filter and Euclidean clustering cannot be enabled at the same time" << std::endl;
    else if(this->enable_density_filter && !this->density_first)
      this->density_filter();
    else if(this->enable_euclidean_clustering)
        this->euclidean_clustering();
    break;

  case MODE::HYBRID:
    this->ratio_threshold = this->node_width / this->sac_threshold;
    this->module_threshold = this->sac_threshold;
    this->coarse_segmentation();
    this->fine_segmentation();
    this->update_segmentation();
    if(this->enable_density_filter and this->enable_euclidean_clustering)
      std::cout << "ERROR: Density filter and Euclidean clustering cannot be enabled at the same time" << std::endl;
    else if(this->enable_density_filter && !this->density_first)
      this->density_filter();
    else if(this->enable_euclidean_clustering)
        this->euclidean_clustering();
    break;

  case MODE::WOCOARSE_RATIO:
    this->ratio_threshold = this->node_width / this->node_length;
    this->fine_segmentation();
    this->update_segmentation();
    if(this->enable_density_filter and this->enable_euclidean_clustering)
      std::cout << "ERROR: Density filter and Euclidean clustering cannot be enabled at the same time" << std::endl;
    else if(this->enable_density_filter && !this->density_first)
      this->density_filter();
    else if(this->enable_euclidean_clustering)
        this->euclidean_clustering();
    break;

  case MODE::WOCOARSE_MODULE:
    this->module_threshold = this->node_length;
    this->fine_segmentation();
    this->update_segmentation();
    if(this->enable_density_filter and this->enable_euclidean_clustering)
      std::cout << "ERROR: Density filter and Euclidean clustering cannot be enabled at the same time" << std::endl;
    else if(this->enable_density_filter && !this->density_first)
      this->density_filter();
    else if(this->enable_euclidean_clustering)
        this->euclidean_clustering();
    break;

  case MODE::WOCOARSE_HYBRID:
    this->ratio_threshold = this->node_width / this->node_length;
    this->module_threshold = this->node_length;
    this->fine_segmentation();
    this->update_segmentation();
    if(this->enable_density_filter and this->enable_euclidean_clustering)
      std::cout << "ERROR: Density filter and Euclidean clustering cannot be enabled at the same time" << std::endl;
    else if(this->enable_density_filter && !this->density_first)
      this->density_filter();
    else if(this->enable_euclidean_clustering)
        this->euclidean_clustering();
    break;

  case MODE::WOFINE:
    this->coarse_segmentation();
    this->update_segmentation();
    if(this->enable_density_filter and this->enable_euclidean_clustering)
      std::cout << "ERROR: Density filter and Euclidean clustering cannot be enabled at the same time" << std::endl;
    else if(this->enable_density_filter && !this->density_first)
      this->density_filter();
    else if(this->enable_euclidean_clustering)
        this->euclidean_clustering();
    break;

  default:
    this->cons.debug("ERROR: Invalid mode selected, options are: RATIO, MODULE, HYBRID, WOFINE, WOCOARSE_RATIO, WOCOARSE_MODULE, WOCOARSE_HYBRID", "RED");
    break;
  }
  
  if (this->cons.enable_vis)
    this->view_final_segmentation();

  if(this->enable_metrics)
    this->compute_metrics();

  return 0;
}




void GroundFilter::coarse_segmentation(){
  this->cons.debug("Coarse segmentation", "GREEN");
  PointCloud::Ptr tmp_cloud (new PointCloud);
  pcl::ModelCoefficientsPtr tmp_plane_coefss (new pcl::ModelCoefficients);

  tmp_cloud = arvc::voxel_filter(this->cloud_in, this->voxel_size);
  tmp_plane_coefss = arvc::compute_planar_ransac(tmp_cloud, true, this->sac_threshold, 1000);
  auto coarse_indices = arvc::get_points_near_plane(this->cloud_in, tmp_plane_coefss, this->sac_threshold);
  this->coarse_ground_idx = coarse_indices.first;
  this->coarse_truss_idx = coarse_indices.second;
}

void GroundFilter::fine_segmentation(){
  this->cons.debug("Fine segmentation", "GREEN");

  std::pair<vector<pcl::PointIndices>, int> regrow_output;
  
  if(this->coarse_ground_idx->size() > 0)
    regrow_output = arvc::regrow_segmentation(this->cloud_in, this->coarse_ground_idx, false);
  else
    regrow_output = arvc::regrow_segmentation(this->cloud_in, false);  

  this->regrow_clusters = regrow_output.first;
  this->normals_time = regrow_output.second;

  this->validate_clusters();

  // Append valid clusters to truss indices
  for(int clus_indx : this->valid_clusters)
    this->coarse_truss_idx->insert(this->coarse_truss_idx->end(), this->regrow_clusters[clus_indx].indices.begin(), this->regrow_clusters[clus_indx].indices.end());

}

void GroundFilter::validate_clusters(){
  this->cons.debug("Validating clusters");

  map<string, int> mode_dict;
  mode_dict["ratio"] = 0;
  mode_dict["module"] = 1;
  mode_dict["hybrid"] = 2;


  switch (this->mode)
  {
  case MODE::RATIO:
    this->valid_clusters = this->validate_clusters_by_ratio();
    break;
  case MODE::MODULE:
    this->valid_clusters = this->validate_clusters_by_module();
    break;
  case MODE::HYBRID:
    this->valid_clusters = this->validate_clusters_hybrid();
    break;
  case MODE::WOCOARSE_RATIO:
    this->valid_clusters = this->validate_clusters_by_ratio();
    break;
  case MODE::WOCOARSE_MODULE:
    this->valid_clusters = this->validate_clusters_by_module();
    break;
  case MODE::WOCOARSE_HYBRID:
    this->valid_clusters = this->validate_clusters_hybrid();
    break;
  case MODE::WOFINE:
    break;
  default:
    break;
  }
}

void GroundFilter::density_filter(){

  // if (this->density_first){
  //   this->cons.debug("Density filter FIRST");
  //   pcl::RadiusOutlierRemoval<pcl::PointXYZL> radius_removal;
  //   radius_removal.setInputCloud(this->cloud_in_labeled);
  //   radius_removal.setRadiusSearch(this->density_radius);
  //   radius_removal.setMinNeighborsInRadius(this->density_threshold);
  //   radius_removal.setNegative(false);
  //   radius_removal.filter(*this->cloud_in_labeled);
  // }
  // else {
    this->cons.debug("Density filter");
    this->truss_idx = arvc::radius_outlier_removal(this->cloud_in, this->truss_idx, this->density_radius, this->density_threshold, false);
    this->ground_idx = arvc::inverseIndices(this->cloud_in, this->truss_idx);
  // }

}

void GroundFilter::euclidean_clustering(){
  this->cons.debug("Euclidean clustering");

  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud(this->cloud_in);

  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance( this->cluster_radius ); // 2cm
  ec.setMinClusterSize(this->cluster_min_size); // 100 points
  ec.setMaxClusterSize(this->cloud_in->size());
  ec.setSearchMethod(tree);
  ec.setInputCloud(this->cloud_in);
  ec.setIndices(this->truss_idx);
  ec.extract(this->euclid_clusters);

  // Get largest cluster
  int largest_cluster_size = 0;
  int largest_cluster_idx = 0;
  for (int i = 0; i < this->euclid_clusters.size(); i++)
  {
    if (this->euclid_clusters[i].indices.size() > largest_cluster_size)
    {
      largest_cluster_size = this->euclid_clusters[i].indices.size();
      largest_cluster_idx = i;
    }
  }

  *this->truss_idx = this->euclid_clusters[largest_cluster_idx].indices;
  *this->ground_idx = *arvc::inverseIndices(this->cloud_in, this->truss_idx);
}


void GroundFilter::update_segmentation(){
  // Update truss and ground indices for final segmentation
  *this->truss_idx = *this->coarse_truss_idx;
  *this->ground_idx = *arvc::inverseIndices(this->cloud_in, this->truss_idx);
}


bool GroundFilter::valid_ratio(pcl::IndicesPtr& _cluster_indices)
{
    auto eig_decomp = arvc::compute_eigen_decomposition(this->cloud_in, _cluster_indices, true);
    float size_ratio = eig_decomp.values(1)/eig_decomp.values(2);

    if (size_ratio <= this->ratio_threshold){
      return true;
    }
    else
      return false;
}

bool GroundFilter::valid_module(pcl::IndicesPtr& _cluster_indices){

  // GET THE CLOUD REPRESENTING THE CLUSTER
  PointCloud::Ptr cluster_cloud (new PointCloud);
  cluster_cloud = arvc::extract_indices(this->cloud_in, _cluster_indices);
  
  // COMPUTE CLOUD CENTROID
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cluster_cloud, centroid);

  // COMPUTE EIGEN DECOMPOSITION
  arvc::eig_decomp eigen_decomp = arvc::compute_eigen_decomposition(this->cloud_in, _cluster_indices, false);

  // Compute transform between the original cloud and the eigen vectors of the cluster
  Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
  projectionTransform.block<3,3>(0,0) = eigen_decomp.vectors.transpose();
  projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * centroid.head<3>());
  
  // Transform the origin to the centroid of the cluster and to its eigen vectors
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cluster_cloud, *cloudPointsProjected, projectionTransform);
  
  // Get the minimum and maximum points of the transformed cloud.
  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);

  Eigen::Vector3f max_values;
  max_values.x() = std::abs(maxPoint.x - minPoint.x);
  max_values.y() = std::abs(maxPoint.y - minPoint.y);
  max_values.z() = std::abs(maxPoint.z - minPoint.z);

  if (max_values.maxCoeff() < this->module_threshold)
    return true;
  else
    return false;
}


vector<int> GroundFilter::validate_clusters_by_ratio()
{
  vector<int> valid_clusters;
  valid_clusters.clear();
  pcl::IndicesPtr current_cluster (new pcl::Indices);
  
  int clust_indx = 0;
  for(auto cluster : this->regrow_clusters)
  {
    *current_cluster = cluster.indices;

    if (this->valid_ratio(current_cluster))
      valid_clusters.push_back(clust_indx);
      
    clust_indx++;
  }

  return valid_clusters;
}

vector<int> GroundFilter::validate_clusters_by_module()
{
  vector<int> valid_clusters;
  valid_clusters.clear();

  PointCloud::Ptr tmp_cloud (new PointCloud);
  PointCloud::Ptr current_cluster_cloud (new PointCloud);
  pcl::IndicesPtr current_cluster (new pcl::Indices);

  int clust_indx = 0;
  for(auto cluster : this->regrow_clusters)
  {
    *current_cluster = cluster.indices;

    if (this->valid_module(current_cluster))
      valid_clusters.push_back(clust_indx);
      
    clust_indx++;
  }

  return valid_clusters;
}

vector<int> GroundFilter::validate_clusters_hybrid()
{
  // cout << GREEN <<"Checking regrow clusters..." << RESET << endl;
  vector<int> valid_clusters;
  valid_clusters.clear();
  PointCloud::Ptr current_cluster_cloud (new PointCloud);
  PointCloud::Ptr remain_input_cloud (new PointCloud);
  pcl::IndicesPtr current_cluster (new pcl::Indices);

  int clust_indx = 0;
  for(auto cluster : this->regrow_clusters)
  {
    *current_cluster = cluster.indices;

    if (this->valid_module(current_cluster) && this->valid_ratio(current_cluster))
      valid_clusters.push_back(clust_indx);
    
    clust_indx++;
  }
  return valid_clusters;
}

void GroundFilter::compute_metrics(){
  auto start = std::chrono::high_resolution_clock::now();
  
  this->cm = arvc::compute_conf_matrix(this->gt_truss_idx, this->gt_ground_idx, this->truss_idx, this->ground_idx);

  this->metricas.computeMetricsFromConfusionMatrix(this->cm.TP, this->cm.FP, this->cm.FN, this->cm.TN);
  // this->metricas = arvc::compute_metrics(cm);

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  this->metrics_time = duration.count();
}


void GroundFilter::view_final_segmentation(){
    this->getConfMatrixIndexes();
    PointCloud::Ptr error_cloud (new PointCloud);
    PointCloud::Ptr truss_cloud (new PointCloud);
    PointCloud::Ptr ground_cloud (new PointCloud);
    pcl::IndicesPtr error_idx (new pcl::Indices);

    error_idx->insert(error_idx->end(), this->fp_idx->begin(), this->fp_idx->end());
    error_idx->insert(error_idx->end(), this->fn_idx->begin(), this->fn_idx->end());

    truss_cloud = arvc::extract_indices(cloud_in, this->tp_idx, false);
    ground_cloud = arvc::extract_indices(cloud_in,this->tn_idx, false);
    error_cloud = arvc::extract_indices(cloud_in, error_idx, false);

    pcl::visualization::PCLVisualizer my_vis;
    my_vis.setBackgroundColor(1,1,1);

    fs::path camera_params_path = "/home/fran/workSpaces/arvc_ws/src/" + this->cloud_id + "_camera_params.txt";

    try
    {
      my_vis.loadCameraParameters(camera_params_path.string());
      // my_vis.loadCameraParameters("/home/arvc/workSpaces/code_ws/build/" + this->cloud_id + "_camera_params.txt");

    }
    catch(const std::exception& e)
    {
      
    }
    

    pcl::visualization::PointCloudColorHandlerCustom<PointT> truss_color (truss_cloud, 50,190,50);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> ground_color (ground_cloud, 100,100,100);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> error_color (error_cloud, 200,10,10);


    my_vis.addPointCloud(truss_cloud, truss_color, "truss_cloud");
    my_vis.addPointCloud(ground_cloud, ground_color, "ground_cloud");
    my_vis.addPointCloud(error_cloud, error_color, "error_cloud");
    my_vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "truss_cloud");
    my_vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "ground_cloud");
    my_vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "error_cloud");

    my_vis.addCoordinateSystem(0.8, "sensor_origin");
    auto pos = cloud_in->sensor_origin_;
    auto ori = cloud_in->sensor_orientation_;
    

    while (!my_vis.wasStopped())
    {
      my_vis.saveCameraParameters(camera_params_path.string());
      my_vis.spinOnce(100);
    }
}


void GroundFilter::getConfMatrixIndexes()
{
  // For the index points classified as truss, check if this point is in the ground truth truss. If it is, it is a true positive, otherwise it is a false positive.
  for (size_t i = 0; i < this->truss_idx->size(); i++)
  {
    if(std::find(this->gt_truss_idx->begin(), this->gt_truss_idx->end(), this->truss_idx->at(i)) != this->gt_truss_idx->end())
      this->tp_idx->push_back(this->truss_idx->at(i));
    else
      this->fp_idx->push_back(this->truss_idx->at(i));
  }

  // For the index points classified as ground, check if this point is in the ground truth ground. If it is, it is a true negative, otherwise it is a false negative.
  for (size_t i = 0; i < this->ground_idx->size(); i++)
  {
    if(std::find(this->gt_ground_idx->begin(), this->gt_ground_idx->end(), this->ground_idx->at(i)) != this->gt_ground_idx->end())
      this->tn_idx->push_back(this->ground_idx->at(i));
    else
      this->fn_idx->push_back(this->ground_idx->at(i));
  }
  
}  