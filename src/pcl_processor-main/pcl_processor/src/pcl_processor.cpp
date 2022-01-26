#include "pcl_processor/pcl_processor.h"


PclProcessor::PclProcessor(){
  /* Niet netjes, weet nog niet hoe dat moet in 1 regel */
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  filtered_cloud = local_filtered_cloud;
  //filtered_cloud = new pcl::PointCloud<pcl::PointXYZ>;
  ROS_INFO("1");
}

PclProcessor::~PclProcessor(){
}

void PclProcessor::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  input_cloud = cloud;
  ROS_INFO("2");
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclProcessor::getFilteredCloud(){
  return filtered_cloud;
  ROS_INFO("3j");
}

geometry_msgs::Vector3 PclProcessor::getObjectPosition(){
  return object_position;
  ROS_INFO("4");
}

void PclProcessor::process() {
ROS_INFO("5");
  /* Filter the cloud */
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (input_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  //pass.setFilterLimitsNegative (true);
  /* The actal filtering */
  pass.filter(*filtered_cloud);

  /* Find the nearest objecy */
	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(input_cloud); // should be filtered_cloud, to be done
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);

	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(50);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(30);
	reg.setInputCloud(input_cloud);
	//reg.setIndices (indices); // No indices
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(1.0);

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
	int cnt = 0;
	pcl::PointXYZ CenterPoint;
	pcl::PointXYZ CenterPointMin;
	// Find neraest object
	for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
	{
		/* Create pointcloud from cluster */
		cloud_cluster->points.clear();
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->points.push_back(input_cloud->points[*pit]);
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		// Find centroid of the object
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*cloud_cluster, centroid);

		// Find distance of the object
		pcl::PointXYZ minPt, maxPt;
		pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);

		// Determinate neraest object from previos objects
		CenterPoint.x = centroid[0];
		CenterPoint.y = centroid[1];
		CenterPoint.z = minPt.z;
		if(!cnt){
			CenterPointMin = CenterPoint;
		}
		else{
			if(CenterPoint.z < CenterPointMin.z)
				CenterPointMin = CenterPoint;
		}
		cnt++;
	}

#if 0
  printf("x=%f\n", CenterPointMin.x);
  printf("y=%f\n", CenterPointMin.y);
  printf("z=%f\n", CenterPointMin.z);
#endif

	object_position.x = CenterPointMin.x;
	object_position.y = CenterPointMin.y;
	object_position.z = CenterPointMin.z;

	return;
}
