#include <iostream>
#include <stdio.h>
#include <vector>
#include <math.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>

using namespace std;
using namespace pcl;

void stl2pcd(const char* stl_name){
	vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
	reader->SetFileName(stl_name);
	reader->Update();
	// 轉為polydata格式
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	polydata = reader->GetOutput();
	polydata->GetNumberOfPoints();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// 從polydata轉pcd
	pcl::io::vtkPolyDataToPointCloud(polydata, *cloud);
	// 顯示點雲
	pcl::visualization::PCLVisualizer visu3("stl2pcd");
	// visu3.setBackgroundColor(255, 255, 255);
	visu3.addPointCloud(cloud, "input_cloud");
	visu3.spin();
	// 保存pcd文件
	// pcl::io::savePCDFileASCII("stl2pcd.pcd", *cloud);
}

void show(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

    pcl::visualization::PCLVisualizer viewer("VOXELIZED SAMPLES CLOUD");
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "sample_cloud");

    viewer.setBackgroundColor(0, 0, 0);                                                                        // 窗口背景色，默认[0,0,0]，范围[0~255,0~255,0~255]
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample_cloud"); // 设置点的大小，默认 1
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "sample_cloud");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "sample_cloud");			//设置点云透明度，默认 1 【Float going from 0.0 (transparent) to 1.0 (opaque)】
    // viewer.addCoordinateSystem(100.0);
    // viewer.initCameraParameters ();

    viewer.spin();
}

vector<pcl::PointIndices> segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	// estimate normal
	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);

	// segmetation
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(100);
	reg.setMaxClusterSize(10000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(20);
	reg.setInputCloud(cloud);
	// reg.setIndices (indices);
	reg.setInputNormals(normals);
	//segment plane by curvature and smooth rate, which can avoid plan broken
	reg.setSmoothnessThreshold(10.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(1.0);

	std::vector<pcl::PointIndices> clusters;
	reg.extract(clusters);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	return clusters;
}

int main(int argc, char **argv){	
	if (argc < 2){
		cout<<"./3D_vision cad_name.pcd"<<endl;
		return (-1);
	}
	std::vector<int> pcd_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
	if(pcd_file_indices.size () != 1){
		cout<<"./3D_vision cad_name.pcd"<<endl;
		return (-1);
	}

	// Read in the cloud data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	// std::string name = "/home/isci/colcon_ws/PCD/";
	// name += argv[pcd_file_indices[0]];
	// stl2pcd(argv[1]);
	pcl::io::loadPCDFile<pcl::PointXYZ>(argv[pcd_file_indices[0]], *cloud);
	// pcl::io::loadPCDFile<pcl::PointXYZ>("cad_workpiece1.pcd", *cloud);
	show(cloud); 

	// filter part
	//set the parameter depended on enviriment
	pcl::IndicesPtr indices(new std::vector<int>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.4, 0.7);
	// pass.setFilterLimits(0.3, 0.50);
	pass.filter(*cloud);

	pass.setInputCloud(cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-0.20, 0.20);
	pass.filter(*cloud);

	pass.setInputCloud(cloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-0.30, -0.1);
	pass.filter(*cloud);

	// //segmentation and get the workpiece surface
	std::vector<pcl::PointIndices> clusters;
	pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
	clusters = segmentation(cloud);
	int count = 0;
	for(auto i : clusters){
		// pcl::copyPointCloud(*cloud, clusters[0].indices, *outputCloud);
		pcl::copyPointCloud(*cloud, i.indices, *outputCloud);
		std::cout << count <<std::endl;
		show(outputCloud); 
		count ++;
	}
	size_t index;
	std::cout << "choice pcd" << std::endl;
	std::cin >> index ;
	pcl::copyPointCloud(*cloud, clusters[index].indices, *outputCloud);

	show(outputCloud);
	pcl::io::savePCDFileASCII("damin1030.pcd", *outputCloud);

	return 0;
}
