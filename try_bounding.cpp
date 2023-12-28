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
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

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

	// Eigen::Matrix3f covariance_matrix;
    // Eigen::Vector4f xyz_centroid_get;
    // Eigen::Vector3f xyz_centroid, translation;

    // pcl::compute3DCentroid(*cloud, xyz_centroid_get);
    // for (int i = 0; i < 3; i++)
    // {
    //     xyz_centroid[i] = xyz_centroid_get[i];
    // }
    // pcl::computeCovarianceMatrix(*cloud, xyz_centroid_get, covariance_matrix);

    // Eigen::EigenSolver<Eigen::Matrix3f> eigensolver(covariance_matrix);
    // Eigen::VectorXf eigen_values = eigensolver.eigenvalues().real();
    // Eigen::Matrix3f eigen_vectors = eigensolver.eigenvectors().real();
	// Eigen::Matrix4f transform;
	
    // translation = -eigen_vectors.transpose() * xyz_centroid;
    // for (int i = 0; i < 3; i++)
    // {
    //     for (int j = 0; j < 3; j++)
    //     {
    //         transform(i, j) = (eigen_vectors.transpose())(i, j);
    //     }
    // }
    // for (int i = 0; i < 3; i++)
    // {
    //     transform(i, 3) = translation[i];
    // }
    // pcl::transformPointCloud(*cloud, *cloud, transform);

	show(cloud);

	pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
	std::cout << max_pt.x - min_pt.x << std::endl;
	std::cout << max_pt.y - min_pt.y << std::endl;
	// std::cout << max_pt.z - min_pt.z << std::endl;


	return 0;
}
