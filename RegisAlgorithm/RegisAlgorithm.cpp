#include "RegisAlgorithm.h"

#include <vector>

//#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>


Eigen::Matrix4f TRegisAlgorithm::icp_point_point(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target)
{
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	icp.setMaxCorrespondenceDistance(1); //设置最大对应点的欧氏距离
	icp.setMaximumIterations(100); //终止条件：设置最大迭代次数
	icp.setTransformationEpsilon(1e-10); //终止条件：设置前后两次迭代的转换矩阵的最大容差
	icp.setEuclideanFitnessEpsilon(0.01); //终止条件：设置前后两次迭代的点对的欧式距离均值的最大容差
	//icp.min_number_correspondences_ = 100;//最小匹配点对数量 建议将该值设置大点，否则容易出现错误匹配

	icp.setInputSource(cloud_source);
	icp.setInputTarget(cloud_target);

	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);

	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;

	return icp.getFinalTransformation();
}

Eigen::Matrix4f TRegisAlgorithm::ndt(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target)
{
	//滤波预处理	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
	approximate_voxel_filter.setInputCloud(cloud_source);
	approximate_voxel_filter.filter(*cloud_filtered);

	//参数设置
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	//根据输入数据的尺度设置NDT相关参数
	ndt.setMaximumIterations(35); //设置匹配迭代的最大次数
	ndt.setResolution(1.0); //设置NDT网格结构的分辨率（VoxelGridCovariance）
	ndt.setStepSize(0.1); //More-Thuente线搜索设置最大步长
	ndt.setTransformationEpsilon(0.01); //终止条件设置最小转换差异

	ndt.setInputSource(cloud_filtered); //设置源点云
	ndt.setInputTarget(cloud_target); //设置目标点云

	//计算需要的刚体变换以便将输入的源点云匹配到目标点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	ndt.align(*output_cloud);

	std::cout << "has converged:" << ndt.hasConverged() << " score: " <<
		ndt.getFitnessScore() << std::endl;

	return ndt.getFinalTransformation();
}
