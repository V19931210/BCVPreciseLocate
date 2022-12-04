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

	icp.setMaxCorrespondenceDistance(1); //��������Ӧ���ŷ�Ͼ���
	icp.setMaximumIterations(100); //��ֹ��������������������
	icp.setTransformationEpsilon(1e-10); //��ֹ����������ǰ�����ε�����ת�����������ݲ�
	icp.setEuclideanFitnessEpsilon(0.01); //��ֹ����������ǰ�����ε����ĵ�Ե�ŷʽ�����ֵ������ݲ�
	//icp.min_number_correspondences_ = 100;//��Сƥ�������� ���齫��ֵ���ô�㣬�������׳��ִ���ƥ��

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
	//�˲�Ԥ����	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
	approximate_voxel_filter.setInputCloud(cloud_source);
	approximate_voxel_filter.filter(*cloud_filtered);

	//��������
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	//�����������ݵĳ߶�����NDT��ز���
	ndt.setMaximumIterations(35); //����ƥ�������������
	ndt.setResolution(1.0); //����NDT����ṹ�ķֱ��ʣ�VoxelGridCovariance��
	ndt.setStepSize(0.1); //More-Thuente������������󲽳�
	ndt.setTransformationEpsilon(0.01); //��ֹ����������Сת������

	ndt.setInputSource(cloud_filtered); //����Դ����
	ndt.setInputTarget(cloud_target); //����Ŀ�����

	//������Ҫ�ĸ���任�Ա㽫�����Դ����ƥ�䵽Ŀ�����
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	ndt.align(*output_cloud);

	std::cout << "has converged:" << ndt.hasConverged() << " score: " <<
		ndt.getFitnessScore() << std::endl;

	return ndt.getFinalTransformation();
}
