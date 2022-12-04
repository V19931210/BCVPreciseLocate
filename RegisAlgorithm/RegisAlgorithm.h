#pragma once

#include <iostream>
#include <pcl/io/pcd_io.h>

#ifndef TRREGISALRORITHM_EXPORT
#define TRREGISALRORITHM_EXPORT __declspec(dllexport)
#endif

namespace TRegisAlgorithm
{
	/**
	 * \brief ICP�㷨
	 * \param cloud_source  ������������
	 * \param cloud_target  �������
	 * \return
	 */
	Eigen::Matrix4f TRREGISALRORITHM_EXPORT icp_point_point(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target);
	Eigen::Matrix4f TRREGISALRORITHM_EXPORT ndt(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target);
};


