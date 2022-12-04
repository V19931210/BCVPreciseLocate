#pragma once

#include <QWidget>
#include "ui_FrmHandEyeCali3D.h"

#include "../CameraChishine/CameraChishine.h"
#include "../RegisAlgorithm/RegisAlgorithm.h"

#include <pcl/visualization/pcl_visualizer.h>

#include "QVTKWidget.h"
#include "vtkRenderWindow.h"

#include <Eigen/Core>
#include <vector>

//存储标定数据
//TCP_position：TCP位置
//mat_cam2CAD：相机点云到CAD点云的转换矩阵
struct THandEyeCaliData
{
	TPoint6D TCP_position;
	Eigen::Matrix4d mat_cam2CAD;

	THandEyeCaliData()
	{
		TCP_position.zero();
		mat_cam2CAD = Eigen::Matrix4d::Zero();
	}
};

typedef Eigen::Matrix4d Pose;
typedef std::vector<Pose> Poses;

class THandEyeCali3D : public QWidget
{
	Q_OBJECT

public:
	THandEyeCali3D(TCameraChishine* a_camera, QWidget* parent = Q_NULLPTR);
	~THandEyeCali3D() override;

private slots:
	void on_btn_load_CAD_clicked(); //加载CAD点云
	void on_btn_get_pointcloud_clicked(); //获取当前点云
	void on_btn_roi_cut_clicked(); //根据roi过滤点云
	void on_btn_registration_clicked(); //配准（配准成功的话记录TCP坐标位置）
	void on_btn_save_cali_data_clicked(); //保存标定数据
	void on_btn_cal_hadeye_mat_clicked(); //计算手眼矩阵
	void on_btn_test_clicked();

private:
	void init_vtkwidget();
	Eigen::Matrix4d tcp2tcp(TPoint6D target, TPoint6D source);
	Eigen::Matrix4d mat2mat(Eigen::Matrix4d target, Eigen::Matrix4d source);
	bool solveAXXB_1(Poses A_, Poses B_, Pose& X_);//conventional
	bool solveAXXB_2(Poses A_, Poses B_, Pose& X_);//Andreff
	bool solveAXXB_3(Poses A_, Poses B_, Pose& X_);//SVD
	Eigen::Matrix3d skew(Eigen::Vector3d u);

	void test();

private:
	Ui::THandEyeCali3D ui;
	TCameraChishine* m_camera_chishine_handeye3D;

	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud_cad; //加载的CAD点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud_cam; //相机拍摄的点云
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer_cad; //显示CAD点云的窗口
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer_cam; //显示相机点云的窗口
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer_ret; //显示配准结果的窗口
	Eigen::Matrix4f m_mat_regis;//点云配准结果
	std::vector<THandEyeCaliData> m_handeye_cali_data;//标定数据
	Eigen::Matrix4d m_mat_X;//手眼矩阵
};
