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

//�洢�궨����
//TCP_position��TCPλ��
//mat_cam2CAD��������Ƶ�CAD���Ƶ�ת������
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
	void on_btn_load_CAD_clicked(); //����CAD����
	void on_btn_get_pointcloud_clicked(); //��ȡ��ǰ����
	void on_btn_roi_cut_clicked(); //����roi���˵���
	void on_btn_registration_clicked(); //��׼����׼�ɹ��Ļ���¼TCP����λ�ã�
	void on_btn_save_cali_data_clicked(); //����궨����
	void on_btn_cal_hadeye_mat_clicked(); //�������۾���
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

	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud_cad; //���ص�CAD����
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud_cam; //�������ĵ���
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer_cad; //��ʾCAD���ƵĴ���
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer_cam; //��ʾ������ƵĴ���
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer_ret; //��ʾ��׼����Ĵ���
	Eigen::Matrix4f m_mat_regis;//������׼���
	std::vector<THandEyeCaliData> m_handeye_cali_data;//�궨����
	Eigen::Matrix4d m_mat_X;//���۾���
};
