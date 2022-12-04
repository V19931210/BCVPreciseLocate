#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_FrmMainWindow.h"

#include "../CameraChishine/CameraChishine.h"

#include "HandEyeCali2D.h"
#include "HandEyeCali3D.h"
#include "Register.h"

//#include <bcvfunctions.h>

//#include <QVTKWidget.h>

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


//#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL2);

class TMainWindow : public QMainWindow
{
	Q_OBJECT

public:
	TMainWindow(QWidget* parent = Q_NULLPTR);
	~TMainWindow();
	void connect_assemble(); //connect��������
	void set_para_ui(TCameraPara para); //����para����ui��ʾ�Ĳ���
	void set_para_camera(TCameraPara para); //����para�������
	void set_hdr_model_ui(PropertyExtension value); //����value����ui��ʾ�Ĳ���
	void set_hdr_para_ui(PropertyExtension value); //����value����ui��ʾ�Ĳ���
	void set_hdr_camera(PropertyExtension value); //����value�������hdr

	void realtime_display_Depth();//��������
	void realtime_display_pointcloud();//���������

private slots:
	void open_hand_eye_2D(); //��2D�궨����
	void open_hand_eye_3D(); //��3D�궨����
	void open_registration(); //����׼����
	void set_depth_range(); //������ȷ�Χ
	void set_exposure(); //�����ع�
	void set_gain(); //��������
	void set_auto_exposure(); //�����Զ��ع�
	void on_btn_save_para_clicked(); //�����������
	void on_btn_load_para_clicked(); //�����������
	void set_HDR_model(int hdr_model); //HDRģʽ����
	void set_HDR_level(int hdr_level); //HDR��������
	void set_HDR_level1_exposure(); //HDR��һ�������ع�ʱ��
	void set_HDR_level2_exposure(); //HDR�ڶ��������ع�ʱ��
	void set_HDR_level3_exposure(); //HDR�����������ع�ʱ��
	void set_HDR_level4_exposure(); //HDR���ļ������ع�ʱ��
	void set_HDR_level1_gain(); //HDR��һ����������
	void set_HDR_level2_gain(); //HDR�ڶ�����������
	void set_HDR_level3_gain(); //HDR��������������
	void set_HDR_level4_gain(); //HDR���ļ���������

	void realtime_stream_changed(int index);//�������л������

	void on_btn_get_point_cloud_ply_clicked(); //��ȡply��ʽ���ͼ
	void on_btn_get_point_cloud_bmp_clicked(); //��ȡbmp��ʽ���ͼ
	void on_btn_open_pc_clicked(); //�򿪵����ļ�
	void on_btn_restart_depth_stream_clicked();//���������
	void init_vtk_realtime();//��ʼ��qvtkwidget
	void init_viewer();//��ʼ��qvtkwidget

	void on_btn_stl2pcd_clicked();
	void on_btn_stl2ply_clicked();
	void trans_stl(int a_type = 0);//stlתpcd or ply  0Ϊpcd 1Ϊply
	void on_btn_obj2pcd_clicked();
	void on_btn_obj2ply_clicked();
	void trans_obj(int a_type = 0);//stlתpcd or ply  0Ϊpcd 1Ϊply
	void on_btn_test_clicked();

private:
	Ui::TMainWindowClass ui;
	TCameraChishine* m_camera_chishine;
	THandEyeCali2D* m_widgrt_handeye;
	THandEyeCali3D* m_widgrt_handeye_3D;
	TRegister* m_widgrt_regiostration;
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;//�����ץȡ���ĵ���
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer_realtime;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer_once;
};