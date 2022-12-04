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
	void connect_assemble(); //connect函数集合
	void set_para_ui(TCameraPara para); //根据para设置ui显示的参数
	void set_para_camera(TCameraPara para); //根据para设置相机
	void set_hdr_model_ui(PropertyExtension value); //根据value设置ui显示的参数
	void set_hdr_para_ui(PropertyExtension value); //根据value设置ui显示的参数
	void set_hdr_camera(PropertyExtension value); //根据value设置相机hdr

	void realtime_display_Depth();//输出深度流
	void realtime_display_pointcloud();//输出点云流

private slots:
	void open_hand_eye_2D(); //打开2D标定界面
	void open_hand_eye_3D(); //打开3D标定界面
	void open_registration(); //打开配准界面
	void set_depth_range(); //设置深度范围
	void set_exposure(); //设置曝光
	void set_gain(); //设置增益
	void set_auto_exposure(); //设置自动曝光
	void on_btn_save_para_clicked(); //保存相机参数
	void on_btn_load_para_clicked(); //加载相机参数
	void set_HDR_model(int hdr_model); //HDR模式设置
	void set_HDR_level(int hdr_level); //HDR级数设置
	void set_HDR_level1_exposure(); //HDR第一级参数曝光时间
	void set_HDR_level2_exposure(); //HDR第二级参数曝光时间
	void set_HDR_level3_exposure(); //HDR第三级参数曝光时间
	void set_HDR_level4_exposure(); //HDR第四级参数曝光时间
	void set_HDR_level1_gain(); //HDR第一级参数增益
	void set_HDR_level2_gain(); //HDR第二级参数增益
	void set_HDR_level3_gain(); //HDR第三级参数增益
	void set_HDR_level4_gain(); //HDR第四级参数增益

	void realtime_stream_changed(int index);//主界面切换深度流

	void on_btn_get_point_cloud_ply_clicked(); //获取ply格式深度图
	void on_btn_get_point_cloud_bmp_clicked(); //获取bmp格式深度图
	void on_btn_open_pc_clicked(); //打开点云文件
	void on_btn_restart_depth_stream_clicked();//重启深度流
	void init_vtk_realtime();//初始化qvtkwidget
	void init_viewer();//初始化qvtkwidget

	void on_btn_stl2pcd_clicked();
	void on_btn_stl2ply_clicked();
	void trans_stl(int a_type = 0);//stl转pcd or ply  0为pcd 1为ply
	void on_btn_obj2pcd_clicked();
	void on_btn_obj2ply_clicked();
	void trans_obj(int a_type = 0);//stl转pcd or ply  0为pcd 1为ply
	void on_btn_test_clicked();

private:
	Ui::TMainWindowClass ui;
	TCameraChishine* m_camera_chishine;
	THandEyeCali2D* m_widgrt_handeye;
	THandEyeCali3D* m_widgrt_handeye_3D;
	TRegister* m_widgrt_regiostration;
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;//从相机抓取到的点云
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer_realtime;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer_once;
};