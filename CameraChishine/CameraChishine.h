#pragma once

#include "bcvWeldNCShareMM.h"
#include "3DCamera.hpp"
#include <chrono>
#include <memory>
#include <functional>

#include <QJsonObject>
#include <QImage>

#include "hpp/HandEye/HandEye.hpp"
#include <Eigen/Dense>
#include <opencv.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#ifndef CAMERACHISHINE_EXPORT
#define CAMERACHISHINE_EXPORT __declspec(dllexport)
#endif

typedef Eigen::Matrix<float, 4, 4, 0, 4, 4> mat4;

struct TCameraPara
{
	float exposure_time;  //曝光时间
	float gain;           //增益
	float auto_exposure;  //是否开启自动曝光
	int depth_min;        //深度范围最小值
	int depth_max;        //深度范围最大值

	TCameraPara()
	{
		exposure_time = 50000;
		gain = 2;
		auto_exposure = 0;
		depth_min = 200;
		depth_max = 2000;
	}

	TCameraPara& operator=(const TCameraPara& m)
	{
		exposure_time = m.exposure_time;
		gain = m.gain;
		auto_exposure = m.auto_exposure;
		depth_min = m.depth_min;
		depth_max = m.depth_max;
		return *this;
	}

	QJsonObject toJson()
	{
		QJsonObject json;
		json.insert("exposure_time", static_cast<double>(exposure_time));
		json.insert("gain", static_cast<double>(gain));
		json.insert("auto_exposure", static_cast<double>(auto_exposure));
		json.insert("depth_min", static_cast<double>(depth_min));
		json.insert("depth_max", static_cast<double>(depth_max));
		return json;
	}

	void fromJson(QJsonObject json)
	{
		exposure_time = json.value("exposure_time").toDouble();
		gain = json.value("gain").toDouble();
		auto_exposure = json.value("auto_exposure").toDouble();
		depth_min = json.value("depth_min").toInt();
		depth_max = json.value("depth_max").toInt();
	}
};


/**
 * \brief 枚举 生成的点云文件格式为ply还是bmp
 */
enum class TPC_TYPE
{
	ePC_TYPE_PLY,
	ePC_TYPE_BMP
};

/**
 * \brief 枚举 左右IR图象
 */
enum class TIR_TYPE
{
	eIR_LEFT,
	eIR_RIGHT
};

/**
 * \brief 枚举 抓取图象的类型
 */
enum class TCAPTURE_TYPE
{
	eCAPTURE_SINGLE,  //单张抓取 适用于捕获单帧深度图 IR图 点云等
	eCAPTURE_STREAM   //连续抓取 
};

//XYZ的Point类型 现在暂时用知象SDK中的代替使用
typedef struct TPoint3f {
	float x;
	float y;
	float z;
}TPoint3f;


class CAMERACHISHINE_EXPORT TCameraChishine
{
private:
	//单例模式下将构造函数声明为私有 删除拷贝和赋值操作
	TCameraChishine();
	TCameraChishine(const TCameraChishine&) = delete;
	TCameraChishine(const TCameraChishine&&) = delete;
	TCameraChishine& operator=(const TCameraChishine&) = delete;
	TCameraChishine& operator=(const TCameraChishine&&) = delete;
	static TCameraChishine* m_this; //指向单例对象自身的静态指针 在static回调函数中使用 用于访问camera类内的非静态变量

public:
	//初始化相关
	~TCameraChishine() = default;
	static TCameraChishine* get_TCameraChishine(); //获取单例模式的实例 只能通过该函数实例化相机对象
	bool open_camera(); //打开相机
	bool is_open(); //判断相机是否打开
	void close_camera(); //关闭相机
	void show_information(); //显示相机信息
	void get_stream_info(); //存储并显示获取到的流信息
	bool start_depth_stream(STREAM_FORMAT format, cs::FrameCallback callback = nullptr); //初始化时开启深度流、切换流时stop之后start
	void stop_depth_stream(STREAM_TYPE streamType);//切换流时需要关闭当前流

	//相机参数相关
	TCameraPara get_para(); //获取相机参数
	void set_para_exposure(float exposure_time); //设置相机参数：曝光
	void set_para_gain(float gain); //设置相机参数：增益
	void set_para_auto_exposure(bool auto_exposure); //设置相机参数：自动曝光
	void set_para_depth_range(int depth_min, int depth_max); //设置相机参数：深度范围
	void set_HDR_model(HDR_MODE hdr_mode); //设置HDR模式
	void set_para_camera(TCameraPara para); //根据para设置相机参数
	void set_hdr_camera(PropertyExtension value); //根据value设置相机HDR参数
	bool get_hdr_para(PropertyExtension& value); //获取HDR参数信息
	bool get_hdr_model(PropertyExtension& value); //获取HDR模式信息
	void set_para_trigger_mode(); //设置相机参数：触发模式//暂时不需要改变触发模式

	//与QT界面参数调整相关的函数
	void set_ui_exposure();
	void set_ui_gain();
	void set_ui_set_para_depth_range();

	//主要功能
	static void depthCallback_IR(cs::IFramePtr frame, void* usrData); //IR图回调函数
	static void depthCallback_Depth(cs::IFramePtr frame, void* usrData); //深度图回调函数

	bool capture_frame(TCAPTURE_TYPE a_type); //从深度流中抓取数据
	bool calibrateEyeInHand(); //计算手眼矩阵
	bool get_TPoint6D(TPoint6D& a_TPoint6D); //共享内存获取TCP坐标
	static QImage Mat_to_QImage(cv::Mat a_mat); //将Mat格式转换为QImage格式
	bool get_pointcloud_api(pcl::PointCloud<pcl::PointNormal>& a_cloud);

private:
	bool get_IR(bool is_cali, cs::IFramePtr frame, void* usrData); //获取IR图 IR图回调函数中用
	bool cal_circle_points(cs::ICameraPtr camera, cs::IFramePtr frame, const char* imageL, const char* imageR,
		std::vector<cs::Point3f>& points, float& error); //角点重建函数 IR图回调函数中用
	bool get_point_cloud_ply_or_bmp(bool m_depth_is_ply, cs::IFramePtr frame, void* usrData); //获取深度图ply或bmp格式 深度图回调函数中用
	bool get_pointcloud(pcl::PointCloud<pcl::PointNormal>& a_cloud, cs::IFramePtr frame, cs::ICameraPtr camera); //获取pcl点云

public:
	//切换流的标志
	bool stream_IR_flag; //标记开启的深度流是否为IR
	bool stream_Depth_flag; //标记开启的深度流是否为深度

	//获取图象格式的标志
	bool m_depth_single_is_ply; //单帧深度图获取格式是否为ply true为ply文件 false为bmp文件
	bool m_depth_stream_is_ply; //连续深度图获取格式是否为ply true为点云流 false为深度流
	bool m_IR_single_is_cali; //获取的单帧IR图是否为标定所用
	
	//回调函数中需要保存的中间变量
	std::vector<std::vector<cs::Point3f>> board_points; //标定板上的三维角点坐标
	std::vector<cs::RobotPose> TCP_points; //拍照时的TCP坐标
	int capture_count; //记录取图成功的张数
	int frame_height; //回调函数中保存frame的高度
	int frame_width; //回调函数中保存frame的宽度
	unsigned char* imageL; //IR回调函数中保存的IR_L数据
	unsigned char* imageR; //IR回调函数中保存的IR_R数据
	std::vector<unsigned char> image_depth; //深度回调函数中保存的深度图地址指针
	pcl::PointCloud<pcl::PointNormal>::Ptr image_cloud;//深度回调函数中保存的点云数据指针

private:
	cs::ICameraPtr m_camera;//知象3D相机指针
	cs::IFramePtr m_frame;
	bool m_camera_is_open;//相机是否开启
	TCameraPara m_camera_para;//相机参数
	std::vector<StreamInfo> streamInfos_depth;//相机流参数
	TShareMemCypWeld m_share_memory;//共享内存指针
	std::mutex lock_for_callback;//互斥锁 回调函数中用 用于保护imageL、imageR、image_depth不会被同时写入数据

	//从流中抓取数据用的标志
	bool m_capture_single; //获取单帧数据请求
	bool m_detect_single; //获取单帧数据是否成功
	bool m_capture_stream; //获取连续数据请求
	bool m_detect_stream; //获取连续数据是否成功
};