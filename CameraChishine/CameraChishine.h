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
	float exposure_time;  //�ع�ʱ��
	float gain;           //����
	float auto_exposure;  //�Ƿ����Զ��ع�
	int depth_min;        //��ȷ�Χ��Сֵ
	int depth_max;        //��ȷ�Χ���ֵ

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
 * \brief ö�� ���ɵĵ����ļ���ʽΪply����bmp
 */
enum class TPC_TYPE
{
	ePC_TYPE_PLY,
	ePC_TYPE_BMP
};

/**
 * \brief ö�� ����IRͼ��
 */
enum class TIR_TYPE
{
	eIR_LEFT,
	eIR_RIGHT
};

/**
 * \brief ö�� ץȡͼ�������
 */
enum class TCAPTURE_TYPE
{
	eCAPTURE_SINGLE,  //����ץȡ �����ڲ���֡���ͼ IRͼ ���Ƶ�
	eCAPTURE_STREAM   //����ץȡ 
};

//XYZ��Point���� ������ʱ��֪��SDK�еĴ���ʹ��
typedef struct TPoint3f {
	float x;
	float y;
	float z;
}TPoint3f;


class CAMERACHISHINE_EXPORT TCameraChishine
{
private:
	//����ģʽ�½����캯������Ϊ˽�� ɾ�������͸�ֵ����
	TCameraChishine();
	TCameraChishine(const TCameraChishine&) = delete;
	TCameraChishine(const TCameraChishine&&) = delete;
	TCameraChishine& operator=(const TCameraChishine&) = delete;
	TCameraChishine& operator=(const TCameraChishine&&) = delete;
	static TCameraChishine* m_this; //ָ������������ľ�ָ̬�� ��static�ص�������ʹ�� ���ڷ���camera���ڵķǾ�̬����

public:
	//��ʼ�����
	~TCameraChishine() = default;
	static TCameraChishine* get_TCameraChishine(); //��ȡ����ģʽ��ʵ�� ֻ��ͨ���ú���ʵ�����������
	bool open_camera(); //�����
	bool is_open(); //�ж�����Ƿ��
	void close_camera(); //�ر����
	void show_information(); //��ʾ�����Ϣ
	void get_stream_info(); //�洢����ʾ��ȡ��������Ϣ
	bool start_depth_stream(STREAM_FORMAT format, cs::FrameCallback callback = nullptr); //��ʼ��ʱ������������л���ʱstop֮��start
	void stop_depth_stream(STREAM_TYPE streamType);//�л���ʱ��Ҫ�رյ�ǰ��

	//����������
	TCameraPara get_para(); //��ȡ�������
	void set_para_exposure(float exposure_time); //��������������ع�
	void set_para_gain(float gain); //�����������������
	void set_para_auto_exposure(bool auto_exposure); //��������������Զ��ع�
	void set_para_depth_range(int depth_min, int depth_max); //���������������ȷ�Χ
	void set_HDR_model(HDR_MODE hdr_mode); //����HDRģʽ
	void set_para_camera(TCameraPara para); //����para�����������
	void set_hdr_camera(PropertyExtension value); //����value�������HDR����
	bool get_hdr_para(PropertyExtension& value); //��ȡHDR������Ϣ
	bool get_hdr_model(PropertyExtension& value); //��ȡHDRģʽ��Ϣ
	void set_para_trigger_mode(); //�����������������ģʽ//��ʱ����Ҫ�ı䴥��ģʽ

	//��QT�������������صĺ���
	void set_ui_exposure();
	void set_ui_gain();
	void set_ui_set_para_depth_range();

	//��Ҫ����
	static void depthCallback_IR(cs::IFramePtr frame, void* usrData); //IRͼ�ص�����
	static void depthCallback_Depth(cs::IFramePtr frame, void* usrData); //���ͼ�ص�����

	bool capture_frame(TCAPTURE_TYPE a_type); //���������ץȡ����
	bool calibrateEyeInHand(); //�������۾���
	bool get_TPoint6D(TPoint6D& a_TPoint6D); //�����ڴ��ȡTCP����
	static QImage Mat_to_QImage(cv::Mat a_mat); //��Mat��ʽת��ΪQImage��ʽ
	bool get_pointcloud_api(pcl::PointCloud<pcl::PointNormal>& a_cloud);

private:
	bool get_IR(bool is_cali, cs::IFramePtr frame, void* usrData); //��ȡIRͼ IRͼ�ص���������
	bool cal_circle_points(cs::ICameraPtr camera, cs::IFramePtr frame, const char* imageL, const char* imageR,
		std::vector<cs::Point3f>& points, float& error); //�ǵ��ؽ����� IRͼ�ص���������
	bool get_point_cloud_ply_or_bmp(bool m_depth_is_ply, cs::IFramePtr frame, void* usrData); //��ȡ���ͼply��bmp��ʽ ���ͼ�ص���������
	bool get_pointcloud(pcl::PointCloud<pcl::PointNormal>& a_cloud, cs::IFramePtr frame, cs::ICameraPtr camera); //��ȡpcl����

public:
	//�л����ı�־
	bool stream_IR_flag; //��ǿ�����������Ƿ�ΪIR
	bool stream_Depth_flag; //��ǿ�����������Ƿ�Ϊ���

	//��ȡͼ���ʽ�ı�־
	bool m_depth_single_is_ply; //��֡���ͼ��ȡ��ʽ�Ƿ�Ϊply trueΪply�ļ� falseΪbmp�ļ�
	bool m_depth_stream_is_ply; //�������ͼ��ȡ��ʽ�Ƿ�Ϊply trueΪ������ falseΪ�����
	bool m_IR_single_is_cali; //��ȡ�ĵ�֡IRͼ�Ƿ�Ϊ�궨����
	
	//�ص���������Ҫ������м����
	std::vector<std::vector<cs::Point3f>> board_points; //�궨���ϵ���ά�ǵ�����
	std::vector<cs::RobotPose> TCP_points; //����ʱ��TCP����
	int capture_count; //��¼ȡͼ�ɹ�������
	int frame_height; //�ص������б���frame�ĸ߶�
	int frame_width; //�ص������б���frame�Ŀ��
	unsigned char* imageL; //IR�ص������б����IR_L����
	unsigned char* imageR; //IR�ص������б����IR_R����
	std::vector<unsigned char> image_depth; //��Ȼص������б�������ͼ��ַָ��
	pcl::PointCloud<pcl::PointNormal>::Ptr image_cloud;//��Ȼص������б���ĵ�������ָ��

private:
	cs::ICameraPtr m_camera;//֪��3D���ָ��
	cs::IFramePtr m_frame;
	bool m_camera_is_open;//����Ƿ���
	TCameraPara m_camera_para;//�������
	std::vector<StreamInfo> streamInfos_depth;//���������
	TShareMemCypWeld m_share_memory;//�����ڴ�ָ��
	std::mutex lock_for_callback;//������ �ص��������� ���ڱ���imageL��imageR��image_depth���ᱻͬʱд������

	//������ץȡ�����õı�־
	bool m_capture_single; //��ȡ��֡��������
	bool m_detect_single; //��ȡ��֡�����Ƿ�ɹ�
	bool m_capture_stream; //��ȡ������������
	bool m_detect_stream; //��ȡ���������Ƿ�ɹ�
};