#include "CameraChishine.h"

#include <iostream>
#include <Thread>
#include <QDateTime>


#include "BmpUtil.hpp"

TCameraChishine* TCameraChishine::m_this = nullptr;

TCameraChishine::TCameraChishine()
{
	m_this = this;

	board_points.clear();
	TCP_points.clear();
	capture_count = 0;
	frame_height = 0;
	frame_width = 0;
	imageL = nullptr;
	imageR = nullptr;
	image_depth.clear();
	image_cloud.reset(new pcl::PointCloud<pcl::PointNormal>);
	stream_IR_flag = false;
	stream_Depth_flag = false;
	m_depth_single_is_ply = false;
	m_depth_stream_is_ply = false;
	m_IR_single_is_cali = false;

	m_camera = cs::getCameraPtr();
	m_camera_is_open = false;
	//m_camera_para;
	//streamInfos_depth;
	//m_share_memory;
	m_capture_single = false;
	m_detect_single = false;
	m_capture_stream = false;
	m_detect_stream = false;
}

TCameraChishine* TCameraChishine::get_TCameraChishine()
{
	static TCameraChishine single_instance;
	return &single_instance;
}

bool TCameraChishine::open_camera()
{
	if (is_open())
	{
		std::cout << "相机已打开" << std::endl << std::endl;
		return true;
	}

	ERROR_CODE ret;
	ret = m_camera->connect();
	if (ret != SUCCESS)
	{
		printf("camera open failed(%d)!\n\n", ret);
		return false;
	}
	std::cout << "相机连接成功" << std::endl << std::endl;
	m_camera_is_open = true;

	return true;
}

bool TCameraChishine::is_open()
{
	return m_camera_is_open;
}

void TCameraChishine::close_camera()
{
	ERROR_CODE ret;

	if (!is_open())
	{
		std::cout << "相机未打开！" << std::endl << std::endl;
		return;
	}
	ret = m_camera->disconnect();
	if (ret != SUCCESS)
	{
		printf("camera close failed(%d)!\n\n", ret);
		return;
	}
	m_camera = nullptr;
	m_camera_is_open = false;
	std::cout << "相机关闭成功！" << std::endl << std::endl;
}

void TCameraChishine::show_information()
{
	// get  informations of camera
	ERROR_CODE ret;
	CameraInfo info;
	ret = m_camera->getInfo(info);
	if (ret != SUCCESS)
	{
		printf("camera get info failed(%d)!\n", ret);
		return;
	}

	// display informations of camera
	printf("%20s  :  %s\n", "name", info.name);
	printf("%20s  :  %s\n", "serial", info.serial);
	printf("%20s  :  %s\n", "unique id", info.uniqueId);
	printf("%20s  :  %s\n", "firmware version", info.firmwareVersion);
	printf("%20s  :  %s\n", "algorithm version", info.algorithmVersion);
	printf("\n");
}

void TCameraChishine::get_stream_info()
{
	ERROR_CODE ret;

	//存储深度流信息
	ret = m_camera->getStreamInfos(STREAM_TYPE_DEPTH, streamInfos_depth);
	if (ret != SUCCESS)
	{
		printf("camera get stream info failed(%d)!\n", ret);
		return;
	}
	printf("获取到的相机流：\n");
	for (auto streamInfo : streamInfos_depth)
	{
		printf("depth format:%2d, width:%4d, height:%4d, fps:%2.1f\n",
			streamInfo.format, streamInfo.width, streamInfo.height, streamInfo.fps);
	}
	std::cout << std::endl;
}

bool TCameraChishine::start_depth_stream(STREAM_FORMAT format, cs::FrameCallback callback)
{
	ERROR_CODE ret;

	for (auto streamInfo : streamInfos_depth)
	{
		if (streamInfo.format == format)
		{
			//开启深度流，如果第一次开启失败，需要关闭现有流
			if (callback)
				ret = m_camera->startStream(STREAM_TYPE_DEPTH, streamInfo, callback, &m_camera);
			else
				ret = m_camera->startStream(STREAM_TYPE_DEPTH, streamInfo);

			if (ret != SUCCESS) 
			{
				m_camera->stopStream(STREAM_TYPE_DEPTH);
			}
			else
			{
				std::cout << "连接到相机流：" << std::endl;
				printf("start depth format:%2d, width:%4d, height:%4d, fps:%2.1f\n", streamInfo.format, streamInfo.width, streamInfo.height, streamInfo.fps);
				return true;
			}

			//开启流失败，关闭后二次打开
			if (callback)
				ret = m_camera->startStream(STREAM_TYPE_DEPTH, streamInfo, callback, (void*)&m_camera);
			else
				ret = m_camera->startStream(STREAM_TYPE_DEPTH, streamInfo);

			if (ret != SUCCESS)
			{
				printf("camera start depth stream failed(%d)!\n", ret);
				return false;
			}
			else
			{
				std::cout << "连接到相机流：" << std::endl;
				printf("start depth format:%2d, width:%4d, height:%4d, fps:%2.1f\n", streamInfo.format, streamInfo.width, streamInfo.height, streamInfo.fps);
				return true;
			}
		}
	}

	std::cout << "没有可用的相机流" << std::endl;
	return false;
}

void TCameraChishine::stop_depth_stream(STREAM_TYPE streamType)
{
	m_camera->stopStream(streamType);
}

TCameraPara TCameraChishine::get_para()
{
	TCameraPara para;

	m_camera->getProperty(STREAM_TYPE_DEPTH, PROPERTY_EXPOSURE, para.exposure_time);
	m_camera->getProperty(STREAM_TYPE_DEPTH, PROPERTY_GAIN, para.gain);

	PropertyExtension pros;
	m_camera->getPropertyExtension(PROPERTY_EXT_AUTO_EXPOSURE_MODE, pros);
	para.auto_exposure = pros.autoExposureMode;
	m_camera->getPropertyExtension(PROPERTY_EXT_DEPTH_RANGE, pros);
	para.depth_min = pros.depthRange.min;
	para.depth_max = pros.depthRange.max;

	return para;
}

void TCameraChishine::set_para_exposure(float exposure_time)
{
	ERROR_CODE ret;
	ret = m_camera->setProperty(STREAM_TYPE_DEPTH, PROPERTY_EXPOSURE, exposure_time);
	if (ret != SUCCESS)
	{
		printf("camera set exposure failed(%d)!\n\n", ret);
		return;
	}
	else
	{
		printf("camera set exposure success!\n\n");
	}
}

void TCameraChishine::set_para_gain(float gain)
{
	ERROR_CODE ret;
	ret = m_camera->setProperty(STREAM_TYPE_DEPTH, PROPERTY_GAIN, gain);
	if (ret != SUCCESS)
	{
		printf("camera set gain failed(%d)!\n\n", ret);
		return;
	}
	else
	{
		printf("camera set gain success!\n\n");
	}
}

void TCameraChishine::set_para_auto_exposure(bool auto_exposure)
{
	ERROR_CODE ret;
	PropertyExtension pros;
	if (auto_exposure)
		pros.autoExposureMode = AUTO_EXPOSURE_MODE_HIGH_QUALITY;
	else
		pros.autoExposureMode = AUTO_EXPOSURE_MODE_CLOSE;
	ret = m_camera->setPropertyExtension(PROPERTY_EXT_AUTO_EXPOSURE_MODE, pros);
	if (ret != SUCCESS)
	{
		printf("camera set auto exposure failed(%d)!\n\n", ret);
		return;
	}
	if (auto_exposure)
	{
		printf("camera set open auto exposure success!\n\n");
	}
	else
	{
		printf("camera set close auto exposure success!\n\n");
	}
}

void TCameraChishine::set_para_depth_range(int depth_min, int depth_max)
{
	ERROR_CODE ret;
	PropertyExtension depth_range;
	depth_range.depthRange.min = depth_min;
	depth_range.depthRange.max = depth_max;
	ret = m_camera->setPropertyExtension(PROPERTY_EXT_DEPTH_RANGE, depth_range);
	if (ret != SUCCESS)
	{
		printf("camera set depth range failed(%d)!\n\n", ret);
		return;
	}
	else
	{
		printf("camera set depth range success!\n\n");
	}
}

void TCameraChishine::set_HDR_model(HDR_MODE hdr_mode)
{
	ERROR_CODE ret;

	PropertyExtension value;
	value.hdrMode = hdr_mode;
	ret = m_camera->setPropertyExtension(PROPERTY_EXT_HDR_MODE, value);
	if (ret != SUCCESS)
	{
		printf("set hdr mode failed(%d)!\n", ret);
		return;
	}
	std::cout << "HDR模式" << hdr_mode << "设置成功" << std::endl << std::endl;
}

void TCameraChishine::set_para_camera(TCameraPara para)
{
	set_para_depth_range(para.depth_min, para.depth_max);
	set_para_exposure(para.exposure_time);
	set_para_gain(para.gain);
	set_para_auto_exposure(static_cast<bool>(para.auto_exposure));
}

void TCameraChishine::set_hdr_camera(PropertyExtension value)
{
	ERROR_CODE ret;
	ret = m_camera->setPropertyExtension(PROPERTY_EXT_HDR_EXPOSURE, value);
	if (ret != SUCCESS)
	{
		printf("HDR各级参数设置失败(%d)!\n\n", ret);
		return;
	}
	else
	{
		printf("HDR各级参数设置成功\n\n");
	}
}

bool TCameraChishine::get_hdr_para(PropertyExtension& value)
{
	ERROR_CODE ret = m_camera->getPropertyExtension(PROPERTY_EXT_HDR_EXPOSURE, value);

	if (ret != SUCCESS)
		return false;
	return true;
}

bool TCameraChishine::get_hdr_model(PropertyExtension& value)
{
	ERROR_CODE ret = m_camera->getPropertyExtension(PROPERTY_EXT_HDR_MODE, value);

	if (ret != SUCCESS)
		return false;
	return true;
}

//暂时不需要改变触发模式
void TCameraChishine::set_para_trigger_mode()
{
	//ERROR_CODE ret;
	//PropertyExtension triggerMode;
	//triggerMode.triggerMode = TRIGGER_MODE_SOFTWAER;
	//ret = m_camera->setPropertyExtension(PROPERTY_EXT_TRIGGER_MODE, triggerMode);
	//if (ret != SUCCESS)
	//{
	//	printf("camera set soft trigger mode failed(%d)!\n", ret);
	//	return;
	//}
	//else
	//{
	//	printf("camera set soft trigger mode success!\n");
	//}
}

void TCameraChishine::depthCallback_IR(cs::IFramePtr frame, void* usrData)
{
	m_this->lock_for_callback.lock();

	ERROR_CODE ret;
	cs::ICameraPtr camera = *((cs::ICameraPtr*)usrData);

	//获取单张IR图象
	if (m_this->m_capture_single == true)
	{
		const char* imageL = frame->getData(FRAME_DATA_FORMAT_IR_LEFT);
		const char* imageR = frame->getData(FRAME_DATA_FORMAT_IR_RIGHT);

		//获取普通IR图象
		if (m_this->m_IR_single_is_cali == false)
		{
			if (m_this->get_IR(m_this->m_IR_single_is_cali, frame, &camera))
			{
				//标记获取true
				m_this->m_detect_single = true;
			}
			else
			{
				//标记获取false
				m_this->m_detect_single = false;
			}
		}

		//进行手眼标定取图、重建等操作
		else
		{
			//保存标定用IR图象
			if (m_this->get_IR(m_this->m_IR_single_is_cali, frame, &camera))
			{
				std::cout << "标定用IR图象获取成功" << std::endl;

				std::vector<cs::Point3f> points;
				float error;//error值需要记录一下

				//角点提取失败 标记获取false
				if (!m_this->cal_circle_points(camera, frame, imageL, imageR, points, error))
				{
					m_this->m_detect_single = false;
					std::cout << "角点提取失败" << std::endl;
				}
				//角点提取成功 则继续获取TCP坐标
				else
				{
					TPoint6D tcp_tmp;
					//TCP获取失败 标记获取false
					if (!m_this->get_TPoint6D(tcp_tmp))
					{
						m_this->m_detect_single = false;
						std::cout << "TCP坐标记录失败!" << std::endl;
					}
					//TCP坐标也获取成功 标记获取true 且取图成功张数索引+1
					else
					{
						m_this->capture_count++;
						m_this->m_detect_single = true;

						//将角点存入变量board_points
						m_this->board_points.push_back(points);
						std::cout << "角点提取成功" << std::endl;

						//将TCP坐标存入static变量TCP_points  封装一下
						cs::RobotPose tcp;
						tcp.x = tcp_tmp.x;
						tcp.y = tcp_tmp.y;
						tcp.z = tcp_tmp.z;
						tcp.alfa = tcp_tmp.rx;
						tcp.beta = tcp_tmp.ry;
						tcp.gamma = tcp_tmp.rz;
						m_this->TCP_points.push_back(tcp);
						std::cout << "TCP坐标记录成功" << std::endl;
					}
				}
			}
			else
			{
				std::cout << "标定用IR图象获取失败" << std::endl;
			}
		}

		//结束该次capture
		m_this->m_capture_single = false;
	}

	//连续输出IR图象 将数据存至全局变量imageL和imageR 供标定界面使用
	if (m_this->m_capture_stream == true)
	{
		m_this->frame_height = 0;
		m_this->frame_width = 0;
		m_this->imageL = nullptr;
		m_this->imageR = nullptr;
		m_this->frame_height = frame->getHeight();
		m_this->frame_width = frame->getWidth();
		m_this->imageL = (unsigned char*)frame->getData(FRAME_DATA_FORMAT_IR_LEFT);
		m_this->imageR = (unsigned char*)frame->getData(FRAME_DATA_FORMAT_IR_RIGHT);

		//获取数据失败 标记false
		if (m_this->frame_height == 0 || m_this->frame_width == 0 || !m_this->imageL || !m_this->imageR)
		{
			m_this->m_detect_stream = false;
		}
		//获取数据成功 标记true
		else
			m_this->m_detect_stream = true;

		//结束该次capture
		m_this->m_capture_stream = false;
	}

	m_this->lock_for_callback.unlock();
}

void TCameraChishine::depthCallback_Depth(cs::IFramePtr frame, void* usrData)
{
	m_this->lock_for_callback.lock();

	cs::ICameraPtr camera = *((cs::ICameraPtr*)usrData);

	//获取单张深度图
	if (m_this->m_capture_single == true)
	{
		if (m_this->get_point_cloud_ply_or_bmp(m_this->m_depth_single_is_ply, frame, &camera))
		{
			//标记获取true
			m_this->m_detect_single = true;
		}
		else
		{
			//标记获取false
			m_this->m_detect_single = true;
		}

		//结束该次capture
		m_this->m_capture_single = false;
	}

	//获取连续深度图
	if (m_this->m_capture_stream == true)
	{
		//连续点云流
		if (m_this->m_depth_stream_is_ply)
		{
			pcl::PointCloud<pcl::PointNormal> cloud;

			//获取数据失败 标记获取false
			if (!m_this->get_pointcloud(cloud, frame, camera))//点云有效点数少于100
			{
				m_this->m_detect_stream = false;
			}
			//获取数据成功 标记获取true
			else
			{
				//将pc中保存的点云数据保存至image_cloud
				m_this->m_detect_stream = true;
				m_this->image_cloud->points = std::move(cloud.points);//移动拷贝 省空间
			}
		}

		//连续深度流
		else
		{
			m_this->frame_height = 0;
			m_this->frame_width = 0;
			m_this->image_depth.clear();
			m_this->frame_height = frame->getHeight();
			m_this->frame_width = frame->getWidth();
			int length = m_this->frame_height * m_this->frame_width;
			m_this->image_depth.resize(length * 3);

			cs::colorizer color;

			float scale = 0.1f;
			PropertyExtension value;
			if (SUCCESS == camera->getPropertyExtension(PROPERTY_EXT_DEPTH_SCALE, value))
				scale = value.depthScale;//深度图缩放系数

			//给深度数据渲染颜色
			color.process((unsigned short*)frame->getData(FRAME_DATA_FORMAT_Z16), scale, m_this->image_depth.data(), length);

			//获取数据失败 标记获取false
			if (m_this->frame_height == 0 || m_this->frame_width == 0 || m_this->image_depth.empty())
			{
				m_this->m_detect_stream = false;
			}
			//获取数据成功 标记获取true
			else
				m_this->m_detect_stream = true;
		}
		 
		//结束该次capture
		m_this->m_capture_stream = false;
	}

	m_this->lock_for_callback.unlock();
}

bool TCameraChishine::capture_frame(TCAPTURE_TYPE a_type)
{
	if (!is_open())
	{
		std::cout << "相机未打开！" << std::endl;
		return false;
	}

	if (a_type == TCAPTURE_TYPE::eCAPTURE_SINGLE)
	{
		m_capture_single = true;
		while (m_capture_single)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(30));
		}

		if (m_detect_single == true)
		{
			m_detect_single = false;
			return true;
		}
		else
		{
			return false;
		}
	}

	if (a_type == TCAPTURE_TYPE::eCAPTURE_STREAM)
	{
		m_capture_stream = true;
		while (m_capture_stream)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(30));
		}

		if (m_detect_stream == true)
		{
			m_detect_stream = false;
			return true;
		}
		else
		{
			return false;
		}
	}

	return false;
}

bool TCameraChishine::calibrateEyeInHand()
{
	ERROR_CODE ret;

	std::vector<cs::HandEyeCalibrationInput> inputs;
	for (int i = 0; i < TCP_points.size(); i++)
	{
		cs::HandEyeCalibrationInput input(cs::RobotPoseMatrix4f(TCP_points[i], cs::POSE_TYPE_EULER_XYZ, cs::POSE_UNIT_DEGREE), board_points[i]);
		inputs.push_back(input);
	}

	// calculate eye in hand matrix
	cs::HandEyeMatrix matrix;
	double error;
	ret = cs::calibrateEyeInHand(inputs, matrix, &error);

	if (ret != SUCCESS)
	{
		std::cout << "手眼矩阵计算失败 ERROR CODE： " << ret << std::endl;
		return false;
	}
	printf("eye in hand matrix :\n");
	printf("[%.4f\t%.4f\t%.4f\t%.4f \n", matrix.r00, matrix.r01, matrix.r02, matrix.tx);
	printf(" %.4f\t%.4f\t%.4f\t%.4f \n", matrix.r10, matrix.r11, matrix.r12, matrix.ty);
	printf(" %.4f\t%.4f\t%.4f\t%.4f \n", matrix.r20, matrix.r21, matrix.r22, matrix.tz);
	printf(" %.4f\t%.4f\t%.4f\t%.4f]\n", matrix.zero0, matrix.zero1, matrix.zero2, matrix.one);
	printf("error: %.4f\n", float(error));

	// save hand-eye matrix to `matrix_para.txt"`
	std::ofstream matrix_para;
	matrix_para.open("matrix_para.txt");
	if (matrix_para.is_open())
	{
		matrix_para << matrix.r00 << " " << matrix.r01 << " " << matrix.r02 << " " << matrix.tx << " " <<
			matrix.r10 << " " << matrix.r11 << " " << matrix.r12 << " " << matrix.ty << " " <<
			matrix.r20 << " " << matrix.r21 << " " << matrix.r22 << " " << matrix.tz << " " <<
			matrix.zero0 << " " << matrix.zero1 << " " << matrix.zero2 << " " << matrix.one << "\n";
	}
	matrix_para.close();

	return true;
}

bool TCameraChishine::get_TPoint6D(TPoint6D& a_TPoint6D)
{
	m_share_memory.openShareMM();
	if (m_share_memory.mOpenMMSuccess == false)
		return false;
	if (!m_share_memory.getRobotTcpPos(a_TPoint6D, 0))
		return false;
	return true;
}

QImage TCameraChishine::Mat_to_QImage(cv::Mat a_mat)
{
	QImage img;

	//灰度图象
	if (a_mat.channels() == 1)
	{
		img = QImage((const unsigned char*)(a_mat.data), a_mat.cols, a_mat.rows,
			a_mat.step, QImage::Format_Indexed8);
	}
	//RGB图象
	if (a_mat.channels() == 3)
	{
		img = QImage((a_mat.data), a_mat.cols, a_mat.rows,
			a_mat.step, QImage::Format_RGB888);
	}

	return img;
}

bool TCameraChishine::get_pointcloud_api(pcl::PointCloud<pcl::PointNormal>& a_cloud)
{
	if (get_pointcloud(a_cloud, m_frame, m_camera))
		return true;
	return false;
}

bool TCameraChishine::get_IR(bool is_cali, cs::IFramePtr frame, void* usrData)
{
	ERROR_CODE ret;

	cs::ICameraPtr camera = *((cs::ICameraPtr*)usrData);
	const char* imageL = frame->getData(FRAME_DATA_FORMAT_IR_LEFT);
	const char* imageR = frame->getData(FRAME_DATA_FORMAT_IR_RIGHT);

	//标定用IR图象
	if (is_cali)
	{
		if (imageL)
		{
			std::string filename = "HandEyeLeft" + std::to_string(capture_count);
			filename += ".bmp";
			saveBmp(imageL, frame->getWidth(), frame->getHeight(), 1, filename.c_str());
		}
		if (imageR)
		{
			std::string filename = "HandEyeRight" + std::to_string(capture_count);
			filename += ".bmp";
			saveBmp(imageR, frame->getWidth(), frame->getHeight(), 1, filename.c_str());
		}
	}

	//非标定用IR图象
	else
	{
		if (imageL)
		{
			QString filename = "./DepthIRLeft";
			filename += QDateTime::currentDateTime().toString("yyyyMMddhhmmss");
			filename += ".bmp";
			saveBmp(imageL, frame->getWidth(), frame->getHeight(), 1, filename.toStdString().c_str());
		}
		if (imageR)
		{
			QString filename = "./DepthIRRight";
			filename += QDateTime::currentDateTime().toString("yyyyMMddhhmmss");
			filename += ".bmp";
			saveBmp(imageR, frame->getWidth(), frame->getHeight(), 1, filename.toStdString().c_str());
		}
	}

	return imageL && imageR;
}

bool TCameraChishine::cal_circle_points(cs::ICameraPtr camera, cs::IFramePtr frame, const char* imageL,
	const char* imageR, std::vector<cs::Point3f>& points, float& error)
{
	ERROR_CODE ret;

	std::vector<cs::Point3f> constraintPoints;
	const int CircleBoardHorizontalPoints = 11;
	const int CircleBoardVerticalPoints = 9;
	const int gridsize = 15;
	const int circleradius = 4.5;
	std::vector<std::pair<int, int>> bigCircleSetting = { { 4,2 },{ 5,2 },{ 2,4 },{ 8,4 },{ 5,6 } };

	if (imageL)
	{
		std::string filename = "HandEyeLeft" + std::to_string(capture_count);
		filename += ".bmp";
		saveBmp(imageL, frame->getWidth(), frame->getHeight(), 1, filename.c_str());
	}
	if (imageR)
	{
		std::string filename = "HandEyeRight" + std::to_string(capture_count);
		filename += ".bmp";
		saveBmp(imageR, frame->getWidth(), frame->getHeight(), 1, filename.c_str());
	}
	points.reserve(CircleBoardHorizontalPoints * CircleBoardVerticalPoints);
	ret = cs::reconstructCircleboardPoints(camera, (const unsigned char*)imageL, (const unsigned char*)imageR,
		frame->getWidth(), frame->getHeight(), points, constraintPoints,
		CircleBoardHorizontalPoints, CircleBoardVerticalPoints, gridsize, circleradius, bigCircleSetting, error);

	if (ret == SUCCESS)
		return true;
	else
	{
		std::cout << "重建角点失败ERROR CODE: " << ret << std::endl;
		return false;
	}

}

bool TCameraChishine::get_point_cloud_ply_or_bmp(bool m_depth_is_ply, cs::IFramePtr frame, void* usrData)
{
	cs::ICameraPtr camera = *((cs::ICameraPtr*)usrData);

	if (!is_open())
	{
		std::cout << "相机未打开！" << std::endl << std::endl;
		return false;
	}

	//线程sleep100ms 保证获取到的流图象稳定
	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	//获取ply格式的深度图
	if (m_depth_is_ply)
	{
		cs::Pointcloud pc;

		Intrinsics intrinsic;
		camera->getIntrinsics(STREAM_TYPE_DEPTH, intrinsic);

		float scale = 0.1f;
		PropertyExtension value;
		if (SUCCESS == camera->getPropertyExtension(PROPERTY_EXT_DEPTH_SCALE, value))
			scale = value.depthScale;//深度图缩放系数

		//生成点云
		pc.generatePoints((unsigned short*)frame->getData(FRAME_DATA_FORMAT_Z16), frame->getWidth(), frame->getHeight(),
			scale, &intrinsic, nullptr, nullptr);

		QString cur_time = QDateTime::currentDateTime().toString("yyyyMMddhhmmss");
		QString file_name = "./pc";
		file_name += cur_time;
		file_name += ".ply";
		//保存点云
		pc.exportToFile(file_name.toStdString(), nullptr, 0, 0, true);

		//std::cout << "点云.ply获取成功" << std::endl << std::endl;
	}

	//获取bmp格式的深度图
	else
	{
		cs::colorizer color;

		std::vector<unsigned char> rgb;
		int length = frame->getHeight() * frame->getWidth();
		rgb.resize(length * 3);

		float scale = 0.1f;
		PropertyExtension value;
		if (SUCCESS == camera->getPropertyExtension(PROPERTY_EXT_DEPTH_SCALE, value))
			scale = value.depthScale;//深度图缩放系数

		//给深度数据渲染颜色
		color.process((unsigned short*)frame->getData(FRAME_DATA_FORMAT_Z16), scale, rgb.data(), length);

		QString cur_time = QDateTime::currentDateTime().toString("yyyyMMddhhmmss");
		QString file_name = "./pc";
		file_name += cur_time;
		file_name += ".bmp";
		//保存深度图
		saveBmp((const char*)rgb.data(), frame->getWidth(), frame->getHeight(), 3, file_name.toStdString().c_str());
	}

	return true;
}

bool TCameraChishine::get_pointcloud(pcl::PointCloud<pcl::PointNormal>& a_cloud, cs::IFramePtr frame, cs::ICameraPtr camera)
{
	m_frame = frame;
	if (!m_frame || !m_frame->getData(FRAME_DATA_FORMAT_Z16)) return false;

	cs::Pointcloud pc;

	Intrinsics intrinsic;
	camera->getIntrinsics(STREAM_TYPE_DEPTH, intrinsic);

	float scale = 0.1f;
	PropertyExtension value;
	if (SUCCESS == camera->getPropertyExtension(PROPERTY_EXT_DEPTH_SCALE, value))
		scale = value.depthScale;//深度图缩放系数

	//生成点云
	pc.generatePoints((unsigned short*)m_frame->getData(FRAME_DATA_FORMAT_Z16), m_frame->getWidth(), m_frame->getHeight(),
		scale, &intrinsic, nullptr, nullptr);

	if (pc.validSize() < 100)//如果有效点数少于100 返回false
		return false;

	a_cloud.points.clear();
	a_cloud.points.resize(pc.size());
	const auto vertices = pc.getVertices();
	const auto normals = pc.getNormals();

	//将pc中保存的点云数据保存至a_cloud
	for (int i = 0; i < pc.size(); i++)
	{
		a_cloud.points[i].x = vertices[i].x;
		a_cloud.points[i].y = vertices[i].y;
		a_cloud.points[i].z = vertices[i].z;
		a_cloud.points[i].normal_x = normals[i].x;
		a_cloud.points[i].normal_y = normals[i].y;
		a_cloud.points[i].normal_z = normals[i].z;
	}

	return true;
}