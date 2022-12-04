#include "HandEyeCali2D.h"

#include <iostream>
#include <thread>
#include <hpp/HandEye/HandEye.hpp>

#include <QMessageBox>
#include <QValidator>
#include <QCloseEvent>

#include "opencv2/imgproc/types_c.h"

THandEyeCali2D::THandEyeCali2D(TCameraChishine* a_camera, QWidget* parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	this->setFixedSize(this->width(), this->height());

	m_camera_chishine_handeye = a_camera;

	std::cout << "进入2D标定模式..." << std::endl << std::endl;

	//进入标定模式时清空camera记录的坐标
	m_camera_chishine_handeye->capture_count = 0;//清空capture记录
	m_camera_chishine_handeye->board_points.clear();//清空存储的角点
	m_camera_chishine_handeye->TCP_points.clear();//清空存储的TCP坐标

	if (m_camera_chishine_handeye->open_camera())
	{
		//更改ui状态
		ui.lineEdit_exposure_time->setEnabled(true);
		ui.lineEdit_gain->setEnabled(true);

		//初始化相机参数
		TCameraPara para = m_camera_chishine_handeye->get_para();
		ui.lineEdit_exposure_time->setText(QString::number(static_cast<int>(para.exposure_time)));
		ui.lineEdit_gain->setText(QString::number(static_cast<int>(para.gain)));

		//关闭主界面的流，开启IR流进入标定模式
		m_camera_chishine_handeye->stream_IR_flag = false;
		m_camera_chishine_handeye->stream_Depth_flag = false;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		m_camera_chishine_handeye->stop_depth_stream(STREAM_TYPE_DEPTH);

		if (m_camera_chishine_handeye->start_depth_stream(STREAM_FORMAT_PAIR, TCameraChishine::depthCallback_IR))
		{
			m_camera_chishine_handeye->stream_IR_flag = true;
			std::thread thread1(&THandEyeCali2D::realtime_display_IR, this);
			thread1.detach();
		}
		else
		{
			std::cout << "切换IR流失败，请重启标定界面" << std::endl;
		}
	}
	else
	{
		std::cout << "相机打开失败！" << std::endl;
	}

	connect(ui.lineEdit_exposure_time, &QLineEdit::returnPressed, this, &THandEyeCali2D::set_exposure);
	connect(ui.lineEdit_gain, &QLineEdit::returnPressed, this, &THandEyeCali2D::set_gain);
	ui.lineEdit_exposure_time->setValidator(new QIntValidator(0, 60000, this));
	ui.lineEdit_gain->setValidator(new QIntValidator(1, 15, this));

}

void THandEyeCali2D::closeEvent(QCloseEvent* event)
{
	QMessageBox::StandardButton button = QMessageBox::question(this, tr("Close Calibration"),
		QString(tr("Are you sure to leave?")),
		QMessageBox::Yes | QMessageBox::No);
	if (button == QMessageBox::No)
	{
		event->ignore();
	}
	else
	{
		m_camera_chishine_handeye->stream_IR_flag = false;
		m_camera_chishine_handeye->stream_Depth_flag = false;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		m_camera_chishine_handeye->stop_depth_stream(STREAM_TYPE_DEPTH);

		//关闭界面时开启主界面的深度流
		//if (m_camera_chishine_handeye->start_depth_stream(STREAM_FORMAT_Z16, TCameraChishine::depthCallback_Depth))
		//{
		//	m_camera_chishine_handeye->stream_Depth_flag = true;
		//	//如何通知主线程调用realtime_display_depth函数？xinhaocao?
		//}
		//else
		//{
		//	std::cout << "切换深度图流失败，请点击主界面重启深度流" << std::endl;
		//}
		std::cout << "标定界面已关闭" << std::endl << std::endl;

		event->accept();
		emit restart_frmmain_stream();
	}
}

//这个函数可以封装到camera中 把lineedit作为参数传入进去
void THandEyeCali2D::set_exposure()
{
	float exposure_time = ui.lineEdit_exposure_time->text().toFloat();
	m_camera_chishine_handeye->set_para_exposure(exposure_time);

	TCameraPara para = m_camera_chishine_handeye->get_para();
	ui.lineEdit_exposure_time->setText(QString::number(static_cast<int>(para.exposure_time)));
	ui.lineEdit_gain->setText(QString::number(static_cast<int>(para.gain)));
}

void THandEyeCali2D::set_gain()
{
	float gain = ui.lineEdit_gain->text().toFloat();
	m_camera_chishine_handeye->set_para_gain(gain);

	TCameraPara para = m_camera_chishine_handeye->get_para();
	ui.lineEdit_exposure_time->setText(QString::number(static_cast<int>(para.exposure_time)));
	ui.lineEdit_gain->setText(QString::number(static_cast<int>(para.gain)));
}

void THandEyeCali2D::on_btn_get_IR_clicked()
{
	//标记要采集单张IR图不重建角点 然后开启单张捕获
	m_camera_chishine_handeye->m_IR_single_is_cali = false;
	if (m_camera_chishine_handeye->capture_frame(TCAPTURE_TYPE::eCAPTURE_SINGLE))
	{
		std::cout << "IR图象获取成功" << std::endl;
	}
	else
	{
		std::cout << "IR图象获取失败" << std::endl;
	}
}

void THandEyeCali2D::on_btn_capture_clicked()
{
	if (m_camera_chishine_handeye->capture_count >= 5) return;

	//标记要采集单张IR图并重建角点 然后开启单张捕获
	m_camera_chishine_handeye->m_IR_single_is_cali = true;
	if (m_camera_chishine_handeye->capture_frame(TCAPTURE_TYPE::eCAPTURE_SINGLE))
	{
		std::cout << "标定图象已获取并重建角点成功" << std::endl;
		std::cout << "已取图 " << m_camera_chishine_handeye->capture_count << " 张,还需取图 "
			<< 5 - m_camera_chishine_handeye->capture_count << " 张" << std::endl;
	}

	else
	{
		std::cout << "相机取图失败，请重新拍照" << std::endl;
	}

	if (m_camera_chishine_handeye->capture_count >= 5)
	{
		ui.btn_capture->setEnabled(false);
		ui.btn_calculate->setEnabled(true);
	}
}

void THandEyeCali2D::on_btn_calculate_clicked()
{
	if (m_camera_chishine_handeye->calibrateEyeInHand())
	{
		std::cout << "手眼矩阵计算成功" << std::endl;

		//手眼矩阵计算成功后即清空掉拍照张数、存储的角点、TCP坐标等的记录
		//如果需要重复计算的话 就在退出手眼标定界面的时候清空这些记录而不是在这个函数里清空
		m_camera_chishine_handeye->capture_count = 0;
		m_camera_chishine_handeye->board_points.clear();
		m_camera_chishine_handeye->TCP_points.clear();

		ui.btn_capture->setEnabled(true);
		ui.btn_calculate->setEnabled(false);
	}
	else
	{
		std::cout << "手眼矩阵计算失败" << std::endl;
	}
}

void THandEyeCali2D::realtime_display_IR()
{
	cv::Mat mat_left;
	cv::Mat mat_right;
	QImage image_left;
	QImage image_right;

	//检测到深度IR流处于开启状态即进入循环显示实时图像
	//如果需要更换输出流的话可以将stream_IR_flag设置为false，其他流处理完毕之后切回IR流再将stream_IR_flag设置为true，同时调用该函数
	while (m_camera_chishine_handeye->stream_IR_flag)
	{
		if (m_camera_chishine_handeye->capture_frame(TCAPTURE_TYPE::eCAPTURE_STREAM))
		{
			mat_left = cv::Mat(m_camera_chishine_handeye->frame_height, m_camera_chishine_handeye->frame_width,
				CV_8UC1, m_camera_chishine_handeye->imageL);
			mat_right = cv::Mat(m_camera_chishine_handeye->frame_height, m_camera_chishine_handeye->frame_width,
				CV_8UC1, m_camera_chishine_handeye->imageR);

			if (mat_left.data != nullptr)
			{
				ui.label_IR_L->setScaledContents(true);
				image_left = TCameraChishine::Mat_to_QImage(mat_left);
				if (!image_left.isNull())
					ui.label_IR_L->setPixmap(QPixmap::fromImage(image_left));
			}

			else
			{
				std::cout << "左相机数据为空" << std::endl;
			}

			if (mat_right.data != nullptr)
			{
				ui.label_IR_R->setScaledContents(true);
				image_right = TCameraChishine::Mat_to_QImage(mat_right);
				if (!image_right.isNull())
					ui.label_IR_R->setPixmap(QPixmap::fromImage(image_right));

			}

			else
			{
				std::cout << "右相机数据为空" << std::endl;
			}

			//cv::waitKey(50);
			//线程sleep50ms
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
		}
	}
}

void THandEyeCali2D::on_btn_show_TCP_clicked()
{
	TPoint6D tmp;
	if (m_camera_chishine_handeye->get_TPoint6D(tmp))
	{
		std::cout << tmp.x << "  " << tmp.y << "  " << tmp.z << "  " << tmp.rx << "  " << tmp.ry << "  " << tmp.rz << std::endl;
	}
	else
	{
		std::cout << "TCP获取失败" << std::endl;
	}
}

