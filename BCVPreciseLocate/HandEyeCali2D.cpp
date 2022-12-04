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

	std::cout << "����2D�궨ģʽ..." << std::endl << std::endl;

	//����궨ģʽʱ���camera��¼������
	m_camera_chishine_handeye->capture_count = 0;//���capture��¼
	m_camera_chishine_handeye->board_points.clear();//��մ洢�Ľǵ�
	m_camera_chishine_handeye->TCP_points.clear();//��մ洢��TCP����

	if (m_camera_chishine_handeye->open_camera())
	{
		//����ui״̬
		ui.lineEdit_exposure_time->setEnabled(true);
		ui.lineEdit_gain->setEnabled(true);

		//��ʼ���������
		TCameraPara para = m_camera_chishine_handeye->get_para();
		ui.lineEdit_exposure_time->setText(QString::number(static_cast<int>(para.exposure_time)));
		ui.lineEdit_gain->setText(QString::number(static_cast<int>(para.gain)));

		//�ر����������������IR������궨ģʽ
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
			std::cout << "�л�IR��ʧ�ܣ��������궨����" << std::endl;
		}
	}
	else
	{
		std::cout << "�����ʧ�ܣ�" << std::endl;
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

		//�رս���ʱ����������������
		//if (m_camera_chishine_handeye->start_depth_stream(STREAM_FORMAT_Z16, TCameraChishine::depthCallback_Depth))
		//{
		//	m_camera_chishine_handeye->stream_Depth_flag = true;
		//	//���֪ͨ���̵߳���realtime_display_depth������xinhaocao?
		//}
		//else
		//{
		//	std::cout << "�л����ͼ��ʧ�ܣ��������������������" << std::endl;
		//}
		std::cout << "�궨�����ѹر�" << std::endl << std::endl;

		event->accept();
		emit restart_frmmain_stream();
	}
}

//����������Է�װ��camera�� ��lineedit��Ϊ���������ȥ
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
	//���Ҫ�ɼ�����IRͼ���ؽ��ǵ� Ȼ�������Ų���
	m_camera_chishine_handeye->m_IR_single_is_cali = false;
	if (m_camera_chishine_handeye->capture_frame(TCAPTURE_TYPE::eCAPTURE_SINGLE))
	{
		std::cout << "IRͼ���ȡ�ɹ�" << std::endl;
	}
	else
	{
		std::cout << "IRͼ���ȡʧ��" << std::endl;
	}
}

void THandEyeCali2D::on_btn_capture_clicked()
{
	if (m_camera_chishine_handeye->capture_count >= 5) return;

	//���Ҫ�ɼ�����IRͼ���ؽ��ǵ� Ȼ�������Ų���
	m_camera_chishine_handeye->m_IR_single_is_cali = true;
	if (m_camera_chishine_handeye->capture_frame(TCAPTURE_TYPE::eCAPTURE_SINGLE))
	{
		std::cout << "�궨ͼ���ѻ�ȡ���ؽ��ǵ�ɹ�" << std::endl;
		std::cout << "��ȡͼ " << m_camera_chishine_handeye->capture_count << " ��,����ȡͼ "
			<< 5 - m_camera_chishine_handeye->capture_count << " ��" << std::endl;
	}

	else
	{
		std::cout << "���ȡͼʧ�ܣ�����������" << std::endl;
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
		std::cout << "���۾������ɹ�" << std::endl;

		//���۾������ɹ�����յ������������洢�Ľǵ㡢TCP����ȵļ�¼
		//�����Ҫ�ظ�����Ļ� �����˳����۱궨�����ʱ�������Щ��¼��������������������
		m_camera_chishine_handeye->capture_count = 0;
		m_camera_chishine_handeye->board_points.clear();
		m_camera_chishine_handeye->TCP_points.clear();

		ui.btn_capture->setEnabled(true);
		ui.btn_calculate->setEnabled(false);
	}
	else
	{
		std::cout << "���۾������ʧ��" << std::endl;
	}
}

void THandEyeCali2D::realtime_display_IR()
{
	cv::Mat mat_left;
	cv::Mat mat_right;
	QImage image_left;
	QImage image_right;

	//��⵽���IR�����ڿ���״̬������ѭ����ʾʵʱͼ��
	//�����Ҫ����������Ļ����Խ�stream_IR_flag����Ϊfalse���������������֮���л�IR���ٽ�stream_IR_flag����Ϊtrue��ͬʱ���øú���
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
				std::cout << "���������Ϊ��" << std::endl;
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
				std::cout << "���������Ϊ��" << std::endl;
			}

			//cv::waitKey(50);
			//�߳�sleep50ms
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
		std::cout << "TCP��ȡʧ��" << std::endl;
	}
}

