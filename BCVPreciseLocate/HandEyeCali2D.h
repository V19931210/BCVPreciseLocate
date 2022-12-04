#pragma once

#include <QWidget>
#include "ui_FrmHandEyeCali2D.h"

#include "../CameraChishine/CameraChishine.h"

#include "opencv.hpp"

class THandEyeCali2D : public QWidget
{
	Q_OBJECT

public:
	THandEyeCali2D(TCameraChishine* a_camera, QWidget* parent = Q_NULLPTR);
	~THandEyeCali2D() = default;
	void realtime_display_IR(); //��ʾ�����

signals:
	void restart_frmmain_stream();

private slots:
	void set_exposure(); //�����ع�
	void set_gain(); //��������
	void on_btn_get_IR_clicked(); //���ղ��ؽ��ǵ�
	void on_btn_capture_clicked(); //���ղ��ؽ��ǵ�
	void on_btn_calculate_clicked(); //�������۾���
	void on_btn_show_TCP_clicked();

public:
	void closeEvent(QCloseEvent* event);

private:
	Ui::THandEyeCali2D ui;
	TCameraChishine* m_camera_chishine_handeye;

};
