#pragma once

#include <QWidget>
#include "ui_FrmRegister.h"

#include "../CameraChishine/CameraChishine.h"

class TRegister : public QWidget
{
	Q_OBJECT

public:
	TRegister(TCameraChishine* a_camera, QWidget* parent = Q_NULLPTR);
	~TRegister();

private:
	Ui::TRegister ui;
	TCameraChishine* m_camera_chishine_register;
};