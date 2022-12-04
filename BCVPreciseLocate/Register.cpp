#include "Register.h"

TRegister::TRegister(TCameraChishine* a_camera, QWidget* parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	m_camera_chishine_register = a_camera;
}

TRegister::~TRegister()
{
}
