#include "bcvWeldNCShareMM.h"

#include <iostream>


#ifndef CV_PI
#define CV_PI 3.1415926
#endif

TShareMemCypWeld::TShareMemCypWeld()
{

}

void TShareMemCypWeld::openShareMM()
{
	fileHandle = OpenFileMapping(FILE_MAP_ALL_ACCESS, false, L"CypWeld_NcContext_SharedMem");
	if (fileHandle != nullptr)
	{
		//bcv::APPshowMessage("已成功打开内存映射文件");
	}
	else
	{
		mOpenMMSuccess = false;
		//bcv::APPshowMessage("打开内存映射文件失败，创建共享内存");
		std::cout << "打开内存映射文件失败，创建共享内存" << std::endl;
		fileHandle = CreateFileMapping(INVALID_HANDLE_VALUE, nullptr,
			PAGE_READWRITE, 0, sizeof(TNcContextRec),
			L"CypWeld_NcContext_SharedMem");
	}

	//建立映射关系
	PShareMem = (TNcContextRec*)MapViewOfFile(fileHandle, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(TNcContextRec));

	if (PShareMem != nullptr)
	{
		//bcv::APPshowMessage("已成功建立映射关系");
	}
	else
	{
		mOpenMMSuccess = false;
		//bcv::APPshowMessage("建立映射关系失败", 1);
		std::cout << "建立映射关系失败" << std::endl;
		return;
	}

	mOpenMMSuccess = true;
}


bool TShareMemCypWeld::getRobotTcpPos(TPoint6D& pos, int angleType)
{
	if (!mOpenMMSuccess) return false;
	pos.x = PShareMem->NcCoor.Robot.WcsPos.x;
	pos.y = PShareMem->NcCoor.Robot.WcsPos.y;
	pos.z = PShareMem->NcCoor.Robot.WcsPos.z;
	if (angleType == 0)
	{
		pos.rx = PShareMem->NcCoor.Robot.WcsPos.rx * 180.0 / CV_PI;
		pos.ry = PShareMem->NcCoor.Robot.WcsPos.ry * 180.0 / CV_PI;
		pos.rz = PShareMem->NcCoor.Robot.WcsPos.rz * 180.0 / CV_PI;
	}
	else
	{
		pos.rx = PShareMem->NcCoor.Robot.WcsPos.rx;
		pos.ry = PShareMem->NcCoor.Robot.WcsPos.ry;
		pos.rz = PShareMem->NcCoor.Robot.WcsPos.rz;
	}
	return true;
}

bool TShareMemCypWeld::getTcpPara(TPoint6D& pos, int angleType)
{
	if (!mOpenMMSuccess) return false;
	pos.x = PShareMem->TcpParam.x;
	pos.y = PShareMem->TcpParam.y;
	pos.z = PShareMem->TcpParam.z;
	if (angleType == 0)
	{
		pos.rx = PShareMem->TcpParam.rx * 180.0 / CV_PI;
		pos.ry = PShareMem->TcpParam.ry * 180.0 / CV_PI;
		pos.rz = PShareMem->TcpParam.rz * 180.0 / CV_PI;
	}
	else
	{
		pos.rx = PShareMem->TcpParam.rx;
		pos.ry = PShareMem->TcpParam.ry;
		pos.rz = PShareMem->TcpParam.rz;
	}

	return true;
}
