//////////////////////////////////////////////////////////////////////////////
// 
//                      与CypWeld的共享内存
//详见   http://docs.fscut.com/pages/viewpage.action?pageId=104817261
//  此文件，BCVWeldTrack和 BCVRoughLocate 共用
//---------------------------------------------------------------------------
//   Created      by xuchao @ 2021.11.11
//   Last updated by xuchao @ 2021.11.11
//---------------------------------------------------------------------------
//
//                                    
//
////////////////////////////////////////////////////////////////////////////////

#pragma once

//#include"opencv.h"
//#include"QFunctions.h"
//#include"bcvfunctions.h"

#include <Windows.h>

#ifndef BCVWELDNCSHAREMM_EXPORT
#define BCVWELDNCSHAREMM_EXPORT __declspec(dllexport)
#endif

struct TPoint6D
{
    TPoint6D()
    {
        x = 0;
        y = 0;
        z = 0;
        rx = 0;
        ry = 0;
        rz = 0;
    }

    TPoint6D(double x, double y, double z, double rx, double ry, double rz) :x(x), y(y), z(z), rx(rx), ry(ry), rz(rz)
    {

    }

    TPoint6D& operator=(const TPoint6D& m)
    {
        x = m.x;
        y = m.y;
        z = m.z;
        rx = m.rx;
        ry = m.ry;
        rz = m.rz;
        return *this;
    }
    void zero()
    {
        x = 0;
        y = 0;
        z = 0;
        rx = 0;
        ry = 0;
        rz = 0;
    }

    double x;
    double y;
    double z;
    double rx;
    double ry;
    double rz;
};


struct TPoint3D
{
    double x;
    double y;
    double z;
};

struct TRect3D
{
    TPoint3D TopLeft;
    TPoint3D TopRight;
    TPoint3D BottomLeft;
    TPoint3D BottomRight;
};

struct TRobotContext
{
    TPoint6D WcsPos;
    TPoint6D McsPos;
};

struct TPositionerContext
{
    double McsPos[8];
};

struct TCoordinateContext
{
    TRobotContext Robot;
    TPositionerContext Positioner;
    TPoint3D Gantry;
};

struct TCadContextRec
{
    int CadState;
    TCoordinateContext CadCoor;
};

//从CypWeld读取 nc的状态
// cypWeld共享内存中的角度都是弧度制
struct TNcContextRec
{
    int Version;                                  // 共享内存版本，当前版本为1
    int NcState;                                  // NC系统状态
    TCoordinateContext NcCoor;                    // NC系统坐标状态
    TPoint6D TcpParam;                            // TCP标定参数
    TRect3D VisionField;                          // 寻缝器视野参数
    //BYTE NcReserved[4096];                        // 预留给NC系统的储存区段
    TCadContextRec CadContext;                    // CAD状态
    //BYTE CadReserved[4096];                       // 预留给CAD的储存区段
};



class BCVWELDNCSHAREMM_EXPORT TShareMemCypWeld
{
public:
    TShareMemCypWeld();
    void openShareMM();

    // tcp的坐标
    //angleType； 0表示角度，1表示弧度
    bool getRobotTcpPos(TPoint6D& pos, int angleType = 0);

    //获取tcp参数
    //angleType； 0表示角度，1表示弧度
    bool getTcpPara(TPoint6D& pos, int angleType = 0);
public:
    bool mOpenMMSuccess = false;
    TNcContextRec* PShareMem = NULL;//指向共享内存的指针
private:
    HANDLE fileHandle;//内存映射文件句柄
};
