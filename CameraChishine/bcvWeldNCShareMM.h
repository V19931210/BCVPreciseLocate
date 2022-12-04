//////////////////////////////////////////////////////////////////////////////
// 
//                      ��CypWeld�Ĺ����ڴ�
//���   http://docs.fscut.com/pages/viewpage.action?pageId=104817261
//  ���ļ���BCVWeldTrack�� BCVRoughLocate ����
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

//��CypWeld��ȡ nc��״̬
// cypWeld�����ڴ��еĽǶȶ��ǻ�����
struct TNcContextRec
{
    int Version;                                  // �����ڴ�汾����ǰ�汾Ϊ1
    int NcState;                                  // NCϵͳ״̬
    TCoordinateContext NcCoor;                    // NCϵͳ����״̬
    TPoint6D TcpParam;                            // TCP�궨����
    TRect3D VisionField;                          // Ѱ������Ұ����
    //BYTE NcReserved[4096];                        // Ԥ����NCϵͳ�Ĵ�������
    TCadContextRec CadContext;                    // CAD״̬
    //BYTE CadReserved[4096];                       // Ԥ����CAD�Ĵ�������
};



class BCVWELDNCSHAREMM_EXPORT TShareMemCypWeld
{
public:
    TShareMemCypWeld();
    void openShareMM();

    // tcp������
    //angleType�� 0��ʾ�Ƕȣ�1��ʾ����
    bool getRobotTcpPos(TPoint6D& pos, int angleType = 0);

    //��ȡtcp����
    //angleType�� 0��ʾ�Ƕȣ�1��ʾ����
    bool getTcpPara(TPoint6D& pos, int angleType = 0);
public:
    bool mOpenMMSuccess = false;
    TNcContextRec* PShareMem = NULL;//ָ�����ڴ��ָ��
private:
    HANDLE fileHandle;//�ڴ�ӳ���ļ����
};
