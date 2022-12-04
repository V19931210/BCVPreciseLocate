/********************************************************************************
** Form generated from reading UI file 'FrmHandEyeCali3D.ui'
**
** Created by: Qt User Interface Compiler version 5.15.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FRMHANDEYECALI3D_H
#define UI_FRMHANDEYECALI3D_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_THandEyeCali3D
{
public:
    QVTKWidget *vtk_cad;
    QLabel *label_3;
    QLabel *label_6;
    QVTKWidget *vtk_ret;
    QPushButton *btn_roi_cut;
    QPushButton *btn_get_pointcloud;
    QLabel *label_4;
    QVTKWidget *vtk_cam;
    QVTKWidget *vtk_realtime;
    QLabel *label;
    QLabel *label_2;
    QPushButton *btn_registration;
    QPushButton *btn_cal_hadeye_mat;
    QPushButton *btn_load_CAD;
    QLabel *label_5;
    QPushButton *btn_save_cali_data;
    QPushButton *btn_test;

    void setupUi(QWidget *THandEyeCali3D)
    {
        if (THandEyeCali3D->objectName().isEmpty())
            THandEyeCali3D->setObjectName(QString::fromUtf8("THandEyeCali3D"));
        THandEyeCali3D->resize(924, 577);
        vtk_cad = new QVTKWidget(THandEyeCali3D);
        vtk_cad->setObjectName(QString::fromUtf8("vtk_cad"));
        vtk_cad->setGeometry(QRect(160, 10, 371, 271));
        label_3 = new QLabel(THandEyeCali3D);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(10, 70, 41, 16));
        label_6 = new QLabel(THandEyeCali3D);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(10, 160, 41, 16));
        vtk_ret = new QVTKWidget(THandEyeCali3D);
        vtk_ret->setObjectName(QString::fromUtf8("vtk_ret"));
        vtk_ret->setGeometry(QRect(540, 290, 371, 271));
        btn_roi_cut = new QPushButton(THandEyeCali3D);
        btn_roi_cut->setObjectName(QString::fromUtf8("btn_roi_cut"));
        btn_roi_cut->setEnabled(false);
        btn_roi_cut->setGeometry(QRect(50, 70, 91, 23));
        btn_get_pointcloud = new QPushButton(THandEyeCali3D);
        btn_get_pointcloud->setObjectName(QString::fromUtf8("btn_get_pointcloud"));
        btn_get_pointcloud->setEnabled(false);
        btn_get_pointcloud->setGeometry(QRect(50, 40, 91, 23));
        label_4 = new QLabel(THandEyeCali3D);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(10, 100, 41, 16));
        vtk_cam = new QVTKWidget(THandEyeCali3D);
        vtk_cam->setObjectName(QString::fromUtf8("vtk_cam"));
        vtk_cam->setGeometry(QRect(540, 10, 371, 271));
        vtk_realtime = new QVTKWidget(THandEyeCali3D);
        vtk_realtime->setObjectName(QString::fromUtf8("vtk_realtime"));
        vtk_realtime->setGeometry(QRect(160, 290, 371, 271));
        label = new QLabel(THandEyeCali3D);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 10, 41, 16));
        label_2 = new QLabel(THandEyeCali3D);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(10, 40, 41, 16));
        btn_registration = new QPushButton(THandEyeCali3D);
        btn_registration->setObjectName(QString::fromUtf8("btn_registration"));
        btn_registration->setEnabled(false);
        btn_registration->setGeometry(QRect(50, 100, 91, 23));
        btn_cal_hadeye_mat = new QPushButton(THandEyeCali3D);
        btn_cal_hadeye_mat->setObjectName(QString::fromUtf8("btn_cal_hadeye_mat"));
        btn_cal_hadeye_mat->setEnabled(false);
        btn_cal_hadeye_mat->setGeometry(QRect(50, 160, 91, 23));
        btn_load_CAD = new QPushButton(THandEyeCali3D);
        btn_load_CAD->setObjectName(QString::fromUtf8("btn_load_CAD"));
        btn_load_CAD->setEnabled(false);
        btn_load_CAD->setGeometry(QRect(50, 10, 91, 23));
        label_5 = new QLabel(THandEyeCali3D);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(10, 130, 41, 16));
        btn_save_cali_data = new QPushButton(THandEyeCali3D);
        btn_save_cali_data->setObjectName(QString::fromUtf8("btn_save_cali_data"));
        btn_save_cali_data->setEnabled(false);
        btn_save_cali_data->setGeometry(QRect(50, 130, 91, 23));
        btn_test = new QPushButton(THandEyeCali3D);
        btn_test->setObjectName(QString::fromUtf8("btn_test"));
        btn_test->setGeometry(QRect(40, 480, 91, 23));

        retranslateUi(THandEyeCali3D);

        QMetaObject::connectSlotsByName(THandEyeCali3D);
    } // setupUi

    void retranslateUi(QWidget *THandEyeCali3D)
    {
        THandEyeCali3D->setWindowTitle(QCoreApplication::translate("THandEyeCali3D", "THandEyeCali3D", nullptr));
        label_3->setText(QCoreApplication::translate("THandEyeCali3D", "step3:", nullptr));
        label_6->setText(QCoreApplication::translate("THandEyeCali3D", "step6:", nullptr));
        btn_roi_cut->setText(QCoreApplication::translate("THandEyeCali3D", "ROI\350\243\201\345\211\252", nullptr));
        btn_get_pointcloud->setText(QCoreApplication::translate("THandEyeCali3D", "\350\216\267\345\217\226\347\202\271\344\272\221", nullptr));
        label_4->setText(QCoreApplication::translate("THandEyeCali3D", "step4:", nullptr));
        label->setText(QCoreApplication::translate("THandEyeCali3D", "step1:", nullptr));
        label_2->setText(QCoreApplication::translate("THandEyeCali3D", "step2:", nullptr));
        btn_registration->setText(QCoreApplication::translate("THandEyeCali3D", "\347\202\271\344\272\221\351\205\215\345\207\206", nullptr));
        btn_cal_hadeye_mat->setText(QCoreApplication::translate("THandEyeCali3D", "\350\256\241\347\256\227\346\211\213\347\234\274\347\237\251\351\230\265", nullptr));
        btn_load_CAD->setText(QCoreApplication::translate("THandEyeCali3D", "\345\212\240\350\275\275CAD\347\202\271\344\272\221", nullptr));
        label_5->setText(QCoreApplication::translate("THandEyeCali3D", "step5:", nullptr));
        btn_save_cali_data->setText(QCoreApplication::translate("THandEyeCali3D", "\344\277\235\345\255\230\346\240\207\345\256\232\346\225\260\346\215\256", nullptr));
        btn_test->setText(QCoreApplication::translate("THandEyeCali3D", "test", nullptr));
    } // retranslateUi

};

namespace Ui {
    class THandEyeCali3D: public Ui_THandEyeCali3D {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FRMHANDEYECALI3D_H
