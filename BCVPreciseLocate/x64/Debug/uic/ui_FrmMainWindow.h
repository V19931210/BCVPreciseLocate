/********************************************************************************
** Form generated from reading UI file 'FrmMainWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FRMMAINWINDOW_H
#define UI_FRMMAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_TMainWindowClass
{
public:
    QAction *ac_calibration2D;
    QAction *ac_calibration3D;
    QAction *ac_calibration_accuracy;
    QAction *ac_register;
    QWidget *centralWidget;
    QPushButton *btn_obj2pcd;
    QGroupBox *gpb_camera_para;
    QLineEdit *lineEdit_depth_min;
    QLineEdit *lineEdit_depth_max;
    QLabel *label;
    QLabel *label_2;
    QLineEdit *lineEdit_exposure_time;
    QLabel *label_3;
    QPushButton *btn_save_para;
    QPushButton *btn_load_para;
    QLineEdit *lineEdit_gain;
    QLabel *label_4;
    QComboBox *cbb_hdr;
    QComboBox *cbb_hdr_level;
    QCheckBox *ckb_auto_exposure;
    QPushButton *btn_edit_roi;
    QLineEdit *lineEdit_hdr_gain_1;
    QLineEdit *lineEdit_hdr_gain_2;
    QLineEdit *lineEdit_hdr_gain_3;
    QLabel *label_5;
    QLineEdit *lineEdit_hdr_exposure_1;
    QLineEdit *lineEdit_hdr_exposure_3;
    QLineEdit *lineEdit_hdr_exposure_2;
    QLabel *label_6;
    QLabel *label_7;
    QLineEdit *lineEdit_hdr_gain_4;
    QLineEdit *lineEdit_hdr_exposure_4;
    QLabel *label_8;
    QPushButton *btn_get_point_cloud_bmp;
    QGroupBox *gbx_live_stream;
    QTabWidget *realtimeWidget;
    QWidget *tab_Depth;
    QLabel *label_Depth;
    QWidget *tab_pointcloud;
    QVTKWidget *vtk_realtime;
    QPushButton *btn_obj2ply;
    QPushButton *btn_open_pc;
    QPushButton *btn_stl2ply;
    QPushButton *btn_restart_depth_stream;
    QPushButton *btn_stl2pcd;
    QPushButton *btn_get_point_cloud_ply;
    QPushButton *btn_test;
    QMenuBar *menuBar;
    QMenu *menu;
    QMenu *menu_2;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *TMainWindowClass)
    {
        if (TMainWindowClass->objectName().isEmpty())
            TMainWindowClass->setObjectName(QString::fromUtf8("TMainWindowClass"));
        TMainWindowClass->resize(908, 579);
        ac_calibration2D = new QAction(TMainWindowClass);
        ac_calibration2D->setObjectName(QString::fromUtf8("ac_calibration2D"));
        ac_calibration3D = new QAction(TMainWindowClass);
        ac_calibration3D->setObjectName(QString::fromUtf8("ac_calibration3D"));
        ac_calibration_accuracy = new QAction(TMainWindowClass);
        ac_calibration_accuracy->setObjectName(QString::fromUtf8("ac_calibration_accuracy"));
        ac_register = new QAction(TMainWindowClass);
        ac_register->setObjectName(QString::fromUtf8("ac_register"));
        centralWidget = new QWidget(TMainWindowClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        btn_obj2pcd = new QPushButton(centralWidget);
        btn_obj2pcd->setObjectName(QString::fromUtf8("btn_obj2pcd"));
        btn_obj2pcd->setEnabled(true);
        btn_obj2pcd->setGeometry(QRect(580, 470, 111, 21));
        gpb_camera_para = new QGroupBox(centralWidget);
        gpb_camera_para->setObjectName(QString::fromUtf8("gpb_camera_para"));
        gpb_camera_para->setGeometry(QRect(0, 10, 221, 411));
        lineEdit_depth_min = new QLineEdit(gpb_camera_para);
        lineEdit_depth_min->setObjectName(QString::fromUtf8("lineEdit_depth_min"));
        lineEdit_depth_min->setEnabled(false);
        lineEdit_depth_min->setGeometry(QRect(100, 20, 51, 20));
        lineEdit_depth_max = new QLineEdit(gpb_camera_para);
        lineEdit_depth_max->setObjectName(QString::fromUtf8("lineEdit_depth_max"));
        lineEdit_depth_max->setEnabled(false);
        lineEdit_depth_max->setGeometry(QRect(160, 20, 51, 20));
        label = new QLabel(gpb_camera_para);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 20, 71, 20));
        label_2 = new QLabel(gpb_camera_para);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(10, 50, 71, 20));
        lineEdit_exposure_time = new QLineEdit(gpb_camera_para);
        lineEdit_exposure_time->setObjectName(QString::fromUtf8("lineEdit_exposure_time"));
        lineEdit_exposure_time->setEnabled(false);
        lineEdit_exposure_time->setGeometry(QRect(100, 50, 111, 20));
        label_3 = new QLabel(gpb_camera_para);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(10, 80, 71, 20));
        btn_save_para = new QPushButton(gpb_camera_para);
        btn_save_para->setObjectName(QString::fromUtf8("btn_save_para"));
        btn_save_para->setEnabled(false);
        btn_save_para->setGeometry(QRect(10, 380, 91, 21));
        btn_load_para = new QPushButton(gpb_camera_para);
        btn_load_para->setObjectName(QString::fromUtf8("btn_load_para"));
        btn_load_para->setEnabled(false);
        btn_load_para->setGeometry(QRect(110, 380, 91, 21));
        lineEdit_gain = new QLineEdit(gpb_camera_para);
        lineEdit_gain->setObjectName(QString::fromUtf8("lineEdit_gain"));
        lineEdit_gain->setEnabled(false);
        lineEdit_gain->setGeometry(QRect(100, 80, 111, 20));
        label_4 = new QLabel(gpb_camera_para);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(10, 140, 201, 20));
        cbb_hdr = new QComboBox(gpb_camera_para);
        cbb_hdr->addItem(QString());
        cbb_hdr->addItem(QString());
        cbb_hdr->addItem(QString());
        cbb_hdr->addItem(QString());
        cbb_hdr->setObjectName(QString::fromUtf8("cbb_hdr"));
        cbb_hdr->setEnabled(false);
        cbb_hdr->setGeometry(QRect(10, 170, 91, 22));
        cbb_hdr_level = new QComboBox(gpb_camera_para);
        cbb_hdr_level->addItem(QString());
        cbb_hdr_level->addItem(QString());
        cbb_hdr_level->addItem(QString());
        cbb_hdr_level->setObjectName(QString::fromUtf8("cbb_hdr_level"));
        cbb_hdr_level->setEnabled(false);
        cbb_hdr_level->setGeometry(QRect(110, 170, 91, 22));
        ckb_auto_exposure = new QCheckBox(gpb_camera_para);
        ckb_auto_exposure->setObjectName(QString::fromUtf8("ckb_auto_exposure"));
        ckb_auto_exposure->setEnabled(false);
        ckb_auto_exposure->setGeometry(QRect(110, 110, 101, 21));
        QFont font;
        font.setKerning(true);
        ckb_auto_exposure->setFont(font);
        ckb_auto_exposure->setIconSize(QSize(16, 16));
        ckb_auto_exposure->setCheckable(true);
        ckb_auto_exposure->setChecked(false);
        ckb_auto_exposure->setAutoRepeatDelay(300);
        btn_edit_roi = new QPushButton(gpb_camera_para);
        btn_edit_roi->setObjectName(QString::fromUtf8("btn_edit_roi"));
        btn_edit_roi->setEnabled(false);
        btn_edit_roi->setGeometry(QRect(10, 110, 91, 21));
        lineEdit_hdr_gain_1 = new QLineEdit(gpb_camera_para);
        lineEdit_hdr_gain_1->setObjectName(QString::fromUtf8("lineEdit_hdr_gain_1"));
        lineEdit_hdr_gain_1->setEnabled(false);
        lineEdit_hdr_gain_1->setGeometry(QRect(120, 200, 81, 20));
        lineEdit_hdr_gain_2 = new QLineEdit(gpb_camera_para);
        lineEdit_hdr_gain_2->setObjectName(QString::fromUtf8("lineEdit_hdr_gain_2"));
        lineEdit_hdr_gain_2->setEnabled(false);
        lineEdit_hdr_gain_2->setGeometry(QRect(120, 230, 81, 20));
        lineEdit_hdr_gain_3 = new QLineEdit(gpb_camera_para);
        lineEdit_hdr_gain_3->setObjectName(QString::fromUtf8("lineEdit_hdr_gain_3"));
        lineEdit_hdr_gain_3->setEnabled(false);
        lineEdit_hdr_gain_3->setGeometry(QRect(120, 260, 81, 20));
        label_5 = new QLabel(gpb_camera_para);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(10, 200, 21, 20));
        lineEdit_hdr_exposure_1 = new QLineEdit(gpb_camera_para);
        lineEdit_hdr_exposure_1->setObjectName(QString::fromUtf8("lineEdit_hdr_exposure_1"));
        lineEdit_hdr_exposure_1->setEnabled(false);
        lineEdit_hdr_exposure_1->setGeometry(QRect(30, 200, 81, 20));
        lineEdit_hdr_exposure_3 = new QLineEdit(gpb_camera_para);
        lineEdit_hdr_exposure_3->setObjectName(QString::fromUtf8("lineEdit_hdr_exposure_3"));
        lineEdit_hdr_exposure_3->setEnabled(false);
        lineEdit_hdr_exposure_3->setGeometry(QRect(30, 260, 81, 20));
        lineEdit_hdr_exposure_2 = new QLineEdit(gpb_camera_para);
        lineEdit_hdr_exposure_2->setObjectName(QString::fromUtf8("lineEdit_hdr_exposure_2"));
        lineEdit_hdr_exposure_2->setEnabled(false);
        lineEdit_hdr_exposure_2->setGeometry(QRect(30, 230, 81, 20));
        label_6 = new QLabel(gpb_camera_para);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(10, 230, 21, 20));
        label_7 = new QLabel(gpb_camera_para);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(10, 260, 21, 20));
        lineEdit_hdr_gain_4 = new QLineEdit(gpb_camera_para);
        lineEdit_hdr_gain_4->setObjectName(QString::fromUtf8("lineEdit_hdr_gain_4"));
        lineEdit_hdr_gain_4->setEnabled(false);
        lineEdit_hdr_gain_4->setGeometry(QRect(120, 290, 81, 20));
        lineEdit_hdr_exposure_4 = new QLineEdit(gpb_camera_para);
        lineEdit_hdr_exposure_4->setObjectName(QString::fromUtf8("lineEdit_hdr_exposure_4"));
        lineEdit_hdr_exposure_4->setEnabled(false);
        lineEdit_hdr_exposure_4->setGeometry(QRect(30, 290, 81, 20));
        label_8 = new QLabel(gpb_camera_para);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(10, 290, 21, 20));
        btn_get_point_cloud_bmp = new QPushButton(centralWidget);
        btn_get_point_cloud_bmp->setObjectName(QString::fromUtf8("btn_get_point_cloud_bmp"));
        btn_get_point_cloud_bmp->setEnabled(false);
        btn_get_point_cloud_bmp->setGeometry(QRect(10, 430, 91, 21));
        gbx_live_stream = new QGroupBox(centralWidget);
        gbx_live_stream->setObjectName(QString::fromUtf8("gbx_live_stream"));
        gbx_live_stream->setEnabled(true);
        gbx_live_stream->setGeometry(QRect(230, 10, 661, 451));
        realtimeWidget = new QTabWidget(gbx_live_stream);
        realtimeWidget->setObjectName(QString::fromUtf8("realtimeWidget"));
        realtimeWidget->setEnabled(true);
        realtimeWidget->setGeometry(QRect(10, 20, 650, 430));
        tab_Depth = new QWidget();
        tab_Depth->setObjectName(QString::fromUtf8("tab_Depth"));
        label_Depth = new QLabel(tab_Depth);
        label_Depth->setObjectName(QString::fromUtf8("label_Depth"));
        label_Depth->setGeometry(QRect(0, 0, 640, 400));
        realtimeWidget->addTab(tab_Depth, QString());
        tab_pointcloud = new QWidget();
        tab_pointcloud->setObjectName(QString::fromUtf8("tab_pointcloud"));
        vtk_realtime = new QVTKWidget(tab_pointcloud);
        vtk_realtime->setObjectName(QString::fromUtf8("vtk_realtime"));
        vtk_realtime->setGeometry(QRect(0, 0, 640, 400));
        realtimeWidget->addTab(tab_pointcloud, QString());
        btn_obj2ply = new QPushButton(centralWidget);
        btn_obj2ply->setObjectName(QString::fromUtf8("btn_obj2ply"));
        btn_obj2ply->setEnabled(true);
        btn_obj2ply->setGeometry(QRect(700, 470, 111, 21));
        btn_open_pc = new QPushButton(centralWidget);
        btn_open_pc->setObjectName(QString::fromUtf8("btn_open_pc"));
        btn_open_pc->setEnabled(true);
        btn_open_pc->setGeometry(QRect(110, 460, 91, 21));
        btn_stl2ply = new QPushButton(centralWidget);
        btn_stl2ply->setObjectName(QString::fromUtf8("btn_stl2ply"));
        btn_stl2ply->setEnabled(true);
        btn_stl2ply->setGeometry(QRect(460, 470, 111, 21));
        btn_restart_depth_stream = new QPushButton(centralWidget);
        btn_restart_depth_stream->setObjectName(QString::fromUtf8("btn_restart_depth_stream"));
        btn_restart_depth_stream->setEnabled(false);
        btn_restart_depth_stream->setGeometry(QRect(110, 490, 91, 21));
        btn_stl2pcd = new QPushButton(centralWidget);
        btn_stl2pcd->setObjectName(QString::fromUtf8("btn_stl2pcd"));
        btn_stl2pcd->setEnabled(true);
        btn_stl2pcd->setGeometry(QRect(340, 470, 111, 21));
        btn_get_point_cloud_ply = new QPushButton(centralWidget);
        btn_get_point_cloud_ply->setObjectName(QString::fromUtf8("btn_get_point_cloud_ply"));
        btn_get_point_cloud_ply->setEnabled(false);
        btn_get_point_cloud_ply->setGeometry(QRect(110, 430, 91, 21));
        btn_test = new QPushButton(centralWidget);
        btn_test->setObjectName(QString::fromUtf8("btn_test"));
        btn_test->setEnabled(true);
        btn_test->setGeometry(QRect(240, 470, 91, 21));
        TMainWindowClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(TMainWindowClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 908, 22));
        menu = new QMenu(menuBar);
        menu->setObjectName(QString::fromUtf8("menu"));
        menu_2 = new QMenu(menuBar);
        menu_2->setObjectName(QString::fromUtf8("menu_2"));
        TMainWindowClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(TMainWindowClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        TMainWindowClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(TMainWindowClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        TMainWindowClass->setStatusBar(statusBar);

        menuBar->addAction(menu->menuAction());
        menuBar->addAction(menu_2->menuAction());
        menu->addAction(ac_calibration2D);
        menu->addAction(ac_calibration3D);
        menu->addAction(ac_calibration_accuracy);
        menu_2->addAction(ac_register);

        retranslateUi(TMainWindowClass);

        realtimeWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(TMainWindowClass);
    } // setupUi

    void retranslateUi(QMainWindow *TMainWindowClass)
    {
        TMainWindowClass->setWindowTitle(QCoreApplication::translate("TMainWindowClass", "TMainWindow", nullptr));
        ac_calibration2D->setText(QCoreApplication::translate("TMainWindowClass", "2D\346\211\213\347\234\274\346\240\207\345\256\232", nullptr));
        ac_calibration3D->setText(QCoreApplication::translate("TMainWindowClass", "3D\346\211\213\347\234\274\346\240\207\345\256\232", nullptr));
        ac_calibration_accuracy->setText(QCoreApplication::translate("TMainWindowClass", "\346\240\207\345\256\232\347\262\276\345\272\246\351\252\214\350\257\201", nullptr));
        ac_register->setText(QCoreApplication::translate("TMainWindowClass", "\347\262\276\351\205\215\345\207\206", nullptr));
        btn_obj2pcd->setText(QCoreApplication::translate("TMainWindowClass", "trans obj to pcd", nullptr));
        gpb_camera_para->setTitle(QCoreApplication::translate("TMainWindowClass", "\347\233\270\346\234\272\345\217\202\346\225\260\350\256\276\347\275\256", nullptr));
        lineEdit_depth_min->setText(QCoreApplication::translate("TMainWindowClass", "200", nullptr));
        lineEdit_depth_max->setText(QCoreApplication::translate("TMainWindowClass", "600", nullptr));
        label->setText(QCoreApplication::translate("TMainWindowClass", "\346\267\261\345\272\246\350\214\203\345\233\264(mm)", nullptr));
        label_2->setText(QCoreApplication::translate("TMainWindowClass", "\346\233\235\345\205\211\346\227\266\351\227\264(us)", nullptr));
        lineEdit_exposure_time->setText(QCoreApplication::translate("TMainWindowClass", "20000", nullptr));
        label_3->setText(QCoreApplication::translate("TMainWindowClass", "\345\242\236\347\233\212", nullptr));
        btn_save_para->setText(QCoreApplication::translate("TMainWindowClass", "\344\277\235\345\255\230\347\233\270\346\234\272\345\217\202\346\225\260", nullptr));
        btn_load_para->setText(QCoreApplication::translate("TMainWindowClass", "\345\212\240\350\275\275\347\233\270\346\234\272\345\217\202\346\225\260", nullptr));
        lineEdit_gain->setText(QCoreApplication::translate("TMainWindowClass", "2", nullptr));
        label_4->setText(QCoreApplication::translate("TMainWindowClass", "HDR\346\250\241\345\274\217", nullptr));
        cbb_hdr->setItemText(0, QCoreApplication::translate("TMainWindowClass", "\345\205\263\351\227\255", nullptr));
        cbb_hdr->setItemText(1, QCoreApplication::translate("TMainWindowClass", "\351\253\230\345\205\211", nullptr));
        cbb_hdr->setItemText(2, QCoreApplication::translate("TMainWindowClass", "\351\273\221\347\231\275", nullptr));
        cbb_hdr->setItemText(3, QCoreApplication::translate("TMainWindowClass", "\345\244\215\345\220\210", nullptr));

        cbb_hdr_level->setItemText(0, QCoreApplication::translate("TMainWindowClass", "2", nullptr));
        cbb_hdr_level->setItemText(1, QCoreApplication::translate("TMainWindowClass", "3", nullptr));
        cbb_hdr_level->setItemText(2, QCoreApplication::translate("TMainWindowClass", "4", nullptr));

        ckb_auto_exposure->setText(QCoreApplication::translate("TMainWindowClass", "\345\274\200\345\220\257\350\207\252\345\212\250\346\233\235\345\205\211", nullptr));
        btn_edit_roi->setText(QCoreApplication::translate("TMainWindowClass", "\347\274\226\350\276\221ROI", nullptr));
        lineEdit_hdr_gain_1->setText(QCoreApplication::translate("TMainWindowClass", "1", nullptr));
        lineEdit_hdr_gain_2->setText(QCoreApplication::translate("TMainWindowClass", "2", nullptr));
        lineEdit_hdr_gain_3->setText(QCoreApplication::translate("TMainWindowClass", "2", nullptr));
        label_5->setText(QCoreApplication::translate("TMainWindowClass", "1", nullptr));
        lineEdit_hdr_exposure_1->setText(QCoreApplication::translate("TMainWindowClass", "15000", nullptr));
        lineEdit_hdr_exposure_3->setText(QCoreApplication::translate("TMainWindowClass", "30000", nullptr));
        lineEdit_hdr_exposure_2->setText(QCoreApplication::translate("TMainWindowClass", "20000", nullptr));
        label_6->setText(QCoreApplication::translate("TMainWindowClass", "2", nullptr));
        label_7->setText(QCoreApplication::translate("TMainWindowClass", "3", nullptr));
        lineEdit_hdr_gain_4->setText(QCoreApplication::translate("TMainWindowClass", "4", nullptr));
        lineEdit_hdr_exposure_4->setText(QCoreApplication::translate("TMainWindowClass", "30000", nullptr));
        label_8->setText(QCoreApplication::translate("TMainWindowClass", "4", nullptr));
        btn_get_point_cloud_bmp->setText(QCoreApplication::translate("TMainWindowClass", "\350\216\267\345\217\226\347\202\271\344\272\221.bmp", nullptr));
        gbx_live_stream->setTitle(QCoreApplication::translate("TMainWindowClass", "\347\233\270\346\234\272\345\256\236\346\227\266\345\233\276\345\203\217", nullptr));
        label_Depth->setText(QString());
        realtimeWidget->setTabText(realtimeWidget->indexOf(tab_Depth), QCoreApplication::translate("TMainWindowClass", "\346\267\261\345\272\246\346\265\201", nullptr));
        realtimeWidget->setTabText(realtimeWidget->indexOf(tab_pointcloud), QCoreApplication::translate("TMainWindowClass", "\347\202\271\344\272\221\346\265\201", nullptr));
        btn_obj2ply->setText(QCoreApplication::translate("TMainWindowClass", "trans obj to ply", nullptr));
        btn_open_pc->setText(QCoreApplication::translate("TMainWindowClass", "\346\211\223\345\274\200\347\202\271\344\272\221\346\226\207\344\273\266", nullptr));
        btn_stl2ply->setText(QCoreApplication::translate("TMainWindowClass", "trans stl to ply", nullptr));
        btn_restart_depth_stream->setText(QCoreApplication::translate("TMainWindowClass", "\351\207\215\345\220\257\346\267\261\345\272\246\346\265\201", nullptr));
        btn_stl2pcd->setText(QCoreApplication::translate("TMainWindowClass", "trans stl to pcd", nullptr));
        btn_get_point_cloud_ply->setText(QCoreApplication::translate("TMainWindowClass", "\350\216\267\345\217\226\347\202\271\344\272\221.ply", nullptr));
        btn_test->setText(QCoreApplication::translate("TMainWindowClass", "test", nullptr));
        menu->setTitle(QCoreApplication::translate("TMainWindowClass", "\346\240\207\345\256\232", nullptr));
        menu_2->setTitle(QCoreApplication::translate("TMainWindowClass", "\351\205\215\345\207\206", nullptr));
    } // retranslateUi

};

namespace Ui {
    class TMainWindowClass: public Ui_TMainWindowClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FRMMAINWINDOW_H
