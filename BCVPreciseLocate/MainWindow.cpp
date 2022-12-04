#include "MainWindow.h"

//C库
#include <iostream>

//C++库
#include <Thread>

//QT库
#include <QMessageBox>
#include <QValidator>
#include <QFile>
#include <QFileDialog>
#include <QJsonObject>
#include <QJsonParseError>

//opencv
#include <opencv.hpp>

//pcl库
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
#include <pcl/filters/filter.h>
#include <pcl/common/impl/io.hpp>

//bcvlib
#include <BCQipc.h>
//#include <QFunctions.h>
//#include <bcvfunctions.h>

TMainWindow::TMainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    this->setFixedSize(this->width(), this->height());
    setWindowTitle("BCVPreciseLocate");
    setWindowIcon(QIcon(QStringLiteral("title.ico")));
    
    m_camera_chishine = TCameraChishine::get_TCameraChishine();

    std::cout << "正在连接相机..." << std::endl << std::endl;
    if (m_camera_chishine->open_camera())
    {
        m_camera_chishine->show_information();
        m_camera_chishine->get_stream_info();

        //ui状态更改
        ui.lineEdit_depth_min->setEnabled(true);
        ui.lineEdit_depth_max->setEnabled(true);
        ui.lineEdit_exposure_time->setEnabled(true);
        ui.lineEdit_gain->setEnabled(true);
        ui.cbb_hdr->setEnabled(true);
        ui.btn_edit_roi->setEnabled(true);//未实现
        ui.ckb_auto_exposure->setEnabled(true);
        ui.btn_save_para->setEnabled(true);
        ui.btn_load_para->setEnabled(true);
        ui.btn_get_point_cloud_bmp->setEnabled(true);
        ui.btn_get_point_cloud_ply->setEnabled(true);
        ui.btn_restart_depth_stream->setEnabled(true);

        //初始化para
        set_para_ui(m_camera_chishine->get_para());
        PropertyExtension value;
        if (m_camera_chishine->get_hdr_model(value))
            set_hdr_model_ui(value);
        if (value.hdrMode != HDR_MODE_OFF)
        {
            if (m_camera_chishine->get_hdr_para(value))
                set_hdr_para_ui(value);
        }

        //开启深度流
        m_camera_chishine->stream_IR_flag = false;
        m_camera_chishine->stream_Depth_flag = false;
        ui.realtimeWidget->setCurrentIndex(0);
        if (m_camera_chishine->start_depth_stream(STREAM_FORMAT_Z16, TCameraChishine::depthCallback_Depth))
        {
            m_camera_chishine->stream_Depth_flag = true;
            std::thread thread1(&TMainWindow::realtime_display_Depth, this);
            thread1.detach();
        }
        else
        {
            std::cout << "开启深度图流失败，请点击主界面重启深度流" << std::endl;
        }

    }

    ui.lineEdit_depth_min->setValidator(new QIntValidator(0, 5000, this));
    ui.lineEdit_depth_max->setValidator(new QIntValidator(0, 5000, this));
    ui.lineEdit_exposure_time->setValidator(new QIntValidator(0, 60000, this));
    ui.lineEdit_gain->setValidator(new QIntValidator(1, 15, this));
    init_vtk_realtime();
    init_viewer();
    connect_assemble();
}

TMainWindow::~TMainWindow()
{
    m_camera_chishine->stream_Depth_flag = false;
    m_camera_chishine->stream_IR_flag = false;
    m_camera_chishine->close_camera();
}

void TMainWindow::connect_assemble()
{
    connect(ui.lineEdit_depth_min, &QLineEdit::returnPressed, this, &TMainWindow::set_depth_range);
    connect(ui.lineEdit_depth_max, &QLineEdit::returnPressed, this, &TMainWindow::set_depth_range);
    connect(ui.lineEdit_exposure_time, &QLineEdit::returnPressed, this, &TMainWindow::set_exposure);
    connect(ui.lineEdit_gain, &QLineEdit::returnPressed, this, &TMainWindow::set_gain);
    connect(ui.ckb_auto_exposure, &QCheckBox::stateChanged, this, &TMainWindow::set_auto_exposure);
    connect(ui.cbb_hdr, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &TMainWindow::set_HDR_model);
    connect(ui.cbb_hdr_level, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &TMainWindow::set_HDR_level);
    connect(ui.lineEdit_hdr_exposure_1, &QLineEdit::returnPressed, this, &TMainWindow::set_HDR_level1_exposure);
    connect(ui.lineEdit_hdr_exposure_2, &QLineEdit::returnPressed, this, &TMainWindow::set_HDR_level2_exposure);
    connect(ui.lineEdit_hdr_exposure_3, &QLineEdit::returnPressed, this, &TMainWindow::set_HDR_level3_exposure);
    connect(ui.lineEdit_hdr_exposure_4, &QLineEdit::returnPressed, this, &TMainWindow::set_HDR_level4_exposure);
    connect(ui.lineEdit_hdr_gain_1, &QLineEdit::returnPressed, this, &TMainWindow::set_HDR_level1_gain);
    connect(ui.lineEdit_hdr_gain_2, &QLineEdit::returnPressed, this, &TMainWindow::set_HDR_level2_gain);
    connect(ui.lineEdit_hdr_gain_3, &QLineEdit::returnPressed, this, &TMainWindow::set_HDR_level3_gain);
    connect(ui.lineEdit_hdr_gain_4, &QLineEdit::returnPressed, this, &TMainWindow::set_HDR_level4_gain);
    connect(ui.ac_calibration2D, &QAction::triggered, this, &TMainWindow::open_hand_eye_2D);
    connect(ui.ac_calibration3D, &QAction::triggered, this, &TMainWindow::open_hand_eye_3D);
    connect(ui.ac_register, &QAction::triggered, this, &TMainWindow::open_registration);
    connect(ui.realtimeWidget, &QTabWidget::currentChanged, this, &TMainWindow::realtime_stream_changed);
}

void TMainWindow::set_para_ui(TCameraPara para)
{
    ui.lineEdit_depth_min->setText(QString::number(static_cast<int>(para.depth_min)));
    ui.lineEdit_depth_max->setText(QString::number(static_cast<int>(para.depth_max)));
    ui.lineEdit_exposure_time->setText(QString::number(static_cast<int>(para.exposure_time)));
    ui.lineEdit_gain->setText(QString::number(static_cast<int>(para.gain)));
    ui.ckb_auto_exposure->setCheckState(para.auto_exposure == 0 ? Qt::Unchecked : Qt::Checked);
}

void TMainWindow::set_para_camera(TCameraPara para)
{
    m_camera_chishine->set_para_camera(para);
}

void TMainWindow::set_hdr_model_ui(PropertyExtension value)
{
    HDR_MODE model = value.hdrMode;

    switch (model)
    {
    case HDR_MODE_OFF:
    {
        ui.cbb_hdr->setCurrentIndex(0);
        ui.cbb_hdr_level->setEnabled(false);
        ui.lineEdit_hdr_exposure_1->setEnabled(false);
        ui.lineEdit_hdr_exposure_2->setEnabled(false);
        ui.lineEdit_hdr_exposure_3->setEnabled(false);
        ui.lineEdit_hdr_exposure_4->setEnabled(false);
        ui.lineEdit_hdr_gain_1->setEnabled(false);
        ui.lineEdit_hdr_gain_2->setEnabled(false);
        ui.lineEdit_hdr_gain_3->setEnabled(false);
        ui.lineEdit_hdr_gain_4->setEnabled(false);

        break;
    }
    case HDR_MODE_HIGH_RELECT:
    {
        ui.cbb_hdr->setCurrentIndex(1);
        ui.cbb_hdr_level->setEnabled(true);

        break;
    }
    case HDR_MODE_LOW_RELECT:
    {
        ui.cbb_hdr->setCurrentIndex(2);
        ui.cbb_hdr_level->setEnabled(true);

        break;
    }
    case HDR_MODE_ALL_RELECT:
    {
        ui.cbb_hdr->setCurrentIndex(3);
        ui.cbb_hdr_level->setEnabled(true);

        break;
    }
    default:
    {
        break;
    }
    }
}

void TMainWindow::set_hdr_para_ui(PropertyExtension value)
{

    if (value.hdrExposureSetting.count < 2)
        return;
    int level = (int)value.hdrExposureSetting.count;
    ui.cbb_hdr_level->setCurrentIndex(level - 2);

    for (int i = 2; i <= level; i++)
    {
        switch (i)
        {
        case 2:
        {
            ui.lineEdit_hdr_exposure_1->setEnabled(true);
            ui.lineEdit_hdr_exposure_2->setEnabled(true);
            ui.lineEdit_hdr_gain_1->setEnabled(true);
            ui.lineEdit_hdr_gain_2->setEnabled(true);
            ui.lineEdit_hdr_exposure_1->setText(QString::number((int)value.hdrExposureSetting.param[0].exposure));
            ui.lineEdit_hdr_gain_1->setText(QString::number((int)value.hdrExposureSetting.param[0].gain));
            ui.lineEdit_hdr_exposure_2->setText(QString::number((int)value.hdrExposureSetting.param[1].exposure));
            ui.lineEdit_hdr_gain_2->setText(QString::number((int)value.hdrExposureSetting.param[1].gain));

            break;
        }
        case 3:
        {
            ui.lineEdit_hdr_exposure_3->setEnabled(true);
            ui.lineEdit_hdr_gain_3->setEnabled(true);
            ui.lineEdit_hdr_exposure_3->setText(QString::number((int)value.hdrExposureSetting.param[2].exposure));
            ui.lineEdit_hdr_gain_3->setText(QString::number((int)value.hdrExposureSetting.param[2].gain));

            break;
        }
        case 4:
        {
            ui.lineEdit_hdr_exposure_4->setEnabled(true);
            ui.lineEdit_hdr_gain_4->setEnabled(true);
            ui.lineEdit_hdr_exposure_4->setText(QString::number((int)value.hdrExposureSetting.param[3].exposure));
            ui.lineEdit_hdr_gain_4->setText(QString::number((int)value.hdrExposureSetting.param[3].gain));

            break;
        }
        default:
        {
            break;
        }
        }
    }//end for
}

void TMainWindow::open_hand_eye_2D()
{
    m_camera_chishine->stream_Depth_flag = false;//保险起见在切换界面之前就将流的flag设置为false
    m_widgrt_handeye = new THandEyeCali2D(m_camera_chishine);
    connect(m_widgrt_handeye, &THandEyeCali2D::restart_frmmain_stream, this, &TMainWindow::on_btn_restart_depth_stream_clicked);
    m_widgrt_handeye->setWindowModality(Qt::ApplicationModal);
    m_widgrt_handeye->show();
}

void TMainWindow::open_hand_eye_3D()
{
    //m_camera_chishine->stream_Depth_flag = false;//保险起见在切换界面之前就将流的flag设置为false
    if (!m_camera_chishine->m_depth_single_is_ply)
    {
        m_camera_chishine->m_depth_stream_is_ply = true;
        std::thread thread2(&TMainWindow::realtime_display_pointcloud, this);
        thread2.detach();
        std::cout << "已切换为点云流" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    m_widgrt_handeye_3D = new THandEyeCali3D(m_camera_chishine);
    m_widgrt_handeye_3D->setWindowModality(Qt::ApplicationModal);
    m_widgrt_handeye_3D->show();
}

void TMainWindow::open_registration()
{
    m_widgrt_regiostration = new TRegister(m_camera_chishine);
    m_widgrt_regiostration->setWindowModality(Qt::ApplicationModal);
    m_widgrt_regiostration->show();
}

void TMainWindow::set_hdr_camera(PropertyExtension value)
{
    m_camera_chishine->set_hdr_camera(value);
}

void TMainWindow::realtime_display_Depth()
{
    //m_camera_chishine->m_depth_stream_is_ply = false;
    cv::Mat mat;
    QImage image;

    //检测到深度流处于开启状态即进入循环显示实时图像
    while (m_camera_chishine->stream_Depth_flag)
    {
        if (m_camera_chishine->m_depth_stream_is_ply)
            break;
        if (m_camera_chishine->capture_frame(TCAPTURE_TYPE::eCAPTURE_STREAM))
        {
            mat = cv::Mat(m_camera_chishine->frame_height, m_camera_chishine->frame_width,
                CV_8UC3, m_camera_chishine->image_depth.data());

            if (mat.data != nullptr)
            {
                ui.label_Depth->setScaledContents(true);
                image = TCameraChishine::Mat_to_QImage(mat);
                if (!image.isNull())
                    ui.label_Depth->setPixmap(QPixmap::fromImage(image));
            }

            else
            {
                std::cout << "深度数据为空!" << std::endl;
            }
        }

        //线程sleep50ms
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void TMainWindow::realtime_display_pointcloud()
{
    m_viewer_realtime.reset(new pcl::visualization::PCLVisualizer("3DViewer"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> src_h(
        m_camera_chishine->image_cloud, 0, 0, 255); //给点云定义一个颜色 RGB模式  为全G 绿色  
    m_viewer_realtime->setBackgroundColor(255, 255, 255);  //设置背景颜色  255,255,255就是白色
    m_viewer_realtime->addPointCloud(m_camera_chishine->image_cloud, src_h, "source cloud");  //把点云加入到显示器里
    m_viewer_realtime->addCoordinateSystem(1.0);

    //检测到深度流处于开启状态即进入循环显示实时图像
    while (m_camera_chishine->stream_Depth_flag)
    {
        if (!m_camera_chishine->m_depth_stream_is_ply)
            break;
        if (m_camera_chishine->capture_frame(TCAPTURE_TYPE::eCAPTURE_STREAM))
        {
            int show_type = 0;
            if (show_type == 0)//pcl
            {
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> src_color(m_camera_chishine->image_cloud, 0, 0, 255); //给点云定义颜色 blue
                m_viewer_realtime->removeAllPointClouds();
                m_viewer_realtime->addPointCloud(m_camera_chishine->image_cloud, src_color, "source cloud");

                m_viewer_realtime->spinOnce(100);
            }
            if (show_type == 1)//vtk
            {
                m_viewer_realtime->removeAllPointClouds();

                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
                cloud_tmp->clear();
                for (auto it = m_camera_chishine->image_cloud->begin(); it != m_camera_chishine->image_cloud->end(); it++)
                {
                    cloud_tmp->emplace_back(it->x, it->y, it->z);
                }

                m_cloud->clear();
                pcl::copyPointCloud(*cloud_tmp, *m_cloud);

                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_color(m_cloud, 0, 0, 255); //点云流颜色为绿色
                m_viewer_realtime->addPointCloud(m_cloud, src_color);
                ui.vtk_realtime->update();
            }
        }
        else
        {
            std::cout << "点云数据为空!" << std::endl;
        }
        //线程sleep50ms
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void TMainWindow::on_btn_get_point_cloud_ply_clicked()
{
    //标记要采集ply格式的深度图 然后开启单张捕获
    m_camera_chishine->m_depth_single_is_ply = true;
    if (m_camera_chishine->capture_frame(TCAPTURE_TYPE::eCAPTURE_SINGLE))
    {
        std::cout << "ply格式深度图象获取成功" << std::endl;
    }
    else
    {
        std::cout << "ply格式深度图象获取失败" << std::endl;
    }
}

void TMainWindow::on_btn_get_point_cloud_bmp_clicked()
{
    //标记要采集bmp格式的深度图 然后开启单张捕获
    m_camera_chishine->m_depth_single_is_ply = false;
    if (m_camera_chishine->capture_frame(TCAPTURE_TYPE::eCAPTURE_SINGLE))
    {
        std::cout << "bmp格式深度图象获取成功" << std::endl;
    }
    else
    {
        std::cout << "bmp格式深度图象获取失败" << std::endl;
    }
}

void TMainWindow::set_depth_range()
{
    int depth_min = ui.lineEdit_depth_min->text().toInt();
    int depth_max = ui.lineEdit_depth_max->text().toInt();
    if (depth_min > depth_max)
    {
        std::cout << "请输入正确的深度范围" << std::endl << std::endl;
        return;
    }
    m_camera_chishine->set_para_depth_range(depth_min, depth_max);

    set_para_ui(m_camera_chishine->get_para());
}

void TMainWindow::set_exposure()
{
    float exposure_time = ui.lineEdit_exposure_time->text().toFloat();
    m_camera_chishine->set_para_exposure(exposure_time);

    set_para_ui(m_camera_chishine->get_para());
}

void TMainWindow::set_gain()
{
    float gain = ui.lineEdit_gain->text().toFloat();
    m_camera_chishine->set_para_gain(gain);

    set_para_ui(m_camera_chishine->get_para());
}

void TMainWindow::set_auto_exposure()
{
    bool auto_exposure = ui.ckb_auto_exposure->isChecked();
    m_camera_chishine->set_para_auto_exposure(auto_exposure);

    set_para_ui(m_camera_chishine->get_para());
}

void TMainWindow::on_btn_save_para_clicked()
{
    TCameraPara para;
    //para = m_camera_chishine->get_para();
    para.depth_min = ui.lineEdit_depth_min->text().toInt();
    para.depth_max = ui.lineEdit_depth_max->text().toInt();
    para.exposure_time = ui.lineEdit_exposure_time->text().toFloat();
    para.gain = ui.lineEdit_gain->text().toFloat();
    para.auto_exposure = (ui.ckb_auto_exposure->checkState() == Qt::Unchecked ? 0 : 2);

    //将para保存至文件中
    QJsonObject json = para.toJson();
    QString file_name = QFileDialog::getSaveFileName(this, "select save dir", "", "json(*.json)");
    if (!file_name.isNull())
    {
        QFile file(file_name);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        {
            QMessageBox::information(this, "information", tr("文件保存失败！"), QMessageBox::Yes, QMessageBox::Yes);
            return;
        }
        QJsonDocument tmp_doc(json);
        file.write(tmp_doc.toJson());
        std::cout << file_name.toStdString() << "保存成功" << std::endl << std::endl;
    }
}

void TMainWindow::on_btn_load_para_clicked()
{
    TCameraPara para;

    //读取文件中的参数，保存至para
    QString para_file = QFileDialog::getOpenFileName(this, "select camera parameter file", "", "json(*.json);;All files(*.*)");
    QFile camera_para(para_file);
    if (!para_file.isEmpty())
    {
        QFile camera_para(para_file);
        if (!camera_para.exists())
        {
            QMessageBox::information(this, "Information", "camera parameter file doesn't existed!");
            return;
        }
        if (!camera_para.open(QIODevice::ReadOnly))
        {
            QMessageBox::information(this, "Information", "camera parameter file open failed!");
            return;
        }
        QString value = camera_para.readAll();
        camera_para.close();

        QJsonParseError parseJsonErr;
        QJsonDocument document = QJsonDocument::fromJson(value.toUtf8(), &parseJsonErr);
        if (!(parseJsonErr.error == QJsonParseError::NoError))
        {
            std::cout << "解析json文件错误！" << std::endl << std::endl;
            return;
        }
        QJsonObject json = document.object();
        para.fromJson(json);

        set_para_camera(para);
        TCameraPara para_show = m_camera_chishine->get_para();
        set_para_ui(para_show);
    }
}

void TMainWindow::set_HDR_model(int hdr_model)
{
    switch (hdr_model)
    {
    case 0://关闭
    {
        ui.cbb_hdr_level->setEnabled(false);
        ui.lineEdit_hdr_exposure_1->setEnabled(false);
        ui.lineEdit_hdr_exposure_2->setEnabled(false);
        ui.lineEdit_hdr_exposure_3->setEnabled(false);
        ui.lineEdit_hdr_exposure_4->setEnabled(false);
        ui.lineEdit_hdr_gain_1->setEnabled(false);
        ui.lineEdit_hdr_gain_2->setEnabled(false);
        ui.lineEdit_hdr_gain_3->setEnabled(false);
        ui.lineEdit_hdr_gain_4->setEnabled(false);

        m_camera_chishine->set_HDR_model(HDR_MODE_OFF);

        break;
    }
    case 1://高光
    {
        //开启hdr
        ui.cbb_hdr_level->setEnabled(true);
        m_camera_chishine->set_HDR_model(HDR_MODE_HIGH_RELECT);

        //获取hdr等级 获取参数	
        PropertyExtension value;
        if (m_camera_chishine->get_hdr_para(value))
            set_hdr_para_ui(value);

        break;
    }
    case 2://黑白
    {
        ui.cbb_hdr_level->setEnabled(true);
        m_camera_chishine->set_HDR_model(HDR_MODE_LOW_RELECT);

        //获取hdr等级 获取参数	
        PropertyExtension value;
        if (m_camera_chishine->get_hdr_para(value))
            set_hdr_para_ui(value);

        break;
    }
    case 3://复合
    {
        ui.cbb_hdr_level->setEnabled(true);
        m_camera_chishine->set_HDR_model(HDR_MODE_ALL_RELECT);

        //获取hdr等级 获取参数	
        PropertyExtension value;
        if (m_camera_chishine->get_hdr_para(value))
            set_hdr_para_ui(value);

        break;
    }
    default:
    {
        break;
    }
    }

    if (hdr_model != 0)
    {
        int level = ui.cbb_hdr_level->currentIndex() + 2;
        for (int i = 2; i <= level; i++)
        {
            switch (i)
            {
            case 2:
            {
                ui.lineEdit_hdr_exposure_1->setEnabled(true);
                ui.lineEdit_hdr_exposure_2->setEnabled(true);
                ui.lineEdit_hdr_gain_1->setEnabled(true);
                ui.lineEdit_hdr_gain_2->setEnabled(true);

                break;
            }
            case 3:
            {
                ui.lineEdit_hdr_exposure_3->setEnabled(true);
                ui.lineEdit_hdr_gain_3->setEnabled(true);

                break;
            }
            case 4:
            {
                ui.lineEdit_hdr_exposure_4->setEnabled(true);
                ui.lineEdit_hdr_gain_4->setEnabled(true);

                break;
            }
            default:
            {
                break;
            }
            }
        }//end for
    }
}

void TMainWindow::set_HDR_level(int hdr_level)
{
    PropertyExtension value;
    value.hdrExposureSetting.count = hdr_level + 2;

    for (int i = 2; i <= hdr_level + 2; i++)
    {
        switch (i)
        {
        case 2:
        {
            //set enabled
            ui.lineEdit_hdr_exposure_1->setEnabled(true);
            ui.lineEdit_hdr_exposure_2->setEnabled(true);
            ui.lineEdit_hdr_exposure_3->setEnabled(false);
            ui.lineEdit_hdr_exposure_4->setEnabled(false);
            ui.lineEdit_hdr_gain_1->setEnabled(true);
            ui.lineEdit_hdr_gain_2->setEnabled(true);
            ui.lineEdit_hdr_gain_3->setEnabled(false);
            ui.lineEdit_hdr_gain_4->setEnabled(false);

            value.hdrExposureSetting.param[0].exposure = ui.lineEdit_hdr_exposure_1->text().toInt();
            value.hdrExposureSetting.param[0].gain = ui.lineEdit_hdr_gain_1->text().toInt();
            value.hdrExposureSetting.param[1].exposure = ui.lineEdit_hdr_exposure_2->text().toInt();
            value.hdrExposureSetting.param[1].gain = ui.lineEdit_hdr_gain_2->text().toInt();

            break;
        }
        case 3:
        {
            ui.lineEdit_hdr_exposure_3->setEnabled(true);
            ui.lineEdit_hdr_gain_3->setEnabled(true);

            value.hdrExposureSetting.param[2].exposure = ui.lineEdit_hdr_exposure_3->text().toInt();
            value.hdrExposureSetting.param[2].gain = ui.lineEdit_hdr_gain_3->text().toInt();

            break;
        }
        case 4:
        {
            ui.lineEdit_hdr_exposure_4->setEnabled(true);
            ui.lineEdit_hdr_gain_4->setEnabled(true);

            value.hdrExposureSetting.param[3].exposure = ui.lineEdit_hdr_exposure_4->text().toInt();
            value.hdrExposureSetting.param[3].gain = ui.lineEdit_hdr_gain_4->text().toInt();

            break;
        }
        default:
        {
            break;
        }
        }
    }

    //根据value设置camera
    set_hdr_camera(value);

    m_camera_chishine->get_hdr_para(value);
    set_hdr_para_ui(value);
}

void TMainWindow::set_HDR_level1_exposure()
{
    PropertyExtension value;

    m_camera_chishine->get_hdr_para(value);
    float exposure_time = ui.lineEdit_hdr_exposure_1->text().toFloat();
    value.hdrExposureSetting.param[0].exposure = exposure_time;
    m_camera_chishine->set_hdr_camera(value);

    m_camera_chishine->get_hdr_para(value);
    set_hdr_para_ui(value);
}

void TMainWindow::set_HDR_level2_exposure()
{
    PropertyExtension value;

    m_camera_chishine->get_hdr_para(value);
    float exposure_time = ui.lineEdit_hdr_exposure_2->text().toFloat();
    value.hdrExposureSetting.param[1].exposure = exposure_time;
    m_camera_chishine->set_hdr_camera(value);

    m_camera_chishine->get_hdr_para(value);
    set_hdr_para_ui(value);
}

void TMainWindow::set_HDR_level3_exposure()
{
    PropertyExtension value;

    m_camera_chishine->get_hdr_para(value);
    float exposure_time = ui.lineEdit_hdr_exposure_3->text().toFloat();
    value.hdrExposureSetting.param[2].exposure = exposure_time;
    m_camera_chishine->set_hdr_camera(value);

    m_camera_chishine->get_hdr_para(value);
    set_hdr_para_ui(value);
}

void TMainWindow::set_HDR_level4_exposure()
{
    PropertyExtension value;

    m_camera_chishine->get_hdr_para(value);
    float exposure_time = ui.lineEdit_hdr_exposure_4->text().toFloat();
    value.hdrExposureSetting.param[3].exposure = exposure_time;
    m_camera_chishine->set_hdr_camera(value);

    m_camera_chishine->get_hdr_para(value);
    set_hdr_para_ui(value);
}

void TMainWindow::set_HDR_level1_gain()
{
    PropertyExtension value;

    m_camera_chishine->get_hdr_para(value);
    float gain = ui.lineEdit_hdr_gain_1->text().toFloat();
    value.hdrExposureSetting.param[0].gain = gain;
    m_camera_chishine->set_hdr_camera(value);

    m_camera_chishine->get_hdr_para(value);
    set_hdr_para_ui(value);
}

void TMainWindow::set_HDR_level2_gain()
{
    PropertyExtension value;

    m_camera_chishine->get_hdr_para(value);
    float gain = ui.lineEdit_hdr_gain_2->text().toFloat();
    value.hdrExposureSetting.param[1].gain = gain;
    m_camera_chishine->set_hdr_camera(value);

    m_camera_chishine->get_hdr_para(value);
    set_hdr_para_ui(value);
}

void TMainWindow::set_HDR_level3_gain()
{
    PropertyExtension value;

    m_camera_chishine->get_hdr_para(value);
    float gain = ui.lineEdit_hdr_gain_3->text().toFloat();
    value.hdrExposureSetting.param[2].gain = gain;
    m_camera_chishine->set_hdr_camera(value);

    m_camera_chishine->get_hdr_para(value);
    set_hdr_para_ui(value);
}

void TMainWindow::set_HDR_level4_gain()
{
    PropertyExtension value;

    m_camera_chishine->get_hdr_para(value);
    float gain = ui.lineEdit_hdr_gain_4->text().toFloat();
    value.hdrExposureSetting.param[3].gain = gain;
    m_camera_chishine->set_hdr_camera(value);

    m_camera_chishine->get_hdr_para(value);
    set_hdr_para_ui(value);
}

void TMainWindow::realtime_stream_changed(int index)
{
    if (index == 0)//深度流
    {
        m_camera_chishine->m_depth_stream_is_ply = false;
        std::thread thread1(&TMainWindow::realtime_display_Depth, this);
        thread1.detach();
        std::cout << "已切换为深度流" << std::endl;
    }

    if (index == 1)//点云流
    {
        m_camera_chishine->m_depth_stream_is_ply = true;
        std::thread thread2(&TMainWindow::realtime_display_pointcloud, this);
        thread2.detach();
        std::cout << "已切换为点云流" << std::endl;
    }
}

void TMainWindow::on_btn_open_pc_clicked()
{
    //if (m_viewer_once->wasStopped())
    //    return;
    QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", "", "ply(*.ply);;pcd(*.pcd)");
    if (fileName.isEmpty())
        return;
    std::string filepath = fileName.toStdString();
    std::string ext = filepath.substr(filepath.size() - 3, 3);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (ext == "pcd")
    {
        if (-1 == pcl::io::loadPCDFile(filepath, *cloud)) //打开点云文件
        {
            std::cout << "打开pcd点云文件失败" << std::endl;
            return;
        }
    }

    if (ext == "ply")
    { 
        if (-1 == pcl::io::loadPLYFile(filepath, *cloud)) //打开点云文件
        {
            std::cout << "打开ply点云文件失败" << std::endl;
            return;
        }
    }

    m_viewer_once.reset(new pcl::visualization::PCLVisualizer("3DViewer"));

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_color(cloud, 0, 0, 255); //给点云定义一个颜色
    m_viewer_once->addPointCloud(cloud, src_color, "source cloud");  //把点云加入到显示器里
    m_viewer_once->setBackgroundColor(255, 255, 255);  //设置背景颜色  255,255,255就是白色
    m_viewer_once->addCoordinateSystem(1.0);

    while (m_viewer_once->wasStopped())
        m_viewer_once->spinOnce(100);

    //cloud->~PointCloud();
    //有个bug 打开两个窗口  关闭第二个的时候会崩
}

void TMainWindow::on_btn_restart_depth_stream_clicked()
{
    //重启深度流
    m_camera_chishine->stream_IR_flag = false;
    m_camera_chishine->stream_Depth_flag = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    m_camera_chishine->stop_depth_stream(STREAM_TYPE_DEPTH);

    if (m_camera_chishine->start_depth_stream(STREAM_FORMAT_Z16, TCameraChishine::depthCallback_Depth))
    {
        m_camera_chishine->stream_Depth_flag = true;

        int index = ui.realtimeWidget->currentIndex();
        if (index == 0)
        {
            std::thread thread1(&TMainWindow::realtime_display_Depth, this);
            thread1.detach();
        }
        if (index == 1)
        {
            std::thread thread2(&TMainWindow::realtime_display_pointcloud, this);
            thread2.detach();
        }
    }
    else
    {
        std::cout << "开启深度图流失败，请点击主界面重启深度流" << std::endl;
    }
}

void TMainWindow::init_vtk_realtime()
{
    m_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    m_viewer_realtime.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    m_viewer_realtime->addPointCloud(m_cloud);
    m_viewer_realtime->setBackgroundColor(255, 255, 255); //设置背景颜色  255,255,255就是白色
    m_viewer_realtime->addCoordinateSystem(1.0);

    ui.vtk_realtime->SetRenderWindow(m_viewer_realtime->getRenderWindow());
    m_viewer_realtime->setupInteractor(ui.vtk_realtime->GetInteractor(), ui.vtk_realtime->GetRenderWindow());
    ui.vtk_realtime->update();
}

void TMainWindow::init_viewer()
{
    //m_viewer_once.reset(new pcl::visualization::PCLVisualizer("3DViewer"));
    //m_viewer_once->setBackgroundColor(255, 255, 255);  //设置背景颜色  255,255,255就是白色
    //m_viewer_once->addCoordinateSystem(1.0);
}

void TMainWindow::on_btn_stl2pcd_clicked()
{
    trans_stl(0);
}

void TMainWindow::on_btn_stl2ply_clicked()
{
    trans_stl(1);
}

void TMainWindow::on_btn_obj2pcd_clicked()
{
    trans_obj(0);
}

void TMainWindow::on_btn_obj2ply_clicked()
{
    trans_obj(1);
}

void TMainWindow::trans_stl(int a_type)
{
    //读取CAD模型
    QString fileName = QFileDialog::getOpenFileName(this, "Select stl file", "", "stl(*.stl)");
    if (fileName.isEmpty())
        return;
    std::string filepath = fileName.toStdString();

    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(filepath.c_str());
    reader->Update();

    //先转出到polydata格式
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    polydata = reader->GetOutput();
    polydata->GetNumberOfPoints();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    //从polydata转pcd
    pcl::io::vtkPolyDataToPointCloud(polydata, *cloud);

    //保存pcd or ply文件
    if (a_type == 0)
        pcl::io::savePCDFileASCII("stl2pcd.pcd", *cloud);
    if (a_type == 1)
        pcl::io::savePLYFileASCII("stl2ply.ply", *cloud);
}

void TMainWindow::trans_obj(int a_type)
{
    //读取CAD模型
    QString fileName = QFileDialog::getOpenFileName(this, "Select obj file", "", "obj(*.obj)");
    if (fileName.isEmpty())
        return;
    std::string filepath = fileName.toStdString();

    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileOBJ(filepath, mesh);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

    if (a_type == 0)
        pcl::io::savePCDFileASCII("obj2pcd.pcd", *cloud);
    if (a_type == 1)
        pcl::io::savePLYFileASCII("obj2ply.ply", *cloud);
}

void TMainWindow::on_btn_test_clicked()
{
    if (m_camera_chishine->open_camera())
    {
        m_camera_chishine->show_information();
        m_camera_chishine->get_stream_info();

        //ui状态更改
        ui.lineEdit_depth_min->setEnabled(true);
        ui.lineEdit_depth_max->setEnabled(true);
        ui.lineEdit_exposure_time->setEnabled(true);
        ui.lineEdit_gain->setEnabled(true);
        ui.cbb_hdr->setEnabled(true);
        ui.btn_edit_roi->setEnabled(true);//未实现
        ui.ckb_auto_exposure->setEnabled(true);
        ui.btn_save_para->setEnabled(true);
        ui.btn_load_para->setEnabled(true);
        ui.btn_get_point_cloud_bmp->setEnabled(true);
        ui.btn_get_point_cloud_ply->setEnabled(true);
        ui.btn_restart_depth_stream->setEnabled(true);

        //初始化para
        set_para_ui(m_camera_chishine->get_para());
        PropertyExtension value;
        if (m_camera_chishine->get_hdr_model(value))
            set_hdr_model_ui(value);
        if (value.hdrMode != HDR_MODE_OFF)
        {
            if (m_camera_chishine->get_hdr_para(value))
                set_hdr_para_ui(value);
        }

        //开启深度流
        m_camera_chishine->stream_IR_flag = false;
        m_camera_chishine->stream_Depth_flag = false;
        ui.realtimeWidget->setCurrentIndex(0);
        if (m_camera_chishine->start_depth_stream(STREAM_FORMAT_Z16, TCameraChishine::depthCallback_Depth))
        {
            m_camera_chishine->stream_Depth_flag = true;
            std::thread thread1(&TMainWindow::realtime_display_Depth, this);
            thread1.detach();
        }
        else
        {
            std::cout << "开启深度图流失败，请点击主界面重启深度流" << std::endl;
        }

    }
}

