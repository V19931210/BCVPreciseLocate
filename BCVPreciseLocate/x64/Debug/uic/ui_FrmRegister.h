/********************************************************************************
** Form generated from reading UI file 'FrmRegister.ui'
**
** Created by: Qt User Interface Compiler version 5.15.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FRMREGISTER_H
#define UI_FRMREGISTER_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_TRegister
{
public:
    QGroupBox *algo_para;
    QGroupBox *gpb_icp_para;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *label_5;
    QDoubleSpinBox *sbx_icp_distance;
    QSpinBox *sbx_icp_iteration;
    QDoubleSpinBox *sbx_icp_fitness_epsilon;
    QSpinBox *sbx_icp_trans_epsilon;
    QLabel *label_6;
    QLabel *label_11;
    QSpinBox *sbx_icp_correspond_number;
    QGroupBox *gpb_ndt_para;
    QSpinBox *sbx_ndt_iteration;
    QLabel *label_9;
    QLabel *label_8;
    QDoubleSpinBox *sbx_ndt_stepsize;
    QLabel *label_10;
    QLabel *label_7;
    QDoubleSpinBox *sbx_ndt_trans_epsilon;
    QDoubleSpinBox *sbx_ndt_resolution;
    QLabel *label;
    QComboBox *cbb_algo_select;

    void setupUi(QWidget *TRegister)
    {
        if (TRegister->objectName().isEmpty())
            TRegister->setObjectName(QString::fromUtf8("TRegister"));
        TRegister->resize(802, 498);
        algo_para = new QGroupBox(TRegister);
        algo_para->setObjectName(QString::fromUtf8("algo_para"));
        algo_para->setGeometry(QRect(10, 40, 261, 351));
        gpb_icp_para = new QGroupBox(algo_para);
        gpb_icp_para->setObjectName(QString::fromUtf8("gpb_icp_para"));
        gpb_icp_para->setGeometry(QRect(10, 20, 241, 171));
        label_2 = new QLabel(gpb_icp_para);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(20, 20, 91, 16));
        label_3 = new QLabel(gpb_icp_para);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(20, 50, 91, 16));
        label_4 = new QLabel(gpb_icp_para);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(20, 80, 91, 16));
        label_5 = new QLabel(gpb_icp_para);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(20, 110, 91, 16));
        sbx_icp_distance = new QDoubleSpinBox(gpb_icp_para);
        sbx_icp_distance->setObjectName(QString::fromUtf8("sbx_icp_distance"));
        sbx_icp_distance->setGeometry(QRect(130, 20, 91, 22));
        sbx_icp_distance->setMaximum(20.000000000000000);
        sbx_icp_distance->setSingleStep(0.100000000000000);
        sbx_icp_distance->setValue(1.000000000000000);
        sbx_icp_iteration = new QSpinBox(gpb_icp_para);
        sbx_icp_iteration->setObjectName(QString::fromUtf8("sbx_icp_iteration"));
        sbx_icp_iteration->setGeometry(QRect(130, 50, 91, 22));
        sbx_icp_iteration->setMaximum(300);
        sbx_icp_iteration->setValue(100);
        sbx_icp_fitness_epsilon = new QDoubleSpinBox(gpb_icp_para);
        sbx_icp_fitness_epsilon->setObjectName(QString::fromUtf8("sbx_icp_fitness_epsilon"));
        sbx_icp_fitness_epsilon->setGeometry(QRect(130, 110, 91, 22));
        sbx_icp_fitness_epsilon->setMaximum(0.500000000000000);
        sbx_icp_fitness_epsilon->setSingleStep(0.010000000000000);
        sbx_icp_fitness_epsilon->setValue(0.010000000000000);
        sbx_icp_trans_epsilon = new QSpinBox(gpb_icp_para);
        sbx_icp_trans_epsilon->setObjectName(QString::fromUtf8("sbx_icp_trans_epsilon"));
        sbx_icp_trans_epsilon->setGeometry(QRect(130, 80, 91, 22));
        sbx_icp_trans_epsilon->setMinimum(-20);
        sbx_icp_trans_epsilon->setMaximum(0);
        sbx_icp_trans_epsilon->setValue(-10);
        label_6 = new QLabel(gpb_icp_para);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(110, 80, 16, 16));
        label_11 = new QLabel(gpb_icp_para);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        label_11->setGeometry(QRect(20, 140, 101, 16));
        sbx_icp_correspond_number = new QSpinBox(gpb_icp_para);
        sbx_icp_correspond_number->setObjectName(QString::fromUtf8("sbx_icp_correspond_number"));
        sbx_icp_correspond_number->setGeometry(QRect(130, 140, 91, 22));
        sbx_icp_correspond_number->setMaximum(300);
        sbx_icp_correspond_number->setValue(100);
        gpb_ndt_para = new QGroupBox(algo_para);
        gpb_ndt_para->setObjectName(QString::fromUtf8("gpb_ndt_para"));
        gpb_ndt_para->setGeometry(QRect(10, 190, 241, 151));
        sbx_ndt_iteration = new QSpinBox(gpb_ndt_para);
        sbx_ndt_iteration->setObjectName(QString::fromUtf8("sbx_ndt_iteration"));
        sbx_ndt_iteration->setGeometry(QRect(130, 20, 91, 22));
        sbx_ndt_iteration->setMaximum(300);
        sbx_ndt_iteration->setValue(30);
        label_9 = new QLabel(gpb_ndt_para);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setGeometry(QRect(20, 20, 91, 16));
        label_8 = new QLabel(gpb_ndt_para);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(20, 80, 91, 16));
        sbx_ndt_stepsize = new QDoubleSpinBox(gpb_ndt_para);
        sbx_ndt_stepsize->setObjectName(QString::fromUtf8("sbx_ndt_stepsize"));
        sbx_ndt_stepsize->setGeometry(QRect(130, 80, 91, 22));
        sbx_ndt_stepsize->setMaximum(0.500000000000000);
        sbx_ndt_stepsize->setSingleStep(0.010000000000000);
        sbx_ndt_stepsize->setValue(0.100000000000000);
        label_10 = new QLabel(gpb_ndt_para);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setGeometry(QRect(20, 50, 91, 16));
        label_7 = new QLabel(gpb_ndt_para);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(20, 110, 91, 16));
        sbx_ndt_trans_epsilon = new QDoubleSpinBox(gpb_ndt_para);
        sbx_ndt_trans_epsilon->setObjectName(QString::fromUtf8("sbx_ndt_trans_epsilon"));
        sbx_ndt_trans_epsilon->setGeometry(QRect(130, 110, 91, 22));
        sbx_ndt_trans_epsilon->setMaximum(0.100000000000000);
        sbx_ndt_trans_epsilon->setSingleStep(0.010000000000000);
        sbx_ndt_trans_epsilon->setValue(0.010000000000000);
        sbx_ndt_resolution = new QDoubleSpinBox(gpb_ndt_para);
        sbx_ndt_resolution->setObjectName(QString::fromUtf8("sbx_ndt_resolution"));
        sbx_ndt_resolution->setGeometry(QRect(130, 50, 91, 22));
        sbx_ndt_resolution->setMaximum(5.000000000000000);
        sbx_ndt_resolution->setSingleStep(0.100000000000000);
        sbx_ndt_resolution->setValue(1.000000000000000);
        label = new QLabel(TRegister);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(20, 10, 54, 12));
        cbb_algo_select = new QComboBox(TRegister);
        cbb_algo_select->addItem(QString());
        cbb_algo_select->addItem(QString());
        cbb_algo_select->setObjectName(QString::fromUtf8("cbb_algo_select"));
        cbb_algo_select->setGeometry(QRect(80, 10, 91, 22));

        retranslateUi(TRegister);

        QMetaObject::connectSlotsByName(TRegister);
    } // setupUi

    void retranslateUi(QWidget *TRegister)
    {
        TRegister->setWindowTitle(QCoreApplication::translate("TRegister", "TRegister", nullptr));
        algo_para->setTitle(QCoreApplication::translate("TRegister", "\345\217\202\346\225\260\350\260\203\346\225\264", nullptr));
        gpb_icp_para->setTitle(QCoreApplication::translate("TRegister", "ICP\345\217\202\346\225\260", nullptr));
        label_2->setText(QCoreApplication::translate("TRegister", "\346\234\200\345\244\247\346\254\247\345\274\217\350\267\235\347\246\273\357\274\232", nullptr));
        label_3->setText(QCoreApplication::translate("TRegister", "\346\234\200\345\244\247\350\277\255\344\273\243\346\254\241\346\225\260\357\274\232", nullptr));
        label_4->setText(QCoreApplication::translate("TRegister", "\350\275\254\346\215\242\347\237\251\351\230\265\345\256\271\345\267\256\357\274\232", nullptr));
        label_5->setText(QCoreApplication::translate("TRegister", "\345\235\207\346\226\271\350\257\257\345\267\256\345\256\271\345\267\256\357\274\232", nullptr));
        label_6->setText(QCoreApplication::translate("TRegister", "E", nullptr));
        label_11->setText(QCoreApplication::translate("TRegister", "\346\234\200\345\260\217\345\214\271\351\205\215\347\202\271\345\257\271\346\225\260\351\207\217\357\274\232", nullptr));
        gpb_ndt_para->setTitle(QCoreApplication::translate("TRegister", "NDT\345\217\202\346\225\260", nullptr));
        label_9->setText(QCoreApplication::translate("TRegister", "\346\234\200\345\244\247\350\277\255\344\273\243\346\254\241\346\225\260\357\274\232", nullptr));
        label_8->setText(QCoreApplication::translate("TRegister", "\346\234\200\345\244\247\346\220\234\347\264\242\346\255\245\351\225\277\357\274\232", nullptr));
        label_10->setText(QCoreApplication::translate("TRegister", "\347\275\221\346\240\274\345\210\206\350\276\250\347\216\207\357\274\232", nullptr));
        label_7->setText(QCoreApplication::translate("TRegister", "\346\234\200\345\260\217\350\275\254\346\215\242\345\267\256\345\274\202", nullptr));
        label->setText(QCoreApplication::translate("TRegister", "\351\205\215\345\207\206\347\256\227\346\263\225\357\274\232", nullptr));
        cbb_algo_select->setItemText(0, QCoreApplication::translate("TRegister", "ICP", nullptr));
        cbb_algo_select->setItemText(1, QCoreApplication::translate("TRegister", "NDT", nullptr));

    } // retranslateUi

};

namespace Ui {
    class TRegister: public Ui_TRegister {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FRMREGISTER_H
