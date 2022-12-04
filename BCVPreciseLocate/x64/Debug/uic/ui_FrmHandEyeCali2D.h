/********************************************************************************
** Form generated from reading UI file 'FrmHandEyeCali2D.ui'
**
** Created by: Qt User Interface Compiler version 5.15.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FRMHANDEYECALI2D_H
#define UI_FRMHANDEYECALI2D_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_THandEyeCali2D
{
public:
    QLabel *label_IR_R;
    QLineEdit *lineEdit_gain;
    QLabel *label_1;
    QPushButton *btn_show_TCP;
    QLineEdit *lineEdit_exposure_time;
    QLabel *label_IR_L;
    QLabel *label_2;
    QPushButton *btn_calculate;
    QPushButton *btn_capture;
    QPushButton *btn_get_IR;

    void setupUi(QWidget *THandEyeCali2D)
    {
        if (THandEyeCali2D->objectName().isEmpty())
            THandEyeCali2D->setObjectName(QString::fromUtf8("THandEyeCali2D"));
        THandEyeCali2D->resize(989, 544);
        label_IR_R = new QLabel(THandEyeCali2D);
        label_IR_R->setObjectName(QString::fromUtf8("label_IR_R"));
        label_IR_R->setGeometry(QRect(500, 40, 480, 300));
        lineEdit_gain = new QLineEdit(THandEyeCali2D);
        lineEdit_gain->setObjectName(QString::fromUtf8("lineEdit_gain"));
        lineEdit_gain->setEnabled(false);
        lineEdit_gain->setGeometry(QRect(270, 10, 111, 20));
        label_1 = new QLabel(THandEyeCali2D);
        label_1->setObjectName(QString::fromUtf8("label_1"));
        label_1->setGeometry(QRect(10, 10, 71, 20));
        btn_show_TCP = new QPushButton(THandEyeCali2D);
        btn_show_TCP->setObjectName(QString::fromUtf8("btn_show_TCP"));
        btn_show_TCP->setGeometry(QRect(550, 510, 75, 23));
        lineEdit_exposure_time = new QLineEdit(THandEyeCali2D);
        lineEdit_exposure_time->setObjectName(QString::fromUtf8("lineEdit_exposure_time"));
        lineEdit_exposure_time->setEnabled(false);
        lineEdit_exposure_time->setGeometry(QRect(100, 10, 111, 20));
        label_IR_L = new QLabel(THandEyeCali2D);
        label_IR_L->setObjectName(QString::fromUtf8("label_IR_L"));
        label_IR_L->setGeometry(QRect(10, 40, 480, 300));
        label_2 = new QLabel(THandEyeCali2D);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(230, 10, 71, 20));
        btn_calculate = new QPushButton(THandEyeCali2D);
        btn_calculate->setObjectName(QString::fromUtf8("btn_calculate"));
        btn_calculate->setEnabled(false);
        btn_calculate->setGeometry(QRect(450, 510, 81, 23));
        btn_capture = new QPushButton(THandEyeCali2D);
        btn_capture->setObjectName(QString::fromUtf8("btn_capture"));
        btn_capture->setGeometry(QRect(350, 510, 81, 23));
        btn_get_IR = new QPushButton(THandEyeCali2D);
        btn_get_IR->setObjectName(QString::fromUtf8("btn_get_IR"));
        btn_get_IR->setGeometry(QRect(250, 510, 81, 23));

        retranslateUi(THandEyeCali2D);

        QMetaObject::connectSlotsByName(THandEyeCali2D);
    } // setupUi

    void retranslateUi(QWidget *THandEyeCali2D)
    {
        THandEyeCali2D->setWindowTitle(QCoreApplication::translate("THandEyeCali2D", "THandEyeCali2D", nullptr));
        label_IR_R->setText(QString());
        lineEdit_gain->setText(QCoreApplication::translate("THandEyeCali2D", "2", nullptr));
        label_1->setText(QCoreApplication::translate("THandEyeCali2D", "\346\233\235\345\205\211\346\227\266\351\227\264(us)", nullptr));
        btn_show_TCP->setText(QCoreApplication::translate("THandEyeCali2D", "show TCP", nullptr));
        lineEdit_exposure_time->setText(QCoreApplication::translate("THandEyeCali2D", "30000", nullptr));
        label_IR_L->setText(QString());
        label_2->setText(QCoreApplication::translate("THandEyeCali2D", "\345\242\236\347\233\212", nullptr));
        btn_calculate->setText(QCoreApplication::translate("THandEyeCali2D", "\350\256\241\347\256\227", nullptr));
        btn_capture->setText(QCoreApplication::translate("THandEyeCali2D", "\346\213\215\347\205\247\346\217\220\345\217\226\350\247\222\347\202\271", nullptr));
        btn_get_IR->setText(QCoreApplication::translate("THandEyeCali2D", "\344\273\205\346\213\215\347\205\247", nullptr));
    } // retranslateUi

};

namespace Ui {
    class THandEyeCali2D: public Ui_THandEyeCali2D {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FRMHANDEYECALI2D_H
