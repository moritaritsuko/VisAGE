/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QSplitter>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QLabel *lblExibirIMG;
    QSplitter *splitter_6;
    QSplitter *splitter_4;
    QSplitter *splitter;
    QLabel *labelDX;
    QLineEdit *editDeltaX;
    QSplitter *splitter_2;
    QLabel *labelDY;
    QLineEdit *editDeltaY;
    QSplitter *splitter_3;
    QSplitter *splitter_5;
    QLabel *labelDZ;
    QLineEdit *editDeltaZ;
    QPushButton *btnMover;
    QSplitter *splitter_7;
    QSplitter *splitter_8;
    QSplitter *splitter_9;
    QLabel *labelDA;
    QLineEdit *editDeltaA;
    QSplitter *splitter_10;
    QLabel *labelDB;
    QLineEdit *editDeltaB;
    QSplitter *splitter_11;
    QSplitter *splitter_12;
    QLabel *labelDC;
    QLineEdit *editDeltaC;
    QPushButton *btnRotacionar;
    QLabel *lblImgCamE;
    QLabel *lblImgCamD;
    QSplitter *splitter_16;
    QSplitter *splitter_15;
    QSplitter *splitter_14;
    QSplitter *splitter_13;
    QPushButton *btnManual;
    QPushButton *btnRodar;
    QPushButton *btnStereo;
    QPushButton *btnGAMAGON;
    QPushButton *btnGAMAGOFF;
    QPushButton *btnCaptura;
    QPushButton *btnCapturaMono;
    QPushButton *btnCalibStr;
    QPushButton *btnDsip;
    QPushButton *btnFoco;
    QPushButton *btnPararCap;
    QPushButton *btnSalvar;
    QCheckBox *checkBoxSalvar;
    QPushButton *btninVision;
    QPushButton *btnIV_2;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1841, 801);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        lblExibirIMG = new QLabel(centralWidget);
        lblExibirIMG->setObjectName(QString::fromUtf8("lblExibirIMG"));
        lblExibirIMG->setGeometry(QRect(370, 190, 271, 16));
        splitter_6 = new QSplitter(centralWidget);
        splitter_6->setObjectName(QString::fromUtf8("splitter_6"));
        splitter_6->setGeometry(QRect(10, 10, 590, 25));
        splitter_6->setOrientation(Qt::Horizontal);
        splitter_4 = new QSplitter(splitter_6);
        splitter_4->setObjectName(QString::fromUtf8("splitter_4"));
        splitter_4->setOrientation(Qt::Horizontal);
        splitter = new QSplitter(splitter_4);
        splitter->setObjectName(QString::fromUtf8("splitter"));
        splitter->setOrientation(Qt::Horizontal);
        labelDX = new QLabel(splitter);
        labelDX->setObjectName(QString::fromUtf8("labelDX"));
        splitter->addWidget(labelDX);
        editDeltaX = new QLineEdit(splitter);
        editDeltaX->setObjectName(QString::fromUtf8("editDeltaX"));
        splitter->addWidget(editDeltaX);
        splitter_4->addWidget(splitter);
        splitter_2 = new QSplitter(splitter_4);
        splitter_2->setObjectName(QString::fromUtf8("splitter_2"));
        splitter_2->setOrientation(Qt::Horizontal);
        labelDY = new QLabel(splitter_2);
        labelDY->setObjectName(QString::fromUtf8("labelDY"));
        splitter_2->addWidget(labelDY);
        editDeltaY = new QLineEdit(splitter_2);
        editDeltaY->setObjectName(QString::fromUtf8("editDeltaY"));
        splitter_2->addWidget(editDeltaY);
        splitter_4->addWidget(splitter_2);
        splitter_6->addWidget(splitter_4);
        splitter_3 = new QSplitter(splitter_6);
        splitter_3->setObjectName(QString::fromUtf8("splitter_3"));
        splitter_3->setOrientation(Qt::Horizontal);
        splitter_5 = new QSplitter(splitter_3);
        splitter_5->setObjectName(QString::fromUtf8("splitter_5"));
        splitter_5->setOrientation(Qt::Horizontal);
        labelDZ = new QLabel(splitter_5);
        labelDZ->setObjectName(QString::fromUtf8("labelDZ"));
        splitter_5->addWidget(labelDZ);
        editDeltaZ = new QLineEdit(splitter_5);
        editDeltaZ->setObjectName(QString::fromUtf8("editDeltaZ"));
        splitter_5->addWidget(editDeltaZ);
        splitter_3->addWidget(splitter_5);
        splitter_6->addWidget(splitter_3);
        btnMover = new QPushButton(splitter_6);
        btnMover->setObjectName(QString::fromUtf8("btnMover"));
        splitter_6->addWidget(btnMover);
        splitter_7 = new QSplitter(centralWidget);
        splitter_7->setObjectName(QString::fromUtf8("splitter_7"));
        splitter_7->setGeometry(QRect(10, 40, 590, 25));
        splitter_7->setOrientation(Qt::Horizontal);
        splitter_8 = new QSplitter(splitter_7);
        splitter_8->setObjectName(QString::fromUtf8("splitter_8"));
        splitter_8->setOrientation(Qt::Horizontal);
        splitter_9 = new QSplitter(splitter_8);
        splitter_9->setObjectName(QString::fromUtf8("splitter_9"));
        splitter_9->setOrientation(Qt::Horizontal);
        labelDA = new QLabel(splitter_9);
        labelDA->setObjectName(QString::fromUtf8("labelDA"));
        splitter_9->addWidget(labelDA);
        editDeltaA = new QLineEdit(splitter_9);
        editDeltaA->setObjectName(QString::fromUtf8("editDeltaA"));
        splitter_9->addWidget(editDeltaA);
        splitter_8->addWidget(splitter_9);
        splitter_10 = new QSplitter(splitter_8);
        splitter_10->setObjectName(QString::fromUtf8("splitter_10"));
        splitter_10->setOrientation(Qt::Horizontal);
        labelDB = new QLabel(splitter_10);
        labelDB->setObjectName(QString::fromUtf8("labelDB"));
        splitter_10->addWidget(labelDB);
        editDeltaB = new QLineEdit(splitter_10);
        editDeltaB->setObjectName(QString::fromUtf8("editDeltaB"));
        splitter_10->addWidget(editDeltaB);
        splitter_8->addWidget(splitter_10);
        splitter_7->addWidget(splitter_8);
        splitter_11 = new QSplitter(splitter_7);
        splitter_11->setObjectName(QString::fromUtf8("splitter_11"));
        splitter_11->setOrientation(Qt::Horizontal);
        splitter_12 = new QSplitter(splitter_11);
        splitter_12->setObjectName(QString::fromUtf8("splitter_12"));
        splitter_12->setOrientation(Qt::Horizontal);
        labelDC = new QLabel(splitter_12);
        labelDC->setObjectName(QString::fromUtf8("labelDC"));
        splitter_12->addWidget(labelDC);
        editDeltaC = new QLineEdit(splitter_12);
        editDeltaC->setObjectName(QString::fromUtf8("editDeltaC"));
        splitter_12->addWidget(editDeltaC);
        splitter_11->addWidget(splitter_12);
        splitter_7->addWidget(splitter_11);
        btnRotacionar = new QPushButton(splitter_7);
        btnRotacionar->setObjectName(QString::fromUtf8("btnRotacionar"));
        splitter_7->addWidget(btnRotacionar);
        lblImgCamE = new QLabel(centralWidget);
        lblImgCamE->setObjectName(QString::fromUtf8("lblImgCamE"));
        lblImgCamE->setGeometry(QRect(10, 120, 351, 16));
        lblImgCamD = new QLabel(centralWidget);
        lblImgCamD->setObjectName(QString::fromUtf8("lblImgCamD"));
        lblImgCamD->setGeometry(QRect(880, 120, 411, 16));
        splitter_16 = new QSplitter(centralWidget);
        splitter_16->setObjectName(QString::fromUtf8("splitter_16"));
        splitter_16->setGeometry(QRect(10, 80, 894, 27));
        splitter_16->setOrientation(Qt::Horizontal);
        splitter_15 = new QSplitter(splitter_16);
        splitter_15->setObjectName(QString::fromUtf8("splitter_15"));
        splitter_15->setOrientation(Qt::Horizontal);
        splitter_14 = new QSplitter(splitter_15);
        splitter_14->setObjectName(QString::fromUtf8("splitter_14"));
        splitter_14->setOrientation(Qt::Horizontal);
        splitter_13 = new QSplitter(splitter_14);
        splitter_13->setObjectName(QString::fromUtf8("splitter_13"));
        splitter_13->setOrientation(Qt::Horizontal);
        btnManual = new QPushButton(splitter_13);
        btnManual->setObjectName(QString::fromUtf8("btnManual"));
        splitter_13->addWidget(btnManual);
        btnRodar = new QPushButton(splitter_13);
        btnRodar->setObjectName(QString::fromUtf8("btnRodar"));
        splitter_13->addWidget(btnRodar);
        btnStereo = new QPushButton(splitter_13);
        btnStereo->setObjectName(QString::fromUtf8("btnStereo"));
        splitter_13->addWidget(btnStereo);
        btnGAMAGON = new QPushButton(splitter_13);
        btnGAMAGON->setObjectName(QString::fromUtf8("btnGAMAGON"));
        splitter_13->addWidget(btnGAMAGON);
        btnGAMAGOFF = new QPushButton(splitter_13);
        btnGAMAGOFF->setObjectName(QString::fromUtf8("btnGAMAGOFF"));
        splitter_13->addWidget(btnGAMAGOFF);
        btnCaptura = new QPushButton(splitter_13);
        btnCaptura->setObjectName(QString::fromUtf8("btnCaptura"));
        splitter_13->addWidget(btnCaptura);
        btnCapturaMono = new QPushButton(splitter_13);
        btnCapturaMono->setObjectName(QString::fromUtf8("btnCapturaMono"));
        splitter_13->addWidget(btnCapturaMono);
        splitter_14->addWidget(splitter_13);
        splitter_15->addWidget(splitter_14);
        btnCalibStr = new QPushButton(splitter_15);
        btnCalibStr->setObjectName(QString::fromUtf8("btnCalibStr"));
        splitter_15->addWidget(btnCalibStr);
        splitter_16->addWidget(splitter_15);
        btnDsip = new QPushButton(splitter_16);
        btnDsip->setObjectName(QString::fromUtf8("btnDsip"));
        splitter_16->addWidget(btnDsip);
        btnFoco = new QPushButton(splitter_16);
        btnFoco->setObjectName(QString::fromUtf8("btnFoco"));
        splitter_16->addWidget(btnFoco);
        btnPararCap = new QPushButton(splitter_16);
        btnPararCap->setObjectName(QString::fromUtf8("btnPararCap"));
        splitter_16->addWidget(btnPararCap);
        btnSalvar = new QPushButton(centralWidget);
        btnSalvar->setObjectName(QString::fromUtf8("btnSalvar"));
        btnSalvar->setGeometry(QRect(620, 40, 85, 27));
        checkBoxSalvar = new QCheckBox(centralWidget);
        checkBoxSalvar->setObjectName(QString::fromUtf8("checkBoxSalvar"));
        checkBoxSalvar->setGeometry(QRect(710, 40, 131, 22));
        btninVision = new QPushButton(centralWidget);
        btninVision->setObjectName(QString::fromUtf8("btninVision"));
        btninVision->setGeometry(QRect(620, 10, 85, 27));
        btnIV_2 = new QPushButton(centralWidget);
        btnIV_2->setObjectName(QString::fromUtf8("btnIV_2"));
        btnIV_2->setGeometry(QRect(710, 10, 85, 27));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1841, 27));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        lblExibirIMG->setText(QApplication::translate("MainWindow", "Aguardando Imagem para Exibi\303\247\303\243o...", 0, QApplication::UnicodeUTF8));
        labelDX->setText(QApplication::translate("MainWindow", "Delta X", 0, QApplication::UnicodeUTF8));
        editDeltaX->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        labelDY->setText(QApplication::translate("MainWindow", "Delta Y", 0, QApplication::UnicodeUTF8));
        editDeltaY->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        labelDZ->setText(QApplication::translate("MainWindow", "Delta Z", 0, QApplication::UnicodeUTF8));
        editDeltaZ->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        btnMover->setText(QApplication::translate("MainWindow", "Mover", 0, QApplication::UnicodeUTF8));
        labelDA->setText(QApplication::translate("MainWindow", "Delta A", 0, QApplication::UnicodeUTF8));
        editDeltaA->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        labelDB->setText(QApplication::translate("MainWindow", "Delta B", 0, QApplication::UnicodeUTF8));
        editDeltaB->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        labelDC->setText(QApplication::translate("MainWindow", "Delta C", 0, QApplication::UnicodeUTF8));
        editDeltaC->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        btnRotacionar->setText(QApplication::translate("MainWindow", "Rotacionar", 0, QApplication::UnicodeUTF8));
        lblImgCamE->setText(QApplication::translate("MainWindow", "Aguardando Imagem para Exibi\303\247\303\243o...", 0, QApplication::UnicodeUTF8));
        lblImgCamD->setText(QApplication::translate("MainWindow", "Aguardando Imagem para Exibi\303\247\303\243o...", 0, QApplication::UnicodeUTF8));
        btnManual->setText(QApplication::translate("MainWindow", "Manual", 0, QApplication::UnicodeUTF8));
        btnRodar->setText(QApplication::translate("MainWindow", "Posicionar", 0, QApplication::UnicodeUTF8));
        btnStereo->setText(QApplication::translate("MainWindow", "Metrologia", 0, QApplication::UnicodeUTF8));
        btnGAMAGON->setText(QApplication::translate("MainWindow", "GAMAG - ATIVAR", 0, QApplication::UnicodeUTF8));
        btnGAMAGOFF->setText(QApplication::translate("MainWindow", "GAMAG - DESATIVAR", 0, QApplication::UnicodeUTF8));
        btnCaptura->setText(QApplication::translate("MainWindow", "Capturar", 0, QApplication::UnicodeUTF8));
        btnCapturaMono->setText(QApplication::translate("MainWindow", "Capturar Mono", 0, QApplication::UnicodeUTF8));
        btnCalibStr->setText(QApplication::translate("MainWindow", "Calibrar Stereo", 0, QApplication::UnicodeUTF8));
        btnDsip->setText(QApplication::translate("MainWindow", "Disparidade", 0, QApplication::UnicodeUTF8));
        btnFoco->setText(QApplication::translate("MainWindow", "Foco", 0, QApplication::UnicodeUTF8));
        btnPararCap->setText(QApplication::translate("MainWindow", "Parar", 0, QApplication::UnicodeUTF8));
        btnSalvar->setText(QApplication::translate("MainWindow", "Salvar", 0, QApplication::UnicodeUTF8));
        checkBoxSalvar->setText(QApplication::translate("MainWindow", "Salvar Cont.", 0, QApplication::UnicodeUTF8));
        btninVision->setText(QApplication::translate("MainWindow", "inVision", 0, QApplication::UnicodeUTF8));
        btnIV_2->setText(QApplication::translate("MainWindow", "iVision 2", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
