#-------------------------------------------------
#
# Project created by QtCreator 2014-06-11T16:49:40
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = VisAGE
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \   
    ../src/conectrobo.cpp \
    ../src/mensurium.cpp \
    ../src/prog.cpp \
    ../src/pugixml.cpp

HEADERS  += mainwindow.h \    
    ../include/VISAGE/conectrobo.hpp \
    ../include/VISAGE/mensurium.hpp \
    ../include/VISAGE/prog.hpp \
    ../include/VISAGE/pugiconfig.hpp \
    ../include/VISAGE/pugixml.hpp \
    ../include/VISAGE/conectrobo.hpp \
    ../include/VISAGE/mensurium.hpp \
    ../include/VISAGE/prog.hpp \
    ../include/VISAGE/pugiconfig.hpp \
    ../include/VISAGE/pugixml.hpp

FORMS    += mainwindow.ui

unix:!macx: LIBS += -L$$PWD/../../../../../../opt/pylon3/lib64/ -lgxapi

INCLUDEPATH += $$PWD/../../../../../../opt/pylon3/include
DEPENDPATH += $$PWD/../../../../../../opt/pylon3/include


unix:!macx: LIBS += -L$$PWD/../../../../../../opt/pylon3/lib64/ -lpylonbase

INCLUDEPATH += $$PWD/../../../../../../opt/pylon3/include
DEPENDPATH += $$PWD/../../../../../../opt/pylon3/include

unix:!macx: LIBS += -L$$PWD/../../../../../../opt/pylon3/lib64/ -lpylongigesupp

INCLUDEPATH += $$PWD/../../../../../../opt/pylon3/include
DEPENDPATH += $$PWD/../../../../../../opt/pylon3/include

unix:!macx: LIBS += -L$$PWD/../../../../../../opt/pylon3/lib64/ -lpylonutility

INCLUDEPATH += $$PWD/../../../../../../opt/pylon3/include
DEPENDPATH += $$PWD/../../../../../../opt/pylon3/include

unix:!macx: LIBS += -L$$PWD/../../../../../../opt/pylon3/lib64/ -lXerces-C_gcc40_v2_7

INCLUDEPATH += $$PWD/../../../../../../opt/pylon3/lib64
DEPENDPATH += $$PWD/../../../../../../opt/pylon3/lib64

unix:!macx: LIBS += -L$$PWD/../../../../../../opt/pylon3/genicam/library/CPP/lib/Linux64_x64/ -llog4cpp-static_gcc40_v2_3

INCLUDEPATH += $$PWD/../../../../../../opt/pylon3/genicam/library/CPP/include
DEPENDPATH += $$PWD/../../../../../../opt/pylon3/genicam/library/CPP/include

unix:!macx: PRE_TARGETDEPS += $$PWD/../../../../../../opt/pylon3/genicam/library/CPP/lib/Linux64_x64/liblog4cpp-static_gcc40_v2_3.a

unix:!macx: LIBS += -L$$PWD/../../../../../../usr/local/lib/ -lopencv_calib3d

INCLUDEPATH += $$PWD/../../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../../usr/local/include

unix:!macx: LIBS += -L$$PWD/../../../../../../usr/local/lib/ -lopencv_contrib

INCLUDEPATH += $$PWD/../../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../../usr/local/include

unix:!macx: LIBS += -L$$PWD/../../../../../../usr/local/lib/ -lopencv_core

INCLUDEPATH += $$PWD/../../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../../usr/local/include

unix:!macx: LIBS += -L$$PWD/../../../../../../usr/local/lib/ -lopencv_features2d

INCLUDEPATH += $$PWD/../../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../../usr/local/include

unix:!macx: LIBS += -L$$PWD/../../../../../../usr/local/lib/ -lopencv_highgui

INCLUDEPATH += $$PWD/../../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../../usr/local/include

unix:!macx: LIBS += -L$$PWD/../../../../../../usr/local/lib/ -lopencv_imgproc

INCLUDEPATH += $$PWD/../../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../../usr/local/include

unix:!macx: LIBS += -L$$PWD/../../../../../../opt/pylon3/genicam/bin/Linux64_x64/ -lGenApi_gcc40_v2_3

INCLUDEPATH += $$PWD/../../../../../../opt/pylon3/include
DEPENDPATH += $$PWD/../../../../../../opt/pylon3/include

unix:!macx: LIBS += -L$$PWD/../../../../../../opt/pylon3/genicam/bin/Linux64_x64/ -llog4cpp_gcc40_v2_3

INCLUDEPATH += $$PWD/../../../../../../opt/pylon3/include
DEPENDPATH += $$PWD/../../../../../../opt/pylon3/include

unix:!macx: LIBS += -L$$PWD/../../../../../../opt/pylon3/genicam/bin/Linux64_x64/ -lLog_gcc40_v2_3

INCLUDEPATH += $$PWD/../../../../../../opt/pylon3/include
DEPENDPATH += $$PWD/../../../../../../opt/pylon3/include

unix:!macx: LIBS += -L$$PWD/../../../../../../opt/pylon3/genicam/bin/Linux64_x64/ -lMathParser_gcc40_v2_3

INCLUDEPATH += $$PWD/../../../../../../opt/pylon3/include
DEPENDPATH += $$PWD/../../../../../../opt/pylon3/include

unix:!macx: LIBS += -L$$PWD/../../../../../../opt/pylon3/genicam/bin/Linux64_x64/ -lGCBase_gcc40_v2_3

INCLUDEPATH += $$PWD/../../../../../../opt/pylon3/include
DEPENDPATH += $$PWD/../../../../../../opt/pylon3/include

unix:!macx: LIBS += -L$$PWD/../../../../../../opt/pylon3/genicam/bin/Linux64_x64/GenApi/Generic/ -lXalan-C_gcc40_v1_10

INCLUDEPATH += $$PWD/../../../../../../opt/pylon3/include
DEPENDPATH += $$PWD/../../../../../../opt/pylon3/include

unix:!macx: LIBS += -L$$PWD/../../../../../../opt/pylon3/genicam/bin/Linux64_x64/GenApi/Generic/ -lXalanMessages_gcc40_v1_10

INCLUDEPATH += $$PWD/../../../../../../opt/pylon3/include
DEPENDPATH += $$PWD/../../../../../../opt/pylon3/include

unix:!macx: LIBS += -L$$PWD/../../../../../../opt/pylon3/genicam/bin/Linux64_x64/GenApi/Generic/ -lXerces-C_gcc40_v2_7

INCLUDEPATH += $$PWD/../../../../../../opt/pylon3/genicam/bin/Linux64_x64/GenApi/Generic
DEPENDPATH += $$PWD/../../../../../../opt/pylon3/genicam/bin/Linux64_x64/GenApi/Generic

unix:!macx: LIBS += -L$$PWD/../../../../../../opt/pylon3/genicam/bin/Linux64_x64/GenApi/Generic/ -lXMLLoader_gcc40_v2_3

INCLUDEPATH += $$PWD/../../../../../../opt/pylon3/include
DEPENDPATH += $$PWD/../../../../../../opt/pylon3/include

QMAKE_CXXFLAGS += -std=c++11

unix:!macx: LIBS += -L$$PWD/../../../../../opt/AMDAPP/lib/x86_64/ -lglut

INCLUDEPATH += $$PWD/../../../../../opt/AMDAPP/include
DEPENDPATH += $$PWD/../../../../../opt/AMDAPP/include

INCLUDEPATH += $$PWD/../../../../../../usr/lib/x86_64-linux-gnu
DEPENDPATH += $$PWD/../../../../../../usr/lib/x86_64-linux-gnu

#unix:!macx: LIBS += -L$$PWD/../../../../../../usr/lib/x86_64-linux-gnu -lGL
unix:!macx: LIBS += -L$$PWD/../../../../../../usr/lib/ -lGL

INCLUDEPATH += $$PWD/../../../../../../usr/include
DEPENDPATH += $$PWD/../../../../../../usr/include

unix:!macx: LIBS += -L$$PWD/../../../../../../usr/lib/x86_64-linux-gnu/ -lGLU

INCLUDEPATH += $$PWD/../../../../../../usr/lib/x86_64-linux-gnu
DEPENDPATH += $$PWD/../../../../../../usr/lib/x86_64-linux-gnu

unix:!macx: LIBS += -L$$PWD/../../../../../../opt/AMDAPP/lib/x86_64/ -lGLEW

INCLUDEPATH += $$PWD/../../../../../../opt/AMDAPP/include
DEPENDPATH += $$PWD/../../../../../../opt/AMDAPP/include

#INCLUDEPATH += /home/leandro/Projetos/AGE/ViAge/include
INCLUDEPATH += /home/gabriel/projects/AGE/VisAGE/include
