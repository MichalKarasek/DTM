#-------------------------------------------------
#
# Project created by QtCreator 2017-11-27T10:20:58
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = DTM
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS
CONFIG += c++11
# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        widget.cpp \
    qpoint3d.cpp \
    edge.cpp \
    algorithms.cpp \
    sortbyxasc.cpp \
    triangle.cpp \
    graphic.cpp \
    sortbyyasc.cpp \
    sortbyslope.cpp \
    sortbyexposition.cpp

HEADERS += \
        widget.h \
    qpoint3d.h \
    edge.h \
    algorithms.h \
    sortbyxasc.h \
    triangle.h \
    graphic.h \
    sortbyyasc.h \
    sortbyslope.h \
    sortbyexposition.h

FORMS += \
        widget.ui
