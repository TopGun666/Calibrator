TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp

INCLUDEPATH += /usr/include/

LIBS += /usr/local/lib/libopencv_core.so.2.4.13
LIBS += /usr/local/lib/libopencv_highgui.so.2.4.13
LIBS += /usr/local/lib/libopencv_imgproc.so.2.4.13
LIBS += /usr/local/lib/libopencv_calib3d.so.2.4.13

