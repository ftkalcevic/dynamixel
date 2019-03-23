TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    Device.cpp \
    Dynamixel.cpp \
    linuxcnc_test.cpp \
    pch.cpp \
    RX48.cpp

HEADERS += \
    Device.h \
    Dynamixel.h \
    linuxcnc_test.h \
    pch.h \
    RX48.h
