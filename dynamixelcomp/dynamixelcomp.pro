TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt
EMCPATH = $$(EMC2_HOME)
INCLUDEPATH += $$EMCPATH/include
DEFINES += ULAPI
DEFINES += "THREAD_FLAVOR_ID=RTAPI_RT_PREEMPT_ID"
LIBS += -L$$EMCPATH/lib -llinuxcnchal -llinuxcnc -lnml -llinuxcncini


SOURCES += \
    dynamixelcomp.cpp \
    Device.cpp \
    Dynamixel.cpp \
    RX28.cpp

HEADERS += \
    Device.h \
    Dynamixel.h \
    RX28.h \
    pch.h
