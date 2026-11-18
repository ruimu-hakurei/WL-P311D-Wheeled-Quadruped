# 控制链路、日志与启动辅助各自独立编译。
CXX_SOURCES = wl_drive_cpp.cpp robot_io.cpp rolling.cpp kinematics.cpp \
              trajectory.cpp motion.cpp body_control.cpp gait.cpp crawl_controller.cpp estimation.cpp recorder.cpp \
              metadata.cpp file_snapshot.cpp detail/sha256.cpp run_options.cpp trot_controller.cpp wheel_ground.cpp

CFLAGS += -std=c++17 -Wextra -Wpedantic

ifndef WEBOTS_HOME
$(error WEBOTS_HOME is not set. Build from Webots or set WEBOTS_HOME)
endif

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH := $(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
