LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)   

LOCAL_MODULE    := libvpu
LOCAL_SRC_FILES := libvpu.a  

include $(PREBUILT_STATIC_LIBRARY)

