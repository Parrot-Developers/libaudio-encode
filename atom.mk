
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

# API library. This is the library that most programs should use.
LOCAL_MODULE := libaudio-encode
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Audio encoding library
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_CFLAGS := -DAENC_API_EXPORTS -fvisibility=hidden -std=gnu99 -D_GNU_SOURCE
LOCAL_SRC_FILES := \
	src/aenc.c
LOCAL_LIBRARIES := \
	libaudio-defs \
	libaudio-encode-core \
	libpomp \
	libulog
LOCAL_CONFIG_FILES := config.in
$(call load-config)
LOCAL_CONDITIONAL_LIBRARIES := \
	CONFIG_AENC_FDK_AAC:libaudio-encode-fdk-aac \
	CONFIG_AENC_FAKEAAC:libaudio-encode-fakeaac
LOCAL_EXPORT_LDLIBS := -laudio-encode-core

include $(BUILD_LIBRARY)

include $(CLEAR_VARS)

# Core library, common code for all implementations and structures definitions.
# Used by implementations.
LOCAL_MODULE := libaudio-encode-core
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Audio encoding library: core files
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/core/include
LOCAL_CFLAGS := -DAENC_API_EXPORTS -fvisibility=hidden -std=gnu99 -D_GNU_SOURCE
LOCAL_SRC_FILES := \
	core/src/aenc_core.c
LOCAL_LIBRARIES := \
	libaac \
	libaudio-defs \
	libfutils \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libpomp \
	libulog

ifeq ("$(TARGET_OS)","windows")
  LOCAL_LDLIBS += -lws2_32
endif

include $(BUILD_LIBRARY)

include $(CLEAR_VARS)

# FDK AAC implementation. can be enabled in the product configuration.
LOCAL_MODULE := libaudio-encode-fdk-aac
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Audio encoding library: FDK AAC implementation
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/fdk-aac/include
LOCAL_CFLAGS := -DAENC_API_EXPORTS -fvisibility=hidden -std=gnu99 -D_GNU_SOURCE
LOCAL_SRC_FILES := \
	fdk-aac/src/aenc_fdk_aac.c
LOCAL_LIBRARIES := \
	fdk-aac \
	libaac \
	libaudio-defs \
	libaudio-encode-core \
	libfutils \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libpomp \
	libulog

ifeq ("$(TARGET_OS)","windows")
  LOCAL_LDLIBS += -lws2_32
endif

include $(BUILD_LIBRARY)

include $(CLEAR_VARS)

# FAKEAAC implementation. can be enabled in the product configuration.
LOCAL_MODULE := libaudio-encode-fakeaac
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Audio encoding library: fake AAC implementation
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/fakeaac/include
LOCAL_CFLAGS := -DAENC_API_EXPORTS -fvisibility=hidden -std=gnu99 -D_GNU_SOURCE
LOCAL_SRC_FILES := \
	fakeaac/src/aenc_fakeaac.c
LOCAL_LIBRARIES := \
	libaac \
	libaudio-defs \
	libaudio-encode-core \
	libfutils \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libpomp \
	libulog

ifeq ("$(TARGET_OS)","windows")
  LOCAL_LDLIBS += -lws2_32
endif

include $(BUILD_LIBRARY)

include $(CLEAR_VARS)

LOCAL_MODULE := aenc
LOCAL_DESCRIPTION := Audio encoding program
LOCAL_CATEGORY_PATH := multimedia
LOCAL_SRC_FILES := tools/aenc.c
LOCAL_LIBRARIES := \
	libaudio-defs \
	libaudio-encode \
	libaudio-raw \
	libfutils \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libpomp \
	libulog

ifeq ("$(TARGET_OS)","windows")
  LOCAL_LDLIBS += -lws2_32
endif

include $(BUILD_EXECUTABLE)
