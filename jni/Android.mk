# script expect linphone-root-dir variable to be set by parent !

#enable video on armv7 and x86 targets only
#since we want to modify BUILD_VIDEO and BUILD_X264 depending on platform, we need to make a copy because the
#variables given on command line take precedence over the ones defined internally.
ifeq ($(TARGET_ARCH_ABI), armeabi)
_BUILD_X264=0
_BUILD_VIDEO=0
else
_BUILD_X264=$(BUILD_X264)
_BUILD_VIDEO=$(BUILD_VIDEO)
endif
ifeq ($(_BUILD_VIDEO),0)
ifeq (,$(DUMP_VAR))
$(info $(TARGET_ARCH_ABI): Video is disabled for targets other than armeabi-v7a and x86)
endif
endif


ifeq ($(BUILD_GPLV3_ZRTP), 1)
	BUILD_SRTP=1
ZRTP_C_INCLUDE= \
	$(linphone-root-dir)/submodules/externals/libzrtpcpp/src
endif

ifeq ($(BUILD_SRTP), 1)
SRTP_C_INCLUDE= \
	$(linphone-root-dir)/submodules/externals/srtp \
	$(linphone-root-dir)/submodules/externals/srtp/include \
	$(linphone-root-dir)/submodules/externals/srtp/crypto/include
endif


#sqlite
ifeq ($(BUILD_SQLITE),1)
include $(linphone-root-dir)/submodules/externals/build/sqlite/Android.mk
endif

#libupnp
ifeq ($(BUILD_UPNP),1)
include $(linphone-root-dir)/submodules/externals/build/libupnp/Android.mk
endif

#libxml2 
include $(linphone-root-dir)/submodules/externals/build/libxml2/Android.mk

# Speex
include $(linphone-root-dir)/submodules/externals/build/speex/Android.mk

# Gsm
include $(linphone-root-dir)/submodules/externals/build/gsm/Android.mk

ifeq ($(BUILD_MEDIASTREAMER2_SDK), 0)
include $(linphone-root-dir)/submodules/externals/build/polarssl/Android.mk
include $(linphone-root-dir)/submodules/externals/build/antlr3/Android.mk
include $(linphone-root-dir)/submodules/belle-sip/build/android/Android.mk
endif



include $(linphone-root-dir)/submodules/linphone/oRTP/build/android/Android.mk

include $(linphone-root-dir)/submodules/linphone/mediastreamer2/build/android/Android.mk
include $(linphone-root-dir)/submodules/linphone/mediastreamer2/tools/Android.mk


# Openssl
ifeq ($(BUILD_GPLV3_ZRTP), 1)
ifeq (,$(DUMP_VAR))
$(info Openssl is required)
endif
include $(linphone-root-dir)/submodules/externals/openssl/Android.mk
endif

#tunnel
ifeq ($(BUILD_TUNNEL), 1)
include $(linphone-root-dir)/submodules/tunnel/Android.mk
endif

ifeq ($(BUILD_SILK), 1)
ifeq (,$(DUMP_VAR))
$(info $(TARGET_ARCH_ABI): Build proprietary SILK plugin for mediastreamer2)
endif
include $(linphone-root-dir)/submodules/mssilk/Android.mk
endif

include $(linphone-root-dir)/submodules/msilbc/Android.mk

ifeq ($(_BUILD_VIDEO),1)

ifeq ($(_BUILD_X264),1)
ifeq (,$(DUMP_VAR))
$(info $(TARGET_ARCH_ABI): Build X264 plugin for mediastreamer2)
endif
include $(linphone-root-dir)/submodules/msx264/Android.mk
include $(linphone-root-dir)/submodules/externals/build/vpu/Android.mk
include $(linphone-root-dir)/submodules/externals/build/x264/Android.mk
endif

include $(linphone-root-dir)/submodules/externals/build/ffmpeg/Android.mk
include $(linphone-root-dir)/submodules/externals/build/libvpx/Android.mk

endif #_BUILD_VIDEO


ifeq ($(BUILD_GPLV3_ZRTP), 1)
ifeq (,$(DUMP_VAR))
$(info $(TARGET_ARCH_ABI): Build ZRTP support - makes application GPLv3)
endif
include $(linphone-root-dir)/submodules/externals/build/libzrtpcpp/Android.mk
endif

ifeq ($(BUILD_SRTP), 1)
include $(linphone-root-dir)/submodules/externals/build/srtp/Android.mk
endif

include $(linphone-root-dir)/submodules/linphone/build/android/Android.mk

_BUILD_AMR=0
ifneq ($(BUILD_AMRNB), 0)
_BUILD_AMR=1
endif

ifneq ($(BUILD_AMRWB), 0)
_BUILD_AMR=1
endif

ifneq ($(_BUILD_AMR), 0)
include $(linphone-root-dir)/submodules/externals/build/opencore-amr/Android.mk
include $(linphone-root-dir)/submodules/msamr/Android.mk
endif

ifneq ($(BUILD_AMRWB), 0)
include $(linphone-root-dir)/submodules/externals/build/vo-amrwbenc/Android.mk
endif

ifneq ($(BUILD_G729), 0)
include $(linphone-root-dir)/submodules/bcg729/Android.mk
include $(linphone-root-dir)/submodules/bcg729/msbcg729/Android.mk
endif

ifneq ($(BUILD_OPUS), 0)
include $(linphone-root-dir)/submodules/externals/build/opus/Android.mk
endif

WEBRTC_BUILD_NEON_LIBS=false

# AECM
ifneq ($(BUILD_WEBRTC_AECM),0)

    ifneq ($(TARGET_ARCH), x86)

        ifeq ($(TARGET_ARCH_ABI), armeabi-v7a)
            $(info $(TARGET_ARCH_ABI): Build NEON modules for AECM)
            WEBRTC_BUILD_NEON_LIBS=true
        endif

        $(info $(TARGET_ARCH_ABI): Build AECM from WebRTC)

        include $(linphone-root-dir)/submodules/externals/build/webrtc/system_wrappers/Android.mk
        include $(linphone-root-dir)/submodules/externals/build/webrtc/modules/audio_processing/utility/Android.mk
        include $(linphone-root-dir)/submodules/externals/build/webrtc/modules/audio_processing/aecm/Android.mk
    endif
endif

# iSAC
ifneq ($(BUILD_WEBRTC_ISAC),0)

    # don't build for neon in x86
    ifeq ($(TARGET_ARCH_ABI), armeabi-v7a)
        $(info $(TARGET_ARCH_ABI): Build NEON modules for ISAC)
        WEBRTC_BUILD_NEON_LIBS=true
    endif

     $(info $(TARGET_ARCH_ABI): Build proprietary iSAC plugin for mediastreamer2)
     include $(linphone-root-dir)/submodules/externals/build/webrtc/modules/audio_coding/codecs/isac/fix/source/Android.mk
     include $(linphone-root-dir)/submodules/msisac/Android.mk
endif

# common modules for ISAC and AECM
ifneq ($(BUILD_WEBRTC_AECM)$(BUILD_WEBRTC_ISAC),00)
    $(info $(TARGET_ARCH_ABI): Build common modules for iSAC and AECM ($(BUILD_WEBRTC_AECM)$(BUILD_WEBRTC_ISAC)))
    include $(linphone-root-dir)/submodules/externals/build/webrtc/common_audio/signal_processing/Android.mk
endif

