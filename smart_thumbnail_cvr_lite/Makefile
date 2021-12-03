##########################################################################
# If not stated otherwise in this file or this component's LICENSE
# file the following copyright and licenses apply:
#
# Copyright 2019 RDK Management
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
##########################################################################

# Add dependent libraries
#USE_OPENCV = yes
USE_HTTPCLIENT = yes
USE_RFCCONFIG = yes
USE_CONFIGMGR = yes
USE_RTMESSAGE = yes
USE_DATAPROVIDER = no
USE_MFRLIB = yes
USE_LIBSYSWRAPPER = yes
USE_BREAKPAD = yes

ifeq ($(XCAM_MODEL), XHB1)
CFLAGS  += -DXHB1
#CFLAGS  += -D_HAS_DING_
USE_DEWARP = yes
#SRC += ../ding/DingNotification.cpp
#CFLAGS  += -I../ding
else
ifeq ($(XCAM_MODEL), XHC3)
CFLAGS  += -DXHC3
USE_DEWARP = yes
else
SUPPORT_MXML = yes
endif
endif

ifeq ($(XCAM_MODEL), SCHC2)
USE_DEWARP = yes
endif

ENABLE_DIRECT_FRAME_READ := true

ifneq ($(ENABLE_XSTREAMER), true)
USE_PLUGINS = yes
else
USE_CONSUMER = yes
endif

include ${RDK_PROJECT_ROOT_PATH}/utility/AppsRule.mak
LIBS = $(LIBFLAGS)

USE_LEGACY_CONFIG_MGR = no

ifeq ($(XCAM_MODEL), SCHC2)
CFLAGS  += -DXCAM2
endif

CFLAGS  += -I./include
CFLAGS  += -g -std=c++14 -fPIC -Wall -Wextra

CFLAGS	+= -DUSE_FILE_UPLOAD

ifeq ($(USE_LEGACY_CONFIG_MGR), yes)
CFLAGS  += -DLEGACY_CFG_MGR
endif

ifeq ($(ENABLE_XSTREAMER), true)
$(info( '*************XSTREAMER ENABLED**************')
CFLAGS  += -D_HAS_XSTREAM_
ifeq ($(ENABLE_DIRECT_FRAME_READ), true)
$(info( '*************DIRECT READ FRAME ENABLED**************')
CFLAGS   += -D_DIRECT_FRAME_READ_
endif
endif
LIBS    += -L$(RDK_PROJECT_ROOT_PATH)/opensource/lib  -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs
LIBS    += -Wl,-rpath-link=$(PROJ_ROOT)/vendor/img/fs/shadow_root/usr/lib
CFLAGS += -DUSE_MFRLIB

SRC += smart_thumbnail.cpp main.cpp
OBJ = $(SRC:.cpp=.o)

RELEASE_TARGET = smartthumbnail_cvr_lite
DEBUG_TARGET = smartthumbnail_cvr_lite_debug

all: $(RELEASE_TARGET) $(DEBUG_TARGET)

RM = rm
INSTALL = install

$(RELEASE_TARGET): $(OBJ)
	$(CXX) -o $(@) $^ $(LIBS)
	$(STRIP) $(RELEASE_TARGET)

$(DEBUG_TARGET): $(OBJ)
	$(CXX) -o $(@) $^ $(LIBS)

%.o:%.cpp
	$(CXX) -c $< $(CFLAGS)  -o $@

install:
	$(INSTALL) -D $(RELEASE_TARGET) ${RDK_SDROOT}/usr/local/bin/$(RELEASE_TARGET)
	$(PLATFORM_BREAKPAD_BINARY) $(DEBUG_TARGET) > $(RELEASE_TARGET).sym
	sed -i "1s/$(DEBUG_TARGET)/$(RELEASE_TARGET)/" $(RELEASE_TARGET).sym
	mv -f $(RELEASE_TARGET).sym  $(PLATFORM_SYMBOL_PATH)
	echo "Debug symbol created for $(RELEASE_TARGET)"

uninstall:
	$(RM) -f $(RDK_SDROOT)/usr/local/bin/$(RELEASE_TARGET)

clean:
	$(RM) -rf $(RELEASE_TARGET) $(DEBUG_TARGET) $(OBJ)
