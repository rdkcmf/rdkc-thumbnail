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
USE_HTTPCLIENT = yes
USE_RFCCONFIG = yes
USE_CONFIGMGR = yes
USE_RTMESSAGE = yes
USE_DATAPROVIDER = no
SUPPORT_MXML = yes
USE_MFRLIB = yes
USE_SYSUTILS = yes
USE_LIBSYSWRAPPER =yes
USE_BREAKPAD = yes
USE_MISC = yes

include ${RDK_PROJECT_ROOT_PATH}/utility/AppsRule.mak

CFLAGS  += -DSUPPORT_IMAGETOOLS
ifeq ($(XCAM_MODEL), SCHC2)
CFLAGS += -DUSE_MFRLIB
CFLAGS += -DXCAM2
endif
ifeq ($(XCAM_MODEL), XHB1)
CFLAGS += -DUSE_MFRLIB
CFLAGS += -D_HAS_DING_
CFLAGS += -DOSI
CFLAGS += -DXHB1
CFLAGS  += -I../ding
SRCS += ../ding/DingNotification.cpp
endif
ifeq ($(XCAM_MODEL), XHC3)
CFLAGS += -DUSE_MFRLIB
CFLAGS += -DOSI
endif
CFLAGS  += -g -std=c++14 -fPIC -Wall -Wextra
SRCS   += thumbnailUpload.cpp
SRCS   += thumbnail_upload_main.cpp

OBJS	= $(SRCS:.cpp=.o)

RELEASE_TARGET = thumbnail_upload
DEBUG_TARGET = thumbnail_upload_debug

##all: $(RELEASE_TARGET) install
all: $(RELEASE_TARGET) $(DEBUG_TARGET)
RM = rm
INSTALL = install

$(OBJS): %.o: %.cpp
	$(CXX) -c $(CFLAGS) $< -o $@

$(RELEASE_TARGET): $(OBJS)
	$(CXX) $(CFLAGS) -o $@ $(OBJS) $(LIBFLAGS)
	$(STRIP) $(RELEASE_TARGET)

$(DEBUG_TARGET): $(OBJS)
	$(CXX) $(CFLAGS) -o $@ $(OBJS) $(LIBFLAGS)

clean:
	$(RM) -rf $(OBJS) *~ $(RELEASE_TARGET) $(DEBUG_TARGET)

install:
	$(INSTALL)  -D  $(RELEASE_TARGET) ${RDK_SDROOT}/usr/local/bin/$(RELEASE_TARGET)
	$(PLATFORM_BREAKPAD_BINARY) $(DEBUG_TARGET) > $(RELEASE_TARGET).sym
	sed -i "1s/$(DEBUG_TARGET)/$(RELEASE_TARGET)/" $(RELEASE_TARGET).sym
	mv -f $(RELEASE_TARGET).sym  $(PLATFORM_SYMBOL_PATH)
	echo "Debug symbol created for $(RELEASE_TARGET)"

uninstall:
	$(RM) -f $(RDK_SDROOT)/usr/local/bin/$(RELEASE_TARGET)
