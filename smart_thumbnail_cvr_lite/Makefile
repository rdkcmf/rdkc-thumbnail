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
USE_OPENCV = yes
USE_HTTPCLIENT = yes
USE_CONFIGMGR = yes
USE_RTMESSAGE = yes
USE_LIBSYSWRAPPER = yes
USE_PLUGINS = yes

include ${RDK_PROJECT_ROOT_PATH}/utility/AppsRule.mak
LIBS = $(LIBFLAGS)

USE_LEGACY_CONFIG_MGR = no

ifeq ($(XCAM_MODEL), SCHC2)
CFLAGS  += -DXCAM2
endif

CFLAGS  += -I./include
CFLAGS  += -std=c++14 -fPIC -Wall -Wextra

CFLAGS	+= -DUSE_FILE_UPLOAD

ifeq ($(USE_LEGACY_CONFIG_MGR), yes)
CFLAGS  += -DLEGACY_CFG_MGR
endif

CFLAGS += -DUSE_MFRLIB

SRC += smart_thumbnail.cpp main.cpp
OBJ = $(SRC:.cpp=.o)

TARGET = smartthumbnail_cvr_lite

all: $(TARGET)

RM = rm
INSTALL = install

$(TARGET): $(OBJ)
	$(CXX) -o $(@) $^ $(LIBS)

%.o:%.cpp
	$(CXX) -c $< $(CFLAGS)  -o $@

install:
	$(INSTALL) -D $(TARGET) ${RDK_SDROOT}/usr/local/bin/$(TARGET)

uninstall:
	$(RM) -f $(RDK_SDROOT)/usr/local/bin/$(TARGET)

clean:
	$(RM) -rf $(TARGET) $(OBJ)
