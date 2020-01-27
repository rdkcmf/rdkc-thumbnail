/*
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

*/
#include "smart_thumbnail.h"
#include <unistd.h>

int main(int argc, char** argv)
{
	SmartThumbnail *smTnInstance = NULL;
	STH_STATUS status = STH_SUCCESS;
        time_t remainingTime = 0;

	struct timespec currTime;
	struct timespec startTime;
	memset(&startTime, 0, sizeof(startTime));
	memset(&currTime, 0, sizeof(currTime));

	signal(SIGTERM, SmartThumbnail::sigHandler);
	signal(SIGINT, SmartThumbnail::sigHandler);

	//initialize rdklogger
	rdk_logger_init("/etc/debug.ini");

	//create instance
	smTnInstance = SmartThumbnail::getInstance();
	if (!smTnInstance) {
    	    RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Error creating Smart thumbnail instance.\n", __FILE__, __LINE__);
	    return STH_ERROR;
	}

	//initialize smart thumbnail
	status = smTnInstance-> init();
	if (STH_ERROR == status) {
    	    RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Error creating Smart thumbnail instance.\n", __FILE__, __LINE__);
	    return STH_ERROR;
	}

	RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Notify xvision and cvr daemon.\n", __FILE__, __LINE__);

	//Notify start status 
	smTnInstance-> notify("start");

	smTnInstance-> receiveRtmessage();
		
	//notify exit status
	smTnInstance -> notify("stop");

	//destroy smartThumbnail instance
	if(smTnInstance) {
		RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Deleting smart thumnail instance!!!\n", __FILE__, __LINE__);
		smTnInstance-> destroy();
	}

	return 0;
}

