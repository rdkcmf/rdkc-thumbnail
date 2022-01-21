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
#ifdef BREAKPAD
#include "breakpadwrap.h"
#endif

#ifdef _OBJ_DETECTION_
SmartThumbnail *smTnInstance = NULL;
#endif

int main(int argc, char** argv)
{
#ifndef _OBJ_DETECTION_
	SmartThumbnail *smTnInstance = NULL;
#endif
	STH_STATUS status = STH_SUCCESS;
	time_t remainingTime = 0;
	char *param = NULL;
	const char* debugConfigFile = "/etc/debug.ini";
	int  cvrEnabled = 0;
	int  stnondelayType = 0;
	int  stnondelayTime = 60;
	int  isDetectionEnabled = 0;

	struct timespec currTime;
	struct timespec startTime;
	memset(&startTime, 0, sizeof(startTime));
	memset(&currTime, 0, sizeof(currTime));

	signal(SIGTERM, SmartThumbnail::sigHandler);
	signal(SIGINT, SmartThumbnail::sigHandler);

	/* Registering callback function for Breakpadwrap Function */
#ifdef BREAKPAD
	sleep(1);
	BreakPadWrapExceptionHandler eh;
	eh = newBreakPadWrapExceptionHandler();
#endif

	int itr =0;

	// Parse command line arguments
	while (itr < argc) {
		if(strcmp(argv[itr],"--debugconfig")==0) {
			itr++;
			if (itr < argc) {
				debugConfigFile = argv[itr];
			}
			else {
				break;
			}
		}

		if(strcmp(argv[itr],"--hw-mac")==0) {
			itr++;
			if (itr < argc) {
				param = argv[itr];
			}
			else {
				break;
			}
		}

		if(strcmp(argv[itr],"--cvrEnabled")==0) {
			itr++;

			if (itr < argc) {
				cvrEnabled = atoi(argv[itr]);
			}
			else {
				break;
			}
		}

		if(strcmp(argv[itr],"--stnondelaytype")==0) {
			itr++;

			if (itr < argc) {
				stnondelayType = atoi(argv[itr]);
			}
			else {
				break;
			}
		}

		if(strcmp(argv[itr],"--stnondelaytime")==0) {
			itr++;

			if (itr < argc) {
				stnondelayTime = atoi(argv[itr]);
			}
			else {
				break;
			}
		}

		if(strcmp(argv[itr],"--detectionEnabled")==0) {
			itr++;

			if (itr < argc) {
				isDetectionEnabled = atoi(argv[itr]);
			}
			else {
				break;
			}
		}

		itr++;
	}

	// Initialize rdklogger
	rdk_logger_init(debugConfigFile);

	// Create instance
	smTnInstance = SmartThumbnail::getInstance();
	if (!smTnInstance) {
		RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Error creating Smart thumbnail instance.\n", __FILE__, __LINE__);
		return STH_ERROR;
	}

	// Initialize smart thumbnail/event notification
	status = smTnInstance->init(param, cvrEnabled, stnondelayType, stnondelayTime, isDetectionEnabled);
	if (STH_ERROR == status) {
		RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Error creating Smart thumbnail instance.\n", __FILE__, __LINE__);
		return STH_ERROR;
	}

	RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Notify xvision and cvr daemon.\n", __FILE__, __LINE__);

	// Notify start status
	smTnInstance->notify("start");

#ifndef ENABLE_TEST_HARNESS
	if(!cvrEnabled) {

		// Run clip event simulator routine
		RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Creating Clip event simulator thread.\n", __FUNCTION__, __LINE__);
		std::thread eventSimulatorThread(SmartThumbnail::generateCVREvents);
		eventSimulatorThread.detach();

	}
#endif

	smTnInstance->receiveRtmessage();

	// Notify exit status
	smTnInstance->notify("stop");

	// Destroy smartThumbnail instance
	if(smTnInstance) {
		RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Deleting smart thumnail instance!!!\n", __FILE__, __LINE__);
		smTnInstance->destroy();
	}

	return 0;
}

