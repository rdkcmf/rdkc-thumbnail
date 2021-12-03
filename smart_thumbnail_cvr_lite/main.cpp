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

#if 0 
/** @description: Checks if the feature is enabled via RFC
 *  @param[in] rfc_feature_fname: RFC feature filename
 *  @param[in] feature_name: RFC parameter name
 *  @return: bool
 */
bool checkEnabledRFCFeature(char* rfcFeatureFname, char* featureName)
{
    /* set cvr audio through RFC files */
    char value[10] = {0};

    if((NULL == rfcFeatureFname) ||
       (NULL == featureName)) {
        return STN_FALSE;
    }

    /* Check if RFC configuration file exists */
    if(0 == IsRFCFileAvailable(rfcFeatureFname)) {
        /* Get the value from RFC file */
        if( STH_SUCCESS == GetValueFromRFCFile(rfcFeatureFname, featureName, value) ) {
            if( strcmp(value, STN_TRUE) == 0) {
                RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): %s is enabled via RFC.\n",__FILE__, __LINE__, featureName);
                return true;
            } else {
                RDK_LOG( RDK_LOG_INFO,"LOG.RDKSMARTTHUMBNAIL","%s(%d): %s is disabled via RFC.\n",__FILE__, __LINE__, featureName);
                return false;
            }
        }
        /* If RFC file is not present, disable the feature */
    } else {
        RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): rfc feature file %s is not present.\n",__FILE__, __LINE__, rfcFeatureFname);
        return false;
    }
}
#endif

int main(int argc, char** argv)
{
	SmartThumbnail *smTnInstance = NULL;
	STH_STATUS status = STH_SUCCESS;
        time_t remainingTime = 0;
	char *param = NULL;
    	const char* debugConfigFile = "/etc/debug.ini";
    	int  cvrEnabled = 0;
	struct timespec currTime;
	struct timespec startTime;
	struct timespec prevTime;
	memset(&startTime, 0, sizeof(startTime));
	memset(&currTime, 0, sizeof(currTime));
	memset(&prevTime, 0, sizeof(prevTime));
	int itr =0;

	/* Registering callback function for Breakpadwrap Function */
#ifdef BREAKPAD
	sleep(1);
	BreakPadWrapExceptionHandler eh;
	eh = newBreakPadWrapExceptionHandler();
#endif

    	while (itr < argc)
    	{
                if(strcmp(argv[itr],"--debugconfig")==0)
                {
                        itr++;
                        if (itr < argc)
                        {
                                debugConfigFile = argv[itr];
                        }
                        else
                        {
                                break;
                        }
                }
                if(strcmp(argv[itr],"--hw-mac")==0)
                {
                        itr++;
                        if (itr < argc)
                        {
                                param = argv[itr];
                        }
                        else
                        {
                                break;
                        }
                }

                if(strcmp(argv[itr],"--cvrEnabled")==0)
                {
                        itr++;

                        if (itr < argc)
                        {
                                cvrEnabled = atoi(argv[itr]);
                        }
                        else
                        {
                                break;
                        }
                }
                itr++;
    	}

	//initialize rdklogger
	rdk_logger_init(debugConfigFile);
	//initialize RFC
	RFCConfigInit();

	//create instance
	smTnInstance = SmartThumbnail::getInstance();
	if (!smTnInstance) {
    	    RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Error creating Smart thumbnail instance.\n", __FILE__, __LINE__);
	    return STH_ERROR;
	}

	//initialize smart thumbnail
	status = smTnInstance-> init(param,cvrEnabled);
	if (STH_ERROR == status) {
    	    RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Error creating Smart thumbnail instance.\n", __FILE__, __LINE__);
	    return STH_ERROR;
	}

	RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Notify xvision and cvr daemon.\n", __FILE__, __LINE__);
		
        //Initially sleep for upload interval time, to allow smart thumbnail to be generated.
	RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Putting the Smart Thumnail Upload to sleep for %d seconds.\n", __FILE__, __LINE__, STN_UPLOAD_TIME_INTERVAL);
#ifdef ENABLE_TEST_HARNESS
        smTnInstance -> waitForClipEnd();
#else
        sleep(STN_UPLOAD_TIME_INTERVAL);
#endif
	RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Smart Thumbnail Upload is ready.\n", __FILE__, __LINE__);

	while (true) {

		//Notify start status 
		smTnInstance-> notify("start");

		//clock the start time
		memset(&startTime, 0, sizeof(startTime));
	        clock_gettime(CLOCK_REALTIME, &startTime);
#if 0
		//exit app if smt Thumbnail is disabled via RFC
		if(!checkEnabledRFCFeature(RFC_SMART_TN_UPLOAD, SMART_TN_UPLOAD)) {
			RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Exiting smart thumbnail, Disable via RFC!!!\n", __FILE__, __LINE__);
			break;
		}
#endif		
		//create payload
		status = smTnInstance->createPayload();
		if ( (STH_NO_PAYLOAD == status) ||
	    		(STH_ERROR == status) ) {
				RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Unable to create payload for smart thumnail, hence skipping!!!\n", __FILE__, __LINE__);

				//clock current time
				memset(&currTime, 0, sizeof(currTime));
	       			clock_gettime(CLOCK_REALTIME, &currTime);
				RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Time spent for payload creation %d seconds!!!\n", __FILE__, __LINE__, (currTime.tv_sec - startTime.tv_sec));

				// sleep maximum of smart thumbnail time interval(~15 seconds)
				if( (currTime.tv_sec - startTime.tv_sec) >= STN_UPLOAD_TIME_INTERVAL ) {
					RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Exceed upload time interval!!!\n", __FILE__, __LINE__);
					continue;
				} else {
					remainingTime =  STN_UPLOAD_TIME_INTERVAL - (currTime.tv_sec - startTime.tv_sec);
					RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Sleep for remaining %d seconds!!!\n", __FILE__, __LINE__, remainingTime);
#ifdef ENABLE_TEST_HARNESS
                                        smTnInstance -> waitForClipEnd();
#else
					sleep (remainingTime);
#endif
					continue;
				}
                }

		RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Created payload for smart thumnail successfully.. Going to upload now!!!\n", __FILE__, __LINE__);

		// clock the current time
		memset(&currTime, 0, sizeof(currTime));
	       	clock_gettime(CLOCK_REALTIME, &currTime);
		RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): currTime.tv_sec %d startTime.tv_sec %d (currTime.tv_sec - startTime.tv_sec)!!!\n", __FILE__, __LINE__, currTime.tv_sec, startTime.tv_sec, (currTime.tv_sec - startTime.tv_sec));

	        smTnInstance->uploadPayload(STN_UPLOAD_TIME_INTERVAL - (currTime.tv_sec - startTime.tv_sec));

		// clock the current time
		memset(&currTime, 0, sizeof(currTime));
	       	clock_gettime(CLOCK_REALTIME, &currTime);

		RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Time spent to upload smart thumbnail %d seconds!!!\n", __FILE__, __LINE__, (currTime.tv_sec - startTime.tv_sec));
		if( (currTime.tv_sec - startTime.tv_sec) >= STN_UPLOAD_TIME_INTERVAL ) {
			RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Exceed upload time interval!!!\n", __FILE__, __LINE__);
			continue;
		}
		else {
			//calculate the time needed to sleep for next interval.
			remainingTime =  STN_UPLOAD_TIME_INTERVAL - (currTime.tv_sec - startTime.tv_sec);
			RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Sleep for remaining %d seconds!!!\n", __FILE__, __LINE__, remainingTime);
#ifdef ENABLE_TEST_HARNESS
                        smTnInstance -> waitForClipEnd();
#else
			sleep (remainingTime);
#endif
		}
	}

	//notify exit status
	smTnInstance -> notify("stop");

	//destroy smartThumbnail instance
	if(smTnInstance) {
		RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Deleting smart thumnail instance!!!\n", __FILE__, __LINE__);
		smTnInstance-> destroy();
	}
	RFCRelease();
	return 0;
}

