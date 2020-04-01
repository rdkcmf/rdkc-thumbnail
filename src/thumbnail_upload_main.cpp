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
#include <pthread.h>
#include "thumbnailUpload.h"

#define MAXRETRY 5

int main(int argc, char *argv[])
{
        /* ENABLING RDK LOGGER */
        rdk_logger_init("/etc/debug.ini");
        config_init();
	ThumbnailUpload *thumbnail_upload = ThumbnailUpload::getTNUploadInstance();

        thumbnail_upload->rtConnection_init();

        pthread_t thumbnailUploadThread;
        if( RDKC_SUCCESS != pthread_create(&thumbnailUploadThread, NULL, &ThumbnailUpload::rtMessage_Receive, thumbnail_upload ))
        {
          RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): can't create thread. \n", __FILE__, __LINE__ );
            return RDKC_FAILURE;
        }

        pthread_setname_np(thumbnailUploadThread,"thumbnail_upload");
        RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Created thumbnail upload thread. \n", __FILE__, __LINE__ );

	/* Register signal handler */
        if( NULL != thumbnail_upload) {
		thumbnail_upload->TNURegisterSignalHandler();
	}

        int retry = 0;
        if( NULL != thumbnail_upload) {
		do {
			if(!thumbnail_upload->getTNUploadProvAttr()) {
				RDK_LOG(RDK_LOG_INFO ,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Failed to read json thumbnail configuration retrying...\n", __FILE__, __LINE__);
                                retry++;
                                if ( retry > MAXRETRY ) {
                                        RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d) : FATAL : Max retry reached in  getTNUploadProvAttr exit process %d\n", __FILE__, __LINE__);
                                        exit(1);
                                }
				sleep(2);
			}
			else {
				RDK_LOG(RDK_LOG_INFO ,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): success in reading thumbnail configuration from livecache.conf\n", __FILE__, __LINE__);
                                break;
			}
		} while ( retry <= MAXRETRY) ;
	}

	//Do Thumbnail Upload
        if( NULL != thumbnail_upload) {
		RDK_LOG(RDK_LOG_INFO ,"LOG.RDK.THUMBNAILUPLOAD","%s(%d):Starting Thumbnail Upload.\n", __FILE__, __LINE__);
		thumbnail_upload->doTNUpload();
	}

	//Exiting Thumbnail Upload
        if( NULL != thumbnail_upload) {
                thumbnail_upload->rtConnection_destroy();
                thumbnail_upload->TNUExit();
                //pthread_exit(0);
		pthread_kill(thumbnailUploadThread, 0);
		if( NULL != thumbnail_upload) {
			delete thumbnail_upload;
			thumbnail_upload = NULL;
		}
		RDK_LOG(RDK_LOG_INFO ,"LOG.RDK.THUMBNAILUPLOAD","%s(%d):Stopping Thumbnail Upload.\n", __FILE__, __LINE__);
	}

	return 0;
}
