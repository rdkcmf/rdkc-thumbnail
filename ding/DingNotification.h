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
#ifndef __DINGNOTIFICATION_H__
#define __DINGNOTIFICATION_H__

#include <mutex>
#include <thread>
#include <condition_variable>
#include <sys/stat.h>
#include <fcntl.h>
#include "polling_config.h"
#include "HttpClient.h"
#include "rdk_debug.h"
#define DEFAULT_DNS_CACHE_TIMEOUT	60
#define MAX_RETRY_COUNT			3
#define UPLOAD_TIME_INTERVAL            30
#define UPLOAD_DATA_LEN                 2048
#define FW_MAX_LENGTH                   512
#define DEFAULT_QUITE_TIME              4
#define COMPRESSION_SCALE               40
#define WIDTH                           848
#define HEIGHT                          480
#define SNAPSHOT_FILE                   "/opt/tn_snapshot.jpg"

class DingNotification
{
	public:
		static DingNotification* getInstance();
		DingNotification();
		~DingNotification();
		void init(char* mac,char* modelName,char* firmware);
		bool waitForDing();
		bool signalDing(bool status,uint64_t dingTime);
                bool getDingTNUploadStatus() { return m_DingTNuploadStatus ; }
		int  getQuiteTime();
                void uploadDingThumbnail();
	private:
		static DingNotification* m_Instance;
		static volatile bool m_termFlag;
                static volatile bool m_confRefreshed;
                static int waitInterval;
		static bool monitorDingNotification();
                static void uploadDingSnapShot();

 		HttpClient* m_httpClient;
		std::mutex m_dingMutex;
		std::condition_variable m_cv;
		int m_dnsCacheTimeout;
                char m_dingNotifUploadURL[CONFIG_STRING_MAX];
                char m_dingNotifAuthCode[AUTH_TOKEN_MAX];
		char m_snapShotUploadURL[CONFIG_STRING_MAX];
		char m_snapShotAuthCode[AUTH_TOKEN_MAX];
                char m_modelName[CONFIG_STRING_MAX];
                char m_macAddress[CONFIG_STRING_MAX];
                char m_firmwareName[FW_MAX_LENGTH];
		bool m_uploadReady;
                bool m_DingTNuploadStatus;
		int m_quiteTime;
		int m_snapShotHeight;
		int m_snapShotWidth;
		uint64_t m_dingTime;
		int retryAtExpRate();
		int getDingConf();
		int getTnUploadConf();
		void sendDingNotification();
		void uploadSnapShot();
		void stringifyDateTime(char* strEvtDateTime , size_t evtdatetimeSize, time_t evtDateTime);

};
#endif
