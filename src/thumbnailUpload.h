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
#ifndef _THUMBNAIL_UPLOAD_H_
#define _THUMBNAIL_UPLOAD_H_

#include <stdio.h>
#include <string.h>
#include <vector>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <fileUtils.h>
#include <string>
#include "rdk_debug.h"
#include "HttpClient.h"
#include "RFCCommon.h"
#ifdef __cplusplus
extern "C" {
#endif
#include "dev_config.h"
#include "polling_config.h"
#include "sysUtils.h"

#if !defined ( OSI ) && !defined ( THUMBNAIL_PLATFORM_RPI )
#include "cgi_image.h"
#include "SYS_log.h"
#endif

#ifdef USE_MFRLIB
#include "mfrApi.h"
#endif

#ifdef __cplusplus
}
#endif


#ifdef _HAS_DING_
#include "DingNotification.h"
#endif

/* RtMessage */
#include "rtLog.h"
#include "rtConnection.h"
#include "rtMessage.h"
#define WEBPA_ADDRESS "tcp://127.0.0.1:10001"
#define SMA_FACTOR 10

#define RDKC_FAILURE                           -1
#define RDKC_SUCCESS                    	0

#define THUMBNAIL_UPLOAD_PARAM_MAX_LENGTH      		512
#define THUMBNAIL_UPLOAD_AUTH_MAX_LENGTH       		1024
#define THUMBNAIL_UPLOAD_SEND_LEN              		2048
#define THUMBNAIL_UPLOAD_MAC_STRING_LEN        		12
#define SIZE     			       		50

#define DEFAULT_THUMBNAIL_UPLOAD_ENABLE 		true
#define DEFAULT_THUMBNAIL_UPLOAD_PASSIVE_INTERVAL	300
#define DEFAULT_THUMBNAIL_UPLOAD_ACTIVE_INTERVAL	30
#define DEFAULT_THUMBNAIL_UPLOAD_ACTIVE_DURATION	120

//Need to change the value once provided with actual certs, using test certs for now.
#define THUMBNAIL_UPLOAD_CA_CERT_FILE    		"/etc/ssl/certs/ca-chain.cert.pem"
#define THUMBNAIL_UPLOAD_CERT_FILE       		"/etc/ssl/certs/testclient1031.cert.pem"
#define THUMBNAIL_UPLOAD_KEY_FILE       		"/etc/ssl/certs/testclient1031.key.pem"

#define LOCK_FILENAME_TNU               		"/tmp/thumbnail_upload.lock"

#define FW_NAME_MAX_LENGTH      			256
#define VER_NUM_MAX_LENGTH      			128

//SUPPORT_IMAGETOOLS
#define COMPRESSION_SCALE				40
#define TN_OP_WIDTH					640
#define TN_OP_HEIGHT					360
#define SNAPSHOT_FILE					"/opt/tn_snapshot.jpg"
#define PASSIVE_TN_THRESHHOLD_COUNT 			6
#define ACTIVE_TN_THRESHHOLD_COUNT 			12
#define MAX_UPLOAD_RETRY         			2
#define MAX_RETRY_SLEEP					10
#define MAXSIZE						100

#define ASPECTRATIO_FILE                                "/opt/usr_config/aspect_ratio.conf"
#define TN_OP_WIDTH_4_3                                 640
#define TN_OP_HEIGHT_4_3                                480

#define RTMSG_DYNAMIC_LOG_REQ_RES_TOPIC			"RDKC.ENABLE_DYNAMIC_LOG"
#define RTMSG_THUMBNAIL_TOPIC				"RDKC.THUMBNAIL"

int getCameraImageName(char *out);
int getCameraVersionNum(char *out);

const char gcpThumbnailSnapshotPath[]							= "/opt/tn_snapshot.jpg";
const char gcpSnapshooterOpt[]								= " -r 13 -q 3";
const char DEFAULT_THUMBNAIL_UPLOAD_URL[THUMBNAIL_UPLOAD_PARAM_MAX_LENGTH]    		= "https://livecache-stg-cvr-nc.sys.comcast.net/camera";
const char DEFAULT_THUMBNAIL_UPLOAD_AUTH_TOKEN[THUMBNAIL_UPLOAD_AUTH_MAX_LENGTH]    	= "";
const char DEFAULT_THUMBNAIL_UPLOAD_RESOLUTION[SIZE]    				= "640x360";

enum TN_UPLOAD_STATUS
{
        TN_UPLOAD_OK = 0,
        TN_UPLOAD_FAIL,
        TN_UPLOAD_CONNECT_ERR,
        TN_UPLOAD_SEND_ERR,
        TN_UPLOAD_RESPONSE_ERROR,
        TN_UPLOAD_TIMEOUT,
        TN_UPLOAD_MAX
};

class ThumbnailUpload {

public:
	ThumbnailUpload();
	~ThumbnailUpload();
        void rtConnection_init();
	static ThumbnailUpload *getTNUploadInstance();
	static void *doTNUpload();
	int controlTNUploadProcess(char *process);
	bool getTNUploadProvAttr();
	static void TNURegisterSignalHandler(void);
	void TNUExit();
        static void* rtMessage_Receive(void * arg);
        void rtConnection_destroy();
private:
	void getTNUploadAttr();
	bool m_uploadReady;
	int  postFileToTNUploadServer(char * fileName, int fileLen, char* serverUrl, long * responseCode);
	int  uploadThumbnailImage();
	int  checkTNUploadfilelock(char *fname);
	int  updateActiveUploadDuration();
	bool waitFor(int quiteInterVal);
        int  setUploadStatus(bool status);
	void stringifyEventDateTime(char* strEvtDateTime , size_t evtdatetimeSize, time_t evtDateTime);
	//Callback function for topics on thumbnail
	static void onMessage(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure);
	//Callback function for topics on dynamic Logging
	static void dynLogOnMessage(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure);
	static void setActiveInterval(void);
	static void TNUSignalHandler(int s);
	void releaseResources();
	static ThumbnailUpload* thumbnailUpload;
	HttpClient *http_client;
        char tn_upload_server_url[THUMBNAIL_UPLOAD_PARAM_MAX_LENGTH];
        char tn_upload_auth_token[THUMBNAIL_UPLOAD_AUTH_MAX_LENGTH];
	int json_prov_tn_upload_enabled;
	static int tn_upload_interval;
	static bool tn_upload_enable;
	livecache_provision_info_t *liveCacheConf;
	static volatile sig_atomic_t term_flag;
	char* tn_upload_file_name;
	static bool isActiveInterval;
	static int activeUploadDuration;
	static char fw_name[FW_NAME_MAX_LENGTH];
	static char ver_num[VER_NUM_MAX_LENGTH];
	static char mac_string[THUMBNAIL_UPLOAD_MAC_STRING_LEN + 1];
	static char modelName[THUMBNAIL_UPLOAD_MAC_STRING_LEN + 1];
	static rtConnection con;
	static unsigned int activeModeUploadCounter;
	char cmd[MAXSIZE];
	static int uploadRetryCount;
	int m_count;
	curl_off_t m_avgUploadSpeed;
	std::vector<curl_off_t> m_smVector;
	std::condition_variable m_cv;
        std::mutex m_uploadMutex;
#ifdef _HAS_DING_
        DingNotification* m_ding;
        bool m_dingNotif;
        uint64_t m_dingTime;
        static void onDingNotification(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure);
#endif

};

#endif
