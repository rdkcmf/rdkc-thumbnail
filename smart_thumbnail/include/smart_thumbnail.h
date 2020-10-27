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
#ifndef __SMART_THUMBNAIL_H__
#define __SMART_THUMBNAIL_H__

#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <sys/stat.h>
#include <fcntl.h>
#include <vector>
#include <iterator>

#ifdef LEGACY_CFG_MGR
#include "dev_config.h"
#else
#include "polling_config.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif
#ifdef USE_MFRLIB
#include "mfrApi.h"
#endif
#ifdef __cplusplus
}
#endif

#include "rdk_debug.h"
#include "rtConnection.h"
#include "rtLog.h"
#include "rtMessage.h"
#include "HttpClient.h"

#ifdef _HAS_XSTREAM_
#include "xStreamerConsumer.h"
#else
#include "RdkCVideoCapturer.h"
#include "RdkCPluginFactory.h"
#endif
#ifdef _HAS_DING_
#include "DingNotification.h"
#endif

#include "opencv2/opencv.hpp"

#define CACHE_SMARTTHUMBNAIL "/tmp/cache_smart_thumbnail.txt"

#define FW_NAME_MAX_LENGTH 512
#define CONFIG_STRING_MAX (256)
#define YUV_HRES_BUFFER_ID		0
#define YUV_HRES_FRAME_WIDTH		1280
#define YUV_HRES_FRAME_HEIGHT		720

#ifdef XCAM2
#define STN_HRES_BUFFER_ID              2
#elif XHB1
#define STN_HRES_BUFFER_ID              2
#else
#define STN_HRES_BUFFER_ID              0
#endif

//actual width and height of smart thumbnail to be uploaded
#define STN_FRAME_WIDTH 		212
#define STN_FRAME_HEIGHT		119

#define STN_DEFAULT_DNS_CACHE_TIMEOUT	60

//default width and height of smart thumbnail
#define STN_DEFAULT_WIDTH		640
#define STN_DEFAULT_HEIGHT		480

#define STN_TIMESTAMP_TAG		"timestamp"
#define STN_UPLOAD_TIME_INTERVAL	30

#define STN_PATH 			"/tmp"
//#define STN_PATH			"."
#define STN_UPLOAD_SEND_LEN		2048
#define STN_COMPRESSION_SCALE		40
#define STN_DATA_MAX_SIZE		1024*1024 	//1 MB
#define STN_TSTAMP_SIZE			50
#define STN_DEFAULT_EVT_QUIET_TIME	30
#define STN_MAX_PAYLOAD_COUNT		4

#define STN_TRUE			"true"
#define STN_FALSE			"false"

#define RTMSG_DYNAMIC_LOG_REQ_RES_TOPIC   "RDKC.ENABLE_DYNAMIC_LOG"

#define STN_MAX( a, b ) ( ( a > b) ? a : b )
#define STN_MIN( a, b ) ( ( a < b) ? a : b )

#define UPPER_LIMIT_BLOB_BB     5
#define MAX_BLOB_SIZE           4
#define BLOB_BB_MAX_LEN         256
#define INVALID_BBOX_ORD        (-1)

typedef enum {
    STH_ERROR = -1,
    STH_SUCCESS,
    STH_NO_PAYLOAD
}STH_STATUS;

typedef enum {
    CVR_CLIP_GEN_START = 0,
    CVR_CLIP_GEN_END,
    CVR_CLIP_GEN_UNKNOWN
}CVR_CLIP_GEN_STATUS;

typedef enum {
    CVR_UPLOAD_OK = 0,
    CVR_UPLOAD_FAIL,
    CVR_UPLOAD_CONNECT_ERR,
    CVR_UPLOAD_SEND_ERR,
    CVR_UPLOAD_RESPONSE_ERROR,
    CVR_UPLOAD_TIMEOUT,
    CVR_UPLOAD_MAX,
    CVR_UPLOAD_CURL_ERR
}CVR_UPLOAD_STATUS;

typedef struct {
    char fname[64];
    uint64_t tstamp;
#ifdef _HAS_DING_
    uint64_t dingtstamp;
#endif
}STHPayload;

typedef struct {
    uint32_t boundingBoxXOrd;
    uint32_t boundingBoxYOrd;
    uint32_t boundingBoxWidth;
    uint32_t boundingBoxHeight;
    uint64_t currTime;
    cv::Mat  maxBboxObjYUVFrame;
}objFrameData;

typedef struct _tBoundingBox
{
    int32_t boundingBoxXOrd;
    int32_t boundingBoxYOrd;
    int32_t boundingBoxWidth;
    int32_t boundingBoxHeight;
}BoundingBox;

class SmartThumbnail
{
    public:
	static SmartThumbnail* getInstance();
	//Initialize the buffers and starts msg monitoring, upload thread.
	STH_STATUS init(char* mac,bool isCVREnabled);
	//get upload status
	bool getUploadStatus();
	//set upload status
	STH_STATUS setUploadStatus(bool status);
	//Upload smart thumbnail data
	void uploadPayload();
	//notify start or end of smart thumbnail process
	STH_STATUS notify(const char* status);
	//call Smart thumbnail destructor and deallocates dynamic allocated memory.
	STH_STATUS destroy();
	//Thread routine to receive data
	STH_STATUS receiveRtmessage();

	static void sigHandler(int signum);

    private:
	SmartThumbnail();
	~SmartThumbnail();
	STH_STATUS saveSTN();
	STH_STATUS addSTN();
	STH_STATUS delSTN(char* uploadFname);
	void printSTNList();
	STH_STATUS createPayload(char* uploadFname);
	STH_STATUS getTnUploadConf();
	STH_STATUS getEventConf();
	//sets the camera firmware version.
	int setCameraImageName(char* out);
	//sets the camera firmware version.
	int setModelName(char* modelName);
	//sets the camera's mac address.
	int setMacAddress(char* macAddress);
	// registers Callback for rtmessage
	STH_STATUS rtMsgInit();
	//reset smart thumbnail resources
	void resetResources();
	//Resize the cropped area keeping the aspect ratio.
	STH_STATUS resizeAspect(cv::Mat im, int w, int h, cv::Mat& im_resized);
	cv::Point2f getActualCentroid(cv::Rect boundRect);
	cv::Point2f alignCentroid(cv::Point2f orgCenter, cv::Mat origFrame, cv::Size cropSize);
	cv::Size getCropSize(cv::Rect boundRect,double w,double h);
	cv::Rect getRelativeBoundingBox(cv::Rect boundRect, cv::Size cropSize, cv::Point2f allignedCenter);
	void stringifyEventDateTime(char* strEvtDateTime , size_t evtdatetimeSize, time_t evtDateTime);
	void resetObjFrameData();
	//Updates object frame
	void  updateObjFrameData(int32_t boundingBoxXOrd,int32_t boundingBoxYOrd,int32_t boundingBoxWidth,int32_t boundingBoxHeight,uint64_t currTime);
        //Updates object Boxes
        void  updateObjectBoxs(BoundingBox *objectBox, int32_t index);
	int retryAtExpRate();

	//Read the High resolution YUV frame.
	static void onMsgCaptureFrame(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure);
	//Generate the RGB object detection frame.
	static void onMsgProcessFrame(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure);
	static void onMsgCvr(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure);
	static void onMsgCvrUpload(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure);
	static void onMsgRefreshTnUpload(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure);
	static void dynLogOnMessage(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure);
	//Routine to upload STN Payload
	static void uploadSTN();

        static volatile bool termFlag;
        static volatile bool tnUploadConfRefreshed;

	static SmartThumbnail* smartThInst;

#ifdef _HAS_XSTREAM_
    XStreamerConsumer* consumer;
    frameInfoYUV  *frameInfo;
#ifndef _DIRECT_FRAME_READ_
    curlInfo frameHandler;
#endif
#else
	RdkCVideoCapturer* recorder;
	RdkCPluginFactory* pluginFactory;
	RDKC_PLUGIN_YUVInfo* hres_frame_info;
#endif
	//static bool hres_yuvDataMemoryAllocationDone;
#ifdef _HAS_DING_
	DingNotification* m_ding;
	bool m_dingNotif;
	uint64_t m_dingTime;
	static void onDingNotification(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure);
#endif
	bool logMotionEvent;

	bool uploadReady;
	//static bool rtmessageSTHThreadExit;
	//static bool uploadSTHThreadExit;
	bool isPayloadAvailable;
	std::mutex stnMutex;
	std::mutex uploadMutex;
	std::condition_variable cv;
	rtConnection connectionRecv;
	rtConnection connectionSend;
	rtError err;
	int maxBboxArea;
	objFrameData ofData;
	//unsigned char*  hres_yuvData;
	int hres_y_size;
	int hres_uv_size;
	int hres_y_height;
	int hres_y_width;
	bool isHresFrameReady;
	std::vector<STHPayload> STNList;
	bool cvrClipGenStarted;
	bool cvrEnabled;
	HttpClient* httpClient;
	int dnsCacheTimeout;
	STHPayload currSTN;
	STHPayload payload;
	uint64_t motion_time;
	int32_t event_quiet_time;

	char smtTnUploadURL[CONFIG_STRING_MAX];
	char smtTnAuthCode[AUTH_TOKEN_MAX];

	int sTnHeight;
	int sTnWidth;
	char uploadFname[CONFIG_STRING_MAX];
	char currSTNFname[CONFIG_STRING_MAX];
	char modelName[CONFIG_STRING_MAX];
    	char macAddress[CONFIG_STRING_MAX];
    	char firmwareName[FW_NAME_MAX_LENGTH];
	static int waitingInterval;
	cv::Rect relativeBBox;
        BoundingBox objectBoxs [UPPER_LIMIT_BLOB_BB];
};

struct SmarttnMetadata_thumb
{

    /*SmarttnMetadata constructor*/
    SmarttnMetadata_thumb();
    /*update sm details with rtMessage m*/
    static void from_rtMessage(SmarttnMetadata_thumb *smInfo, const rtMessage m);

    char const *strFramePTS;
    int32_t event_type;
    double motionScore;
    BoundingBox unionBox;
    BoundingBox objectBoxs [UPPER_LIMIT_BLOB_BB];
    char const *s_curr_time;
};

#endif //__SMART_THUMBNAIL_H__
