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
#include "RdkCVideoCapturer.h"
#include "RdkCPluginFactory.h"
#include "RFCCommon.h"
#include "opencv2/opencv.hpp"


#define FW_NAME_MAX_LENGTH 512
#define CONFIG_STRING_MAX (256)
#define YUV_HRES_BUFFER_ID		0
#define YUV_HRES_FRAME_WIDTH		1280
#define YUV_HRES_FRAME_HEIGHT		720

#ifndef XCAM2
#define STN_HRES_BUFFER_ID		0
#else
#define STN_HRES_BUFFER_ID		2
#endif

//actual width and height of smart thumbnail to be uploaded
#define STN_FRAME_WIDTH 		212
#define STN_FRAME_HEIGHT		119

#define STN_DEFAULT_DNS_CACHE_TIMEOUT	60

//default width and height of smart thumbnail
#define STN_DEFAULT_WIDTH		640
#define STN_DEFAULT_HEIGHT		480

#define STN_TIMESTAMP_TAG		"timestamp"
#define STN_UPLOAD_TIME_INTERVAL	15

#define STN_PATH 			"/tmp"
#define STN_UPLOAD_SEND_LEN		2048
#define STN_MAX_RETRY_COUNT             3
#define STN_COMPRESSION_SCALE		40
#define STN_DATA_MAX_SIZE		1024*1024 	//1 MB
#define STN_TSTAMP_SIZE			50
#define STN_DEFAULT_EVT_QUIET_TIME	30

#define STN_TRUE			"true"
#define STN_FALSE			"false"

#define RTMSG_DYNAMIC_LOG_REQ_RES_TOPIC   "RDKC.ENABLE_DYNAMIC_LOG"

#define STN_MAX( a, b ) ( ( a > b) ? a : b )
#define STN_MIN( a, b ) ( ( a < b) ? a : b )

typedef enum {
    STH_ERROR = -1,
    STH_SUCCESS,
    STH_NO_PAYLOAD
}STH_STATUS;

typedef struct {
    cv::Mat objFrame;
    uint64_t tstamp;
}STHPayload;

typedef struct {
    uint32_t boundingBoxXOrd;
    uint32_t boundingBoxYOrd;
    uint32_t boundingBoxWidth;
    uint32_t boundingBoxHeight;
    cv::Mat  maxBboxObjYUVFrame;
    uint64_t currTime;
}objFrameData;

class SmartThumbnail
{
    public:
	static SmartThumbnail* getInstance();
	//Initialize the buffers and starts msg monitoring, upload thread.
	STH_STATUS init();
	//Pushes the data to the upload queue at the end of interval.
	STH_STATUS createPayload();
	//Upload smart thumbnail data 
	void uploadPayload(time_t timeLeft);
	//notify start or end of smart thumbnail process
	STH_STATUS notify(const char* status);
	//call Smart thumbnail destructor and deallocates dynamic allocated memory.
	STH_STATUS destroy();

    private:
	SmartThumbnail();
	~SmartThumbnail();
	STH_STATUS getTnUploadConf();
	STH_STATUS getEventConf();
	//sets the camera firmware version.
	int setCameraImageName(char* out);
	//sets the camera firmware version.
	int setModelName();
	//sets the camera's mac address.
	int setMacAddress();
	// registers Callback for rtmessage
	STH_STATUS rtMsgInit();
	//Read the High resolution YUV frame.
	static void onMsgCaptureFrame(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure);
	//Generate the RGB object detection frame.
	static void onMsgProcessFrame(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure);

	//Updates object frame
	static void  updateObjFrameData(int32_t boundingBoxXOrd,int32_t boundingBoxYOrd,int32_t boundingBoxWidth,int32_t boundingBoxHeight,uint64_t currTime);

	//Resize the cropped area keeping the aspect ratio.
	STH_STATUS resizeAspect(cv::Mat im, int w, int h, cv::Mat& im_resized);

	//Thread routine to receive data
	static void receiveRtmessage();
	//Callback function for dynamic logging. 
	static void dynLogOnMessage(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure);

	cv::Point2f getActualCentroid(cv::Rect boundRect);
	cv::Point2f alignCentroid(cv::Point2f orgCenter, cv::Mat origFrame, cv::Size cropSize);
	cv::Size getCropSize(cv::Rect boundRect,double w,double h);
	cv::Rect getRelativeBoundingBox(cv::Rect boundRect, cv::Size cropSize, cv::Point2f allignedCenter);

	static SmartThumbnail* smartThInst;
	RdkCPluginFactory* pluginFactory;
	int g_hres_buf_id;
	RdkCVideoCapturer* recorder;
	RDKC_PLUGIN_YUVInfo* hres_frame_info;
	bool hres_yuvDataMemoryAllocationDone;

	std::thread rtMessageReceive;
	std::thread uploadThread;
	bool rtmessageSTHThreadExit;
	bool isPayloadAvailable;
	std::mutex QMutex;
	rtConnection connectionRecv;
	rtConnection connectionSend;
	rtError err;
	int maxBboxArea;
	objFrameData ofData;
	unsigned char*  hres_yuvData;
	unsigned char*  hres_y_data;
	unsigned char*  hres_uv_data;
	int hres_y_size;
	int hres_y_height;
	int hres_y_width;
	int hres_uv_size;
	bool isHresFrameReady;

	HttpClient* httpClient;
	int dnsCacheTimeout;
	STHPayload payload;

	uint64_t prev_time;
	int32_t event_quiet_time;

	char smtTnUploadURL[CONFIG_STRING_MAX];
	char smtTnAuthCode[AUTH_TOKEN_MAX];
	char modelName[CONFIG_STRING_MAX];
	char macAddress[CONFIG_STRING_MAX];
	char firmwareName[FW_NAME_MAX_LENGTH];

	int sTnHeight;
	int sTnWidth;
	char uploadFname[256];
	cv::Rect relativeBBox;
};

#endif //__SMART_THUMBNAIL_H__
