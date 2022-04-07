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
#ifdef _OBJ_DETECTION_
#include <jansson.h>
#include <fileUtils.h>
#include "RFCCommon.h"
#endif
#ifdef ENABLE_TEST_HARNESS
#include "dev_config.h"
#endif

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

#ifdef _OBJ_DETECTION_
#include "mpipe_port.h"
#include "base64.h"
#endif

#define FILEPATH_LEN                    64
#define CACHE_SMARTTHUMBNAIL "/tmp/cache_smart_thumbnail.txt"

#define FW_NAME_MAX_LENGTH 512
#define CONFIG_STRING_MAX (256)
#define YUV_HRES_BUFFER_ID		0
#define YUV_HRES_FRAME_WIDTH		1280
#define YUV_HRES_FRAME_HEIGHT		720

#ifdef XCAM2
#define STN_HRES_BUFFER_ID              2
#elif defined(XHB1) || defined (XHC3)
#define STN_HRES_BUFFER_ID              2
#define STN_MRES_BUFFER_ID              1
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
#ifdef XHB1
#define STN_HRES_CROP_WIDTH             640
#define STN_HRES_CROP_HEIGHT            480
#define STN_MRES_CROP_WIDTH             400
#define STN_MRES_CROP_HEIGHT            300
#endif

#define STN_TIMESTAMP_TAG		"timestamp"
#define STN_UPLOAD_TIME_INTERVAL	30
#define STN_UPLOAD_THRESHOLD_INTERVAL	60

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
#define DEFAULT_DOI_BITMAP              "/opt/usr_config/doi_bitmap"
#define DEFAULT_DOI_BITMAP_BINARY       "/opt/usr_config/doi_bitmap-binary.jpg"

#define RTMSG_DYNAMIC_LOG_REQ_RES_TOPIC   "RDKC.ENABLE_DYNAMIC_LOG"

#define STN_MAX( a, b ) ( ( a > b) ? a : b )
#define STN_MIN( a, b ) ( ( a < b) ? a : b )

#define UPPER_LIMIT_BLOB_BB     5
#define BLOB_BB_MAX_LEN         256
#define INVALID_BBOX_ORD        (-1)

#ifdef ENABLE_TEST_HARNESS
#define TEST_HARNESS_ON_FILE_ENABLED "Test_Harness_on_File_Enabled"
#endif

#ifdef _OBJ_DETECTION_
#define DEFAULT_INPUT_DEV "/dev/video0"
#define DEFAULT_GRAPH_PATH "/etc/mediapipe/graphs/rdk/delivery_detection/g_delivery_detection_cpu.pbtxt"
#define DEFAULT_DELIVERY_MODEL_PATH "/etc/mediapipe/models/xcv-delivery-detection-224x224-v2.3.2.tflite"
#define DEFAULT_FRAME_READ_DELAY "1000"
#define DEFAULT_MAX_FRAMES_CACHED_FOR_DELIVERY_DETECTION "5"
#define DEFAULT_DELIVERY_DETECTION_MODEL_MIN_SCORE_THRESHOLD "0.87"
#define DEFAULT_DELIVERY_DETECTION_MIN_SCORE_THRESHOLD "1"
#define DETECTION_CONFIG_FILE "/opt/usr_config/detection_attr.conf"
#define DEFAULT_FRAME_COUNT_TO_PROCESS "5"
#define DEFAULT_ROI_FILTER_ENABLE "1"
#define DEFAULT_MOTION_FILTER_ENABLE "0"
#define DEFAULT_MOTION_CUE_FILTER_ENABLE "0"
#define DEFAULT_SIZE_FILTER_THRESHOLD "50"
#define REQUEST_RECOVERY_FILE "/tmp/.stn_recovery_needed"
#define RECOVERY_TIME_THRESHOLD 60
#endif

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

typedef struct _tBoundingBox
{
    int32_t boundingBoxXOrd;
    int32_t boundingBoxYOrd;
    int32_t boundingBoxWidth;
    int32_t boundingBoxHeight;
}BoundingBox;

typedef struct {
    char fname[64];
    uint64_t tstamp;
#ifdef _HAS_DING_
    uint64_t dingtstamp;
#endif
#ifdef _OBJ_DETECTION_
    json_t *detectionResult;
    uint64_t motionTime;
    bool deliveryDetected;
#endif
    BoundingBox objectBoxs [UPPER_LIMIT_BLOB_BB];
    BoundingBox unionBox;
    uint64_t tsDelta;
}STHPayload;

typedef struct {
    uint32_t boundingBoxXOrd;
    uint32_t boundingBoxYOrd;
    uint32_t boundingBoxWidth;
    uint32_t boundingBoxHeight;
    uint64_t currTime;
    uint64_t tsDelta;
    cv::Mat  maxBboxObjYUVFrame;
}objFrameData;

class SmartThumbnail
{
    public:
	static SmartThumbnail* getInstance();
	//Initialize the buffers and starts msg monitoring, upload thread.
#ifdef _OBJ_DETECTION_
	STH_STATUS init(char* mac,bool isCVREnabled,int stnondelayType,int stnondelayTime,bool isDetectionEnabled, DetectionConfig detectionConfig);
#else
	STH_STATUS init(char* mac,bool isCVREnabled,int stnondelayType,int stnondelayTime);
#endif
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
#ifdef _OBJ_DETECTION_
#ifdef ENABLE_TEST_HARNESS
	void notifyXvision(const DetectionResult &result, double motionTriggeredTime, int mpipeProcessedframes, double time_taken, double time_waited);
	bool waitForNextDetectionFrame();
#endif
        bool waitForNextMotionFrame();
        STH_STATUS setMotionFrame(bool status);
        STH_STATUS setClipEnd(bool status);
	STH_STATUS setDetectionStatus(bool status);
	bool getDetectionStatus();
	void onCompletedDeliveryDetection(const DetectionResult &result);
        std::vector<cv::Point> getPolygonInCroppedRegion(std::vector<cv::Point> polygon, std::vector<cv::Point> croppedRegion);
	friend cv::Mat mpipe_port_getNextFrame(std::vector<cv::Point>& roiCoords, std::vector<std::vector<cv::Point>>& motionblobs);
        friend unsigned char * mpipe_port_getNextFrame(int *h, int *w);
#endif

	static void sigHandler(int signum);

    private:
	SmartThumbnail();
	~SmartThumbnail();
	STH_STATUS applyDOIonSTN(const objFrameData& ofData, const cv::Mat &DOIBitmap);
	STH_STATUS getDOIConf();
	STH_STATUS applyDOIthreshold(const char* bitmappath, uint8_t threshold);
	STH_STATUS saveSTN();
	STH_STATUS addSTN();
	STH_STATUS checkSTN();
	STH_STATUS delSTN(char* uploadFname);
	void printSTNList();
	STH_STATUS createPayload(char* uploadFname);
#ifdef _OBJ_DETECTION_
	STH_STATUS updateUploadPayload(char * fname, DetectionResult result);
        void createJSONFromDetectionResult(DetectionResult result, json_t* &resultJson);
#endif
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
	cv::Size getCropSize(cv::Rect boundRect,double w,double h, double *rescaleSize);
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
	static void onMsgRefresh(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure);
	static void onMsgDOIConfRefresh(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure);
	static void dynLogOnMessage(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure);
#ifdef _OBJ_DETECTION_
        static void onMsgROIChanged(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure);
        void printPolygonCoords(const char * str, std::vector<cv::Point>& polygon);
        void printROI();
#endif
	//Routine to upload STN Payload
	static void uploadSTN();

        static volatile bool termFlag;
        static volatile bool tnUploadConfRefreshed;
        static volatile bool eventConfRefreshed;
        static volatile bool DOIEnabled;
	static volatile int kDOIBitmapWidth;
	static volatile int kDOIBitmapHeight;
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
	bool logROIMotionEvent;
        char motionLog[CONFIG_STRING_MAX];

	bool uploadReady;
	//static bool rtmessageSTHThreadExit;
	//static bool uploadSTHThreadExit;
	bool isPayloadAvailable;
	std::mutex stnMutex;
	std::mutex uploadMutex;
	std::mutex hres_data_lock;
#ifdef _OBJ_DETECTION_
	std::mutex detectionMutex;
	std::mutex detectionNextFrameMutex;
	std::condition_variable detectionNextFrame_cv;
        bool isMotionFrame, clipEnd;
	unsigned char*  mpipe_hres_yuvData;
	cv::Rect currentBbox;
        BoundingBox currentMotionBlobs[UPPER_LIMIT_BLOB_BB];
        bool detectionCompleted;
	json_t *detectionResult;
        struct timeval detectionEndTime, uploadTriggeredTime;
        int mpipeProcessedframes;
        double motionTriggeredTime;
	char currDetectionSTNFname[CONFIG_STRING_MAX];
	bool detectionInProgress;
        bool detectionEnabled;
        std::vector<double> roi;
        bool detectionHang;
#ifdef ENABLE_TEST_HARNESS
	bool testHarnessOnFileFeed;
	std::vector<cv::Mat> yuvPlanes, yuvChannels;
	cv::Mat fileFrameYUV, planeUV, curr_frame;
	uint64_t currTstamp, detectionTstamp;
	std::condition_variable detectionCv;
	int THFileNum, THFrameNum, lastProcessedFrame;
	int FileNum = 0, FrameNum = 0, fps;
	uint64_t detectionStartTime, clipStartTime;
#else
	struct timeval detectionStartTime, clipStartTime;
#endif
#endif

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
        int stnOnDelayType;
        int stnOnDelayTime;
	HttpClient* httpClient;
	int dnsCacheTimeout;
	STHPayload currSTN;
	STHPayload payload;
	uint64_t motion_time;
	int32_t event_quiet_time;
	uint64_t tsDelta;
	char smtTnUploadURL[CONFIG_STRING_MAX];
	char smtTnAuthCode[AUTH_TOKEN_MAX];
	int sTnHeight;
	int sTnWidth;
        uint16_t buf_id;
	char uploadFname[CONFIG_STRING_MAX];
	char currSTNFname[CONFIG_STRING_MAX];
	char modelName[CONFIG_STRING_MAX];
    	char macAddress[CONFIG_STRING_MAX];
    	char firmwareName[FW_NAME_MAX_LENGTH];
	static int waitingInterval;
	cv::Rect relativeBBox;
	cv::Rect smartThumbCoord;
        BoundingBox objectBoxs [UPPER_LIMIT_BLOB_BB];
        time_t stnUploadTime;
        time_t eventquietTimeStart;
        bool debugBlob;
        bool debugBlobOnFullFrame;
        cv::Mat DOIBitmap;
        uint8_t doi_enable;
        uint8_t doi_threshold;
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
#ifdef _OBJ_DETECTION_
    BoundingBox deliveryUnionBox;
#endif
    BoundingBox unionBox;
    BoundingBox objectBoxs [UPPER_LIMIT_BLOB_BB];
    char const *s_curr_time;
};

#endif //__SMART_THUMBNAIL_H__
