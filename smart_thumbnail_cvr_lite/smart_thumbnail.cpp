/*
i##########################################################################
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

SmartThumbnail* SmartThumbnail::smartThInst = NULL;
#if 0
SmartThumbnail* SmartThumbnail::smartThInst = NULL;

bool SmartThumbnail::rtmessageSTHThreadExit = false;

bool SmartThumbnail::isHresFrameReady = false;
bool SmartThumbnail::isPayloadAvailable = false;
HttpClient* SmartThumbnail::httpClient = NULL;

STHPayload SmartThumbnail::payload ;
objFrameData SmartThumbnail::ofData ;

RDKC_PLUGIN_YUVInfo *SmartThumbnail::hres_frame_info = NULL;
RdkCVideoCapturer *SmartThumbnail::recorder = NULL;
bool SmartThumbnail::hres_yuvDataMemoryAllocationDone = false;
int SmartThumbnail::hres_y_size = 0;
int SmartThumbnail::hres_uv_size = 0;
unsigned char *SmartThumbnail::hres_yuvData = NULL;

int SmartThumbnail::hres_y_height = 0;
int SmartThumbnail::hres_y_width = 0;

rtConnection SmartThumbnail::connectionSend;
rtConnection SmartThumbnail::connectionRecv;
rtError SmartThumbnail::err;

uint64_t SmartThumbnail::prev_time = 0;
int SmartThumbnail::maxBboxArea = 0;

std::mutex SmartThumbnail::QMutex;

char SmartThumbnail::smtTnUploadURL[CONFIG_STRING_MAX] = {0};
char SmartThumbnail::smtTnAuthCode[AUTH_TOKEN_MAX] = {0};
char SmartThumbnail::modelName[CONFIG_STRING_MAX] = {0};
char SmartThumbnail::macAddress[CONFIG_STRING_MAX] = {0};
char SmartThumbnail::firmwareName[FW_NAME_MAX_LENGTH] = {0};

int32_t SmartThumbnail::event_quiet_time = STN_DEFAULT_EVT_QUIET_TIME;
#endif

/** @description: Constructor
 *  @param[in] void
 *  @return: void
 */
SmartThumbnail::SmartThumbnail():dnsCacheTimeout(STN_DEFAULT_DNS_CACHE_TIMEOUT),
				sTnHeight(STN_DEFAULT_HEIGHT),
				sTnWidth(STN_DEFAULT_WIDTH),
				rtmessageSTHThreadExit(false),
				isHresFrameReady(false),
				isPayloadAvailable(false),
#ifdef _HAS_DING_
				m_ding(NULL),
				m_dingNotif(false),
				m_uploadReady(false),
				m_dingTime(0),
#endif	
				httpClient(NULL),
#ifdef _HAS_XSTREAM_
				consumer(NULL),
				frameInfo(NULL),
#else
				recorder(NULL),
				hres_frame_info(NULL),
#endif
				hres_yuvDataMemoryAllocationDone(false),
				hres_y_size(0),
				hres_uv_size(0),
				hres_yuvData(NULL),
				hres_y_height(0),
				hres_y_width(0),
				cvrEnabled(false),
                                prev_time(0),
				maxBboxArea(0),
				event_quiet_time(STN_DEFAULT_EVT_QUIET_TIME)
{
    memset(uploadFname, 0, sizeof(uploadFname));
    memset(smtTnUploadURL, 0, sizeof(smtTnUploadURL));
    memset(smtTnAuthCode, 0, sizeof(smtTnAuthCode));
    memset(modelName, 0, sizeof(modelName));
    memset(macAddress, 0, sizeof(macAddress));
    memset(firmwareName, 0, sizeof(firmwareName));
#ifdef _HAS_XSTREAM_
#ifndef _DIRECT_FRAME_READ_
	frameHandler = {NULL, -1};
	//frameHandler.curl_handle = NULL;
#endif
#endif


    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Smart thumbnail constructor invoked.\n", __FUNCTION__, __LINE__);
}

/** @description: Destructor
 *  @param[in] void
 *  @return: void
 */
SmartThumbnail::~SmartThumbnail()
{
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Smart thumbnail constructor invoked.\n", __FUNCTION__, __LINE__);
}

/** @description: creates the instance for smart thumbnail
 *  @param[in] void
 *  @return: pointer to instance of SmartThumbnail
 */
SmartThumbnail *SmartThumbnail::getInstance()
{
    RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Creating smart thumbnail instance.\n", __FUNCTION__, __LINE__);
    if (!smartThInst) {
	smartThInst =  new SmartThumbnail();
    }
    RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Smart thumbnail instance created.\n", __FUNCTION__, __LINE__);

    return smartThInst;
}

/** @description: initialize smart thumbnail
 *  @param[in] void
 *  @return: STH_SUCCESS on success, STH_ERROR otherwise
 */
STH_STATUS SmartThumbnail::init(char* mac,bool isCvrEnabled)
{
    int ret= STH_SUCCESS;
    char usrVal[CONFIG_STRING_MAX];

    if(STH_SUCCESS == getTnUploadConf()) {
    	RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): getTnUploadConf success!!", __FUNCTION__, __LINE__);
    }

    if(STH_SUCCESS == getEventConf()) {
    	RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): getEventConf success!!", __FUNCTION__, __LINE__);
    }

    // update mac/model/camera image
    memset(smartThInst -> modelName, 0, sizeof(smartThInst -> modelName));
    memset(smartThInst -> macAddress, 0, sizeof(smartThInst -> macAddress));
    memset(smartThInst -> firmwareName, 0,sizeof(smartThInst -> firmwareName));
    
    setMacAddress();
    setModelName();
    setCameraImageName(smartThInst -> firmwareName);
    cvrEnabled = isCvrEnabled;
    RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): smart_thumbnail upload URL: %s \n",__FUNCTION__, __LINE__,smtTnUploadURL);
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): smart_thumbnail auth code: %s \n",__FUNCTION__, __LINE__,smtTnAuthCode);
    RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): smart_thumbnail event quiet time: %d \n",__FUNCTION__, __LINE__,smartThInst -> event_quiet_time);
    RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): smart_thumbnail height: %d \n",__FUNCTION__, __LINE__,sTnHeight);
    RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): smart_thumbnail width: %d \n",__FUNCTION__, __LINE__,sTnWidth);
#ifdef _HAS_DING_
    m_ding = DingNotification::getInstance();
    m_ding->init(modelName,macAddress,firmwareName);
#endif
#ifdef _HAS_XSTREAM_
	//Create XStreamerConsumer instance
	consumer = new XStreamerConsumer;
	if (NULL == consumer){
		RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):Failed to create consumer object. \n", __FUNCTION__ , __LINE__);
		return STH_ERROR;
	}

	//Initialize
#ifdef _DIRECT_FRAME_READ_
    ret = consumer->RAWInit((u16)STN_HRES_BUFFER_ID);
    if( ret < 0){
#else
	frameHandler = consumer->RAWInit(STN_HRES_BUFFER_ID, FORMAT_YUV, 0);
	if (frameHandler.sockfd < 0){
#endif
		RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): StreamInit failed.\n", __FUNCTION__, __LINE__);
		return STH_ERROR;
	}

	RDK_LOG(RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): StreamInit is successful\n", __FUNCTION__, __LINE__);

	frameInfo =  consumer->GetRAWFrameContainer();
	if (NULL == frameInfo){
	RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Failed to create application frame buffer\n", __FUNCTION__, __LINE__);
		return STH_ERROR;
	}
#else
    //creating plugin factory instance.
    pluginFactory = CreatePluginFactoryInstance();
    if (!pluginFactory) {
	RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Error creating plugin factory instance.\n", __FUNCTION__, __LINE__);
	return STH_ERROR;
    }

    //create recorder
    recorder = ( RdkCVideoCapturer* )pluginFactory->CreateVideoCapturer();
    if (!recorder) {
	RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Error creating instance of plugin recorder.\n", __FUNCTION__, __LINE__);
	return STH_ERROR;
    }

    //allocate memory for yuv high resolution buffer
    smartThInst -> hres_frame_info = (RDKC_PLUGIN_YUVInfo *) malloc(sizeof(RDKC_PLUGIN_YUVInfo));
    if (NULL == smartThInst -> hres_frame_info) {
        RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):Frame malloc error for high res frame info. \n", __FUNCTION__ , __LINE__);
        return STH_ERROR;
    }
#endif

    //initialize http client
    httpClient = new HttpClient();

    /* Open the URL */
    if (NULL != httpClient) {
	httpClient->open(smtTnUploadURL, dnsCacheTimeout);
    } else {
	RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Failed to open the URL\n", __FUNCTION__, __LINE__);
    }

    //Initializing thread to listen to incoming messages
    RDK_LOG(RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Creating rtMessage receive thread.\n", __FUNCTION__, __LINE__);
    rtMsgInit();
    std::thread rtMessageReceiveThread(receiveRtmessage);
    rtMessageReceiveThread.detach();
    return STH_SUCCESS;
}

/** @description: Get thumbnail upload conf
 *  @param[in] void
 *  @return: STH_SUCCESS on success, STH_ERROR otherwise
 */
STH_STATUS SmartThumbnail::getTnUploadConf()
{
    tn_provision_info_t *stnCfg = NULL;
    bool retry = true;

    // Read Thumbnail config.
    stnCfg = (tn_provision_info_t*) malloc(sizeof(tn_provision_info_t));

    if (NULL == stnCfg) {
	RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Error allocating memory.\n", __FILE__, __LINE__);
	return STH_ERROR;
    }

    if (STH_SUCCESS != polling_config_init()) {
	RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Error initializing polling config.\n", __FILE__, __LINE__);
	if (stnCfg) {
  	    free(stnCfg);
        }
       	return STH_ERROR;
    }

    while (retry) {
        if (STH_SUCCESS != readTNConfig(stnCfg)) {
            RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Error reading TNConfig.\n", __FILE__, __LINE__);
        } else {
	    break;
	}
	//Sleep 10 sec before retrying
	sleep(10);
    }

    // get url and auth
    strcpy(smtTnUploadURL, stnCfg -> url);
    strcpy(smtTnAuthCode, stnCfg -> auth_token);
    sTnHeight = atoi(stnCfg -> height);
    sTnWidth = atoi(stnCfg -> width);

    if (stnCfg) {
        free(stnCfg);
        stnCfg = NULL;
    }

    polling_config_exit();

    return STH_SUCCESS;
}

/** @description: Get Event conf
 *  @param[in] void
 *  @return: STH_SUCCESS on success, STH_ERROR otherwise
 */
STH_STATUS SmartThumbnail::getEventConf()
{
    bool retry = true;
    events_provision_info_t *eventsCfg = NULL;

    // Read event config.
    eventsCfg = (events_provision_info_t*) malloc(sizeof(events_provision_info_t));

    if (NULL == eventsCfg) {
        RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Error allocating memory.\n", __FILE__, __LINE__);
        return STH_ERROR;
    }

    if (STH_SUCCESS != polling_config_init()) {
        RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Error initializing polling config.\n", __FILE__, __LINE__);
        if (eventsCfg) {
            free(eventsCfg);
        }
        return STH_ERROR;
    }

    while (retry) {
	if (STH_SUCCESS != readEventConfig(eventsCfg)) {
     	    RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Error reading EVENTS Config.\n", __FILE__, __LINE__);
    	} else {
	    break;
	}
	//Sleep 10 sec before retrying
	sleep(10);
    }

    // get event quiet interval
    if (strlen(eventsCfg->quite_interval) > 0) {
	smartThInst -> event_quiet_time = atoi(eventsCfg->quite_interval);
    }

    if (eventsCfg) {
        free(eventsCfg);
        eventsCfg = NULL;
    }

    polling_config_exit();

    return STH_SUCCESS;
}

/** @description: retrieve event quiet interval
 *  @param[in] : void
 *  @return: event quiet interval
 */
int SmartThumbnail::getQuietInterval()
{
	int quiet_interval =smartThInst -> event_quiet_time;
	events_provision_info_t *eventsCfg = NULL;

	// Allocate memory for event config
	eventsCfg = (events_provision_info_t*) malloc(sizeof(events_provision_info_t));

	if (NULL == eventsCfg) {
		RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Error allocating memory. Use existing quiet interval %d\n", __FILE__, __LINE__, quiet_interval);
		return quiet_interval;
	}

	if (STH_SUCCESS != polling_config_init()) {
		RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Error initializing polling config. Use existing quiet interval %d\n", __FILE__, __LINE__, quiet_interval);
		if (eventsCfg) {
			free(eventsCfg);
		}
		return quiet_interval;
	}

	if (STH_SUCCESS != readEventConfig(eventsCfg)) {
		RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Error reading EVENTS Config. Use existing quiet interval %d\n", __FILE__, __LINE__, quiet_interval);
		return quiet_interval;
	}

	// get event quiet interval
	if (strlen(eventsCfg->quite_interval) > 0) {
		quiet_interval = atoi(eventsCfg->quite_interval);
	}
	else {
		RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Invalid Quiet Interval. Use existing quiet interval %d\n", __FILE__, __LINE__, quiet_interval);
		return quiet_interval;

	}

	if (smartThInst -> event_quiet_time != quiet_interval) {
		RDK_LOG(RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Retrieved New Quiet Interval: %d %d\n", __FILE__, __LINE__, smartThInst -> event_quiet_time, quiet_interval);
	}

	if (eventsCfg) {
		free(eventsCfg);
		eventsCfg = NULL;
	}

	polling_config_exit();

	return quiet_interval;
}

/** @description: create the payload for smart thumbnail
 *  @param[in] : void
 *  @return: STH_SUCCESS on success, STH_ERROR otherwise
 */
STH_STATUS SmartThumbnail::createPayload()
{

    STH_STATUS ret = STH_ERROR;

    cv::Rect unionBox;
    cv::Mat lHresRGBMat;
    cv::Mat croppedObj;
#ifdef USE_FILE_UPLOAD
    struct tm* tv = NULL;
    struct timespec currTime;
#endif

    // update the event quiet interval
    {
	//Acquire lock
	std::unique_lock<std::mutex> lock(smartThInst -> QMutex);
	if (smartThInst -> isPayloadAvailable)
	{
	    memset(&payload,0,sizeof(payload));
            RDK_LOG(RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):Payload is available.\n", __FILE__, __LINE__);
	    payload.tstamp   = smartThInst -> ofData.currTime;

	    // matrix to store color image
	    lHresRGBMat = cv::Mat(hres_y_height, hres_y_width, CV_8UC4);
	    // convert the frame to BGR format
	    cv::cvtColor(smartThInst -> ofData.maxBboxObjYUVFrame,lHresRGBMat, cv::COLOR_YUV2BGR_NV12);

	    unionBox.x = smartThInst -> ofData.boundingBoxXOrd;
	    unionBox.y = smartThInst -> ofData.boundingBoxYOrd;
	    unionBox.width = smartThInst -> ofData.boundingBoxWidth;
	    unionBox.height = smartThInst -> ofData.boundingBoxHeight;

	    // extracted the below logic from server scala code
            RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):unionBox.x %d unionBox.y %d unionBox.height %d unionBox.width %d\n", __FILE__, __LINE__, unionBox.x, unionBox.y, unionBox.height, unionBox.width);
	    cv::Point2f orgCenter = getActualCentroid(unionBox);
	    cv::Size cropSize = getCropSize(unionBox, sTnWidth, sTnHeight);
	    cv::Point2f allignedCenter =  alignCentroid(orgCenter, lHresRGBMat, cropSize);
	    getRectSubPix(lHresRGBMat, cropSize, allignedCenter, croppedObj);
	    relativeBBox = getRelativeBoundingBox(unionBox, cropSize, allignedCenter);

           //Update cropped SmartThumbnail Coordinates
           smartThumbCoord.x = (allignedCenter.x - (cropSize.width / 2));
           smartThumbCoord.y = (allignedCenter.y - (cropSize.height / 2));
           smartThumbCoord.width = cropSize.width;
           smartThumbCoord.height = cropSize.height;

#ifdef USE_FILE_UPLOAD
            memset(&currTime, 0, sizeof(currTime));
            clock_gettime(CLOCK_REALTIME, &currTime);
            tv = gmtime(&currTime.tv_sec);

    	    memset(uploadFname, 0, sizeof(uploadFname));
            snprintf(uploadFname, sizeof(uploadFname), "%s/%04d%02d%02d%02d%02d%02d.jpg", STN_PATH,(tv->tm_year+1900), tv->tm_mon+1, tv->tm_mday, tv->tm_hour, tv->tm_min, tv->tm_sec);
            RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Creating smart thumbnail upload file: %s\n",__FILE__, __LINE__, uploadFname);
            //Write smart thumbnail to file.
            imwrite(uploadFname,croppedObj);
#endif
	    //reset payload flag
	    smartThInst -> isPayloadAvailable = false;
	    smartThInst -> maxBboxArea = 0;
	    ret = STH_SUCCESS;
    	} else {
    	    RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):Nothing to upload.\n", __FILE__, __LINE__);
	    ret = STH_NO_PAYLOAD;
        }
#ifdef _HAS_DING_
	if(smartThInst -> m_dingNotif )
	{
	   payload.dingtstamp = smartThInst ->m_dingTime;
	   smartThInst -> m_dingNotif = false;
	}
#endif      
        lHresRGBMat.release();
        croppedObj.release();
	ofData.maxBboxObjYUVFrame.release();
	//Release lock
	lock.unlock();
    }
    return ret;
}

/** @description: find the actual centroid of the bounding rectangle
 *  @param[in] bounding rectangle
 *  @return: central point {x,y}
 */
cv::Point2f SmartThumbnail::getActualCentroid(cv::Rect boundRect) {

    cv::Point2f pts;

    int adjustFactor = 1; // multi-by-6 to re-adjust centroid
    float heightPading = 0.4;
    float xPoint =  adjustFactor *(boundRect.x + (boundRect.width / 2));
    float yPoint =  adjustFactor *(boundRect.y + (boundRect.height /2));

    pts.x = xPoint;
    pts.y = yPoint;

    return pts;
}

/** @description: Allign the centroid of the bounding window
 *  @param[in] original centroid
 *  @param[in] original frame resolution
 *  @param[in] crop window size
 *  @return: alligned centroid
 */
cv::Point2f SmartThumbnail::alignCentroid(cv::Point2f orgCenter, cv::Mat origFrame, cv::Size cropSize) {

    cv::Point2f pts;

    float shiftX = (orgCenter.x + (cropSize.width/2)) - origFrame.cols;
    float adjustedX = orgCenter.x - STN_MAX(0, shiftX);
    float shiftXleft = (adjustedX - (cropSize.width/2));
    float adjustedXfinal = adjustedX - STN_MIN(0, shiftXleft);
    float shiftY = (orgCenter.y + (cropSize.height/2)) - origFrame.rows;
    float adjustedY = orgCenter.y - STN_MAX(0, shiftY);
    float shiftYdown = (adjustedY - (cropSize.height/2));
    float adjustedYfinal = adjustedY - STN_MIN(0, shiftYdown);

    pts.x = adjustedXfinal;
    pts.y = adjustedYfinal;

    std::cout << "\n\n Original Center { " << orgCenter.x << ", " << orgCenter.y << " } " << "Alligned Center: { " << adjustedXfinal << ", " << adjustedYfinal << " } \n";
    std::cout << " Cropping Resolution {W, H}:  { " << cropSize.width << ", " << cropSize.height << " } " << "Original frame Resolution {W, H}: { " << origFrame.cols << ", " << origFrame.rows << " } \n";
    std::cout << " Intermediate Adjustments {shiftX, adjustedX, shiftXleft}: { " << shiftX << ", " << adjustedX << ", " << shiftXleft << " } \n";
    std::cout << " Intermediate Adjustments {shiftY, adjustedY, shiftYdown}: { " << shiftY << ", " << adjustedY << ", " << shiftYdown << " } \n\n";

    return pts;
}

/** @description: find the crop size window {w, h}
 *  @param[in] bounding rectangle
 *  @param[in] minimum width of the crop window
 *  @param[in] minimum height of the crop window
 *  @return: size of the crop window
 */
cv::Size SmartThumbnail::getCropSize(cv::Rect boundRect,double w,double h) {
    int newWidth = 0;
    int newHeight = 0;
    int adjustFactor = 1;

    cv::Size sz;
    newWidth = boundRect.width;
    newHeight = boundRect.height;

#if 0
    if (boundRect.width > (boundRect.height *(w/h))) {
        newWidth = boundRect.width;
      } else {
        newWidth = (int)(boundRect.height *(w/h));
    }

    if (boundRect.height > (boundRect.width *(h/w))) {
        newHeight = boundRect.height;
      } else {
        newHeight = (int)(boundRect.width * (h/w));
    }
#endif

    // always ensure the minimum size of the crop size window is {w, h}
    sz.height = STN_MAX(h,(newHeight* adjustFactor));
    sz.width = STN_MAX(w,(newWidth* adjustFactor));

    return sz;
}

/** @description: Get the bounding box ordinates related to the resolution of smart thumbnail
 *  @param[in] boundRect: original bounding box ordinates
 *  @param[in] cropSize: resolution of the cropped image
 *  @param[in] allignedCenter: alligned center of the bounding box
 *  @return: relative bounding box
 */
cv::Rect SmartThumbnail::getRelativeBoundingBox(cv::Rect boundRect, cv::Size cropSize, cv::Point2f allignedCenter) {

    cv::Rect newBBox; // to store the new bounding box co-ordinate

    // to find the relative x-ordinate and width of the bounding box in the smart thumbnail image
    if (boundRect.width >= cropSize.width) {
    	newBBox.x = 0;
    	newBBox.width = cropSize.width; // restrict the width of the relative bounding box to width of the final cropped image 
    }
    else {
    	float deltaX = allignedCenter.x - boundRect.x;
    	newBBox.x = cropSize.width/2 - deltaX;
    	newBBox.width = boundRect.width;
    }

    // to find the relative y-ordinate and height of the bounding box in the smart thumbnail image
    if (boundRect.height >= cropSize.height) {
        newBBox.y = 0;
        newBBox.height = cropSize.height; // restrict the height of the relative bounding box to height of the final cropped image
    }
    else {
    	float deltaY = allignedCenter.y - boundRect.y;
    	newBBox.y = cropSize.height/2 - deltaY;
        newBBox.height = boundRect.height;
    }

    return newBBox;
}

STH_STATUS SmartThumbnail::notify( const char* status)
{
    if(!status) {
        RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):Trying to use invalid memory location!! \n", __FUNCTION__ , __LINE__);
	return STH_ERROR;
    }

    rtMessage req;
    rtMessage_Create(&req);
    rtMessage_SetString(req, "status", status);

    rtError err = rtConnection_SendMessage(connectionSend, req, "RDKC.SMARTTN.STATUS");
    rtLog_Debug("SendRequest:%s", rtStrError(err));

    if (err != RT_OK)
    {
        RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d) Error sending msg via rtmessage\n", __FILE__,__LINE__);
    }
    rtMessage_Release(req);

    return STH_SUCCESS;
}

/** @description: destroy the instance of smart thumbnail
 *  @param[in] : void
 *  @return	: STH_SUCCESS on success, STH_ERROR otherwise
 */
STH_STATUS SmartThumbnail::destroy()
{
	RDK_LOG(RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d) destroy smart thumbnail\n", __FUNCTION__ , __LINE__ );

	STH_STATUS ret = STH_SUCCESS;

	//Exit the rtmessage receive and upload thread.
	rtmessageSTHThreadExit =  true;
	//uploadSTHThreadExit = true;

#ifdef _HAS_XSTREAM_
#ifdef _DIRECT_FRAME_READ_
    if( (NULL != consumer) && (STH_ERROR != consumer->RAWClose())){
#else
  	//Close the CURL
	if( (NULL != consumer) && (STH_ERROR != consumer->RAWClose(frameHandler.curl_handle))){
#endif
		RDK_LOG(RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d) StreamClose Successful\n", __FUNCTION__ , __LINE__);
	}
	else{
		RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d) StreamClose Failed\n", __FUNCTION__ , __LINE__);
		ret = STH_ERROR;
	}

	//Delete the XStreamerConsumer instance
	if (consumer) {
		delete consumer;
		consumer = NULL;
	}

	frameInfo = NULL;
#ifndef _DIRECT_FRAME_READ_
	frameHandler.curl_handle = NULL;
	frameHandler.sockfd = -1;
#endif
#else
	if (smartThInst -> hres_frame_info) {
		free(smartThInst -> hres_frame_info);
		smartThInst -> hres_frame_info = NULL;
	}
#endif

	//Delete the smart thumbnail instance
	RDK_LOG(RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d) Deleting the smart thumbnail instance!!!\n", __FUNCTION__ , __LINE__);
	if (smartThInst) {
		delete smartThInst;
		smartThInst =NULL;
	}
#ifdef _HAS_DING_
	if(m_ding)
		delete m_ding;	
#endif
#ifdef LEGACY_CFG_MGR
	config_release();
#endif

	return ret;
}

#ifdef _HAS_DING_
/** @description    : Callback function to generate ding notification and smartthumbnail.
 *  @param[in]  hdr : constant pointer rtMessageHeader
 *  @param[in] buff : constant pointer uint8_t
 *  @param[in]    n : uint32_t
 *  @param[in] closure : void pointer
 *  @return: void
 */

void SmartThumbnail::onDingNotification(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure)
{
    char const*  status = NULL;
    
    rtConnection con = (rtConnection) closure;
    int doorbell_state =0;
    rtMessage req;
    rtMessage_FromBytes(&req, buff, n);
    rtMessage_GetInt32(req, "doorbell_press", &doorbell_state);

    RDK_LOG(RDK_LOG_INFO,"LOG.RDK.BUTTONMGR","(%s):%d doorbell_state:%d\n", __FUNCTION__, __LINE__, doorbell_state);
    
    if(doorbell_state)
    {
	struct timespec currTime;
	memset (&currTime, 0, sizeof(struct timespec));
    	clock_gettime(CLOCK_REALTIME, &currTime);
	
	if((currTime.tv_sec - smartThInst ->m_dingTime) > smartThInst->m_ding->getQuiteTime())
	{
	   smartThInst ->m_dingTime = currTime.tv_sec;
	   smartThInst -> m_dingNotif = true;
	   smartThInst->m_ding->signalDing(true,smartThInst->m_dingTime);
	   smartThInst->setUploadStatus(true);
	}
    }
    rtMessage_Release(req);
}


STH_STATUS SmartThumbnail::setUploadStatus(bool status)
{
    {
        std::unique_lock<std::mutex> lock(m_uploadMutex);
        m_uploadReady = status;
        lock.unlock();
    }

    m_cv.notify_one();
    return STH_SUCCESS;
}

bool SmartThumbnail::waitFor(int quiteInterVal)
{
    bool isTimedOut = false;
    {
        std::unique_lock<std::mutex> lock(m_uploadMutex);

        m_cv.wait_for(lock, std::chrono::seconds(quiteInterVal), [this]{return (m_uploadReady);});

        if(m_uploadReady)
        {
          RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Wait over due to uploadReady flag!!\n",__FUNCTION__,__LINE__);
          m_uploadReady = false;
        }
        else
        {
	   isTimedOut = true;
           RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Wait over due to timeout!!\n",__FUNCTION__,__LINE__);
        }
        lock.unlock();
   }
   return isTimedOut;
}

#endif
/** @description: Callback function for dynamic logging
 *  @param[in] hdr : pointer to rtMessage Header
 *  @param[in] buff : buffer for data received via rt message
 *  @param[in] n : number of bytes received
 *  @return: void
 */
void SmartThumbnail::dynLogOnMessage(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure)
{
    char const*  module = NULL;
    char const*  logLevel = NULL;

    rtConnection con = (rtConnection) closure;

    rtMessage req;
    rtMessage_FromBytes(&req, buff, n);

    //Handle the rtmessage request
    if (rtMessageHeader_IsRequest(hdr)) {
	char* tmp_buff = NULL;
        uint32_t tmp_buff_length = 0;

        rtMessage_ToString(req, &tmp_buff, &tmp_buff_length);
        rtLog_Info("Req : %.*s", tmp_buff_length, tmp_buff);
        free(tmp_buff);

        rtMessage_GetString(req, "module", &module);
        rtMessage_GetString(req, "logLevel", &logLevel);

        RDK_LOG(RDK_LOG_INFO,"LOG.RDK.DYNAMICLOG","(%s):%d Module name: %s\n", __FUNCTION__, __LINE__, module);
        RDK_LOG(RDK_LOG_INFO,"LOG.RDK.DYNAMICLOG","(%s):%d log level: %s\n", __FUNCTION__, __LINE__, logLevel);

        RDK_LOG_ControlCB(module, NULL, logLevel, 1);

        // create response
        rtMessage res;
        rtMessage_Create(&res);
        rtMessage_SetString(res, "reply", "Success");
        rtConnection_SendResponse(con, hdr, res, 1000);
        rtMessage_Release(res);
    }
    rtMessage_Release(req);
}

/** @description    : Callback function to capture high resolution frame
 *  @param[in]  hdr : constant pointer rtMessageHeader
 *  @param[in] buff : constant pointer uint8_t
 *  @param[in]    n : uint32_t
 *  @param[in] closure : void pointer
 *  @return: void
 */
void SmartThumbnail::onMsgCaptureFrame(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure)
{
    int processPID = -1;
    int ret = STH_ERROR;
    char const*  strFramePTS = NULL;
    uint64_t lResFramePTS = 0;
    uint64_t hResFramePTS = 0;
    struct timespec currTime;

    //clock the current time
    memset (&currTime, 0, sizeof(struct timespec));
    clock_gettime(CLOCK_REALTIME, &currTime);

    // read the message received
    (void) closure;
    rtMessage m;
    rtMessage_FromBytes(&m, buff, n);
    rtMessage_GetInt32(m, "processID", &processPID);
    rtMessage_GetString(m, "timestamp", &strFramePTS);

    RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d)  strFramePTS:%s \n", __FUNCTION__ , __LINE__, strFramePTS);
    RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d) capture invoked by process %d \n", __FUNCTION__ , __LINE__, processPID);

    std::istringstream iss(strFramePTS);
    iss >> lResFramePTS;
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): lResframePTS:%llu\n", __FUNCTION__, __LINE__, lResFramePTS);

    rtMessage_Release(m);


    RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):prev_time :%d\n", __FUNCTION__ , __LINE__, smartThInst -> prev_time);

    // ignore frames and metadata for event_quiet_time
    if( (currTime.tv_sec < (smartThInst -> prev_time + smartThInst -> event_quiet_time)) &&  (0 != smartThInst -> prev_time) ) {
        RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):Within event quiet time, Ignoring event, time passed: %lu\n", __FUNCTION__ , __LINE__, (currTime.tv_sec- smartThInst -> prev_time));
	return;
    }

    RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):Not within event quiet time, Capturing frame.\n", __FUNCTION__ , __LINE__);

    //read the 720*1280 YUV data.
    std::unique_lock<std::mutex> lock(smartThInst -> QMutex);
#ifdef _HAS_XSTREAM_
    if ((NULL != smartThInst) && (NULL != smartThInst->consumer)){
#ifdef _DIRECT_FRAME_READ_
        ret = smartThInst -> consumer -> ReadRAWFrame((u16)STN_HRES_BUFFER_ID, (u16)FORMAT_YUV, smartThInst -> frameInfo);
#else
	ret = smartThInst -> consumer -> ReadRAWFrame(STN_HRES_BUFFER_ID, FORMAT_YUV, smartThInst -> frameInfo);
#endif
    }
#else
    if (NULL == smartThInst -> hres_frame_info) {
        RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):High Res YUV Frame malloc error \n", __FUNCTION__ , __LINE__);
    }
    memset(smartThInst -> hres_frame_info, 0, sizeof(RDKC_PLUGIN_YUVInfo));

    ret = smartThInst -> recorder -> ReadYUVData(STN_HRES_BUFFER_ID, smartThInst -> hres_frame_info);

#endif
    if( STH_SUCCESS != ret) {
        RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):Error Reading High Res YUV Frame  \n", __FUNCTION__, __LINE__);
    }
    //RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):Success in Reading High Res YUV Frame return status is %d \n", __FUNCTION__, __LINE__, ret);

#ifdef _HAS_XSTREAM_
    hResFramePTS = smartThInst ->frameInfo -> mono_pts;
#else
    hResFramePTS = smartThInst -> hres_frame_info -> mono_pts;
#endif
    smartThInst -> isHresFrameReady = true;
    lock.unlock();
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): hResframePTS:%llu\n", __FUNCTION__, __LINE__, hResFramePTS);
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):Time gap (hResFramePTS - lResframePTS):%llu\n", __FUNCTION__, __LINE__, (hResFramePTS - lResFramePTS));
}

/** @description    : Callback function to generate smart thumbnail based on motion.
 *  @param[in]  hdr : constant pointer rtMessageHeader
 *  @param[in] buff : constant pointer uint8_t
 *  @param[in]    n : uint32_t
 *  @param[in] closure : void pointer
 *  @return: void
 */
void SmartThumbnail::onMsgProcessFrame(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure)
{
    SmarttnMetadata_thumb sm;
    uint64_t curr_time = 0;
    uint64_t lResFramePTS = 0;

    cv::Mat l_hres_yuvMat;
    cv::Mat l_hres_RGBMat;
    cv::Mat cropped_object;
    cv::Mat resized_cropped_object;
    int lBboxArea = 0;

    (void) closure;

    rtMessage m;
    rtMessage_FromBytes(&m, buff, n);

    RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d) process frame invoked.\n", __FUNCTION__ , __LINE__);

    //read the metadata.
    SmarttnMetadata_thumb::from_rtMessage(&sm, m);

    std::istringstream iss(sm.strFramePTS);
    iss >> lResFramePTS;
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): lResframePTS:%llu\n", __FUNCTION__, __LINE__, lResFramePTS);
    iss.clear();

    iss.str(sm.s_curr_time);
    iss >> curr_time;
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): curr timestamp (uint64):%llu\n", __FILE__, __LINE__,curr_time);
    iss.clear();

    lBboxArea = sm.unionBox.boundingBoxWidth * sm.unionBox.boundingBoxHeight;

    RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Current Area : %d\n", __FILE__ , __LINE__, lBboxArea);

    // if motion is detected update the metadata.
    if ((smartThInst -> isHresFrameReady) &&
       //(motionScore != 0.0) &&
       (sm.event_type == 4) &&
       (lBboxArea > smartThInst -> maxBboxArea)) {

    	RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d) Processing metadata .\n", __FUNCTION__ , __LINE__);

	smartThInst -> isHresFrameReady =  false;
        //Update the max bounding area.
        smartThInst -> maxBboxArea = lBboxArea;
        {
	    //Acquire lock
	    std::unique_lock<std::mutex> lock(smartThInst -> QMutex);
	    if (!smartThInst -> isPayloadAvailable) {
    		RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d) Payload will be available for this interval.\n", __FUNCTION__ , __LINE__);
		//To indicate payload will there to upload
	        smartThInst -> isPayloadAvailable = true;
	    }
            updateObjFrameData(sm.unionBox.boundingBoxXOrd, sm.unionBox.boundingBoxYOrd, sm.unionBox.boundingBoxWidth, sm.unionBox.boundingBoxHeight, curr_time);

            memset(smartThInst->objectBoxs, INVALID_BBOX_ORD, sizeof(smartThInst->objectBoxs));
            for(int32_t i=0; i<MAX_BLOB_SIZE; i++) {

                if(sm.objectBoxs[i].boundingBoxXOrd == INVALID_BBOX_ORD) {
                    break;
                }
                smartThInst -> updateObjectBoxs(&(sm.objectBoxs[i]), i);
            }
	    lock.unlock();
	}

	smartThInst -> prev_time = curr_time;
    } else {
    	RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d) Metadata discarded .\n", __FUNCTION__ , __LINE__);
    }

    rtMessage_Release(m);
}

/** @description   : resizes smart thumbnail to default height & width
 *  @param[in]  im : cropped image
 *  @param[in]   h : height
 *  @param[in]   w : width
 *  @param[out] im_resized : resized smart thumbnail
 *  @return: STH_SUCCESS on success, STH_ERROR otherwise
 */

#if 0
STH_STATUS SmartThumbnail::resizeAspect(cv::Mat im, int w, int h, cv::Mat& im_resized)
{
    im_resized = cv::Mat();

    if (im.empty()) {
        //std::cerr << __FUNCTION__ << " : input image is empty" << std::endl;
        RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):input image is empty.\n", __FUNCTION__ , __LINE__);
        return STH_ERROR;
    }

    int new_w = im.cols;
    int new_h = im.rows;

    if (((float)w/new_w) < ((float)h/new_h)) {
        new_w = w;
        new_h = (im.rows * w)/im.cols;
    } else {
        new_h = h;
        new_w = (im.cols * h)/im.rows;
    }

    cv::Mat resized;
    cv::resize(im,resized,cv::Size(new_w, new_h));

    // create image the size of the net layer
    im_resized = cv::Mat(h,w,resized.type());

    // embed image into im_resized image
    int dx = (w-new_w)/2;
    int dy = (h-new_h)/2;

    resized.copyTo(im_resized(cv::Rect(dx,dy,resized.cols, resized.rows)));

    return STH_SUCCESS;
}
#endif
void SmartThumbnail::resetObjFrameData()
{
    smartThInst -> ofData.boundingBoxXOrd = 0;
    smartThInst -> ofData.boundingBoxYOrd = 0;
    smartThInst -> ofData.boundingBoxWidth = 0;
    smartThInst -> ofData.boundingBoxHeight = 0;
    smartThInst -> ofData.currTime = 0;
    if(!smartThInst -> ofData.maxBboxObjYUVFrame.empty())
    {
	RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Releasing maxBboxObjYUVFrame\n", __FILE__, __LINE__);                        
	smartThInst -> ofData.maxBboxObjYUVFrame.release();
    } 
}
/** @description   : resizes smart thumbnail to default height & width
 *  @param[in]     : x ordinate of bounding box
 *  @param[in]     : y ordinate of bounding box
 *  @param[in]     : width of bounding box
 *  @param[in]     : height of bounding box
 *  @return        : void
 */
void SmartThumbnail::updateObjFrameData(int32_t boundingBoxXOrd,int32_t boundingBoxYOrd,int32_t boundingBoxWidth,int32_t boundingBoxHeight,			      						uint64_t currTime)
{
    unsigned char*  hres_yuvData = NULL;
    smartThInst -> resetObjFrameData();
    smartThInst -> ofData.boundingBoxXOrd = boundingBoxXOrd;
    smartThInst -> ofData.boundingBoxYOrd = boundingBoxYOrd;
    smartThInst -> ofData.boundingBoxWidth = boundingBoxWidth;
    smartThInst -> ofData.boundingBoxHeight = boundingBoxHeight;
    smartThInst -> ofData.currTime = currTime;

#ifdef _HAS_XSTREAM_
    smartThInst -> hres_y_height = smartThInst -> frameInfo->height;
    smartThInst -> hres_y_width = smartThInst -> frameInfo-> width;
    smartThInst -> hres_y_size = smartThInst -> frameInfo-> width * smartThInst -> frameInfo-> height;
    smartThInst -> hres_uv_size = smartThInst -> frameInfo-> width * smartThInst -> frameInfo-> height/2;
    hres_yuvData = (unsigned char *) malloc((smartThInst -> hres_y_size + smartThInst -> hres_uv_size) * sizeof(unsigned char));

    memset( hres_yuvData, 0, (smartThInst -> hres_y_size + smartThInst -> hres_uv_size) * sizeof(unsigned char) );
    memcpy( hres_yuvData, smartThInst -> frameInfo ->y_addr, smartThInst -> hres_y_size);
    memcpy( hres_yuvData + smartThInst -> hres_y_size, smartThInst -> frameInfo ->uv_addr, smartThInst -> hres_uv_size);

    //Full 720*1280 frame containing max bounding box
    smartThInst -> ofData.maxBboxObjYUVFrame = cv::Mat(smartThInst -> frameInfo -> height + (smartThInst -> frameInfo -> height)/2, smartThInst -> frameInfo -> width, CV_8UC1, hres_yuvData).clone();
#else
    smartThInst -> hres_y_height = smartThInst -> hres_frame_info->height;
    smartThInst -> hres_y_width = smartThInst -> hres_frame_info->width;
    smartThInst -> hres_y_size = smartThInst -> hres_frame_info->width * smartThInst -> hres_frame_info->height;
    smartThInst -> hres_uv_size = smartThInst -> hres_frame_info->width * smartThInst -> hres_frame_info->height;
    hres_yuvData = (unsigned char *) malloc((smartThInst -> hres_y_size + smartThInst -> hres_uv_size) * sizeof(unsigned char));

    memset( hres_yuvData, 0, (smartThInst -> hres_y_size + smartThInst -> hres_uv_size) * sizeof(unsigned char) );
    memcpy( hres_yuvData, smartThInst -> hres_frame_info->y_addr, smartThInst -> hres_y_size);
    memcpy( hres_yuvData + smartThInst -> hres_y_size, smartThInst -> hres_frame_info->uv_addr, smartThInst -> hres_uv_size);

    //Full 720*1280 frame containing max bounding box
    smartThInst -> ofData.maxBboxObjYUVFrame = cv::Mat(smartThInst -> hres_frame_info -> height + (smartThInst -> hres_frame_info -> height)/2, smartThInst -> hres_frame_info -> width, CV_8UC1, hres_yuvData).clone();
#endif

    if(hres_yuvData) {
        free(hres_yuvData);
        hres_yuvData = NULL;
    }
}

/** @description: register callback for receiving msg via rtmessage.
 *  @param[in] void.
 *  @return: STH_SUCCESS on success, STH_ERROR otherwise
 */
STH_STATUS SmartThumbnail::rtMsgInit()
{
    rtLog_SetLevel(RT_LOG_INFO);
    rtLog_SetOption(rdkLog);

    rtConnection_Create(&connectionSend, "SMART_TN_SEND", "tcp://127.0.0.1:10001");
    rtConnection_Create(&smartThInst -> connectionRecv, "SMART_TN_RECV", "tcp://127.0.0.1:10001");

    rtConnection_AddListener(smartThInst -> connectionRecv, "RDKC.SMARTTN.CAPTURE",onMsgCaptureFrame, NULL);
    rtConnection_AddListener(smartThInst -> connectionRecv, "RDKC.SMARTTN.METADATA",onMsgProcessFrame, NULL);

    //Add listener for dynamic log topics
    rtConnection_AddListener(smartThInst -> connectionRecv, RTMSG_DYNAMIC_LOG_REQ_RES_TOPIC, dynLogOnMessage, smartThInst -> connectionRecv);
#ifdef _HAS_DING_
    rtConnection_AddListener(connectionRecv, "RDKC.BUTTON.DOORBELL",onDingNotification, NULL);
#endif
    return STH_SUCCESS;
}
/**
 * @description: Convert event date and time to ISO 8601 format.
 *
 * @param[in]: strEvtDateTime,evtdatetimeSize, evtDateTime.
 *
 * @return: void
 */
void SmartThumbnail::stringifyEventDateTime(char* strEvtDateTime , size_t evtdatetimeSize, time_t evtDateTime)
{
        struct tm *tv = NULL;

        if(NULL == strEvtDateTime) {
                RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.CVRUPLOAD","%s(%d): Using invalid memory location!\n",__FILE__, __LINE__);
                return;
        }

        tv = gmtime(&evtDateTime);

        strftime(strEvtDateTime, evtdatetimeSize,"%FT%TZ",tv);
}

/** @description: thread routine to upload smart thumbnail payload
 *  @param[in] : time left
 *  @return: void
 */
int  SmartThumbnail::uploadPayload(time_t timeLeft)
{
    STH_STATUS ret = STH_SUCCESS;
    int curlCode = 0;
    long response_code = 0;
    char objectBoxsBuf[BLOB_BB_MAX_LEN] = {0};
    char smartThumbBuf[BLOB_BB_MAX_LEN] = {0};
#ifdef USE_FILE_UPLOAD
    char *data  = NULL;
    struct stat fileStat;
    int fileLen = 0;
    int readLen = 0;
    char *ptr   = NULL;
    char readBuf[STN_UPLOAD_SEND_LEN];
    int fd = 0;
#else
    char *dataPtr  = NULL;
    int dataLen = 0;
    //std::vector<uchar> dataPtr;
    //dataPtr.reserve(STN_DATA_MAX_SIZE);
#endif
    char packHead[STN_UPLOAD_SEND_LEN+1];
    int retry = 0;
    char sTnTStamp[256]={0};
    time_t stnTS = (time_t)(smartThInst->payload.tstamp);
    struct timespec currTime;
    struct timespec startTime;
    memset(&startTime, 0, sizeof(startTime));
    memset(&currTime, 0, sizeof(currTime));

    //clock the start time
    clock_gettime(CLOCK_REALTIME, &startTime);
    RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):Uploading smart thumbnail!!! Time left %ld\n", __FILE__, __LINE__, timeLeft);
    stringifyEventDateTime(sTnTStamp, sizeof(sTnTStamp), smartThInst->payload.tstamp);

    while (true) {
	//clock the current time
	memset(&currTime, 0, sizeof(currTime));
	clock_gettime(CLOCK_REALTIME, &currTime);

	//Check for max retry or time limit and break if so
        if ( (retry >= STN_MAX_RETRY_COUNT) ||
	    ((currTime.tv_sec - startTime.tv_sec) >= timeLeft) ) {
            RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL", "%s(%d): Max retry count/time exceeded, Retry count %d Time spent %d. currTime.tv_sec %d startTime.tv_sec %d Upload failed!!!\n", __FILE__,__LINE__, retry, (currTime.tv_sec - startTime.tv_sec), currTime.tv_sec, startTime.tv_sec);
	    ret = STH_ERROR;
            break;
        }

	if (0 == retry) {

#ifdef USE_FILE_UPLOAD
            /* get file attribute */
            if (stat(uploadFname, &fileStat) < 0) {
                RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): invalid file [%s], errmsg=%s!!!\n",__FILE__, __LINE__, uploadFname, strerror(errno));
		ret = STH_ERROR;
	        break;
            }

            fileLen = fileStat.st_size;
            RDK_LOG(RDK_LOG_INFO, "LOG.RDK.SMARTTHUMBNAIL", "%s(%d):Length of the smart thumbnail file to be uploaded: %d!!!\n", __FILE__, __LINE__,fileLen);

            fd = open(uploadFname, O_RDONLY);
            if (fd <= 0) {
            	RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Failed to Open smart thumbnail upload File :%s !!!\n", __FILE__, __LINE__, uploadFname);
		ret = STH_ERROR;
	    	break;
            }

            data =(char*)malloc(fileLen*sizeof(char));
            if (NULL == data) {
            	RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Failed to allocate memory :%s !!!\n", __FILE__, __LINE__, uploadFname);
            	close(fd);
		ret = STH_ERROR;
	    	break;
            }

            memset(data,0,fileLen);
            memset(readBuf,0,STN_UPLOAD_SEND_LEN);
            ptr=data;

	    // read thumbnail file into buffer
            while((readLen = read(fd, readBuf, sizeof(readBuf))) > 0) {
            	memcpy(ptr, readBuf, readLen);
            	ptr += readLen;
            	memset(readBuf,0,STN_UPLOAD_SEND_LEN);
            }
#else
	    //dataLen = payload.objFrame.total() * payload.objFrame.elemSize();
            //dataPtr = reinterpret_cast<char*>(payload.objFrame.data);
	    //cv::imencode(".jpg", payload.objFrame, dataPtr);
	    //dataLen = dataPtr.size();

#endif
	}

        /* Add Header */
        httpClient->resetHeaderList();
        httpClient->addHeader( "Expect", "");   //removing expect header condition by explicitly setting Expect header to ""
        memset(packHead, 0, sizeof(packHead));
        snprintf(packHead, sizeof(packHead), "%s", smtTnAuthCode);
        httpClient->addHeader( "Authorization", packHead);
        memset(packHead, 0, sizeof(packHead));
        snprintf(packHead, sizeof(packHead), "image/jpeg");
        httpClient->addHeader( "Content-Type", packHead);
        memset(packHead, 0, sizeof(packHead));
        snprintf(packHead, sizeof(packHead), "Sercomm %s %s %s", modelName, firmwareName, macAddress);
        httpClient->addHeader( "User-Agent", packHead);
        memset(packHead, 0, sizeof(packHead));
        snprintf(packHead, sizeof(packHead), "%s", sTnTStamp);
        httpClient->addHeader( "X-EVENT-DATETIME", packHead);
#ifdef _HAS_DING_	
	if(payload.dingtstamp)
        {
	    char dTnTStamp[256]={0};
    	    stringifyEventDateTime(dTnTStamp, sizeof(dTnTStamp), smartThInst->payload.dingtstamp);
	    memset(packHead, 0, sizeof(packHead));
            snprintf(packHead, sizeof(packHead), "%s", dTnTStamp);
       	    smartThInst->httpClient->addHeader( "X-DING-EVENT-TIME", packHead);
	    RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Ding: X-DING-EVENT-TIME: %s\n",__FUNCTION__,__LINE__,dTnTStamp);
        }
#endif
        memset(packHead, 0, sizeof(packHead));
        //SMTN's Bounding box of Motion Detection area
        snprintf(packHead, sizeof(packHead), "%d, %d, %d, %d", relativeBBox.x, relativeBBox.y, relativeBBox.width, relativeBBox.height);
        smartThInst->httpClient->addHeader( "X-BoundingBox", packHead);

        memset(packHead, 0, sizeof(packHead));

        //SMTN's objectBoxs of Motion Detction area
        for(int32_t i =0; i< UPPER_LIMIT_BLOB_BB; i++)
        {
            if((smartThInst->objectBoxs[i].boundingBoxXOrd) == INVALID_BBOX_ORD)
                break;

            memset(objectBoxsBuf, 0 , sizeof(objectBoxsBuf));
            snprintf(objectBoxsBuf, sizeof(objectBoxsBuf), "(%d, %d, %d, %d)," , smartThInst ->objectBoxs[i].boundingBoxXOrd, smartThInst ->objectBoxs[i].boundingBoxYOrd , smartThInst ->objectBoxs[i].boundingBoxWidth, smartThInst ->objectBoxs[i].boundingBoxHeight );
            strcat(packHead, objectBoxsBuf);
        }

        memset(smartThumbBuf, 0 , sizeof(smartThumbBuf));
        snprintf(smartThumbBuf, sizeof(smartThumbBuf), "(%d, %d, %d, %d)", smartThumbCoord.x, smartThumbCoord.y, smartThumbCoord.width, smartThumbCoord.height);
        smartThumbBuf[strlen(smartThumbBuf)] = '\0';
        strcat(packHead, smartThumbBuf);
        packHead[strlen(packHead)] = '\0';

        memset(objectBoxsBuf, 0 , sizeof(objectBoxsBuf));
        strcpy(objectBoxsBuf, packHead);
        smartThInst->httpClient->addHeader("X-BoundingBoxes", packHead);

        memset(packHead, 0, sizeof(packHead));
	if(smartThInst->cvrEnabled)
	{
		snprintf(packHead, sizeof(packHead), "CVR");
	}
	else
	{
        	snprintf(packHead, sizeof(packHead), "OFF");
	}
        smartThInst->httpClient->addHeader("X-VIDEO-RECORDING", packHead);

#ifdef USE_FILE_UPLOAD
        memset(packHead, 0, sizeof(packHead));
        snprintf(packHead, sizeof(packHead), "%s", uploadFname);
        httpClient->addHeader( "X-FILE-NAME", packHead);
        memset(packHead, 0, sizeof(packHead));
        snprintf(packHead, sizeof(packHead), "%d",fileLen);
        httpClient->addHeader( "Content-Length", packHead);

        //upload data
        curlCode =  httpClient->post_binary(smtTnUploadURL, data, &response_code, fileLen);
#else
        memset(packHead, 0, sizeof(packHead));
        snprintf(packHead, sizeof(packHead), "%d",dataLen);
        httpClient->addHeader( "Content-Length", packHead);

        //curlCode =  httpClient->post_binary(smtTnUploadURL, reinterpret_cast<char*> (&dataPtr[0]), &response_code, dataLen);
        curlCode =  httpClient->post_binary(smtTnUploadURL, dataPtr, &response_code, dataLen);
#endif
        RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): stn upload url is %s\n", __FUNCTION__, __LINE__, smtTnUploadURL);

        if ((response_code >= RDKC_HTTP_RESPONSE_OK) && (response_code < RDKC_HTTP_RESPONSE_REDIRECT_START)) {
            clock_gettime(CLOCK_REALTIME, &currTime);
            RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Smart Thumbnail uploaded successfully with header X-EVENT-DATETIME: %s X-BoundingBox: %d %d %d %d  X-VIDEO-RECORDING :OFF  X-BoundingBoxes: %s\n", __FUNCTION__, __LINE__, sTnTStamp, relativeBBox.x, relativeBBox.y, relativeBBox.width, relativeBBox.height, objectBoxsBuf);
            RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): StnTimestamp,CurrentTimestamp,Latency:%ld,%ld,%ld\n",__FUNCTION__,__LINE__, stnTS, currTime.tv_sec, (currTime.tv_sec-stnTS));
            break;

        } else {
            RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Smart Thumbnail upload failed, with response code:%lu!!!\n",__FUNCTION__,__LINE__, response_code);
            retry++;
	    RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Retrying again for %d times!!!\n",__FUNCTION__,__LINE__,retry);
        }
    }

#ifdef USE_FILE_UPLOAD
        if(NULL != data) {
            free(data);
            data = NULL;
        }
        close(fd);
        RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Removing smart thumbnail upload file: %s\n",__FILE__, __LINE__, uploadFname);
	unlink (uploadFname);
#endif
    return ret;
}


/** @description : thread routine to listen to messages
 *  @param[in] : void.
 *  @return : void
 */
void SmartThumbnail::receiveRtmessage()
{
    while (!smartThInst -> rtmessageSTHThreadExit) {
	smartThInst -> err = rtConnection_Dispatch(smartThInst -> connectionRecv);
	if (smartThInst -> err != RT_OK) {
	    //rtLog_Debug("dispatch:%s", rtStrError(err));
	    RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): dispatch:%s",__FUNCTION__,__LINE__,rtStrError(smartThInst -> err));
	    RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Error receiving msg via rtmessage\n",__FUNCTION__,__LINE__);
	}
	usleep(10000);
    }
}

/**
 * @description: This function is used to get the camera firmware version.
 *
 * @param[out]: void
 *
 * @return: Error code.
 */
int SmartThumbnail::setCameraImageName(char *out)
{
	size_t max_line_length = FW_NAME_MAX_LENGTH;
	char *file_buffer;
	char *locate_1 = NULL;
	FILE* fp;
	char* temp = out;

	fp = fopen("/version.txt","r");
	if (NULL == fp) {
		RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.CVRUPLOAD","%s(%d): Error in opening version.txt \n", __FILE__, __LINE__);
		return RDKC_FAILURE;
	}

	file_buffer = (char*)malloc(FW_NAME_MAX_LENGTH + 1);
	if(file_buffer == NULL)
	{
		RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.CVRUPLOAD","%s(%d): Error in malloc \n", __FILE__, __LINE__);
		fclose(fp);
		return RDKC_FAILURE;
	}

	while(getline(&file_buffer,&max_line_length,fp) != -1)
	{
		/* find the imagename string */
		locate_1 = strstr(file_buffer,"imagename");
		if(locate_1)
		{
			locate_1 += strlen("imagename:");
			/* copy the contents till linefeed */
			while(*locate_1 != '\n')
				*out++ = *locate_1++;
			free(file_buffer);
			file_buffer = NULL;
			fclose(fp);
			return RDKC_SUCCESS;
		}
	}

	/* unable to get the image name */
	strcpy(out,"imagename entry not found");
	free(file_buffer);
	file_buffer = NULL;
	fclose(fp);
	return RDKC_SUCCESS;
}

/**
 * @description: This function is used to set the camera firmware version.
 * @param[out]: void
 * @return: Error code.
* */
int SmartThumbnail::setModelName()
{
#ifdef USE_MFRLIB
    mfrSerializedData_t stdata = {NULL, 0, NULL};
    mfrSerializedType_t stdatatype = mfrSERIALIZED_TYPE_MODELNAME;
    char mac[CONFIG_STRING_MAX];

    if (mfrGetSerializedData(stdatatype, &stdata) == mfrERR_NONE) {
	strncpy(modelName,stdata.buf,stdata.bufLen);
	modelName[stdata.bufLen] = '\0';
	RDK_LOG( RDK_LOG_INFO,"LOG.RDK.CVRPOLL","%s(%d):Model Name = %s,%s,%d\n",__FILE__, __LINE__,modelName,stdata.buf,stdata.bufLen);
	if (stdata.freeBuf != NULL) {
		stdata.freeBuf(stdata.buf);
		stdata.buf = NULL;
	}
    }
    else {
	RDK_LOG( RDK_LOG_INFO,"LOG.RDK.CVRPOLL","%s(%d):GET ModelName failed : %d\n", __FILE__, __LINE__);
    }
#endif
}

/**
 * @description: This function is used to set the camera's mac address.
 * @param[out]: void
 * @return: Error code.
* */
int SmartThumbnail::setMacAddress()
{
	char mac[CONFIG_STRING_MAX] = {0};

#ifdef USE_MFRLIB
	mfrSerializedData_t stdata = {NULL, 0, NULL};
	mfrSerializedType_t stdatatype = mfrSERIALIZED_TYPE_DEVICEMAC;

	if (mfrGetSerializedData(stdatatype, &stdata) == mfrERR_NONE) {
		strncpy(mac,stdata.buf,stdata.bufLen);
		mac[stdata.bufLen] = '\0';
		RDK_LOG( RDK_LOG_INFO,"LOG.RDK.CVRPOLL","%s(%d):mac= %s,%s,%d\n",__FILE__, __LINE__,mac,stdata.buf,stdata.bufLen);

		char tmpMac[CONFIG_STRING_MAX+1] = {0};
		char *tmpField;
		int fieldNum = 0;

		strcpy(tmpMac, mac);
		tmpField = strtok(tmpMac, ":");

		while (tmpField != NULL && fieldNum < 6) {
			char *chk;
			unsigned long tmpVal;

			tmpVal = strtoul(tmpField, &chk, 16);

			if (tmpVal > 0xff) {
				RDK_LOG( RDK_LOG_WARN,"LOG.RDK.CVRPOLL","field %d value %0x out of range\n", fieldNum, tmpVal);
			}
			if (*chk != 0) {
				RDK_LOG( RDK_LOG_WARN,"LOG.RDK.CVRPOLL","Non-digit character %c (%0x) detected in field %d\n", *chk, *chk, fieldNum);

			}
			fieldNum++;
			strcat(macAddress, tmpField);
			tmpField = strtok(NULL, ":");
		}
		RDK_LOG( RDK_LOG_INFO,"LOG.RDK.CVRPOLL","%s(%d):mac address= %s\n",__FILE__, __LINE__,macAddress);
		if (stdata.freeBuf != NULL) {
			stdata.freeBuf(stdata.buf);
			stdata.buf = NULL;
		}

	}
	else {
		RDK_LOG( RDK_LOG_INFO,"LOG.RDK.CVRPOLL","%s(%d):GET MAC failed : %d\n", __FILE__, __LINE__);
	}
#endif
}



/** @description   : update ObjectBoxs data for smart thumbnail upload
 *  @param[in]     : objectBox
 *  @param[in]     : index
 *  @return        : void
 */
void SmartThumbnail::updateObjectBoxs(BoundingBox *objectBox, int32_t index)
{
    smartThInst -> objectBoxs[index].boundingBoxXOrd = objectBox->boundingBoxXOrd;
    smartThInst -> objectBoxs[index].boundingBoxYOrd = objectBox->boundingBoxYOrd;
    smartThInst -> objectBoxs[index].boundingBoxWidth = objectBox->boundingBoxWidth;
    smartThInst -> objectBoxs[index].boundingBoxHeight = objectBox->boundingBoxHeight;

}

/** @description    : update SmarttnMetadat_thumb details to sm from the rtMessage m
 *  @param[in]      : smInfo
 *  @param[in]      : rtMessage m
 *  @return         : void
 */
void SmarttnMetadata_thumb::from_rtMessage(SmarttnMetadata_thumb *smInfo, const rtMessage m)
{
    int32_t len =0;

    if(!smInfo) {
        RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Error smInfo should not be NULL\n",__FUNCTION__,__LINE__);
        return;
    }

    rtMessage_GetString(m, "timestamp", &(smInfo->strFramePTS));
    rtMessage_GetInt32(m, "event_type", &(smInfo->event_type));
    rtMessage_GetDouble(m, "motionScore", &(smInfo->motionScore));
    rtMessage_GetString(m, "currentTime", &(smInfo->s_curr_time));
    //unionBox
    rtMessage_GetInt32(m, "boundingBoxXOrd", &(smInfo->unionBox.boundingBoxXOrd));
    rtMessage_GetInt32(m, "boundingBoxYOrd", &(smInfo->unionBox.boundingBoxYOrd));
    rtMessage_GetInt32(m, "boundingBoxWidth", &(smInfo->unionBox.boundingBoxWidth));
    rtMessage_GetInt32(m, "boundingBoxHeight", &(smInfo->unionBox.boundingBoxHeight));

    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): strFramePTS:%s \n", __FUNCTION__ , __LINE__, smInfo->strFramePTS);
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): event Type: %d\n",__FILE__, __LINE__, smInfo->event_type);
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): motionScore: %f\n",__FILE__, __LINE__, smInfo->motionScore);
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): boundingBoxXOrd:%d\n", __FILE__, __LINE__, smInfo->unionBox.boundingBoxXOrd);
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): boundingBoxYOrd:%d\n", __FILE__, __LINE__, smInfo->unionBox.boundingBoxYOrd);
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): boundingBoxWidth:%d\n", __FILE__, __LINE__, smInfo->unionBox.boundingBoxWidth);
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): boundingBoxHeight:%d\n", __FILE__, __LINE__, smInfo->unionBox.boundingBoxHeight);
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): curr timestamp:%s\n", __FILE__, __LINE__, smInfo->s_curr_time);

    rtMessage_GetArrayLength(m, "objectBoxs", &len);

    //objectBoxs
    for (int32_t i = 0; i < len; i++)
    {
        rtMessage bbox;
        rtMessage_GetMessageItem(m, "objectBoxs", i, &bbox);
        rtMessage_GetInt32(bbox, "boundingBoxXOrd", &(smInfo->objectBoxs[i].boundingBoxXOrd));
        rtMessage_GetInt32(bbox, "boundingBoxYOrd", &(smInfo->objectBoxs[i].boundingBoxYOrd));
        rtMessage_GetInt32(bbox, "boundingBoxWidth", &(smInfo->objectBoxs[i].boundingBoxWidth));
        rtMessage_GetInt32(bbox, "boundingBoxHeight", &(smInfo->objectBoxs[i].boundingBoxHeight));

        RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): objectBoxs[%d].boundingBoxXOrd:%d\n", __FILE__, __LINE__,i,smInfo->objectBoxs[i].boundingBoxXOrd);
        RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): objectBoxs[%d].boundingBoxYOrd:%d\n", __FILE__, __LINE__,i,smInfo->objectBoxs[i].boundingBoxYOrd);
        RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): objectBoxs[%d].boundingBoxWidth:%d\n", __FILE__, __LINE__,i,smInfo->objectBoxs[i].boundingBoxWidth);
        RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): objectBoxs[%d].boundingBoxHeight:%d\n", __FILE__, __LINE__,i,smInfo->objectBoxs[i].boundingBoxHeight);
        rtMessage_Release(bbox);
    }


}
/* @description: SmarttnMetadata_thumb constructor
 * @parametar: void
 * @return: void
 */
SmarttnMetadata_thumb::SmarttnMetadata_thumb()
{
    strFramePTS = NULL;
    event_type = 0;
    motionScore = 0;
    memset(&unionBox, 0, sizeof(unionBox));
    memset(objectBoxs, INVALID_BBOX_ORD, sizeof(objectBoxs));
    s_curr_time = NULL;
}


