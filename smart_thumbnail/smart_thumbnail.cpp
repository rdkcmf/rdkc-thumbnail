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

SmartThumbnail* SmartThumbnail::smartThInst = NULL;
volatile bool SmartThumbnail::termFlag = false;
volatile bool SmartThumbnail::tnUploadConfRefreshed = false;
int SmartThumbnail::waitingInterval = 1 ;

/** @description: Constructor
 *  @param[in] void
 *  @return: void
 */
//SmartThumbnail::SmartThumbnail():hres_yuvData(NULL),
SmartThumbnail::SmartThumbnail():
#ifdef _HAS_XSTREAM_
				consumer(NULL),
				frameInfo(NULL),
#else
				recorder(NULL),
#endif
				httpClient(NULL),
				hres_y_size(0),
				hres_uv_size(0),
				hres_y_height(0),
				hres_y_width(0),
				maxBboxArea(0),
				isHresFrameReady(false),
				event_quiet_time(STN_DEFAULT_EVT_QUIET_TIME),
				uploadReady(false),
				isPayloadAvailable(false),
				cvrClipGenStarted(false),
				dnsCacheTimeout(STN_DEFAULT_DNS_CACHE_TIMEOUT),
				sTnHeight(STN_DEFAULT_HEIGHT),
				sTnWidth(STN_DEFAULT_WIDTH)
{
    memset(uploadFname, 0, sizeof(uploadFname));
    memset(smtTnUploadURL, 0, sizeof(smtTnUploadURL));
    memset(smtTnAuthCode, 0, sizeof(smtTnAuthCode));
    memset(currSTNFname, 0, sizeof(currSTNFname));
#ifdef _HAS_XSTREAM_
#ifndef _DIRECT_FRAME_READ_
	frameHandler = {NULL, -1};
	//frameHandler.curl_handle = NULL;
	//frameHandler.sockfd = -1;
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
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Smart thumbnail destructor invoked.\n", __FUNCTION__, __LINE__);
}

void SmartThumbnail::sigHandler(int signum){
    if(SIGTERM == signum){
        termFlag = true;
    } else if( SIGINT == signum){
        termFlag = true;
    }
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
STH_STATUS SmartThumbnail::init()
{

#ifdef LEGACY_CFG_MGR
    char usrVal[CONFIG_STRING_MAX];
    char *configParam = NULL;

    //initializing config Manager
    if (STH_SUCCESS != config_init()) {
	RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Error loading config manager.\n", __FILE__, __LINE__);
	return STH_ERROR;
    }

    // get upload url
    memset(usrVal, 0, sizeof(usrVal));
    if (STH_SUCCESS != rdkc_get_user_setting(SMART_THUMBNAIL_UPLOAD_URL,usrVal)) {
	configParam=(char*)rdkc_envGet(SMART_THUMBNAIL_UPLOAD_URL);
    } else {
	configParam =usrVal;
    }

    if (NULL != configParam) {
	strcpy(smtTnUploadURL,configParam);
	configParam = NULL;
    }

   // get auth code
    memset(usrVal, 0, sizeof(usrVal));
    if (STH_SUCCESS != rdkc_get_user_setting(SMART_THUMBNAIL_AUTH_CODE ,usrVal)) {
	configParam=(char*)rdkc_envGet(SMART_THUMBNAIL_AUTH_CODE );
    } else {
	configParam =usrVal;
    }
    if (NULL != configParam) {
	strcpy(smtTnAuthCode,configParam);
	configParam = NULL;
    }
#else
    if(STH_SUCCESS == getTnUploadConf()) {
    	RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): getTnUploadConf success!!", __FUNCTION__, __LINE__);
    }

    if(STH_SUCCESS == getEventConf()) {
    	RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): getEventConf success!!", __FUNCTION__, __LINE__);
    }

#endif

    // update mac/model/camera image
#if 0
    memset(modelName, 0, sizeof(modelName));
    memset(macAddress, 0, sizeof(macAddress));
    memset(firmwareName, 0,sizeof(firmwareName));
    setMacAddress();
    setModelName();
    setCameraImageName(firmwareName);
#endif

    RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): smart_thumbnail upload URL: %s \n",__FUNCTION__, __LINE__,smtTnUploadURL);
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): smart_thumbnail auth code: %s \n",__FUNCTION__, __LINE__,smtTnAuthCode);
    RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): smart_thumbnail event quiet time: %d \n",__FUNCTION__, __LINE__,event_quiet_time);
    RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): smart_thumbnail height: %d \n",__FUNCTION__, __LINE__,sTnHeight);
    RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): smart_thumbnail width: %d \n",__FUNCTION__, __LINE__,sTnWidth);

#ifdef _HAS_XSTREAM_
    //Create the consumer class object
    consumer = new XStreamerConsumer;
    if (!consumer){
          RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Error creating instance of xstreamerConsumer.\n", __FUNCTION__, __LINE__);
		  return STH_ERROR;
    }

    //initialize
#ifdef _DIRECT_FRAME_READ_
    if(STH_SUCCESS != smartThInst->consumer->RAWInit((u16)STN_HRES_BUFFER_ID)){
#else
    frameHandler = smartThInst->consumer->RAWInit(STN_HRES_BUFFER_ID, FORMAT_YUV, 1);
    if(frameHandler.sockfd < 0) {
#endif
        RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Failed to initialize resources for reading raw frames.\n", __FUNCTION__, __LINE__);
		return STH_ERROR;
    }

    frameInfo = consumer->GetRAWFrameContainer();
    if( NULL == frameInfo ){
        RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Failed to create applicaton buffer\n", __FUNCTION__, __LINE__);
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
    hres_frame_info = (RDKC_PLUGIN_YUVInfo *) malloc(sizeof(RDKC_PLUGIN_YUVInfo));
    if (NULL == hres_frame_info) {
        RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):Frame malloc error for high res frame info. \n", __FUNCTION__ , __LINE__);
        return STH_ERROR;
    }
#endif

    //initialize http client
    httpClient = new HttpClient();

    /* Open the URL */
    if (NULL != httpClient) {
	//httpClient->open(smtTnUploadURL, dnsCacheTimeout, STN_MAX_UPLOAD_TIMEOUT);
        httpClient->open(smtTnUploadURL, dnsCacheTimeout);
    } else {
	RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Failed to open the URL\n", __FUNCTION__, __LINE__);
    }

    rtMsgInit();

    //Initialize upload routine
    RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Creating smart thumbnail upload thread.\n", __FUNCTION__, __LINE__);
    std::thread uploadPayloadThread(uploadSTN);
    uploadPayloadThread.detach();

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
	event_quiet_time = atoi(eventsCfg->quite_interval);
    }

    if (eventsCfg) {
        free(eventsCfg);
        eventsCfg = NULL;
    }

    polling_config_exit();

    return STH_SUCCESS;
}

/** @description: saves smart thumbnail to file
 *  @param[in] : void
 *  @return: STH_SUCCESS on success, STH_ERROR otherwise
 */
STH_STATUS SmartThumbnail::saveSTN()
{
    //std::pair<cv::Mat, unsigned long long> objPair;

    STH_STATUS ret = STH_ERROR;

    cv::Rect unionBox;
    cv::Mat lHresRGBMat;
    cv::Mat croppedObj;

    char fPath [64] = {0};
    //cv::Mat resizedCroppedObject;
    cv::Mat resizedRGBMat;
#ifdef USE_FILE_UPLOAD
    struct tm* tv = NULL;
    struct timespec currTime;
#endif
    //{
	//Acquire lock
	//std::unique_lock<std::mutex> lock(stnMutex);
	if (isPayloadAvailable) {
            RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):Payload is available.\n", __FILE__, __LINE__);
	    currSTN.tstamp   = ofData.currTime;

	    // matrix to store color image
	    lHresRGBMat = cv::Mat(hres_y_height, hres_y_width, CV_8UC4);
	    // convert the frame to BGR format
	    cv::cvtColor(ofData.maxBboxObjYUVFrame,lHresRGBMat, cv::COLOR_YUV2BGR_NV12);

	    unionBox.x = ofData.boundingBoxXOrd;
	    unionBox.y = ofData.boundingBoxYOrd;
	    unionBox.width = ofData.boundingBoxWidth;
	    unionBox.height = ofData.boundingBoxHeight;

	    // extracted the below logic from server scala code
            RDK_LOG( RDK_LOG_TRACE1,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):unionBox.x %d unionBox.y %d unionBox.height %d unionBox.width %d\n", __FILE__, __LINE__, unionBox.x, unionBox.y, unionBox.height, unionBox.width);
	    cv::Point2f orgCenter = getActualCentroid(unionBox);
	    cv::Size cropSize = getCropSize(unionBox, sTnWidth, sTnHeight);
	    cv::Point2f allignedCenter =  alignCentroid(orgCenter, lHresRGBMat, cropSize);
	    getRectSubPix(lHresRGBMat, cropSize, allignedCenter, croppedObj);
	    relativeBBox = getRelativeBoundingBox(unionBox, cropSize, allignedCenter);

#if 0
	    // create cropped object
            RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):before cropping object frame.\n", __FILE__, __LINE__);
            croppedObj = lHresRGBMat(unionBox).clone();
            RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):after cropping object frame.\n", __FILE__, __LINE__);
            // Resize object to fixed resolution, keep aspect ratio
            RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):before resizing object frame.\n", __FILE__, __LINE__);
            resizeAspect(croppedObj, sTnWidth, sTnHeight, resizedCroppedObject);
            RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):after resizing object frame.\n", __FILE__, __LINE__);

            payload.objFrame = resizedCroppedObject.clone();
	    //resize the RGG data to required size.
	    cv::resize(lHresRGBMat,resizedRGBMat,cv::Size(sTnWidth,sTnHeight));
	    payload.objFrame = resizedRGBMat.clone();
#endif
            //RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):Resized the cropped object frame.\n", __FILE__, __LINE__);
	    //cv::resize(croppedObj, resizedRGBMat,cv::Size(sTnWidth, sTnHeight));
	    strcpy(currSTN.fname, currSTNFname);

	    snprintf(fPath, sizeof(fPath), "%s/%s", STN_PATH, currSTNFname);
            RDK_LOG( RDK_LOG_TRACE1,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):smart thumbnail full path:%s .\n", __FILE__, __LINE__, fPath);
#ifdef USE_FILE_UPLOAD
            //Write smart thumbnail to file.
            //imwrite(fPath, resizedRGBMat);
            imwrite(fPath, croppedObj);
#endif

	    ret = STH_SUCCESS;
    	}

	ofData.maxBboxObjYUVFrame.release();
	lHresRGBMat.release();
	croppedObj.release();
	resizedRGBMat.release();

	// else {
    	//    RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):Nothing to save.\n", __FILE__, __LINE__);
	//    ret = STH_NO_SAVE;
        //}
	//Release lock
	//lock.unlock();
    //}

    return ret;
}

/** @description: Logs the smart thumbnail list to be uploaded.
 *  @param[in] : void
 *  @return: STH_SUCCESS on success, STH_ERROR otherwise
 */
void SmartThumbnail::printSTNList()
{

    RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):STN list size:%d\n", __FILE__, __LINE__, STNList.size());
    RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):STN fname  ->  STN tstamp", __FILE__, __LINE__);
    //vector<STHPayload>::iterator it;
    for (std::vector<STHPayload>::iterator it = STNList.begin(); it != STNList.end(); ++it) {
    	RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):		%s  -> %llu \n", __FILE__, __LINE__, (*it).fname, (*it).tstamp);
    }
}

/** @description: adds smart thumbnail to list for uploading.
 *  @param[in] : void
 *  @return: STH_SUCCESS on success, STH_ERROR otherwise
 */
STH_STATUS SmartThumbnail::addSTN()
{
    char stnFname[256] ={0};

    std::unique_lock<std::mutex> lock(stnMutex);
    if(STNList.size() == STN_MAX_PAYLOAD_COUNT) {
	//delete the files in the list
	int ctr = 0;
	while(ctr < STN_MAX_PAYLOAD_COUNT) {
	    std::vector<STHPayload>::iterator it = STNList.begin();
	    memset(stnFname, 0 , sizeof(stnFname));
	    snprintf(stnFname, sizeof(stnFname), "%s/%s", STN_PATH, (*it).fname);
	    RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","addSTN %s(%d) Deleting %s file, and erasing from list\n", __FUNCTION__ , __LINE__, stnFname);
	    unlink(stnFname);
	    STNList.erase(it);
	    ctr++;
	}
    }

    STNList.push_back(currSTN);
    //printSTNList();
    lock.unlock();

    return STH_SUCCESS;
}

/** @description: deletes smart thumbnail from list for uploading.
 *  @param[in] : uploadFname
 *  @return: STH_SUCCESS on success, STH_ERROR otherwise
 */
STH_STATUS SmartThumbnail::delSTN(char* uploadFname)
{
    const char* stnFname  = strrchr(uploadFname, '/');
    struct stat statbuf;

    if(stnFname) {
	stnFname++;
    } else {
	stnFname = uploadFname;
    }

    std::unique_lock<std::mutex> lock(stnMutex);
    //vector<STHPayload>::iterator it;
    for (std::vector<STHPayload>::iterator it = STNList.begin(); it != STNList.end(); ++it) {
	if(!strcmp((*it).fname, stnFname)){

	    //delete the file first
	    if(stat(CACHE_SMARTTHUMBNAIL, &statbuf) < 0) {
	        RDK_LOG(RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d) Deleting %s file, and erasing from list\n", __FUNCTION__ , __LINE__, uploadFname);
	        unlink(uploadFname);
	    }
	    else {
	        RDK_LOG(RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d) Caching %s file for reference\n", __FUNCTION__ , __LINE__, uploadFname);
	    }
	    STNList.erase(it);
	    break;

	}
    }
    lock.unlock();
    return STH_SUCCESS;
}

/** @description: creates the payload STN list for uploading.
 *  @param[in] : void
 *  @return: STH_SUCCESS on success, STH_ERROR otherwise
 */
STH_STATUS SmartThumbnail::createPayload(char* uploadFname)
{
    STH_STATUS ret = STH_NO_PAYLOAD;

    std::unique_lock<std::mutex> lock(stnMutex);
    if( STNList.size() != 0) { 
    	for (std::vector<STHPayload>::iterator it = STNList.begin(); it != STNList.end(); ++it) {
       	    if(!strcmp((*it).fname, uploadFname)){
		memset(&payload, 0 , sizeof(STHPayload));
		memcpy(&payload, &(*it), sizeof(STHPayload));
		ret =  STH_SUCCESS;
		RDK_LOG(RDK_LOG_TRACE1,"LOG.RDK.SMARTTHUMBNAIL","%s(%d) payload.fname:%s  payload.tstamp :%llu \n", __FUNCTION__ , __LINE__, payload.fname,payload.tstamp);
		break;
       	    }
    	}
    }

    lock.unlock();
    return ret;
}

/** @description: get upload status.
 *  @param[in] : void
 *  @return: true if payload is ready, false otherwise
 */
bool SmartThumbnail::getUploadStatus()
{
    bool status = false;

    {
	std::unique_lock<std::mutex> lock(uploadMutex);
	cv.wait(lock, [this] {return (uploadReady || termFlag);});

	if(!termFlag) {
	    RDK_LOG( RDK_LOG_TRACE1,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Wait over due uploadReady flag!!\n",__FUNCTION__,__LINE__);
	    status = uploadReady;
	    uploadReady = false;
    	} else {
	    RDK_LOG( RDK_LOG_TRACE1,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Wait over due term flag!!\n",__FUNCTION__,__LINE__);
	    status = false;
	}

	lock.unlock();
    }

    return status;
}

/** @description: set upload status.
 *  @param[in] : status
 *  @return: STH_SUCCESS on success, STH_ERROR otherwise
 */
STH_STATUS SmartThumbnail::setUploadStatus(bool status)
{
    {
	std::unique_lock<std::mutex> lock(uploadMutex);
 	uploadReady = status;
	lock.unlock();
    }

    cv.notify_one();
    return STH_SUCCESS;
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

    /*cout << "\n\n Original Center { " << orgCenter.x << ", " << orgCenter.y << " } " << "Alligned Center: { " << adjustedXfinal << ", " << adjustedYfinal << " } \n";
    cout << " Cropping Resolution {W, H}:  { " << cropSize.width << ", " << cropSize.height << " } " << "Original frame Resolution {W, H}: { " << origFrame.cols << ", " << origFrame.rows << " } \n";
    cout << " Intermediate Adjustments {shiftX, adjustedX, shiftXleft}: { " << shiftX << ", " << adjustedX << ", " << shiftXleft << " } \n";
    cout << " Intermediate Adjustments {shiftY, adjustedY, shiftYdown}: { " << shiftY << ", " << adjustedY << ", " << shiftYdown << " } \n\n";*/

    return pts;
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
#if 0
    // always ensure the minimum size of the crop size window is {w, h}
    sz.height = STN_MAX(h,(newHeight* adjustFactor));
    sz.width = STN_MAX(w,(newWidth* adjustFactor));
#endif
    sz.height = h;
    sz.width = w;
    return sz;
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

    rtError err = rtConnection_SendMessage(smartThInst -> connectionSend, req, "RDKC.SMARTTN.STATUS");
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

    //Exit the rtmessage receive and upload thread.
    //rtmessageSTHThreadExit =  true;
    //uploadSTHThreadExit = true;

	STH_STATUS ret = STH_SUCCESS;

#ifdef _HAS_XSTREAM_
#ifdef _DIRECT_FRAME_READ_
    if( (NULL != consumer) && (STH_ERROR != consumer->RAWClose()) ){
#else
    //Destroy the curl handle
    if( (NULL != consumer) && (STH_ERROR != consumer->RAWClose(frameHandler.curl_handle)) ){
#endif
        RDK_LOG(RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d) StreamClose Successful\n", __FUNCTION__ , __LINE__);
    }
    else{
        RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d) StreamClose Failed\n", __FUNCTION__ , __LINE__);
        ret = STH_ERROR;
    }

    //Destroy the XStreamerConsumer instance
    if(consumer){
        delete consumer;
        consumer = NULL;
    }

	frameInfo = NULL;
#ifndef _DIRECT_FRAME_READ_
	frameHandler.curl_handle = NULL;
	frameHandler.sockfd = -1;
#endif
#else
	if (hres_frame_info) {
		free(hres_frame_info);
		hres_frame_info = NULL;
	}
#endif

	//Delete the smart thumbnail instance
	RDK_LOG(RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d) Deleting the smart thumbnail instance!!!\n", __FUNCTION__ , __LINE__);
	if (smartThInst) {
		delete smartThInst;
		smartThInst =NULL;
	}

#ifdef LEGACY_CFG_MGR
	config_release();
#endif

	return ret;
}

/** @description    : Callback function to generate smart thumbnail based on motion.
 *  @param[in]  hdr : constant pointer rtMessageHeader
 *  @param[in] buff : constant pointer uint8_t
 *  @param[in]    n : uint32_t
 *  @param[in] closure : void pointer
 *  @return: void
 */
void SmartThumbnail::onMsgRefreshTnUpload(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure)
{
    char const*  status = NULL;

    rtConnection con = (rtConnection) closure;

    rtMessage req;
    rtMessage_FromBytes(&req, buff, n);

    char* tempbuff = NULL;
    uint32_t buff_length = 0;

    rtMessage_ToString(req, &tempbuff, &buff_length);
    rtLog_Debug("Req : %.*s", buff_length, tempbuff);
    free(tempbuff);

    rtMessage_GetString(req, "status", &status);

    RDK_LOG(RDK_LOG_INFO,"LOG.RDK.CVR","(%s):%d status:%s\n", __FUNCTION__, __LINE__, status);

    if(!strcmp(status, "refresh")) {
        tnUploadConfRefreshed = true;
    } else {
        tnUploadConfRefreshed = false;
    }

    rtMessage_Release(req);
}

/** @description: Callback function for CVR clip gen status
 *  @param[in] hdr : pointer to rtMessage Header
 *  @param[in] buff : buffer for data received via rt message
 *  @param[in] n : number of bytes received
 *  @return: void
 */
void SmartThumbnail::onMsgCvr(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure)
{
    int clipGenStatus = -1;
    char const*  cvrClipFname = NULL;
    char const*  cvrEvtTstamp = NULL;
    uint64_t cvrEventTS = 0;
    const char* delimPos = 0;
    rtConnection con = (rtConnection) closure;

    rtMessage req;
    rtMessage_FromBytes(&req, buff, n);

    rtMessage_GetInt32(req, "clipStatus", &clipGenStatus);
    rtMessage_GetString(req, "clipname" , &cvrClipFname);
    rtMessage_GetString(req, "eventTimeStamp" , &cvrEvtTstamp);

    RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","(%s):%d CVR clip status: %d\n", __FUNCTION__, __LINE__, clipGenStatus);
    RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","(%s):%d CVR clip name: %s\n", __FUNCTION__, __LINE__, cvrClipFname);
    RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","(%s):%d CVR event timestamp: %s\n", __FUNCTION__, __LINE__, cvrEvtTstamp);

    //take action on clip gen status
    if(clipGenStatus == CVR_CLIP_GEN_START) {
	memset(smartThInst->currSTNFname, 0, sizeof(smartThInst->currSTNFname));
	strcpy(smartThInst->currSTNFname, cvrClipFname);
	strcat(smartThInst->currSTNFname, ".jpg");
	RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","(%s):%d STN fname to be generated: %s\n", __FUNCTION__, __LINE__, smartThInst->currSTNFname);
	smartThInst -> cvrClipGenStarted = true;
	smartThInst->logMotionEvent = true;
	RDK_LOG(RDK_LOG_TRACE1,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Resetting the logMotionEvent to true \n", __FILE__ , __LINE__);

	//reset payload flag
	smartThInst->isPayloadAvailable = false;
	smartThInst->maxBboxArea = 0;

    } else if(clipGenStatus == CVR_CLIP_GEN_END) {

	if(!smartThInst->cvrClipGenStarted) {
		RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","(%s):%d \t2. Recevied CVR_CLIP_GEN_END before CVR_CLIP_GEN_START.\n", __FUNCTION__, __LINE__);
	}

	if((smartThInst->cvrClipGenStarted) &&
	   (smartThInst->isPayloadAvailable) &&
		// to check if motion in the clip
	   (0 != strcmp("TIMESTAMP_NOT_AVAILABLE", cvrEvtTstamp))) {
		//save smart thumbnail from memory to file
		smartThInst->saveSTN();
		smartThInst->addSTN();
		smartThInst->cvrClipGenStarted = false;

		std::istringstream iss(cvrEvtTstamp);
		iss >> cvrEventTS;
		RDK_LOG( RDK_LOG_TRACE1,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Motion Timestamp: %llu\n", __FUNCTION__, __LINE__, cvrEventTS);
		iss.clear();
		smartThInst->motion_time = cvrEventTS;
	}

	if(!smartThInst->isPayloadAvailable) {
		RDK_LOG(RDK_LOG_TRACE1,"LOG.RDK.SMARTTHUMBNAIL","(%s):%d \t3. Smart Thumbnail is not available during current period.\n", __FUNCTION__, __LINE__);
	}

	if(0 == strcmp("TIMESTAMP_NOT_AVAILABLE", cvrEvtTstamp)) {
		RDK_LOG(RDK_LOG_TRACE1,"LOG.RDK.SMARTTHUMBNAIL","(%s):%d \t4. Motion Timestamp is not available during current period.\n", __FUNCTION__, __LINE__);
	}


    } else {
	//RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","(%s):%d Ignoring CVR clip gen status.\n", __FUNCTION__, __LINE__);
	//RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","(%s):%d Possible Reason:\n", __FUNCTION__, __LINE__);

	RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","(%s):%d \t1. Invalid Clip Gen status.\n", __FUNCTION__, __LINE__);
    }

    rtMessage_Release(req);
}

/** @description: Callback function for CVR clip upload status
 *  @param[in] hdr : pointer to rtMessage Header
 *  @param[in] buff : buffer for data received via rt message
 *  @param[in] n : number of bytes received
 *  @return: void
 */
void SmartThumbnail::onMsgCvrUpload(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure)
{
    int clipUploadStatus = 0;
    char const*  cvrClipFname = NULL;
    const char* delimPos = 0;
    int createPayloadStatus = STH_NO_PAYLOAD;
    char stnFname[256] = {0};

    rtConnection con = (rtConnection) closure;

    rtMessage req;
    rtMessage_FromBytes(&req, buff, n);

    rtMessage_GetInt32(req, "status", &clipUploadStatus);
    rtMessage_GetString(req, "uploadFileName", &cvrClipFname);

    RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","(%s):%d CVR clip upload status: %d\n", __FUNCTION__, __LINE__, clipUploadStatus);
    RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","(%s):%d CVR clip upload file: %s\n", __FUNCTION__, __LINE__, cvrClipFname);

    memset(smartThInst->uploadFname, 0, sizeof(smartThInst->uploadFname));
    //delimPos = strchr(cvrClipFname, '.');
    //strncpy(uploadFname, cvrClipFname, delimPos-cvrClipFname+1);
    strcpy(smartThInst->uploadFname, cvrClipFname);
    strcat(smartThInst->uploadFname, ".jpg");
    snprintf(stnFname, sizeof(stnFname), "%s/%s", STN_PATH, smartThInst->uploadFname);

    //take action on clip upload status
    if(clipUploadStatus == CVR_UPLOAD_OK) {
    	RDK_LOG(RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","(%s):%d Generating payload, CVR clip %s upload success.\n", __FUNCTION__, __LINE__, cvrClipFname);
	createPayloadStatus =  smartThInst->createPayload(smartThInst->uploadFname);
	if(STH_SUCCESS == createPayloadStatus) {
    		RDK_LOG(RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","(%s):%d  payload created for CVR clip :%s.\n", __FUNCTION__, __LINE__, cvrClipFname);
		smartThInst->setUploadStatus(true);
	} else if (STH_NO_PAYLOAD == createPayloadStatus) {
    		RDK_LOG(RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","(%s):%d  No payload available for CVR clip:%s.\n", __FUNCTION__, __LINE__, cvrClipFname);
	} else {
    		RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","(%s):%d  invalid  create payload status.\n", __FUNCTION__, __LINE__);
	}
    } else if(clipUploadStatus >= CVR_UPLOAD_FAIL &&
	      clipUploadStatus <= CVR_UPLOAD_CURL_ERR) {
	smartThInst->delSTN(stnFname);
    	RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","(%s):%d Discard STN, CVR clip %s falied to upload.\n", __FUNCTION__, __LINE__, cvrClipFname);
    } else {
    	RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","(%s):%d Unknown CVR clip upload status.\n", __FUNCTION__, __LINE__);
    }

    rtMessage_Release(req);
}

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

#if 0
    //RDK_LOG(RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d)  time-spent:%lu \n", __FUNCTION__ , __LINE__, currTime.tv_nsec - va_msg_prev_time);
    //va_msg_prev_time = currTime.tv_nsec;
#endif

    RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d)  strFramePTS:%s \n", __FUNCTION__ , __LINE__, strFramePTS);

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

/*
    RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Last Motion time stamp: %d\n", __FUNCTION__ , __LINE__, smartThInst->motion_time);
    // ignore frames and metadata for event_quiet_time
    if( (currTime.tv_sec < (smartThInst->motion_time + smartThInst->event_quiet_time)) &&  (0 != smartThInst->motion_time) ) {
        RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):Within event quiet time, Ignoring event, time passed: %lu\n", __FUNCTION__ , __LINE__, (currTime.tv_sec-smartThInst->motion_time));
	return;
    }
    RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):Not within event quiet time, Capturing frame.\n", __FUNCTION__ , __LINE__);
*/

    //read the 720*1280 YUV data.
#ifdef _HAS_XSTREAM_

#ifdef _DIRECT_FRAME_READ_
    ret = smartThInst->consumer->ReadRAWFrame((u16)STN_HRES_BUFFER_ID, (u16)FORMAT_YUV, smartThInst->frameInfo);
#else
    ret = smartThInst->consumer->ReadRAWFrame(STN_HRES_BUFFER_ID, FORMAT_YUV, smartThInst->frameInfo);
#endif

#else
    if (NULL == smartThInst -> hres_frame_info) {
        RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):High Res YUV Frame malloc error \n", __FUNCTION__ , __LINE__);
    }
    memset(smartThInst -> hres_frame_info, 0, sizeof(RDKC_PLUGIN_YUVInfo));

    ret = smartThInst -> recorder -> ReadYUVData(STN_HRES_BUFFER_ID, smartThInst -> hres_frame_info);
#endif

	if( STH_SUCCESS != ret) {
		RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):Error Reading High Res YUV Frame return status is %d \n", __FUNCTION__, __LINE__, ret);
		smartThInst->isHresFrameReady = false;
		return;
	}
	//RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):Success in Reading High Res YUV Frame return status is %d \n", __FUNCTION__, __LINE__, ret);

#ifdef _HAS_XSTREAM_
    hResFramePTS = smartThInst->frameInfo->mono_pts;
#else
    hResFramePTS = smartThInst->hres_frame_info->mono_pts;
#endif

    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): hResframePTS:%llu\n", __FUNCTION__, __LINE__, hResFramePTS);
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):Time gap (hResFramePTS - lResframePTS):%llu\n", __FUNCTION__, __LINE__, (hResFramePTS - lResFramePTS));

#if 0
    if(!hres_yuvDataMemoryAllocationDone) {
	smartThInst -> hres_y_height = smartThInst -> hres_frame_info->height;
	smartThInst -> hres_y_width = smartThInst -> hres_frame_info->width;
	smartThInst -> hres_y_size = smartThInst -> hres_frame_info->width * smartThInst -> hres_frame_info->height;
	smartThInst -> hres_uv_size = smartThInst -> hres_frame_info->width * smartThInst -> hres_frame_info->height;
    	smartThInst -> hres_yuvData = (unsigned char *) malloc((smartThInst -> hres_y_size + smartThInst -> hres_uv_size) * sizeof(unsigned char));

        hres_yuvDataMemoryAllocationDone = true;
    }

    //Acquire lock
    std::unique_lock<std::mutex> lock(stnMutex);
    memset( smartThInst -> hres_yuvData, 0, (smartThInst -> hres_y_size + smartThInst -> hres_uv_size) * sizeof(unsigned char) );
    memcpy( smartThInst -> hres_yuvData, smartThInst -> hres_frame_info->y_addr, smartThInst -> hres_y_size);
    memcpy( smartThInst -> hres_yuvData + smartThInst -> hres_y_size, smartThInst -> hres_frame_info->uv_addr, smartThInst -> hres_uv_size);
    //Release lock
    lock.unlock();
#endif

    //Ensure metadata is received after the frame is captured.
    smartThInst -> isHresFrameReady = true;

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
    double   motionScore = 0.0;
    int32_t  eventType = 0;
    int32_t  boundingBoxXOrd = 0;
    int32_t  boundingBoxYOrd = 0;
    int32_t  boundingBoxHeight = 0;
    int32_t  boundingBoxWidth = 0;
    uint64_t curr_time = 0;
    char const*  s_curr_time = NULL;
    char const*  strFramePTS = NULL;
    uint64_t lResFramePTS = 0;

    int lBboxArea = 0;

#if 0
    struct timespec currTime;

    //clock the current time
    memset (&currTime, 0, sizeof(struct timespec));
    clock_gettime(CLOCK_REALTIME, &currTime);

    RDK_LOG(RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d)  time-spent:%lu \n", __FUNCTION__ , __LINE__, currTime.tv_nsec - va_msg_prev_time);
    va_msg_prev_time = currTime.tv_nsec;
#endif

    (void) closure;

    rtMessage m;
    rtMessage_FromBytes(&m, buff, n);

    RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d) process frame invoked.\n", __FUNCTION__ , __LINE__);

    //read the metadata.
    rtMessage_GetString(m, "timestamp", &strFramePTS);
    rtMessage_GetInt32(m, "event_type", &eventType);
    rtMessage_GetDouble(m, "motionScore", &motionScore);
    rtMessage_GetInt32(m, "boundingBoxXOrd", &boundingBoxXOrd);
    rtMessage_GetInt32(m, "boundingBoxYOrd", &boundingBoxYOrd);
    rtMessage_GetInt32(m, "boundingBoxHeight", &boundingBoxHeight);
    rtMessage_GetInt32(m, "boundingBoxWidth", &boundingBoxWidth);
    rtMessage_GetString(m, "currentTime", &s_curr_time);

    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): strFramePTS:%s \n", __FUNCTION__ , __LINE__, strFramePTS);
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): event Type: %d\n",__FILE__, __LINE__, eventType);
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): motionScore: %f\n",__FILE__, __LINE__, motionScore);
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): boundingBoxXOrd:%d\n", __FILE__, __LINE__,boundingBoxXOrd);
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): boundingBoxYOrd:%d\n", __FILE__, __LINE__,boundingBoxYOrd);
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): boundingBoxHeight:%d\n", __FILE__, __LINE__,boundingBoxHeight);
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): boundingBoxWidth:%d\n", __FILE__, __LINE__,boundingBoxWidth);
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): curr timestamp:%s\n", __FILE__, __LINE__,s_curr_time);

    std::istringstream iss(strFramePTS);
    iss >> lResFramePTS;
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): lResframePTS:%llu\n", __FUNCTION__, __LINE__, lResFramePTS);
    iss.clear();

    iss.str(s_curr_time);
    iss >> curr_time;
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): curr timestamp (uint64):%llu\n", __FILE__, __LINE__,curr_time);
    iss.clear();

    lBboxArea = boundingBoxWidth * boundingBoxHeight;

    RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Current Area : %d\n", __FILE__ , __LINE__, lBboxArea);

    if( (smartThInst->logMotionEvent == true)
	&& (eventType == 4) ) {

	smartThInst->logMotionEvent = false;
	RDK_LOG(RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Received Motion event from xvision during the current cvr interval,  time received: %llu\n", __FILE__ , __LINE__, curr_time);
    }

    // if motion is detected update the metadata.
    if ((smartThInst->isHresFrameReady) &&
       //(motionScore != 0.0) &&
       (eventType == 4) &&
       (lBboxArea > smartThInst->maxBboxArea)) {

    	RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d) Processing metadata .\n", __FUNCTION__ , __LINE__);

	smartThInst -> isHresFrameReady =  false;
        //Update the max bounding area.
        smartThInst -> maxBboxArea = lBboxArea;
        //{
	    //Acquire lock
	    //std::unique_lock<std::mutex> lock(stnMutex);
	    if (!(smartThInst -> isPayloadAvailable)) {
    		RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d) Payload will be available for this interval.\n", __FUNCTION__ , __LINE__);
		//To indicate payload will there to upload
	        smartThInst -> isPayloadAvailable = true;
	    }
            smartThInst -> updateObjFrameData(boundingBoxXOrd, boundingBoxYOrd, boundingBoxWidth, boundingBoxHeight, curr_time);
	    //lock.unlock();
	//}

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
#if 0
    char tmpFname[256] = {'\0'};
    cv::Mat lHresRGBMat;
#endif
    ofData.boundingBoxXOrd = boundingBoxXOrd;
    ofData.boundingBoxYOrd = boundingBoxYOrd;
    ofData.boundingBoxWidth = boundingBoxWidth;
    ofData.boundingBoxHeight = boundingBoxHeight;
    ofData.currTime = currTime;

#if 0
    snprintf(tmpFname, sizeof(tmpFname), "%llu.jpg", ofData.currTime);
#endif

#ifdef _HAS_XSTREAM_
    smartThInst -> hres_y_height = smartThInst -> frameInfo->height;
    smartThInst -> hres_y_width = smartThInst -> frameInfo->width;
    smartThInst -> hres_y_size = smartThInst -> frameInfo->width * smartThInst -> frameInfo->height;
    smartThInst -> hres_uv_size = smartThInst -> frameInfo->width * smartThInst -> frameInfo->height/2;
    hres_yuvData = (unsigned char *) malloc((smartThInst -> hres_y_size + smartThInst -> hres_uv_size) * sizeof(unsigned char));

    if( NULL == hres_yuvData){
        RDK_LOG(RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d) malloc failed for hres_yuvData.\n", __FUNCTION__ , __LINE__);
    }

    memset( hres_yuvData, 0, (smartThInst -> hres_y_size + smartThInst -> hres_uv_size) * sizeof(unsigned char) );
    memcpy( hres_yuvData, smartThInst -> frameInfo->y_addr, smartThInst -> hres_y_size);
    memcpy( hres_yuvData + smartThInst -> hres_y_size, smartThInst -> frameInfo->uv_addr, smartThInst -> hres_uv_size);

    //Full 720*1280 frame containing max bounding box
    ofData.maxBboxObjYUVFrame = cv::Mat(smartThInst -> frameInfo -> height + (smartThInst -> frameInfo -> height)/2, smartThInst -> frameInfo -> width, CV_8UC1, hres_yuvData).clone();
#else
    smartThInst -> hres_y_height = smartThInst -> hres_frame_info->height;
    smartThInst -> hres_y_width = smartThInst -> hres_frame_info->width;
    smartThInst -> hres_y_size = smartThInst -> hres_frame_info->width * smartThInst -> hres_frame_info->height;
    smartThInst -> hres_uv_size = smartThInst -> hres_frame_info->width * smartThInst -> hres_frame_info->height;
    hres_yuvData = (unsigned char *) malloc((smartThInst -> hres_y_size + smartThInst -> hres_uv_size) * sizeof(unsigned char));

    //Acquire lock
    //std::unique_lock<std::mutex> lock(stnMutex);
    memset( hres_yuvData, 0, (smartThInst -> hres_y_size + smartThInst -> hres_uv_size) * sizeof(unsigned char) );
    memcpy( hres_yuvData, smartThInst -> hres_frame_info->y_addr, smartThInst -> hres_y_size);
    memcpy( hres_yuvData + smartThInst -> hres_y_size, smartThInst -> hres_frame_info->uv_addr, smartThInst -> hres_uv_size);
    //Release lock
    //lock.unlock();

    //Full 720*1280 frame containing max bounding box
    ofData.maxBboxObjYUVFrame = cv::Mat(smartThInst -> hres_frame_info -> height + (smartThInst -> hres_frame_info -> height)/2, smartThInst -> hres_frame_info -> width, CV_8UC1, hres_yuvData).clone();
#endif

    if(hres_yuvData) {
	free(hres_yuvData);
	hres_yuvData = NULL;
    }

#if 0
    cv::cvtColor(ofData.maxBboxObjYUVFrame,lHresRGBMat, cv::COLOR_YUV2BGR_NV12);
    cv::rectangle(lHresRGBMat, cv::Rect(ofData.boundingBoxXOrd, ofData.boundingBoxYOrd, ofData.boundingBoxWidth, ofData.boundingBoxHeight), cv::Scalar(0,0,255), 2);

    cv::putText(lHresRGBMat, (std::to_string(ofData.currTime)).c_str(), cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
    cv::putText(lHresRGBMat, (std::to_string(ofData.boundingBoxHeight)).c_str(), cv::Point(40, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
    cv::putText(lHresRGBMat, (std::to_string(ofData.boundingBoxWidth)).c_str(), cv::Point(60, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
    cv::putText(lHresRGBMat, (std::to_string(ofData.boundingBoxWidth*boundingBoxHeight)).c_str(), cv::Point(80, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);

    imwrite(tmpFname, lHresRGBMat);
#endif
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
    rtConnection_Create(&connectionRecv, "SMART_TN_RECV", "tcp://127.0.0.1:10001");

    rtConnection_AddListener(connectionRecv, "RDKC.SMARTTN.CAPTURE",onMsgCaptureFrame, NULL);
    rtConnection_AddListener(connectionRecv, "RDKC.SMARTTN.METADATA",onMsgProcessFrame, NULL);
    rtConnection_AddListener(connectionRecv, "RDKC.CVR.CLIP.STATUS",onMsgCvr, NULL);
    rtConnection_AddListener(connectionRecv, "RDKC.CVR.UPLOAD.STATUS",onMsgCvrUpload, NULL);
    rtConnection_AddListener(connectionRecv, "RDKC.CONF.TNUPLOAD.REFRESH",onMsgRefreshTnUpload, NULL);

    //Add listener for dynamic log topics
    rtConnection_AddListener(connectionRecv, RTMSG_DYNAMIC_LOG_REQ_RES_TOPIC, dynLogOnMessage, connectionRecv);

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

/**
 * @description: This function is used to sleep for some waitingInterval  before next retry.
 *
 * @return: Error code.
 */
int SmartThumbnail::retryAtExpRate()
{
    int ret = RDKC_FAILURE;
    int retryFactor = 2;

    if(waitingInterval <= STN_UPLOAD_TIME_INTERVAL)
    {
        RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.CVRPOLL","%s: %d: Waiting for %d seconds!\n", __FILE__, __LINE__, (int)waitingInterval);
        sleep(waitingInterval);
        waitingInterval *= retryFactor;
        ret = RDKC_SUCCESS;
    }
    return ret;
}

/** @description: thread routine to upload smart thumbnail payload
 *  @param[in] : time left
 *  @return: void
 */
void SmartThumbnail::uploadPayload()
{
    int curlCode = 0;
    long response_code = 0;
    char modelName[CONFIG_STRING_MAX] = {0};
    char macAddress[CONFIG_STRING_MAX] = {0};
    char firmwareName[FW_NAME_MAX_LENGTH] = {0};
#ifdef USE_FILE_UPLOAD
    char *data  = NULL;
    struct stat fileStat;
    int fileLen = 0;
    int readLen = 0;
    char *ptr   = NULL;
    char readBuf[STN_UPLOAD_SEND_LEN];
    int fd = 0;
    char sTnUploadFpath[256] = {0};
    char sTnTStamp[256] = {0};
    //struct tm* tv = NULL;
#else
    char *dataPtr  = NULL;
    int dataLen = 0;
    //std::vector<uchar> dataPtr;
    //dataPtr.reserve(STN_DATA_MAX_SIZE);
#endif
    char packHead[STN_UPLOAD_SEND_LEN+1];
    int retry = 0;
    int remainingTime = 0;

    struct timespec currTime;
    struct timespec startTime;
    memset(&startTime, 0, sizeof(startTime));
    memset(&currTime, 0, sizeof(currTime));

    setMacAddress(macAddress);
    setModelName(modelName);
    setCameraImageName(firmwareName);

    //clock the start time
    clock_gettime(CLOCK_REALTIME, &startTime);
    RDK_LOG( RDK_LOG_TRACE1,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):Uploading smart thumbnail!!! Time left %ld\n", __FILE__, __LINE__, STN_UPLOAD_TIME_INTERVAL);

    while (true) {
        //clock the current time
        memset(&currTime, 0, sizeof(currTime));
        clock_gettime(CLOCK_REALTIME, &currTime);
        remainingTime = STN_UPLOAD_TIME_INTERVAL - (currTime.tv_sec - startTime.tv_sec);

        if ( remainingTime <= 0 ) {
            RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL", "%s(%d): Max time exceeded for stn upload after retry count %d !!!\n", __FILE__,__LINE__, retry);
            break;
        }

        if (0 == retry) {
            std::unique_lock<std::mutex> lock(smartThInst -> stnMutex);
            snprintf(sTnUploadFpath, sizeof(sTnUploadFpath), "%s/%s", STN_PATH, smartThInst->payload.fname);
            //snprintf(sTnTStamp, sizeof(sTnTStamp), "%llu", payload.tstamp);
            stringifyEventDateTime(sTnTStamp, sizeof(sTnTStamp), smartThInst->payload.tstamp);
            RDK_LOG( RDK_LOG_TRACE1,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):STN Time is  %s\n", __FILE__, __LINE__, sTnTStamp);

            //check for empty payload and break if so
            /*if (payload.objFrame.empty()) {
                RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):payload is empty!!!\n", __FILE__, __LINE__);
                break;
            }*/
#if 0
            tv = gmtime(&currTime.tv_sec);
            snprintf(uploadFname, sizeof(uploadFname), "%s/%04d%02d%02d%02d%02d%02d.jpg", STN_PATH,(tv->tm_year+1900), tv->tm_mon+1, tv->tm_mday, tv->tm_hour, tv->tm_min, tv->tm_sec);
            RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Creating smart thumbnail upload file: %s\n",__FILE__, __LINE__, uploadFname);
            //Write smart thumbnail to file.
            imwrite(uploadFname, payload.objFrame);
#endif

#ifdef USE_FILE_UPLOAD
            /* get file attribute */
            if (stat(sTnUploadFpath, &fileStat) < 0) {
                RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): invalid file [%s], errmsg=%s\n",__FILE__, __LINE__, sTnUploadFpath, strerror(errno));
                lock.unlock();
                break;
                /*retry++;
                RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Retry again for %d times!!!\n",__FILE__,__LINE__,retry);
                continue;*/
            }

            fileLen = fileStat.st_size;
            RDK_LOG(RDK_LOG_INFO, "LOG.RDK.SMARTTHUMBNAIL", "%s(%d):Length of the smart thumbnail file to be uploaded:%d\n", __FILE__, __LINE__,fileLen);

            fd = open(sTnUploadFpath, O_RDONLY);
            if (fd <= 0) {
                RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Failed to Open smart thumbnail upload File :%s : error : [%s]\n", __FILE__, __LINE__, sTnUploadFpath, strerror(errno));
                lock.unlock();
                pid_t pid = 0;
                pid = getpid();
                RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.CVRUPLOAD","%s(%d): Open failed sending sigterm to thread id : %d\n", __FUNCTION__, __LINE__,pid);
                kill(pid, SIGTERM);
                break;
                /*retry++;
                RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Retrying again for %d times!!!\n",__FILE__,__LINE__,retry);
                continue;*/
            }

            data =(char*)malloc(fileLen*sizeof(char));
            if (NULL == data) {
                RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Failed to allocate memory :%s \n", __FILE__, __LINE__, sTnUploadFpath);
                if (fd >= 0) {
                    close(fd);
                }
                lock.unlock();
                break;
                /*retry++;
                RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Retrying again for %d times!!!\n",__FUNCTION__,__LINE__,retry);
                continue;*/
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

            lock.unlock();
#else
            dataLen = payload.objFrame.total() * payload.objFrame.elemSize();
            dataPtr = reinterpret_cast<char*>(payload.objFrame.data);
            //cv::imencode(".jpg", payload.objFrame, dataPtr);
            //dataLen = dataPtr.size();
#endif
            }

            /* Add Header */
            smartThInst->httpClient->resetHeaderList();
            smartThInst->httpClient->addHeader( "Expect", "");   //removing expect header condition by explicitly setting Expect header to ""
            memset(packHead, 0, sizeof(packHead));
            snprintf(packHead, sizeof(packHead), "%s", smartThInst -> smtTnAuthCode);
            smartThInst->httpClient->addHeader( "Authorization", packHead);
            memset(packHead, 0, sizeof(packHead));
            snprintf(packHead, sizeof(packHead), "image/jpeg");
            smartThInst->httpClient->addHeader( "Content-Type", packHead);
            memset(packHead, 0, sizeof(packHead));
            snprintf(packHead, sizeof(packHead), "Sercomm %s %s %s", modelName, firmwareName, macAddress);
            smartThInst->httpClient->addHeader( "User-Agent", packHead);
            memset(packHead, 0, sizeof(packHead));
            //snprintf(packHead, sizeof(packHead), "%llu", payload.tstamp);
            smartThInst->httpClient->addHeader( "X-EVENT-DATETIME", sTnTStamp);

            memset(packHead, 0, sizeof(packHead));
            //SMTN's Bounding box of Motion Detection area
            snprintf(packHead, sizeof(packHead), "%d, %d, %d, %d", relativeBBox.x, relativeBBox.y, relativeBBox.width, relativeBBox.height);
            smartThInst->httpClient->addHeader( "X-BoundingBox", packHead);
            memset(packHead, 0, sizeof(packHead));
            snprintf(packHead, sizeof(packHead), "CVR");
            smartThInst->httpClient->addHeader("X-VIDEO-RECORDING", packHead);

#ifdef USE_FILE_UPLOAD
            memset(packHead, 0, sizeof(packHead));
            snprintf(packHead, sizeof(packHead), "%s", sTnUploadFpath);
            smartThInst->httpClient->addHeader( "X-FILE-NAME", packHead);
            memset(packHead, 0, sizeof(packHead));
            snprintf(packHead, sizeof(packHead), "%d",fileLen);
            smartThInst->httpClient->addHeader( "Content-Length", packHead);

            //upload data
            curlCode =  smartThInst->httpClient->post_binary(smartThInst -> smtTnUploadURL, data, &response_code, fileLen, remainingTime);
#else
            memset(packHead, 0, sizeof(packHead));
            snprintf(packHead, sizeof(packHead), "%d",dataLen);
            smartThInst->httpClient->addHeader( "Content-Length", packHead);

            curlCode =  smartThInst->httpClient->post_binary(smartThInst -> smtTnUploadURL, dataPtr, &response_code, dataLen, remainingTime);
#endif
            if ((response_code >= RDKC_HTTP_RESPONSE_OK) && (response_code < RDKC_HTTP_RESPONSE_REDIRECT_START))
            {
                break;
            } else {
                retry++;
                if(RDKC_SUCCESS != retryAtExpRate())
                {
                    RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.CVRUPLOAD","%s(%d): retries done with current exp wait interval %d\n",__FILE__, __LINE__,waitingInterval);
                    break;
                }
                RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Retrying again for %d times with %d sec remaining timeout  because last try failed  with response code %lu and curl code %d\n",__FUNCTION__,__LINE__,retry, remainingTime, response_code, curlCode);
            }
        }

        //log success/failure for telemetry
        if ((response_code >= RDKC_HTTP_RESPONSE_OK) && (response_code < RDKC_HTTP_RESPONSE_REDIRECT_START)) {
            RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Smart Thumbnail uploaded successfully with header X-EVENT-TIME: %s X-BoundingBox: %d %d %d %d X-VIDEO-RECORDING:CVR\n",__FUNCTION__,__LINE__,sTnTStamp, relativeBBox.x, relativeBBox.y, relativeBBox.width, relativeBBox.height);
        }else {
            RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Smart Thumbnail upload failed with response code %lu and curl code %d\n",__FUNCTION__,__LINE__, response_code, curlCode);
        }

#ifdef USE_FILE_UPLOAD
        if(NULL != data) {
            free(data);
            data = NULL;
        }

        if (fd >= 0) {
            close(fd);
        }
        RDK_LOG( RDK_LOG_TRACE1,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Removing smart thumbnail upload file: %s\n",__FILE__, __LINE__, sTnUploadFpath);
	//unlink (uploadFname);
	delSTN(sTnUploadFpath);
#endif
}


/** @description : thread routine to listen to messages
 *  @param[in] : void.
 *  @return : void
 */
STH_STATUS SmartThumbnail::receiveRtmessage()
{
    rtError err;

    while (!termFlag) {
	err = rtConnection_Dispatch(connectionRecv);
	if (err != RT_OK) {
	    //rtLog_Debug("dispatch:%s", rtStrError(err));
	    RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): dispatch:%s\n",__FUNCTION__,__LINE__,rtStrError(err));
	    RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Error receiving msg via rtmessage\n",__FUNCTION__,__LINE__);
	}
	usleep(10000);
    }
    RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Exit rtMessage listening loop .\n",__FUNCTION__,__LINE__);
    cv.notify_one();
}

void SmartThumbnail::uploadSTN()
{
     while (!termFlag){

	if(tnUploadConfRefreshed){
            RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Tn Upload Conf refreshed, fetching new URL and auth token!!\n", __FUNCTION__, __LINE__);

            smartThInst->httpClient->close();

            memset(smartThInst->smtTnUploadURL, 0 , sizeof(smartThInst->smtTnUploadURL));
            memset(smartThInst->smtTnAuthCode, 0 , sizeof(smartThInst->smtTnAuthCode));

            if(STH_SUCCESS == smartThInst -> getTnUploadConf()) {
                RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): getTnUploadConf success!!", __FUNCTION__, __LINE__);
            }

            /* Open the URL */
            if (NULL != smartThInst->httpClient) {
                //smartThInst->httpClient->open(smartThInst->smtTnUploadURL,smartThInst -> dnsCacheTimeout, STN_MAX_UPLOAD_TIMEOUT);
                smartThInst->httpClient->open(smartThInst->smtTnUploadURL,smartThInst -> dnsCacheTimeout);
            } else {
                RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Failed to open the URL\n", __FUNCTION__, __LINE__);
            }

            tnUploadConfRefreshed = false;
        }

	if(smartThInst -> getUploadStatus()) {
	    smartThInst->uploadPayload();
        }
     }
     RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Exiting smart thumbnail upload thread.\n",__FUNCTION__,__LINE__);
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
		RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAILUPLOAD","%s(%d): Error in opening version.txt \n", __FILE__, __LINE__);
		return RDKC_FAILURE;
	}

	file_buffer = (char*)malloc(FW_NAME_MAX_LENGTH + 1);
	if(file_buffer == NULL)
	{
		RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.SMARTTHUMBNAILUPLOAD","%s(%d): Error in malloc \n", __FILE__, __LINE__);
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
int SmartThumbnail::setModelName(char* modelName)
{
#ifdef USE_MFRLIB
    mfrSerializedData_t stdata = {NULL, 0, NULL};
    mfrSerializedType_t stdatatype = mfrSERIALIZED_TYPE_MODELNAME;
    char mac[CONFIG_STRING_MAX];

    if (mfrGetSerializedData(stdatatype, &stdata) == mfrERR_NONE) {
	strncpy(modelName,stdata.buf,stdata.bufLen);
	modelName[stdata.bufLen] = '\0';
	RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):Model Name = %s,%s,%d\n",__FILE__, __LINE__,modelName,stdata.buf,stdata.bufLen);
	if (stdata.freeBuf != NULL) {
		stdata.freeBuf(stdata.buf);
		stdata.buf = NULL;
	}
    }
    else {
	RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):GET ModelName failed : %d\n", __FILE__, __LINE__);
    }
#endif
}

/**
 * @description: This function is used to set the camera's mac address.
 * @param[out]: void
 * @return: Error code.
* */
int SmartThumbnail::setMacAddress(char* macAddress)
{
	char mac[CONFIG_STRING_MAX] = {0};

#ifdef USE_MFRLIB
	mfrSerializedData_t stdata = {NULL, 0, NULL};
	mfrSerializedType_t stdatatype = mfrSERIALIZED_TYPE_DEVICEMAC;

	if (mfrGetSerializedData(stdatatype, &stdata) == mfrERR_NONE) {
		strncpy(mac,stdata.buf,stdata.bufLen);
		mac[stdata.bufLen] = '\0';
		RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):mac= %s,%s,%d\n",__FILE__, __LINE__,mac,stdata.buf,stdata.bufLen);

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
				RDK_LOG( RDK_LOG_WARN,"LOG.RDK.SMARTTHUMBNAIL","field %d value %0x out of range\n", fieldNum, tmpVal);
			}
			if (*chk != 0) {
				RDK_LOG( RDK_LOG_WARN,"LOG.RDK.SMARTTHUMBNAIL","Non-digit character %c (%0x) detected in field %d\n", *chk, *chk, fieldNum);

			}
			fieldNum++;
			strcat(macAddress, tmpField);
			tmpField = strtok(NULL, ":");
		}
		RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):mac address= %s\n",__FILE__, __LINE__,macAddress);
		if (stdata.freeBuf != NULL) {
			stdata.freeBuf(stdata.buf);
			stdata.buf = NULL;
		}

	}
	else {
		RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d):GET MAC failed : %d\n", __FILE__, __LINE__);
	}
#endif
}
