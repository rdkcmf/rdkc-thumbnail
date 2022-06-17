#ifdef _OBJ_DETECTION_
#include "smart_thumbnail.h"
#define CROPPING_WIDTH 848
#define CROPPING_HEIGHT 480

extern SmartThumbnail *smTnInstance;
static int isMotionFilter;

int mpipe_port_initialize(const std::string &input_video_path, int &width, int &height) {
  std::cout << "rdkc PORT Initialize the camera" << std::endl;
  return 0;
}

int mpipe_port_initialize(int &width, int &height) {
  mpipe_port_initialize(std::string(""), width, height);
  return 0;
}

cv::Mat mpipe_port_getNextFrame(std::vector<cv::Point>& roiCoords, std::vector<std::vector<cv::Point>>& motionblobs) {
    cv::Mat cv_frame, croppedObj;
    std::vector<cv::Point> cropRegion = {}, roiRegion = {};

#ifdef ENABLE_TEST_HARNESS
    if(!smTnInstance -> waitForNextDetectionFrame()) return cv_frame;
    if(isMotionFilter && !smTnInstance -> waitForNextMotionFrame()) return cv_frame;
    if(smTnInstance -> lastProcessedFrame == smTnInstance -> FrameNum) return cv_frame;
    smTnInstance -> lastProcessedFrame = smTnInstance -> FrameNum;
    cv_frame = cv::Mat(smTnInstance ->curr_frame.rows, smTnInstance ->curr_frame.cols, CV_8UC3);
    cv::cvtColor(smTnInstance ->curr_frame, cv_frame, cv::COLOR_BGR2RGB);
    smTnInstance -> detectionTstamp = smTnInstance -> currTstamp;
    std::unique_lock<std::mutex> lock(smTnInstance -> hres_data_lock);
#else

    if(isMotionFilter && !smTnInstance -> waitForNextMotionFrame()) return cv_frame;
    std::unique_lock<std::mutex> lock(smTnInstance -> hres_data_lock);
    smTnInstance -> hres_y_height = smTnInstance -> frameInfo->height;
    smTnInstance -> hres_y_width = smTnInstance -> frameInfo->width;
    smTnInstance -> hres_y_size = smTnInstance -> frameInfo->width * smTnInstance -> frameInfo->height;
    smTnInstance -> hres_uv_size = smTnInstance -> frameInfo->width * smTnInstance -> frameInfo->height/2;
    int alloc_size = smTnInstance -> hres_y_size + smTnInstance -> hres_uv_size;

    if(smTnInstance -> mpipe_hres_yuvData == NULL){
        smTnInstance -> mpipe_hres_yuvData = (unsigned char *) malloc(alloc_size * sizeof(unsigned char));
    }else if(sizeof(smTnInstance -> mpipe_hres_yuvData) != (alloc_size* sizeof(unsigned char))) {
        smTnInstance -> mpipe_hres_yuvData = (unsigned char *) realloc(smTnInstance -> mpipe_hres_yuvData, alloc_size * sizeof(unsigned char));
    }
    if( NULL == smTnInstance -> mpipe_hres_yuvData){
        RDK_LOG(RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d) malloc failed for hres_yuvData.\n", __FUNCTION__ , __LINE__);
	return cv_frame;
    }

//    if(!smTnInstance -> nextFrameForDetection.empty()) smTnInstance -> nextFrameForDetection.release();
    memset( smTnInstance -> mpipe_hres_yuvData, 0, (smTnInstance -> hres_y_size + smTnInstance -> hres_uv_size) * sizeof(unsigned char) );
    memcpy( smTnInstance -> mpipe_hres_yuvData, smTnInstance -> frameInfo->y_addr, smTnInstance -> hres_y_size);
    memcpy( smTnInstance -> mpipe_hres_yuvData + smTnInstance -> hres_y_size, smTnInstance -> frameInfo->uv_addr, smTnInstance -> hres_uv_size);

    cv::Mat next_frame = cv::Mat(smTnInstance -> frameInfo -> height + (smTnInstance -> frameInfo -> height)/2, smTnInstance -> frameInfo -> width, CV_8UC1, smTnInstance -> mpipe_hres_yuvData);


    cv_frame = cv::Mat(next_frame.rows, next_frame.cols, CV_8UC3);
    cv::cvtColor(next_frame, cv_frame, cv::COLOR_YUV2RGB_NV12/*cv::COLOR_BGR2RGB*/);
    next_frame.release();
#endif

    double scaleFactor = 1;
    cv::Size cropSize = smTnInstance -> getCropSize(smTnInstance -> currentBbox, smTnInstance -> sTnWidth, smTnInstance -> sTnHeight, &scaleFactor);
    //Resize the union blob with scaleFactor
    smTnInstance -> currentBbox.width = smTnInstance -> currentBbox.width/scaleFactor;
    smTnInstance -> currentBbox.height = smTnInstance -> currentBbox.height/scaleFactor;
    smTnInstance -> currentBbox.x = smTnInstance -> currentBbox.x/scaleFactor;
    smTnInstance -> currentBbox.y = smTnInstance -> currentBbox.y/scaleFactor;
    cv::Size rescaleSize = cv::Size(cv_frame.cols/scaleFactor, cv_frame.rows/scaleFactor);
    //resize the frame to fit the union blob in the thumbnail
    cv::resize(cv_frame, cv_frame, rescaleSize);
    cv::Point2f orgCenter = smTnInstance -> getActualCentroid(smTnInstance -> currentBbox);
    cv::Point2f allignedCenter =  smTnInstance -> alignCentroid(orgCenter, cv_frame, cropSize);
    getRectSubPix(cv_frame, cropSize, allignedCenter, croppedObj);

    //Get relative ROI coordinates
    roiCoords.clear();

    if(smTnInstance -> roi.size() > 3) {
        /* Upscale the normalized ROI coordinate to the resized frame size */
        for( int i = 0; i < (smTnInstance->roi.size()/2) - 1; i++) {
            int x = (cv_frame.cols * (smTnInstance -> roi[i*2]) )/ scaleFactor;
            int y = (cv_frame.rows * (smTnInstance -> roi[(i*2)+1])) / scaleFactor;
            RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d) [%d,%d]roi[%d] = %d,%d\n", __FILE__, __LINE__, cv_frame.cols, cv_frame.rows, i, x, y);

            roiRegion.push_back(cv::Point(x,y));
        }
    }

    /* Calculate corner points of copping region */
    cropRegion.push_back(cv::Point(allignedCenter.x - (cropSize.width/2), allignedCenter.y -(cropSize.height / 2)));
    cropRegion.push_back(cv::Point(allignedCenter.x - (cropSize.width/2), allignedCenter.y +(cropSize.height / 2)));
    cropRegion.push_back(cv::Point(allignedCenter.x + (cropSize.width/2), allignedCenter.y +(cropSize.height / 2)));
    cropRegion.push_back(cv::Point(allignedCenter.x + (cropSize.width/2), allignedCenter.y -(cropSize.height / 2)));
    if(!roiRegion.empty()) {
        roiCoords = smTnInstance->getPolygonInCroppedRegion(roiRegion,cropRegion);
    }

    smTnInstance -> printPolygonCoords("Relative ROI", roiCoords);

    for(int i = 0; i < UPPER_LIMIT_BLOB_BB; i++) {
        if(smTnInstance->currentMotionBlobs[i].boundingBoxXOrd == INVALID_BBOX_ORD) break;

        std::vector<cv::Point> motionblob;
        motionblob.push_back(cv::Point(smTnInstance->currentMotionBlobs[i].boundingBoxXOrd / scaleFactor, smTnInstance->currentMotionBlobs[i].boundingBoxYOrd / scaleFactor));
        motionblob.push_back(cv::Point((smTnInstance->currentMotionBlobs[i].boundingBoxXOrd + smTnInstance->currentMotionBlobs[i].boundingBoxWidth) / scaleFactor, smTnInstance->currentMotionBlobs[i].boundingBoxYOrd / scaleFactor));
        motionblob.push_back(cv::Point((smTnInstance->currentMotionBlobs[i].boundingBoxXOrd + smTnInstance->currentMotionBlobs[i].boundingBoxWidth) / scaleFactor, (smTnInstance->currentMotionBlobs[i].boundingBoxYOrd + smTnInstance->currentMotionBlobs[i].boundingBoxHeight) / scaleFactor));
        motionblob.push_back(cv::Point(smTnInstance->currentMotionBlobs[i].boundingBoxXOrd / scaleFactor, (smTnInstance->currentMotionBlobs[i].boundingBoxYOrd + smTnInstance->currentMotionBlobs[i].boundingBoxHeight) / scaleFactor));

        motionblobs.push_back(smTnInstance->getPolygonInCroppedRegion(motionblob, cropRegion));
    }

    lock.unlock();

    //Clearing
    cropRegion.clear();
    roiRegion.clear();

    for(int i = 0; i < motionblobs.size(); i++) {
        char str[CONFIG_STRING_MAX] = {};
        sprintf(str, "Relative motion Blob[%d]", i);
        smTnInstance -> printPolygonCoords(str, motionblobs[i]);
    }

    RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d)The motion blob in the frame is : x = %d, y = %d, w = %d, h = %d\n", __FILE__, __LINE__, smTnInstance -> currentBbox.x, smTnInstance -> currentBbox.y, smTnInstance -> currentBbox.width, smTnInstance -> currentBbox.height);
    RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d)Cropped Frame center [x = %f, y = %f].\n", __FILE__, __LINE__, allignedCenter.x, allignedCenter.y);
    RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.SMARTTHUMBNAIL","%s(%d)Cropped Frame coordinates [x = %f, y = %f].\n", __FILE__, __LINE__, allignedCenter.x - (cropSize.width / 2), allignedCenter.y - (cropSize.height / 2));

      uint8_t *camera_buf = new uint8_t [cropSize.height * cropSize.width * 3];
      memcpy(camera_buf, croppedObj.data, cropSize.height*cropSize.width * 3);
      cv::Mat cv_frame_dst = cv::Mat(croppedObj.rows, croppedObj.cols, CV_8UC3, camera_buf);
#ifdef ENABLE_TEST_HARNESS
    RDK_LOG(RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d) Sending %dth frame for detection in %dth file in THlist.\n", __FUNCTION__ , __LINE__, smTnInstance -> FrameNum, smTnInstance -> FileNum);
#endif
    smTnInstance->mpipeProcessedframes++;
        
    cv_frame.release();
    croppedObj.release();

  return (cv_frame_dst);
}

void loadDetectionConfig(DetectionConfig *config, char *configFile)
{
    FileUtils m_settings;

#ifdef ENABLE_TEST_HARNESS
    config->frame_read_delay = "0";
#endif

    std::string motionFilterStatus = DEFAULT_MOTION_FILTER_ENABLE;
    struct stat statbuf;
    if((stat(configFile, &statbuf) < 0) || (!m_settings.loadFromFile(std::string(configFile)))) {
        RDK_LOG(RDK_LOG_WARN,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Loading detection config file failed, loading default settings\n", __FILE__, __LINE__);
    } else
    {
        m_settings.get("input_video_path", config->input_video_path);
        m_settings.get("delivery_detection_graph_path", config->delivery_detection_graph_path);
        m_settings.get("delivery_detection_model_path", config->delivery_detection_model_path);
        m_settings.get("frame_read_delay", config->frame_read_delay);
        m_settings.get("max_num_frames_cached_for_delivery_detection", config->max_num_frames_cached_for_delivery_detection);
        m_settings.get("delivery_detection_model_min_score_threshold", config->delivery_detection_model_min_score_threshold);
        m_settings.get("delivery_detection_min_score_threshold", config->delivery_detection_min_score_threshold);
        m_settings.get("frame_count_to_process", config->frame_count_to_process);
        m_settings.get("roi_filter", config->roi_filter);
        m_settings.get("motion_cue_filter", config->motion_cue_filter);
        m_settings.get("motion_filter", motionFilterStatus);
        m_settings.get("size_filter_threshold", config->size_filter_threshold);
    }
    isMotionFilter = std::stoi(motionFilterStatus);
}

void mpipe_port_term() {
}

void *__mpipe_thread_main__(DetectionConfig config) {
  std::string argv0 = __FUNCTION__;

  loadDetectionConfig(&config, DETECTION_CONFIG_FILE);
  std::string argv1 = "--input_video_path=\"" + config.input_video_path + "\"";
  std::string argv2 = "--delivery_detection_graph_path=" + config.delivery_detection_graph_path;
  std::string argv3 = "--delivery_detection_model_path=" + config.delivery_detection_model_path;
  std::string argv4 = "--frame_read_delay=" + config.frame_read_delay;
  std::string argv5 = "--max_num_frames_cached_for_delivery_detection=" + config.max_num_frames_cached_for_delivery_detection;
  std::string argv6 = "--delivery_detection_model_min_score_threshold=" + config.delivery_detection_model_min_score_threshold;
  std::string argv7 = "--delivery_detection_min_score_threshold=" + config.delivery_detection_min_score_threshold;
  std::string argv8 = "--frame_count_to_process=" + config.frame_count_to_process;
  std::string argv9 = "--roi_filter=" + config.roi_filter;
  std::string argv10 = "--motion_cue_filter=" + config.motion_cue_filter;
  std::string argv11 = "--size_filter_threshold=" + config.size_filter_threshold;

  const char *argv[] = { argv0.c_str(), argv1.c_str(), argv2.c_str(), argv3.c_str(), argv4.c_str(), argv5.c_str(), argv6.c_str(), argv7.c_str(), argv8.c_str(), argv9.c_str(), argv10.c_str(), argv11.c_str() };

  extern int __mpipe_main__(int, char **);
  __mpipe_main__(sizeof(argv)/sizeof(argv[0]), (char **)argv);
  return NULL;
}

#endif /* _OBJ_DETECTION_ */
