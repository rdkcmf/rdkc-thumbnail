#ifdef _OBJ_DETECTION_

#if 0
#include <iostream>

#include <atomic>
#include<opencv2/core.hpp>
#include<opencv2/videoio.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#endif
//#include "mpipe_port.h"
#include "smart_thumbnail.h"
#define CROPPING_WIDTH 848
#define CROPPING_HEIGHT 480

extern SmartThumbnail *smTnInstance;

int mpipe_port_initialize(const std::string &input_video_path, int &width, int &height) {
  std::cout << "rdkc PORT Initialize the camera" << std::endl;
  return 0;
}

int mpipe_port_initialize(int &width, int &height) {
  mpipe_port_initialize(std::string(""), width, height);
  return 0;
}
#if 1
cv::Mat mpipe_port_getNextFrame() {
#if 1
//  cv::Mat next_frame;
//  cv::Mat next_frame_rgb;
  
//    if(enableDetectionOnHighResolution) { //High resolution frame
      cv::Mat cv_frame, croppedObj;
      smTnInstance->mpipeProcessedframes++;

#ifdef ENABLE_TEST_HARNESS
    smTnInstance -> waitForNextDetectionFrame();
    if( smTnInstance -> lastProcessedFrame == smTnInstance -> FrameNum) return cv_frame;
    smTnInstance -> lastProcessedFrame = smTnInstance -> FrameNum;
    cv_frame = cv::Mat(smTnInstance ->curr_frame.rows, smTnInstance ->curr_frame.cols, CV_8UC3);
    cv::cvtColor(smTnInstance ->curr_frame, cv_frame, cv::COLOR_BGR2RGB);
    smTnInstance -> detectionTstamp = smTnInstance -> currTstamp;
#else

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

    RDK_LOG(RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d)The motion blob in the frame is : x = %d, y = %d, w = %d, h = %d\n", __FILE__, __LINE__, smTnInstance -> currentBbox.x, smTnInstance -> currentBbox.y, smTnInstance -> currentBbox.width, smTnInstance -> currentBbox.height);
    RDK_LOG(RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d)Cropped Frame center [x = %f, y = %f].\n", __FILE__, __LINE__, allignedCenter.x, allignedCenter.y);
    RDK_LOG(RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d)Cropped Frame coordinates [x = %f, y = %f].\n", __FILE__, __LINE__, allignedCenter.x - (cropSize.width / 2), allignedCenter.y - (cropSize.height / 2));

/*      uint8_t *camera_buf = (uint8_t *) malloc(next_frame.rows * next_frame.cols * 3);
      memcpy(camera_buf, cv_frame.data, next_frame.rows * next_frame.cols* 3);
      cv::Mat cv_frame_dst = cv::Mat(next_frame.rows, next_frame.cols, CV_8UC3, camera_buf);
*/
      //uint8_t *camera_buf = (uint8_t *) malloc(848 * 480 * 3);
      uint8_t *camera_buf = new uint8_t [cropSize.height * cropSize.width * 3];
      memcpy(camera_buf, croppedObj.data, cropSize.height*cropSize.width * 3);
      cv::Mat cv_frame_dst = cv::Mat(croppedObj.rows, croppedObj.cols, CV_8UC3, camera_buf);
//      cv::Mat oFrame;
//      cv::cvtColor(cv_frame_dst, oFrame, cv::COLOR_RGB2BGR);
//      imwrite("/tmp/frame.jpg", cv_frame_dst);
#ifdef ENABLE_TEST_HARNESS
    RDK_LOG(RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d) Sending %dth frame for detection in %dth file in THlist.\n", __FUNCTION__ , __LINE__, smTnInstance -> FrameNum, smTnInstance -> FileNum);
#endif
        
    cv_frame.release();
    croppedObj.release();

#if 0
    if(hres_yuvData) {
        free(hres_yuvData);
        hres_yuvData = NULL;
    }
#endif
#if 0
    } else{ //Low resolution frame
	//TODO: Add ReadYUVFrame code here to read frame from camera low resolution buffer
	nextFrame_rgb = cv::Mat(frameInfo -> height, frameInfo -> width, CV_8UC4);
	nextFrame = cv::Mat(frameInfo -> height + (frameInfo -> height)/2, frameInfo -> width, CV_8UC1, frameInfo->frame_ptr).clone();
        cv::cvtColor(next_frame, smTnInstance -> nextFrameForDetection, cv::COLOR_BGR2RGB);
#endif

  return (cv_frame_dst);
#else
  //return smTnInstance -> nextFrameForDetection;
      cv::Mat cv_frame = cv::imread("/opt/sajna/person2.jpg", cv::IMREAD_COLOR);
      cv::Mat cv_frame_rgb;
      cv::cvtColor(cv_frame, cv_frame_rgb, cv::COLOR_BGR2RGB);

      return cv_frame_rgb;
#endif
}
#else
unsigned char * mpipe_port_getNextFrame(int *h, int *w) {
    unsigned char * frame_buf = NULL;
    smTnInstance -> hres_y_height = smTnInstance -> frameInfo->height;
    smTnInstance -> hres_y_width = smTnInstance -> frameInfo->width;
    smTnInstance -> hres_y_size = smTnInstance -> frameInfo->width * smTnInstance -> frameInfo->height;
    smTnInstance -> hres_uv_size = smTnInstance -> frameInfo->width * smTnInstance -> frameInfo->height/2;
    int alloc_size = smTnInstance -> hres_y_size + smTnInstance -> hres_uv_size;

    frame_buf = (unsigned char *) malloc(alloc_size * sizeof(unsigned char));
    if( NULL == frame_buf){
        RDK_LOG(RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d) malloc failed for hres_yuvData.\n", __FUNCTION__ , __LINE__);
        return NULL;
    }

    memset( frame_buf, 0, (smTnInstance -> hres_y_size + smTnInstance -> hres_uv_size) * sizeof(unsigned char) );
    memcpy( frame_buf, smTnInstance -> frameInfo->y_addr, smTnInstance -> hres_y_size);
    memcpy( frame_buf + smTnInstance -> hres_y_size, smTnInstance -> frameInfo->uv_addr, smTnInstance -> hres_uv_size);

    *w = smTnInstance -> frameInfo->width;
    *h = smTnInstance -> frameInfo->height;
    return frame_buf;
}
#endif

void loadDetectionConfig(DetectionConfig *config, char *configFile)
{
    FileUtils m_settings;

    config->input_video_path = DEFAULT_INPUT_DEV;
    config->delivery_detection_graph_path = DEFAULT_GRAPH_PATH;
#ifdef ENABLE_TEST_HARNESS
    config->frame_read_delay = "0";
#else
    config->frame_read_delay = DEFAULT_FRAME_READ_DELAY;
#endif
    config->max_num_frames_cached_for_delivery_detection = DEFAULT_MAX_FRAMES_CACHED_FOR_DELIVERY_DETECTION;
    config->delivery_detection_model_min_score_threshold = DEFAULT_DELIVERY_DETECTION_MODEL_MIN_SCORE_THRESHOLD;
    config->delivery_detection_min_score_threshold = DEFAULT_DELIVERY_DETECTION_MIN_SCORE_THRESHOLD;
    config->frame_count_to_process = DEFAULT_FRAME_COUNT_TO_PROCESS;
    
    if(!m_settings.loadFromFile(std::string(configFile))) {
        RDK_LOG(RDK_LOG_WARN,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Loading detection config file failed, loading default settings\n", __FILE__, __LINE__);
    } else
    {
        m_settings.get("input_video_path", config->input_video_path);
        m_settings.get("delivery_detection_graph_path", config->delivery_detection_graph_path);
        m_settings.get("frame_read_delay", config->frame_read_delay);
        m_settings.get("max_num_frames_cached_for_delivery_detection", config->max_num_frames_cached_for_delivery_detection);
        m_settings.get("delivery_detection_model_min_score_threshold", config->delivery_detection_model_min_score_threshold);
        m_settings.get("delivery_detection_min_score_threshold", config->delivery_detection_min_score_threshold);
        m_settings.get("frame_count_to_process", config->frame_count_to_process);
    }
}

void mpipe_port_term() {
}

void *__mpipe_thread_main__() {
  std::string argv0 = __FUNCTION__;

  DetectionConfig config;

  loadDetectionConfig(&config, DETECTION_CONFIG_FILE);
  std::string argv1 = "--input_video_path=\"" + config.input_video_path + "\"";
  std::string argv2 = "--delivery_detection_graph_path=" + config.delivery_detection_graph_path;
//  std::string argv2 = "--graph_path=/etc/mediapipe/graphs/rdk/delivery_detection/g_person_detection_rdkc_live_cpu.pbtxt";
  std::string argv3 = "--frame_read_delay=" + config.frame_read_delay;
  std::string argv4 = "--max_num_frames_cached_for_delivery_detection=" + config.max_num_frames_cached_for_delivery_detection;
  std::string argv5 = "--delivery_detection_model_min_score_threshold=" + config.delivery_detection_model_min_score_threshold;
  std::string argv6 = "--delivery_detection_min_score_threshold=" + config.delivery_detection_min_score_threshold;
  std::string argv7 = "--frame_count_to_process=" + config.frame_count_to_process;


  std::vector<char> vargv0(argv0.begin(), argv0.end());
  std::vector<char> vargv1(argv1.begin(), argv1.end());
  std::vector<char> vargv2(argv2.begin(), argv2.end());
  std::vector<char> vargv3(argv3.begin(), argv3.end());
  std::vector<char> vargv4(argv4.begin(), argv4.end());
  std::vector<char> vargv5(argv5.begin(), argv5.end());
  std::vector<char> vargv6(argv6.begin(), argv6.end());
  std::vector<char> vargv7(argv7.begin(), argv7.end());

  char *argv[] = { vargv0.data(), vargv1.data(), vargv2.data(), vargv3.data(), vargv4.data(), vargv5.data(), vargv6.data(), vargv7.data()};

  extern int __mpipe_main__(int, char **);
  __mpipe_main__(sizeof(argv)/sizeof(argv[0]), argv);
  return NULL;
}

#endif /* _OBJ_DETECTION_ */
