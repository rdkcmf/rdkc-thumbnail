#ifndef MPIPE_PORT_H__
#define MPIPE_PORT_H__

#include <vector>
#include <string>

typedef struct DetectionResult_ {
  /* The bounding boxes of each (n) person detected in the thumbnail */
  std::vector<std::vector<int>> personBBoxes;
  /* The detection score of each (n) person detected in the thumbnail */
  std::vector<float> personScores;
  float deliveryScore;
  int maxAugScore;
  int motion_frame_count;

} DetectionResult;

typedef struct detection_config_ {
    std::string input_video_path;
    std::string delivery_detection_graph_path;
    std::string delivery_detection_model_path;
    std::string frame_read_delay;
    std::string max_num_frames_cached_for_delivery_detection;
    std::string delivery_detection_model_min_score_threshold;
    std::string delivery_detection_min_score_threshold;
    std::string frame_count_to_process;
    std::string roi_filter;
    std::string motion_cue_filter;
    std::string size_filter_threshold;
    std::string doi_filter;
}DetectionConfig;

void *__mpipe_thread_main__(DetectionConfig config);

/* open the device in the given path. The 'width and height' is the resolution that
 * mpipe desires. If the camera does not support, the camera's resolution is updated
 * to widht and height */
int mpipe_port_initialize(const std::string &input_video_path, int &width, int &height);
int mpipe_port_initialize(int &width, int &height);

/* get next frame, in COLOR_RGB format */
cv::Mat mpipe_port_getNextFrame(std::vector<cv::Point>& roiCoords, std::vector<std::vector<cv::Point>>& motionblobs, cv::Mat& doi_bitmap);
void mpipe_port_term();


/* ----------------------------------------------------------------------
 * The following are mpipe APIs available for porting application to call.
 * porting application should not implement these APIs.
 * ----------------------------------------------------------------------*/
/* header only. Do not implement */
/* notify mpipe of last frame for person detection, in cv::Mat, RGB
 * the lastFrame can be empty or non-empty. If non-empty, it will be
 * process for person dection before continue onto delivery detection.
 */
void mpipe_port_onLastFrameEvent(const cv::Mat &thumbnail, int width, int height);
/* notify mpipe of motion deteced event */
/* header only. Do not implement */
void mpipe_port_onMotionEvent(bool motionDetected);
typedef void (* OnDetectionChanged_t) (const DetectionResult &detection_result);
/* mpipe will notify application of detection results */
/* header only. Do not implement */
void mpipe_port_setOnDetectionChanged(OnDetectionChanged_t cb);
#endif
