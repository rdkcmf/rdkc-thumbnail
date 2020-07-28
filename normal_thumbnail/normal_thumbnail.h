/**
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
**/

/***** Included Header FIle *****/
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <mongoose.h>
#include <pthread.h>
#include <semaphore.h>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/base/gstbasesrc.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>

#include <time.h>
#include <sys/time.h>

/***** RDK Logging *****/
#include "rdk_debug.h"
#include <sys/wait.h>
#include <unistd.h>    /* For pause() */

/***** Defines *****/
#define GST_ARRAYMAXSIZE 	150
#define GST_BUFFER_MULTIPLIER	1.5

/***** Structure ****/
typedef struct GST_YUV_Info
{
        guint8 *y_addr;
        guint8 *uv_addr;
        guint32 width;
        guint32 height;
	int size;
}GST_YUV_Info;

/***** static (or) global Variable *****/

static int    exit_flag;        /* Program termination flag     */

static int captured_frame_size = 0;
static guint8* captured_frame_buffer = NULL;

static int buffer_width = 1280;
static int buffer_height = 720;

pthread_attr_t attr;
pthread_t getYUVFrame;
pthread_t getJpegBuffer;
sem_t yuvsem;

/***** Prototype *****/

int main(int argc, char *argv[]);
static void signal_handler(int sig_num);
static void *start_streaming();
static gboolean on_message( GstBus * bus, GstMessage * message, gpointer user_data );
void  *on_new_sample( GstElement * elt );
int gst_ReadYUVData( GST_YUV_Info *yuv_info );
void GenerateJpegImage();
static void *stop_streaming();
static char *current_time(  );
