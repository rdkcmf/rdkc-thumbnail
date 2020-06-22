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

#include <normal_thumbnail.h>

/** @description: Triggered to get yuv data from mongoose server and triggered to generate JPEG image.
*/
/* {{{ main() */
int main(int argc, char *argv[])
{
	int ret;

    	/* RDK logger initialization */
    	rdk_logger_init("/etc/debug.ini");
	
    	gst_init(NULL,NULL);

    	(void) signal(SIGCHLD, signal_handler);

    	(void) signal(SIGTERM, signal_handler);

	(void) signal(SIGINT, signal_handler);

 	if ( -1 == sem_init( &yuvsem, 0, 0 ) )
        {
                RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Failed to create Semaphore \n",
                        __FILE__, __LINE__);
        }

	ret = pthread_attr_init(&attr);
	if(ret != 0 )
	{
		RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Failed to initialize attributes,error_code: %d\n",
			__FILE__, __LINE__,ret);
		return -1;
	}

	ret = pthread_create( &getYUVFrame, &attr, start_streaming, NULL );
	if(ret != 0)
	{
		RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Failed to create start_streaming thread,error_code: %d\n",
			__FILE__, __LINE__,ret);
	}

        ret = pthread_create( &getJpegBuffer, &attr, GenerateJpegImage, NULL );
        if(ret != 0)
        {
                RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Failed to create GenerateJpegImage thread,error_code: %d\n",
                        __FILE__, __LINE__,ret);
        }

   	while (exit_flag == 0)
  	{
        	sleep(1);

  	}

    RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Exiting on signal %d waiting for all threads to finish...",__FILE__, __LINE__,exit_flag);

    fflush(stdout);

    RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): done.\n", __FILE__, __LINE__);

    return (EXIT_SUCCESS);

}

/** @description: Operation performed based on signal value.
 *  @param[in]sig_num: signal value
 *  @return: None
 */
static void signal_handler(int sig_num)
{
    if (sig_num == SIGCHLD)
    {
        do
        {
        } while (waitpid(-1, &sig_num, WNOHANG) > 0);
    }
    else
    {
        exit_flag = sig_num;
    }
}

/** @description: Created pipeline to get yuv data from Mongoose server
 *  @param[in]: None
 *  @return: None
 */
static void *start_streaming()
{
       RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD", "%s(%d): Entering normalthumbnail start streaming thread...\n",
                __FILE__, __LINE__);

l1:     GstElement *souphttpsrc = NULL, *pipeline = NULL, *appsink = NULL;
        GMainLoop *loop = NULL;

        loop = g_main_loop_new( NULL, FALSE );

        appsink = gst_element_factory_make( "appsink", NULL );
        pipeline = gst_element_factory_make( "pipeline", NULL );
        souphttpsrc = gst_element_factory_make( "souphttpsrc", NULL );

        if ( !pipeline || !appsink || !souphttpsrc )
        {
                RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Failed to make elements %d %d %d \n",
                        __FILE__, __LINE__, pipeline, appsink, souphttpsrc);
                sleep(1);
                goto l1;
        }

        GstBus *bus = NULL;

        g_object_set( G_OBJECT( appsink ), "emit-signals", TRUE, "sync", FALSE,
                        NULL );

        bus = gst_pipeline_get_bus( GST_PIPELINE( pipeline ) );

        gst_bus_add_watch( bus, ( GstBusFunc ) on_message, ( gpointer ) loop );

        gst_bin_add_many( GST_BIN( pipeline ), souphttpsrc, appsink, NULL );

        if( !gst_element_link_many( souphttpsrc, appsink, NULL ) )
                RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Failed to link elements \n",__FILE__, __LINE__);

        g_signal_connect( appsink, "new-sample", G_CALLBACK( on_new_sample ),   NULL ); /* This API is called when frame is captured */

        /* Stream params with flags */
        int do_timestamp = 0;        /* timestamp required for Video */
        int framerate = 1;         /* Video framerate */
	char format[10] = "NV12"; /* Buffer format */
	char videotype[20] = "video/x-raw";/* capturing video type */
        char startrequest[GST_ARRAYMAXSIZE] = { 0 };

        sprintf( startrequest,
                        "http://127.0.0.1:8080/startstream&do_timestamp=%d&videotype=%s&format=%s&framerate=%d&width=%d&height=%d&",
                       do_timestamp,videotype,format,framerate, buffer_width, buffer_height );

        RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): gstrmsframe Start request URL is: %s\n",__FILE__, __LINE__,startrequest);

        g_object_set( G_OBJECT( souphttpsrc ), "location", startrequest, NULL );
        g_object_set( G_OBJECT( souphttpsrc ), "is-live", TRUE, NULL );
        g_object_set( G_OBJECT( souphttpsrc ), "blocksize", (buffer_width*buffer_height)*2, NULL );

        gst_element_set_state( pipeline, GST_STATE_PLAYING );

        RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): gst_element_set_state\n",__FILE__, __LINE__);

        g_main_loop_run( loop );

        RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Exited from normalthumbnail loop\n",__FILE__, __LINE__);

        gst_element_set_state( appsink, GST_STATE_READY );
        gst_element_set_state( souphttpsrc, GST_STATE_READY );
        gst_element_set_state( appsink, GST_STATE_NULL );
        gst_element_set_state( souphttpsrc, GST_STATE_NULL );

        gst_element_set_state( pipeline, GST_STATE_NULL );

        if ( gst_element_get_state( pipeline, NULL, NULL, GST_CLOCK_TIME_NONE ) ==
                                        GST_STATE_CHANGE_SUCCESS )
        {
                RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): The state of pipeline changed to GST_STATE_NULL successfully\n",
                        __FILE__, __LINE__);
        }
        else
        {
                RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Changing the state of pipeline to GST_STATE_NULL failed\n",
                        __FILE__, __LINE__);
        }

        gst_element_unlink_many( souphttpsrc, appsink, NULL );

        gst_object_ref( souphttpsrc );
        gst_object_ref( appsink );

        gst_bin_remove_many( GST_BIN( pipeline ), appsink, souphttpsrc, NULL );

        gst_object_unref( bus );
        gst_object_unref( pipeline );
        gst_object_unref( souphttpsrc );
        gst_object_unref( appsink );

        RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Pipeline deleted\n",__FILE__, __LINE__);
        RDK_LOG( RDK_LOG_INFO,"LOG.RDL.THUMBNAILUPLOAD","************* Exiting GST stream start  \n");

        return 0;

}
/* }}} */

/** @description: Callback function from GST on EOS or stream error message
 *  @param[in] bus: GSTBus
 *  @param[in] message: GstMessage (EOS or ERROR)
 *  @param[in] user_data: User data related to stream
 *  @return: None
 */
static gboolean on_message( GstBus * bus, GstMessage * message, gpointer user_data )
{
        GError *err = NULL;
        gchar *dbg_info = NULL;

        switch ( GST_MESSAGE_TYPE( message ) )
        {
                case GST_MESSAGE_EOS:
                        RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): EOS of startstream request reached\n",__FILE__, __LINE__);
                        g_main_loop_quit( ( GMainLoop * ) user_data );
                        break;

                case GST_MESSAGE_ERROR:
                        RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Stream request ERROR has occured,error_code: %d\n",
                                __FILE__, __LINE__,GST_MESSAGE_ERROR);
                        gst_message_parse_error (message, &err, &dbg_info);
                        g_error_free (err);
                        g_free (dbg_info);
                        g_main_loop_quit( ( GMainLoop * ) user_data );
                        break;

                default:
                        break;
        }

        return TRUE;
}

/** @description: Callback function from GST pipeline to get yuv data from Mongoose server.
 *  @param[in] elt: Gstreamer element
 *  @return: None
 */
void  *on_new_sample( GstElement * elt )
{
	GstSample *sample;
	GstBuffer *buffer;
	GstMapInfo map;
	int frame_tot_size = 0;

	sample = gst_app_sink_pull_sample( GST_APP_SINK( elt ) );
	buffer = gst_sample_get_buffer( sample );
	gst_buffer_map( buffer, &map, GST_MAP_READ );

        RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.THUMBNAILUPLOAD", "%s(%d): Entering New Sample Callback \n",__FILE__, __LINE__ );

        frame_tot_size = buffer_width * buffer_height * GST_BUFFER_MULTIPLIER;

        if( NULL == captured_frame_buffer)
        {
                captured_frame_buffer = ( guint8 * ) ( malloc( frame_tot_size ) );

                RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Allocated address for Captured Frame buffer =%d ...\n", __FILE__, __LINE__, captured_frame_buffer);

                if(captured_frame_buffer == NULL)
                {
                        RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Malloc for frameBuffer failed...\n", __FILE__, __LINE__);
                        return -1;
		}
        }

        if((captured_frame_size + map.size) > (frame_tot_size))
        {
		int semvalue;

                memcpy( (captured_frame_buffer+captured_frame_size), map.data, ( frame_tot_size-captured_frame_size));

		captured_frame_size = 0;

		sem_getvalue(&yuvsem, &semvalue);
                if ( semvalue <=0 && -1 == sem_post( &yuvsem ) )
                {
                        RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Semaphore posting failed...\n", __FILE__, __LINE__);
                }	
        }
        else
        {
                memcpy( (captured_frame_buffer+captured_frame_size), map.data, map.size);
                captured_frame_size += map.size;
        }

	return NULL;
}

/** @description: Read YUV data based on gstreamer output.
 *  @param[in] yuv_info: YUV buffer structure.
 *  @return: Success ro failure.
 */
int gst_ReadYUVData( GST_YUV_Info *yuv_info )
{
        RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Entering gst_ReadYUVData() fn \n",__FILE__, __LINE__);

	yuv_info->size = buffer_width * buffer_height;
        yuv_info->y_addr = (unsigned char *) malloc (yuv_info->size);
        yuv_info->uv_addr = (unsigned char *) malloc(yuv_info->size);

	if( ( NULL == yuv_info->y_addr ) || ( NULL == yuv_info->uv_addr ) )
	{
		RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Memory Allocattion Failed \n",__FILE__, __LINE__);
		return false;
	}

        if((captured_frame_buffer != NULL))
        {
             yuv_info->width = buffer_width;
             yuv_info->height = buffer_height;
             memcpy ( yuv_info->y_addr, captured_frame_buffer, (yuv_info->size) );
             memcpy ( yuv_info->uv_addr, captured_frame_buffer + (yuv_info->size), ( yuv_info->size * 0.5 ) );

             RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): YUV frame data Read\n",__FILE__, __LINE__);
        }

	return true;
}

/** @description: Generate Jpegimage with opencv.
 *  @param[in]: None
 *  @return: None
 */
void GenerateJpegImage()
{
        GST_YUV_Info yuv_info = { 0 };
        cv::Mat RGBMat;
        cv::Mat yuvMat;
        static unsigned char * yuvSnapshotData;
	char *sys_time = NULL;
	char filename[50];
	int ret = 0;

        if ( -1 == sem_wait( &yuvsem )  )
	{
                RDK_LOG( RDK_LOG_ERROR, "LOG.RDK.THUMBNAILUPLOAD", "%s(%d): Error in sem wait of yuvsem..!\n", __FILE__, __LINE__ );
	}

        ret = gst_ReadYUVData( &yuv_info );

	if( !ret )
	{
		RDK_LOG( RDK_LOG_ERROR, "LOG.RDK.THUMBNAILUPLOAD", "%s(%d): Read yuv data failed\n", __FILE__, __LINE__ );
		captured_frame_size = 0;
		return false;
	}
        /* Allocate memory as per the frame width and height */
        yuvSnapshotData = (unsigned char *) malloc((yuv_info.size + (yuv_info.size/2)) * sizeof(unsigned char));

        if(NULL == yuvSnapshotData ) {
                RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Invalid pointer to yuvData\n",__FUNCTION__,__LINE__);
                return -1;
        }

        /* Converting an image buffer into Mat */
        memset( yuvSnapshotData, 0, (yuv_info.size + (yuv_info.size/2)) * sizeof(unsigned char));
        memcpy( yuvSnapshotData, yuv_info.y_addr,yuv_info.size);
        memcpy( yuvSnapshotData+yuv_info.size,yuv_info.uv_addr,yuv_info.size/2 );
        yuvMat = cv::Mat( yuv_info.height + yuv_info.height/2, yuv_info.width, CV_8UC1, yuvSnapshotData );

        /* matrix to store color image */
        RGBMat = cv::Mat( yuv_info.height, yuv_info.width, CV_8UC4);

        /* convert the frame to BGR format */
        cv::cvtColor(yuvMat, RGBMat, CV_YUV2BGR_NV12);

    	sys_time = current_time(  );
	
	if( NULL != sys_time)
	{
   		sprintf( filename, "/tmp/thumbnail%s.jpeg", sys_time );

        	cv::imwrite(filename, RGBMat);

		free( sys_time );
		sys_time = NULL;
	}

        /* Release memory */
        if( NULL != yuvSnapshotData ) {
                free(yuvSnapshotData);
                yuvSnapshotData = NULL;
        }

	if( NULL != captured_frame_buffer )
	{
		free( captured_frame_buffer );
		captured_frame_buffer = NULL;

		captured_frame_size = 0;
	}

	stop_streaming();
	exit_flag = SIGTERM;
}

/** @description: Get Current syste date and time.
 *  @param[in]: None
 *  @return: System data and time
 */
static char *current_time(  )
{
    time_t time_now;
    struct tm *timeinfo;
    char *tm_buffer = ( char * ) malloc( sizeof( char ) * 21 );

    if ( NULL == tm_buffer )
    {
        RDK_LOG( RDK_LOG_ERROR, "LOG.RDK.NORMALTHUMBNAIL",
                 "%s(%d): Failed to allocate memory for tm_buffer\n",
                 __FILE__, __LINE__ );
    }

    time( &time_now );
    timeinfo = localtime( &time_now );

    strftime( tm_buffer, 21, "%F:%T.", timeinfo );  //Setting format of time

    return tm_buffer;
}

/** @description: Stop streming.
 *  @param[in]: None
 *  @return: None
 */
static void *stop_streaming()
{
       RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD", "%s(%d): Entering normalthumbnail stop streaming thread...\n",
                __FILE__, __LINE__);

l1:     GstElement *souphttpsrc = NULL, *pipeline = NULL, *appsink = NULL;
        GMainLoop *loop = NULL;

        loop = g_main_loop_new( NULL, FALSE );

        appsink = gst_element_factory_make( "appsink", NULL );
        pipeline = gst_element_factory_make( "pipeline", NULL );
        souphttpsrc = gst_element_factory_make( "souphttpsrc", NULL );

        if ( !pipeline || !appsink || !souphttpsrc )
        {
                RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Failed to make elements %d %d %d \n",
                        __FILE__, __LINE__, pipeline, appsink, souphttpsrc);

                sleep(1);
                goto l1;
        }

        GstBus *bus = NULL;

        g_object_set( G_OBJECT( appsink ), "emit-signals", TRUE, "sync", FALSE,
                        NULL );
        bus = gst_pipeline_get_bus( GST_PIPELINE( pipeline ) );

        gst_bus_add_watch( bus, ( GstBusFunc ) on_message, ( gpointer ) loop );

        gst_bin_add_many( GST_BIN( pipeline ), souphttpsrc, appsink, NULL );

        if( !gst_element_link_many( souphttpsrc, appsink, NULL ) )
                RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Failed to link elements \n",__FILE__, __LINE__);

        char stoprequest[GST_ARRAYMAXSIZE] = { 0 };

        sprintf( stoprequest,
                        "http://127.0.0.1:8080/stopstream");

	RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): gstrmsframe Stop request URL is: %s\n",__FILE__, __LINE__,stoprequest);

        g_object_set( G_OBJECT( souphttpsrc ), "location", stoprequest, NULL );

        gst_element_set_state( pipeline, GST_STATE_PLAYING );

        RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): gst_element_set_state\n",__FILE__, __LINE__);

        g_main_loop_run( loop );

        RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Exited from gstrmsframe loop\n",__FILE__, __LINE__);

        gst_element_set_state( appsink, GST_STATE_READY );
        gst_element_set_state( souphttpsrc, GST_STATE_READY );
        gst_element_set_state( appsink, GST_STATE_NULL );
        gst_element_set_state( souphttpsrc, GST_STATE_NULL );

        gst_element_set_state( pipeline, GST_STATE_NULL );

        if ( gst_element_get_state( pipeline, NULL, NULL, GST_CLOCK_TIME_NONE ) ==
                                        GST_STATE_CHANGE_SUCCESS )
        {
                RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): The state of pipeline changed to GST_STATE_NULL successfully\n",
                        __FILE__, __LINE__);
        }
        else
        {
                RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Changing the state of pipeline to GST_STATE_NULL failed\n",
                        __FILE__, __LINE__);
        }

        gst_element_unlink_many( souphttpsrc, appsink, NULL );

        gst_object_ref( souphttpsrc );
        gst_object_ref( appsink );

        gst_bin_remove_many( GST_BIN( pipeline ), appsink, souphttpsrc, NULL );

        gst_object_unref( bus );
        gst_object_unref( pipeline );
        gst_object_unref( souphttpsrc );
        gst_object_unref( appsink );

        RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Pipeline deleted\n",__FILE__, __LINE__);
        RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","************* Exiting GST stream start  \n");

        return 0;
}

