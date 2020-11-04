
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

#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace std;
using namespace cv;

int main( int argc, const char** argv );
void detectAndDraw( Mat& img, CascadeClassifier& cascade );
static char *current_time(  );

/* {{{ main() */
int main( int argc, const char** argv )
{
    VideoCapture capture;
    Mat frame;

    CascadeClassifier cascade;

    cascade.load( "/lib/rdk/haarcascade_frontalface_alt.xml" ) ;

    capture.open(0);

    if( capture.isOpened() )
    {
        printf(" \n Face Detection Started....");
        
        while(1)
        {
            capture >> frame;

            if( frame.empty() )
                break;

            Mat frame1 = frame.clone();

            detectAndDraw( frame1, cascade );
        }
    }
    else
    {
        printf("\n Could not open camera");
    }
}
/* }}} */

/* {{{ detectAndDraw() */
void detectAndDraw( Mat& img, CascadeClassifier& cascade )
{
    vector<Rect> faces;
    char *sys_time = NULL;
    char filename[50];

    cascade.detectMultiScale( img, faces, 1.1,
                             2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );

    if( faces.size() >= 1 )
    {
        sys_time = current_time(  );

        if( NULL != sys_time)
        {
            sprintf( filename, "/var/www/pages/image/thumbnail%s.jpeg", sys_time );

            imwrite(filename, img);

            free( sys_time );
            sys_time = NULL;
        }

        printf("\n Face Detected From Live Camera Buffer...");
    }
    
}
/* }}} */

/* {{{ current_time() */
static char *current_time(  )
{
    time_t time_now;
    struct tm *timeinfo;
    char *tm_buffer = ( char * ) malloc( sizeof( char ) * 21 );

    if ( NULL == tm_buffer )
    {
        printf( "\n %s(%d): Failed to allocate memory for tm_buffer\n",
                 __FILE__, __LINE__ );
    }

    time( &time_now );
    timeinfo = localtime( &time_now );

    strftime( tm_buffer, 21, "%F:%T.", timeinfo );  //Setting format of time

    return tm_buffer;
}
/* }}} */
