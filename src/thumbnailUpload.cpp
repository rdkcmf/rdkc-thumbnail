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
#include "thumbnailUpload.h"
extern "C"
{
#include "secure_wrapper.h"
}
ThumbnailUpload *ThumbnailUpload::thumbnailUpload = NULL;
int  ThumbnailUpload::tn_upload_interval = DEFAULT_THUMBNAIL_UPLOAD_PASSIVE_INTERVAL;
bool ThumbnailUpload::tn_upload_enable = DEFAULT_THUMBNAIL_UPLOAD_ENABLE;
volatile sig_atomic_t  ThumbnailUpload::term_flag = 0;
int  ThumbnailUpload::activeUploadDuration = 0;

bool ThumbnailUpload::isActiveInterval = false;
char ThumbnailUpload::fw_name[FW_NAME_MAX_LENGTH] = "";
char ThumbnailUpload::ver_num[VER_NUM_MAX_LENGTH] = "";
char ThumbnailUpload::mac_string[THUMBNAIL_UPLOAD_MAC_STRING_LEN+1] = "";
char ThumbnailUpload::modelName[THUMBNAIL_UPLOAD_MAC_STRING_LEN+1] = "";
unsigned int ThumbnailUpload::activeModeUploadCounter = 0;
int ThumbnailUpload::uploadRetryCount = 0;

rtConnection ThumbnailUpload::con = NULL;

/** @description initialize rtmessage connection.
 */
void ThumbnailUpload::rtConnection_init()
{
  rtLog_SetLevel(RT_LOG_INFO);
  rtLog_SetOption(rdkLog);
  rtConnection_Create(&con, "THUMBNAILUPLOAD", WEBPA_ADDRESS);
  //Add listener for thumbnail topics
  rtConnection_AddListener(con, RTMSG_THUMBNAIL_TOPIC, onMessage, con);
  //Add listener for dynamic log topics
  rtConnection_AddListener(con, RTMSG_DYNAMIC_LOG_REQ_RES_TOPIC, dynLogOnMessage, con);
#ifdef _HAS_DING_
  rtConnection_AddListener(con, "RDKC.BUTTON.DOORBELL",onDingNotification, con);
#endif
}

/** @description ThumbnailUpload constructor.
 */
ThumbnailUpload::ThumbnailUpload():http_client(NULL)
				, tn_upload_file_name(NULL)
				, m_avgUploadSpeed(0)
				, m_count(0)
				, json_prov_tn_upload_enabled(-1)
				, liveCacheConf(NULL)
				, m_uploadReady(false)
#ifdef _HAS_DING_
                                , m_ding(NULL)
			  	, m_dingNotif(false)
                                , m_dingTime(0)

#endif
{
	char url_string[THUMBNAIL_UPLOAD_PARAM_MAX_LENGTH+1];
	m_smVector.reserve(10);
	http_client = new HttpClient();
	if(NULL == http_client)
	{
		RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Failed to create HttpClient instance \n", __FILE__, __LINE__);
	}

	/*get upload url*/
	strncpy(tn_upload_server_url, DEFAULT_THUMBNAIL_UPLOAD_URL, THUMBNAIL_UPLOAD_PARAM_MAX_LENGTH);
	strncpy(tn_upload_auth_token, DEFAULT_THUMBNAIL_UPLOAD_AUTH_TOKEN, THUMBNAIL_UPLOAD_AUTH_MAX_LENGTH);

	/* get upload file name */
	tn_upload_file_name = (char*)malloc(SIZE);
	if(NULL != tn_upload_file_name)
	{
#ifdef SUPPORT_IMAGETOOLS
		sprintf(tn_upload_file_name,"%s",SNAPSHOT_FILE);
#else
		snprintf(tn_upload_file_name, strlen(gcpThumbnailSnapshotPath)+1, "%s", gcpThumbnailSnapshotPath);
#endif
	}

	memset(url_string, 0, sizeof(url_string));
	snprintf(url_string, THUMBNAIL_UPLOAD_PARAM_MAX_LENGTH, "%s/%s/thumbnail", tn_upload_server_url, mac_string);

	/* Open the URL */
	if(NULL != http_client)
	{
		/* Create connection */
		RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Opening http client connection with URL %s\n", __FILE__, __LINE__,url_string);
		http_client->open((char*)url_string, DEFAULT_DNS_CACHE_TIMEOUT);
	}
	else
	{
		RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Failed to open the URL\n", __FILE__, __LINE__);
	}
	memset(cmd,0,MAXSIZE);
	snprintf(cmd,sizeof(cmd)-1, "rdkc_snapshooter %s %d %d %d", SNAPSHOT_FILE, COMPRESSION_SCALE, TN_OP_WIDTH, TN_OP_HEIGHT);

	liveCacheConf = (livecache_provision_info_t*) malloc(sizeof(livecache_provision_info_t));
	int ret = getCameraImageName(fw_name);
        	
	if (ret == RDKC_FAILURE)
	{
		RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): ERROR in reading camera firmware name\n", __FILE__, __LINE__);
	}

        int ret1 = getCameraVersionNum(ver_num);
        if (ret1 == RDKC_FAILURE)
	{
		RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): ERROR in reading camera version num\n", __FILE__, __LINE__);
	}
#if !defined ( THUMBNAIL_PLATFORM_RPI )
#ifdef OSI
	memset(mac_string, 0, sizeof(mac_string));
	char mac[CONFIG_STRING_MAX];
        mfrSerializedData_t stdata = {NULL, 0, NULL};
        mfrSerializedType_t stdatatype = mfrSERIALIZED_TYPE_DEVICEMAC;
        
        if(mfrGetSerializedData(stdatatype, &stdata) == mfrERR_NONE)
        {       
                strncpy(mac,stdata.buf,stdata.bufLen);
                mac[stdata.bufLen] = '\0';
                RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d):mac= %s,%s,%d\n",__FILE__, __LINE__,mac,stdata.buf,stdata.bufLen);
                
                char tmpMac[CONFIG_STRING_MAX+1];
                char *tmpField;
                int fieldNum=0;
                
                strcpy(tmpMac, mac);
                tmpField = strtok(tmpMac, ":");
                
                while (tmpField != NULL && fieldNum < 6)
                {       
                     char *chk;
                     unsigned long tmpVal;
                        
                     tmpVal = strtoul(tmpField, &chk, 16);
                        
                      if (tmpVal > 0xff)
                      {       
                            RDK_LOG( RDK_LOG_WARN,"LOG.RDK.THUMBNAILUPLOAD","field %d value %0x out of range\n", fieldNum, tmpVal);
                      }
                      if (*chk != 0)
                      {       
                            RDK_LOG( RDK_LOG_WARN,"LOG.RDK.THUMBNAILUPLOAD","Non-digit character %c (%0x) detected in field %d\n", *chk, *chk, fieldNum);
                      }
                        
                      fieldNum++;
                      strcat(mac_string, tmpField);
                      tmpField = strtok(NULL, ":");
               	}
                mac_string[THUMBNAIL_UPLOAD_MAC_STRING_LEN+1] = '\0';
                
                RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d):mac address= %s\n",__FILE__, __LINE__,mac_string);
                if (stdata.freeBuf != NULL)
                {       
                       	stdata.freeBuf(stdata.buf);
                       	stdata.buf = NULL;
                }
        }                

	stdata = {NULL, 0, NULL};
        stdatatype = mfrSERIALIZED_TYPE_MODELNAME;
        if(mfrGetSerializedData(stdatatype, &stdata) == mfrERR_NONE)
        {
                strncpy(modelName,stdata.buf,stdata.bufLen);
		modelName[stdata.bufLen] = '\0';
                RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d):Model Name = %s,%s,%d\n",__FILE__, __LINE__,modelName,stdata.buf,stdata.bufLen);
                if (stdata.freeBuf != NULL)
                {
                        stdata.freeBuf(stdata.buf);
                        stdata.buf = NULL;
                }
        }
        else
        {
         	RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d):GET ModelName failed : %d\n", __FILE__, __LINE__);
        }
#else
	unsigned char macaddr[MAC_ADDR_LEN];

	if (0 == get_mac_address(macaddr)) {
                memset(mac_string, 0, sizeof(mac_string));
                transcode_mac_to_string_by_separator(macaddr, '\0', mac_string, XFINITY_MAC_STRING_LEN+1, 0);
        }
	else {
		RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): ERROR in reading camera mac address\n", __FILE__, __LINE__);
		strcpy(mac_string,"No MACADDR");
	}
#endif
#endif
	RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): ThumbnailUpload constructor \n", __FILE__, __LINE__);
#ifdef _HAS_DING_
        m_ding = DingNotification::getInstance();
        m_ding->init(modelName,mac_string,fw_name);
#endif
}

void ThumbnailUpload::rtConnection_destroy()
{
  rtConnection_Destroy(con);
}

/** @description ThumbnailUpload destructor.
 */
ThumbnailUpload::~ThumbnailUpload()
{
	if(NULL != http_client)
	{
		RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Closing http client connection.\n", __FILE__, __LINE__);
		http_client->close();
	}

	if(NULL != http_client)
	{
		delete http_client;
		http_client = NULL;
	}
	if(NULL != tn_upload_file_name)
	{
		free(tn_upload_file_name);
		tn_upload_file_name = NULL;
	}

	if(liveCacheConf) {
		free(liveCacheConf);
		liveCacheConf = NULL;
	}
 
	RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): ThumbnailUpload destructor \n", __FILE__, __LINE__);
}

/**
 * @description Check file lock used for process control.
 *
 *  @param fname, file name to be checked.
 *
 *  @return fd, file descriptor.
 *
 */
int ThumbnailUpload::checkTNUploadfilelock(char *fname)
{
        int fd = -1;
        pid_t pid = -1;

        char str[50] = "thumbnail_upload" ;

        fd =  open(fname, O_WRONLY | O_CREAT | O_EXCL, 0644);
        if(fd < 0 && errno == EEXIST)
        {
                fd = open(fname, O_RDONLY, 0644);
                if (fd >= 0)
                {
                        read(fd, &pid, sizeof(pid));
                        kill(pid, SIGTERM);
                        close(fd);
                        sleep(1);
#if !defined ( OSI ) && !defined ( THUMBNAIL_PLATFORM_RPI )
                        if (CheckAppsPidAlive( (char*)str , pid))
                        {
                                kill(pid, SIGTERM);
                        }
#endif
                }
                unlink(fname);
                return -2;
        }

        return fd;
}

/**
 * @description: This function is used to control Thumbnail upload process.
 *
 * @param: process- main process.
 *
 * @return: Error code.
 */
int ThumbnailUpload:: controlTNUploadProcess(char *process)
{
	pid_t pid = 0;
        int file_fd = 0;

        file_fd = checkTNUploadfilelock((char*)LOCK_FILENAME_TNU);
        if ( -2 == file_fd)
        {
                file_fd = checkTNUploadfilelock((char*)LOCK_FILENAME_TNU);
        }

        if (file_fd < 0)
        {
                RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): %s run error!\n", __FILE__, __LINE__, process);
		unlink(LOCK_FILENAME_TNU);
                TNUExit();
                return RDKC_FAILURE;
        }

        pid = getpid();
        write(file_fd, &pid, sizeof(pid));
        close(file_fd);

        return RDKC_SUCCESS;


}

/** @description: This function is to get thumbnail upload attribute via JSON prov
 *
 *  @param: void.
 *
 *  @return: Error code, success or failure.
 *
 */
bool ThumbnailUpload::getTNUploadProvAttr()
{
	bool ret = true;

	if(!liveCacheConf)
	{
		ret = false;
		return ret;
	}

	if(RDKC_SUCCESS != polling_config_init()) {
		RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): polling config init failure.\n", __FILE__, __LINE__);
		ret = false;
	} else {
		/* Read live cache config */
		memset(liveCacheConf, 0, sizeof(livecache_provision_info_t));
		if (RDKC_SUCCESS != readLiveCacheConfig(liveCacheConf)) {
			RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Error reading livecache.conf.\n", __FILE__, __LINE__);
			ret = false;
		} else {
			json_prov_tn_upload_enabled = atoi(liveCacheConf->enable);
			/* Check thumbnail upload is enabled */
			if(json_prov_tn_upload_enabled) {
				memset(tn_upload_server_url, 0, sizeof(tn_upload_server_url));
				memset(tn_upload_auth_token, 0, sizeof(tn_upload_auth_token));

				strncpy(tn_upload_server_url, liveCacheConf->url, strlen(liveCacheConf->url));
				tn_upload_server_url[strlen(liveCacheConf->url)] = '\0';
				strncpy(tn_upload_auth_token, liveCacheConf->auth_token, strlen(liveCacheConf->auth_token));
				tn_upload_auth_token[strlen(liveCacheConf->auth_token)] = '\0';
			} else {
				RDK_LOG(RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): thumbnail upload not enabled.\n", __FILE__, __LINE__);
				ret = false;
			}
		}
	}
	polling_config_exit();

	RDK_LOG(RDK_LOG_INFO ,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Thumbnail Configuration: URL %s, enable %d\n", __FILE__, __LINE__, tn_upload_server_url, json_prov_tn_upload_enabled);

	return ret;
}


/**
 * @description: This function is used to get the instance for Thumbnail upload.
 *
 * @param: void
 *
 * @return: Thumbnail upload instance.
 */
ThumbnailUpload *ThumbnailUpload::getTNUploadInstance()
{
	if (NULL == thumbnailUpload)
	{
    		thumbnailUpload = new ThumbnailUpload();
		int ret = getCameraImageName(fw_name);
        	
		if (ret == RDKC_FAILURE)
		{
			RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): ERROR in reading camera firmware name\n", __FILE__, __LINE__);
		}

        	int ret1 = getCameraVersionNum(ver_num);
        	if (ret1 == RDKC_FAILURE)
		{
			RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): ERROR in reading camera version num\n", __FILE__, __LINE__);
		}
#if !defined ( THUMBNAIL_PLATFORM_RPI )
#ifdef OSI
		memset(mac_string, 0, sizeof(mac_string));
		char mac[CONFIG_STRING_MAX];
        	mfrSerializedData_t stdata = {NULL, 0, NULL};
        	mfrSerializedType_t stdatatype = mfrSERIALIZED_TYPE_DEVICEMAC;
        
        	if(mfrGetSerializedData(stdatatype, &stdata) == mfrERR_NONE)
        	{       
                	strncpy(mac,stdata.buf,stdata.bufLen);
                	mac[stdata.bufLen] = '\0';
                	RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d):mac= %s,%s,%d\n",__FILE__, __LINE__,mac,stdata.buf,stdata.bufLen);
                
                	char tmpMac[CONFIG_STRING_MAX+1];
                	char *tmpField;
                	int fieldNum=0;
                
                	strcpy(tmpMac, mac);
                	tmpField = strtok(tmpMac, ":");
                
                	while (tmpField != NULL && fieldNum < 6)
                	{       
                        	char *chk;
                        	unsigned long tmpVal;
                        
                        	tmpVal = strtoul(tmpField, &chk, 16);
                        
                       	 	if (tmpVal > 0xff)
                        	{       
                                	RDK_LOG( RDK_LOG_WARN,"LOG.RDK.THUMBNAILUPLOAD","field %d value %0x out of range\n", fieldNum, tmpVal);
                       	 	}
                        	if (*chk != 0)
                        	{       
                                	RDK_LOG( RDK_LOG_WARN,"LOG.RDK.THUMBNAILUPLOAD","Non-digit character %c (%0x) detected in field %d\n", *chk, *chk, fieldNum);
                        	}
                        
                        	fieldNum++;
                        	strcat(mac_string, tmpField);
                        	tmpField = strtok(NULL, ":");
                	}
                	mac_string[THUMBNAIL_UPLOAD_MAC_STRING_LEN+1] = '\0';
                
                	RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d):mac address= %s\n",__FILE__, __LINE__,mac_string);
                	if (stdata.freeBuf != NULL)
                	{       
                        	stdata.freeBuf(stdata.buf);
                        	stdata.buf = NULL;
                	}
        	}                

		//strcpy(mac_string,"142E5E063FE6");
#else
		unsigned char macaddr[MAC_ADDR_LEN];

		if (0 == get_mac_address(macaddr)) {
                	memset(mac_string, 0, sizeof(mac_string));
                	transcode_mac_to_string_by_separator(macaddr, '\0', mac_string, XFINITY_MAC_STRING_LEN+1, 0);
        	}
		else {
			RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): ERROR in reading camera mac address\n", __FILE__, __LINE__);
			strcpy(mac_string,"No MACADDR");
		}
#endif
#endif
	}
	return thumbnailUpload;
}

/** @description Thumbnail Upload Signal handler.
 *
 *  @param s signal
 *
 *  @return void.
 */
void ThumbnailUpload::TNUSignalHandler(int s)
{
        if (SIGTERM == s)
        {
                term_flag = 1;
        }
        else if (SIGINT == s)
        {
                term_flag = 1;
        }
	else if (SIGUSR1 == s)
	{
	}
}

/** @description Register Thumbnail Upload Signal handler.
 *  @param  void.
 *  @return void.
 */
void ThumbnailUpload::TNURegisterSignalHandler(void)
{
        signal(SIGTERM, TNUSignalHandler);
        signal(SIGINT, TNUSignalHandler);
	 signal(SIGUSR1, TNUSignalHandler);
}

int ThumbnailUpload::setUploadStatus(bool status)
{
    {
        std::unique_lock<std::mutex> lock(m_uploadMutex);
        m_uploadReady = status;
        lock.unlock();
    }

    m_cv.notify_one();
    return RDK_SUCCESS;
}

bool ThumbnailUpload::waitFor(int quiteInterVal)
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

/** @description: This function is to set thumbnail upload active interval from user config file
 *
 *  @param: void.
 *
 *  @return: void.
 *
 */
void ThumbnailUpload::setActiveInterval(void)
{
	time_t currentTime;
	char  thumbnail_upload_interval[THUMBNAIL_UPLOAD_PARAM_MAX_LENGTH];
	char  duration[THUMBNAIL_UPLOAD_PARAM_MAX_LENGTH];
	char* thumbnail_upload_interval_value = NULL ;
	char* duration_value = NULL;
#if defined ( OSI ) || defined ( THUMBNAIL_PLATFORM_RPI )
	currentTime = getCurrentTime(NULL); 
#else
	currentTime = sc_linear_time(NULL);
#endif
  	// Reset the upload count only if the mode moves from passive to active.
	if(false == isActiveInterval) {
	        RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Moving to Active mode. Resetting the Upload Count to 0\n", __FILE__, __LINE__);
		activeModeUploadCounter = 0;
	}
  	else {
          	RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Already in Active mode. Upload count %d\n", __FILE__, __LINE__, activeModeUploadCounter);
        }

	memset(thumbnail_upload_interval,0,THUMBNAIL_UPLOAD_PARAM_MAX_LENGTH);
	if(RDKC_SUCCESS != rdkc_get_user_setting(THUMBNAIL_UPLOAD_ACTIVE_INTERVAL, thumbnail_upload_interval))
	{
		thumbnail_upload_interval_value = (char*)rdkc_envGet(THUMBNAIL_UPLOAD_ACTIVE_INTERVAL);
	}
	else
	{
		thumbnail_upload_interval_value = thumbnail_upload_interval;
	}

	if (NULL != thumbnail_upload_interval_value)
	{
		tn_upload_interval = atoi(thumbnail_upload_interval_value);
		tn_upload_interval = (0 >= tn_upload_interval) ? DEFAULT_THUMBNAIL_UPLOAD_ACTIVE_INTERVAL: tn_upload_interval;
	}

	memset(duration,0,THUMBNAIL_UPLOAD_PARAM_MAX_LENGTH);
	if(RDKC_SUCCESS != rdkc_get_user_setting(THUMBNAIL_ACTIVE_UPLOAD_DURATION, duration))
	{
		duration_value = (char*)rdkc_envGet(THUMBNAIL_ACTIVE_UPLOAD_DURATION);
	}
	else
	{
		duration_value = duration;
	}

	if(NULL != duration_value)
	{
		activeUploadDuration = atoi(duration_value) + currentTime;
	}
	isActiveInterval = true;

}
/**
 * @description: Convert event date and time to ISO 8601 format.
 *
 * @param[in]: strEvtDateTime,evtdatetimeSize, evtDateTime.
 *
 * @return: void
 */
void ThumbnailUpload::stringifyEventDateTime(char* strEvtDateTime , size_t evtdatetimeSize, time_t evtDateTime)
{
        struct tm *tv = NULL;

        if(NULL == strEvtDateTime) {
                RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.CVRUPLOAD","%s(%d): Using invalid memory location!\n",__FILE__, __LINE__);
                return;
        }

        tv = gmtime(&evtDateTime);

        strftime(strEvtDateTime, evtdatetimeSize,"%FT%TZ",tv);
}

/** @description: This function is to update thumbnail upload active upload duration
 *
 *  @param: void.
 *
 *  @return: void.
 *
 */
int ThumbnailUpload::updateActiveUploadDuration()
{
	time_t currentTime = 0;
	char *duration = NULL;
        int ret = RDKC_SUCCESS;
	int uploadDuration = 0;
	char* duration_value = NULL;
#if defined ( OSI ) || defined ( THUMBNAIL_PLATFORM_RPI )
	currentTime = getCurrentTime(NULL);
#else
	currentTime = sc_linear_time(NULL);
#endif
	uploadDuration = activeUploadDuration - currentTime;

	duration = (char*)malloc(SIZE);
	memset(duration,0,SIZE);

	sprintf(duration, "%d", uploadDuration);
	if (-1 < uploadDuration)
	{
		if(RDKC_SUCCESS != rdkc_set_user_setting(THUMBNAIL_ACTIVE_UPLOAD_DURATION, duration))
		{
			RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Failed to release the paramters\n", __FILE__, __LINE__);
			ret = RDKC_FAILURE;
		}
	}
	else
	{
		if(RDKC_SUCCESS == rdkc_set_user_setting(THUMBNAIL_ACTIVE_UPLOAD_DURATION, "0"))
		{
			RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Settng back the Active Upload Duration as zero.\n", __FILE__, __LINE__);
		}
		isActiveInterval = false;
          	activeModeUploadCounter = 0;
                RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Moving to Passive mode. Resetting Active mode Upload count to 0 \n",__FILE__, __LINE__);
	}

	if(NULL != duration)
	{
		free(duration);
		duration = NULL;
	}

        return ret;

}

/** @description: This function is to get thumbnail upload attribute from user config file
 *
 *  @param: void.
 *
 *  @return: void.
 *
 */
void ThumbnailUpload::getTNUploadAttr()
{
	char  thumbnail_upload_interval[THUMBNAIL_UPLOAD_PARAM_MAX_LENGTH];
	char  thumbnail_enable[THUMBNAIL_UPLOAD_PARAM_MAX_LENGTH];
	char *thumbnail_upload_interval_value = NULL;
	char *thumbnail_enable_value = NULL;
	char  thumbnail_upload_server_url[THUMBNAIL_UPLOAD_PARAM_MAX_LENGTH];
	char *thumbnail_upload_server_url_value = NULL;

	RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.THUMBNAILUPLOAD","%s: %d: json_prov_tn_upload_enabled = %d Thumbnail Upload parameters : URL = %s, Interval = %d, Enabled = %s\n"
		, __FILE__, __LINE__,json_prov_tn_upload_enabled, tn_upload_server_url, tn_upload_interval, tn_upload_enable ? "true":"false");

	if(RDKC_SUCCESS != polling_config_init()) {
		RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): polling config init failure.\n", __FILE__, __LINE__);
	} else {
		if(liveCacheConf) {
			memset(liveCacheConf, 0, sizeof(livecache_provision_info_t));
			/* Read live cache config */
			if (RDKC_SUCCESS != readLiveCacheConfig(liveCacheConf)) {
				RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Error reading livecache.conf.\n", __FILE__, __LINE__);
			} else {
				json_prov_tn_upload_enabled = atoi(liveCacheConf->enable);
				/* Check thumbnail upload is enabled */
				if(json_prov_tn_upload_enabled) {
					memset(tn_upload_server_url, 0, sizeof(tn_upload_server_url));
					memset(tn_upload_auth_token, 0, sizeof(tn_upload_auth_token));
					//strcpy(tn_upload_server_url, liveCacheConf->url);
					//strcpy(tn_upload_auth_token, liveCacheConf->auth_token);
					strncpy(tn_upload_server_url, liveCacheConf->url, strlen(liveCacheConf->url));
					tn_upload_server_url[strlen(liveCacheConf->url)] = '\0';
					strncpy(tn_upload_auth_token, liveCacheConf->auth_token, strlen(liveCacheConf->auth_token));
					tn_upload_auth_token[strlen(liveCacheConf->auth_token)] = '\0';
					tn_upload_enable = true;
				} else {
					tn_upload_enable = false;
	                                RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Thumbnail is disabled from json config.\n", __FILE__, __LINE__);
				}
			}
			polling_config_exit();
		}
	}

	memset(thumbnail_upload_interval,0,THUMBNAIL_UPLOAD_PARAM_MAX_LENGTH);
	if(RDKC_SUCCESS != rdkc_get_user_setting((isActiveInterval ? THUMBNAIL_UPLOAD_ACTIVE_INTERVAL:THUMBNAIL_UPLOAD_PASSIVE_INTERVAL), thumbnail_upload_interval))
	{
		thumbnail_upload_interval_value = (char*)rdkc_envGet((isActiveInterval ? THUMBNAIL_UPLOAD_ACTIVE_INTERVAL:THUMBNAIL_UPLOAD_PASSIVE_INTERVAL));
	}
	else
	{
		thumbnail_upload_interval_value = thumbnail_upload_interval;
	}

	if (NULL != thumbnail_upload_interval_value)
	{
		tn_upload_interval = atoi(thumbnail_upload_interval_value);
		tn_upload_interval = (0 >= tn_upload_interval) ? (isActiveInterval ? DEFAULT_THUMBNAIL_UPLOAD_ACTIVE_INTERVAL : DEFAULT_THUMBNAIL_UPLOAD_PASSIVE_INTERVAL): tn_upload_interval;
	}

	RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s: %d: Thumbnail Upload parameters : URL = %s, Interval = %d, Enabled = %s\n"
		, __FILE__, __LINE__,tn_upload_server_url, tn_upload_interval, tn_upload_enable ? "true":"false");
}

/**
 * @description: This function is used to post the file to the http server.
 *
 * @param[in]: Path of the file to be uploaded, file length.
 * @param[out]: response code.
 *
 * @return: Error code.
 */
int ThumbnailUpload::postFileToTNUploadServer(char *file_path, int file_len, char* server_url, long *response_code)
{
	int ret = TN_UPLOAD_FAIL;
	int file_fd = 0;
	int read_len = 0;
	char read_buf[THUMBNAIL_UPLOAD_SEND_LEN];
	//struct timeval beforeUpload;
	//struct timeval afterUpload;
	struct timespec beforeUpload;
	struct timespec afterUpload;
	long int uploadDuration = 0;
	char *data=NULL;
	char *ptr = NULL;
	int curlCode = 0;

	if (NULL == file_path)
	{
		RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Invalid file name!\n", __FILE__, __LINE__);
		return TN_UPLOAD_FAIL;
	}

	file_fd = open(file_path, O_RDONLY);
	if (file_fd <= 0)
	{
		RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Failed to Open File :%s !\n", __FILE__, __LINE__, file_path);
		return TN_UPLOAD_FAIL;
	}

	data =(char*)malloc(file_len*sizeof(char));
	if(NULL == data)
	{
		RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Failed to allocate memory :%s !\n", __FILE__, __LINE__, file_path);
		close(file_fd);
		return TN_UPLOAD_FAIL;
	}
	memset(data,0,file_len);
	memset(read_buf,0,THUMBNAIL_UPLOAD_SEND_LEN);
	ptr=data;

	while((read_len = read(file_fd, read_buf, sizeof(read_buf))) > 0)
	{
		memcpy(ptr, read_buf, read_len);
		ptr += read_len;
		memset(read_buf,0,THUMBNAIL_UPLOAD_SEND_LEN);
	}

	if(NULL == http_client)
	{
		RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Data post Failed - http client is null .\n", __FILE__, __LINE__);
		ret = TN_UPLOAD_FAIL;
	}
	else
	{
		/* Time Just before Upload */
		//gettimeofday(&beforeUpload, NULL);
		clock_gettime(CLOCK_REALTIME, &beforeUpload);
		/*Uploading the file */
		curlCode =  http_client->post_binary((char*)server_url,(char*)data, response_code, file_len);

		/* Time Just after Upload */
		//gettimeofday(&afterUpload, NULL);
		clock_gettime(CLOCK_REALTIME, &afterUpload);
		uploadDuration = (afterUpload.tv_sec - beforeUpload.tv_sec)*1000 + ( afterUpload.tv_nsec - beforeUpload.tv_nsec)/1000000;

		if ((*response_code >= RDKC_HTTP_RESPONSE_OK) && (*response_code < RDKC_HTTP_RESPONSE_REDIRECT_START))
		{
			if (0 == uploadRetryCount ) {
				RDK_LOG(RDK_LOG_INFO ,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Data post Successfully, Response Code : %ld, Upload Duration(in ms) : %ld\n", __FILE__, __LINE__,*response_code, uploadDuration);
			} else {
				RDK_LOG(RDK_LOG_INFO ,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Data post Successfully after retry, Response Code : %ld, Upload Duration(in ms) : %ld Retry Count: %d\n", __FILE__, __LINE__,*response_code, uploadDuration, uploadRetryCount);
			}
			curl_off_t uploadSpeed = http_client->getUploadSpeed();
			curl_off_t u1 = 0;
			RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.THUMBNAILUPLOAD","Average download speed: %" CURL_FORMAT_CURL_OFF_T " kbyte/sec and m_count = %d \n", uploadSpeed,m_count);
			
			m_smVector.push_back(uploadSpeed);
			if(m_count > SMA_FACTOR)
			{
				u1 =  m_smVector.front();
				m_smVector.erase (m_smVector.begin());
				RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.THUMBNAILUPLOAD","front  %" CURL_FORMAT_CURL_OFF_T " kbyte/sec.\n", u1);
				m_avgUploadSpeed = m_avgUploadSpeed + uploadSpeed / (curl_off_t)SMA_FACTOR - u1 /(curl_off_t)SMA_FACTOR;													
			}
			else
			{
				if(m_count == SMA_FACTOR)
				{
					curl_off_t sum = 0;
			
					for (int i = 0; i < m_smVector.size(); i++) 
					{
						RDK_LOG(RDK_LOG_DEBUG ,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): m_smVector[%d]= %" CURL_FORMAT_CURL_OFF_T " kbyte/sec\n", __FILE__, __LINE__,i,m_smVector[i]);
						sum += m_smVector[i];
					}
					m_avgUploadSpeed = sum / (curl_off_t)SMA_FACTOR;
					//RDK_LOG(RDK_LOG_INFO ,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): 1st m_avgUploadSpeed after last 10 speeds %" CURL_FORMAT_CURL_OFF_T " bytes/sec\n", __FILE__, __LINE__,m_avgUploadSpeed);
    				}
			}
			
			if(m_avgUploadSpeed > 0)
			{
		        	FILE* fpUsr=NULL;
				if((fpUsr=fopen("/tmp/.upstream","w"))!=NULL) 
				{
					/*write the value to the user config file*/
                        		fprintf(fpUsr,"%" CURL_FORMAT_CURL_OFF_T "\n",m_avgUploadSpeed);
					fclose(fpUsr);
    				}
			}

   	
	
			if(m_count <= SMA_FACTOR)
				m_count++;
	
			RDK_LOG(RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","Simple Moving Average Upload speed:%" CURL_FORMAT_CURL_OFF_T "\n", m_avgUploadSpeed);
			ret = TN_UPLOAD_OK;
		}
		else
		{
			RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Data post Failed. Response code = %ld : curl code = %d\n", __FILE__, __LINE__, *response_code,curlCode);
			ret = TN_UPLOAD_FAIL;
		}
	}
	if(NULL != data)
	{
		free(data);
		data = NULL;
	}
	close(file_fd);

	return ret;
}

/**
 * @description: This function is used to free memory.
 *
 * @param: void
 *
 * @return: void.
 */
void ThumbnailUpload::releaseResources()
{
	if(NULL != tn_upload_file_name)
	{
		RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Remove file [%s] from camera ram.\n", __FILE__, __LINE__, tn_upload_file_name);
		unlink(tn_upload_file_name);               // Remove the file at the end
	}

}

/**
 * @description: This function is used to exit thumbnail upload process.
 *
 * @param: void
 *
 * @return: void.
 */
void ThumbnailUpload::TNUExit()
{
	//unlink(LOCK_FILENAME_TNU);
	releaseResources();
}

/**
 * @description: This function is used to call the function which upload data to the http server.
 *
 * @param[in]: File path,start time,end time, event type,event date time, m file path, motion level, num of arguments.
 *
 * @return: Error code.
 */
int ThumbnailUpload::uploadThumbnailImage()
{
	int ret = TN_UPLOAD_OK;
	int file_len = 0;
	struct stat file_stat;
	char pack_head[THUMBNAIL_UPLOAD_SEND_LEN+1];
//	unsigned char macaddr[MAC_ADDR_LEN];
	char url_string[THUMBNAIL_UPLOAD_PARAM_MAX_LENGTH+1];
	long response_code = 0;
	int ret_jpeg = 0;
	char dTnTStamp[256]={0};

	/* As customer's requirement, not support http */
	if (!strncasecmp(tn_upload_server_url,"http://", strlen("http://")))
	{
		RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Invalid upload url [%s], not support http!\n",__FILE__, __LINE__, tn_upload_server_url);
		ret = TN_UPLOAD_FAIL;
		releaseResources();
		return ret;
	}

#ifdef SUPPORT_IMAGETOOLS
	/* Generate thumbnail image using imagetool(OpenCV) utility */
	ret_jpeg = system(cmd);
	if(-1 == ret_jpeg)
        {
                RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): GenerateThumbnail failed!\n", __FILE__, __LINE__);
                ret = TN_UPLOAD_FAIL;
                releaseResources();
                return ret;
        }

#else
	/* Generate thumbnail image using snapshooter utility */
	//snprintf(cmd,sizeof(cmd)-1, "snapshooter -f %s %s >/dev/null 2>/dev/null", gcpThumbnailSnapshotPath, gcpSnapshooterOpt);

	//ret_jpeg = system(cmd);
         ret_jpeg = v_secure_system("snapshooter -f %s %s >/dev/null 2>/dev/null", gcpThumbnailSnapshotPath, gcpSnapshooterOpt);
	if(-1 == ret_jpeg)
	{
		RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): system call - snapshooter failed!\n", __FILE__, __LINE__);
		ret = TN_UPLOAD_FAIL;
		releaseResources();
		return ret;
	}
#endif

	if(NULL == tn_upload_file_name)
	{
		RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Invalid file\n", __FILE__, __LINE__);
		ret = TN_UPLOAD_FAIL;
                releaseResources();
                return ret;
	}

	/* get file attribute */
	if (stat(tn_upload_file_name, &file_stat) < 0)
	{
		RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): invalid file [%s], errmsg=%s!\n",
				__FILE__, __LINE__, tn_upload_file_name, strerror(errno));
		ret = TN_UPLOAD_FAIL;
		releaseResources();
		return ret;
	}
	file_len = file_stat.st_size;

	/*Adding camera mac in the Server URL */
	memset(url_string, 0, sizeof(url_string));
	snprintf(url_string, THUMBNAIL_UPLOAD_PARAM_MAX_LENGTH, "%s/%s/thumbnail", tn_upload_server_url, mac_string);

        RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Post file %s to %s with size : %d\n", __FILE__, __LINE__, tn_upload_file_name, url_string, file_len);

	http_client->resetHeaderList();

	//logging few chars of token being sent for triage
	if(0 != strlen(tn_upload_auth_token)) {
		printf("Auth token begins with (%.10s) and ends with (%s)\n", tn_upload_auth_token,tn_upload_auth_token + strlen(tn_upload_auth_token) - 5);
	}

	/* Adding Header */
	http_client->addHeader( "Expect", "");   //removing expect header condition by explicitly setting Expect header to ""
	memset(pack_head, 0, sizeof(pack_head));
	snprintf(pack_head, sizeof(pack_head), "%s", tn_upload_auth_token);
	http_client->addHeader( "Authorization", pack_head);
	memset(pack_head, 0, sizeof(pack_head));
	snprintf(pack_head, sizeof(pack_head), "image/jpeg");
	http_client->addHeader( "Content-Type", pack_head);
	memset(pack_head, 0, sizeof(pack_head));
	snprintf(pack_head, sizeof(pack_head), "%d",file_len);
	http_client->addHeader( "Content-Length", pack_head);
	memset(pack_head, 0, sizeof(pack_head));
	snprintf(pack_head, sizeof(pack_head), "%s", mac_string);
	http_client->addHeader( "X-Device-MAC", pack_head);
	memset(pack_head, 0, sizeof(pack_head));
	snprintf(pack_head, sizeof(pack_head), "%s", "thumbnail");
	http_client->addHeader( "X-Upload-Type", pack_head);
	memset(pack_head, 0, sizeof(pack_head));
	snprintf(pack_head, sizeof(pack_head), "%s",DEFAULT_THUMBNAIL_UPLOAD_RESOLUTION);
	http_client->addHeader( "X-Image-Resolution", pack_head);
	memset(pack_head, 0, sizeof(pack_head));
	snprintf(pack_head, sizeof(pack_head), "%" CURL_FORMAT_CURL_OFF_T "",m_avgUploadSpeed);
	http_client->addHeader( "X-Upload-Speed", pack_head);
        memset(pack_head, 0, sizeof(pack_head));
#ifdef USE_MFRLIB
        snprintf(pack_head, sizeof(pack_head), "Sercomm %s %s %s %s", modelName, fw_name, mac_string, ver_num);
#else

#if !defined ( THUMBNAIL_PLATFORM_RPI )
	snprintf(pack_head, sizeof(pack_head), "Sercomm %s %s %s %s", SC_MODEL_NAME, fw_name, mac_string, ver_num);
#endif

#endif
	http_client->addHeader( "User-Agent", pack_head);
#ifdef _HAS_DING_	
	if(thumbnailUpload ->m_dingNotif)
	{
            stringifyEventDateTime(dTnTStamp, sizeof(dTnTStamp), thumbnailUpload ->m_dingTime);
            memset(pack_head, 0, sizeof(pack_head));
            http_client->addHeader( "X-EVENT-TYPE", "ding");
            snprintf(pack_head, sizeof(pack_head), "%s", dTnTStamp);
            http_client->addHeader( "X-EVENT-DATETIME", pack_head);
            RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Ding: X-EVENT-DATETIME: %s\n",__FUNCTION__,__LINE__,dTnTStamp);
	    thumbnailUpload ->m_dingTime = 0;
	}	
#endif
	/* Send file to server */
	ret = postFileToTNUploadServer(tn_upload_file_name, file_len, url_string, &response_code);

	if(TN_UPLOAD_OK == ret) {
		if(true == isActiveInterval) {
			activeModeUploadCounter += 1;
	        	RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Active mode, Upload Count %d\n",__FILE__, __LINE__,activeModeUploadCounter);
		}
#ifdef _HAS_DING_
		if(thumbnailUpload ->m_dingNotif)
		{
            		RDK_LOG( RDK_LOG_INFO,"LOG.RDK.BUTTONMGR","%s(%d): Low resolution thumbnail upload corresponding to ding is successful with header X-EVENT-DATETIME: %s\n",__FUNCTION__,__LINE__,dTnTStamp);
                }
          	m_dingNotif = false;
#endif
	}
	else
	{
#ifdef _HAS_DING_
		if(thumbnailUpload ->m_dingNotif)
                {
                        RDK_LOG( RDK_LOG_INFO,"LOG.RDK.BUTTONMGR","%s(%d): Ligh resolution thumbnail upload corresponding to ding  is failed.\n",__FUNCTION__,__LINE__);
                }
          	m_dingNotif = false;
#endif
	}
	releaseResources();
	return ret;
}

/** @description:Callback function for the message received
 *  @param[in] hdr : pointer to rtMessage Header
 *  @param[in] buff : buffer for data received via rt message
 *  @param[in] n : number of bytes received
 *  @return: void
 */
void ThumbnailUpload::dynLogOnMessage(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure)
{
        char const*  module = NULL;
        char const*  logLevel = NULL;

        rtConnection con = (rtConnection) closure;

        rtMessage req;
        rtMessage_FromBytes(&req, buff, n);

        //Handle the rtmessage request
        if (rtMessageHeader_IsRequest(hdr))
        {
                char* tmp_buff = NULL;
                uint32_t tmp_buff_length = 0;

                rtMessage_ToString(req, &tmp_buff, &tmp_buff_length);
                rtLog_Info("Req : %.*s", tmp_buff_length, tmp_buff);
                free(tmp_buff);

                rtMessage_GetString(req, "module", &module);
                rtMessage_GetString(req, "logLevel", &logLevel);

                RDK_LOG(RDK_LOG_INFO,"LOG.RDK.DYN/AMICLOG","(%s):%d Module name: %s\n", __FUNCTION__, __LINE__, module);
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
#ifdef _HAS_DING_
/** @description    : Callback function to generate ding notification and smartthumbnail.
 *  @param[in]  hdr : constant pointer rtMessageHeader
 *  @param[in] buff : constant pointer uint8_t
 *  @param[in]    n : uint32_t
 *  @param[in] closure : void pointer
 *  @return: void
 */

void ThumbnailUpload::onDingNotification(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure)
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

        if((currTime.tv_sec - thumbnailUpload ->m_dingTime) > thumbnailUpload->m_ding->getQuiteTime())
        {
           thumbnailUpload ->m_dingTime = currTime.tv_sec;
           thumbnailUpload -> m_dingNotif = true;
           thumbnailUpload->m_ding->signalDing(true,thumbnailUpload ->m_dingTime);
	   thumbnailUpload->setUploadStatus(true);

        }
    }
    rtMessage_Release(req);
}
#endif

void ThumbnailUpload::onMessage(rtMessageHeader const* hdr, uint8_t const* buff, uint32_t n, void* closure)
{
  rtConnection con = (rtConnection) closure;

  rtMessage req;
  rtMessage_FromBytes(&req, buff, n);

  if (rtMessageHeader_IsRequest(hdr))
  {
    char* tmp_buff = NULL;
    uint32_t tmp_buff_length = 0;

    rtMessage_ToString(req, &tmp_buff, &tmp_buff_length);
    rtLog_Debug("Req : %.*s", tmp_buff_length, tmp_buff);
    free(tmp_buff);

    // create response
    rtMessage res;
    rtMessage_Create(&res);
    rtMessage_SetString(res, "reply", "Success");
    setActiveInterval();
    rtConnection_SendResponse(con, hdr, res, 1000);
    rtMessage_Release(res);
  }
  rtMessage_Release(req);
//kill(getpid(), SIGUSR1); // Sending signal to own process to wake up from the existing sleep
  thumbnailUpload->setUploadStatus(true);

}


void* ThumbnailUpload::rtMessage_Receive(void* arg)
{
  while (1)
  {
    rtError err = rtConnection_Dispatch(con);
    if (err != RT_OK)
      RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Dispatch Error: %s", __FILE__, __LINE__, rtStrError(err));
  }
}


/** @description: Thread function to continuously do thumbnail upload on configured interval
 *
 *  @param: void.
 *
 *  @return: void.
 */
void *ThumbnailUpload::doTNUpload()
{
	time_t start_upload_time = 0;
	time_t start_active_time = 0;
	time_t current_time = 0;
	time_t waitingInterval = 0;
	time_t durationLeft = 0;
	static int tn_active_uploadfailcount=0;
	static int tn_passive_uploadfailcount=0;
	pid_t pid = 0;
	uploadRetryCount = 0;

        while (!term_flag)
	{
		//Getting Upload Attribute
		ThumbnailUpload::getTNUploadInstance()->getTNUploadAttr();
		#if defined ( OSI ) || defined ( THUMBNAIL_PLATFORM_RPI )
        	current_time = getCurrentTime(NULL);
		#else
        	current_time = sc_linear_time(NULL);
		#endif
		if((0 != start_upload_time) && ((tn_upload_interval + start_upload_time) > current_time))
		{
			waitingInterval = ((tn_upload_interval + start_upload_time) - current_time);
			durationLeft = waitingInterval;
			RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Wait till next upload interval, waiting for %d secs\n",__FILE__, __LINE__,waitingInterval);
			if (true == isActiveInterval)
			{
				if(0 != start_upload_time)
				{
					#if defined ( OSI ) || defined ( THUMBNAIL_PLATFORM_RPI)
        				start_active_time = getCurrentTime(NULL);
					#else
        				start_active_time = sc_linear_time(NULL);
					#endif
					while(0 < durationLeft)
					{
						if(NULL != ThumbnailUpload::getTNUploadInstance())
						{
							if (true == isActiveInterval)
							{
								if(RDKC_SUCCESS != ThumbnailUpload::getTNUploadInstance()->updateActiveUploadDuration())
								{
									RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Failed to update active upload duration..\n", __FILE__, __LINE__);
								}
							}
							else
							{
								RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d):Breaking, Active Upload Duration completed.\n",__FILE__, __LINE__);
								start_active_time = 0;
								break;
							}
						}
						sleep(1);
						#if defined ( OSI ) || defined ( THUMBNAIL_PLATFORM_RPI )
                                        	current_time = getCurrentTime(NULL);
                                        	#else
                                        	current_time = sc_linear_time(NULL);
                                        	#endif

						durationLeft = waitingInterval + start_active_time - current_time;
					}

					if(0 != durationLeft)
					{
						durationLeft = 0;
						continue;
					}
				}
			}
			else
			{
				bool isTimedOut = thumbnailUpload->waitFor(waitingInterval);
                        	//Upon ding we need to process the motion notification immediatley
                        	if(!isTimedOut)
                        	{
                                	RDK_LOG( RDK_LOG_INFO,"LOG.RDK.SMARTTHUMBNAIL","%s(%d): Ding or Active suration set!!!  need to send thumbnail immediatly \n", __FILE__, __LINE__);
                        	}

				//Sleep with passive interval
				//sleep(waitingInterval);
			}
		}

		if(term_flag) {
			break;
		}

		//Starting Time
 		#if defined ( OSI ) || defined ( THUMBNAIL_PLATFORM_RPI )
                start_upload_time = getCurrentTime(NULL);
              	#else
                start_upload_time = sc_linear_time(NULL);
                #endif


		/* Check whether thumbnail upload is enabled */
		if (true == tn_upload_enable)
		{
			time_t upload_start_time; // scope of "upload_start_time" is limited to this block
			time_t time_for_upload;
			//Reset the retry count
			uploadRetryCount = 0;
	
			/*Uploading the Thumbnail Image*/
			do {
				#if defined ( OSI ) || defined ( THUMBNAIL_PLATFORM_RPI )
                		upload_start_time = getCurrentTime(NULL);
                		#else
                		upload_start_time = sc_linear_time(NULL);
                		#endif
				
				if (TN_UPLOAD_OK != thumbnailUpload->uploadThumbnailImage())
				{
					RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Error Upload Thumbnail\n",__FILE__, __LINE__);
					if (true == isActiveInterval)
					{
						tn_active_uploadfailcount++;
						if( ACTIVE_TN_THRESHHOLD_COUNT == tn_active_uploadfailcount )
						{
							RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Threshold limit reached : tn_active_uploadfailcount : %d : sending sigterm \n", __FUNCTION__, __LINE__,tn_active_uploadfailcount);
							tn_active_uploadfailcount = 0;
							pid = getpid();
							kill(pid, SIGTERM);
						}
					}
					else
					{
						tn_passive_uploadfailcount++;
						if( PASSIVE_TN_THRESHHOLD_COUNT == tn_passive_uploadfailcount )
						{
							RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Threshold limit reached : tn_passive_uploadfailcount : %d : sending sigterm \n", __FUNCTION__, __LINE__,tn_passive_uploadfailcount);
							tn_passive_uploadfailcount = 0;
							pid = getpid();
							kill(pid, SIGTERM);
						}
					}				
						
					uploadRetryCount++;

					// break if "total upload time" including all the retries exceeds the current "upload interval"
					#if defined ( OSI ) || defined ( THUMBNAIL_PLATFORM_RPI )
					if( ((getCurrentTime(NULL) - start_upload_time) >= tn_upload_interval) || (uploadRetryCount > MAX_UPLOAD_RETRY) ) {
                                                break;
                                        }

					#else
					if( ((sc_linear_time(NULL) - start_upload_time) >= tn_upload_interval) || (uploadRetryCount > MAX_UPLOAD_RETRY) ) {
						break;
					}
					#endif

					// calculate the time taken for the failed upload
					#if defined ( OSI ) || defined ( THUMBNAIL_PLATFORM_RPI)
					time_for_upload = getCurrentTime(NULL) - upload_start_time;
					#else
					time_for_upload = sc_linear_time(NULL) - upload_start_time;
					#endif
					// Retry should happen for the leftover time
					if( (time_for_upload < 0) ||  (time_for_upload >= MAX_RETRY_SLEEP) ) {
						RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Retry Happens after 0 seconds\n",__FILE__, __LINE__);	
					}
					else if( (time_for_upload < MAX_RETRY_SLEEP) ) {
						RDK_LOG( RDK_LOG_INFO,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Retry Happens after %d seconds\n",__FILE__, __LINE__, (MAX_RETRY_SLEEP - time_for_upload) );
						sleep( MAX_RETRY_SLEEP - time_for_upload ); // Retry every 10(MAX_RETRY_SLEEP) seconds
					}
				} else {
					tn_active_uploadfailcount = 0;
					tn_passive_uploadfailcount = 0;
					uploadRetryCount = 0;
					break;
				}
			#if defined ( OSI ) || defined ( THUMBNAIL_PLATFORM_RPI )
			} while( (uploadRetryCount <= MAX_UPLOAD_RETRY) && ((getCurrentTime(NULL) - start_upload_time) < tn_upload_interval) );
			#else
			} while( (uploadRetryCount <= MAX_UPLOAD_RETRY) && ((sc_linear_time(NULL) - start_upload_time) < tn_upload_interval) );
			#endif
			//Reset the retry count
			uploadRetryCount = 0;
		}
		else
		{
			RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Upload is disable!\n", __FILE__, __LINE__);
			thumbnailUpload->releaseResources();
		}

		if(true == isActiveInterval)
                {
			#if defined ( OSI ) || defined ( THUMBNAIL_PLATFORM_RPI )
			start_active_time = getCurrentTime(NULL);
			#else
			start_active_time = sc_linear_time(NULL);
			#endif
                }
	}

	return NULL;
}

/**
 * @description: This function is used to get the camera firmware version.
 *
 * @param[out]: firmware name
 *
 * @return: Error code.
 */
 
int getCameraImageName(char* out)
{
        size_t max_line_length = FW_NAME_MAX_LENGTH;
        char *file_buffer;
        char *locate_1 = NULL;
        FILE* fp;
        char* temp = out;
        fp = fopen("/version.txt","r");
        if(fp == NULL)
        {
               RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Error in opening version.txt\n", __FILE__, __LINE__);
                return RDKC_FAILURE;
        }

        file_buffer = (char*)malloc(FW_NAME_MAX_LENGTH + 1);
        if(file_buffer == NULL)
        {
               RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): malloc failed\n", __FILE__, __LINE__);
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
                        fclose(fp);
                        return RDKC_SUCCESS;
                }
        }
        /* unable to get the image name */
        //WalError("unable to get the image name");
        strcpy(out,"imagename entry not found");
        free(file_buffer);
        fclose(fp);
        return RDKC_SUCCESS;
}

/**
 * @description: This function is used to get the camera version number.
 *
 * @param[out]: version number
 *
 * @return: Error code.
 */

int getCameraVersionNum(char* out)
{
        size_t max_line_length = VER_NUM_MAX_LENGTH;
        char *file_buffer;
        char *locate_1 = NULL;
        FILE* fp;
        char* temp = out;
        fp = fopen("/version.txt","r");
        if(fp == NULL)
        {
               RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Error in opening version.txt\n", __FILE__, __LINE__);
                return RDKC_FAILURE;
        }

        file_buffer = (char*)malloc(VER_NUM_MAX_LENGTH + 1);
        if(file_buffer == NULL)
        {
               RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): malloc failed\n", __FILE__, __LINE__);
                fclose(fp);
                return RDKC_FAILURE;
        }
        while(getline(&file_buffer,&max_line_length,fp) != -1)
        {
                /* find the imagename string */
                locate_1 = strstr(file_buffer,"VERSION");
                if(locate_1)
                {
                        locate_1 += strlen("VERSION=");
                        /* copy the contents till linefeed */
                        while(*locate_1 != '\n')
                                *out++ = *locate_1++;
                        free(file_buffer);
                        fclose(fp);
                        return RDKC_SUCCESS;
                }
        }
        /* unable to get the image name */
        //WalError("unable to get the image name");
        strcpy(out,"No Ver Num");
        free(file_buffer);
        fclose(fp);
        return RDKC_SUCCESS;
}



