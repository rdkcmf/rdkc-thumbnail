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
#include "DingNotification.h"
extern "C"
{
#include "secure_wrapper.h"
}

DingNotification* DingNotification::m_Instance = NULL;
volatile bool DingNotification::m_termFlag = false;
volatile bool DingNotification::m_confRefreshed = true;
int DingNotification::waitInterval = 1 ;

/** @description: Constructor
 *  @param[in] void
 *  @return: void
*/

DingNotification::DingNotification():
				m_httpClient(NULL),
				m_uploadReady(false),
				m_dingTime(0),
				m_quiteTime(DEFAULT_QUITE_TIME),
				m_dnsCacheTimeout(DEFAULT_DNS_CACHE_TIMEOUT),
    				m_snapShotHeight(HEIGHT),
    				m_snapShotWidth(WIDTH)
{
    memset(m_dingNotifUploadURL, 0, sizeof(m_dingNotifUploadURL));
    memset(m_dingNotifAuthCode, 0, sizeof(m_dingNotifAuthCode));
    memset(m_snapShotUploadURL,0,sizeof(m_snapShotUploadURL));
    memset(m_snapShotAuthCode, 0,sizeof(m_snapShotAuthCode));
    memset(m_modelName, 0, sizeof(m_modelName));
    memset(m_macAddress, 0, sizeof(m_macAddress));
    memset(m_firmwareName, 0, sizeof(m_firmwareName));
    RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.BUTTONMGR","%s(%d): DingNotification constructor invoked.\n", __FUNCTION__, __LINE__);
}

/** @description: Destructor
 *  @param[in] void
 *  @return: void
 */
DingNotification::~DingNotification()
{
   RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.BUTTONMGR","%s(%d): DingNotification destructor invoked.\n", __FUNCTION__, __LINE__);
   memset(m_dingNotifUploadURL, 0, sizeof(m_dingNotifUploadURL));
   memset(m_dingNotifAuthCode, 0, sizeof(m_dingNotifAuthCode));
   memset(m_modelName, 0, sizeof(m_modelName));
   memset(m_macAddress, 0, sizeof(m_macAddress));
   memset(m_firmwareName, 0, sizeof(m_firmwareName));
    
   if(m_httpClient)
   {
	m_httpClient->close();
	delete m_httpClient;
   }
}
/** @description: creates the instance for DingNotification
 *  @param[in] void
 *  @return: pointer to instance of DingNotification
 */
DingNotification *DingNotification::getInstance()
{
    RDK_LOG( RDK_LOG_INFO,"LOG.RDK.BUTTONMGR","%s(%d): Creating smart thumbnail instance.\n", __FUNCTION__, __LINE__);
    if (!m_Instance) {
        m_Instance =  new DingNotification();
    }
    RDK_LOG( RDK_LOG_INFO,"LOG.RDK.BUTTONMGR","%s(%d): Smart thumbnail instance created.\n", __FUNCTION__, __LINE__);

    return m_Instance;
}


void DingNotification::init(char* mac,char* modelName,char* firmware)
{
   snprintf(m_macAddress, sizeof(m_macAddress), "%s",mac);	
   snprintf(m_modelName, sizeof(m_modelName), "%s",modelName);	
   snprintf(m_firmwareName, sizeof(m_firmwareName), "%s",firmware);	
   m_httpClient = new HttpClient(); 
   //Initialize upload routine
   RDK_LOG( RDK_LOG_INFO,"LOG.RDK.BUTTONMGR","%s(%d): Creating smart thumbnail upload thread.\n", __FUNCTION__, __LINE__);
   std::thread uploadDingNotifThread(monitorDingNotification);
   uploadDingNotifThread.detach();

   

}
bool DingNotification::waitForDing()
{
    bool status = false;
    {
        std::unique_lock<std::mutex> lock(m_dingMutex);
        m_cv.wait(lock, [this] {return (m_uploadReady || m_termFlag);});

        if(!m_termFlag) {
            RDK_LOG( RDK_LOG_INFO,"LOG.RDK.BUTTONMGR","%s(%d): Wait over due to uploadReady flag!!\n",__FUNCTION__,__LINE__);
            status = m_uploadReady;
            m_uploadReady = false;
        } else {
            RDK_LOG( RDK_LOG_INFO,"LOG.RDK.BUTTONMGR","%s(%d): Wait over due to term flag!!\n",__FUNCTION__,__LINE__);
            status = false;
        }

        lock.unlock();
    }
    return status;
}
bool DingNotification::signalDing(bool status,uint64_t currTime)
{
    m_dingTime = currTime;
    {
        std::unique_lock<std::mutex> lock(m_dingMutex);
        m_uploadReady = status;
        lock.unlock();
    }

    m_cv.notify_one();
    return RDKC_SUCCESS;
}

int  DingNotification::getQuiteTime()
{
	getDingConf();
	return m_quiteTime;
}


int  DingNotification::getDingConf()
{
    ding_config_info_t *dingCfg = NULL;
    bool retry = true;

    // Read Thumbnail config.
    dingCfg = (ding_config_info_t*) malloc(sizeof(ding_config_info_t));

    if (NULL == dingCfg) {
        RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.BUTTONMGR","%s(%d): Error allocating memory.\n", __FILE__, __LINE__);
        return RDKC_FAILURE;
    }

    while (retry) {
        if (RDKC_SUCCESS != readDingConfig(dingCfg)) {
            RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.BUTTONMGR","%s(%d): Error reading TNConfig.\n", __FILE__, __LINE__);
        } else {
            break;
        }
        //Sleep 10 sec before retrying
        sleep(10);
    }

    // get url and auth
    snprintf(m_dingNotifUploadURL, sizeof(m_dingNotifUploadURL), "%s",dingCfg->url);
    snprintf(m_dingNotifAuthCode, sizeof(m_dingNotifAuthCode), "%s",dingCfg->auth_token);
    if (strlen(dingCfg->quite_interval) > 0) {
       m_quiteTime = atoi(dingCfg->quite_interval);
    }

    if (dingCfg) {
        free(dingCfg);
        dingCfg = NULL;
    }

    return RDKC_SUCCESS;

}
/** @description: Get snap shot upload conf
 *  @param[in] void
 *  @return: RDKC_SUCCESS on success, RDKC_ERROR otherwise
 */
int DingNotification::getTnUploadConf()
{
    tn_provision_info_t *stnCfg = NULL;
    bool retry = true;

    // Read Thumbnail config.
    stnCfg = (tn_provision_info_t*) malloc(sizeof(tn_provision_info_t));

    if (NULL == stnCfg) {
        RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.BUTTONMGR","%s(%d): Error allocating memory.\n", __FILE__, __LINE__);
        return RDKC_FAILURE;
    }
    
    if (RDKC_SUCCESS != readTNConfig(stnCfg)) {
            RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.BUTTONMGR","%s(%d): Error reading TNConfig.\n", __FILE__, __LINE__);
	    return RDKC_FAILURE;
    }

    // get url and auth
    strcpy(m_snapShotUploadURL, stnCfg -> url);
    strcpy(m_snapShotAuthCode, stnCfg -> auth_token);
    m_snapShotHeight = atoi(stnCfg -> height);
    m_snapShotWidth = atoi(stnCfg -> width);

    if (stnCfg) {
        free(stnCfg);
        stnCfg = NULL;
    }

    return RDKC_SUCCESS;
}
bool  DingNotification::monitorDingNotification()
{
   while (!m_termFlag)
   {
	if(m_confRefreshed){
            RDK_LOG( RDK_LOG_INFO,"LOG.RDK.BUTTONMGR","%s(%d): Fetching new URL and auth token\n", __FUNCTION__, __LINE__);
	    memset(m_Instance->m_dingNotifUploadURL, 0, sizeof(m_Instance->m_dingNotifUploadURL));
   	    memset(m_Instance->m_dingNotifAuthCode, 0, sizeof(m_Instance->m_dingNotifAuthCode));
            if(RDKC_SUCCESS == m_Instance->getDingConf()) {
               	 RDK_LOG( RDK_LOG_INFO,"LOG.RDK.BUTTONMGR","%s(%d): getDingConf success m_dingNotifUploadURL=%s m_quiteTime=%d\n", __FUNCTION__, __LINE__,m_Instance->m_dingNotifUploadURL,m_Instance->m_quiteTime);
            }
            m_confRefreshed = false;
        }
	RDK_LOG(RDK_LOG_INFO,"LOG.RDK.BUTTONMGR","%s(%d): Waiting for Ding\n", __FUNCTION__, __LINE__);
        if(m_Instance->waitForDing()) {
            m_Instance->sendDingNotification();
        }
	RDK_LOG(RDK_LOG_DEBUG,"LOG.RDK.BUTTONMGR","%s(%d): Process the Ding\n", __FUNCTION__, __LINE__);
     }
     RDK_LOG( RDK_LOG_INFO,"LOG.RDK.BUTTONMGR","%s(%d): Exiting smart thumbnail upload thread.\n",__FUNCTION__,__LINE__);
}

/**
 * @description: This function is used to sleep for some waitingInterval  before next retry.
 *
 * @return: Error code.
 */
int DingNotification::retryAtExpRate()
{
    int ret = RDKC_FAILURE;
    int retryFactor = 2;

    if(waitInterval <= UPLOAD_TIME_INTERVAL)
    {
        RDK_LOG( RDK_LOG_DEBUG,"LOG.RDK.BUTTONMGR","%s: %d: Waiting for %d seconds!\n", __FILE__, __LINE__, (int)waitInterval);
        sleep(waitInterval);
        waitInterval *= retryFactor;
        ret = RDKC_SUCCESS;
    }
    return ret;
}

void DingNotification::stringifyDateTime(char* strEvtDateTime , size_t evtdatetimeSize, time_t evtDateTime)
{
        struct tm *tv = NULL;

        if(NULL == strEvtDateTime) {
                RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.BUTTONMGR","%s(%d): Using invalid memory location!\n",__FILE__, __LINE__);
                return;
        }

        tv = gmtime(&evtDateTime);

        strftime(strEvtDateTime, evtdatetimeSize,"%FT%TZ",tv);
}

/** @description: thread routine to upload DingNotification
 *  @param[in] : time left
 *  @return: void
 */
void DingNotification::sendDingNotification()
{
    int curlCode = 0;
    long response_code = 0;
    char packHead[UPLOAD_DATA_LEN+1];
    int retry = 0;
    int remainingTime = 0;
    const char* data = "";
    char dingT[256]={0};
    struct timespec currTime;
    struct timespec startTime;
    memset(&startTime, 0, sizeof(startTime));
    memset(&currTime, 0, sizeof(currTime));
    stringifyDateTime(dingT,sizeof(dingT),m_dingTime);
    /* Open the URL */
    if (NULL != m_Instance->m_httpClient) {
         m_Instance->m_httpClient->open(m_Instance->m_dingNotifUploadURL,m_Instance->m_dnsCacheTimeout);
    } else {
        RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.BUTTONMGR","%s(%d): Failed to open the URL\n", __FUNCTION__, __LINE__);
	return;
    }

    //clock the start time
    clock_gettime(CLOCK_REALTIME, &startTime);

    RDK_LOG(RDK_LOG_INFO,"LOG.RDK.BUTTONMGR","%s(%d):Ding notification with User-Agent (%s,%s,%s)\n", __FILE__, __LINE__, m_modelName,m_macAddress,m_firmwareName);

    while (true) {
	memset(&currTime, 0, sizeof(currTime));
        clock_gettime(CLOCK_REALTIME, &currTime);

        //Check for max retry or time limit and break if so
        if ( (retry >= MAX_RETRY_COUNT) ||
            ((currTime.tv_sec - startTime.tv_sec) > UPLOAD_TIME_INTERVAL) ) {
            RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.BUTTONMGR", "%s(%d): Max retry count/time exceeded, Retry count %d Time spent %d. currTime.tv_sec %d startTime.tv_sec %d Upload failed!!!\n", __FILE__,__LINE__, retry, (currTime.tv_sec - startTime.tv_sec), currTime.tv_sec, startTime.tv_sec);
            break;
        }


	clock_gettime(CLOCK_REALTIME, &currTime);

	/* Add Header */
        m_httpClient->resetHeaderList();
        m_httpClient->addHeader( "Expect", "");   //removing expect header condition by explicitly setting Expect header to ""
        memset(packHead, 0, sizeof(packHead));
        snprintf(packHead, sizeof(packHead), "%s", m_dingNotifAuthCode);
        m_httpClient->addHeader( "Authorization", packHead);
        m_httpClient->addHeader( "X-EVENT-TYPE", "ding");
	//memset(packHead, 0, sizeof(packHead));
        //snprintf(packHead, sizeof(packHead), "application/json");
        //m_httpClient->addHeader( "Content-Type", packHead);
        memset(packHead, 0, sizeof(packHead));
        snprintf(packHead, sizeof(packHead), "Sercomm %s %s %s", m_modelName, m_firmwareName, m_macAddress);
        m_httpClient->addHeader( "User-Agent", packHead);
        memset(packHead, 0, sizeof(packHead));
	snprintf(packHead, sizeof(packHead), "%s", dingT);
        m_httpClient->addHeader( "X-EVENT-DATETIME", packHead);
	memset(packHead, 0, sizeof(packHead));
        snprintf(packHead, sizeof(packHead), "%d",0);
        m_httpClient->addHeader( "Content-Length", packHead);
	RDK_LOG(RDK_LOG_TRACE1,"LOG.RDK.BUTTONMGR","%s(%d):Posting Ding notification to  %s \n", __FILE__, __LINE__,m_dingNotifUploadURL );
	curlCode =  m_httpClient->post(m_dingNotifUploadURL, data, &response_code);
        if ((response_code >= RDKC_HTTP_RESPONSE_OK) && (response_code < RDKC_HTTP_RESPONSE_REDIRECT_START)){	
	    break;
        } 
	else{
            retry++;
           if(RDKC_SUCCESS != retryAtExpRate()){
               RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.BUTTONMGR","%s(%d): retries done with current exp wait interval %d\n",__FILE__, __LINE__,waitInterval);
               break;
           }
           RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.BUTTONMGR","%s(%d): Retrying again for %d times because last try failed  with response code %lu and curl code %d\n",__FUNCTION__,__LINE__,retry, response_code, curlCode);
        }
    }
    //log success/failure for telemetry
    if ((response_code >= RDKC_HTTP_RESPONSE_OK) && (response_code < RDKC_HTTP_RESPONSE_REDIRECT_START)) {
            RDK_LOG( RDK_LOG_INFO,"LOG.RDK.BUTTONMGR","%s(%d): Posting Ding is successfull with header X-EVENT-DATETIME: %s\n",__FUNCTION__,__LINE__,dingT);
    }else {
            RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.BUTTONMGR","%s(%d): Posting Ding is failed with response code %lu and curl code %d\n",__FUNCTION__,__LINE__, response_code, curlCode);
    }
    if (NULL != m_Instance->m_httpClient) {
         m_Instance->m_httpClient->close();
    }
    m_Instance->uploadSnapShot();
}

void  DingNotification::captureSnapShot()
{
  int ret = 0;

  if(RDKC_SUCCESS != getTnUploadConf())
  {
	RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.BUTTONMGR","%s(%d): Failed to read the configuration\n",__FUNCTION__,__LINE__);
	return;
  }

  ret = v_secure_system("rdkc_snapshooter %s %d %d %d",SNAPSHOT_FILE,COMPRESSION_SCALE,m_snapShotWidth,m_snapShotHeight);
  
  if(-1 == ret)
  {
	RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.BUTTONMGR","%s(%d): system call - snapshooter failed!\n", __FILE__, __LINE__);
  }
  return;
}
void  DingNotification::uploadSnapShot()
{
  /* get file attribute */
  	int file_fd = 0;
	int file_len = 0;
  	int read_len = 0;
	struct stat file_stat;
  	char read_buf[UPLOAD_DATA_LEN];
  	char *data=NULL;
  	char *ptr = NULL;
	long response_code = 0;
  	int curlCode = 0;
	int uploadRetryCount = 0;
	struct timespec startTime;
        struct timespec currTime;;
        long int uploadDuration = 0;
	char packHead[UPLOAD_DATA_LEN+1];
	char dingT[256]={0};
        int isCVREnabled = 0;
	RDK_LOG( RDK_LOG_INFO,"LOG.RDK.BUTTONMGR","%s(%d): Capturing snapShot\n",__FUNCTION__,__LINE__);
	m_Instance->captureSnapShot();
	
  	if (stat(SNAPSHOT_FILE, &file_stat) < 0)
  	{
        	RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.BUTTONMGR","%s(%d): invalid file [%s], errmsg=%s\n", __FILE__, __LINE__, SNAPSHOT_FILE, strerror(errno));
        	return;
  	}
	
  	file_len = file_stat.st_size;

  	file_fd = open(SNAPSHOT_FILE, O_RDONLY);

  	if (file_fd <= 0)
  	{
		RDK_LOG( RDK_LOG_INFO,"LOG.RDK.BUTTONMGR","%s(%d): ERROR 1\n",__FUNCTION__,__LINE__);
      		RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.BUTTONMGR","%s(%d): Failed to Open File :%s !\n", __FILE__, __LINE__,SNAPSHOT_FILE);
      		return;
  	}
  	data =(char*)malloc(file_len*sizeof(char));
  	if(NULL == data)
        {
                RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.THUMBNAILUPLOAD","%s(%d): Failed to allocate memory :%s !\n", __FILE__, __LINE__,SNAPSHOT_FILE);
                close(file_fd);
                return;
        }
        memset(data,0,file_len);
        memset(read_buf,0,UPLOAD_DATA_LEN);
        ptr=data;

        while((read_len = read(file_fd, read_buf, sizeof(read_buf))) > 0)
        {
                memcpy(ptr, read_buf, read_len);
                ptr += read_len;
                memset(read_buf,0,UPLOAD_DATA_LEN);
        }
	if (NULL != m_Instance->m_httpClient) {
         	m_Instance->m_httpClient->open(m_Instance->m_snapShotUploadURL,m_Instance->m_dnsCacheTimeout);
    	} 
	else 
	{
        	RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.BUTTONMGR","%s(%d): Failed to open the URL\n", __FUNCTION__, __LINE__);
        	return;
    	}
	stringifyDateTime(dingT,sizeof(dingT),m_dingTime);
	m_httpClient->resetHeaderList();
        m_httpClient->addHeader( "Expect", "");   //removing expect header condition by explicitly setting Expect header to ""
        m_httpClient->addHeader( "X-EVENT-TYPE", "ding");
        memset(packHead, 0, sizeof(packHead));
        snprintf(packHead, sizeof(packHead), "%s", m_snapShotAuthCode);
        m_httpClient->addHeader( "Authorization", packHead);
        memset(packHead, 0, sizeof(packHead));
        snprintf(packHead, sizeof(packHead), "image/jpeg");
        m_httpClient->addHeader( "Content-Type", packHead);
        memset(packHead, 0, sizeof(packHead));
        snprintf(packHead, sizeof(packHead), "Sercomm %s %s %s", m_modelName, m_firmwareName, m_macAddress);
        m_httpClient->addHeader( "User-Agent", packHead);
        memset(packHead, 0, sizeof(packHead));
        snprintf(packHead, sizeof(packHead), "%s", dingT);
        m_httpClient->addHeader( "X-EVENT-DATETIME", packHead);
        memset(packHead, 0, sizeof(packHead));
        snprintf(packHead, sizeof(packHead), "%d",file_len);
        m_httpClient->addHeader( "Content-Length", packHead);

        kvs_provision_info_t * cvrConfig = (kvs_provision_info_t*)malloc (sizeof(kvs_provision_info_t));
        if(cvrConfig)
        {
                if (0 == readKVSConfig(cvrConfig))
                {
                        isCVREnabled = atoi(cvrConfig->enable);
                }
                free(cvrConfig);
        }
        memset(packHead, 0, sizeof(packHead));
        if(isCVREnabled)
        {
                snprintf(packHead, sizeof(packHead), "CVR");
        }
        else
        {
                snprintf(packHead, sizeof(packHead), "OFF");
        }
        m_httpClient->addHeader("X-VIDEO-RECORDING", packHead);

	clock_gettime(CLOCK_REALTIME, &startTime);

        while (true) 
	{
		memset(&currTime, 0, sizeof(currTime));
        	clock_gettime(CLOCK_REALTIME, &currTime);

        	//Check for max retry or time limit and break if so
        	if ( (uploadRetryCount >= MAX_RETRY_COUNT) ||((currTime.tv_sec - startTime.tv_sec) > UPLOAD_TIME_INTERVAL) ) 
		{
            		RDK_LOG(RDK_LOG_ERROR,"LOG.RDK.BUTTONMGR", "%s(%d): Max retry count/time exceeded, Retry count %d Time spent %d. currTime.tv_sec %d startTime.tv_sec %d Upload failed!!!\n", __FILE__,__LINE__, uploadRetryCount, (currTime.tv_sec - startTime.tv_sec), currTime.tv_sec, startTime.tv_sec);
            		break;
        	}

		/*Uploading the file */
                curlCode =  m_httpClient->post_binary(m_Instance->m_snapShotUploadURL,(char*)data, &response_code, file_len);
  		clock_gettime(CLOCK_REALTIME, &currTime);
                uploadDuration = (currTime.tv_sec - startTime.tv_sec)*1000 + ( currTime.tv_nsec - startTime.tv_nsec)/1000000;

                if ((response_code >= RDKC_HTTP_RESPONSE_OK) && (response_code < RDKC_HTTP_RESPONSE_REDIRECT_START))
                {
			break;
                }
                else
                {
                        RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.BUTTONMGR","%s(%d): Data post Failed. Response code = %ld : curl code = %d\n", __FILE__, __LINE__, response_code,curlCode);
			uploadRetryCount++;
			
			if(RDKC_SUCCESS != retryAtExpRate()){
               			RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.BUTTONMGR","%s(%d): retries done with current exp wait interval %d\n",__FILE__, __LINE__,waitInterval);
               			break;
           		}
                }
        }
	//log success/failure for telemetry
    	if ((response_code >= RDKC_HTTP_RESPONSE_OK) && (response_code < RDKC_HTTP_RESPONSE_REDIRECT_START)) {
		 if (0 == uploadRetryCount ) {
				RDK_LOG( RDK_LOG_INFO,"LOG.RDK.BUTTONMGR","%s(%d):High resolution thumbnail upload corresponding to ding is successful with header X-EVENT-DATETIME: %s uploadDuration =%ld\n",__FUNCTION__,__LINE__,dingT,uploadDuration);
                
                        } else {
                                RDK_LOG(RDK_LOG_INFO ,"LOG.RDK.BUTTONMGR","%s(%d): High resolution thumbnail upload corresponding to ding is successful after %d retries  with header X-EVENT-DATETIME: %s uploadDuration =%ld\n", __FILE__, __LINE__,uploadRetryCount,dingT,uploadDuration);
                        }
	}else {
            RDK_LOG( RDK_LOG_ERROR,"LOG.RDK.BUTTONMGR","%s(%d): High resolution thumbnail upload corresponding to Ding failed with response code %lu and curl code %d\n",__FUNCTION__,__LINE__, response_code, curlCode);
    	}

        if(NULL != data)
        {
                free(data);
                data = NULL;
        }
        close(file_fd);
 	unlink(SNAPSHOT_FILE);

	if (NULL != m_Instance->m_httpClient) {
                m_Instance->m_httpClient->close();
	}
	return;
}
