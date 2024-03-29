/**
 * @file DataHandlerClass.cpp
 * @version 0.0.0
 * @author yzc
 * @brief 这个.cpp文件实现了DataHandlerClass.h中的类。这个文件将以下三个函数通过线程的方式启动，来处理从雷达端获取的数据。第一个函数是读取数据，第二个函数是将数据分类，
 * 第三个函数是交换同步数据。
 */



#include <DataHandlerClass.h>
#include <pthread.h>
#include <algorithm>
#include "pcl_ros/point_cloud.h"
#include "sensor_msgs/PointField.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <cmath>


DataUARTHandler::DataUARTHandler(ros::NodeHandle* nh) : currentBufp(&pingPongBuffers[0]) , nextBufp(&pingPongBuffers[1]) 
{
    nodeHandle = nh;
    DataUARTHandler_pub = nodeHandle->advertise< sensor_msgs::PointCloud2 >("RScan", 100);
    maxAllowedElevationAngleDeg = 90; // Use max angle if none specified
    maxAllowedAzimuthAngleDeg = 90; // Use max angle if none specified
}

/**
 * @brief 设置获取数据的端口
 * @param[in] mySerialPort 端口名称
 */
void DataUARTHandler::setUARTPort(char* mySerialPort)
{
    dataSerialPort = mySerialPort;
}

/**
 * @brief 设置波特率 
 * @param[in] myBaudRate 波特率
 */
void DataUARTHandler::setBaudRate(int myBaudRate)
{
    dataBaudRate = myBaudRate;
}

/**
 * @brief 设置雷达获取数据的最大垂直角度
 * @param[in] myMaxAllowedElevationAngleDeg 最大垂直角度
 */
void DataUARTHandler::setMaxAllowedElevationAngleDeg(int myMaxAllowedElevationAngleDeg)
{
    maxAllowedElevationAngleDeg = myMaxAllowedElevationAngleDeg;
}

/**
 * @brief 设置雷达获取数据的最大水平角度
 * @param[in] myMaxAllowedAzimuthAngleDeg 最大水平角度
 */
void DataUARTHandler::setMaxAllowedAzimuthAngleDeg(int myMaxAllowedAzimuthAngleDeg)
{
    maxAllowedAzimuthAngleDeg = myMaxAllowedAzimuthAngleDeg;
}

/**
 * @brief 读线程所启动的函数。该函数将雷达从串口传输过来的数据读到读内存中，在读的过程中，会对所读的内存进行加锁。等读入的数据读满内存后，便会调用内存交换函数内存
 * 中的内容进行交换。
 */
void *DataUARTHandler::readIncomingData(void)
{
    
    int firstPacketReady = 0;
    uint8_t last8Bytes[8] = {0};


    serial::Serial mySerialObject("", dataBaudRate, serial::Timeout::simpleTimeout(100));
    mySerialObject.setPort(dataSerialPort);
    try
    {
        mySerialObject.open();
    } catch (std::exception &e1) {
        ROS_INFO("DataUARTHandler Read Thread: Failed to open Data serial port with error: %s", e1.what());
        ROS_INFO("DataUARTHandler Read Thread: Waiting 20 seconds before trying again...");
        try
        {
            ros::Duration(20).sleep();
            mySerialObject.open();
        } catch (std::exception &e2) {
            ROS_ERROR("DataUARTHandler Read Thread: Failed second time to open Data serial port, error: %s", e1.what());
            ROS_ERROR("DataUARTHandler Read Thread: Port could not be opened. Port is \"%s\" and baud rate is %d", dataSerialPort, dataBaudRate);
            pthread_exit(NULL);
        }
    }
    
    if(mySerialObject.isOpen())
        ROS_INFO("DataUARTHandler Read Thread: Port is open");
    else
        ROS_ERROR("DataUARTHandler Read Thread: Port could not be opened");
    

    while(!isMagicWord(last8Bytes))
    {

        last8Bytes[0] = last8Bytes[1];
        last8Bytes[1] = last8Bytes[2];
        last8Bytes[2] = last8Bytes[3];
        last8Bytes[3] = last8Bytes[4];
        last8Bytes[4] = last8Bytes[5];
        last8Bytes[5] = last8Bytes[6];
        last8Bytes[6] = last8Bytes[7];
        mySerialObject.read(&last8Bytes[7], 1);
        
    }
    

    pthread_mutex_lock(&nextBufp_mutex);
    
    while(ros::ok())
    {

        last8Bytes[0] = last8Bytes[1];
        last8Bytes[1] = last8Bytes[2];
        last8Bytes[2] = last8Bytes[3];
        last8Bytes[3] = last8Bytes[4];
        last8Bytes[4] = last8Bytes[5];
        last8Bytes[5] = last8Bytes[6];
        last8Bytes[6] = last8Bytes[7];
        mySerialObject.read(&last8Bytes[7], 1);
        
        nextBufp->push_back( last8Bytes[7] );  //push byte onto buffer
        
        //ROS_INFO("DataUARTHandler Read Thread: last8bytes = %02x%02x %02x%02x %02x%02x %02x%02x",  last8Bytes[7], last8Bytes[6], last8Bytes[5], last8Bytes[4], last8Bytes[3], last8Bytes[2], last8Bytes[1], last8Bytes[0]);
        

        if( isMagicWord(last8Bytes) )
        {
            //ROS_INFO("Found magic word");
        

            pthread_mutex_lock(&countSync_mutex);
            pthread_mutex_unlock(&nextBufp_mutex);
            
            /*increment countSync*/
            countSync++;
            
            /*If this is the first packet to be found, increment countSync again since Sort thread is not reading data yet*/
            if(firstPacketReady == 0)
            {
                countSync++;
                firstPacketReady = 1;
            }
            
            /*Signal Swap Thread to run if countSync has reached its max value*/
            if(countSync == COUNT_SYNC_MAX)
            {
                pthread_cond_signal(&countSync_max_cv);
            }
            
            /*Wait for the Swap thread to finish swapping pointers and signal us to continue*/
            pthread_cond_wait(&read_go_cv, &countSync_mutex);
            
            /*Unlock countSync so that Swap Thread can use it*/
            pthread_mutex_unlock(&countSync_mutex);
            pthread_mutex_lock(&nextBufp_mutex);
            
            nextBufp->clear();
            memset(last8Bytes, 0, sizeof(last8Bytes));
              
        }
      
    }
    
    
    mySerialObject.close();
    
    pthread_exit(NULL);
}

/**
 * @brief 判断是否是MagicWord
 */
int DataUARTHandler::isMagicWord(uint8_t last8Bytes[8])
{
    int val = 0, i = 0, j = 0;
    
    for(i = 0; i < 8 ; i++)
    {
    
       if( last8Bytes[i] == magicWord[i])
       {
          j++;
       }
    
    }
    
    if( j == 8)
    {
       val = 1;
    }
    
    return val;  
}

void *DataUARTHandler::syncedBufferSwap(void)
{
    while(ros::ok())
    {
        pthread_mutex_lock(&countSync_mutex);
    
        while(countSync < COUNT_SYNC_MAX)
        {
            pthread_cond_wait(&countSync_max_cv, &countSync_mutex);
            
            pthread_mutex_lock(&currentBufp_mutex);
            pthread_mutex_lock(&nextBufp_mutex);
            
            std::vector<uint8_t>* tempBufp = currentBufp;
        
            this->currentBufp = this->nextBufp;
            
            this->nextBufp = tempBufp;
            
            pthread_mutex_unlock(&currentBufp_mutex);
            pthread_mutex_unlock(&nextBufp_mutex);
            
            countSync = 0;
            
            pthread_cond_signal(&sort_go_cv);
            pthread_cond_signal(&read_go_cv);
            
        }
    
        pthread_mutex_unlock(&countSync_mutex);

    }

    pthread_exit(NULL);
    
}
/**
 * @brief 处理线程所启动的函数。会处理处理内存中的内容，并将其转换为PointCloud2类型数据结构。
 */
void *DataUARTHandler::sortIncomingData( void )
{
    MmwDemo_Output_TLV_Types tlvType = MMWDEMO_OUTPUT_MSG_NULL;
    uint32_t tlvLen = 0;
    uint32_t headerSize;
    unsigned int currentDatap = 0;
    SorterState sorterState = READ_HEADER;
    int i = 0, tlvCount = 0, offset = 0;
    int j = 0;
    float maxElevationAngleRatioSquared;
    float maxAzimuthAngleRatio;
    
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> RScan(new pcl::PointCloud<pcl::PointXYZI>);
    
    //wait for first packet to arrive
    pthread_mutex_lock(&countSync_mutex);
    pthread_cond_wait(&sort_go_cv, &countSync_mutex);
    pthread_mutex_unlock(&countSync_mutex);
    
    pthread_mutex_lock(&currentBufp_mutex);
    
    while(ros::ok())
    {
        
        switch(sorterState)
        {
            
        case READ_HEADER:
            
            //init variables
            mmwData.numObjOut = 0;

            //make sure packet has at least first four fields ( bytes) before we read them (not include magicWord)
            if(currentBufp->size() < 16)
            {
               sorterState = SWAP_BUFFERS;
               break;
            }
            
            //get version (4 bytes)
            memcpy( &mmwData.header.version, &currentBufp->at(currentDatap), sizeof(mmwData.header.version));
            currentDatap += ( sizeof(mmwData.header.version) );

            //get platform (4 bytes)
            memcpy( &mmwData.heakder.platform, &currentBufp->at(currentDatap), sizeof(mmwData.header.platform));
            currentDatap += ( sizeof(mmwData.header.platform) );

            //get timeStamp (4 bytes)
            memcpy( &mmwData.header.timeCpuCycles, &currentBufp->at(currentDatap), sizeof(mmwData.header.timeCpuCycles));
            currentDatap += ( sizeof(mmwData.header.timeCpuCycles) );

            //get totalPacketLen (4 bytes)
            memcpy( &mmwData.header.totalPacketLen, &currentBufp->at(currentDatap), sizeof(mmwData.header.totalPacketLen));
            currentDatap += ( sizeof(mmwData.header.totalPacketLen) );

                //if packet doesn't have correct header size (which is based on platform), throw it away
            //  (does not include magicWord since it was already removed)
            if ((mmwData.header.platform & 0xFFFF) == 0x1443)  // platform is xWR1443)
            {
               headerSize = 7 * 4;  // xWR1443 SDK demo header does not have subFrameNumber field
            }
            else
            {
               headerSize = 8 * 4;  // header includes subFrameNumber field
            }
            if(currentBufp->size() < headerSize)
            {
               sorterState = SWAP_BUFFERS;
               break;
            }
            
            //get frameNumber (4 bytes)
            memcpy( &mmwData.header.frameNumber, &currentBufp->at(currentDatap), sizeof(mmwData.header.frameNumber));
            currentDatap += ( sizeof(mmwData.header.frameNumber) );

            //get subFrameNumber (4 bytes) (not used for XWR1443)
            if ((mmwData.header.platform & 0xFFFF) == 0x1443)  // platform is xWR1443)
            {
                // xWR1443 SDK demo header does not have subFrameNumber field
            }
            else
            {
                memcpy( &mmwData.header.subFrameNumber, &currentBufp->at(currentDatap), sizeof(mmwData.header.subFrameNumber));
                currentDatap += ( sizeof(mmwData.header.subFrameNumber) );
            }

            // skip chirpProcessingMargin (4 bytes)
            // chirpProcessingMargin (4 bytes)
            // trackingProcessingTime;(4 bytes)
            // uartSendingTime;(4 bytes)
            currentDatap += ( 4*sizeof(mmwData.header.frameNumber) );

            //get numDetectedObj (4 bytes) (no numDetectedObj)
            //  memcpy( &mmwData.header.numDetectedObj, &currentBufp->at(currentDatap), sizeof(mmwData.header.numDetectedObj));
            //  currentDatap += ( sizeof(mmwData.header.numDetectedObj) );
            
            //get numTLVs (2 bytes)
            memcpy( &mmwData.header.numTLVs, &currentBufp->at(currentDatap), sizeof(uint16_t));
            currentDatap += ( sizeof(uint16_t) );

            // skip checkSum;(2 bytes)
            currentDatap += ( sizeof(uint16_t) );

            if(mmwData.header.totalPacketLen == currentBufp->size() )
            {
               sorterState = CHECK_TLV_TYPE;
            }
            else sorterState = SWAP_BUFFERS;

            break;
            
         case READ_OBJ_STRUCT:
            
            // CHECK_TLV_TYPE code has already read tlvType and tlvLen

            i = 0;
            offset = 0;

            // FIXME: Removed code for getting numObjs from header
            RScan->header.seq = 0;
            //RScan->header.stamp = (uint32_t) mmwData.header.timeCpuCycles;
            RScan->header.frame_id = "base_radar_link";
            RScan->height = 1;
            RScan->width = mmwData.numObjOut;
            RScan->is_dense = 1;
            RScan->points.resize(RScan->width * RScan->height);
            
            // Calculate ratios for max desired elevation and azimuth angles
            if ((maxAllowedElevationAngleDeg >= 0) && (maxAllowedElevationAngleDeg < 90))
            {
                maxElevationAngleRatioSquared = tan(maxAllowedElevationAngleDeg * M_PI / 180.0);
                maxElevationAngleRatioSquared = maxElevationAngleRatioSquared * maxElevationAngleRatioSquared;
            }
            else
            {
                maxElevationAngleRatioSquared = -1;
            }
            if ((maxAllowedAzimuthAngleDeg >= 0) && (maxAllowedAzimuthAngleDeg < 90))
            {
                maxAzimuthAngleRatio = tan(maxAllowedAzimuthAngleDeg * M_PI / 180.0);
            }
            else
            {
                maxAzimuthAngleRatio = -1;
            }
            //ROS_INFO("maxElevationAngleRatioSquared = %f", maxElevationAngleRatioSquared);
            //ROS_INFO("maxAzimuthAngleRatio = %f", maxAzimuthAngleRatio);
            //ROS_INFO("mmwData.numObjOut before = %d", mmwData.numObjOut);


            // Populate pointcloud
            // TODO: Change point-cloud parsing module

            //get object point unit (Reduce cost of uart data transport)
            memcpy(&mmwData.unitOut, &currentBufp->at(currentDatap), sizeof(MmwDemo_output_message_point_uint));
            currentDatap += sizeof(MmwDemo_output_message_point_uint);

            while( i < mmwData.numObjOut )
            {

                if (((mmwData.header.version >> 24) & 0xFF) < 3)  // SDK version is older than 3.x
                {

                    //get point azimuth
                    memcpy( &mmwData.pointOut.azimuth, &currentBufp->at(currentDatap), sizeof(mmwData.pointOut.azimuth));
                    currentDatap += ( sizeof(mmwData.pointOut.azimuth) );
                
                    //get point doppler
                    memcpy( &mmwData.pointOut.doppler, &currentBufp->at(currentDatap), sizeof(mmwData.pointOut.doppler));
                    currentDatap += ( sizeof(mmwData.pointOut.doppler) );

                    //get point range
                    memcpy( &mmwData.pointOut.range, &currentBufp->at(currentDatap), sizeof(mmwData.pointOut.range));
                    currentDatap += ( sizeof(mmwData.pointOut.range) );

                    //get point snr
                    memcpy( &mmwData.pointOut.snr, &currentBufp->at(currentDatap), sizeof(mmwData.pointOut.snr));
                    currentDatap += ( sizeof(mmwData.pointOut.snr) );

                    // FIXME: Import math lib
                    float tempAzimuth, tempRange, tempSnr;
                    tempAzimuth = (mmwData.unitOut.azimuthUnit * pi/180);
                    tempRange = mmwData.unitOut.rangeUnit * mmwData.pointOut.range;
                    tempSnr = mmwData.unitOut.snrUnit * mmwData.pointOut.snr;

                    mmwData.objOut.x = tempRange*sin(tempAzimuth);
                    mmwData.objOut.y = tempRange*cos(tempAzimuth);
                    mmwData.objOut.z = 0.0f;

                    //convert from Qformat to float(meters)
                    float temp[4];
                
                    temp[0] = (float) mmwData.objOut.x;
                    temp[1] = (float) mmwData.objOut.y;
                    temp[2] = (float) mmwData.objOut.z;
                    //temp[4] = //doppler 
                
                    for(int j = 0; j < 3; j++)
                    {
                        if(temp[j] > 32767)
                            temp[j] -= 65535;
                    
                        temp[j] = temp[j] / pow(2,mmwData.xyzQFormat);
                     }   
                 
                    // Convert intensity to dB
                    temp[3] = 10 * log10(tempSnr + 1);  // intensity
                
                    // Map mmWave sensor coordinates to ROS coordinate system
                    RScan->points[i].x = temp[1];   // ROS standard coordinate system X-axis is forward which is the mmWave sensor Y-axis
                    RScan->points[i].y = -temp[0];  // ROS standard coordinate system Y-axis is left which is the mmWave sensor -(X-axis)
                    RScan->points[i].z = temp[2];   // ROS standard coordinate system Z-axis is up which is the same as mmWave sensor Z-axis
                    RScan->points[i].intensity = temp[3];
                }
                else  // SDK version is 3.x+
                {
                    //get object x-coordinate (meters)
                    memcpy( &mmwData.newObjOut.x, &currentBufp->at(currentDatap), sizeof(mmwData.newObjOut.x));
                    currentDatap += ( sizeof(mmwData.newObjOut.x) );
                
                    //get object y-coordinate (meters)
                    memcpy( &mmwData.newObjOut.y, &currentBufp->at(currentDatap), sizeof(mmwData.newObjOut.y));
                    currentDatap += ( sizeof(mmwData.newObjOut.y) );
                
                    //get object z-coordinate (meters)
                    memcpy( &mmwData.newObjOut.z, &currentBufp->at(currentDatap), sizeof(mmwData.newObjOut.z));
                    currentDatap += ( sizeof(mmwData.newObjOut.z) );
                
                    //get object velocity (m/s)
                    memcpy( &mmwData.newObjOut.velocity, &currentBufp->at(currentDatap), sizeof(mmwData.newObjOut.velocity));
                    currentDatap += ( sizeof(mmwData.newObjOut.velocity) );

                    // Map mmWave sensor coordinates to ROS coordinate system
                    RScan->points[i].x = mmwData.newObjOut.y;   // ROS standard coordinate system X-axis is forward which is the mmWave sensor Y-axis
                    RScan->points[i].y = -mmwData.newObjOut.x;  // ROS standard coordinate system Y-axis is left which is the mmWave sensor -(X-axis)
                    RScan->points[i].z = mmwData.newObjOut.z;   // ROS standard coordinate system Z-axis is up which is the same as mmWave sensor Z-axis

                    // For SDK 3.x, intensity is replaced by snr in sideInfo and is parsed in the READ_SIDE_INFO code
                }
                i++;

            }
            
            sorterState = CHECK_TLV_TYPE;
            
            break;
            
        case READ_SIDE_INFO:

            // Make sure we already received and parsed detected obj list (READ_OBJ_STRUCT)
            if (mmwData.numObjOut > 0)
            {
                for (i = 0; i < mmwData.numObjOut; i++)
                {
                    //get snr (unit is 0.1 steps of dB)
                    memcpy( &mmwData.sideInfo.snr, &currentBufp->at(currentDatap), sizeof(mmwData.sideInfo.snr));
                    currentDatap += ( sizeof(mmwData.sideInfo.snr) );
                
                    //get noise (unit is 0.1 steps of dB)
                    memcpy( &mmwData.sideInfo.noise, &currentBufp->at(currentDatap), sizeof(mmwData.sideInfo.noise));
                    currentDatap += ( sizeof(mmwData.sideInfo.noise) );

                    RScan->points[i].intensity = (float) mmwData.sideInfo.snr / 10.0;   // Use snr for "intensity" field (divide by 10 since unit of snr is 0.1dB)
                }
            }
            else  // else just skip side info section if we have not already received and parsed detected obj list
            {
              i = 0;
            
              while (i++ < tlvLen - 1)
              {
                     //ROS_INFO("DataUARTHandler Sort Thread : Parsing Side Info i=%d and tlvLen = %u", i, tlvLen);
              }
            
              currentDatap += tlvLen;
            }

            sorterState = CHECK_TLV_TYPE;
            
            break;
        
        case READ_LOG_MAG_RANGE:
            {
        
            
              sorterState = CHECK_TLV_TYPE;
            }
            
            break;
            
        case READ_NOISE:
            {
        
              i = 0;
            
              while (i++ < tlvLen - 1)
              {
                     //ROS_INFO("DataUARTHandler Sort Thread : Parsing Noise Profile i=%d and tlvLen = %u", i, tlvLen);
              }
            
              currentDatap += tlvLen;
            
              sorterState = CHECK_TLV_TYPE;
            }
           
            break;
            
        case READ_AZIMUTH:
            {
        
              i = 0;
            
              while (i++ < tlvLen - 1)
              {
                     //ROS_INFO("DataUARTHandler Sort Thread : Parsing Azimuth Profile i=%d and tlvLen = %u", i, tlvLen);
              }
            
              currentDatap += tlvLen;
            
              sorterState = CHECK_TLV_TYPE;
            }
            
            break;
            
        case READ_DOPPLER:
            {
        
              i = 0;
            
              while (i++ < tlvLen - 1)
              {
                     //ROS_INFO("DataUARTHandler Sort Thread : Parsing Doppler Profile i=%d and tlvLen = %u", i, tlvLen);
              }
            
              currentDatap += tlvLen;
            
              sorterState = CHECK_TLV_TYPE;
            }
            
            break;
            
        case READ_STATS:
            {
        
              i = 0;
            
              while (i++ < tlvLen - 1)
              {
                     //ROS_INFO("DataUARTHandler Sort Thread : Parsing Stats Profile i=%d and tlvLen = %u", i, tlvLen);
              }
            
              currentDatap += tlvLen;
            
              sorterState = CHECK_TLV_TYPE;
            }
            
            break;
        
        case CHECK_TLV_TYPE:
        
            //ROS_INFO("DataUARTHandler Sort Thread : tlvCount = %d, numTLV = %d", tlvCount, mmwData.header.numTLVs);
        
            if(tlvCount++ >= mmwData.header.numTLVs)  // Done parsing all received TLV sections
            {
                // Publish detected object pointcloud
                if (mmwData.numObjOut > 0)
                {
                    j = 0;
                    for (i = 0; i < mmwData.numObjOut; i++)
                    {
                        // Keep point if elevation and azimuth angles are less than specified max values
                        // (NOTE: The following calculations are done using ROS standard coordinate system axis definitions where X is forward and Y is left)
                        if (((maxElevationAngleRatioSquared == -1) ||
                             (((RScan->points[i].z * RScan->points[i].z) / (RScan->points[i].x * RScan->points[i].x +
                                                                            RScan->points[i].y * RScan->points[i].y)
                              ) < maxElevationAngleRatioSquared)
                            ) &&
                            ((maxAzimuthAngleRatio == -1) || (fabs(RScan->points[i].y / RScan->points[i].x) < maxAzimuthAngleRatio)) &&
                                    (RScan->points[i].x != 0)
                           )
                        {
                            //ROS_INFO("Kept point");
                            // copy: points[i] => points[j]
                            memcpy( &RScan->points[j], &RScan->points[i], sizeof(RScan->points[i]));
                            j++;
                        }

                        // Otherwise, remove the point
                        else
                        {
                            //ROS_INFO("Removed point");
                        }
                    }
                    mmwData.numObjOut = j;  // update number of objects as some points may have been removed

                    // Resize point cloud since some points may have been removed
                    RScan->width = mmwData.numObjOut;
                    RScan->points.resize(RScan->width * RScan->height);
                    
                    //ROS_INFO("mmwData.numObjOut after = %d", mmwData.numObjOut);
                    //ROS_INFO("DataUARTHandler Sort Thread: number of obj = %d", mmwData.numObjOut );
                    
                    DataUARTHandler_pub.publish(RScan);
                }

                //ROS_INFO("DataUARTHandler Sort Thread : CHECK_TLV_TYPE state says tlvCount max was reached, going to switch buffer state");
                sorterState = SWAP_BUFFERS;
            }

            else  // More TLV sections to parse
            {
               //get tlvType (32 bits) & remove from queue
                memcpy( &tlvType, &currentBufp->at(currentDatap), sizeof(tlvType));
                currentDatap += ( sizeof(tlvType) );
                
                //ROS_INFO("DataUARTHandler Sort Thread : sizeof(tlvType) = %d", sizeof(tlvType));
            
                //get tlvLen (32 bits) & remove from queue
                memcpy( &tlvLen, &currentBufp->at(currentDatap), sizeof(tlvLen));
                currentDatap += ( sizeof(tlvLen) );
                
                //ROS_INFO("DataUARTHandler Sort Thread : sizeof(tlvLen) = %d", sizeof(tlvLen));
                
                //ROS_INFO("DataUARTHandler Sort Thread : tlvType = %d, tlvLen = %d", (int) tlvType, tlvLen);
            
                switch(tlvType)
                {
                case MMWDEMO_OUTPUT_MSG_NULL:
                
                    break;
                // FIXME: Modified tlv_type for indoor_false_det bin
                case MMWDEMO_OUTPUT_MSG_POINT_CLOUD:
                    //ROS_INFO("DataUARTHandler Sort Thread : Object TLV");
                    mmwData.numObjOut = (tlvLen - sizeof(MmwDemo_output_message_tl) - sizeof(MmwDemo_output_message_point_uint)) /
                            sizeof(MmwDemo_output_message_UARTpoint);
                    sorterState = READ_OBJ_STRUCT;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_RANGE_PROFILE:
                    //ROS_INFO("DataUARTHandler Sort Thread : Range TLV");
                    sorterState = READ_LOG_MAG_RANGE;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_NOISE_PROFILE:
                    //ROS_INFO("DataUARTHandler Sort Thread : Noise TLV");
                    sorterState = READ_NOISE;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_AZIMUTH_STATIC_HEAT_MAP:
                    //ROS_INFO("DataUARTHandler Sort Thread : Azimuth Heat TLV");
                    sorterState = READ_AZIMUTH;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP:
                    //ROS_INFO("DataUARTHandler Sort Thread : R/D Heat TLV");
                    sorterState = READ_DOPPLER;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_STATS:
                    //ROS_INFO("DataUARTHandler Sort Thread : Stats TLV");
                    sorterState = READ_STATS;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO:
                    //ROS_INFO("DataUARTHandler Sort Thread : Side info TLV");
                    sorterState = READ_SIDE_INFO;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_MAX:
                    //ROS_INFO("DataUARTHandler Sort Thread : Header TLV");
                    sorterState = READ_HEADER;
                    break;
                
                default:
                    // Skip target info of Indoor_false_det
                    if(tlvLen != 0) {
                        currentDatap += tlvLen;
                    }
                    sorterState = CHECK_TLV_TYPE;

                    break;
                }
            }
            
        break;
            
       case SWAP_BUFFERS:
       
            pthread_mutex_lock(&countSync_mutex);
            pthread_mutex_unlock(&currentBufp_mutex);
                            
            countSync++;
                
            if(countSync == COUNT_SYNC_MAX)
            {
                pthread_cond_signal(&countSync_max_cv);
            }
                
            pthread_cond_wait(&sort_go_cv, &countSync_mutex);
                
            pthread_mutex_unlock(&countSync_mutex);
            pthread_mutex_lock(&currentBufp_mutex);
                
            currentDatap = 0;
            tlvCount = 0;
                
            sorterState = READ_HEADER;
            
            break;
                
            
        default: break;
        }
    }
    
    
    pthread_exit(NULL);
}

/**
 * @brief 该系统利用了多线程双缓存机制来与雷达板的数据进行串口通讯。双缓存的基本原理是读线程先清空一部分缓存，然后将数据读到这部分缓存地址中，与此同时，处理线程
 * 也同时处理处理缓存中的数据，当读缓存数据读满，并且处理缓存中的数据处理完毕以后，利用交换线程将处理缓存与读缓存中的内容进行交换，再运行读线程和处理线程。该函数
 * 首先创建了三个线程，分别用来读取数据、处理数据以及交换缓存区的内容。其次创建了互斥锁以及条件变量，防止三个线程对在读缓存以及在处理缓存中的数据同时进行操作。最
 * 后等程序停止后销毁了互斥锁以及条件变量。
 */
void DataUARTHandler::start(void)
{
    
    pthread_t uartThread, sorterThread, swapThread;
    
    int  iret1, iret2, iret3;
    
    pthread_mutex_init(&countSync_mutex, NULL);
    pthread_mutex_init(&nextBufp_mutex, NULL);
    pthread_mutex_init(&currentBufp_mutex, NULL);
    pthread_cond_init(&countSync_max_cv, NULL);
    pthread_cond_init(&read_go_cv, NULL);
    pthread_cond_init(&sort_go_cv, NULL);
    
    countSync = 0;

    iret1 = pthread_create( &uartThread, NULL, this->readIncomingData_helper, this);
    if(iret1)
    {
     ROS_INFO("Error - pthread_create() return code: %d\n",iret1);
     ros::shutdown();
    }
    
    iret2 = pthread_create( &sorterThread, NULL, this->sortIncomingData_helper, this);
    if(iret2)
    {
        ROS_INFO("Error - pthread_create() return code: %d\n",iret1);
        ros::shutdown();
    }
    
    iret3 = pthread_create( &swapThread, NULL, this->syncedBufferSwap_helper, this);
    if(iret3)
    {
        ROS_INFO("Error - pthread_create() return code: %d\n",iret1);
        ros::shutdown();
    }
    
    ros::spin();

    pthread_join(iret1, NULL);
    ROS_INFO("DataUARTHandler Read Thread joined");
    pthread_join(iret2, NULL);
    ROS_INFO("DataUARTHandler Sort Thread joined");
    pthread_join(iret3, NULL);
    ROS_INFO("DataUARTHandler Swap Thread joined");
    
    pthread_mutex_destroy(&countSync_mutex);
    pthread_mutex_destroy(&nextBufp_mutex);
    pthread_mutex_destroy(&currentBufp_mutex);
    pthread_cond_destroy(&countSync_max_cv);
    pthread_cond_destroy(&read_go_cv);
    pthread_cond_destroy(&sort_go_cv);
    
    
}

void* DataUARTHandler::readIncomingData_helper(void *context)
{  
    return (static_cast<DataUARTHandler*>(context)->readIncomingData());
}

void* DataUARTHandler::sortIncomingData_helper(void *context)
{  
    return (static_cast<DataUARTHandler*>(context)->sortIncomingData());
}

void* DataUARTHandler::syncedBufferSwap_helper(void *context)
{  
    return (static_cast<DataUARTHandler*>(context)->syncedBufferSwap());
}
