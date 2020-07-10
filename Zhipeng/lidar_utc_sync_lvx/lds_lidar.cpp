//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "lds_lidar.h"

#include <stdio.h>
#include <string.h>

#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <thread>
#include <memory>
#include <mutex>


std::mutex mtx;

// Following variables are added by Zhipeng
bool is_finish_extrinsic_parameter = false;
extern bool is_read_extrinsic_from_xml;
std::condition_variable extrinsic_condition;

// Following variables are added by Zhipeng
LvxFileHandle lvx_file_handler;
std::list<LvxBasePackDetail> point_packet_list;
int lvx_file_save_time = 10;
std::condition_variable point_pack_condition;
#define FRAME_RATE 20
using namespace std::chrono;

// Add by Zhipeng
DeviceItem devices[kMaxLidarCount];
uint8_t connected_lidar_count = 0;


/** Const varible ------------------------------------------------------------------------------- */
/** User add broadcast code here */
static const char* local_broadcast_code_list[] = 
{
    "000000000000001",
};

/** For callback use only */
LdsLidar* g_lidars = nullptr;

/** Lds lidar function ---------------------------------------------------------------------------*/
LdsLidar::LdsLidar() 
{
    auto_connect_mode_ = true;
    whitelist_count_   = 0;
    is_initialized_    = false;

    lidar_count_       = 0;
    memset(broadcast_code_whitelist_, 0, sizeof(broadcast_code_whitelist_));

    memset(lidars_, 0, sizeof(lidars_));
    for (uint32_t i=0; i<kMaxLidarCount; i++) 
    {
        lidars_[i].handle = kMaxLidarCount;
        // Unallocated state
        lidars_[i].connect_state = kConnectStateOff;
    }

    // Add by Zhipeng
    memset(devices, 0, sizeof(devices));
    connected_lidar_count = 0;
}

LdsLidar::~LdsLidar() 
{
}

int LdsLidar::InitLdsLidar(std::vector<std::string>& broadcast_code_strs) 
{
    if (is_initialized_) 
    {
        printf("LiDAR data source is already inited!\n");
        return -1;
    }

    if (!Init()) 
    {
        Uninit();
        printf("Livox-SDK init fail!\n");
        return -1;
    }
    printf("Livox SDK has been initialized.\n");

    LivoxSdkVersion _sdkversion;
    GetLivoxSdkVersion(&_sdkversion);
    printf("Livox SDK version %d.%d.%d\n", _sdkversion.major, _sdkversion.minor, _sdkversion.patch);

    /** Set the callback function receiving broadcast message from Livox LiDAR. */
    SetBroadcastCallback(LdsLidar::OnDeviceBroadcast);
    /** Set the callback function called when device state change,
     * which means connection/disconnection and changing of LiDAR state.*/
    SetDeviceStateUpdateCallback(LdsLidar::OnDeviceChange);

    /** Add commandline input broadcast code */
    for (auto input_str : broadcast_code_strs) 
    {
        LdsLidar::AddBroadcastCodeToWhitelist(input_str.c_str());
    }

    /** Add local broadcast code */
    LdsLidar::AddLocalBroadcastCode();

    if (whitelist_count_) 
    {
        LdsLidar::DisableAutoConnectMode();
        printf("Disable auto connect mode!\n");

        printf("List all broadcast code in whiltelist:\n");
        for (uint32_t i=0; i<whitelist_count_; i++) 
        {
            printf("%s\n", broadcast_code_whitelist_[i]);
        }
    }
    else 
    {
        LdsLidar::EnableAutoConnectMode();
        printf("No broadcast code was added to whitelist, swith to automatic connection mode!\n");
    }

    /** Start the device discovering routine. */
    if (!Start())
    {
        Uninit();
        printf("Livox-SDK init fail!\n");
        return -1;
    }

    /** Add here, only for callback use */
    if(g_lidars == nullptr)
    {
        g_lidars = this;
    }

    WaitForExtrinsicParameter();

    // Add by Zhipeng
    printf("Wait for lidar connected...\n");
    #ifdef WIN32
    Sleep(10000);
    #else
    sleep(10);
    #endif

    if (connected_lidar_count == 0) 
    {
        printf("No device will be connected.\n");
        Uninit();
        return -1;
    }

    /** Init lvx_file_handler */
    printf("Start initialize lvx file.\n");
    if (!lvx_file_handler.InitLvxFile())
    {
        Uninit();
        return -1;
    }

    lvx_file_handler.InitLvxFileHeader();
#if 0
    steady_clock::time_point last_time = steady_clock::now();
    for (int i = 0; i < lvx_file_save_time * FRAME_RATE; ++i)
    {
        std::list<LvxBasePackDetail> point_packet_list_temp;
        {
            std::unique_lock<std::mutex> lock(mtx);
            point_pack_condition.wait_for(lock, milliseconds(kDefaultFrameDurationTime) - (steady_clock::now() - last_time));
            last_time = steady_clock::now();
            point_packet_list_temp.swap(point_packet_list);
        }
        if(point_packet_list_temp.empty()) 
        {
            printf("Point cloud packet is empty.\n");
            break;
        }

        printf("Finish save %d frame to lvx file.\n", i);
        lvx_file_handler.SaveFrameToLvxFile(point_packet_list_temp);
    }
    lvx_file_handler.CloseLvxFile();
#endif
    is_initialized_= true;
    printf("Livox-SDK and lvx file handler init success!\n");

    return 0;
}

void LdsLidar::WaitForExtrinsicParameter() 
{
    std::unique_lock<std::mutex> lock(mtx);
    extrinsic_condition.wait(lock);
}

int LdsLidar::DeInitLdsLidar(void) 
{
    if (!is_initialized_) 
    {
        printf("LiDAR data source is not exit");
        return -1;
    }

    // Add by Zhipeng
    lvx_file_handler.CloseLvxFile();

    for (int i = 0; i < kMaxLidarCount; ++i)
    {
        if (devices[i].device_state == kDeviceStateSampling)
        {
            /** Stop the sampling of Livox LiDAR. */
            LidarStopSampling(devices[i].handle, LdsLidar::OnStopSampleCallback, nullptr);
        }
    }

    Uninit();
    printf("Livox SDK Deinit completely!\n");

    return 0;
}

/** Callback function of stopping sampling. */
void LdsLidar::OnStopSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data)
{
}


//-------------------------------------------------------------------------------------
// OnDeviceBroadcast() and its subfunctions
//-------------------------------------------------------------------------------------
void LdsLidar::OnDeviceBroadcast(const BroadcastDeviceInfo *info) 
{
    if (info == nullptr) 
    {
        return;
    }

    if (info->dev_type == kDeviceTypeHub) 
    {
        printf("In lidar mode, couldn't connect a hub : %s\n", info->broadcast_code);
        return;
    }
    std::unique_lock<std::mutex> lock(mtx);
    if (g_lidars->IsAutoConnectMode())
    {
        printf("In automatic connection mode, will connect %s\n", info->broadcast_code);
    }
    else
    {
        if (!g_lidars->FindInWhitelist(info->broadcast_code)) 
        {
            printf("Not in the whitelist, please add %s to if want to connect!\n",\
                    info->broadcast_code);
            return;
        }
    }

    bool result = false;
    uint8_t handle = 0;
    result = AddLidarToConnect(info->broadcast_code, &handle);
    if (result == kStatusSuccess && handle < kMaxLidarCount)
    {
        // Set the point cloud data for a specific Livox LiDAR.
        SetDataCallback(handle, LdsLidar::GetLidarData, NULL);    // Add by --Zhipeng
        //g_lidars->lidars_[handle].handle = handle;
        //g_lidars->lidars_[handle].connect_state = kConnectStateOff; // OR kDeviceStateDisconnect ??
        //g_lidars->lidar_count_++;
        devices[handle].handle = handle;
        devices[handle].device_state = kDeviceStateDisconnect;
        connected_lidar_count++;
    }
    else
    {
        printf("Add lidar to connect is failed : %d %d \n", result, handle);
    }
}

// Add by Zhipeng, not finished !!!
/** Receiving point cloud data from Livox LiDAR. */
void LdsLidar::GetLidarData(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data)
{
    if(data)
    {
        if (handle < connected_lidar_count && is_finish_extrinsic_parameter) 
        {
            std::unique_lock<std::mutex> lock(mtx);
            LvxBasePackDetail packet;
            packet.device_index = handle;
            lvx_file_handler.BasePointsHandle(data, packet);
            point_packet_list.push_back(packet);
#if 1
            /** Zhipeng: Save point_packet_list to lvx file */
            std::list<LvxBasePackDetail> point_packet_list_temp;
            point_packet_list_temp.swap(point_packet_list);
            //printf("Finish save frame to lvx file.\n");
            lvx_file_handler.SaveFrameToLvxFile(point_packet_list_temp);
#endif
        }
    }
}

//-------------------------------------------------------------------------------------
// SetRmcSyncTime() and its subfunctions
//-------------------------------------------------------------------------------------
void LdsLidar::SetRmcSyncTime(const char* rmc, uint16_t rmc_length) 
{
    printf("Rmc: %s\n",rmc);
    std::unique_lock<std::mutex> lock(mtx);
#if 0
    LidarDevice* p_lidar = nullptr;
    for (uint8_t handle = 0; handle < kMaxLidarCount; handle++)
    {
        p_lidar = &(g_lidars->lidars_[handle]);
        if (p_lidar->connect_state != kConnectStateOff && p_lidar->info.state == kLidarStateNormal) 
        {
            livox_status status = LidarSetRmcSyncTime(handle, rmc, rmc_length, LidarSetRmcSyncTimeCb, g_lidars);
            if (status != kStatusSuccess) 
            {
                printf("Lidar set GPRMC synchronization time error code: %d.\n",status);
            }
        }
    }
#else
    for(uint8_t handle = 0; handle < connected_lidar_count; handle++)
    {
        if(devices[handle].device_state != kDeviceStateDisconnect && devices[handle].info.state == kLidarStateNormal)
        {
            livox_status status = LidarSetRmcSyncTime(handle, rmc, rmc_length, LidarSetRmcSyncTimeCb, g_lidars);
            if (status != kStatusSuccess) 
            {
                printf("Lidar set GPRMC synchronization time error code: %d.\n",status);
            }
        }
    }
#endif
}

/** Callback function of set synchronization time. */
void LdsLidar::LidarSetRmcSyncTimeCb(livox_status status, uint8_t handle,uint8_t response, void* client_data) 
{
    if (handle >= kMaxLidarCount) 
    {
        return;
    }
    printf("OnLidarSetRmcSyncTimeCallback statue %d handle %d response %d. \n", status, handle, response);
}

//-------------------------------------------------------------------------------------
// OnDeviceChange() and its subfunctions
//-------------------------------------------------------------------------------------
/** Callback function of changing of device state. */
void LdsLidar::OnDeviceChange(const DeviceInfo *info, DeviceEvent type) 
{
    if (info == nullptr) 
    {
        return;
    }

    printf("OnDeviceChange broadcast code %s update type %d\n", info->broadcast_code, type);
    uint8_t handle = info->handle;
    if (handle >= kMaxLidarCount) 
    {
        return;
    }

    std::unique_lock<std::mutex> lock(mtx);
    //LidarDevice* p_lidar = &(g_lidars->lidars_[handle]);
    if (type == kEventConnect) 
    {
        //QueryDeviceInformation(handle, DeviceInformationCb, g_lidars);
        //if (p_lidar->connect_state == kConnectStateOff) 
        //{
        //    p_lidar->connect_state = kConnectStateOn;
        //    p_lidar->info = *info;
        //}
        QueryDeviceInformation(handle, DeviceInformationCb, NULL);
        if (devices[handle].device_state == kDeviceStateDisconnect)
        {
            devices[handle].device_state = kDeviceStateConnect;
            devices[handle].info = *info;
        }
        printf("[WARNING] Lidar sn: [%s] Connect!!!\n", info->broadcast_code);
    }
    else if(type == kEventDisconnect)
    {
        //p_lidar->connect_state = kConnectStateOff;
        devices[handle].device_state = kDeviceStateDisconnect;
        printf("[WARNING] Lidar sn: [%s] Disconnect!!!\n", info->broadcast_code);
    }
    else if (type == kEventStateChange)
    {
        //p_lidar->info = *info;
        devices[handle].info = *info;
        printf("[WARNING] Lidar sn: [%s] StateChange!!!\n", info->broadcast_code);
    }

    //if(p_lidar->connect_state == kConnectStateOn)
    if (devices[handle].device_state == kDeviceStateConnect)
    {
        //printf("Device Working State %d\n", p_lidar->info.state);
        printf("Device Working State %d\n", devices[handle].info.state);
        //if (p_lidar->info.state == kLidarStateInit)
        if(devices[handle].info.state == kLidarStateInit)
        {
            //printf("Device State Change Progress %u\n", p_lidar->info.status.progress);
            printf("Device State Change Progress %u\n", devices[handle].info.status.progress);
        } 
        else
        {
            //printf("Device State Error Code 0X%08x\n", p_lidar->info.status.status_code.error_code);
            printf("Device State Error Code 0X%08x\n", devices[handle].info.status.status_code.error_code);
        }
        //printf("Device feature %d\n", p_lidar->info.feature);
        printf("Device feature %d\n", devices[handle].info.feature);
        SetErrorMessageCallback(handle, LdsLidar::LidarErrorStatusCb);

        //if (p_lidar->info.state == kLidarStateNormal)
        if (devices[handle].info.state == kLidarStateNormal)
        {
            // Read extrinsic parameters --Zhipeng
            if (!is_read_extrinsic_from_xml) 
            {
                LidarGetExtrinsicParameter(handle, OnGetLidarExtrinsicParameter, nullptr);
            }
            else
            {
                LidarGetExtrinsicFromXml(handle);
            }
            LidarStartSampling(handle, LdsLidar::OnSampleCallback, nullptr);
            //p_lidar->device_state = kDeviceStateSampling;
            devices[handle].device_state = kDeviceStateSampling;
        }
    }
}

/** Callback function of starting sampling. */
void LdsLidar::OnSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data)
{
    printf("OnSampleCallback statues %d handle %d response %d \n", status, handle, response);
    LidarDevice* p_lidar = &(g_lidars->lidars_[handle]);
    if (status == kStatusSuccess) 
    {
        if (response != 0) 
        {
            //p_lidar->device_state = kDeviceStateConnect;
            devices[handle].device_state = kDeviceStateConnect;
        }
    }
    else if(status == kStatusTimeout) 
    {
        //p_lidar->device_state = kDeviceStateConnect;
        devices[handle].device_state = kDeviceStateConnect;
    }
}

/** Callback function of get LiDARs' extrinsic parameter. */
void LdsLidar::OnGetLidarExtrinsicParameter(livox_status status, uint8_t handle, LidarGetExtrinsicParameterResponse *response, void *data)
{
    if (status == kStatusSuccess)
    {
        if (response != 0)
        {
            printf("OnGetLidarExtrinsicParameter statue %d handle %d response %d \n", status, handle, response->ret_code);
            std::unique_lock<std::mutex> lock(mtx);
            LvxDeviceInfo lidar_info;
            strncpy((char *)lidar_info.lidar_broadcast_code, devices[handle].info.broadcast_code, kBroadcastCodeSize);
            memset(lidar_info.hub_broadcast_code, 0, kBroadcastCodeSize);
            lidar_info.device_index = handle;
            lidar_info.device_type = devices[handle].info.type;
            lidar_info.extrinsic_enable = true;
            lidar_info.pitch = response->pitch;
            lidar_info.roll = response->roll;
            lidar_info.yaw = response->yaw;
            lidar_info.x = static_cast<float>(response->x / 1000.0);
            lidar_info.y = static_cast<float>(response->y / 1000.0);
            lidar_info.z = static_cast<float>(response->z / 1000.0);
            lvx_file_handler.AddDeviceInfo(lidar_info);
            // Zhipeng: I comment the if condition, so we can only add extrinsic parameters for ONE lidar.   --Zhipeng
            //if (lvx_file_handler.GetDeviceInfoListSize() == connected_lidar_count)
            //{
                is_finish_extrinsic_parameter = true;
                extrinsic_condition.notify_one();
            //}
        }
    }
    else if (status == kStatusTimeout) 
    {
        printf("GetLidarExtrinsicParameter timeout! \n");
    }
}

/** Get LiDARs' extrinsic parameter from file named "extrinsic.xml". */
void LdsLidar::LidarGetExtrinsicFromXml(uint8_t handle) 
{
    LvxDeviceInfo lidar_info;
    ParseExtrinsicXml(devices[handle], lidar_info);
    lvx_file_handler.AddDeviceInfo(lidar_info);
    // Zhipeng: I comment the if condition, so we can only add extrinsic parameters for ONE lidar.   --Zhipeng
    //if (lvx_file_handler.GetDeviceInfoListSize() == broadcast_code_list.size())
    //{
        is_finish_extrinsic_parameter = true;
        extrinsic_condition.notify_one();
    //}
}

/** Query the firmware version of Livox LiDAR. */
void LdsLidar::DeviceInformationCb(livox_status status, uint8_t handle, DeviceInformationResponse *ack, void *clent_data)
{
    if (status != kStatusSuccess) 
    {
        printf("Device Query Informations Failed : %d\n", status);
    }
    if(ack)
    {
        printf("firm ver: %d.%d.%d.%d\n",
            ack->firmware_version[0],
            ack->firmware_version[1],
            ack->firmware_version[2],
            ack->firmware_version[3]);
    }
}

/** Callback function of Lidar error message. */
void LdsLidar::LidarErrorStatusCb(livox_status status, uint8_t handle, ErrorMessage *message)
{
    static uint32_t error_message_count = 0;
    if (message != NULL) 
    {
        ++error_message_count;
        if (0 == (error_message_count % 100)) 
        {
            printf("handle: %u\n", handle);
            printf("temp_status : %u\n", message->lidar_error_code.temp_status);
            printf("volt_status : %u\n", message->lidar_error_code.volt_status);
            printf("motor_status : %u\n", message->lidar_error_code.motor_status);
            printf("dirty_warn : %u\n", message->lidar_error_code.dirty_warn);
            printf("firmware_err : %u\n", message->lidar_error_code.firmware_err);
            printf("pps_status : %u\n", message->lidar_error_code.device_status);
            printf("fan_status : %u\n", message->lidar_error_code.fan_status);
            printf("self_heating : %u\n", message->lidar_error_code.self_heating);
            printf("ptp_status : %u\n", message->lidar_error_code.ptp_status);
            printf("time_sync_status : %u\n", message->lidar_error_code.time_sync_status);
            printf("system_status : %u\n", message->lidar_error_code.system_status);
        }
    }
}

//-------------------------------------------------------------------------------------
// Whitelist related functions
//-------------------------------------------------------------------------------------
/** Add broadcast code to whitelist */
int LdsLidar::AddBroadcastCodeToWhitelist(const char* bd_code) 
{
    if(!bd_code || (strlen(bd_code) > kBroadcastCodeSize) || \
        (whitelist_count_ >= kMaxLidarCount)) 
    {
        return -1;
    }

    if(LdsLidar::FindInWhitelist(bd_code)) 
    {
        printf("%s is alrealy exist!\n", bd_code);
        return -1;
    }

    strcpy(broadcast_code_whitelist_[whitelist_count_], bd_code);
    ++whitelist_count_;

    return 0;
}

void LdsLidar::AddLocalBroadcastCode(void)
{
    for (size_t i=0; i<sizeof(local_broadcast_code_list)/sizeof(intptr_t); ++i) 
    {
        std::string invalid_bd = "000000000";
        printf("Local broadcast code : %s\n", local_broadcast_code_list[i]);
        if ((kBroadcastCodeSize == strlen(local_broadcast_code_list[i]) - 1) && \
            (nullptr == strstr(local_broadcast_code_list[i], invalid_bd.c_str()))) 
        {
            LdsLidar::AddBroadcastCodeToWhitelist(local_broadcast_code_list[i]);
        } 
        else 
        {
            printf("Invalid local broadcast code : %s\n", local_broadcast_code_list[i]);
        }
    }
}

bool LdsLidar::FindInWhitelist(const char* bd_code) 
{
    if (!bd_code) 
    {
        return false;
    }

    for (uint32_t i=0; i<whitelist_count_; i++) 
    {
        if (strncmp(bd_code, broadcast_code_whitelist_[i], kBroadcastCodeSize) == 0) 
        {
            return true;
        }
    }

  return false;
}


