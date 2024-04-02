#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Mount/AP_Mount_config.h>
#include <AP_Relay/AP_Relay_config.h>
#include <GCS_MAVLink/GCS_config.h>

#ifndef AP_CAMERA_ENABLED
#define AP_CAMERA_ENABLED 1
#endif

#ifndef AP_CAMERA_SEND_FOV_STATUS_ENABLED
#define AP_CAMERA_SEND_FOV_STATUS_ENABLED AP_MOUNT_POI_TO_LATLONALT_ENABLED
#endif

#ifndef AP_CAMERA_BACKEND_DEFAULT_ENABLED
#define AP_CAMERA_BACKEND_DEFAULT_ENABLED AP_CAMERA_ENABLED
#endif

#ifndef AP_CAMERA_MAVLINK_ENABLED
#define AP_CAMERA_MAVLINK_ENABLED AP_CAMERA_BACKEND_DEFAULT_ENABLED && HAL_GCS_ENABLED
#endif

#ifndef AP_CAMERA_MAVLINKCAMV2_ENABLED
#define AP_CAMERA_MAVLINKCAMV2_ENABLED AP_CAMERA_BACKEND_DEFAULT_ENABLED && BOARD_FLASH_SIZE > 1024
#endif

#ifndef AP_CAMERA_MOUNT_ENABLED
#define AP_CAMERA_MOUNT_ENABLED AP_CAMERA_BACKEND_DEFAULT_ENABLED && HAL_MOUNT_ENABLED
#endif

#ifndef AP_CAMERA_RELAY_ENABLED
#define AP_CAMERA_RELAY_ENABLED AP_CAMERA_BACKEND_DEFAULT_ENABLED && AP_RELAY_ENABLED
#endif

#ifndef AP_CAMERA_SERVO_ENABLED
#define AP_CAMERA_SERVO_ENABLED AP_CAMERA_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_CAMERA_SOLOGIMBAL_ENABLED
#define AP_CAMERA_SOLOGIMBAL_ENABLED AP_CAMERA_BACKEND_DEFAULT_ENABLED && HAL_SOLO_GIMBAL_ENABLED
#endif

#ifndef AP_CAMERA_SCRIPTING_ENABLED
#define AP_CAMERA_SCRIPTING_ENABLED AP_CAMERA_BACKEND_DEFAULT_ENABLED && AP_SCRIPTING_ENABLED
#endif
