#define CANARD_DSDLC_INTERNAL
#include <uavcan.equipment.camera_gimbal.Mode.h>
#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t uavcan_equipment_camera_gimbal_Mode_encode(struct uavcan_equipment_camera_gimbal_Mode* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, UAVCAN_EQUIPMENT_CAMERA_GIMBAL_MODE_MAX_SIZE);
    _uavcan_equipment_camera_gimbal_Mode_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

/*
  return true if the decode is invalid
 */
bool uavcan_equipment_camera_gimbal_Mode_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_camera_gimbal_Mode* msg) {
    uint32_t bit_ofs = 0;
    _uavcan_equipment_camera_gimbal_Mode_decode(transfer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    );

    const uint32_t byte_len = (bit_ofs+7U)/8U;
#if CANARD_ENABLE_TAO_OPTION
    // if this could be CANFD then the dlc could indicating more bytes than
    // we actually have
    if (!transfer->tao) {
        return byte_len > transfer->payload_len;
    }
#endif
    return byte_len != transfer->payload_len;
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_camera_gimbal_Mode sample_uavcan_equipment_camera_gimbal_Mode_msg(void) {
    struct uavcan_equipment_camera_gimbal_Mode msg;

    msg.command_mode = (uint8_t)random_bitlen_unsigned_val(8);
    return msg;
}
#endif
