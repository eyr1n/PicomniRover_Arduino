
// clang-format off
// wireless_control.gatt.h generated from wireless_control.gatt for BTstack
// it needs to be regenerated when the .gatt file is updated. 

// To generate wireless_control.gatt.h:
// /home/rin/btstack/tool/compile_gatt.py wireless_control.gatt wireless_control.gatt.h

// att db format version 1

// binary attribute representation:
// - size in bytes (16), flags(16), handle (16), uuid (16/128), value(...)

#include <stdint.h>

// Reference: https://en.cppreference.com/w/cpp/feature_test
#if __cplusplus >= 200704L
constexpr
#endif
static const uint8_t profile_data[] =
{
    // ATT DB Version
    1,

    // 0x0001 PRIMARY_SERVICE-69321C59-8017-488E-B5E2-B6D30C834BC5
    0x18, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00, 0x28, 0xc5, 0x4b, 0x83, 0x0c, 0xd3, 0xb6, 0xe2, 0xb5, 0x8e, 0x48, 0x17, 0x80, 0x59, 0x1c, 0x32, 0x69, 
    // 0x0002 CHARACTERISTIC-87BC2DC5-2207-408D-99F6-3D35573C4472 - DYNAMIC | WRITE_WITHOUT_RESPONSE | NOTIFY
    0x1b, 0x00, 0x02, 0x00, 0x02, 0x00, 0x03, 0x28, 0x14, 0x03, 0x00, 0x72, 0x44, 0x3c, 0x57, 0x35, 0x3d, 0xf6, 0x99, 0x8d, 0x40, 0x07, 0x22, 0xc5, 0x2d, 0xbc, 0x87, 
    // 0x0003 VALUE CHARACTERISTIC-87BC2DC5-2207-408D-99F6-3D35573C4472 - DYNAMIC | WRITE_WITHOUT_RESPONSE | NOTIFY
    // WRITE_ANYBODY
    0x16, 0x00, 0x04, 0x03, 0x03, 0x00, 0x72, 0x44, 0x3c, 0x57, 0x35, 0x3d, 0xf6, 0x99, 0x8d, 0x40, 0x07, 0x22, 0xc5, 0x2d, 0xbc, 0x87, 
    // 0x0004 CLIENT_CHARACTERISTIC_CONFIGURATION
    // READ_ANYBODY, WRITE_ANYBODY
    0x0a, 0x00, 0x0e, 0x01, 0x04, 0x00, 0x02, 0x29, 0x00, 0x00, 
    // END
    0x00, 0x00, 
}; // total size 53 bytes 


//
// list service handle ranges
//
#define ATT_SERVICE_69321C59_8017_488E_B5E2_B6D30C834BC5_START_HANDLE 0x0001
#define ATT_SERVICE_69321C59_8017_488E_B5E2_B6D30C834BC5_END_HANDLE 0x0004
#define ATT_SERVICE_69321C59_8017_488E_B5E2_B6D30C834BC5_01_START_HANDLE 0x0001
#define ATT_SERVICE_69321C59_8017_488E_B5E2_B6D30C834BC5_01_END_HANDLE 0x0004

//
// list mapping between characteristics and handles
//
#define ATT_CHARACTERISTIC_87BC2DC5_2207_408D_99F6_3D35573C4472_01_VALUE_HANDLE 0x0003
#define ATT_CHARACTERISTIC_87BC2DC5_2207_408D_99F6_3D35573C4472_01_CLIENT_CONFIGURATION_HANDLE 0x0004
