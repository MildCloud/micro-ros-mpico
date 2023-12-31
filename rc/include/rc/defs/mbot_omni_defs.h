#include <rc/defs/common_defs.h>

#define OMNI_BASE_RADIUS        0.10
#define OMNI_WHEEL_RADIUS       0.0505

typedef enum mbot_fram_cfg_offset_t{
	WHEEL_CALIBRATION_ADDR = MPU_FINAL_FRAM_ADDR, // have to start at 102 since thats where the MPU stops
} mbot_fram_cfg_offset_t;

typedef enum mbot_fram_cfg_length_t{
	WHEEL_CALIBRATION_LEN = 6 * sizeof(float), // 6 floats
} mbot_fram_cfg_length_t;


