#include <arm_math.h>
#include <MPU9250.h>

#define IMU_VEC_SIZE 3

namespace SensorFusion {

    struct IMUSensor {
        int8_t status;
        float_t acc_x, acc_y, acc_z;
        float_t omg_x, omg_y, omg_z;
        float_t mag_x, mag_y, mag_z;
        float_t b_x, b_y;
        float_t theta_w[3], theta_am[3];
        float_t alpha_w, alpha_am;
        float_t theta_w_new[3], theta_am_new[3];
        float_t theta_final[3];
    };

    int8_t IMUInit();
    void IMUGetData(float_t *out);
}