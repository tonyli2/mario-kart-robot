#include <config.h>

namespace SensorFusion {
    
    TwoWire Wire2 = TwoWire(PB11, PB10);
    MPU9250 IMU(Wire2, 0x68);

    float_t dummy[] = {0.0f, 0.0f, 0.0f};       // Dummy vector
    // Initializing sensor fusion
    IMUSensor sensor = {
        .status     = 0,
        .acc_x      = 0.0f,
        .acc_y      = 0.0f,
        .acc_z      = 0.0f,
        .omg_x      = 0.0f,
        .omg_y      = 0.0f,
        .omg_z      = 0.0f,
        .mag_x      = 0.0f,
        .mag_y      = 0.0f,
        .mag_z      = 0.0f,
        .b_x        = 0.0f,
        .b_y        = 0.0f,
        .alpha_w    = 0.0f,
        .alpha_am   = 0.0f
    };

    /**
     * @brief Initializing MPU9250 IMU
     * 
     * @return int8_t IMU initialization status
     */
    int8_t IMUInit() {

        attachInterrupt(COLLISION_PIN, Hivemind::collision, RISING);

        // Initializing all vectors with dummy vector
        memcpy(sensor.theta_w, dummy, sizeof(dummy));
        memcpy(sensor.theta_am, dummy, sizeof(dummy));
        memcpy(sensor.theta_w_new, dummy, sizeof(dummy));
        memcpy(sensor.theta_am_new, dummy, sizeof(dummy));
        memcpy(sensor.theta_final, dummy, sizeof(dummy));

        sensor.status = IMU.begin();
        // setting the accelerometer full scale range to +/-8G 
        IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
        // setting the gyroscope full scale range to +/-500 deg/s
        IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
        // setting DLPF bandwidth to 20 Hz
        IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
        // setting SRD to 19 for a 50 Hz update rate
        IMU.setSrd(19);
        
        return sensor.status;
    }

    /**
     * @brief Interpolating vehicle attitude using IMU sensor fusion
     * @return Vehicle attitude vector in order of roll, pitch, yaw with unit degree
     * 
     * @note Complementary Filter algorithem is used for sensor fusion
     * @cite https://ahrs.readthedocs.io/en/latest/filters/complementary.html
     */
    float_t * IMUGetData() {
        IMU.readSensor();

        // Extract IMU data
        sensor.acc_x = IMU.getAccelX_mss();
        sensor.acc_y = IMU.getAccelY_mss();
        sensor.acc_z = IMU.getAccelZ_mss();
        sensor.omg_x = IMU.getGyroX_rads();
        sensor.omg_y = IMU.getGyroY_rads();
        sensor.omg_z = IMU.getGyroZ_rads();
        sensor.mag_x = IMU.getMagX_uT();
        sensor.mag_y = IMU.getMagY_uT();
        sensor.mag_z = IMU.getMagZ_uT();

        // Compute attitude vector using accelerometer and magnetometer
        sensor.theta_am[0] = atan2(sensor.acc_x, sqrt(sensor.acc_y * sensor.acc_y + sensor.acc_z * sensor.acc_z));
        sensor.theta_am[1] = -atan2(sensor.acc_y, sqrt(sensor.acc_x * sensor.acc_x + sensor.acc_z * sensor.acc_z));

        sensor.b_x = sensor.mag_x * cos(sensor.theta_am[0]) + sensor.mag_y * sin(sensor.theta_am[0]) * sin(sensor.theta_am[1]) + sensor.mag_z * sin(sensor.theta_am[0]) * cos(sensor.theta_am[1]);
        sensor.b_y = sensor.mag_y * cos(sensor.theta_am[1]) - sensor.mag_z * sin(sensor.theta_am[1]);
        
        sensor.theta_am[2] = atan2(-sensor.b_y, sensor.b_x);

        // Compute attitude vector using gyroscope (only yaw angle)
        sensor.theta_w[0] = sensor.theta_am[0];     // Roll
        sensor.theta_w[1] = sensor.theta_am[1];     // Pitch
        sensor.theta_w[2] = IMU.getYawAngle_rad();  //Yaw

        // Fusing two attitude vectors into finalized attitude vector in unit degree
        sensor.alpha_w = 0.95f;
        sensor.alpha_am = 1.0f - sensor.alpha_w;
        arm_scale_f32(sensor.theta_w, sensor.alpha_w, sensor.theta_w_new, IMU_VEC_SIZE);
        arm_scale_f32(sensor.theta_am, sensor.alpha_am, sensor.theta_am_new, IMU_VEC_SIZE);
        arm_add_f32(sensor.theta_w_new, sensor.theta_am_new, sensor.theta_final, IMU_VEC_SIZE);
        arm_scale_f32(sensor.theta_final, 180 / PI, sensor.theta_final, IMU_VEC_SIZE);

        // Check if acceleration drops due to collision
        stopCar();

        return sensor.theta_final;
    }
    

    bool isGoingUpRamp(float_t *pitchArray, uint8_t pitchCounter, uint8_t sizeOfArray) {

        const float_t angleThreshold = 9.0f;

        if(pitchCounter >= sizeOfArray){
            pitchCounter = 0;
        }

        pitchArray[pitchCounter] = IMUGetData()[1];

        for(int i = 0; i < sizeOfArray; i++){
            if(pitchArray[i] < angleThreshold){
                return false;
            }
        }

        return true;
    }

    void resetYaw() {
        IMU.resetYawAngle();
    }

    void stopCar() {
        if(sensor.acc_x < -0.9f) {
            digitalWrite(COLLISION_PIN, HIGH);
        }
    }
}