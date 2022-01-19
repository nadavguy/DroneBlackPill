/*
 * AHRS.c
 *
 *  Created on: May 29, 2021
 *      Author: raing
 */

#include "main.h"

struct tAHRSDATA rawARHSData;

float OneOverSqrt(float x)
{
    return 1.0f / (float)sqrt(x);
}

void CalculateAngles(struct tAHRSDATA *inputAHRS)
{
	float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    double AHRSIMU_RAD2DEG = 180 / PI;
    float Roll;        /*!< Roll angle value. This parameter is in units of degrees */
    float Pitch;       /*!< Pitch angle value. This parameter is in units of degrees */
    float Yaw;         /*!< Yaw angle value. This parameter is in units of degrees */
    float Inclination = 3.5; /*!< Inclination in units of degrees */

    q0 = inputAHRS->q0;
    q1 = inputAHRS->q1;
    q2 = inputAHRS->q2;
    q3 = inputAHRS->q3;

    Roll = atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
    Pitch = asin(-2.0f * (q1 * q3 - q0 * q2));
    Yaw = atan2(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);

    /* Calculate degrees and remove inclination */
    Roll *= AHRSIMU_RAD2DEG;
    Pitch *= AHRSIMU_RAD2DEG;
    Yaw = Yaw * AHRSIMU_RAD2DEG - Inclination;

    inputAHRS->Roll = Roll;
    inputAHRS->Pitch = Pitch;
    inputAHRS->Yaw = Yaw;

    /* Check values because of inclination */
    if (Yaw < -180)
    {
        Yaw = 180.0f - (-180.0f - Yaw);
    }
    else if (Yaw > 180)
    {
        Yaw = -180.0f - (180.0f - Yaw);
    }
}

void UpdateAHRS(struct tAHRSDATA *inputAHRS, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
	float sampleRate = 1/20.0;
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float beta = 0.3;

    q0 = inputAHRS->q0;
    q1 = inputAHRS->q1;
    q2 = inputAHRS->q2;
    q3 = inputAHRS->q3;
    sampleRate = inputAHRS->sampleRate;

    /* Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation) */
    if (( mx != 0.0f) || (my != 0.0f) || (mz != 0.0f))
    {
        /* Rate of change of quaternion from gyroscope */
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        /* Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation) */
        if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
        {

            /* Normalise accelerometer measurement */
            recipNorm = OneOverSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            /* Normalise magnetometer measurement */
            recipNorm = OneOverSqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            /* Auxiliary variables to avoid repeated arithmetic */
            _2q0mx = 2.0f * q0 * mx;
            _2q0my = 2.0f * q0 * my;
            _2q0mz = 2.0f * q0 * mz;
            _2q1mx = 2.0f * q1 * mx;
            _2q0 = 2.0f * q0;
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            _2q0q2 = 2.0f * q0 * q2;
            _2q2q3 = 2.0f * q2 * q3;
            q0q0 = q0 * q0;
            q0q1 = q0 * q1;
            q0q2 = q0 * q2;
            q0q3 = q0 * q3;
            q1q1 = q1 * q1;
            q1q2 = q1 * q2;
            q1q3 = q1 * q3;
            q2q2 = q2 * q2;
            q2q3 = q2 * q3;
            q3q3 = q3 * q3;

            /* Reference direction of Earth's magnetic field */
            hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
            hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
            _2bx = sqrt(hx * hx + hy * hy);
            _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            /* Gradient decent algorithm corrective step */
            s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

            /* normalise step magnitude */
            recipNorm = OneOverSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            /* Apply feedback step */
            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;
        }

        /* Integrate rate of change of quaternion to yield quaternion */
        q0 += qDot1 * sampleRate;
        q1 += qDot2 * sampleRate;
        q2 += qDot3 * sampleRate;
        q3 += qDot4 * sampleRate;

        /* Normalise quaternion */
        recipNorm = OneOverSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;

        inputAHRS->q0 = q0;
        inputAHRS->q1 = q1;
        inputAHRS->q2 = q2;
        inputAHRS->q3 = q3;
    }
    else
    {
        /* Update IMU algorithm */
        // UpdateIMU();
    }

    /* Calculate new angles */
    CalculateAngles(inputAHRS);
}

void initAHRS(struct tAHRSDATA *imuAHRS)
{
	imuAHRS->q0 = 1;
	imuAHRS->q1 = 0;
	imuAHRS->q2 = 0;
	imuAHRS->q3 = 0;
	imuAHRS->sampleRate = 1/20.0;
}

void runAHRSCycle(void)
{
//	if (HAL_GetTick() - lastChasisIMUMeasurement >= 50)
//	{
//		int16_t AccData[3], GyroData[3], MagData[3];
//		MPU9250_GetData(chasisIMU, AccData, MagData, GyroData);
//		chasisIMUAHRS.AccData[0] = 9.80665 * AccData[0] / 16384.0;
//		chasisIMUAHRS.AccData[1] = 9.80665 * AccData[1] / 16384.0;
//		chasisIMUAHRS.AccData[2] = 9.80665 * AccData[2] / 16384.0;
//
//		chasisIMUAHRS.GyroData[0] = GyroData[0] / 131.0; // Degrees / Sec
//		chasisIMUAHRS.GyroData[1] = GyroData[1] / 131.0;
//		chasisIMUAHRS.GyroData[2] = GyroData[2] / 131.0;
//
//		chasisIMUAHRS.MagData[0] = MagData[0] * 0.6;
//		chasisIMUAHRS.MagData[1] = MagData[1] * 0.6;
//		chasisIMUAHRS.MagData[2] = MagData[2] * 0.6;
//
//		UpdateAHRS(&chasisIMUAHRS,
//				chasisIMUAHRS.GyroData[0] * AHRSIMU_DEG2RAD, chasisIMUAHRS.GyroData[1] * AHRSIMU_DEG2RAD,
//				chasisIMUAHRS.GyroData[2] * AHRSIMU_DEG2RAD,
//				chasisIMUAHRS.AccData[0], chasisIMUAHRS.AccData[1], chasisIMUAHRS.AccData[2],
//				chasisIMUAHRS.MagData[0], chasisIMUAHRS.MagData[1], chasisIMUAHRS.MagData[2]);
//		lastChasisIMUMeasurement = HAL_GetTick();
//	}
}
