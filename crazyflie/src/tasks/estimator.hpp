/**
 * Copyright (C) 2011-2018 Bitcraze AB, 2024 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <semphr.h>

#include <crossplatform.h>
#include <ekf.hpp>
#include <rateSupervisor.hpp>
#include <safety.hpp>
#include <task.hpp>

#include <streams.h>

class EstimatorTask : public FreeRTOSTask {

    public:

        // Shared with params
        bool didResetEstimation;

        void begin(Safety * safety)
        {
            _safety = safety;

            // Created in the 'empty' state, meaning the semaphore must first be given,
            // that is it will block in the task until released by the stabilizer loop
            _runTaskSemaphore = xSemaphoreCreateBinary();

            _dataMutex = xSemaphoreCreateMutexStatic(&_dataMutexBuffer);

            _measurementsQueue = xQueueCreateStatic(
                    QUEUE_LENGTH, 
                    QUEUE_ITEM_SIZE,
                    measurementsQueueStorage,
                    &measurementsQueueBuffer);

            FreeRTOSTask::begin(runEstimatorTask, "estimator", this, 4);

            consolePrintf("ESTIMATOR: estimatorTaskStart\n");

            initEkf(msec());
        }

        void getVehicleState(vehicleState_t * state)
        {
            // This function is called from the stabilizer loop. It is important that
            // this call returns as quickly as possible. The dataMutex must only be
            // locked short periods by the task.
            xSemaphoreTake(_dataMutex, portMAX_DELAY);

            // Copy the latest state, calculated by the task
            memcpy(state, &_state, sizeof(vehicleState_t));
            xSemaphoreGive(_dataMutex);

            xSemaphoreGive(_runTaskSemaphore);
        }

        void enqueueGyro(const Axis3f * gyro, const bool isInInterrupt)
        {
            measurement_t m = {};
            m.type = MeasurementTypeGyroscope;
            m.data.gyroscope.gyro = *gyro;
            enqueue(&m, isInInterrupt);
        }

        void enqueueAccel(const Axis3f * accel, const bool isInInterrupt)
        {
            measurement_t m = {};
            m.type = MeasurementTypeAcceleration;
            m.data.acceleration.acc = *accel;
            enqueue(&m, isInInterrupt);
        }

        void enqueueFlow(const flowMeasurement_t * flow, const bool isInInterrupt)
        {
            measurement_t m = {};
            m.type = MeasurementTypeFlow;
            m.data.flow = *flow;
            enqueue(&m, isInInterrupt);
        }

        void enqueueRange(const rangeMeasurement_t * range, const bool isInInterrupt)
        {
            measurement_t m = {};
            m.type = MeasurementTypeRange;
            m.data.range = *range;
            enqueue(&m, isInInterrupt);
        }

    private:

        static const uint32_t WARNING_HOLD_BACK_TIME_MS = 2000;

        // this is slower than the IMU update rate of 1000Hz
        static const uint32_t PREDICT_RATE = Clock::RATE_100_HZ; 
        static const uint32_t PREDICTION_UPDATE_INTERVAL_MS = 1000 / PREDICT_RATE;

        static const size_t QUEUE_LENGTH = 20;
        static const auto QUEUE_ITEM_SIZE = sizeof(measurement_t);
        uint8_t measurementsQueueStorage[QUEUE_LENGTH * QUEUE_ITEM_SIZE];
        StaticQueue_t measurementsQueueBuffer;
        xQueueHandle _measurementsQueue;

        RateSupervisor _rateSupervisor;

        // Mutex to protect data that is shared between the task and
        // functions called by the stabilizer loop
        SemaphoreHandle_t _dataMutex;
        StaticSemaphore_t _dataMutexBuffer;

        // Semaphore to signal that we got data from the stabilizer loop to process
        SemaphoreHandle_t _runTaskSemaphore;

        uint32_t _warningBlockTimeMsec;

        Safety * _safety;

        Ekf _ekf;

        // Data used to enable the task and stabilizer loop to run with minimal locking
        // The estimator state produced by the task, copied to the stabilizer when needed.
        vehicleState_t _state;

        static uint32_t msec(void)
        {
            return T2M(xTaskGetTickCount());
        }

        void initEkf(const uint32_t nowMsec)
        {
            _ekf.init(nowMsec);

            ekf_step(EKF_INIT, nowMsec, 0, false, NULL, NULL, NULL, NULL, NULL);
        }

        uint32_t step(const uint32_t nowMsec, uint32_t nextPredictionMsec) 
        {
            xSemaphoreTake(_runTaskSemaphore, portMAX_DELAY);

            if (didResetEstimation) {
                initEkf(nowMsec);
                didResetEstimation = false;
            }

            _ekf.predict(nowMsec, nextPredictionMsec, _safety->isFlying());

            ekf_step(EKF_PREDICT, nowMsec, nextPredictionMsec,
                    _safety->isFlying(), NULL, NULL, NULL, NULL, NULL);

            // Run the system dynamics to predict the state forward.
            if (nowMsec >= nextPredictionMsec) {

                nextPredictionMsec = nowMsec + PREDICTION_UPDATE_INTERVAL_MS;

                if (!_rateSupervisor.validate(nowMsec)) {
                    consolePrintf(
                            "ESTIMATOR: WARNING: Kalman prediction rate off (%lu)\n", 
                            _rateSupervisor.getLatestCount());
                }
            }

            // Sensor measurements can come in sporadically and faster
            // than the stabilizer loop frequency, we therefore consume all
            // measurements since the last loop, rather than accumulating

            // Pull the latest sensors values of interest; discard the rest
            measurement_t measurement = {};
            while (pdTRUE == xQueueReceive(
                        _measurementsQueue, &measurement, 0)) {

                switch (measurement.type) {

                    case MeasurementTypeRange:
                        _ekf.updateWithRange(&measurement.data.range);
                        ekf_step(EKF_UPDATE_WITH_RANGE, 0, 0, false, 
                                NULL,  // gyro
                                NULL,  // accel
                                NULL,  // flow
                                &measurement.data.range,
                                NULL); // vehicleState
                         break;

                    case MeasurementTypeFlow:
                        _ekf.updateWithFlow(&measurement.data.flow);
                        ekf_step(EKF_UPDATE_WITH_FLOW, 0, 0, false, 
                                NULL,  // gyro
                                NULL,  // accel
                                &measurement.data.flow,
                                NULL,  // range
                                NULL); // vehicleState
                        break;

                    case MeasurementTypeGyroscope:
                        _ekf.updateWithGyro(measurement.data.gyroscope.gyro);
                        ekf_step(EKF_UPDATE_WITH_GYRO, 0, 0, false, 
                                &measurement.data.gyroscope.gyro,
                                NULL,  // accel
                                NULL,  // flow
                                NULL,  // range
                                NULL); // vehicleState
                        break;

                    case MeasurementTypeAcceleration:
                        _ekf.updateWithAccel(measurement.data.acceleration.acc);
                        ekf_step(EKF_UPDATE_WITH_ACCEL, 0, 0, false,
                                NULL,  // gyro
                                &measurement.data.acceleration.acc, 
                                NULL,  // flow
                                NULL,  // range
                                NULL); // vehicleState
                        break;

                    default:
                        break;
                }
            }

            auto isStateInBounds = _ekf.finalize();

            auto new_isStateInBounds = ekf_step(
                    EKF_FINALIZE, 0, 0, false, NULL, NULL, NULL, NULL, NULL);
            (void)new_isStateInBounds;

            if (!isStateInBounds) { 

                didResetEstimation = true;

                if (nowMsec > _warningBlockTimeMsec) {
                    _warningBlockTimeMsec = nowMsec + WARNING_HOLD_BACK_TIME_MS;
                    consolePrintf("ESTIMATOR: State out of bounds, resetting\n");
                }
            }

            xSemaphoreTake(_dataMutex, portMAX_DELAY);

            _ekf.getState(_state);

            vehicleState_t new_state = {};
            ekf_step(EKF_GET_STATE, 0, 0, false, NULL, NULL, NULL, NULL, &new_state);

            xSemaphoreGive(_dataMutex);

            return nextPredictionMsec;
        }

        static void runEstimatorTask(void * obj) 
        {
            ((EstimatorTask *)obj)->run();
        }

        void run(void)
        {
            consolePrintf("ESTIMATOR: running\n");

            systemWaitStart();

            auto nextPredictionMsec = msec();

            _rateSupervisor.init(
                    nextPredictionMsec, 
                    1000, 
                    PREDICT_RATE - 1, 
                    PREDICT_RATE + 1, 
                    1); 

            while (true) {

                // would be nice if this had a precision higher than 1ms...
                nextPredictionMsec = step(msec(), nextPredictionMsec);
            }
        }

        void enqueue(
                const measurement_t * measurement, 
                const bool isInInterrupt)
        {
            if (!_measurementsQueue) {
                return;
            }

            if (isInInterrupt) {
                auto xHigherPriorityTaskWoken = pdFALSE;
                xQueueSendFromISR(
                        _measurementsQueue, measurement, &xHigherPriorityTaskWoken);
                if (xHigherPriorityTaskWoken == pdTRUE) {
                    portYIELD();
                }
            } else {
                xQueueSend(_measurementsQueue, measurement, 0);
            }
        }
};
