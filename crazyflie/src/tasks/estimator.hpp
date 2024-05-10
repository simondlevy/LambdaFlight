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

#include <clock.hpp>
#include <crossplatform.h>
#include <ekf.h>
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

        void enqueueRange(const int16_t distance, const bool isInInterrupt)
        {
            measurement_t m = {};
            m.type = MeasurementTypeRange;
            m.data.rangefinder_distance = distance;
            enqueue(&m, isInInterrupt);
        }

    private:

        // this is slower than the IMU update rate of 1000Hz
        static const uint32_t PREDICT_RATE = Clock::RATE_100_HZ; 
        static const uint32_t PREDICTION_INTERVAL_MSEC = 1000 / PREDICT_RATE;

        // The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
        static constexpr float MAX_COVARIANCE = 100;
        static constexpr float MIN_COVARIANCE = 1e-6;


        static const uint32_t WARNING_HOLD_BACK_TIME_MS = 2000;

        static const size_t QUEUE_LENGTH = 20;
        static const auto QUEUE_ITEM_SIZE = sizeof(measurement_t);
        uint8_t measurementsQueueStorage[QUEUE_LENGTH * QUEUE_ITEM_SIZE];
        StaticQueue_t measurementsQueueBuffer;
        xQueueHandle _measurementsQueue;

        RateSupervisor _rateSupervisor;

        TinyEkf _ekf;

        // Mutex to protect data that is shared between the task and
        // functions called by the stabilizer loop
        SemaphoreHandle_t _dataMutex;
        StaticSemaphore_t _dataMutexBuffer;

        // Semaphore to signal that we got data from the stabilizer loop to process
        SemaphoreHandle_t _runTaskSemaphore;

        uint32_t _warningBlockTimeMsec;

        Safety * _safety;

        // Data used to enable the task and stabilizer loop to run with minimal locking
        // The estimator state produced by the task, copied to the stabilizer when needed.
        vehicleState_t _state;

        static uint32_t msec(void)
        {
            return T2M(xTaskGetTickCount());
        }

        void initEkf(const uint32_t nowMsec)
        {
            _ekf.initialize(
                    nowMsec, 
                    PREDICTION_INTERVAL_MSEC,
                    MIN_COVARIANCE,
                    MAX_COVARIANCE);
        }        

        uint32_t step(const uint32_t nowMsec, uint32_t nextPredictionMsec) 
        {
            xSemaphoreTake(_runTaskSemaphore, portMAX_DELAY);

            if (didResetEstimation) {
                initEkf(nowMsec);
                didResetEstimation = false;
            }

            _ekf.predict(nowMsec);

            // Run the system dynamics to predict the state forward.
            if (nowMsec >= nextPredictionMsec) {

                nextPredictionMsec = nowMsec + PREDICTION_INTERVAL_MSEC;

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

                if (measurement.type == MeasurementTypeRange) {

                    float h[7] = {};
                    float error = 0;
                    float noise = 0;

                    if (shouldUpdateWithRange(
                                _ekf.getState(),
                                measurement.data.rangefinder_distance, 
                                h, 
                                error, 
                                noise)) {

                        _ekf.update(h, error, noise);

                    }
                }

                else if (measurement.type == MeasurementTypeFlow) {

                    float hx[7] = {};
                    float errx = 0;
                    float hy[7] = {};
                    float erry = 0;
                    float stdev = 0;

                    getFlowUpdates(
                            _ekf.getState(),
                            measurement.data.flow.dt, 
                            measurement.data.flow.dpixelx,
                            measurement.data.flow.dpixely,
                            hx, errx, hy, erry, stdev);

                    _ekf.update(hx, errx, stdev);

                    _ekf.update(hy, erry, stdev);
                }

                else if (measurement.type == MeasurementTypeGyroscope ) {
                    axis3_t gyro = {};
                    memcpy(&gyro, &measurement.data.gyroscope.gyro, sizeof(gyro));
                    accumulateGyro(nowMsec, gyro);
                }

                else if (measurement.type == MeasurementTypeAcceleration) {
                    axis3_t accel = {};
                    memcpy(&accel, &measurement.data.acceleration.acc, 
                            sizeof(accel));
                    accumulateAccel(nowMsec, accel);
                }
            }

            _ekf.finalize(nowMsec); 

            if (!isStateWithinBounds(_ekf.getState())) { 

                didResetEstimation = true;

                if (nowMsec > _warningBlockTimeMsec) {
                    _warningBlockTimeMsec = nowMsec + WARNING_HOLD_BACK_TIME_MS;
                    consolePrintf("ESTIMATOR: State out of bounds, resetting\n");
                }
            }

            xSemaphoreTake(_dataMutex, portMAX_DELAY);

            ekf_getVehicleState(_ekf.getState(), _state);

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
