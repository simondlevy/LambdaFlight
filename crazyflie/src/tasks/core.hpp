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

#include <task.hpp>
#include <tasks/estimator.hpp>
#include <tasks/imu.hpp>

#include <crossplatform.h>
#include <main.hpp>
#include <kalman.hpp>
#include <motors.h>
#include <rateSupervisor.hpp>
#include <safety.hpp>

class CoreTask : public FreeRTOSTask {

    public:

        void setMotors(float m1, float m2, float m3, float m4)
        {
            _uncapped[0] = m1;
            _uncapped[1] = m2;
            _uncapped[2] = m3;
            _uncapped[3] = m4;
        }

        // Called from main program; returns true on success, false on failure
        bool begin(
                Safety * safety,
                EstimatorTask * estimatorTask,
                ImuTask * imuTask,
                const openLoopFun_t openLoopFun,
                const mixFun_t mixFun)
        {
            if (didInit) {
                return true;
            }

            _safety = safety;

            _estimatorTask = estimatorTask;

            _imuTask = imuTask;

            safety->init();

            _openLoopFun = openLoopFun;

            _miniflie.init(mixFun);

            motorsInit();

            FreeRTOSTask::begin(runCoreTask, "core", this, 5);
        
            auto pass = true;

            pass &= _imuTask->test();
            pass &= _estimatorTask->didInit();
            pass &= motorsTest();

            return pass;
        }

        // Called from crtp_commander_openloop
        void resetControllers(void)
        {
            extern bool resetPids;
            resetPids = true;
        }

    private:

        Miniflie _miniflie;

        openLoopFun_t _openLoopFun;

        EstimatorTask * _estimatorTask;

        ImuTask * _imuTask;

        Safety * _safety;

        float _uncapped[4];

        void runMotors(const float motorvals[4]) 
        {
            const uint16_t motorsPwm[4]  = {
                (uint16_t)motorvals[0],
                (uint16_t)motorvals[1],
                (uint16_t)motorvals[2],
                (uint16_t)motorvals[3]
            };

            motorsSetRatios(motorsPwm);
        }

        static void runCoreTask(void* obj)
        {
            ((CoreTask *)obj)->run();
        }

        /* The core loop runs at 1kHz. It is the
         * responsibility of the different functions to run slower by skipping call
         * (ie. returning without modifying the output structure).
         */
        void run(void)
        {
            static RateSupervisor rateSupervisor;

            vTaskSetApplicationTaskTag(0, (TaskHookFunction_t)TASK_CORE_ID_NBR);

            //Wait for the system to be fully started to start core loop
            systemWaitStart();

            consolePrintf("CORE: Wait for sensor calibration...\n");

            // Wait for sensors to be calibrated
            auto lastWakeTime = xTaskGetTickCount();
            while(!_imuTask->areCalibrated()) {
                vTaskDelayUntil(&lastWakeTime, F2T(Clock::RATE_MAIN_LOOP));
            }
            consolePrintf("CORE: Starting loop\n");
            rateSupervisor.init(xTaskGetTickCount(), M2T(1000), 997, 1003, 1);

            for (uint32_t step=1; ; step++) {

                // The IMU should unlock at 1kHz
                _imuTask->waitDataReady();
                sensorData_t sensorData = {};
                _imuTask->acquire(&sensorData);

                // Get state vector linear positions and velocities and
                // angles from estimator
                extern vehicleState_t vehicleState;
                _estimatorTask->getVehicleState(&vehicleState);

                const auto areMotorsAllowedToRun = _safety->areMotorsAllowedToRun();

                static float _motorvals[4];

                if (Clock::rateDoExecute(PID_UPDATE_RATE, step)) {

                    uint32_t timestamp = 0;
                    extern bool inHoverMode;

                    // Get open-loop demands in [-1,+1], as well as timestamp
                    // when they received, and whether hover mode is indicated
                    extern demands_t openLoopDemands;
                    _openLoopFun(openLoopDemands, timestamp, inHoverMode);

                    // Use safety algorithm to modify demands based on sensor data
                    // and open-loop info
                    _safety->update(sensorData, step, timestamp, openLoopDemands);

                    // Run miniflie core algorithm to get uncapped motor spins from open
                    // loop demands via closed-loop control and mixer.   This will
                    // cause setMotors() to be called
                    //_miniflie.step();

                    demands_t demands = {};
                    _miniflie.getDemands(demands);

                    _miniflie.runMixer(demands);

                    // Cancel PID resetting
                    extern bool resetPids;
                    resetPids = false;

                    // Scale motors spins for output
                    scaleMotors(_uncapped, _motorvals);
                }

                if (areMotorsAllowedToRun) {
                    runMotors(_motorvals);
                } else {
                    motorsStop();
                }

                if (!rateSupervisor.validate(xTaskGetTickCount())) {
                    static bool rateWarningDisplayed;
                    if (!rateWarningDisplayed) {
                        consolePrintf("CORE: WARNING: loop rate is off (%lu)\n", 
                                rateSupervisor.getLatestCount());
                        rateWarningDisplayed = true;
                    }
                }

                // motorsCheckDshot();
            }
        }

        static void scaleMotors(const float uncapped[], float motorvals[])
        {
            float highestThrustFound = 0;
            for (uint8_t k=0; k<4; k++) {

                const auto thrust = uncapped[k];

                if (thrust > highestThrustFound) {
                    highestThrustFound = thrust;
                }
            }

            float reduction = 0;
            const float maxAllowedThrust = UINT16_MAX;
            if (highestThrustFound > maxAllowedThrust) {
                reduction = highestThrustFound - maxAllowedThrust;
            }

            for (uint8_t k = 0; k < 4; k++) {
                float thrustCappedUpper = uncapped[k] - reduction;
                motorvals[k] = capMinThrust(thrustCappedUpper);
            }
        }

        static uint16_t capMinThrust(float thrust) 
        {
            return thrust < 0 ? 0 : thrust;
        }
};
