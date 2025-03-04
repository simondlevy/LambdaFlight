/**
 * Copyright (C) 2011-2018 Bitcraze AB, 2025 Simon D. Levy
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

/*
#include <semphr.h>
#include <clock.hpp>
#include <crossplatform.h>
#include <ekf.hpp>
#include <rateSupervisor.hpp>
#include <safety.hpp>
#include <streams.h>
*/

#include <task.hpp>
#include <hal/uart1.h>

class RaspberryPiTask : public FreeRTOSTask {

    public:


        void begin()
        {
            FreeRTOSTask::begin(runRaspberryPiTask, "rpi", this, 4);

            consolePrintf("RASPBERRY PI: taskStart\n");

            uart1Init(115200);
        }

    private:

        static void runRaspberryPiTask(void * obj) 
        {
            ((RaspberryPiTask *)obj)->run();
        }

        void run(void)
        {
            consolePrintf("RASPBERRY PI TASK: running\n");

            systemWaitStart();

            while (true) {

                static uint8_t byte;

                uart1SendData(1, &byte);

                /*
                   if (uart1GetData(1, &byte) > 0) {
                   consolePrintf("%d\n", byte);
                   }
                 */

                byte = (byte + 1) % 256;

                vTaskDelay(1);
            }
        }
};
