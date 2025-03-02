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

class RaspberryPiTask : public FreeRTOSTask {

    public:


        void begin()
        {
            FreeRTOSTask::begin(runRaspberryPiTask, "rpi", this, 4);

            consolePrintf("RASPBERRY PI: taskStart\n");
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

                vTaskDelay(1);
            }
        }
};
