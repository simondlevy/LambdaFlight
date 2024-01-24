/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Miniflie.

   Miniflie is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Miniflie is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Miniflie. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <datatypes.h>
#include <ladybug/constrain.h>
#include <ladybug/pid.hpp>

#include <vector>

class Mixer {

    private:

        typedef void (*mixerFun_t)(const demands_t & demands, float motors[]);

        uint8_t m_motorCount;

        mixerFun_t m_fun;

    public:

        Mixer(const uint8_t motorCount, const mixerFun_t fun)
        {
            m_motorCount = motorCount;
            m_fun = fun;
        }

        uint8_t getMotorCount(void)
        {
            return m_motorCount;
        }

        void getMotors(const demands_t & demands, float motors[])
        {
            m_fun(demands, motors);
        }
};
