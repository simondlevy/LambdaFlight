/**
 * Scaling constants for simulator
 *
 * Copyright (C) 2024 Simon D. Levy
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

#include <clock.hpp>

static const Clock::rate_t PID_UPDATE_RATE = Clock::RATE_100_HZ;

// These constants allow our PID constants to be in the same intervals as in
// the actual vehicle
static const float THRUST_BASE = 48;
static const float THRUST_SCALE = 0.25;
static const float THRUST_MIN = 0;
static const float THRUST_MAX   = 60;
static const float PITCH_ROLL_SCALE = 1e-4;
static const float YAW_SCALE = 4e-5;
