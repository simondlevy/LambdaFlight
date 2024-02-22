/**
 * Scaling constants for actual vehicles
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

// Approximate thrust needed when in perfect hover. More weight/older
// battery can use a higher value
static constexpr float THRUST_BASE  = 36000;
static constexpr float THRUST_SCALE = 1000;
static constexpr float THRUST_MIN   = 20000;
static constexpr float THRUST_MAX   = UINT16_MAX;

static constexpr float PITCH_ROLL_SCALE = 1;

static constexpr float YAW_SCALE = 1;

static const auto PID_UPDATE_RATE = Clock::RATE_500_HZ;
