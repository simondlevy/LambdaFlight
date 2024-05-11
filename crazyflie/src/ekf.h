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

#include <datatypes.h>

void ekf_initialize(const uint32_t nowMsec);

void ekf_predict(const uint32_t nowMec);

void ekf_predict2(const uint32_t nowMec);

void ekf_update_with_range(const float distance);

void ekf_update_with_flow(const float dt, const float dx, const float dy);

void ekf_accumulate_gyro(const uint32_t nowMsec, const axis3_t & gyro);

void ekf_accumulate_accel(const uint32_t nowMsec, const axis3_t & accel);

bool ekf_finalize(void);

void ekf_get_vehicle_state(vehicleState_t & state);
