/**
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

#include <webots/camera.h>

#include<opencv2/opencv.hpp>

#include "oflow.hpp"

static void runCamera(WbDeviceTag &camera)
{
    auto wid = wb_camera_get_width(camera);
    auto hgt = wb_camera_get_height(camera);

    auto image = cv::Mat(cv::Size(wid, hgt), CV_8UC4); 

    image.data = (uint8_t *)wb_camera_get_image(camera);

    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    static cv::Mat downprev;

    cv::Mat downsized;
    cv::resize(gray, downsized, cv::Size(35, 35), cv::INTER_NEAREST);

    if (downprev.data != NULL) {
    }

    downprev = downsized.clone();

    uint8_t * data = downsized.data;

    for (uint8_t k=0; k<35; ++k) {
        data[k*35 + k] = 0;
    }

    cv::Mat upsized;
    cv::resize(downsized, upsized, cv::Size(wid, hgt), cv::INTER_LINEAR);

    cv::imshow("PAA3905", upsized);

    cv::waitKey(1);
}
