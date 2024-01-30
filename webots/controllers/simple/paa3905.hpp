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

#include <webots/camera.h>

#include<opencv2/opencv.hpp>

static void runCamera(WbDeviceTag &camera)
{
    auto wid = wb_camera_get_width(camera);
    auto hgt = wb_camera_get_height(camera);

    auto image = cv::Mat(cv::Size(wid, hgt), CV_8UC4); 

    image.data = (uint8_t *)wb_camera_get_image(camera);

    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    cv::Mat downsized;
    cv::resize(gray, downsized, cv::Size(35, 35), cv::INTER_LINEAR);

    cv::Mat upsized;
    cv::resize(downsized, upsized, cv::Size(wid, hgt), cv::INTER_NEAREST);

    cv::imshow("PAA3905", upsized);

    cv::waitKey(1);
}
