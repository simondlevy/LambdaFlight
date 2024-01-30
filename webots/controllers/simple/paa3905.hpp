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
    auto image = cv::Mat(
            cv::Size(
                wb_camera_get_width(camera),
                wb_camera_get_height(camera)), 
            CV_8UC4);

    image.data = (uint8_t *)wb_camera_get_image(camera);

    cv::imshow("Image", image);

    cv::waitKey(1);
}
