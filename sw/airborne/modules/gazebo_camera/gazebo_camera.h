/*
 * Copyright (C) Sunyou Hwang <S.Hwang-1@tudelft.nl>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/gazebo_camera/gazebo_camera.h"
 * @author Sunyou Hwang <S.Hwang-1@tudelft.nl>
 * Camera module for gazebo simulation
 * Set vehicle pose to the gazebo model and retrieve an image from camera
 */

#ifndef GAZEBO_CAMERA_H
#define GAZEBO_CAMERA_H

extern void gazebo_camera_init(void);
extern void gazebo_camera_periodic(void);

#endif  // GAZEBO_CAMERA_H
