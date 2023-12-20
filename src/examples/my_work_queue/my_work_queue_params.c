///****************************************************************************
// *
// *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions
// * are met:
// *
// * 1. Redistributions of source code must retain the above copyright
// *    notice, this list of conditions and the following disclaimer.
// * 2. Redistributions in binary form must reproduce the above copyright
// *    notice, this list of conditions and the following disclaimer in
// *    the documentation and/or other materials provided with the
// *    distribution.
// * 3. Neither the name PX4 nor the names of its contributors may be
// *    used to endorse or promote products derived from this software
// *    without specific prior written permission.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
// * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// * POSSIBILITY OF SUCH DAMAGE.
// *
// ****************************************************************************/
//
///**
// * @file my_work_queue_params.c
// * @brief 定义一些本模块需要的参数
// * @author Gray <grayme12345@gmail.com>
// * @time 2023/12/10.
// */
//
///**
// * 姿态环PID控制器的比例系数
// *
// * 该参数仅仅是模拟，并不具备实际的功能。
// *
// * @unit norm
// * @min 0
// * @max 10
// * @decimal 2
// * @increment 0.01
// * @group My Work Queue
// */
//PARAM_DEFINE_FLOAT(MY_ATT_P, 0.00f);
//
///**
// * 姿态环PID控制器的积分系数
// *
// * 该参数仅仅是模拟，并不具备实际的功能。
// *
// * @unit norm
// * @min 0
// * @max 10
// * @decimal 2
// * @increment 0.01
// * @group My Work Queue
// */
//PARAM_DEFINE_FLOAT(MY_ATT_I, 0.00f);
//
///**
// * 姿态环PID控制器的微分系数
// *
// * 该参数仅仅是模拟，并不具备实际的功能。
// *
// * @unit norm
// * @min 0
// * @max 10
// * @decimal 2
// * @increment 0.01
// * @group My Work Queue
// */
//PARAM_DEFINE_FLOAT(MY_ATT_D, 0.00f);
//
///**
// * 姿态环PID控制器的比例误差限幅
// *
// * 该参数仅仅是模拟，并不具备实际的功能。
// *
// * @unit norm
// * @min 0
// * @max 10
// * @decimal 2
// * @increment 0.01
// * @group My Work Queue
// */
//PARAM_DEFINE_FLOAT(MY_ATT_P_LIM, 10.00f);
//
///**
// * 姿态环PID控制器的比例输出限幅
// *
// * 该参数仅仅是模拟，并不具备实际的功能。
// *
// * @unit norm
// * @min 0
// * @max 10
// * @decimal 2
// * @increment 0.01
// * @group My Work Queue
// */
//PARAM_DEFINE_FLOAT(MY_ATT_P_OUT_LIM, 10.00f);
//
///**
// * 姿态环PID控制器的积分误差限幅
// *
// * 该参数仅仅是模拟，并不具备实际的功能。
// *
// * @unit norm
// * @min 0
// * @max 10
// * @decimal 2
// * @increment 0.01
// * @group My Work Queue
// */
//PARAM_DEFINE_FLOAT(MY_ATT_I_E_LIM, 10.00f);
//
///**
// * 姿态环PID控制器的积分限幅
// *
// * 该参数仅仅是模拟，并不具备实际的功能。
// *
// * @unit norm
// * @min 0
// * @max 10
// * @decimal 2
// * @increment 0.01
// * @group My Work Queue
// */
//PARAM_DEFINE_FLOAT(MY_ATT_I_LIM, 10.00f);
//
///**
// * 姿态环PID控制器的积分输出限幅
// *
// * 该参数仅仅是模拟，并不具备实际的功能。
// *
// * @unit norm
// * @min 0
// * @max 10
// * @decimal 2
// * @increment 0.01
// * @group My Work Queue
// */
//PARAM_DEFINE_FLOAT(MY_ATT_I_OUT_LIM, 10.00f);
//
///**
// * 姿态环PID控制器的微分限幅
// *
// * 该参数仅仅是模拟，并不具备实际的功能。
// *
// * @unit norm
// * @min 0
// * @max 10
// * @decimal 2
// * @increment 0.01
// * @group My Work Queue
// */
//PARAM_DEFINE_FLOAT(MY_ATT_D_LIM, 10.00f);
//
///**
// * 姿态环PID控制器的微分输出限幅
// *
// * 该参数仅仅是模拟，并不具备实际的功能。
// *
// * @unit norm
// * @min 0
// * @max 10
// * @decimal 2
// * @increment 0.01
// * @group My Work Queue
// */
//PARAM_DEFINE_FLOAT(MY_ATT_D_OUT_LIM, 10.00f);
//
///**
// * 姿态环PID控制器的输出限幅
// *
// * 该参数仅仅是模拟，并不具备实际的功能。
// *
// * @unit norm
// * @min 0
// * @max 10
// * @decimal 2
// * @increment 0.01
// * @group My Work Queue
// */
//PARAM_DEFINE_FLOAT(MY_ATT_OUT_LIM, 10.00f);
//
///**
// * 姿态环PID控制器的输出增量限幅
// *
// * 该参数仅仅是模拟，并不具备实际的功能。
// *
// * @unit norm
// * @min 0
// * @max 10
// * @decimal 2
// * @increment 0.01
// * @group My Work Queue
// */
//PARAM_DEFINE_FLOAT(MY_ATT_OUT_D_LIM, 10.00f);
