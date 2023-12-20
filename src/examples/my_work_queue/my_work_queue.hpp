/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file my_work_queue.hpp
 * @brief
 * @author Gray <grayme12345@gmail.com>
 * @time 2023/12/2.
 */

/*#pragma once 一般由编译器提供保证：同一个文件不会被包含多次。注意这里所说的“同一个文件”是指物理上的一个文件，而不是指内容相同的两个文件。
 * 相比于传统的声明保护宏的方式（#ifndefine #define #endif），该方式的优点是避免了保护宏方式中的条件判断，从而提高了编译速度，而且写法也更简洁。
 * 但它的缺点是：1、如果某个头文件有多份拷贝，本方法不能保证他们不被重复包含。2、这个语法并不是C/C++的标准，这个语法需要编译器的支持，一般不受一些
 * 较老版本的编译器支持，一些支持了的编译器又打算去掉它，所以它的兼容性可能不够好。*/
#pragma once

#include <px4_platform_common/defines.h>  //定义了PX4中一些常用的宏
#include <px4_platform_common/module.h>   //定义了模块（modules）的模版
#include <px4_platform_common/module_params.h>  //定义了设置模块参数的类，主要是设置某模块的子类和同步父类和子类的参数，并维护更新一个关系表
#include <px4_platform_common/posix.h>  //posix标准的接口
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>  //设置工作队列调度方式的类

#include <drivers/drv_hrt.h>  //hrt：High Resolution Timer（高分辨率定时器）
#include <lib/perf/perf_counter.h>  //perf：Performance  性能分析工具的头文件

#include <uORB/Publication.hpp>  //uORB机制发布消息功能的基类
#include <uORB/Subscription.hpp>  //uORB机制订阅消息功能的基类
#include <uORB/SubscriptionCallback.hpp>  /*实现uORB机制订阅回调功能的类。订阅回调机制：订阅者可以注册一个函数到uORB，然后当订阅者订阅
                                           * 的消息更新时，uORB回调这个函数，这样就可以实现对任务进行消息驱动*/
#include <uORB/topics/parameter_update.h>
#include "uORB/topics/att_pid_param.h"
#include "uORB/topics/my_attitude.h"
#include "uORB/topics/my_att_loop_out.h"

using namespace time_literals;

class MyWorkQueue : public ModuleBase<MyWorkQueue>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    MyWorkQueue();
    ~MyWorkQueue() override;  //因为ModuleBase类模版将析构函数定义为了虚函数，所以子类ModuleBase必须重写父类的析构函数

    /* 工作队列方式必须要实现的三个接口task_spawn()、custom_command()、print_usage()，
     * 接口都最好定义为静态成员函数（static），因为这样可以在不需要类的实例的情况下被调用*/
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char* reason=nullptr);

    int print_status() override;  //ModuleBase类模板中定义了该函数，所以并非必须重写，可由用户自己决定

    bool init();  //在task_spawn()中调用init()设置初始化工作

private:
    /* 注意，Run()方法是ScheduledWorkItem类的一个纯虚函数，ModuleBase类模板中的是run()，
     * 工作队列方式会调用Run，任务方式会调用run*/
    void Run() override;

    /*发布消息初始化*/
    uORB::Publication<my_att_loop_out_s> _my_att_loop_out_pub{ ORB_ID(my_att_loop_out) };
    uORB::Publication<att_pid_param_s> _att_pid_param_pub{ ORB_ID(att_pid_param) };

    /*订阅消息初始化*/
    uORB::SubscriptionInterval _parameter_update_sub{ ORB_ID(parameter_update), 1_s };
    uORB::Subscription _my_attitude_sub{ ORB_ID(my_attitude)};
    uORB::SubscriptionCallbackWorkItem _att_pid_param_sub{this, ORB_ID(att_pid_param)};

    /*性能计数器*/
    perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};  //PC_ELAPSED是一个性能计数器类型，表示测量经过的时间，即测量一个事件从开始到结束的总时间
    perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};  //PC_INTERVAL 是另一种性能计数器类型，用于测量事件或代码段在一定时间间隔内的执行次数或频率

    /*参数*/
    DEFINE_PARAMETERS(
            (ParamFloat<px4::params::MY_ATT_1_P>) _param_my_att_p
//            (ParamFloat<px4::params::MY_ATT_I>) _param_my_att_i
//            (paramFloat<px4::params::MY_ATT_D>) _param_my_att_d
    )
};
