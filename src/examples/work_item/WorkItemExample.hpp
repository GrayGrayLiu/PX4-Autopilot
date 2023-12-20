/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/orb_test.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;  //该命名空间中包含了高分辨率定时器的接口

class WorkItemExample : public ModuleBase<WorkItemExample>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	WorkItemExample();
	~WorkItemExample() override;  //因为ModuleBase类模版将析构函数定义为了虚函数，所以子类ModuleBase必须重写父类的析构函数

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);  //工作队列方式必须要实现的三个接口task_spawn()、custom_command()、print_usage()

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);  //接口都最好定义为静态成员函数（static），因为这样可以在不需要类的实例的情况下被调用

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();  //在task_spawn()中调用init()设置初始化工作

	int print_status() override;  //ModuleBase类模板中定义了该函数，所以并非必须重写，可由用户自己决定

private:
	void Run() override;  //注意，Run()方法是ScheduledWorkItem类的一个纯虚函数，ModuleBase类模板中的是run()，工作队列方式会调用Run，任务方式会调用run

	// Publications  展示了在任务中发布消息的方式
	uORB::Publication<orb_test_s> _orb_test_pub{ORB_ID(orb_test)};

	// Subscriptions  展示了三种不同的通过uORB驱动任务的方式，第一种是消息主动回调任务，第二种与第三种需要用户自己查询消息是否更新，一般以轮询的方式
	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};        // subscription that schedules WorkItemExample when updated  更新则回调方式
	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates  固定消息更新频率（但本质上应该是以固定的频率更新标志位，也就是说无论消息是否更新，总是会定期把标志位置位），从而固定任务频率
	uORB::Subscription                 _vehicle_status_sub{ORB_ID(vehicle_status)};          // regular subscription for additional data  最常规的方式，消息更新时抛出标志位

	// Performance (perf) counters  性能计数器，perf_counter_t是一个性能计数器结构体指针，perf_alloc函数返回一个性能计数器结构体指针，初始化时赋值
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};  //PC_ELAPSED是一个性能计数器类型，表示测量经过的时间，即测量一个事件从开始到结束的总时间
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};  //PC_INTERVAL 是另一种性能计数器类型，用于测量事件或代码段在一定时间间隔内的执行次数或频率

	// Parameters  DEFINE_PARAMETERS()这个宏函数实际上是实现了重写ModuleParams类中updateParamsImpl()这个虚函数接口，该宏函数定义为任意入口参数的形式
	DEFINE_PARAMETERS(  //这个部分的用途实际上有两个。一是定义了这个类需要用到的px4中的参数，比如控制器参数。二是更新用到的参数
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */  //这两行实际上就是声明了要用到的参数，然后就可以在类中，
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */  //比如通过_param_sys_autostart这个句柄来使用或者对该参数赋值
	)                                                                                             //正是因为可以通过这些句柄来赋值，所以需要更新参数


	bool _armed{false};
};
