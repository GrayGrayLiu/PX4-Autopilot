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

#include "WorkItemExample.hpp"

WorkItemExample::WorkItemExample() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

WorkItemExample::~WorkItemExample()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool WorkItemExample::init()
{
	// execute Run() on every sensor_accel publication
	if (!_sensor_accel_sub.registerCallback()) {  //模块对象在初始化时，已经将uORB消息sensor_accel与_sensor_accel_sub对象绑定，registerCallback()方法
		PX4_ERR("callback registration failed");  //估计是通过this指针，将本模块对象注册到了_sensor_accel_sub对象中，当sensor_accel消息更新时，
		return false;                             //_sensor_accel_sub对象就可以通过它记录的本模块对象的指针，调用到本模块对象的Run方法
	}

	// alternatively, Run on fixed interval  //这里的注释其实是注释了ScheduleOnInterval()这个函数，意思是也可以选择用这个函数来实现固定周期调度
	// ScheduleOnInterval(5000_us); // 2000 us interval, 200 Hz rate

	return true;
}

void WorkItemExample::Run()  //注意，Run()方法是ScheduledWorkItem类的一个纯虚函数，ModuleBase类模板中的是run()
{
	if (should_exit()) {  //判断是否发出了结束任务的命令，如果需要结束，则清除调度和回收内存
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);           //perf:Performance，即性能，posix标准下用来分析函数运行时间等信息的工具，叫做性能分析工具，
	perf_count(_loop_interval_perf);  //操作系统中的任务管理器，正是通过这种操作才能获得数据

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;                //uORB机制的copy方法会清楚更新标志位，所以此处复制消息并不是为了获取参数，而是为了清除更新标志位，px4中很多模块都会轮询这个
		_parameter_update_sub.copy(&param_update);  //查询参数更新的方法，但参数更新之后，只需要刷新一次就够了，所以哪个任务处理到了这个参数更新，就由这个任务来清楚更新标志位。
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)  //会更新当前类及其所有子类中的参数，DEFINE_PARAMETERS宏
	}                                                                       //正是定义了每个类中需要更新的参数以及实现了更新的函数


	// Example
	//  update vehicle_status to check arming state
	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {

			const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

			if (armed && !_armed) {
				PX4_WARN("vehicle armed due to %d", vehicle_status.latest_arming_reason);

			} else if (!armed && _armed) {
				PX4_INFO("vehicle disarmed due to %d", vehicle_status.latest_disarming_reason);
			}

			_armed = armed;
		}
	}


	// Example          
	//  grab latest accelerometer data
	if (_sensor_accel_sub.updated()) {
		sensor_accel_s accel;

		if (_sensor_accel_sub.copy(&accel)) {
			// DO WORK

			// access parameter value (SYS_AUTOSTART)
			if (_param_sys_autostart.get() == 1234) {
				// do something if SYS_AUTOSTART is 1234
			}
		}
	}


	// Example
	//  publish some data
	orb_test_s data{};
	data.val = 314159;
	data.timestamp = hrt_absolute_time();  //获取自系统启动以来的时间（us）
	_orb_test_pub.publish(data);  //发布该消息，类模板Publication<orb_test_s>中已经定义好了publish()这个方法，该方法中，如果uORB主题未公布，会自动公布该主题，所以发布消息前不需要用户手动公布一次


	perf_end(_loop_perf);
}

int WorkItemExample::task_spawn(int argc, char *argv[])
{
	WorkItemExample *instance = new WorkItemExample();  //创建类的实例正是在这里完成的，创建实例的同时，实例的构造函数被调用，完成对该实例的初始化，初始化时，配置了模块的参数，设置了工作队列为test1

	if (instance) {  //如果实例化成功
		_object.store(instance);  //将实例的指针记录下来，完成对象的创建
		_task_id = task_id_is_work_queue;  //任务id被设置为-2，表示是一个工作队列

		if (instance->init()) {  //对实例进行一些初始化工作，主要是设置调度方式（按固定周期调度，或者消息驱动调度（订阅的uORB更新时回调）），调度对象为Run方法
			return PX4_OK;  //返回0，表示实例化成功并完成初始化
		}

	} else {  //如果失败
		PX4_ERR("alloc failed");
	}

	delete instance;  //如果实例化失败，删除实例指针
	_object.store(nullptr);  //将_object和_task_id复位
	_task_id = -1;

	return PX4_ERROR;
}

int WorkItemExample::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int WorkItemExample::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int WorkItemExample::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int work_item_example_main(int argc, char *argv[])  //操作系统调用该模块时的入口
{
	return WorkItemExample::main(argc, argv);  //main在ModuleBase类模板中是静态函数（static），所以可以在没有创建对象的情况下，直接通过类调用
}
