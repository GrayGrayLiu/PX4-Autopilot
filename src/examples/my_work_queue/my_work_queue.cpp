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
 * @file my_work_queue.cpp
 * @brief
 * @author Gray <grayme12345@gmail.com>
 * @time 2023/12/2.
 */


#include "my_work_queue.hpp"

MyWorkQueue::MyWorkQueue() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

MyWorkQueue::~MyWorkQueue()
{
    perf_free(_loop_perf);
    perf_free(_loop_interval_perf);
}

int MyWorkQueue::task_spawn(int argc, char* argv[])
{
    MyWorkQueue *instance = new MyWorkQueue();

    if (instance)
    {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        if (instance->init())
        {
            return PX4_OK;
        }

    }
    else
    {
        PX4_ERR("alloc failed");
    }

    delete instance;
    _object.store(nullptr);
    _task_id = -1;

    return PX4_ERROR;
}

int MyWorkQueue::custom_command(int argc, char* argv[])
{
    return print_usage("unknown command");
}

int MyWorkQueue::print_usage(const char* reason)
{
    if (reason)
    {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
            R"DESCR_STR(
                                    ### Description
                                    Example of a simple module running out of a work queue.

                                    )DESCR_STR");

    PRINT_MODULE_USAGE_NAME("my_work_queue", "template");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int MyWorkQueue::print_status()
{
    perf_print_counter(_loop_perf);
    perf_print_counter(_loop_interval_perf);

    return 0;
}

bool MyWorkQueue::init()
{
    if(!_att_pid_param_sub.registerCallback())
    {
        PX4_ERR("callback registration failed");
        return false;
    }

    return true;
}

void MyWorkQueue::Run()
{
    if (should_exit())
    {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    perf_begin(_loop_perf);
    perf_count(_loop_interval_perf);

    if (_parameter_update_sub.updated())
    {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);
        updateParams();
    }

    if(_my_attitude_sub.updated())
    {
        my_attitude_s my_attitude;

        if(_my_attitude_sub.copy(&my_attitude))
        {
        }
    }

    if(_att_pid_param_sub.updated())
    {
        att_pid_param_s att_pid_param;

        if(_att_pid_param_sub.copy(&att_pid_param))
        {
            _param_my_att_p.set(att_pid_param.p);

//            att_pid_param.p /=2;
//            att_pid_param.i /=2;
//            att_pid_param.d /=2;
//            att_pid_param.timestamp = hrt_absolute_time();
//            _att_pid_param_pub.publish(att_pid_param);
        }
    }

    perf_end(_loop_perf);
}

extern "C" __EXPORT int my_work_queue_main(int argc, char *argv[])
{
    return MyWorkQueue::main(argc, argv);
}