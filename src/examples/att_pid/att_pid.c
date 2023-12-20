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
 * @file my_hello.c
 * @brief This is just a simulation to modify the attitude loop PID parameters and does not actually change the real parameters.
 * @author Gray <grayme12345@gmail.com>
 * @time 2023/11/30.
 */

#include "att_pid.h"

__EXPORT int att_pid_main(int argc, char* argv[]);

/*主函数两个入口参数的来源：本文件最终会被编译成静态库，如果被编译进飞控板，则可以通过QGC的MAVLink控制台，在nsh（NuttX Shell）环境下调用,nsh中的调用命令为
 * att_pid，nsh 会从将输入的“命令+参数”转化为一个字符串数组，以空格作为字符串分隔标志，所以主函数的两入口参数时有nsh给出的，argc为字符串数组的大小，argv
 * 是字符串数组本身,根据这两个参数，我们就能知道nsh中被用户输入了什么命令，然后据此编写代码*/
int att_pid_main(int argc, char* argv[])
{
    stm32_gpiowrite(GPIO_VDD_5V_PERIPH_EN, 1);
    /* 订阅一个姿态环PID参数话题 */
    /*因为uORB主题其实是一个对象，这个对象是全局的，具有全生命周期，并不会随着att_pid_main结束而结束，
     * 所以此处的订阅是为了从nsh再次进入att_pid_main时，取得之前的数据*/
    struct att_pid_param_s AttPID;  //姿态环PID参数的结构体
    int AttPIDSub = orb_subscribe( ORB_ID(att_pid_param) );  //订阅姿态环PID参数主题
    orb_copy( ORB_ID(att_pid_param), AttPIDSub, &AttPID );  //复制出主题中的数据

    /* 公告一个姿态环PID参数话题 */
    /*这里的公告其实只需要一次，但是由于att_pid_main是从命令行调用的，att_pid_main不会作为任务一直存在，所以不可避免的会每进入att_pid_main一次就公告一次，
     * 虽然后面的公告已经没意义了。另外一点，估计公告的时候会用AttPID的值初始化att_pid_param主题，所以本来是应先公告，再订阅的过程，在此处不得不把公告放订阅后面，
     * 否则每次先公告的话，这个时候不知道以前的值，那么这个时候初始化就会使以前的值丢失。所以众多矛盾的根本原因在于这里的公告不得不被反复执行，解决方法就是把这个公告放在
     * 可以只执行一次的地方，比如整个程序的初始化过程中。但这个程序只是一个尝试。*/
//    struct att_pid_param_s AttPID;  //姿态环PID参数的结构体
//    memset( &AttPID, 0, sizeof(AttPID) );  //结构体初始化为0
    orb_advert_t AttPIDPub=orb_advertise( ORB_ID(att_pid_param), &AttPID );  //用uORB提供的API公告姿态环PID参数主题，返回一个句柄，实际上是该主题对象的指针，orb_advert_t原本是一个空指针

    if (argc == 1)  //只有att_pid命令，则输出PID参数
    {
        PX4_INFO("P:%-8.2f  I:%-8.2f  D:%-8.2f", (double)AttPID.p, (double)AttPID.i, (double)AttPID.d);

        return 0;
    }
    else if(argc == 2)  //赋值P
    {
        char* EndPtr=NULL;
        float P = (float)strtod(argv[1], &EndPtr);  //函数的第二个参数EndPtr指向被转换后的字符串中第一个不能被转换的字符位置。
        if(*EndPtr!='\0')
        {
            PX4_WARN("PID parameters must be numeric!");
            return 0;
        }
        else
            AttPID.p = P;
    }
    else if(argc == 3)  //赋值PI
    {
        char* EndPtr=NULL;
        float P = (float)strtod(argv[1], &EndPtr);  //函数的第二个参数EndPtr指向被转换后的字符串中第一个不能被转换的字符位置。
        float I = (float)strtod(argv[2], &EndPtr);
        if(*EndPtr!='\0')
        {
            PX4_WARN("PID parameters must be numeric!");
            return 0;
        }
        else
        {
            AttPID.p = P;
            AttPID.i = I;
        }
    }
    else if(argc == 4)  //赋值PID
    {
        char* EndPtr=NULL;
        float P = (float)strtod(argv[1], &EndPtr);  //函数的第二个参数EndPtr指向被转换后的字符串中第一个不能被转换的字符位置。
        float I = (float)strtod(argv[2], &EndPtr);
        float D = (float)strtod(argv[3], &EndPtr);
        if(*EndPtr!='\0')
        {
            PX4_WARN("PID parameters must be numeric!");
            return 0;
        }
        else
        {
            AttPID.p = P;
            AttPID.i = I;
            AttPID.d = D;
        }
    }
    else if(argc > 4)  //参数超过3个
    {
        PrintUsage();

        return 0;
    }

    orb_publish( ORB_ID(att_pid_param), AttPIDPub, &AttPID );  //发布姿态环PID参数主题的数据

    return 0;
}

static void PrintUsage(void)
{
    PX4_INFO("att_pid <p> <i> <d>");
}
