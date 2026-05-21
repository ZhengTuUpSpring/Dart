#ifndef ALL_TASK_H
#define ALL_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>

#include "remote_control.h"
#include "GimbalControl.h"
#include "Shoot_Control.h"
#include "M2006_Driver.h"

extern RC_ctrl_t rc_ctrl;

#endif
