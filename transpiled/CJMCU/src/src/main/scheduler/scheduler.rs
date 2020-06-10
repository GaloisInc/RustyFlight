use ::libc;
extern "C" {
    #[no_mangle]
    fn memmove(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    static mut cfTasks: [cfTask_t; 11];
    #[no_mangle]
    fn micros() -> timeUs_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
// time difference, 32 bits always sufficient
pub type timeDelta_t = int32_t;
// microsecond time
pub type timeUs_t = uint32_t;
pub type C2RustUnnamed = libc::c_uint;
pub const TASK_PRIORITY_MAX: C2RustUnnamed = 255;
pub const TASK_PRIORITY_REALTIME: C2RustUnnamed = 6;
pub const TASK_PRIORITY_HIGH: C2RustUnnamed = 5;
pub const TASK_PRIORITY_MEDIUM_HIGH: C2RustUnnamed = 4;
pub const TASK_PRIORITY_MEDIUM: C2RustUnnamed = 3;
pub const TASK_PRIORITY_LOW: C2RustUnnamed = 1;
pub const TASK_PRIORITY_IDLE: C2RustUnnamed = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct cfCheckFuncInfo_t {
    pub maxExecutionTime: timeUs_t,
    pub totalExecutionTime: timeUs_t,
    pub averageExecutionTime: timeUs_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct cfTaskInfo_t {
    pub taskName: *const libc::c_char,
    pub subTaskName: *const libc::c_char,
    pub isEnabled: bool,
    pub staticPriority: uint8_t,
    pub desiredPeriod: timeDelta_t,
    pub latestDeltaTime: timeDelta_t,
    pub maxExecutionTime: timeUs_t,
    pub totalExecutionTime: timeUs_t,
    pub averageExecutionTime: timeUs_t,
}
pub type cfTaskId_e = libc::c_uint;
pub const TASK_SELF: cfTaskId_e = 12;
pub const TASK_NONE: cfTaskId_e = 11;
pub const TASK_COUNT: cfTaskId_e = 11;
pub const TASK_BATTERY_ALERTS: cfTaskId_e = 10;
pub const TASK_BATTERY_CURRENT: cfTaskId_e = 9;
pub const TASK_BATTERY_VOLTAGE: cfTaskId_e = 8;
pub const TASK_DISPATCH: cfTaskId_e = 7;
pub const TASK_SERIAL: cfTaskId_e = 6;
pub const TASK_RX: cfTaskId_e = 5;
pub const TASK_ATTITUDE: cfTaskId_e = 4;
pub const TASK_ACCEL: cfTaskId_e = 3;
pub const TASK_GYROPID: cfTaskId_e = 2;
pub const TASK_MAIN: cfTaskId_e = 1;
pub const TASK_SYSTEM: cfTaskId_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct cfTask_t {
    pub taskName: *const libc::c_char,
    pub subTaskName: *const libc::c_char,
    pub checkFunc: Option<unsafe extern "C" fn(_: timeUs_t, _: timeDelta_t)
                              -> bool>,
    pub taskFunc: Option<unsafe extern "C" fn(_: timeUs_t) -> ()>,
    pub desiredPeriod: timeDelta_t,
    pub staticPriority: uint8_t,
    pub dynamicPriority: uint16_t,
    pub taskAgeCycles: uint16_t,
    pub taskLatestDeltaTime: timeDelta_t,
    pub lastExecutedAt: timeUs_t,
    pub lastSignaledAt: timeUs_t,
    pub movingSumExecutionTime: timeUs_t,
    pub maxExecutionTime: timeUs_t,
    pub totalExecutionTime: timeUs_t,
}
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
// DEBUG_SCHEDULER, timings for:
// 0 - gyroUpdate()
// 1 - pidController()
// 2 - time spent in scheduler
// 3 - time spent executing check function
static mut currentTask: *mut cfTask_t = 0 as *const cfTask_t as *mut cfTask_t;
static mut totalWaitingTasks: uint32_t = 0;
static mut totalWaitingTasksSamples: uint32_t = 0;
static mut calculateTaskStatistics: bool = false;
#[no_mangle]
pub static mut averageSystemLoadPercent: uint16_t =
    0 as libc::c_int as uint16_t;
static mut taskQueuePos: libc::c_int = 0 as libc::c_int;
static mut taskQueueSize: libc::c_int = 0 as libc::c_int;
// No need for a linked list for the queue, since items are only inserted at startup
static mut taskQueueArray: [*mut cfTask_t; 12] =
    [0 as *const cfTask_t as *mut cfTask_t; 12];
// extra item for NULL pointer at end of queue
#[no_mangle]
pub unsafe extern "C" fn queueClear() {
    memset(taskQueueArray.as_mut_ptr() as *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<[*mut cfTask_t; 12]>() as libc::c_ulong);
    taskQueuePos = 0 as libc::c_int;
    taskQueueSize = 0 as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn queueContains(mut task: *mut cfTask_t) -> bool {
    let mut ii: libc::c_int = 0 as libc::c_int;
    while ii < taskQueueSize {
        if taskQueueArray[ii as usize] == task {
            return 1 as libc::c_int != 0
        }
        ii += 1
    }
    return 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn queueAdd(mut task: *mut cfTask_t) -> bool {
    if taskQueueSize >= TASK_COUNT as libc::c_int ||
           queueContains(task) as libc::c_int != 0 {
        return 0 as libc::c_int != 0
    }
    let mut ii: libc::c_int = 0 as libc::c_int;
    while ii <= taskQueueSize {
        if taskQueueArray[ii as usize].is_null() ||
               ((*taskQueueArray[ii as usize]).staticPriority as libc::c_int)
                   < (*task).staticPriority as libc::c_int {
            memmove(&mut *taskQueueArray.as_mut_ptr().offset((ii +
                                                                  1 as
                                                                      libc::c_int)
                                                                 as isize) as
                        *mut *mut cfTask_t as *mut libc::c_void,
                    &mut *taskQueueArray.as_mut_ptr().offset(ii as isize) as
                        *mut *mut cfTask_t as *const libc::c_void,
                    (::core::mem::size_of::<*mut cfTask_t>() as
                         libc::c_ulong).wrapping_mul((taskQueueSize - ii) as
                                                         libc::c_ulong));
            taskQueueArray[ii as usize] = task;
            taskQueueSize += 1;
            return 1 as libc::c_int != 0
        }
        ii += 1
    }
    return 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn queueRemove(mut task: *mut cfTask_t) -> bool {
    let mut ii: libc::c_int = 0 as libc::c_int;
    while ii < taskQueueSize {
        if taskQueueArray[ii as usize] == task {
            memmove(&mut *taskQueueArray.as_mut_ptr().offset(ii as isize) as
                        *mut *mut cfTask_t as *mut libc::c_void,
                    &mut *taskQueueArray.as_mut_ptr().offset((ii +
                                                                  1 as
                                                                      libc::c_int)
                                                                 as isize) as
                        *mut *mut cfTask_t as *const libc::c_void,
                    (::core::mem::size_of::<*mut cfTask_t>() as
                         libc::c_ulong).wrapping_mul((taskQueueSize - ii) as
                                                         libc::c_ulong));
            taskQueueSize -= 1;
            return 1 as libc::c_int != 0
        }
        ii += 1
    }
    return 0 as libc::c_int != 0;
}
/*
 * Returns first item queue or NULL if queue empty
 */
#[no_mangle]
pub unsafe extern "C" fn queueFirst() -> *mut cfTask_t {
    taskQueuePos = 0 as libc::c_int;
    return taskQueueArray[0 as libc::c_int as usize];
    // guaranteed to be NULL if queue is empty
}
/*
 * Returns next item in queue or NULL if at end of queue
 */
#[no_mangle]
pub unsafe extern "C" fn queueNext() -> *mut cfTask_t {
    taskQueuePos += 1;
    return taskQueueArray[taskQueuePos as usize];
    // guaranteed to be NULL at end of queue
}
#[no_mangle]
pub unsafe extern "C" fn taskSystemLoad(mut currentTimeUs: timeUs_t) {
    // Calculate system load
    if totalWaitingTasksSamples > 0 as libc::c_int as libc::c_uint {
        averageSystemLoadPercent =
            (100 as libc::c_int as
                 libc::c_uint).wrapping_mul(totalWaitingTasks).wrapping_div(totalWaitingTasksSamples)
                as uint16_t;
        totalWaitingTasksSamples = 0 as libc::c_int as uint32_t;
        totalWaitingTasks = 0 as libc::c_int as uint32_t
    };
}
#[no_mangle]
pub static mut checkFuncMaxExecutionTime: timeUs_t = 0;
#[no_mangle]
pub static mut checkFuncTotalExecutionTime: timeUs_t = 0;
#[no_mangle]
pub static mut checkFuncMovingSumExecutionTime: timeUs_t = 0;
#[no_mangle]
pub unsafe extern "C" fn getCheckFuncInfo(mut checkFuncInfo:
                                              *mut cfCheckFuncInfo_t) {
    (*checkFuncInfo).maxExecutionTime = checkFuncMaxExecutionTime;
    (*checkFuncInfo).totalExecutionTime = checkFuncTotalExecutionTime;
    (*checkFuncInfo).averageExecutionTime =
        checkFuncMovingSumExecutionTime.wrapping_div(32 as libc::c_int as
                                                         libc::c_uint);
}
#[no_mangle]
pub unsafe extern "C" fn getTaskInfo(mut taskId: cfTaskId_e,
                                     mut taskInfo: *mut cfTaskInfo_t) {
    (*taskInfo).taskName = cfTasks[taskId as usize].taskName;
    (*taskInfo).subTaskName = cfTasks[taskId as usize].subTaskName;
    (*taskInfo).isEnabled =
        queueContains(&mut *cfTasks.as_mut_ptr().offset(taskId as isize));
    (*taskInfo).desiredPeriod = cfTasks[taskId as usize].desiredPeriod;
    (*taskInfo).staticPriority = cfTasks[taskId as usize].staticPriority;
    (*taskInfo).maxExecutionTime = cfTasks[taskId as usize].maxExecutionTime;
    (*taskInfo).totalExecutionTime =
        cfTasks[taskId as usize].totalExecutionTime;
    (*taskInfo).averageExecutionTime =
        cfTasks[taskId as
                    usize].movingSumExecutionTime.wrapping_div(32 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_uint);
    (*taskInfo).latestDeltaTime =
        cfTasks[taskId as usize].taskLatestDeltaTime;
}
#[no_mangle]
pub unsafe extern "C" fn rescheduleTask(mut taskId: cfTaskId_e,
                                        mut newPeriodMicros: uint32_t) {
    if taskId as libc::c_uint == TASK_SELF as libc::c_int as libc::c_uint {
        let mut task: *mut cfTask_t = currentTask;
        (*task).desiredPeriod =
            ({
                 let mut _a: libc::c_int = 100 as libc::c_int;
                 let mut _b: timeDelta_t = newPeriodMicros as timeDelta_t;
                 if _a > _b { _a } else { _b }
             })
        // Limit delay to 100us (10 kHz) to prevent scheduler clogging
    } else if (taskId as libc::c_uint) <
                  TASK_COUNT as libc::c_int as libc::c_uint {
        let mut task_0: *mut cfTask_t =
            &mut *cfTasks.as_mut_ptr().offset(taskId as isize) as
                *mut cfTask_t;
        (*task_0).desiredPeriod =
            ({
                 let mut _a: libc::c_int = 100 as libc::c_int;
                 let mut _b: timeDelta_t = newPeriodMicros as timeDelta_t;
                 if _a > _b { _a } else { _b }
             })
        // Limit delay to 100us (10 kHz) to prevent scheduler clogging
    };
}
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
// Disables dynamic scheduling, task is executed only if no other task is active this cycle
/* Actual tasks */
/* Count of real tasks */
/* Service task IDs */
// Configuration
// target period of execution
// dynamicPriority grows in steps of this size, shouldn't be zero
// Scheduling
// measurement of how old task was last executed, used to avoid task starvation
// last time of invocation
// time of invocation event for event-driven tasks
// Statistics
// moving sum over 32 samples
// total time consumed by task since boot
#[no_mangle]
pub unsafe extern "C" fn setTaskEnabled(mut taskId: cfTaskId_e,
                                        mut enabled: bool) {
    if taskId as libc::c_uint == TASK_SELF as libc::c_int as libc::c_uint ||
           (taskId as libc::c_uint) <
               TASK_COUNT as libc::c_int as libc::c_uint {
        let mut task: *mut cfTask_t =
            if taskId as libc::c_uint ==
                   TASK_SELF as libc::c_int as libc::c_uint {
                currentTask
            } else {
                &mut *cfTasks.as_mut_ptr().offset(taskId as isize) as
                    *mut cfTask_t
            };
        if enabled as libc::c_int != 0 && (*task).taskFunc.is_some() {
            queueAdd(task);
        } else { queueRemove(task); }
    };
}
#[no_mangle]
pub unsafe extern "C" fn getTaskDeltaTime(mut taskId: cfTaskId_e)
 -> timeDelta_t {
    if taskId as libc::c_uint == TASK_SELF as libc::c_int as libc::c_uint {
        return (*currentTask).taskLatestDeltaTime
    } else if (taskId as libc::c_uint) <
                  TASK_COUNT as libc::c_int as libc::c_uint {
        return cfTasks[taskId as usize].taskLatestDeltaTime
    } else { return 0 as libc::c_int };
}
#[no_mangle]
pub unsafe extern "C" fn schedulerSetCalulateTaskStatistics(mut calculateTaskStatisticsToUse:
                                                                bool) {
    calculateTaskStatistics = calculateTaskStatisticsToUse;
}
#[no_mangle]
pub unsafe extern "C" fn schedulerResetTaskStatistics(mut taskId:
                                                          cfTaskId_e) {
    if taskId as libc::c_uint == TASK_SELF as libc::c_int as libc::c_uint {
        (*currentTask).movingSumExecutionTime = 0 as libc::c_int as timeUs_t;
        (*currentTask).totalExecutionTime = 0 as libc::c_int as timeUs_t;
        (*currentTask).maxExecutionTime = 0 as libc::c_int as timeUs_t
    } else if (taskId as libc::c_uint) <
                  TASK_COUNT as libc::c_int as libc::c_uint {
        cfTasks[taskId as usize].movingSumExecutionTime =
            0 as libc::c_int as timeUs_t;
        cfTasks[taskId as usize].totalExecutionTime =
            0 as libc::c_int as timeUs_t;
        cfTasks[taskId as usize].maxExecutionTime =
            0 as libc::c_int as timeUs_t
    };
}
#[no_mangle]
pub unsafe extern "C" fn schedulerResetTaskMaxExecutionTime(mut taskId:
                                                                cfTaskId_e) {
    if taskId as libc::c_uint == TASK_SELF as libc::c_int as libc::c_uint {
        (*currentTask).maxExecutionTime = 0 as libc::c_int as timeUs_t
    } else if (taskId as libc::c_uint) <
                  TASK_COUNT as libc::c_int as libc::c_uint {
        cfTasks[taskId as usize].maxExecutionTime =
            0 as libc::c_int as timeUs_t
    };
}
#[no_mangle]
pub unsafe extern "C" fn schedulerInit() {
    calculateTaskStatistics = 1 as libc::c_int != 0;
    queueClear();
    queueAdd(&mut *cfTasks.as_mut_ptr().offset(TASK_SYSTEM as libc::c_int as
                                                   isize));
}
#[no_mangle]
pub unsafe extern "C" fn scheduler() {
    // Cache currentTime
    let currentTimeUs: timeUs_t = micros();
    // Check for realtime tasks
    let mut outsideRealtimeGuardInterval: bool = 1 as libc::c_int != 0;
    let mut task: *const cfTask_t = queueFirst();
    while !task.is_null() &&
              (*task).staticPriority as libc::c_int >=
                  TASK_PRIORITY_REALTIME as libc::c_int {
        let nextExecuteAt: timeUs_t =
            (*task).lastExecutedAt.wrapping_add((*task).desiredPeriod as
                                                    libc::c_uint);
        if currentTimeUs.wrapping_sub(nextExecuteAt) as timeDelta_t >=
               0 as libc::c_int {
            outsideRealtimeGuardInterval = 0 as libc::c_int != 0;
            break ;
        } else { task = queueNext() }
    }
    // The task to be invoked
    let mut selectedTask: *mut cfTask_t = 0 as *mut cfTask_t;
    let mut selectedTaskDynamicPriority: uint16_t =
        0 as libc::c_int as uint16_t;
    // Update task dynamic priorities
    let mut waitingTasks: uint16_t = 0 as libc::c_int as uint16_t;
    let mut task_0: *mut cfTask_t = queueFirst();
    while !task_0.is_null() {
        // Task has checkFunc - event driven
        if (*task_0).checkFunc.is_some() {
            let currentTimeBeforeCheckFuncCall: timeUs_t = currentTimeUs;
            // Increase priority for event driven tasks
            if (*task_0).dynamicPriority as libc::c_int > 0 as libc::c_int {
                (*task_0).taskAgeCycles =
                    (1 as libc::c_int as
                         libc::c_uint).wrapping_add(currentTimeUs.wrapping_sub((*task_0).lastSignaledAt).wrapping_div((*task_0).desiredPeriod
                                                                                                                          as
                                                                                                                          libc::c_uint))
                        as uint16_t; // time consumed by scheduler + task
                (*task_0).dynamicPriority =
                    (1 as libc::c_int +
                         (*task_0).staticPriority as libc::c_int *
                             (*task_0).taskAgeCycles as libc::c_int) as
                        uint16_t;
                waitingTasks = waitingTasks.wrapping_add(1)
            } else if (*task_0).checkFunc.expect("non-null function pointer")(currentTimeBeforeCheckFuncCall,
                                                                              currentTimeBeforeCheckFuncCall.wrapping_sub((*task_0).lastExecutedAt)
                                                                                  as
                                                                                  timeDelta_t)
             {
                if calculateTaskStatistics {
                    let checkFuncExecutionTime: uint32_t =
                        micros().wrapping_sub(currentTimeBeforeCheckFuncCall);
                    checkFuncMovingSumExecutionTime =
                        (checkFuncMovingSumExecutionTime as
                             libc::c_uint).wrapping_add(checkFuncExecutionTime.wrapping_sub(checkFuncMovingSumExecutionTime.wrapping_div(32
                                                                                                                                             as
                                                                                                                                             libc::c_int
                                                                                                                                             as
                                                                                                                                             libc::c_uint)))
                            as timeUs_t as timeUs_t;
                    checkFuncTotalExecutionTime =
                        (checkFuncTotalExecutionTime as
                             libc::c_uint).wrapping_add(checkFuncExecutionTime)
                            as timeUs_t as timeUs_t;
                    checkFuncMaxExecutionTime =
                        ({
                             let mut _a: timeUs_t = checkFuncMaxExecutionTime;
                             let _b: uint32_t = checkFuncExecutionTime;
                             if _a > _b { _a } else { _b }
                         })
                }
                (*task_0).lastSignaledAt = currentTimeBeforeCheckFuncCall;
                (*task_0).taskAgeCycles = 1 as libc::c_int as uint16_t;
                (*task_0).dynamicPriority =
                    (1 as libc::c_int +
                         (*task_0).staticPriority as libc::c_int) as uint16_t;
                waitingTasks = waitingTasks.wrapping_add(1)
            } else { (*task_0).taskAgeCycles = 0 as libc::c_int as uint16_t }
        } else {
            // Task is time-driven, dynamicPriority is last execution age (measured in desiredPeriods)
            // Task age is calculated from last execution
            (*task_0).taskAgeCycles =
                currentTimeUs.wrapping_sub((*task_0).lastExecutedAt).wrapping_div((*task_0).desiredPeriod
                                                                                      as
                                                                                      libc::c_uint)
                    as uint16_t;
            if (*task_0).taskAgeCycles as libc::c_int > 0 as libc::c_int {
                (*task_0).dynamicPriority =
                    (1 as libc::c_int +
                         (*task_0).staticPriority as libc::c_int *
                             (*task_0).taskAgeCycles as libc::c_int) as
                        uint16_t;
                waitingTasks = waitingTasks.wrapping_add(1)
            }
        }
        if (*task_0).dynamicPriority as libc::c_int >
               selectedTaskDynamicPriority as libc::c_int {
            let taskCanBeChosenForScheduling: bool =
                outsideRealtimeGuardInterval as libc::c_int != 0 ||
                    (*task_0).taskAgeCycles as libc::c_int > 1 as libc::c_int
                    ||
                    (*task_0).staticPriority as libc::c_int ==
                        TASK_PRIORITY_REALTIME as libc::c_int;
            if taskCanBeChosenForScheduling {
                selectedTaskDynamicPriority = (*task_0).dynamicPriority;
                selectedTask = task_0
            }
        }
        task_0 = queueNext()
    }
    totalWaitingTasksSamples = totalWaitingTasksSamples.wrapping_add(1);
    totalWaitingTasks =
        (totalWaitingTasks as
             libc::c_uint).wrapping_add(waitingTasks as libc::c_uint) as
            uint32_t as uint32_t;
    currentTask = selectedTask;
    if !selectedTask.is_null() {
        // Found a task that should be run
        (*selectedTask).taskLatestDeltaTime =
            currentTimeUs.wrapping_sub((*selectedTask).lastExecutedAt) as
                timeDelta_t;
        (*selectedTask).lastExecutedAt = currentTimeUs;
        (*selectedTask).dynamicPriority = 0 as libc::c_int as uint16_t;
        // Execute task
        if calculateTaskStatistics {
            let currentTimeBeforeTaskCall: timeUs_t =
                micros(); // time consumed by scheduler + task
            (*selectedTask).taskFunc.expect("non-null function pointer")(currentTimeBeforeTaskCall);
            let taskExecutionTime: timeUs_t =
                micros().wrapping_sub(currentTimeBeforeTaskCall);
            (*selectedTask).movingSumExecutionTime =
                ((*selectedTask).movingSumExecutionTime as
                     libc::c_uint).wrapping_add(taskExecutionTime.wrapping_sub((*selectedTask).movingSumExecutionTime.wrapping_div(32
                                                                                                                                       as
                                                                                                                                       libc::c_int
                                                                                                                                       as
                                                                                                                                       libc::c_uint)))
                    as timeUs_t as timeUs_t;
            (*selectedTask).totalExecutionTime =
                ((*selectedTask).totalExecutionTime as
                     libc::c_uint).wrapping_add(taskExecutionTime) as timeUs_t
                    as timeUs_t;
            (*selectedTask).maxExecutionTime =
                ({
                     let mut _a: timeUs_t = (*selectedTask).maxExecutionTime;
                     let _b: timeUs_t = taskExecutionTime;
                     if _a > _b { _a } else { _b }
                 })
        } else {
            (*selectedTask).taskFunc.expect("non-null function pointer")(currentTimeUs);
        }
    };
}
