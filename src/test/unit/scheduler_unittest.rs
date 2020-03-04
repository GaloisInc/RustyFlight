#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(const_raw_ptr_to_usize_cast, register_tool)]
pub type uint32_t = libc::c_uint;
/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */
#[no_mangle]
pub static mut TEST_PID_LOOP_TIME: libc::c_int = 650 as libc::c_int;
#[no_mangle]
pub static mut TEST_UPDATE_ACCEL_TIME: libc::c_int = 192 as libc::c_int;
#[no_mangle]
pub static mut TEST_HANDLE_SERIAL_TIME: libc::c_int = 30 as libc::c_int;
#[no_mangle]
pub static mut TEST_UPDATE_BATTERY_TIME: libc::c_int = 1 as libc::c_int;
#[no_mangle]
pub static mut TEST_UPDATE_RX_CHECK_TIME: libc::c_int = 34 as libc::c_int;
#[no_mangle]
pub static mut TEST_UPDATE_RX_MAIN_TIME: libc::c_int = 1 as libc::c_int;
#[no_mangle]
pub static mut TEST_IMU_UPDATE_TIME: libc::c_int = 5 as libc::c_int;
#[no_mangle]
pub static mut TEST_DISPATCH_TIME: libc::c_int = 1 as libc::c_int;
// set up micros() to simulate time
// set up tasks to take a simulated representative time to execute
#[no_mangle]
pub unsafe extern "C" fn TEST(mut SchedulerUnittest: libc::c_int,
                              mut TestPriorites: libc::c_int) -> libc::c_int {
    panic!("Reached end of non-void function without returning");
}
#[no_mangle]
pub static mut deadBeefPtr: *mut libc::c_int =
    0 as *const libc::c_int as *mut libc::c_int;
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_0(mut SchedulerUnittest: libc::c_int,
                                mut TestQueue: libc::c_int) -> libc::c_int {
    queueClear();
    panic!("Reached end of non-void function without returning");
    // TASK_PRIORITY_MEDIUM_HIGH
    // TASK_PRIORITY_REALTIME
    // TASK_PRIORITY_LOW
    // TASK_PRIORITY_MEDIUM
    // TASK_PRIORITY_HIGH
    // TASK_PRIORITY_HIGH
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_1(mut SchedulerUnittest: libc::c_int,
                                mut TestQueueAddAndRemove: libc::c_int)
 -> libc::c_int {
    queueClear();
    panic!("Reached end of non-void function without returning");
    // fill up the queue
    // double check end of queue
    // last item was indeed added to queue
    // null pointer at end of queue is preserved
    // there hasn't been an out by one error
    // and empty it again
    // double check size and end of queue
    // queue is indeed empty
    // there is a null pointer at the end of the queueu
    // no accidental overwrites past end of queue
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_2(mut SchedulerUnittest: libc::c_int,
                                mut TestQueueArray: libc::c_int)
 -> libc::c_int {
    // test there are no "out by one" errors or buffer overruns when items are added and removed
    queueClear();
    // note, must set deadBeefPtr after queueClear
    let mut enqueuedTasks: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    let mut lastTaskPrev: *const libc::c_int = 0 as *const libc::c_int;
    panic!("Reached end of non-void function without returning");
    // NULL at end of queue
    // check no buffer overrun
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_3(mut SchedulerUnittest: libc::c_int,
                                mut TestSchedulerInit: libc::c_int)
 -> libc::c_int {
    schedulerInit();
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_4(mut SchedulerUnittest: libc::c_int,
                                mut TestScheduleEmptyQueue: libc::c_int)
 -> libc::c_int {
    queueClear();
    // run the with an empty queue
    scheduler();
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_5(mut SchedulerUnittest: libc::c_int,
                                mut TestSingleTask: libc::c_int)
 -> libc::c_int {
    schedulerInit();
    // disable all tasks except TASK_GYROPID
    // run the scheduler and check the task has executed
    scheduler();
    panic!("Reached end of non-void function without returning");
    // task has run, so its dynamic priority should have been set to zero
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_6(mut SchedulerUnittest: libc::c_int,
                                mut TestTwoTasks: libc::c_int)
 -> libc::c_int {
    // disable all tasks except TASK_GYROPID  and TASK_ACCEL
    // set it up so that TASK_ACCEL ran just before TASK_GYROPID
    static mut startTime: uint32_t = 4000 as libc::c_int as uint32_t;
    // run the scheduler
    scheduler();
    // no tasks should have run, since neither task's desired time has elapsed
    // NOTE:
    // TASK_GYROPID desiredPeriod is  1000 microseconds
    // TASK_ACCEL   desiredPeriod is 10000 microseconds
    // 500 microseconds later
    // no tasks should run, since neither task's desired time has elapsed
    scheduler();
    // 500 microseconds later, TASK_GYROPID desiredPeriod has elapsed
    // TASK_GYROPID should now run
    scheduler();
    scheduler();
    // TASK_GYROPID should run again
    scheduler();
    // TASK_GYROPID and TASK_ACCEL desiredPeriods have elapsed
    // of the two TASK_GYROPID should run first
    scheduler();
    // and finally TASK_ACCEL should now run
    scheduler();
    panic!("Reached end of non-void function without returning");
}
