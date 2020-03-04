#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(extern_types, register_tool)]
extern "C" {
    pub type fp_vector;
}
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
pub unsafe extern "C" fn TEST(mut MathsUnittest: libc::c_int,
                              mut TestScaleRange: libc::c_int)
 -> libc::c_int {
    // Within bounds
    EXPECT_EQ(scaleRange(0 as libc::c_int, 0 as libc::c_int,
                         10 as libc::c_int, 0 as libc::c_int,
                         100 as libc::c_int), 0 as libc::c_int);
    EXPECT_EQ(scaleRange(10 as libc::c_int, 0 as libc::c_int,
                         10 as libc::c_int, 0 as libc::c_int,
                         100 as libc::c_int), 100 as libc::c_int);
    EXPECT_EQ(scaleRange(0 as libc::c_int, 0 as libc::c_int,
                         100 as libc::c_int, 0 as libc::c_int,
                         10 as libc::c_int), 0 as libc::c_int);
    EXPECT_EQ(scaleRange(100 as libc::c_int, 0 as libc::c_int,
                         100 as libc::c_int, 0 as libc::c_int,
                         10 as libc::c_int), 10 as libc::c_int);
    // Scale up
    EXPECT_EQ(scaleRange(1 as libc::c_int, 0 as libc::c_int,
                         10 as libc::c_int, 0 as libc::c_int,
                         100 as libc::c_int), 10 as libc::c_int);
    EXPECT_EQ(scaleRange(2 as libc::c_int, 0 as libc::c_int,
                         10 as libc::c_int, 0 as libc::c_int,
                         100 as libc::c_int), 20 as libc::c_int);
    EXPECT_EQ(scaleRange(5 as libc::c_int, 0 as libc::c_int,
                         10 as libc::c_int, 0 as libc::c_int,
                         100 as libc::c_int), 50 as libc::c_int);
    // Scale down
    EXPECT_EQ(scaleRange(10 as libc::c_int, 0 as libc::c_int,
                         100 as libc::c_int, 0 as libc::c_int,
                         10 as libc::c_int), 1 as libc::c_int);
    EXPECT_EQ(scaleRange(20 as libc::c_int, 0 as libc::c_int,
                         100 as libc::c_int, 0 as libc::c_int,
                         10 as libc::c_int), 2 as libc::c_int);
    EXPECT_EQ(scaleRange(50 as libc::c_int, 0 as libc::c_int,
                         100 as libc::c_int, 0 as libc::c_int,
                         10 as libc::c_int), 5 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
// Within bounds
// Scale up
// Scale down
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_0(mut MathsUnittest: libc::c_int,
                                mut TestScaleRangeNegativePositive:
                                    libc::c_int) -> libc::c_int {
    // Within bounds
    EXPECT_EQ(scaleRange(0 as libc::c_int, -(10 as libc::c_int),
                         0 as libc::c_int, 0 as libc::c_int,
                         100 as libc::c_int), 100 as libc::c_int);
    EXPECT_EQ(scaleRange(-(10 as libc::c_int), -(10 as libc::c_int),
                         0 as libc::c_int, 0 as libc::c_int,
                         100 as libc::c_int), 0 as libc::c_int);
    EXPECT_EQ(scaleRange(0 as libc::c_int, -(100 as libc::c_int),
                         0 as libc::c_int, 0 as libc::c_int,
                         10 as libc::c_int), 10 as libc::c_int);
    EXPECT_EQ(scaleRange(-(100 as libc::c_int), -(100 as libc::c_int),
                         0 as libc::c_int, 0 as libc::c_int,
                         10 as libc::c_int), 0 as libc::c_int);
    // Scale up
    EXPECT_EQ(scaleRange(-(1 as libc::c_int), -(10 as libc::c_int),
                         0 as libc::c_int, 0 as libc::c_int,
                         100 as libc::c_int), 90 as libc::c_int);
    EXPECT_EQ(scaleRange(-(2 as libc::c_int), -(10 as libc::c_int),
                         0 as libc::c_int, 0 as libc::c_int,
                         100 as libc::c_int), 80 as libc::c_int);
    EXPECT_EQ(scaleRange(-(5 as libc::c_int), -(10 as libc::c_int),
                         0 as libc::c_int, 0 as libc::c_int,
                         100 as libc::c_int), 50 as libc::c_int);
    // Scale down
    EXPECT_EQ(scaleRange(-(10 as libc::c_int), -(100 as libc::c_int),
                         0 as libc::c_int, 0 as libc::c_int,
                         10 as libc::c_int), 9 as libc::c_int);
    EXPECT_EQ(scaleRange(-(20 as libc::c_int), -(100 as libc::c_int),
                         0 as libc::c_int, 0 as libc::c_int,
                         10 as libc::c_int), 8 as libc::c_int);
    EXPECT_EQ(scaleRange(-(50 as libc::c_int), -(100 as libc::c_int),
                         0 as libc::c_int, 0 as libc::c_int,
                         10 as libc::c_int), 5 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_1(mut MathsUnittest: libc::c_int,
                                mut TestScaleRangeReverse: libc::c_int)
 -> libc::c_int {
    // Within bounds
    EXPECT_EQ(scaleRange(0 as libc::c_int, 0 as libc::c_int,
                         10 as libc::c_int, 100 as libc::c_int,
                         0 as libc::c_int), 100 as libc::c_int);
    EXPECT_EQ(scaleRange(10 as libc::c_int, 0 as libc::c_int,
                         10 as libc::c_int, 100 as libc::c_int,
                         0 as libc::c_int), 0 as libc::c_int);
    EXPECT_EQ(scaleRange(0 as libc::c_int, 0 as libc::c_int,
                         100 as libc::c_int, 10 as libc::c_int,
                         0 as libc::c_int), 10 as libc::c_int);
    EXPECT_EQ(scaleRange(100 as libc::c_int, 0 as libc::c_int,
                         100 as libc::c_int, 10 as libc::c_int,
                         0 as libc::c_int), 0 as libc::c_int);
    // Scale up
    EXPECT_EQ(scaleRange(1 as libc::c_int, 0 as libc::c_int,
                         10 as libc::c_int, 100 as libc::c_int,
                         0 as libc::c_int), 90 as libc::c_int);
    EXPECT_EQ(scaleRange(2 as libc::c_int, 0 as libc::c_int,
                         10 as libc::c_int, 100 as libc::c_int,
                         0 as libc::c_int), 80 as libc::c_int);
    EXPECT_EQ(scaleRange(5 as libc::c_int, 0 as libc::c_int,
                         10 as libc::c_int, 100 as libc::c_int,
                         0 as libc::c_int), 50 as libc::c_int);
    // Scale down
    EXPECT_EQ(scaleRange(10 as libc::c_int, 0 as libc::c_int,
                         100 as libc::c_int, 10 as libc::c_int,
                         0 as libc::c_int), 9 as libc::c_int);
    EXPECT_EQ(scaleRange(20 as libc::c_int, 0 as libc::c_int,
                         100 as libc::c_int, 10 as libc::c_int,
                         0 as libc::c_int), 8 as libc::c_int);
    EXPECT_EQ(scaleRange(50 as libc::c_int, 0 as libc::c_int,
                         100 as libc::c_int, 10 as libc::c_int,
                         0 as libc::c_int), 5 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_2(mut MathsUnittest: libc::c_int,
                                mut TestConstrain: libc::c_int)
 -> libc::c_int {
    // Within bounds
    EXPECT_EQ(constrain(0 as libc::c_int, 0 as libc::c_int, 0 as libc::c_int),
              0 as libc::c_int);
    EXPECT_EQ(constrain(1 as libc::c_int, 1 as libc::c_int, 1 as libc::c_int),
              1 as libc::c_int);
    EXPECT_EQ(constrain(1 as libc::c_int, 0 as libc::c_int, 2 as libc::c_int),
              1 as libc::c_int);
    // Equal to bottom bound.
    EXPECT_EQ(constrain(1 as libc::c_int, 1 as libc::c_int, 2 as libc::c_int),
              1 as libc::c_int);
    // Equal to top bound.
    EXPECT_EQ(constrain(1 as libc::c_int, 0 as libc::c_int, 1 as libc::c_int),
              1 as libc::c_int);
    // Equal to both bottom and top bound.
    EXPECT_EQ(constrain(1 as libc::c_int, 1 as libc::c_int, 1 as libc::c_int),
              1 as libc::c_int);
    // Above top bound.
    EXPECT_EQ(constrain(2 as libc::c_int, 0 as libc::c_int, 1 as libc::c_int),
              1 as libc::c_int);
    // Below bottom bound.
    EXPECT_EQ(constrain(0 as libc::c_int, 1 as libc::c_int, 2 as libc::c_int),
              1 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_3(mut MathsUnittest: libc::c_int,
                                mut TestConstrainNegatives: libc::c_int)
 -> libc::c_int {
    // Within bounds.
    EXPECT_EQ(constrain(-(1 as libc::c_int), -(1 as libc::c_int),
                        -(1 as libc::c_int)), -(1 as libc::c_int));
    EXPECT_EQ(constrain(-(1 as libc::c_int), -(2 as libc::c_int),
                        0 as libc::c_int), -(1 as libc::c_int));
    // Equal to bottom bound.
    EXPECT_EQ(constrain(-(1 as libc::c_int), -(1 as libc::c_int),
                        0 as libc::c_int), -(1 as libc::c_int));
    // Equal to top bound.
    EXPECT_EQ(constrain(-(1 as libc::c_int), -(2 as libc::c_int),
                        -(1 as libc::c_int)), -(1 as libc::c_int));
    // Equal to both bottom and top bound.
    EXPECT_EQ(constrain(-(1 as libc::c_int), -(1 as libc::c_int),
                        -(1 as libc::c_int)), -(1 as libc::c_int));
    // Above top bound.
    EXPECT_EQ(constrain(-(1 as libc::c_int), -(3 as libc::c_int),
                        -(2 as libc::c_int)), -(2 as libc::c_int));
    // Below bottom bound.
    EXPECT_EQ(constrain(-(3 as libc::c_int), -(2 as libc::c_int),
                        -(1 as libc::c_int)), -(2 as libc::c_int));
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_4(mut MathsUnittest: libc::c_int,
                                mut TestConstrainf: libc::c_int)
 -> libc::c_int {
    // Within bounds.
    EXPECT_FLOAT_EQ(constrainf(1.0f32 as libc::c_double,
                               0.0f32 as libc::c_double,
                               2.0f32 as libc::c_double),
                    1.0f32 as libc::c_double);
    // Equal to bottom bound.
    EXPECT_FLOAT_EQ(constrainf(1.0f32 as libc::c_double,
                               1.0f32 as libc::c_double,
                               2.0f32 as libc::c_double),
                    1.0f32 as libc::c_double);
    // Equal to top bound.
    EXPECT_FLOAT_EQ(constrainf(1.0f32 as libc::c_double,
                               0.0f32 as libc::c_double,
                               1.0f32 as libc::c_double),
                    1.0f32 as libc::c_double);
    // Equal to both bottom and top bound.
    EXPECT_FLOAT_EQ(constrainf(1.0f32 as libc::c_double,
                               1.0f32 as libc::c_double,
                               1.0f32 as libc::c_double),
                    1.0f32 as libc::c_double);
    // Above top bound.
    EXPECT_FLOAT_EQ(constrainf(2.0f32 as libc::c_double,
                               0.0f32 as libc::c_double,
                               1.0f32 as libc::c_double),
                    1.0f32 as libc::c_double);
    // Below bottom bound.
    EXPECT_FLOAT_EQ(constrainf(0 as libc::c_int, 1.0f32 as libc::c_double,
                               2.0f32 as libc::c_double),
                    1.0f32 as libc::c_double);
    // Above bouth bounds.
    EXPECT_FLOAT_EQ(constrainf(2.0f32 as libc::c_double,
                               0.0f32 as libc::c_double,
                               1.0f32 as libc::c_double),
                    1.0f32 as libc::c_double);
    // Below bouth bounds.
    EXPECT_FLOAT_EQ(constrainf(0 as libc::c_int, 1.0f32 as libc::c_double,
                               2.0f32 as libc::c_double),
                    1.0f32 as libc::c_double);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_5(mut MathsUnittest: libc::c_int,
                                mut TestDegreesToRadians: libc::c_int)
 -> libc::c_int {
    EXPECT_FLOAT_EQ(degreesToRadians(0 as libc::c_int),
                    0.0f32 as libc::c_double);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_6(mut MathsUnittest: libc::c_int,
                                mut TestApplyDeadband: libc::c_int)
 -> libc::c_int {
    EXPECT_EQ(applyDeadband(0 as libc::c_int, 0 as libc::c_int),
              0 as libc::c_int);
    EXPECT_EQ(applyDeadband(1 as libc::c_int, 0 as libc::c_int),
              1 as libc::c_int);
    EXPECT_EQ(applyDeadband(-(1 as libc::c_int), 0 as libc::c_int),
              -(1 as libc::c_int));
    EXPECT_EQ(applyDeadband(0 as libc::c_int, 10 as libc::c_int),
              0 as libc::c_int);
    EXPECT_EQ(applyDeadband(1 as libc::c_int, 10 as libc::c_int),
              0 as libc::c_int);
    EXPECT_EQ(applyDeadband(10 as libc::c_int, 10 as libc::c_int),
              0 as libc::c_int);
    EXPECT_EQ(applyDeadband(11 as libc::c_int, 10 as libc::c_int),
              1 as libc::c_int);
    EXPECT_EQ(applyDeadband(-(11 as libc::c_int), 10 as libc::c_int),
              -(1 as libc::c_int));
    panic!("Reached end of non-void function without returning");
}
#[no_mangle]
pub unsafe extern "C" fn expectVectorsAreEqual(mut a: *mut fp_vector,
                                               mut b: *mut fp_vector,
                                               mut absTol: libc::c_float) {
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_7(mut MathsUnittest: libc::c_int,
                                mut TestRotateVectorWithNoAngle: libc::c_int)
 -> libc::c_int {
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_8(mut MathsUnittest: libc::c_int,
                                mut TestRotateVectorAroundAxis: libc::c_int)
 -> libc::c_int {
    panic!("Reached end of non-void function without returning");
    // Rotate a vector <1, 0, 0> around an each axis x y and z.
}
