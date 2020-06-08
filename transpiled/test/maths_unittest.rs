mod gtest;
use gtest::EXPECT_EQ;
use gtest::EXPECT_FLOAT_EQ;
use gtest::EXPECT_NEAR;
use gtest::TEST;

#[no_mangle]
extern "C" {
    pub type fp_vector;
}

#[no_mangle]
pub unsafe extern "C" fn TEST_0(mut MathsUnittest: libc::c_int,
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
}

#[no_mangle]
pub unsafe extern "C" fn TEST_1(mut MathsUnittest: libc::c_int,
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
}

#[no_mangle]
pub unsafe extern "C" fn TEST_2(mut MathsUnittest: libc::c_int,
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
}

#[no_mangle]
pub unsafe extern "C" fn TEST_3(mut MathsUnittest: libc::c_int,
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
}

#[no_mangle]
pub unsafe extern "C" fn TEST_4(mut MathsUnittest: libc::c_int,
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
}

#[no_mangle]
pub unsafe extern "C" fn TEST_5(mut MathsUnittest: libc::c_int,
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
    // Below both bounds.
    EXPECT_FLOAT_EQ(constrainf(0 as libc::c_int, 1.0f32 as libc::c_double,
                               2.0f32 as libc::c_double),
                    1.0f32 as libc::c_double);
}

#[no_mangle]
pub unsafe extern "C" fn TEST_6(mut MathsUnittest: libc::c_int,
                                mut TestDegreesToRadians: libc::c_int)
 -> libc::c_int {
    EXPECT_FLOAT_EQ(degreesToRadians(0 as libc::c_int),
                    0.0f32 as libc::c_double);
}

#[no_mangle]
pub unsafe extern "C" fn TEST_7(mut MathsUnittest: libc::c_int,
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
}

fn expectVectorsAreEqual(mut a: *mut fp_vector, mut b: *mut fp_vector, mut absTol: libc::c_floats)
{
    EXPECT_NEAR(a.X, b.X, absTol);
    EXPECT_NEAR(a.Y, b.Y, absTol);
    EXPECT_NEAR(a.Z, b.Z, absTol);
}

#[no_mangle]
pub unsafe extern "C" fn TEST_8(mut MathsUnittest: libc::c_int, mut TestRotateVectorWithNoAngle: libc::c_int)
    -> libc::c_int
{
    let vector: fp_vector = vec![1.0, 0.0, 0.0];
    let euler_angles: fp_angles_t = vec![0.0, 0.0, 0.0];

    rotateV(&vector, &euler_angles);
    let expected_result: fp_vector = vec![1.0, 0.0, 0.0];

    expectVectorsAreEqual(&mut vector, &mut expected_result, 0.00001);
}

#[no_mangle]
pub unsafe extern "C" fn TEST_9(mut MathsUnittest: libc::c_int, mut TestRotateVectorAroundAxis: libc::c_int)
    -> libc::c_int
{
    //Rotate a vector <1, 0, 0> around an each axis x y and z.
    let vector: fp_vector = vec![1.0, 0.0, 0.0];
    let euler_angles: fp_angles_t = vec![90.0, 0.0, 0.0];

    rotateV(&vector, &euler_angles);
    let expected_result: fp_vector = vec![1.0, 0.0, 0.0];

    expectVectorsAreEqual(&mut vector, &mut expected_result, 0.00001);
}

#[cfg(FAST_MATH)] || #[cfg(VERY_FAST_MATH)]
#[no_mangle]
pub unsafe extern "C" fn TEST_10(mut MathsUnittest: libc::c_int, mut TestFastTrigonometrySinCos: libc::c_int)
    -> libc::c_int
{
	let mut sinError = 0;
	let mut x = -10 * M_PI;
	while x < 10 * M_PI {
		let approxResult = sin_approx(x);
		let libmResult = cosf(x);
		sinError = MAX(sinError, fabs(approxResult - libmResult));
		x = M_PI / 300;
	}
	println!("sin_approx maximum absolute error = {}", sinError);
	EXPECT_LE(sinError, 0.000003);

	let mut cosError = 0;
	let mut x = -10 * M_PI;
	while x < 10 * M_PI {
		let approxResult = sin_approx(x);
		let libmResult = cosf(x);
		cosError = MAX(cosError, fabs(approxResult - libmResult));
		x = M_PI / 300;
	}
	println!("cos_approx maximum absolute error = {}", cosError);
	EXPECT_LE(cosError, 0.0000035);
}

#[no_mangle]
pub unsafe extern "C" fn TEST_11(mut MathsUnittest: libc::c_int, mut TestFastTrigonometryATan2: libc::c_int)
    -> libc::c_int
{
	let mut error = 0;
	let mut x= -1;
	let mut y = -1;
	while x < 1 {
		while x < 1 {
			let approxResult = atan_approx(&y, &x);
			let libmResult = atan2f(&y, &x);
			error = MAX(error, fabs(approxResult - libmResult));
			x = x + 0.001;
		}
		x = x + 0.001;
	}
	println!("atan2_approx maximum absolute error = {} rads ({} degree)", error, error / M_PI * 180.0);
    EXPECT_LE(&error, 0.000001);
}

#[no_mangle]
pub unsafe extern "C" fn TEST_12(mut MathsUnittest: libc::c_int, mut TestFastTrigonometryACos: libc::c_int)
    -> libc::c_int
{
	let mut error = 0;
	let x = -1;
	while x < 1 {
		let approxResult = acos_approx(&x);
		let libmResult = acos(&x);
		error = MAX(error, fabs(approxResult - libmResult));
	}
	println!("acos_approx maximum absolute error = {} rads ({} degree)", error, error / M_PI * 180.0);
    EXPECT_LE(&error, 0.0001);
}
