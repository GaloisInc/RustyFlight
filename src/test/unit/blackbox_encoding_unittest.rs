#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
}
pub type int32_t = libc::c_int;
pub type uint8_t = libc::c_uchar;
pub type uint32_t = libc::c_uint;
static mut serialWritePos: libc::c_int = 0 as libc::c_int;
static mut serialReadPos: libc::c_int = 0 as libc::c_int;
static mut serialReadEnd: libc::c_int = 0 as libc::c_int;
static mut serialReadBuffer: [uint8_t; 256] = [0; 256];
static mut serialWriteBuffer: [uint8_t; 256] = [0; 256];
#[no_mangle]
pub static mut serialTestInstance: libc::c_int = 0;
#[no_mangle]
pub unsafe extern "C" fn serialWrite(mut instance: *mut libc::c_int,
                                     mut ch: uint8_t) {
    EXPECT_LT(serialWritePos,
              ::std::mem::size_of::<[uint8_t; 256]>() as libc::c_ulong);
    let fresh0 = serialWritePos;
    serialWritePos = serialWritePos + 1;
    serialWriteBuffer[fresh0 as usize] = ch;
}
#[no_mangle]
pub unsafe extern "C" fn serialWriteBuf(mut instance: *mut libc::c_int,
                                        mut data: *const uint8_t,
                                        mut count: libc::c_int) {
}
#[no_mangle]
pub unsafe extern "C" fn serialBeginWrite(mut instance: *mut libc::c_int) { }
#[no_mangle]
pub unsafe extern "C" fn serialEndWrite(mut instance: *mut libc::c_int) { }
#[no_mangle]
pub unsafe extern "C" fn serialRxBytesWaiting(mut instance:
                                                  *const libc::c_int)
 -> uint32_t {
    EXPECT_GE(serialReadEnd, serialReadPos);
    let mut ret: libc::c_int = serialReadEnd - serialReadPos;
    if ret >= 0 as libc::c_int { return ret as uint32_t }
    return 0 as libc::c_int as uint32_t;
}
#[no_mangle]
pub unsafe extern "C" fn serialRead(mut instance: *mut libc::c_int)
 -> uint8_t {
    EXPECT_LT(serialReadPos, serialReadEnd);
    let fresh1 = serialReadPos;
    serialReadPos = serialReadPos + 1;
    let ch: uint8_t = serialReadBuffer[fresh1 as usize];
    return ch;
}
#[no_mangle]
pub unsafe extern "C" fn serialTxBytesFree(mut instance: *const libc::c_int)
 -> uint32_t {
    return (256 as libc::c_int - serialWritePos) as uint32_t;
}
#[no_mangle]
pub unsafe extern "C" fn isSerialTransmitBufferEmpty(mut instance:
                                                         *const libc::c_int)
 -> libc::c_int {
    panic!("Reached end of non-void function without returning");
}
#[no_mangle]
pub unsafe extern "C" fn serialTestResetBuffers() {
    memset(&mut serialReadBuffer as *mut [uint8_t; 256] as *mut libc::c_void,
           0 as libc::c_int,
           ::std::mem::size_of::<[uint8_t; 256]>() as libc::c_ulong);
    serialReadPos = 0 as libc::c_int;
    serialReadEnd = 0 as libc::c_int;
    memset(&mut serialWriteBuffer as *mut [uint8_t; 256] as *mut libc::c_void,
           0 as libc::c_int,
           ::std::mem::size_of::<[uint8_t; 256]>() as libc::c_ulong);
    serialWritePos = 0 as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn TEST(mut BlackboxEncodingTest: libc::c_int,
                              mut TestWriteUnsignedVB: libc::c_int)
 -> libc::c_int {
    serialTestResetBuffers();
    blackboxWriteUnsignedVB(0 as libc::c_int);
    EXPECT_EQ(0 as libc::c_int,
              serialWriteBuffer[0 as libc::c_int as usize] as libc::c_int);
    blackboxWriteUnsignedVB(128 as libc::c_int);
    EXPECT_EQ(0x80 as libc::c_int,
              serialWriteBuffer[1 as libc::c_int as usize] as libc::c_int);
    EXPECT_EQ(1 as libc::c_int,
              serialWriteBuffer[2 as libc::c_int as usize] as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
// 2 bits per field  ss11 2233,
// ensure next byte has not been written
// 00010000
// ensure next byte has not been written
// 00010101
// ensure next byte has not been written
// 00111111
// ensure next byte has not been written
// 00101010
// ensure next byte has not been written
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_0(mut BlackboxTest: libc::c_int,
                                mut TestWriteTag2_3SVariable_BITS554:
                                    libc::c_int) -> libc::c_int {
    serialTestResetBuffers();
    let mut buf: *mut uint8_t =
        &mut *serialWriteBuffer.as_mut_ptr().offset(0 as libc::c_int as isize)
            as *mut uint8_t;
    let mut selector: libc::c_int = 0;
    let mut v: [int32_t; 3] = [0; 3];
    // 554 bits per field  ss11 1112 2222 3333
    // 5 bits per field [-16, 15], 4 bits per field [-8, 7]
    v[0 as libc::c_int as usize] = 15 as libc::c_int; // 0101 1110
    v[1 as libc::c_int as usize] = 15 as libc::c_int; // 1111 0111
    v[2 as libc::c_int as usize] =
        7 as libc::c_int; // ensure next byte has not been written
    selector = blackboxWriteTag2_3SVariable(v.as_mut_ptr()); // 0110 0001
    EXPECT_EQ(1 as libc::c_int, selector); // 0000 1000
    EXPECT_EQ(0x5e as libc::c_int,
              *buf.offset(0 as libc::c_int as isize) as
                  libc::c_int); // ensure next byte has not been written
    EXPECT_EQ(0xf7 as libc::c_int,
              *buf.offset(1 as libc::c_int as isize) as
                  libc::c_int); // 0100 1110
    EXPECT_EQ(0 as libc::c_int,
              *buf.offset(2 as libc::c_int as isize) as
                  libc::c_int); // 1000 0101
    buf =
        buf.offset(2 as libc::c_int as
                       isize); // ensure next byte has not been written
    v[0 as libc::c_int as usize] = -(16 as libc::c_int);
    v[1 as libc::c_int as usize] = -(16 as libc::c_int);
    v[2 as libc::c_int as usize] = -(8 as libc::c_int);
    selector = blackboxWriteTag2_3SVariable(v.as_mut_ptr());
    EXPECT_EQ(1 as libc::c_int, selector);
    EXPECT_EQ(0x61 as libc::c_int,
              *buf.offset(0 as libc::c_int as isize) as libc::c_int);
    EXPECT_EQ(0x8 as libc::c_int,
              *buf.offset(1 as libc::c_int as isize) as libc::c_int);
    EXPECT_EQ(0 as libc::c_int,
              *buf.offset(2 as libc::c_int as isize) as libc::c_int);
    buf = buf.offset(2 as libc::c_int as isize);
    v[0 as libc::c_int as usize] = 7 as libc::c_int;
    v[1 as libc::c_int as usize] = 8 as libc::c_int;
    v[2 as libc::c_int as usize] = 5 as libc::c_int;
    selector = blackboxWriteTag2_3SVariable(v.as_mut_ptr());
    EXPECT_EQ(1 as libc::c_int, selector);
    EXPECT_EQ(0x4e as libc::c_int,
              *buf.offset(0 as libc::c_int as isize) as libc::c_int);
    EXPECT_EQ(0x85 as libc::c_int,
              *buf.offset(1 as libc::c_int as isize) as libc::c_int);
    EXPECT_EQ(0 as libc::c_int,
              *buf.offset(2 as libc::c_int as isize) as libc::c_int);
    buf = buf.offset(2 as libc::c_int as isize);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_1(mut BlackboxTest: libc::c_int,
                                mut TestWriteTag2_3SVariable_BITS887:
                                    libc::c_int) -> libc::c_int {
    serialTestResetBuffers();
    let mut buf: *mut uint8_t =
        &mut *serialWriteBuffer.as_mut_ptr().offset(0 as libc::c_int as isize)
            as *mut uint8_t;
    let mut selector: libc::c_int = 0;
    let mut v: [int32_t; 3] = [0; 3];
    // 877 bits per field  ss11 1111 1122 2222 2333 3333
    // 8 bits per field [-128, 127], 7 bits per field [-64, 63]
    v[0 as libc::c_int as usize] = 127 as libc::c_int; // 1001 1111
    v[1 as libc::c_int as usize] = 63 as libc::c_int; // 1101 1111
    v[2 as libc::c_int as usize] = 63 as libc::c_int; // 1011 1111
    selector =
        blackboxWriteTag2_3SVariable(v.as_mut_ptr()); // ensure next byte has not been written
    EXPECT_EQ(2 as libc::c_int, selector); // 1010 0000
    EXPECT_EQ(0x9f as libc::c_int,
              *buf.offset(0 as libc::c_int as isize) as
                  libc::c_int); // 0010 0000
    EXPECT_EQ(0xdf as libc::c_int,
              *buf.offset(1 as libc::c_int as isize) as
                  libc::c_int); // 0100 0000
    EXPECT_EQ(0xbf as libc::c_int,
              *buf.offset(2 as libc::c_int as isize) as
                  libc::c_int); // ensure next byte has not been written
    EXPECT_EQ(0 as libc::c_int,
              *buf.offset(3 as libc::c_int as isize) as libc::c_int);
    buf = buf.offset(3 as libc::c_int as isize);
    v[0 as libc::c_int as usize] = -(128 as libc::c_int);
    v[1 as libc::c_int as usize] = -(64 as libc::c_int);
    v[2 as libc::c_int as usize] = -(64 as libc::c_int);
    selector = blackboxWriteTag2_3SVariable(v.as_mut_ptr());
    EXPECT_EQ(2 as libc::c_int, selector);
    EXPECT_EQ(0xa0 as libc::c_int,
              *buf.offset(0 as libc::c_int as isize) as libc::c_int);
    EXPECT_EQ(0x20 as libc::c_int,
              *buf.offset(1 as libc::c_int as isize) as libc::c_int);
    EXPECT_EQ(0x40 as libc::c_int,
              *buf.offset(2 as libc::c_int as isize) as libc::c_int);
    EXPECT_EQ(0 as libc::c_int,
              *buf.offset(3 as libc::c_int as isize) as libc::c_int);
    buf = buf.offset(3 as libc::c_int as isize);
    panic!("Reached end of non-void function without returning");
}
// STUBS
