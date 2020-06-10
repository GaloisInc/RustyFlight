use ::libc;
extern "C" {
    pub type serialPort_s;
    #[no_mangle]
    fn mspFcProcessCommand(cmd: *mut mspPacket_t, reply: *mut mspPacket_t,
                           mspPostProcessFn: *mut mspPostProcessFnPtr)
     -> mspResult_e;
    #[no_mangle]
    fn sbufInit(sbuf: *mut sbuf_t, ptr: *mut uint8_t, end: *mut uint8_t)
     -> *mut sbuf_t;
    #[no_mangle]
    fn sbufWriteU8(dst: *mut sbuf_t, val: uint8_t);
    #[no_mangle]
    fn sbufWriteData(dst: *mut sbuf_t, data: *const libc::c_void,
                     len: libc::c_int);
    #[no_mangle]
    fn sbufReadU8(src: *mut sbuf_t) -> uint8_t;
    #[no_mangle]
    fn sbufReadData(dst: *mut sbuf_t, data: *mut libc::c_void,
                    len: libc::c_int);
    #[no_mangle]
    fn sbufBytesRemaining(buf: *mut sbuf_t) -> libc::c_int;
    #[no_mangle]
    fn sbufAdvance(buf: *mut sbuf_t, size: libc::c_int);
    #[no_mangle]
    fn sbufSwitchToReader(buf: *mut sbuf_t, base: *mut uint8_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type int16_t = __int16_t;
pub type uint8_t = __uint8_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct sbuf_s {
    pub ptr: *mut uint8_t,
    pub end: *mut uint8_t,
}
pub type sbuf_t = sbuf_s;
pub type mspResult_e = libc::c_int;
pub const MSP_RESULT_CMD_UNKNOWN: mspResult_e = -2;
pub const MSP_RESULT_NO_REPLY: mspResult_e = 0;
pub const MSP_RESULT_ERROR: mspResult_e = -1;
pub const MSP_RESULT_ACK: mspResult_e = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mspPacket_s {
    pub buf: sbuf_t,
    pub cmd: int16_t,
    pub flags: uint8_t,
    pub result: int16_t,
    pub direction: uint8_t,
}
pub type mspPacket_t = mspPacket_s;
pub type mspPostProcessFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut serialPort_s) -> ()>;
pub type mspResponseFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut uint8_t) -> ()>;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mspPackage_s {
    pub requestFrame: sbuf_t,
    pub requestBuffer: *mut uint8_t,
    pub responseBuffer: *mut uint8_t,
    pub requestPacket: *mut mspPacket_s,
    pub responsePacket: *mut mspPacket_s,
}
pub type mspPackage_t = mspPackage_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub union mspRxBuffer_u {
    pub smartPortMspRxBuffer: [uint8_t; 64],
    pub crsfMspRxBuffer: [uint8_t; 128],
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
pub type mspRxBuffer_t = mspRxBuffer_u;
#[derive(Copy, Clone)]
#[repr(C)]
pub union mspTxBuffer_u {
    pub smartPortMspTxBuffer: [uint8_t; 256],
    pub crsfMspTxBuffer: [uint8_t; 128],
}
pub type mspTxBuffer_t = mspTxBuffer_u;
pub const TELEMETRY_MSP_ERROR: C2RustUnnamed = 2;
pub const TELEMETRY_MSP_CRC_ERROR: C2RustUnnamed = 1;
pub const TELEMETRY_MSP_VER_MISMATCH: C2RustUnnamed = 0;
pub type C2RustUnnamed = libc::c_uint;
static mut checksum: uint8_t = 0 as libc::c_int as uint8_t;
static mut mspPackage: mspPackage_t =
    mspPackage_t{requestFrame:
                     sbuf_t{ptr: 0 as *const uint8_t as *mut uint8_t,
                            end: 0 as *const uint8_t as *mut uint8_t,},
                 requestBuffer: 0 as *const uint8_t as *mut uint8_t,
                 responseBuffer: 0 as *const uint8_t as *mut uint8_t,
                 requestPacket: 0 as *const mspPacket_s as *mut mspPacket_s,
                 responsePacket:
                     0 as *const mspPacket_s as *mut mspPacket_s,};
static mut mspRxBuffer: mspRxBuffer_t =
    mspRxBuffer_u{smartPortMspRxBuffer: [0; 64],};
static mut mspTxBuffer: mspTxBuffer_t =
    mspTxBuffer_u{smartPortMspTxBuffer: [0; 256],};
static mut mspRxPacket: mspPacket_t =
    mspPacket_t{buf:
                    sbuf_t{ptr: 0 as *const uint8_t as *mut uint8_t,
                           end: 0 as *const uint8_t as *mut uint8_t,},
                cmd: 0,
                flags: 0,
                result: 0,
                direction: 0,};
static mut mspTxPacket: mspPacket_t =
    mspPacket_t{buf:
                    sbuf_t{ptr: 0 as *const uint8_t as *mut uint8_t,
                           end: 0 as *const uint8_t as *mut uint8_t,},
                cmd: 0,
                flags: 0,
                result: 0,
                direction: 0,};
#[no_mangle]
pub unsafe extern "C" fn initSharedMsp() {
    mspPackage.requestBuffer =
        &mut mspRxBuffer as *mut mspRxBuffer_t as *mut uint8_t;
    mspPackage.requestPacket = &mut mspRxPacket;
    (*mspPackage.requestPacket).buf.ptr = mspPackage.requestBuffer;
    (*mspPackage.requestPacket).buf.end = mspPackage.requestBuffer;
    mspPackage.responseBuffer =
        &mut mspTxBuffer as *mut mspTxBuffer_t as *mut uint8_t;
    mspPackage.responsePacket = &mut mspTxPacket;
    (*mspPackage.responsePacket).buf.ptr = mspPackage.responseBuffer;
    (*mspPackage.responsePacket).buf.end = mspPackage.responseBuffer;
}
unsafe extern "C" fn processMspPacket() {
    (*mspPackage.responsePacket).cmd = 0 as libc::c_int as int16_t;
    (*mspPackage.responsePacket).result = 0 as libc::c_int as int16_t;
    (*mspPackage.responsePacket).buf.end = mspPackage.responseBuffer;
    let mut mspPostProcessFn: mspPostProcessFnPtr = None;
    if mspFcProcessCommand(mspPackage.requestPacket,
                           mspPackage.responsePacket, &mut mspPostProcessFn)
           as libc::c_int == MSP_RESULT_ERROR as libc::c_int {
        sbufWriteU8(&mut (*mspPackage.responsePacket).buf,
                    TELEMETRY_MSP_ERROR as libc::c_int as uint8_t);
    }
    if mspPostProcessFn.is_some() {
        mspPostProcessFn.expect("non-null function pointer")(0 as
                                                                 *mut serialPort_s);
    }
    sbufSwitchToReader(&mut (*mspPackage.responsePacket).buf,
                       mspPackage.responseBuffer);
}
#[no_mangle]
pub unsafe extern "C" fn sendMspErrorResponse(mut error: uint8_t,
                                              mut cmd: int16_t) {
    (*mspPackage.responsePacket).cmd = cmd;
    (*mspPackage.responsePacket).result = 0 as libc::c_int as int16_t;
    (*mspPackage.responsePacket).buf.end = mspPackage.responseBuffer;
    sbufWriteU8(&mut (*mspPackage.responsePacket).buf, error);
    (*mspPackage.responsePacket).result = -(10 as libc::c_int) as int16_t;
    sbufSwitchToReader(&mut (*mspPackage.responsePacket).buf,
                       mspPackage.responseBuffer);
}
#[no_mangle]
pub unsafe extern "C" fn handleMspFrame(mut frameStart: *mut uint8_t,
                                        mut frameLength: libc::c_int)
 -> bool {
    static mut mspStarted: uint8_t = 0 as libc::c_int as uint8_t;
    static mut lastSeq: uint8_t = 0 as libc::c_int as uint8_t;
    if sbufBytesRemaining(&mut (*mspPackage.responsePacket).buf) >
           0 as libc::c_int {
        mspStarted = 0 as libc::c_int as uint8_t
    }
    if mspStarted as libc::c_int == 0 as libc::c_int { initSharedMsp(); }
    let mut packet: *mut mspPacket_t = mspPackage.requestPacket;
    let mut frameBuf: *mut sbuf_t =
        sbufInit(&mut mspPackage.requestFrame, frameStart,
                 frameStart.offset(frameLength as uint8_t as libc::c_int as
                                       isize));
    let mut rxBuf: *mut sbuf_t = &mut (*mspPackage.requestPacket).buf;
    let header: uint8_t = sbufReadU8(frameBuf);
    let seqNumber: uint8_t =
        (header as libc::c_int & 0xf as libc::c_int) as uint8_t;
    let version: uint8_t =
        ((header as libc::c_int & (0x7 as libc::c_int) << 5 as libc::c_int) >>
             5 as libc::c_int) as uint8_t;
    if version as libc::c_int != 1 as libc::c_int {
        sendMspErrorResponse(TELEMETRY_MSP_VER_MISMATCH as libc::c_int as
                                 uint8_t, 0 as libc::c_int as int16_t);
        return 1 as libc::c_int != 0
    }
    if header as libc::c_int & (1 as libc::c_int) << 4 as libc::c_int != 0 {
        // first packet in sequence
        let mut mspPayloadSize: uint8_t = sbufReadU8(frameBuf);
        (*packet).cmd = sbufReadU8(frameBuf) as int16_t;
        (*packet).result = 0 as libc::c_int as int16_t;
        (*packet).buf.ptr = mspPackage.requestBuffer;
        (*packet).buf.end =
            mspPackage.requestBuffer.offset(mspPayloadSize as libc::c_int as
                                                isize);
        checksum =
            (mspPayloadSize as libc::c_int ^ (*packet).cmd as libc::c_int) as
                uint8_t;
        mspStarted = 1 as libc::c_int as uint8_t
    } else if mspStarted == 0 {
        // no start packet yet, throw this one away
        return 0 as libc::c_int != 0
    } else {
        if lastSeq as libc::c_int + 1 as libc::c_int & 0xf as libc::c_int !=
               seqNumber as libc::c_int {
            // packet loss detected!
            mspStarted = 0 as libc::c_int as uint8_t;
            return 0 as libc::c_int != 0
        }
    }
    let bufferBytesRemaining: uint8_t = sbufBytesRemaining(rxBuf) as uint8_t;
    let frameBytesRemaining: uint8_t =
        sbufBytesRemaining(frameBuf) as uint8_t;
    let vla = frameBytesRemaining as usize;
    let mut payload: Vec<uint8_t> = ::std::vec::from_elem(0, vla);
    if bufferBytesRemaining as libc::c_int >=
           frameBytesRemaining as libc::c_int {
        sbufReadData(frameBuf, payload.as_mut_ptr() as *mut libc::c_void,
                     frameBytesRemaining as libc::c_int);
        sbufAdvance(frameBuf, frameBytesRemaining as libc::c_int);
        sbufWriteData(rxBuf, payload.as_mut_ptr() as *const libc::c_void,
                      frameBytesRemaining as libc::c_int);
        lastSeq = seqNumber;
        return 0 as libc::c_int != 0
    } else {
        sbufReadData(frameBuf, payload.as_mut_ptr() as *mut libc::c_void,
                     bufferBytesRemaining as libc::c_int);
        sbufAdvance(frameBuf, bufferBytesRemaining as libc::c_int);
        sbufWriteData(rxBuf, payload.as_mut_ptr() as *const libc::c_void,
                      bufferBytesRemaining as libc::c_int);
        sbufSwitchToReader(rxBuf, mspPackage.requestBuffer);
        while sbufBytesRemaining(rxBuf) != 0 {
            checksum =
                (checksum as libc::c_int ^ sbufReadU8(rxBuf) as libc::c_int)
                    as uint8_t
        }
        if checksum as libc::c_int != *(*frameBuf).ptr as libc::c_int {
            mspStarted = 0 as libc::c_int as uint8_t;
            sendMspErrorResponse(TELEMETRY_MSP_CRC_ERROR as libc::c_int as
                                     uint8_t, (*packet).cmd);
            return 1 as libc::c_int != 0
        }
    }
    mspStarted = 0 as libc::c_int as uint8_t;
    sbufSwitchToReader(rxBuf, mspPackage.requestBuffer);
    processMspPacket();
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn sendMspReply(mut payloadSize: uint8_t,
                                      mut responseFn: mspResponseFnPtr)
 -> bool {
    static mut checksum_0: uint8_t = 0 as libc::c_int as uint8_t;
    static mut seq: uint8_t = 0 as libc::c_int as uint8_t;
    let vla = payloadSize as usize;
    let mut payloadOut: Vec<uint8_t> = ::std::vec::from_elem(0, vla);
    let mut payload: sbuf_t =
        sbuf_t{ptr: 0 as *const uint8_t as *mut uint8_t,
               end: 0 as *const uint8_t as *mut uint8_t,};
    let mut payloadBuf: *mut sbuf_t =
        sbufInit(&mut payload, payloadOut.as_mut_ptr(),
                 payloadOut.as_mut_ptr().offset(payloadSize as libc::c_int as
                                                    isize));
    let mut txBuf: *mut sbuf_t = &mut (*mspPackage.responsePacket).buf;
    // detect first reply packet
    if (*txBuf).ptr == mspPackage.responseBuffer {
        // header
        let fresh0 = seq;
        seq = seq.wrapping_add(1);
        let mut head: uint8_t =
            ((1 as libc::c_int) << 4 as libc::c_int |
                 fresh0 as libc::c_int & 0xf as libc::c_int) as uint8_t;
        if ((*mspPackage.responsePacket).result as libc::c_int) <
               0 as libc::c_int {
            head =
                (head as libc::c_int | (1 as libc::c_int) << 5 as libc::c_int)
                    as uint8_t
        }
        sbufWriteU8(payloadBuf, head);
        let mut size: uint8_t = sbufBytesRemaining(txBuf) as uint8_t;
        sbufWriteU8(payloadBuf, size);
    } else {
        // header
        let fresh1 = seq;
        seq = seq.wrapping_add(1);
        sbufWriteU8(payloadBuf,
                    (fresh1 as libc::c_int & 0xf as libc::c_int) as uint8_t);
    }
    let bufferBytesRemaining: uint8_t = sbufBytesRemaining(txBuf) as uint8_t;
    let payloadBytesRemaining: uint8_t =
        sbufBytesRemaining(payloadBuf) as uint8_t;
    let vla_0 = payloadBytesRemaining as usize;
    let mut frame: Vec<uint8_t> = ::std::vec::from_elem(0, vla_0);
    if bufferBytesRemaining as libc::c_int >=
           payloadBytesRemaining as libc::c_int {
        sbufReadData(txBuf, frame.as_mut_ptr() as *mut libc::c_void,
                     payloadBytesRemaining as libc::c_int);
        sbufAdvance(txBuf, payloadBytesRemaining as libc::c_int);
        sbufWriteData(payloadBuf, frame.as_mut_ptr() as *const libc::c_void,
                      payloadBytesRemaining as libc::c_int);
        responseFn.expect("non-null function pointer")(payloadOut.as_mut_ptr());
        return 1 as libc::c_int != 0
    } else {
        sbufReadData(txBuf, frame.as_mut_ptr() as *mut libc::c_void,
                     bufferBytesRemaining as libc::c_int);
        sbufAdvance(txBuf, bufferBytesRemaining as libc::c_int);
        sbufWriteData(payloadBuf, frame.as_mut_ptr() as *const libc::c_void,
                      bufferBytesRemaining as libc::c_int);
        sbufSwitchToReader(txBuf, mspPackage.responseBuffer);
        checksum_0 =
            (sbufBytesRemaining(txBuf) ^
                 (*mspPackage.responsePacket).cmd as libc::c_int) as uint8_t;
        while sbufBytesRemaining(txBuf) != 0 {
            checksum_0 =
                (checksum_0 as libc::c_int ^ sbufReadU8(txBuf) as libc::c_int)
                    as uint8_t
        }
        sbufWriteU8(payloadBuf, checksum_0);
        while sbufBytesRemaining(payloadBuf) > 1 as libc::c_int {
            sbufWriteU8(payloadBuf, 0 as libc::c_int as uint8_t);
        }
    }
    responseFn.expect("non-null function pointer")(payloadOut.as_mut_ptr());
    return 0 as libc::c_int != 0;
}
