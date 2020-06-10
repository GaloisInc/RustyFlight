use ::libc;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
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
    // Maximum page size of all supported SPI flash devices.
// Used to detect flashfs allocation size being too small.
    // Count of the number of erasable blocks on the device
    // In bytes
    // This is just pagesPerSector * pageSize
    // This is just sectorSize * sectors
    #[no_mangle]
    fn flashGetGeometry() -> *const flashGeometry_t;
    #[no_mangle]
    fn flashFlush();
    #[no_mangle]
    fn flashReadBytes(address: uint32_t, buffer: *mut uint8_t,
                      length: libc::c_int) -> libc::c_int;
    #[no_mangle]
    fn flashPageProgramFinish();
    #[no_mangle]
    fn flashPageProgramContinue(data: *const uint8_t, length: libc::c_int);
    #[no_mangle]
    fn flashPageProgramBegin(address: uint32_t);
    #[no_mangle]
    fn flashIsReady() -> bool;
    #[no_mangle]
    fn flashEraseSector(address: uint32_t);
    #[no_mangle]
    fn flashEraseCompletely();
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type flashType_e = libc::c_uint;
pub const FLASH_TYPE_NAND: flashType_e = 1;
pub const FLASH_TYPE_NOR: flashType_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct flashGeometry_s {
    pub sectors: uint16_t,
    pub pageSize: uint16_t,
    pub sectorSize: uint32_t,
    pub totalSize: uint32_t,
    pub pagesPerSector: uint16_t,
    pub flashType: flashType_e,
}
pub type flashGeometry_t = flashGeometry_s;
/* We can choose whatever power of 2 size we like, which determines how much wastage of free space we'll have
         * at the end of the last written data. But smaller blocksizes will require more searching.
         */
pub const FREE_BLOCK_SIZE: C2RustUnnamed_0 = 2048;
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed {
    pub bytes: [uint8_t; 16],
    pub ints: [uint32_t; 4],
}
// XXX This can't be smaller than page size for underlying flash device.
/* We don't expect valid data to ever contain this many consecutive uint32_t's of all 1 bits: */
pub const FREE_BLOCK_TEST_SIZE_INTS: C2RustUnnamed_0 = 4;
// i.e. 16 bytes
pub const FREE_BLOCK_TEST_SIZE_BYTES: C2RustUnnamed_0 = 16;
pub type assert_failed_FREE_BLOCK_SIZE_too_small = [libc::c_char; 1];
pub type C2RustUnnamed_0 = libc::c_uint;
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
/* *
 * This provides a stream interface to a flash chip if one is present.
 *
 * On statup, call flashfsInit() after initialising the flash chip in order to init the filesystem. This will
 * result in the file pointer being pointed at the first free block found, or at the end of the device if the
 * flash chip is full.
 *
 * Note that bits can only be set to 0 when writing, not back to 1 from 0. You must erase sectors in order
 * to bring bits back to 1 again.
 *
 * In future, we can add support for multiple different flash chips by adding a flash device driver vtable
 * and make calls through that, at the moment flashfs just calls m25p16_* routines explicitly.
 */
static mut flashWriteBuffer: [uint8_t; 128] = [0; 128];
/* The position of our head and tail in the circular flash write buffer.
 *
 * The head is the index that a byte would be inserted into on writing, while the tail is the index of the
 * oldest byte that has yet to be written to flash.
 *
 * When the circular buffer is empty, head == tail
 */
static mut bufferHead: uint8_t = 0 as libc::c_int as uint8_t;
static mut bufferTail: uint8_t = 0 as libc::c_int as uint8_t;
// The position of the buffer's tail in the overall flash address space:
static mut tailAddress: uint32_t = 0 as libc::c_int as uint32_t;
unsafe extern "C" fn flashfsClearBuffer() {
    bufferHead = 0 as libc::c_int as uint8_t;
    bufferTail = bufferHead;
}
unsafe extern "C" fn flashfsBufferIsEmpty() -> bool {
    return bufferTail as libc::c_int == bufferHead as libc::c_int;
}
unsafe extern "C" fn flashfsSetTailAddress(mut address: uint32_t) {
    tailAddress = address;
}
#[no_mangle]
pub unsafe extern "C" fn flashfsEraseCompletely() {
    flashEraseCompletely();
    flashfsClearBuffer();
    flashfsSetTailAddress(0 as libc::c_int as uint32_t);
}
/* *
 * Start and end must lie on sector boundaries, or they will be rounded out to sector boundaries such that
 * all the bytes in the range [start...end) are erased.
 */
#[no_mangle]
pub unsafe extern "C" fn flashfsEraseRange(mut start: uint32_t,
                                           mut end: uint32_t) {
    let mut geometry: *const flashGeometry_t = flashGetGeometry();
    if (*geometry).sectorSize <= 0 as libc::c_int as libc::c_uint { return }
    // Round the start down to a sector boundary
    let mut startSector: libc::c_int =
        start.wrapping_div((*geometry).sectorSize) as libc::c_int;
    // And the end upward
    let mut endSector: libc::c_int =
        end.wrapping_div((*geometry).sectorSize) as libc::c_int;
    let mut endRemainder: libc::c_int =
        end.wrapping_rem((*geometry).sectorSize) as libc::c_int;
    if endRemainder > 0 as libc::c_int { endSector += 1 }
    let mut i: libc::c_int = startSector;
    while i < endSector {
        flashEraseSector((i as
                              libc::c_uint).wrapping_mul((*geometry).sectorSize));
        i += 1
    };
}
/* *
 * Return true if the flash is not currently occupied with an operation.
 */
#[no_mangle]
pub unsafe extern "C" fn flashfsIsReady() -> bool { return flashIsReady(); }
#[no_mangle]
pub unsafe extern "C" fn flashfsIsSupported() -> bool {
    return flashfsGetSize() > 0 as libc::c_int as libc::c_uint;
}
#[no_mangle]
pub unsafe extern "C" fn flashfsGetSize() -> uint32_t {
    return (*flashGetGeometry()).totalSize;
}
unsafe extern "C" fn flashfsTransmitBufferUsed() -> uint32_t {
    if bufferHead as libc::c_int >= bufferTail as libc::c_int {
        return (bufferHead as libc::c_int - bufferTail as libc::c_int) as
                   uint32_t
    }
    return (128 as libc::c_int - bufferTail as libc::c_int +
                bufferHead as libc::c_int) as uint32_t;
}
/* *
 * Get the size of the largest single write that flashfs could ever accept without blocking or data loss.
 */
#[no_mangle]
pub unsafe extern "C" fn flashfsGetWriteBufferSize() -> uint32_t {
    return (128 as libc::c_int - 1 as libc::c_int) as uint32_t;
}
/* *
 * Get the number of bytes that can currently be written to flashfs without any blocking or data loss.
 */
#[no_mangle]
pub unsafe extern "C" fn flashfsGetWriteBufferFreeSpace() -> uint32_t {
    return flashfsGetWriteBufferSize().wrapping_sub(flashfsTransmitBufferUsed());
}
#[no_mangle]
pub unsafe extern "C" fn flashfsGetGeometry() -> *const flashGeometry_s {
    return flashGetGeometry();
}
/* *
 * Write the given buffers to flash sequentially at the current tail address, advancing the tail address after
 * each write.
 *
 * In synchronous mode, waits for the flash to become ready before writing so that every byte requested can be written.
 *
 * In asynchronous mode, if the flash is busy, then the write is aborted and the routine returns immediately.
 * In this case the returned number of bytes written will be less than the total amount requested.
 *
 * Modifies the supplied buffer pointers and sizes to reflect how many bytes remain in each of them.
 *
 * bufferCount: the number of buffers provided
 * buffers: an array of pointers to the beginning of buffers
 * bufferSizes: an array of the sizes of those buffers
 * sync: true if we should wait for the device to be idle before writes, otherwise if the device is busy the
 *       write will be aborted and this routine will return immediately.
 *
 * Returns the number of bytes written
 */
unsafe extern "C" fn flashfsWriteBuffers(mut buffers: *mut *const uint8_t,
                                         mut bufferSizes: *mut uint32_t,
                                         mut bufferCount: libc::c_int,
                                         mut sync: bool) -> uint32_t {
    let mut bytesTotal: uint32_t = 0 as libc::c_int as uint32_t;
    let mut i: libc::c_int = 0;
    i = 0 as libc::c_int;
    while i < bufferCount {
        bytesTotal =
            (bytesTotal as
                 libc::c_uint).wrapping_add(*bufferSizes.offset(i as isize))
                as uint32_t as uint32_t;
        i += 1
    }
    if !sync && !flashIsReady() { return 0 as libc::c_int as uint32_t }
    let mut bytesTotalRemaining: uint32_t = bytesTotal;
    let mut pageSize: uint16_t = (*flashfsGetGeometry()).pageSize;
    while bytesTotalRemaining > 0 as libc::c_int as libc::c_uint {
        let mut bytesTotalThisIteration: uint32_t = 0;
        let mut bytesRemainThisIteration: uint32_t = 0;
        /*
         * Each page needs to be saved in a separate program operation, so
         * if we would cross a page boundary, only write up to the boundary in this iteration:
         */
        if tailAddress.wrapping_rem(pageSize as
                                        libc::c_uint).wrapping_add(bytesTotalRemaining)
               > pageSize as libc::c_uint {
            bytesTotalThisIteration =
                (pageSize as
                     libc::c_uint).wrapping_sub(tailAddress.wrapping_rem(pageSize
                                                                             as
                                                                             libc::c_uint))
        } else { bytesTotalThisIteration = bytesTotalRemaining }
        // Are we at EOF already? Abort.
        if flashfsIsEOF() {
            // May as well throw away any buffered data
            flashfsClearBuffer();
            break ;
        } else {
            flashPageProgramBegin(tailAddress);
            bytesRemainThisIteration = bytesTotalThisIteration;
            i = 0 as libc::c_int;
            while i < bufferCount {
                if *bufferSizes.offset(i as isize) >
                       0 as libc::c_int as libc::c_uint {
                    // Is buffer larger than our write limit? Write our limit out of it
                    if *bufferSizes.offset(i as isize) >=
                           bytesRemainThisIteration {
                        flashPageProgramContinue(*buffers.offset(i as isize),
                                                 bytesRemainThisIteration as
                                                     libc::c_int);
                        let ref mut fresh0 = *buffers.offset(i as isize);
                        *fresh0 =
                            (*fresh0).offset(bytesRemainThisIteration as
                                                 isize);
                        let ref mut fresh1 = *bufferSizes.offset(i as isize);
                        *fresh1 =
                            (*fresh1 as
                                 libc::c_uint).wrapping_sub(bytesRemainThisIteration)
                                as uint32_t as uint32_t;
                        bytesRemainThisIteration =
                            0 as libc::c_int as uint32_t;
                        break ;
                    } else {
                        // We'll still have more to write after finishing this buffer off
                        flashPageProgramContinue(*buffers.offset(i as isize),
                                                 *bufferSizes.offset(i as
                                                                         isize)
                                                     as libc::c_int);
                        bytesRemainThisIteration =
                            (bytesRemainThisIteration as
                                 libc::c_uint).wrapping_sub(*bufferSizes.offset(i
                                                                                    as
                                                                                    isize))
                                as uint32_t as uint32_t;
                        let ref mut fresh2 = *buffers.offset(i as isize);
                        *fresh2 =
                            (*fresh2).offset(*bufferSizes.offset(i as isize)
                                                 as isize);
                        *bufferSizes.offset(i as isize) =
                            0 as libc::c_int as uint32_t
                    }
                }
                i += 1
            }
            flashPageProgramFinish();
            bytesTotalRemaining =
                (bytesTotalRemaining as
                     libc::c_uint).wrapping_sub(bytesTotalThisIteration) as
                    uint32_t as uint32_t;
            // Advance the cursor in the file system to match the bytes we wrote
            flashfsSetTailAddress(tailAddress.wrapping_add(bytesTotalThisIteration));
            /*
         * We'll have to wait for that write to complete before we can issue the next one, so if
         * the user requested asynchronous writes, break now.
         */
            if !sync { break ; }
        }
    }
    return bytesTotal.wrapping_sub(bytesTotalRemaining);
}
/*
 * Since the buffered data might wrap around the end of the circular buffer, we can have two segments of data to write,
 * an initial portion and a possible wrapped portion.
 *
 * This routine will fill the details of those buffers into the provided arrays, which must be at least 2 elements long.
 */
unsafe extern "C" fn flashfsGetDirtyDataBuffers(mut buffers:
                                                    *mut *const uint8_t,
                                                mut bufferSizes:
                                                    *mut uint32_t) {
    let ref mut fresh3 = *buffers.offset(0 as libc::c_int as isize);
    *fresh3 =
        flashWriteBuffer.as_mut_ptr().offset(bufferTail as libc::c_int as
                                                 isize);
    let ref mut fresh4 = *buffers.offset(1 as libc::c_int as isize);
    *fresh4 = flashWriteBuffer.as_mut_ptr().offset(0 as libc::c_int as isize);
    if bufferHead as libc::c_int >= bufferTail as libc::c_int {
        *bufferSizes.offset(0 as libc::c_int as isize) =
            (bufferHead as libc::c_int - bufferTail as libc::c_int) as
                uint32_t;
        *bufferSizes.offset(1 as libc::c_int as isize) =
            0 as libc::c_int as uint32_t
    } else {
        *bufferSizes.offset(0 as libc::c_int as isize) =
            (128 as libc::c_int - bufferTail as libc::c_int) as uint32_t;
        *bufferSizes.offset(1 as libc::c_int as isize) =
            bufferHead as uint32_t
    };
}
/* *
 * Get the current offset of the file pointer within the volume.
 */
#[no_mangle]
pub unsafe extern "C" fn flashfsGetOffset() -> uint32_t {
    let mut buffers: [*const uint8_t; 2] = [0 as *const uint8_t; 2];
    let mut bufferSizes: [uint32_t; 2] = [0; 2];
    // Dirty data in the buffers contributes to the offset
    flashfsGetDirtyDataBuffers(buffers.as_mut_ptr(),
                               bufferSizes.as_mut_ptr());
    return tailAddress.wrapping_add(bufferSizes[0 as libc::c_int as
                                                    usize]).wrapping_add(bufferSizes[1
                                                                                         as
                                                                                         libc::c_int
                                                                                         as
                                                                                         usize]);
}
/* *
 * Called after bytes have been written from the buffer to advance the position of the tail by the given amount.
 */
unsafe extern "C" fn flashfsAdvanceTailInBuffer(mut delta: uint32_t) {
    bufferTail =
        (bufferTail as libc::c_uint).wrapping_add(delta) as uint8_t as
            uint8_t;
    // Wrap tail around the end of the buffer
    if bufferTail as libc::c_int >= 128 as libc::c_int {
        bufferTail =
            (bufferTail as libc::c_int - 128 as libc::c_int) as uint8_t
    }
    if flashfsBufferIsEmpty() {
        flashfsClearBuffer();
        // Bring buffer pointers back to the start to be tidier
    };
}
/* *
 * If the flash is ready to accept writes, flush the buffer to it.
 *
 * Returns true if all data in the buffer has been flushed to the device, or false if
 * there is still data to be written (call flush again later).
 */
#[no_mangle]
pub unsafe extern "C" fn flashfsFlushAsync() -> bool {
    if flashfsBufferIsEmpty() {
        return 1 as libc::c_int != 0
        // Nothing to flush
    }
    let mut buffers: [*const uint8_t; 2] = [0 as *const uint8_t; 2];
    let mut bufferSizes: [uint32_t; 2] = [0; 2];
    let mut bytesWritten: uint32_t = 0;
    flashfsGetDirtyDataBuffers(buffers.as_mut_ptr(),
                               bufferSizes.as_mut_ptr());
    bytesWritten =
        flashfsWriteBuffers(buffers.as_mut_ptr(), bufferSizes.as_mut_ptr(),
                            2 as libc::c_int, 0 as libc::c_int != 0);
    flashfsAdvanceTailInBuffer(bytesWritten);
    return flashfsBufferIsEmpty();
}
/* *
 * Wait for the flash to become ready and begin flushing any buffered data to flash.
 *
 * The flash will still be busy some time after this sync completes, but space will
 * be freed up to accept more writes in the write buffer.
 */
#[no_mangle]
pub unsafe extern "C" fn flashfsFlushSync() {
    if flashfsBufferIsEmpty() {
        return
        // Nothing to flush
    }
    let mut buffers: [*const uint8_t; 2] = [0 as *const uint8_t; 2];
    let mut bufferSizes: [uint32_t; 2] = [0; 2];
    flashfsGetDirtyDataBuffers(buffers.as_mut_ptr(),
                               bufferSizes.as_mut_ptr());
    flashfsWriteBuffers(buffers.as_mut_ptr(), bufferSizes.as_mut_ptr(),
                        2 as libc::c_int, 1 as libc::c_int != 0);
    // We've written our entire buffer now:
    flashfsClearBuffer();
}
#[no_mangle]
pub unsafe extern "C" fn flashfsSeekAbs(mut offset: uint32_t) {
    flashfsFlushSync();
    flashfsSetTailAddress(offset);
}
#[no_mangle]
pub unsafe extern "C" fn flashfsSeekRel(mut offset: int32_t) {
    flashfsFlushSync();
    flashfsSetTailAddress(tailAddress.wrapping_add(offset as libc::c_uint));
}
/* *
 * Write the given byte asynchronously to the flash. If the buffer overflows, data is silently discarded.
 */
#[no_mangle]
pub unsafe extern "C" fn flashfsWriteByte(mut byte: uint8_t) {
    let fresh5 = bufferHead;
    bufferHead = bufferHead.wrapping_add(1);
    flashWriteBuffer[fresh5 as usize] = byte;
    if bufferHead as libc::c_int >= 128 as libc::c_int {
        bufferHead = 0 as libc::c_int as uint8_t
    }
    if flashfsTransmitBufferUsed() >= 64 as libc::c_int as libc::c_uint {
        flashfsFlushAsync();
    };
}
/* *
 * Write the given buffer to the flash either synchronously or asynchronously depending on the 'sync' parameter.
 *
 * If writing asynchronously, data will be silently discarded if the buffer overflows.
 * If writing synchronously, the routine will block waiting for the flash to become ready so will never drop data.
 */
#[no_mangle]
pub unsafe extern "C" fn flashfsWrite(mut data: *const uint8_t,
                                      mut len: libc::c_uint, mut sync: bool) {
    let mut buffers: [*const uint8_t; 3] = [0 as *const uint8_t; 3];
    let mut bufferSizes: [uint32_t; 3] = [0; 3];
    // There could be two dirty buffers to write out already:
    flashfsGetDirtyDataBuffers(buffers.as_mut_ptr(),
                               bufferSizes.as_mut_ptr());
    // Plus the buffer the user supplied:
    buffers[2 as libc::c_int as usize] = data;
    bufferSizes[2 as libc::c_int as usize] = len;
    /*
     * Would writing this data to our buffer cause our buffer to reach the flush threshold? If so try to write through
     * to the flash now
     */
    if bufferSizes[0 as libc::c_int as
                       usize].wrapping_add(bufferSizes[1 as libc::c_int as
                                                           usize]).wrapping_add(bufferSizes[2
                                                                                                as
                                                                                                libc::c_int
                                                                                                as
                                                                                                usize])
           >= 64 as libc::c_int as libc::c_uint {
        let mut bytesWritten: uint32_t = 0;
        // Attempt to write all three buffers through to the flash asynchronously
        bytesWritten =
            flashfsWriteBuffers(buffers.as_mut_ptr(),
                                bufferSizes.as_mut_ptr(), 3 as libc::c_int,
                                0 as libc::c_int != 0);
        if bufferSizes[0 as libc::c_int as usize] ==
               0 as libc::c_int as libc::c_uint &&
               bufferSizes[1 as libc::c_int as usize] ==
                   0 as libc::c_int as libc::c_uint {
            // We wrote all the data that was previously buffered
            flashfsClearBuffer();
            if bufferSizes[2 as libc::c_int as usize] ==
                   0 as libc::c_int as libc::c_uint {
                // And we wrote all the data the user supplied! Job done!
                return
            }
        } else {
            // We only wrote a portion of the old data, so advance the tail to remove the bytes we did write from the buffer
            flashfsAdvanceTailInBuffer(bytesWritten);
        }
        // Is the remainder of the data to be written too big to fit in the buffers?
        if bufferSizes[0 as libc::c_int as
                           usize].wrapping_add(bufferSizes[1 as libc::c_int as
                                                               usize]).wrapping_add(bufferSizes[2
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    usize])
               > (128 as libc::c_int - 1 as libc::c_int) as libc::c_uint {
            if sync {
                // Write it through synchronously
                flashfsWriteBuffers(buffers.as_mut_ptr(),
                                    bufferSizes.as_mut_ptr(),
                                    3 as libc::c_int, 1 as libc::c_int != 0);
                flashfsClearBuffer();
            }
            return
        }
        // Fall through and add the remainder of the incoming data to our buffer
        data = buffers[2 as libc::c_int as usize];
        len = bufferSizes[2 as libc::c_int as usize]
    }
    // Buffer up the data the user supplied instead of writing it right away
    // First write the portion before we wrap around the end of the circular buffer
    let mut bufferBytesBeforeWrap: libc::c_uint =
        (128 as libc::c_int - bufferHead as libc::c_int) as libc::c_uint;
    let mut firstPortion: libc::c_uint =
        if len < bufferBytesBeforeWrap { len } else { bufferBytesBeforeWrap };
    memcpy(flashWriteBuffer.as_mut_ptr().offset(bufferHead as libc::c_int as
                                                    isize) as
               *mut libc::c_void, data as *const libc::c_void,
           firstPortion as libc::c_ulong);
    bufferHead =
        (bufferHead as libc::c_uint).wrapping_add(firstPortion) as uint8_t as
            uint8_t;
    data = data.offset(firstPortion as isize);
    len = len.wrapping_sub(firstPortion);
    // If we wrap the head around, write the remainder to the start of the buffer (if any)
    if bufferHead as libc::c_int == 128 as libc::c_int {
        memcpy(flashWriteBuffer.as_mut_ptr().offset(0 as libc::c_int as isize)
                   as *mut libc::c_void, data as *const libc::c_void,
               len as libc::c_ulong);
        bufferHead = len as uint8_t
    };
}
/* *
 * Read `len` bytes from the given address into the supplied buffer.
 *
 * Returns the number of bytes actually read which may be less than that requested.
 */
#[no_mangle]
pub unsafe extern "C" fn flashfsReadAbs(mut address: uint32_t,
                                        mut buffer: *mut uint8_t,
                                        mut len: libc::c_uint)
 -> libc::c_int {
    let mut bytesRead: libc::c_int = 0;
    // Did caller try to read past the end of the volume?
    if address.wrapping_add(len) > flashfsGetSize() {
        // Truncate their request
        len = flashfsGetSize().wrapping_sub(address)
    }
    // Since the read could overlap data in our dirty buffers, force a sync to clear those first
    flashfsFlushSync();
    bytesRead = flashReadBytes(address, buffer, len as libc::c_int);
    return bytesRead;
}
/* *
 * Find the offset of the start of the free space on the device (or the size of the device if it is full).
 */
#[no_mangle]
pub unsafe extern "C" fn flashfsIdentifyStartOfFreeSpace() -> libc::c_int {
    /* Find the start of the free space on the device by examining the beginning of blocks with a binary search,
     * looking for ones that appear to be erased. We can achieve this with good accuracy because an erased block
     * is all bits set to 1, which pretty much never appears in reasonable size substrings of blackbox logs.
     *
     * To do better we might write a volume header instead, which would mark how much free space remains. But keeping
     * a header up to date while logging would incur more writes to the flash, which would consume precious write
     * bandwidth and block more often.
     */
    let mut testBuffer: C2RustUnnamed =
        C2RustUnnamed{bytes:
                          [0;
                              16],}; // Smallest block index in the search region
    let mut left: libc::c_int =
        0 as
            libc::c_int; // One past the largest block index in the search region
    let mut right: libc::c_int =
        flashfsGetSize().wrapping_div(FREE_BLOCK_SIZE as libc::c_int as
                                          libc::c_uint) as libc::c_int;
    let mut mid: libc::c_int = 0;
    let mut result: libc::c_int = right;
    let mut i: libc::c_int = 0;
    let mut blockErased: bool = false;
    while left < right {
        mid = (left + right) / 2 as libc::c_int;
        if flashReadBytes((mid * FREE_BLOCK_SIZE as libc::c_int) as uint32_t,
                          testBuffer.bytes.as_mut_ptr(),
                          FREE_BLOCK_TEST_SIZE_BYTES as libc::c_int) <
               FREE_BLOCK_TEST_SIZE_BYTES as libc::c_int {
            break ;
        }
        // Checking the buffer 4 bytes at a time like this is probably faster than byte-by-byte, but I didn't benchmark it :)
        blockErased = 1 as libc::c_int != 0;
        i = 0 as libc::c_int;
        while i < FREE_BLOCK_TEST_SIZE_INTS as libc::c_int {
            if testBuffer.ints[i as usize] != 0xffffffff as libc::c_uint {
                blockErased = 0 as libc::c_int != 0;
                break ;
            } else { i += 1 }
        }
        if blockErased {
            /* This erased block might be the leftmost erased block in the volume, but we'll need to continue the
             * search leftwards to find out:
             */
            result = mid;
            right = mid
        } else { left = mid + 1 as libc::c_int }
    }
    return result * FREE_BLOCK_SIZE as libc::c_int;
}
/* *
 * Returns true if the file pointer is at the end of the device.
 */
#[no_mangle]
pub unsafe extern "C" fn flashfsIsEOF() -> bool {
    return tailAddress >= flashfsGetSize();
}
#[no_mangle]
pub unsafe extern "C" fn flashfsClose() {
    match (*flashfsGetGeometry()).flashType as libc::c_uint {
        1 => {
            flashFlush();
            // Advance tailAddress to next page boundary.
            let mut pageSize: uint32_t =
                (*flashfsGetGeometry()).pageSize as uint32_t;
            flashfsSetTailAddress(tailAddress.wrapping_add(pageSize).wrapping_sub(1
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                                      &
                                      !pageSize.wrapping_sub(1 as libc::c_int
                                                                 as
                                                                 libc::c_uint));
        }
        0 | _ => { }
    };
}
/* *
 * Call after initializing the flash chip in order to set up the filesystem.
 */
#[no_mangle]
pub unsafe extern "C" fn flashfsInit() {
    // If we have a flash chip present at all
    if flashfsGetSize() > 0 as libc::c_int as libc::c_uint {
        // Start the file pointer off at the beginning of free space so caller can start writing immediately
        flashfsSeekAbs(flashfsIdentifyStartOfFreeSpace() as uint32_t);
    };
}
