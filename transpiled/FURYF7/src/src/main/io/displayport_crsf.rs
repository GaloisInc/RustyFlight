use ::libc;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn memcmp(_: *const libc::c_void, _: *const libc::c_void,
              _: libc::c_ulong) -> libc::c_int;
    #[no_mangle]
    fn strlen(_: *const libc::c_char) -> libc::c_ulong;
    #[no_mangle]
    fn displayInit(instance: *mut displayPort_t,
                   vTable: *const displayPortVTable_t);
    #[no_mangle]
    fn cmsMenuExit(pPort: *mut displayPort_t, ptr: *const libc::c_void)
     -> libc::c_long;
    #[no_mangle]
    fn cmsMenuOpen();
    #[no_mangle]
    fn cmsDisplayPortSelect(instance: *mut displayPort_t) -> bool;
    #[no_mangle]
    static mut cmsInMenu: bool;
    #[no_mangle]
    fn tfp_sprintf(s: *mut libc::c_char, fmt: *const libc::c_char, _: ...)
     -> libc::c_int;
    #[no_mangle]
    fn millis() -> timeMs_t;
}
pub type size_t = libc::c_ulong;
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct displayPortVTable_s {
    pub grab: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                         -> libc::c_int>,
    pub release: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                            -> libc::c_int>,
    pub clearScreen: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                                -> libc::c_int>,
    pub drawScreen: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                               -> libc::c_int>,
    pub screenSize: Option<unsafe extern "C" fn(_: *const displayPort_t)
                               -> libc::c_int>,
    pub writeString: Option<unsafe extern "C" fn(_: *mut displayPort_t,
                                                 _: uint8_t, _: uint8_t,
                                                 _: *const libc::c_char)
                                -> libc::c_int>,
    pub writeChar: Option<unsafe extern "C" fn(_: *mut displayPort_t,
                                               _: uint8_t, _: uint8_t,
                                               _: uint8_t) -> libc::c_int>,
    pub isTransferInProgress: Option<unsafe extern "C" fn(_:
                                                              *const displayPort_t)
                                         -> bool>,
    pub heartbeat: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                              -> libc::c_int>,
    pub resync: Option<unsafe extern "C" fn(_: *mut displayPort_t) -> ()>,
    pub isSynced: Option<unsafe extern "C" fn(_: *const displayPort_t)
                             -> bool>,
    pub txBytesFree: Option<unsafe extern "C" fn(_: *const displayPort_t)
                                -> uint32_t>,
}
pub type displayPort_t = displayPort_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct displayPort_s {
    pub vTable: *const displayPortVTable_s,
    pub device: *mut libc::c_void,
    pub rows: uint8_t,
    pub cols: uint8_t,
    pub posX: uint8_t,
    pub posY: uint8_t,
    pub cleared: bool,
    pub cursorRow: int8_t,
    pub grabCount: int8_t,
}
pub type displayPortVTable_t = displayPortVTable_s;
// millisecond time
pub type timeMs_t = uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct crsfDisplayPortScreen_s {
    pub buffer: [libc::c_char; 288],
    pub pendingTransport: [bool; 9],
    pub rows: uint8_t,
    pub cols: uint8_t,
    pub reset: bool,
}
pub type crsfDisplayPortScreen_t = crsfDisplayPortScreen_s;
#[no_mangle]
pub static mut pCurrentDisplay: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
static mut crsfScreen: crsfDisplayPortScreen_t =
    crsfDisplayPortScreen_t{buffer: [0; 288],
                            pendingTransport: [false; 9],
                            rows: 0,
                            cols: 0,
                            reset: false,};
static mut delayTransportUntilMs: timeMs_t = 0 as libc::c_int as timeMs_t;
#[no_mangle]
pub static mut crsfDisplayPort: displayPort_t =
    displayPort_t{vTable: 0 as *const displayPortVTable_s,
                  device: 0 as *const libc::c_void as *mut libc::c_void,
                  rows: 0,
                  cols: 0,
                  posX: 0,
                  posY: 0,
                  cleared: false,
                  cursorRow: 0,
                  grabCount: 0,};
unsafe extern "C" fn crsfGrab(mut displayPort: *mut displayPort_t)
 -> libc::c_int {
    (*displayPort).grabCount =
        1 as libc::c_int as int8_t; // truncate at colCount
    return (*displayPort).grabCount as libc::c_int;
}
unsafe extern "C" fn crsfClearScreen(mut displayPort: *mut displayPort_t)
 -> libc::c_int {
    memset(crsfScreen.buffer.as_mut_ptr() as *mut libc::c_void, ' ' as i32,
           ::core::mem::size_of::<[libc::c_char; 288]>() as libc::c_ulong);
    memset(crsfScreen.pendingTransport.as_mut_ptr() as *mut libc::c_void,
           0 as libc::c_int,
           ::core::mem::size_of::<[bool; 9]>() as libc::c_ulong);
    crsfScreen.reset = 1 as libc::c_int != 0;
    delayTransportUntilMs =
        millis().wrapping_add(45 as libc::c_int as libc::c_uint);
    return 0 as libc::c_int;
}
unsafe extern "C" fn crsfRelease(mut displayPort: *mut displayPort_t)
 -> libc::c_int {
    (*displayPort).grabCount = 0 as libc::c_int as int8_t;
    return crsfClearScreen(displayPort);
}
unsafe extern "C" fn crsfDrawScreen(mut displayPort: *mut displayPort_t)
 -> libc::c_int {
    return 0 as libc::c_int;
}
unsafe extern "C" fn crsfScreenSize(mut displayPort: *const displayPort_t)
 -> libc::c_int {
    return (*displayPort).rows as libc::c_int *
               (*displayPort).cols as libc::c_int;
}
unsafe extern "C" fn crsfWriteString(mut displayPort: *mut displayPort_t,
                                     mut col: uint8_t, mut row: uint8_t,
                                     mut s: *const libc::c_char)
 -> libc::c_int {
    if row as libc::c_int >= crsfScreen.rows as libc::c_int ||
           col as libc::c_int >= crsfScreen.cols as libc::c_int {
        return 0 as libc::c_int
    }
    let truncLen: size_t =
        ({
             let mut _a: libc::c_int = strlen(s) as libc::c_int;
             let mut _b: libc::c_int =
                 crsfScreen.cols as libc::c_int - col as libc::c_int;
             if _a < _b { _a } else { _b }
         }) as size_t;
    let mut rowStart: *mut libc::c_char =
        &mut *crsfScreen.buffer.as_mut_ptr().offset((row as libc::c_int *
                                                         crsfScreen.cols as
                                                             libc::c_int +
                                                         col as libc::c_int)
                                                        as isize) as
            *mut libc::c_char;
    crsfScreen.pendingTransport[row as usize] =
        memcmp(rowStart as *const libc::c_void, s as *const libc::c_void,
               truncLen) != 0;
    if crsfScreen.pendingTransport[row as usize] {
        memcpy(rowStart as *mut libc::c_void, s as *const libc::c_void,
               truncLen);
    }
    return 0 as libc::c_int;
}
unsafe extern "C" fn crsfWriteChar(mut displayPort: *mut displayPort_t,
                                   mut col: uint8_t, mut row: uint8_t,
                                   mut c: uint8_t) -> libc::c_int {
    let mut s: [libc::c_char; 1] = [0; 1];
    tfp_sprintf(s.as_mut_ptr(), b"%c\x00" as *const u8 as *const libc::c_char,
                c as libc::c_int);
    return crsfWriteString(displayPort, col, row, s.as_mut_ptr());
}
unsafe extern "C" fn crsfIsTransferInProgress(mut displayPort:
                                                  *const displayPort_t)
 -> bool {
    return 0 as libc::c_int != 0;
}
unsafe extern "C" fn crsfIsSynced(mut displayPort: *const displayPort_t)
 -> bool {
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn crsfHeartbeat(mut displayPort: *mut displayPort_t)
 -> libc::c_int {
    return 0 as libc::c_int;
}
unsafe extern "C" fn crsfResync(mut displayPort: *mut displayPort_t) {
    (*displayPort).rows = crsfScreen.rows;
    (*displayPort).cols = crsfScreen.cols;
}
unsafe extern "C" fn crsfTxBytesFree(mut displayPort: *const displayPort_t)
 -> uint32_t {
    return 4294967295 as libc::c_uint;
}
static mut crsfDisplayPortVTable: displayPortVTable_t =
    unsafe {
        {
            let mut init =
                displayPortVTable_s{grab:
                                        Some(crsfGrab as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t)
                                                     -> libc::c_int),
                                    release:
                                        Some(crsfRelease as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t)
                                                     -> libc::c_int),
                                    clearScreen:
                                        Some(crsfClearScreen as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t)
                                                     -> libc::c_int),
                                    drawScreen:
                                        Some(crsfDrawScreen as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t)
                                                     -> libc::c_int),
                                    screenSize:
                                        Some(crsfScreenSize as
                                                 unsafe extern "C" fn(_:
                                                                          *const displayPort_t)
                                                     -> libc::c_int),
                                    writeString:
                                        Some(crsfWriteString as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t,
                                                                      _:
                                                                          uint8_t,
                                                                      _:
                                                                          uint8_t,
                                                                      _:
                                                                          *const libc::c_char)
                                                     -> libc::c_int),
                                    writeChar:
                                        Some(crsfWriteChar as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t,
                                                                      _:
                                                                          uint8_t,
                                                                      _:
                                                                          uint8_t,
                                                                      _:
                                                                          uint8_t)
                                                     -> libc::c_int),
                                    isTransferInProgress:
                                        Some(crsfIsTransferInProgress as
                                                 unsafe extern "C" fn(_:
                                                                          *const displayPort_t)
                                                     -> bool),
                                    heartbeat:
                                        Some(crsfHeartbeat as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t)
                                                     -> libc::c_int),
                                    resync:
                                        Some(crsfResync as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t)
                                                     -> ()),
                                    isSynced:
                                        Some(crsfIsSynced as
                                                 unsafe extern "C" fn(_:
                                                                          *const displayPort_t)
                                                     -> bool),
                                    txBytesFree:
                                        Some(crsfTxBytesFree as
                                                 unsafe extern "C" fn(_:
                                                                          *const displayPort_t)
                                                     -> uint32_t),};
            init
        }
    };
#[no_mangle]
pub unsafe extern "C" fn crsfDisplayPortScreen()
 -> *mut crsfDisplayPortScreen_t {
    return &mut crsfScreen;
}
#[no_mangle]
pub unsafe extern "C" fn crsfDisplayPortMenuOpen() {
    if cmsInMenu { return }
    if cmsDisplayPortSelect(&mut crsfDisplayPort) {
        cmsMenuOpen();
        delayTransportUntilMs =
            millis().wrapping_add(400 as libc::c_int as libc::c_uint)
    };
}
#[no_mangle]
pub unsafe extern "C" fn crsfDisplayPortMenuExit() {
    if !cmsInMenu { return }
    let mut exitMenu: uint8_t = 0 as libc::c_int as uint8_t;
    cmsMenuExit(&mut crsfDisplayPort,
                &mut exitMenu as *mut uint8_t as *const libc::c_void);
}
#[no_mangle]
pub unsafe extern "C" fn crsfDisplayPortSetDimensions(mut rows: uint8_t,
                                                      mut cols: uint8_t) {
    crsfScreen.rows =
        ({
             let mut _a: uint8_t = rows;
             let mut _b: libc::c_int = 9 as libc::c_int;
             if (_a as libc::c_int) < _b { _a as libc::c_int } else { _b }
         }) as uint8_t;
    crsfScreen.cols =
        ({
             let mut _a: uint8_t = cols;
             let mut _b: libc::c_int = 32 as libc::c_int;
             if (_a as libc::c_int) < _b { _a as libc::c_int } else { _b }
         }) as uint8_t;
    crsfResync(&mut crsfDisplayPort);
}
#[no_mangle]
pub unsafe extern "C" fn crsfDisplayPortRefresh() {
    if !cmsInMenu { crsfDisplayPortMenuOpen(); return }
    memset(crsfScreen.pendingTransport.as_mut_ptr() as *mut libc::c_void,
           1 as libc::c_int, crsfScreen.rows as libc::c_ulong);
    crsfScreen.reset = 1 as libc::c_int != 0;
    delayTransportUntilMs =
        millis().wrapping_add(45 as libc::c_int as libc::c_uint);
}
#[no_mangle]
pub unsafe extern "C" fn crsfDisplayPortNextRow() -> libc::c_int {
    let currentTimeMs: timeMs_t = millis();
    if currentTimeMs < delayTransportUntilMs { return -(1 as libc::c_int) }
    let mut i: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while i < 9 as libc::c_int as libc::c_uint {
        if crsfScreen.pendingTransport[i as usize] { return i as libc::c_int }
        i = i.wrapping_add(1)
    }
    return -(1 as libc::c_int);
}
#[no_mangle]
pub unsafe extern "C" fn displayPortCrsfInit() -> *mut displayPort_s {
    crsfDisplayPortSetDimensions(9 as libc::c_int as uint8_t,
                                 32 as libc::c_int as uint8_t);
    displayInit(&mut crsfDisplayPort, &crsfDisplayPortVTable);
    return &mut crsfDisplayPort;
}