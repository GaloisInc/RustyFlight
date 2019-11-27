use core;
use libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
#[no_mangle]
pub static mut vtx58frequencyTable: [[uint16_t; 8]; 5] =
    [[5865i32 as uint16_t, 5845i32 as uint16_t, 5825i32 as uint16_t,
      5805i32 as uint16_t, 5785i32 as uint16_t, 5765i32 as uint16_t,
      5745i32 as uint16_t, 5725i32 as uint16_t],
     [5733i32 as uint16_t, 5752i32 as uint16_t, 5771i32 as uint16_t,
      5790i32 as uint16_t, 5809i32 as uint16_t, 5828i32 as uint16_t,
      5847i32 as uint16_t, 5866i32 as uint16_t],
     [5705i32 as uint16_t, 5685i32 as uint16_t, 5665i32 as uint16_t,
      5645i32 as uint16_t, 5885i32 as uint16_t, 5905i32 as uint16_t,
      5925i32 as uint16_t, 5945i32 as uint16_t],
     [5740i32 as uint16_t, 5760i32 as uint16_t, 5780i32 as uint16_t,
      5800i32 as uint16_t, 5820i32 as uint16_t, 5840i32 as uint16_t,
      5860i32 as uint16_t, 5880i32 as uint16_t],
     [5658i32 as uint16_t, 5695i32 as uint16_t, 5732i32 as uint16_t,
      5769i32 as uint16_t, 5806i32 as uint16_t, 5843i32 as uint16_t,
      5880i32 as uint16_t, 5917i32 as uint16_t]];
#[no_mangle]
pub static mut vtx58BandNames: [*const libc::c_char; 6] =
    [b"--------\x00" as *const u8 as *const libc::c_char,
     b"BOSCAM A\x00" as *const u8 as *const libc::c_char,
     b"BOSCAM B\x00" as *const u8 as *const libc::c_char,
     b"BOSCAM E\x00" as *const u8 as *const libc::c_char,
     b"FATSHARK\x00" as *const u8 as *const libc::c_char,
     b"RACEBAND\x00" as *const u8 as *const libc::c_char];
#[no_mangle]
pub static mut vtx58BandLetter: [libc::c_char; 7] =
    [45, 65, 66, 69, 70, 82, 0];
#[no_mangle]
pub static mut vtx58ChannelNames: [*const libc::c_char; 9] =
    [b"-\x00" as *const u8 as *const libc::c_char,
     b"1\x00" as *const u8 as *const libc::c_char,
     b"2\x00" as *const u8 as *const libc::c_char,
     b"3\x00" as *const u8 as *const libc::c_char,
     b"4\x00" as *const u8 as *const libc::c_char,
     b"5\x00" as *const u8 as *const libc::c_char,
     b"6\x00" as *const u8 as *const libc::c_char,
     b"7\x00" as *const u8 as *const libc::c_char,
     b"8\x00" as *const u8 as *const libc::c_char];
//Converts frequency (in MHz) to band and channel values.
#[no_mangle]
pub unsafe extern "C" fn vtx58_Freq2Bandchan(mut freq: uint16_t,
                                             mut pBand: *mut uint8_t,
                                             mut pChannel: *mut uint8_t)
 -> bool {
    // Use reverse lookup order so that 5880Mhz
    // get Raceband 7 instead of Fatshark 8.
    let mut band: libc::c_int = 5i32 - 1i32;
    while band >= 0i32 {
        let mut channel: libc::c_int = 0i32;
        while channel < 8i32 {
            if vtx58frequencyTable[band as usize][channel as usize] as
                   libc::c_int == freq as libc::c_int {
                *pBand = (band + 1i32) as uint8_t;
                *pChannel = (channel + 1i32) as uint8_t;
                return 1i32 != 0
            }
            channel += 1
        }
        band -= 1
    }
    *pBand = 0i32 as uint8_t;
    *pChannel = 0i32 as uint8_t;
    return 0i32 != 0;
}
//Converts band and channel values to a frequency (in MHz) value.
// band:  Band value (1 to 5).
// channel:  Channel value (1 to 8).
// Returns frequency value (in MHz), or 0 if band/channel out of range.
#[no_mangle]
pub unsafe extern "C" fn vtx58_Bandchan2Freq(mut band: uint8_t,
                                             mut channel: uint8_t)
 -> uint16_t {
    if band as libc::c_int > 0i32 && band as libc::c_int <= 5i32 &&
           channel as libc::c_int > 0i32 && channel as libc::c_int <= 8i32 {
        return vtx58frequencyTable[(band as libc::c_int - 1i32) as
                                       usize][(channel as libc::c_int - 1i32)
                                                  as usize]
    }
    return 0i32 as uint16_t;
}
