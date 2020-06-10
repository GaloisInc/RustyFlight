use ::libc;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn strlen(_: *const libc::c_char) -> libc::c_ulong;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type __uint64_t = libc::c_ulong;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type uint64_t = __uint64_t;
/*
 * Derived from
 * https://github.com/fetisov/emfat/blob/master/project/emfat.c
 * version: 1.0 (4.01.2015)
 */
/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 by Sergey Fetisov <fsenok@gmail.com>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct emfat_entry_s {
    pub name: *const libc::c_char,
    pub dir: bool,
    pub attr: uint8_t,
    pub level: libc::c_int,
    pub offset: uint32_t,
    pub curr_size: uint32_t,
    pub max_size: uint32_t,
    pub user_data: libc::c_long,
    pub cma_time: [uint32_t; 3],
    pub readcb: emfat_readcb_t,
    pub writecb: emfat_writecb_t,
    pub priv_0: C2RustUnnamed,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed {
    pub first_clust: uint32_t,
    pub last_clust: uint32_t,
    pub last_reserved: uint32_t,
    pub num_subentry: uint32_t,
    pub top: *mut emfat_entry_s,
    pub sub: *mut emfat_entry_s,
    pub next: *mut emfat_entry_s,
}
pub type emfat_writecb_t
    =
    Option<unsafe extern "C" fn(_: *const uint8_t, _: libc::c_int,
                                _: uint32_t, _: *mut emfat_entry_s) -> ()>;
pub type emfat_readcb_t
    =
    Option<unsafe extern "C" fn(_: *mut uint8_t, _: libc::c_int, _: uint32_t,
                                _: *mut emfat_entry_s) -> ()>;
pub type emfat_entry_t = emfat_entry_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct emfat_s {
    pub vol_size: uint64_t,
    pub disk_sectors: uint32_t,
    pub vol_label: *const libc::c_char,
    pub priv_0: C2RustUnnamed_0,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed_0 {
    pub boot_lba: uint32_t,
    pub fsinfo_lba: uint32_t,
    pub fat1_lba: uint32_t,
    pub fat2_lba: uint32_t,
    pub root_lba: uint32_t,
    pub num_clust: uint32_t,
    pub free_clust: uint32_t,
    pub entries: *mut emfat_entry_t,
    pub last_entry: *mut emfat_entry_t,
    pub num_entries: libc::c_int,
}
pub type emfat_t = emfat_s;
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct dir_entry {
    pub name: [uint8_t; 8],
    pub extn: [uint8_t; 3],
    pub attr: uint8_t,
    pub reserved: uint8_t,
    pub crt_time_tenth: uint8_t,
    pub crt_time: uint16_t,
    pub crt_date: uint16_t,
    pub lst_access_date: uint16_t,
    pub strt_clus_hword: uint16_t,
    pub lst_mod_time: uint16_t,
    pub lst_mod_date: uint16_t,
    pub strt_clus_lword: uint16_t,
    pub size: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct boot_sector {
    pub jump: [uint8_t; 3],
    pub OEM_name: [uint8_t; 8],
    pub bytes_per_sec: uint16_t,
    pub sec_per_clus: uint8_t,
    pub reserved_sec_cnt: uint16_t,
    pub fat_cnt: uint8_t,
    pub root_dir_max_cnt: uint16_t,
    pub tot_sectors: uint16_t,
    pub media_desc: uint8_t,
    pub sec_per_fat_fat16: uint16_t,
    pub sec_per_track: uint16_t,
    pub number_of_heads: uint16_t,
    pub hidden_sec_cnt: uint32_t,
    pub tol_sector_cnt: uint32_t,
    pub sectors_per_fat: uint32_t,
    pub ext_flags: uint16_t,
    pub fs_version: [uint8_t; 2],
    pub root_dir_strt_cluster: uint32_t,
    pub fs_info_sector: uint16_t,
    pub backup_boot_sector: uint16_t,
    pub reserved: [uint8_t; 12],
    pub drive_number: uint8_t,
    pub reserved1: uint8_t,
    pub boot_sig: uint8_t,
    pub volume_id: [uint8_t; 4],
    pub volume_label: [uint8_t; 11],
    pub file_system_type: [uint8_t; 8],
}
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct fsinfo_t {
    pub signature1: uint32_t,
    pub reserved1: [uint32_t; 120],
    pub signature2: uint32_t,
    pub free_clusters: uint32_t,
    pub next_cluster: uint32_t,
    pub reserved2: [uint32_t; 3],
    pub signature3: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct mbr_t {
    pub Code: [uint8_t; 440],
    pub DiskSig: uint32_t,
    pub Reserved: uint16_t,
    pub PartTable: [mbr_part_t; 4],
    pub BootSignature: [uint8_t; 2],
}
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct mbr_part_t {
    pub status: uint8_t,
    pub start_head: uint8_t,
    pub start_sector: uint8_t,
    pub start_cylinder: uint8_t,
    pub PartType: uint8_t,
    pub end_head: uint8_t,
    pub end_sector: uint8_t,
    pub end_cylinder: uint8_t,
    pub StartLBA: uint32_t,
    pub EndLBA: uint32_t,
}
#[no_mangle]
pub unsafe extern "C" fn emfat_init_entries(mut entries: *mut emfat_entry_t)
 -> bool {
    let mut e: *mut emfat_entry_t = 0 as *mut emfat_entry_t;
    let mut i: libc::c_int = 0;
    let mut n: libc::c_int = 0;
    e = &mut *entries.offset(0 as libc::c_int as isize) as *mut emfat_entry_t;
    if (*e).level != 0 as libc::c_int || !(*e).dir || (*e).name.is_null() {
        return 0 as libc::c_int != 0
    }
    (*e).priv_0.top = 0 as *mut emfat_entry_s;
    (*e).priv_0.next = 0 as *mut emfat_entry_s;
    (*e).priv_0.sub = 0 as *mut emfat_entry_s;
    (*e).priv_0.num_subentry = 0 as libc::c_int as uint32_t;
    n = 0 as libc::c_int;
    i = 1 as libc::c_int;
    while !(*entries.offset(i as isize)).name.is_null() {
        let ref mut fresh0 = (*entries.offset(i as isize)).priv_0.top;
        *fresh0 = 0 as *mut emfat_entry_s;
        let ref mut fresh1 = (*entries.offset(i as isize)).priv_0.next;
        *fresh1 = 0 as *mut emfat_entry_s;
        let ref mut fresh2 = (*entries.offset(i as isize)).priv_0.sub;
        *fresh2 = 0 as *mut emfat_entry_s;
        (*entries.offset(i as isize)).priv_0.num_subentry =
            0 as libc::c_int as uint32_t;
        if (*entries.offset(i as isize)).level == n - 1 as libc::c_int {
            if n == 0 as libc::c_int { return 0 as libc::c_int != 0 }
            e = (*e).priv_0.top;
            n -= 1
        }
        if (*entries.offset(i as isize)).level == n + 1 as libc::c_int {
            if !(*e).dir { return 0 as libc::c_int != 0 }
            (*e).priv_0.sub =
                &mut *entries.offset(i as isize) as *mut emfat_entry_t;
            let ref mut fresh3 = (*entries.offset(i as isize)).priv_0.top;
            *fresh3 = e;
            e = &mut *entries.offset(i as isize) as *mut emfat_entry_t;
            n += 1
        } else if (*entries.offset(i as isize)).level == n {
            if n == 0 as libc::c_int { return 0 as libc::c_int != 0 }
            (*(*e).priv_0.top).priv_0.num_subentry =
                (*(*e).priv_0.top).priv_0.num_subentry.wrapping_add(1);
            let ref mut fresh4 = (*entries.offset(i as isize)).priv_0.top;
            *fresh4 = (*e).priv_0.top;
            (*e).priv_0.next =
                &mut *entries.offset(i as isize) as *mut emfat_entry_t;
            e = &mut *entries.offset(i as isize) as *mut emfat_entry_t
        } else { return 0 as libc::c_int != 0 }
        i += 1
    }
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn lba_to_chs(mut lba: libc::c_int, mut cl: *mut uint8_t,
                                mut ch: *mut uint8_t, mut dh: *mut uint8_t) {
    let mut cylinder: libc::c_int = 0;
    let mut head: libc::c_int = 0;
    let mut sector: libc::c_int = 0;
    let mut sectors: libc::c_int = 63 as libc::c_int;
    let mut heads: libc::c_int = 255 as libc::c_int;
    let mut cylinders: libc::c_int = 1024 as libc::c_int;
    sector = lba % sectors + 1 as libc::c_int;
    head = lba / sectors % heads;
    cylinder = lba / (sectors * heads);
    if cylinder >= cylinders {
        *dh = 0xff as libc::c_int as uint8_t;
        *ch = *dh;
        *cl = *ch;
        return
    }
    *cl =
        (sector | (cylinder & 0x300 as libc::c_int) >> 2 as libc::c_int) as
            uint8_t;
    *ch = (cylinder & 0xff as libc::c_int) as uint8_t;
    *dh = head as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn emfat_init(mut emfat: *mut emfat_t,
                                    mut label: *const libc::c_char,
                                    mut entries: *mut emfat_entry_t) -> bool {
    let mut sect_per_fat: uint32_t = 0;
    let mut clust: uint32_t = 0;
    let mut reserved_clust: uint32_t = 0 as libc::c_int as uint32_t;
    let mut e: *mut emfat_entry_t = 0 as *mut emfat_entry_t;
    let mut i: libc::c_int = 0;
    if emfat.is_null() || label.is_null() || entries.is_null() {
        return 0 as libc::c_int != 0
    }
    if !emfat_init_entries(entries) { return 0 as libc::c_int != 0 }
    clust = 2 as libc::c_int as uint32_t;
    i = 0 as libc::c_int;
    while !(*entries.offset(i as isize)).name.is_null() {
        e = &mut *entries.offset(i as isize) as *mut emfat_entry_t;
        if (*e).dir {
            (*e).curr_size = 0 as libc::c_int as uint32_t;
            (*e).max_size = 0 as libc::c_int as uint32_t;
            (*e).priv_0.first_clust = clust;
            (*e).priv_0.last_clust =
                (clust as
                     libc::c_ulong).wrapping_add((if ((*e).priv_0.num_subentry
                                                          as
                                                          libc::c_ulong).wrapping_mul(::core::mem::size_of::<dir_entry>()
                                                                                          as
                                                                                          libc::c_ulong)
                                                         ==
                                                         0 as libc::c_int as
                                                             libc::c_ulong {
                                                      1 as libc::c_int as
                                                          libc::c_ulong
                                                  } else {
                                                      ((*e).priv_0.num_subentry
                                                           as
                                                           libc::c_ulong).wrapping_mul(::core::mem::size_of::<dir_entry>()
                                                                                           as
                                                                                           libc::c_ulong).wrapping_add(4096
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           libc::c_ulong).wrapping_sub(1
                                                                                                                                                           as
                                                                                                                                                           libc::c_int
                                                                                                                                                           as
                                                                                                                                                           libc::c_ulong).wrapping_div(4096
                                                                                                                                                                                           as
                                                                                                                                                                                           libc::c_int
                                                                                                                                                                                           as
                                                                                                                                                                                           libc::c_ulong)
                                                  })).wrapping_sub(1 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_ulong)
                    as uint32_t;
            (*e).priv_0.last_reserved = (*e).priv_0.last_clust
        } else {
            (*e).priv_0.first_clust = clust;
            (*e).priv_0.last_clust =
                (*e).priv_0.first_clust.wrapping_add((if (*entries.offset(i as
                                                                              isize)).curr_size
                                                             ==
                                                             0 as libc::c_int
                                                                 as
                                                                 libc::c_uint
                                                         {
                                                          1 as libc::c_int as
                                                              libc::c_uint
                                                      } else {
                                                          (*entries.offset(i
                                                                               as
                                                                               isize)).curr_size.wrapping_add(4096
                                                                                                                  as
                                                                                                                  libc::c_int
                                                                                                                  as
                                                                                                                  libc::c_uint).wrapping_sub(1
                                                                                                                                                 as
                                                                                                                                                 libc::c_int
                                                                                                                                                 as
                                                                                                                                                 libc::c_uint).wrapping_div(4096
                                                                                                                                                                                as
                                                                                                                                                                                libc::c_int
                                                                                                                                                                                as
                                                                                                                                                                                libc::c_uint)
                                                      })).wrapping_sub(1 as
                                                                           libc::c_int
                                                                           as
                                                                           libc::c_uint);
            (*e).priv_0.last_reserved =
                (*e).priv_0.first_clust.wrapping_add((if (*entries.offset(i as
                                                                              isize)).max_size
                                                             ==
                                                             0 as libc::c_int
                                                                 as
                                                                 libc::c_uint
                                                         {
                                                          1 as libc::c_int as
                                                              libc::c_uint
                                                      } else {
                                                          (*entries.offset(i
                                                                               as
                                                                               isize)).max_size.wrapping_add(4096
                                                                                                                 as
                                                                                                                 libc::c_int
                                                                                                                 as
                                                                                                                 libc::c_uint).wrapping_sub(1
                                                                                                                                                as
                                                                                                                                                libc::c_int
                                                                                                                                                as
                                                                                                                                                libc::c_uint).wrapping_div(4096
                                                                                                                                                                               as
                                                                                                                                                                               libc::c_int
                                                                                                                                                                               as
                                                                                                                                                                               libc::c_uint)
                                                      })).wrapping_sub(1 as
                                                                           libc::c_int
                                                                           as
                                                                           libc::c_uint)
        }
        reserved_clust =
            (reserved_clust as
                 libc::c_uint).wrapping_add((*e).priv_0.last_reserved.wrapping_sub((*e).priv_0.last_clust))
                as uint32_t as uint32_t;
        clust =
            (*e).priv_0.last_reserved.wrapping_add(1 as libc::c_int as
                                                       libc::c_uint);
        i += 1
    }
    clust =
        (clust as libc::c_uint).wrapping_sub(2 as libc::c_int as libc::c_uint)
            as uint32_t as uint32_t;
    (*emfat).vol_label = label;
    (*emfat).priv_0.num_entries = i;
    (*emfat).priv_0.boot_lba = 62 as libc::c_int as uint32_t;
    (*emfat).priv_0.fsinfo_lba =
        (*emfat).priv_0.boot_lba.wrapping_add(1 as libc::c_int as
                                                  libc::c_uint);
    (*emfat).priv_0.fat1_lba =
        (*emfat).priv_0.fsinfo_lba.wrapping_add(1 as libc::c_int as
                                                    libc::c_uint);
    (*emfat).priv_0.num_clust = clust;
    (*emfat).priv_0.free_clust = reserved_clust;
    sect_per_fat =
        if ((*emfat).priv_0.num_clust as
                uint64_t).wrapping_mul(4 as libc::c_int as libc::c_ulong) ==
               0 as libc::c_int as libc::c_ulong {
            1 as libc::c_int as libc::c_ulong
        } else {
            ((*emfat).priv_0.num_clust as
                 uint64_t).wrapping_mul(4 as libc::c_int as
                                            libc::c_ulong).wrapping_add(512 as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_ulong).wrapping_sub(1
                                                                                                            as
                                                                                                            libc::c_int
                                                                                                            as
                                                                                                            libc::c_ulong).wrapping_div(512
                                                                                                                                            as
                                                                                                                                            libc::c_int
                                                                                                                                            as
                                                                                                                                            libc::c_ulong)
        } as uint32_t;
    (*emfat).priv_0.fat2_lba =
        (*emfat).priv_0.fat1_lba.wrapping_add(sect_per_fat);
    (*emfat).priv_0.root_lba =
        (*emfat).priv_0.fat2_lba.wrapping_add(sect_per_fat);
    (*emfat).priv_0.entries = entries;
    (*emfat).priv_0.last_entry = entries;
    (*emfat).disk_sectors =
        clust.wrapping_mul((4096 as libc::c_int / 512 as libc::c_int) as
                               libc::c_uint).wrapping_add((*emfat).priv_0.root_lba);
    (*emfat).vol_size =
        ((*emfat).disk_sectors as
             uint64_t).wrapping_mul(512 as libc::c_int as libc::c_ulong);
    /* calc cyl number */
//    i = ((emfat->disk_sectors + 63*255 - 1) / (63*255));
//    emfat->disk_sectors = i * 63*255;
    return 1 as libc::c_int != 0; /* 4 kb per cluster */
}
#[no_mangle]
pub unsafe extern "C" fn read_mbr_sector(mut emfat: *const emfat_t,
                                         mut sect: *mut uint8_t) {
    let mut mbr: *mut mbr_t =
        0 as *mut mbr_t; /* boot sector & fsinfo sector */
    memset(sect as *mut libc::c_void, 0 as libc::c_int,
           512 as libc::c_int as libc::c_ulong); /* two tables */
    mbr = sect as *mut mbr_t; /* not used */
    (*mbr).DiskSig = 0 as libc::c_int as uint32_t;
    (*mbr).Reserved = 0 as libc::c_int as uint16_t;
    (*mbr).PartTable[0 as libc::c_int as usize].status =
        0x80 as libc::c_int as uint8_t;
    (*mbr).PartTable[0 as libc::c_int as usize].PartType =
        0xc as libc::c_int as uint8_t;
    (*mbr).PartTable[0 as libc::c_int as usize].StartLBA =
        (*emfat).priv_0.boot_lba;
    (*mbr).PartTable[0 as libc::c_int as usize].EndLBA =
        (*emfat).disk_sectors;
    lba_to_chs((*mbr).PartTable[0 as libc::c_int as usize].StartLBA as
                   libc::c_int,
               &mut (*(*mbr).PartTable.as_mut_ptr().offset(0 as libc::c_int as
                                                               isize)).start_sector,
               &mut (*(*mbr).PartTable.as_mut_ptr().offset(0 as libc::c_int as
                                                               isize)).start_cylinder,
               &mut (*(*mbr).PartTable.as_mut_ptr().offset(0 as libc::c_int as
                                                               isize)).start_head);
    lba_to_chs((*emfat).disk_sectors.wrapping_sub(1 as libc::c_int as
                                                      libc::c_uint) as
                   libc::c_int,
               &mut (*(*mbr).PartTable.as_mut_ptr().offset(0 as libc::c_int as
                                                               isize)).end_sector,
               &mut (*(*mbr).PartTable.as_mut_ptr().offset(0 as libc::c_int as
                                                               isize)).end_cylinder,
               &mut (*(*mbr).PartTable.as_mut_ptr().offset(0 as libc::c_int as
                                                               isize)).end_head);
    (*mbr).BootSignature[0 as libc::c_int as usize] =
        0x55 as libc::c_int as uint8_t;
    (*mbr).BootSignature[1 as libc::c_int as usize] =
        0xaa as libc::c_int as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn read_boot_sector(mut emfat: *const emfat_t,
                                          mut sect: *mut uint8_t) {
    let mut bs: *mut boot_sector = 0 as *mut boot_sector;
    memset(sect as *mut libc::c_void, 0 as libc::c_int,
           512 as libc::c_int as libc::c_ulong);
    bs = sect as *mut boot_sector;
    (*bs).jump[0 as libc::c_int as usize] = 0xeb as libc::c_int as uint8_t;
    (*bs).jump[1 as libc::c_int as usize] = 0x58 as libc::c_int as uint8_t;
    (*bs).jump[2 as libc::c_int as usize] = 0x90 as libc::c_int as uint8_t;
    memcpy((*bs).OEM_name.as_mut_ptr() as *mut libc::c_void,
           b"MSDOS5.0\x00" as *const u8 as *const libc::c_char as
               *const libc::c_void, 8 as libc::c_int as libc::c_ulong);
    (*bs).bytes_per_sec = 512 as libc::c_int as uint16_t;
    (*bs).sec_per_clus = 8 as libc::c_int as uint8_t;
    (*bs).reserved_sec_cnt = 2 as libc::c_int as uint16_t;
    (*bs).fat_cnt = 2 as libc::c_int as uint8_t;
    (*bs).root_dir_max_cnt = 0 as libc::c_int as uint16_t;
    (*bs).tot_sectors = 0 as libc::c_int as uint16_t;
    (*bs).media_desc = 0xf8 as libc::c_int as uint8_t;
    (*bs).sec_per_fat_fat16 = 0 as libc::c_int as uint16_t;
    (*bs).sec_per_track = 63 as libc::c_int as uint16_t;
    (*bs).number_of_heads = 0xff as libc::c_int as uint16_t;
    (*bs).hidden_sec_cnt = 62 as libc::c_int as uint32_t;
    (*bs).tol_sector_cnt =
        (*emfat).disk_sectors.wrapping_sub((*emfat).priv_0.boot_lba);
    (*bs).sectors_per_fat =
        (*emfat).priv_0.fat2_lba.wrapping_sub((*emfat).priv_0.fat1_lba);
    (*bs).ext_flags = 0 as libc::c_int as uint16_t;
    (*bs).fs_version[0 as libc::c_int as usize] = 0 as libc::c_int as uint8_t;
    (*bs).fs_version[1 as libc::c_int as usize] = 0 as libc::c_int as uint8_t;
    (*bs).root_dir_strt_cluster = 2 as libc::c_int as uint32_t;
    (*bs).fs_info_sector = 1 as libc::c_int as uint16_t;
    (*bs).backup_boot_sector = 0 as libc::c_int as uint16_t;
    (*bs).drive_number = 128 as libc::c_int as uint8_t;
    (*bs).boot_sig = 0x29 as libc::c_int as uint8_t;
    (*bs).volume_id[0 as libc::c_int as usize] =
        148 as libc::c_int as uint8_t;
    (*bs).volume_id[1 as libc::c_int as usize] = 14 as libc::c_int as uint8_t;
    (*bs).volume_id[2 as libc::c_int as usize] = 13 as libc::c_int as uint8_t;
    (*bs).volume_id[3 as libc::c_int as usize] = 8 as libc::c_int as uint8_t;
    memcpy((*bs).volume_label.as_mut_ptr() as *mut libc::c_void,
           b"NO NAME     \x00" as *const u8 as *const libc::c_char as
               *const libc::c_void, 12 as libc::c_int as libc::c_ulong);
    memcpy((*bs).file_system_type.as_mut_ptr() as *mut libc::c_void,
           b"FAT32   \x00" as *const u8 as *const libc::c_char as
               *const libc::c_void, 8 as libc::c_int as libc::c_ulong);
    *sect.offset((512 as libc::c_int - 2 as libc::c_int) as isize) =
        0x55 as libc::c_int as uint8_t;
    *sect.offset((512 as libc::c_int - 1 as libc::c_int) as isize) =
        0xaa as libc::c_int as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn find_entry(mut emfat: *const emfat_t,
                                    mut clust: uint32_t,
                                    mut nearest: *mut emfat_entry_t)
 -> *mut emfat_entry_t {
    if nearest.is_null() { nearest = (*emfat).priv_0.entries }
    if (*nearest).priv_0.first_clust > clust {
        while nearest >= (*emfat).priv_0.entries {
            // backward finding
            if clust >= (*nearest).priv_0.first_clust &&
                   clust <= (*nearest).priv_0.last_reserved {
                return nearest
            }
            nearest = nearest.offset(-1)
        }
    } else {
        while !(*nearest).name.is_null() {
            // forward finding
            if clust >= (*nearest).priv_0.first_clust &&
                   clust <= (*nearest).priv_0.last_reserved {
                return nearest
            }
            nearest = nearest.offset(1)
        }
    }
    return 0 as *mut emfat_entry_t;
}
#[no_mangle]
pub unsafe extern "C" fn read_fsinfo_sector(mut emfat: *const emfat_t,
                                            mut sect: *mut uint8_t) {
    let mut info: *mut fsinfo_t = sect as *mut fsinfo_t;
    (*info).signature1 = 0x41615252 as libc::c_long as uint32_t;
    (*info).signature2 = 0x61417272 as libc::c_long as uint32_t;
    //info->free_clusters = 0;
    (*info).free_clusters = (*emfat).priv_0.free_clust;
    //info->next_cluster = emfat->priv.num_clust + 2;
    (*info).next_cluster =
        0xffffffff as libc::c_uint; // 2. not a first sector
    memset((*info).reserved1.as_mut_ptr() as *mut libc::c_void,
           0 as libc::c_int,
           ::core::mem::size_of::<[uint32_t; 120]>() as libc::c_ulong);
    memset((*info).reserved2.as_mut_ptr() as *mut libc::c_void,
           0 as libc::c_int,
           ::core::mem::size_of::<[uint32_t; 3]>() as libc::c_ulong);
    (*info).signature3 = 0xaa550000 as libc::c_uint;
}
#[no_mangle]
pub unsafe extern "C" fn read_fat_sector(mut emfat: *mut emfat_t,
                                         mut sect: *mut uint8_t,
                                         mut index: uint32_t) {
    let mut le: *mut emfat_entry_t = 0 as *mut emfat_entry_t;
    let mut values: *mut uint32_t = 0 as *mut uint32_t;
    let mut count: uint32_t = 0;
    let mut curr: uint32_t = 0;
    values = sect as *mut uint32_t;
    curr = index.wrapping_mul(128 as libc::c_int as libc::c_uint);
    count = 128 as libc::c_int as uint32_t;
    if curr == 0 as libc::c_int as libc::c_uint {
        let fresh5 = values;
        values = values.offset(1);
        *fresh5 = 0xffffff8 as libc::c_int as uint32_t;
        let fresh6 = values;
        values = values.offset(1);
        *fresh6 = 0xffffffff as libc::c_uint;
        count =
            (count as
                 libc::c_uint).wrapping_sub(2 as libc::c_int as libc::c_uint)
                as uint32_t as uint32_t;
        curr =
            (curr as
                 libc::c_uint).wrapping_add(2 as libc::c_int as libc::c_uint)
                as uint32_t as uint32_t
    }
    le = (*emfat).priv_0.last_entry;
    while count != 0 as libc::c_int as libc::c_uint {
        if !(curr >= (*le).priv_0.first_clust &&
                 curr <= (*le).priv_0.last_reserved) {
            le = find_entry(emfat, curr, le);
            if le.is_null() {
                le = (*emfat).priv_0.last_entry;
                *values = 0x1 as libc::c_int as uint32_t;
                values = values.offset(1);
                count = count.wrapping_sub(1);
                curr = curr.wrapping_add(1);
                continue ;
            }
        }
        if (*le).dir {
            if curr == (*le).priv_0.last_clust {
                *values = 0xfffffff as libc::c_int as uint32_t
            } else {
                *values = curr.wrapping_add(1 as libc::c_int as libc::c_uint)
            }
        } else if curr == (*le).priv_0.last_clust {
            *values = 0xfffffff as libc::c_int as uint32_t
        } else if curr > (*le).priv_0.last_clust {
            *values = 0 as libc::c_int as uint32_t
        } else {
            *values = curr.wrapping_add(1 as libc::c_int as libc::c_uint)
        }
        values = values.offset(1);
        count = count.wrapping_sub(1);
        curr = curr.wrapping_add(1)
    }
    (*emfat).priv_0.last_entry = le;
}
#[no_mangle]
pub unsafe extern "C" fn fill_entry(mut entry: *mut dir_entry,
                                    mut name: *const libc::c_char,
                                    mut attr: uint8_t, mut clust: uint32_t,
                                    mut cma: *const uint32_t,
                                    mut size: uint32_t) {
    let mut i: libc::c_int = 0;
    let mut l: libc::c_int = 0;
    let mut l1: libc::c_int = 0;
    let mut l2: libc::c_int = 0;
    let mut dot_pos: libc::c_int = 0;
    memset(entry as *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<dir_entry>() as libc::c_ulong);
    if !cma.is_null() {
        (*entry).crt_date =
            (*cma.offset(0 as libc::c_int as isize) >> 16 as libc::c_int) as
                uint16_t;
        (*entry).crt_time =
            (*cma.offset(0 as libc::c_int as isize) &
                 0xffff as libc::c_int as libc::c_uint) as uint16_t;
        (*entry).lst_mod_date =
            (*cma.offset(1 as libc::c_int as isize) >> 16 as libc::c_int) as
                uint16_t;
        (*entry).lst_mod_time =
            (*cma.offset(1 as libc::c_int as isize) &
                 0xffff as libc::c_int as libc::c_uint) as uint16_t;
        (*entry).lst_access_date =
            (*cma.offset(2 as libc::c_int as isize) >> 16 as libc::c_int) as
                uint16_t
    }
    l = strlen(name) as libc::c_int;
    dot_pos = -(1 as libc::c_int);
    if attr as libc::c_int & 0x10 as libc::c_int == 0 as libc::c_int {
        i = l - 1 as libc::c_int;
        while i >= 0 as libc::c_int {
            if *name.offset(i as isize) as libc::c_int == '.' as i32 {
                dot_pos = i;
                break ;
            } else { i -= 1 }
        }
    }
    if dot_pos == -(1 as libc::c_int) {
        l1 = if l > 8 as libc::c_int { 8 as libc::c_int } else { l };
        l2 = 0 as libc::c_int
    } else {
        l1 = dot_pos;
        l1 = if l1 > 8 as libc::c_int { 8 as libc::c_int } else { l1 };
        l2 = l - dot_pos - 1 as libc::c_int;
        l2 = if l2 > 3 as libc::c_int { 3 as libc::c_int } else { l2 }
    }
    memset((*entry).name.as_mut_ptr() as *mut libc::c_void, ' ' as i32,
           (8 as libc::c_int + 3 as libc::c_int) as libc::c_ulong);
    memcpy((*entry).name.as_mut_ptr() as *mut libc::c_void,
           name as *const libc::c_void, l1 as libc::c_ulong);
    memcpy((*entry).extn.as_mut_ptr() as *mut libc::c_void,
           name.offset(dot_pos as isize).offset(1 as libc::c_int as isize) as
               *const libc::c_void, l2 as libc::c_ulong);
    i = 0 as libc::c_int;
    while i < 8 as libc::c_int {
        if (*entry).name[i as usize] as libc::c_int >= 'a' as i32 &&
               (*entry).name[i as usize] as libc::c_int <= 'z' as i32 {
            (*entry).name[i as usize] =
                ((*entry).name[i as usize] as libc::c_int -
                     0x20 as libc::c_int) as uint8_t
        }
        i += 1
    }
    i = 0 as libc::c_int;
    while i < 3 as libc::c_int {
        if (*entry).extn[i as usize] as libc::c_int >= 'a' as i32 &&
               (*entry).extn[i as usize] as libc::c_int <= 'z' as i32 {
            (*entry).extn[i as usize] =
                ((*entry).extn[i as usize] as libc::c_int -
                     0x20 as libc::c_int) as uint8_t
        }
        i += 1
    }
    (*entry).attr = attr;
    (*entry).reserved = 24 as libc::c_int as uint8_t;
    (*entry).strt_clus_hword = (clust >> 16 as libc::c_int) as uint16_t;
    (*entry).strt_clus_lword = clust as uint16_t;
    (*entry).size = size;
}
#[no_mangle]
pub unsafe extern "C" fn fill_dir_sector(mut emfat: *mut emfat_t,
                                         mut data: *mut uint8_t,
                                         mut entry: *mut emfat_entry_t,
                                         mut rel_sect: uint32_t) {
    let mut de: *mut dir_entry = 0 as *mut dir_entry;
    let mut avail: uint32_t = 0;
    memset(data as *mut libc::c_void, 0 as libc::c_int,
           512 as libc::c_int as libc::c_ulong);
    de = data as *mut dir_entry;
    avail = 512 as libc::c_int as uint32_t;
    if rel_sect == 0 as libc::c_int as libc::c_uint {
        // 1. first sector of directory
        if (*entry).priv_0.top.is_null() {
            let fresh7 = de;
            de = de.offset(1);
            fill_entry(fresh7, (*emfat).vol_label,
                       0x8 as libc::c_int as uint8_t,
                       0 as libc::c_int as uint32_t, 0 as *const uint32_t,
                       0 as libc::c_int as uint32_t);
            avail =
                (avail as
                     libc::c_ulong).wrapping_sub(::core::mem::size_of::<dir_entry>()
                                                     as libc::c_ulong) as
                    uint32_t as uint32_t
        } else {
            let fresh8 = de;
            de = de.offset(1);
            fill_entry(fresh8, b".\x00" as *const u8 as *const libc::c_char,
                       (0x10 as libc::c_int | 0x1 as libc::c_int) as uint8_t,
                       (*entry).priv_0.first_clust, 0 as *const uint32_t,
                       0 as libc::c_int as uint32_t);
            if (*(*entry).priv_0.top).priv_0.top.is_null() {
                let fresh9 = de;
                de = de.offset(1);
                fill_entry(fresh9,
                           b"..\x00" as *const u8 as *const libc::c_char,
                           (0x10 as libc::c_int | 0x1 as libc::c_int) as
                               uint8_t, 0 as libc::c_int as uint32_t,
                           0 as *const uint32_t,
                           0 as libc::c_int as uint32_t);
            } else {
                let fresh10 = de;
                de = de.offset(1);
                fill_entry(fresh10,
                           b"..\x00" as *const u8 as *const libc::c_char,
                           (0x10 as libc::c_int | 0x1 as libc::c_int) as
                               uint8_t,
                           (*(*entry).priv_0.top).priv_0.first_clust,
                           0 as *const uint32_t,
                           0 as libc::c_int as uint32_t);
            }
            avail =
                (avail as
                     libc::c_ulong).wrapping_sub((::core::mem::size_of::<dir_entry>()
                                                      as
                                                      libc::c_ulong).wrapping_mul(2
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_ulong))
                    as uint32_t as uint32_t
        }
        entry = (*entry).priv_0.sub
    } else {
        let mut n: libc::c_int = 0;
        n =
            (rel_sect as
                 libc::c_ulong).wrapping_mul((512 as libc::c_int as
                                                  libc::c_ulong).wrapping_div(::core::mem::size_of::<dir_entry>()
                                                                                  as
                                                                                  libc::c_ulong))
                as libc::c_int;
        n -=
            if (*entry).priv_0.top.is_null() {
                1 as libc::c_int
            } else { 2 as libc::c_int };
        entry = (*entry).priv_0.sub;
        while n > 0 as libc::c_int && !entry.is_null() {
            entry = (*entry).priv_0.next;
            n -= 1
        }
    }
    while !entry.is_null() &&
              avail as libc::c_ulong >=
                  ::core::mem::size_of::<dir_entry>() as libc::c_ulong {
        if (*entry).dir {
            let fresh11 = de;
            de = de.offset(1);
            fill_entry(fresh11, (*entry).name,
                       (0x10 as libc::c_int | 0x1 as libc::c_int) as uint8_t,
                       (*entry).priv_0.first_clust,
                       (*entry).cma_time.as_mut_ptr() as *const uint32_t,
                       0 as libc::c_int as uint32_t);
        } else {
            //fill_entry(de++, entry->name, ATTR_ARCHIVE | ATTR_READ, entry->priv.first_clust, entry->cma_time, entry->curr_size);
            let fresh12 = de;
            de = de.offset(1);
            fill_entry(fresh12, (*entry).name,
                       (0x20 as libc::c_int | 0x1 as libc::c_int |
                            (*entry).attr as libc::c_int) as uint8_t,
                       (*entry).priv_0.first_clust,
                       (*entry).cma_time.as_mut_ptr() as *const uint32_t,
                       (*entry).curr_size);
        }
        entry = (*entry).priv_0.next;
        avail =
            (avail as
                 libc::c_ulong).wrapping_sub(::core::mem::size_of::<dir_entry>()
                                                 as libc::c_ulong) as uint32_t
                as uint32_t
    };
}
#[no_mangle]
pub unsafe extern "C" fn read_data_sector(mut emfat: *mut emfat_t,
                                          mut data: *mut uint8_t,
                                          mut rel_sect: uint32_t) {
    let mut le: *mut emfat_entry_t = 0 as *mut emfat_entry_t;
    let mut cluster: uint32_t = 0;
    cluster =
        rel_sect.wrapping_div(8 as libc::c_int as
                                  libc::c_uint).wrapping_add(2 as libc::c_int
                                                                 as
                                                                 libc::c_uint);
    rel_sect = rel_sect.wrapping_rem(8 as libc::c_int as libc::c_uint);
    le = (*emfat).priv_0.last_entry;
    if !(cluster >= (*le).priv_0.first_clust &&
             cluster <= (*le).priv_0.last_reserved) {
        le = find_entry(emfat, cluster, le);
        if le.is_null() {
            let mut i: libc::c_int = 0;
            i = 0 as libc::c_int;
            while i < 512 as libc::c_int / 4 as libc::c_int {
                *(data as *mut uint32_t).offset(i as isize) =
                    0xefbeadde as libc::c_uint;
                i += 1
            }
            return
        }
        (*emfat).priv_0.last_entry = le
    }
    if (*le).dir { fill_dir_sector(emfat, data, le, rel_sect); return }
    if (*le).readcb.is_none() {
        memset(data as *mut libc::c_void, 0 as libc::c_int,
               512 as libc::c_int as libc::c_ulong);
    } else {
        let mut offset: uint32_t =
            cluster.wrapping_sub((*le).priv_0.first_clust);
        offset =
            offset.wrapping_mul(4096 as libc::c_int as
                                    libc::c_uint).wrapping_add(rel_sect.wrapping_mul(512
                                                                                         as
                                                                                         libc::c_int
                                                                                         as
                                                                                         libc::c_uint));
        (*le).readcb.expect("non-null function pointer")(data,
                                                         512 as libc::c_int,
                                                         offset.wrapping_add((*le).offset),
                                                         le);
    };
}
#[no_mangle]
pub unsafe extern "C" fn emfat_read(mut emfat: *mut emfat_t,
                                    mut data: *mut uint8_t,
                                    mut sector: uint32_t,
                                    mut num_sectors: libc::c_int) {
    while num_sectors > 0 as libc::c_int {
        if sector >= (*emfat).priv_0.root_lba {
            read_data_sector(emfat, data,
                             sector.wrapping_sub((*emfat).priv_0.root_lba));
        } else if sector == 0 as libc::c_int as libc::c_uint {
            read_mbr_sector(emfat, data);
        } else if sector == (*emfat).priv_0.fsinfo_lba {
            read_fsinfo_sector(emfat, data);
        } else if sector == (*emfat).priv_0.boot_lba {
            read_boot_sector(emfat, data);
        } else if sector >= (*emfat).priv_0.fat1_lba &&
                      sector < (*emfat).priv_0.fat2_lba {
            read_fat_sector(emfat, data,
                            sector.wrapping_sub((*emfat).priv_0.fat1_lba));
        } else if sector >= (*emfat).priv_0.fat2_lba &&
                      sector < (*emfat).priv_0.root_lba {
            read_fat_sector(emfat, data,
                            sector.wrapping_sub((*emfat).priv_0.fat2_lba));
        } else {
            memset(data as *mut libc::c_void, 0 as libc::c_int,
                   512 as libc::c_int as libc::c_ulong);
        }
        data = data.offset(512 as libc::c_int as isize);
        num_sectors -= 1;
        sector = sector.wrapping_add(1)
    };
}
#[no_mangle]
pub unsafe extern "C" fn write_data_sector(mut emfat: *mut emfat_t,
                                           mut data: *const uint8_t,
                                           mut rel_sect: uint32_t) {
    let mut le: *mut emfat_entry_t = 0 as *mut emfat_entry_t;
    let mut cluster: uint32_t = 0;
    cluster =
        rel_sect.wrapping_div(8 as libc::c_int as
                                  libc::c_uint).wrapping_add(2 as libc::c_int
                                                                 as
                                                                 libc::c_uint);
    rel_sect = rel_sect.wrapping_rem(8 as libc::c_int as libc::c_uint);
    le = (*emfat).priv_0.last_entry;
    if !(cluster >= (*le).priv_0.first_clust &&
             cluster <= (*le).priv_0.last_reserved) {
        le = find_entry(emfat, cluster, le);
        if le.is_null() { return }
        (*emfat).priv_0.last_entry = le
    }
    if (*le).dir {
        // TODO: handle changing a filesize
        return
    }
    if (*le).writecb.is_some() {
        (*le).writecb.expect("non-null function pointer")(data,
                                                          512 as libc::c_int,
                                                          rel_sect.wrapping_mul(512
                                                                                    as
                                                                                    libc::c_int
                                                                                    as
                                                                                    libc::c_uint).wrapping_add((*le).offset),
                                                          le);
    };
}
static mut month_days: [libc::c_int; 12] =
    [31 as libc::c_int, 28 as libc::c_int, 31 as libc::c_int,
     30 as libc::c_int, 31 as libc::c_int, 30 as libc::c_int,
     31 as libc::c_int, 31 as libc::c_int, 30 as libc::c_int,
     31 as libc::c_int, 30 as libc::c_int, 31 as libc::c_int];
#[no_mangle]
pub unsafe extern "C" fn emfat_cma_time_from_unix(mut tim: uint32_t)
 -> uint32_t {
    let mut i: libc::c_int = 0;
    let mut tmp: libc::c_long = 0;
    let mut day: libc::c_long = 0;
    let mut ymd: [libc::c_int; 3] = [0; 3];
    let mut hms: [libc::c_int; 3] = [0; 3];
    day = tim as libc::c_long / 86400 as libc::c_long;
    tmp = tim as libc::c_long % 86400 as libc::c_long;
    /* Hours, minutes, seconds are easy */
    hms[0 as libc::c_int as usize] =
        (tmp / 3600 as libc::c_int as libc::c_long) as libc::c_int;
    hms[1 as libc::c_int as usize] =
        (tmp % 3600 as libc::c_int as libc::c_long /
             60 as libc::c_int as libc::c_long) as libc::c_int;
    hms[2 as libc::c_int as usize] =
        (tmp % 3600 as libc::c_int as libc::c_long %
             60 as libc::c_int as libc::c_long) as libc::c_int;
    /* Number of years in days */
    i = 1970 as libc::c_int;
    while day >=
              (if i % 4 as libc::c_int == 0 as libc::c_int {
                   366 as libc::c_int
               } else { 365 as libc::c_int }) as libc::c_long {
        day -=
            if i % 4 as libc::c_int == 0 as libc::c_int {
                366 as libc::c_int
            } else { 365 as libc::c_int } as libc::c_long;
        i += 1
    }
    ymd[0 as libc::c_int as usize] = i;
    /* Number of months in days left */
    if ymd[0 as libc::c_int as usize] % 4 as libc::c_int == 0 as libc::c_int {
        month_days[(2 as libc::c_int - 1 as libc::c_int) as usize] =
            29 as libc::c_int
    }
    i = 1 as libc::c_int;
    while day >= month_days[(i - 1 as libc::c_int) as usize] as libc::c_long {
        day -= month_days[(i - 1 as libc::c_int) as usize] as libc::c_long;
        i += 1
    }
    month_days[(2 as libc::c_int - 1 as libc::c_int) as usize] =
        28 as libc::c_int;
    ymd[1 as libc::c_int as usize] = i;
    /* Days are what is left over (+1) from all that. */
    ymd[2 as libc::c_int as usize] =
        (day + 1 as libc::c_int as libc::c_long) as libc::c_int;
    return (((ymd[0 as libc::c_int as usize] - 1980 as libc::c_int) <<
                 9 as libc::c_int |
                 ymd[1 as libc::c_int as usize] << 5 as libc::c_int |
                 ymd[2 as libc::c_int as usize]) << 16 as libc::c_int |
                (hms[0 as libc::c_int as usize] << 11 as libc::c_int |
                     hms[1 as libc::c_int as usize] << 5 as libc::c_int |
                     hms[2 as libc::c_int as usize] >> 1 as libc::c_int)) as
               uint32_t;
}
