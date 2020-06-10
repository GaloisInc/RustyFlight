use ::libc;
use ::c2rust_bitfields;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn strcmp(_: *const libc::c_char, _: *const libc::c_char) -> libc::c_int;
    #[no_mangle]
    fn strncmp(_: *const libc::c_char, _: *const libc::c_char,
               _: libc::c_ulong) -> libc::c_int;
    #[no_mangle]
    fn fat32_decodeClusterNumber(clusterNumber: uint32_t) -> uint32_t;
    #[no_mangle]
    fn fat32_isEndOfChainMarker(clusterNumber: uint32_t) -> bool;
    #[no_mangle]
    fn fat16_isEndOfChainMarker(clusterNumber: uint16_t) -> bool;
    #[no_mangle]
    fn fat_isFreeSpace(clusterNumber: uint32_t) -> bool;
    #[no_mangle]
    fn fat_isDirectoryEntryTerminator(entry: *mut fatDirectoryEntry_t)
     -> bool;
    #[no_mangle]
    fn fat_isDirectoryEntryEmpty(entry: *mut fatDirectoryEntry_t) -> bool;
    #[no_mangle]
    fn fat_convertFilenameToFATStyle(filename: *const libc::c_char,
                                     fatFilename: *mut uint8_t);
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
    /* Card capacity in 512-byte blocks*/
    #[no_mangle]
    fn sdcard_readBlock(blockIndex: uint32_t, buffer: *mut uint8_t,
                        callback: sdcard_operationCompleteCallback_c,
                        callbackData: uint32_t) -> bool;
    #[no_mangle]
    fn sdcard_beginWriteBlocks(blockIndex: uint32_t, blockCount: uint32_t)
     -> sdcardOperationStatus_e;
    #[no_mangle]
    fn sdcard_writeBlock(blockIndex: uint32_t, buffer: *mut uint8_t,
                         callback: sdcard_operationCompleteCallback_c,
                         callbackData: uint32_t) -> sdcardOperationStatus_e;
    #[no_mangle]
    fn sdcard_poll() -> bool;
    #[no_mangle]
    fn rtcGetDateTime(dt: *mut dateTime_t) -> bool;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type __uint64_t = libc::c_ulong;
pub type int8_t = __int8_t;
pub type int16_t = __int16_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type uint64_t = __uint64_t;
pub type fatFilesystemType_e = libc::c_uint;
pub const FAT_FILESYSTEM_TYPE_FAT32: fatFilesystemType_e = 3;
pub const FAT_FILESYSTEM_TYPE_FAT16: fatFilesystemType_e = 2;
pub const FAT_FILESYSTEM_TYPE_FAT12: fatFilesystemType_e = 1;
pub const FAT_FILESYSTEM_TYPE_INVALID: fatFilesystemType_e = 0;
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct mbrPartitionEntry_t {
    pub bootFlag: uint8_t,
    pub chsBegin: [uint8_t; 3],
    pub type_0: uint8_t,
    pub chsEnd: [uint8_t; 3],
    pub lbaBegin: uint32_t,
    pub numSectors: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct fat16Descriptor_t {
    pub driveNumber: uint8_t,
    pub reserved1: uint8_t,
    pub bootSignature: uint8_t,
    pub volumeID: uint32_t,
    pub volumeLabel: [libc::c_char; 11],
    pub fileSystemType: [libc::c_char; 8],
}
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct fat32Descriptor_t {
    pub FATSize32: uint32_t,
    pub extFlags: uint16_t,
    pub fsVer: uint16_t,
    pub rootCluster: uint32_t,
    pub fsInfo: uint16_t,
    pub backupBootSector: uint16_t,
    pub reserved: [uint8_t; 12],
    pub driveNumber: uint8_t,
    pub reserved1: uint8_t,
    pub bootSignature: uint8_t,
    pub volumeID: uint32_t,
    pub volumeLabel: [libc::c_char; 11],
    pub fileSystemType: [libc::c_char; 8],
}
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct fatVolumeID_t {
    pub jmpBoot: [uint8_t; 3],
    pub oemName: [libc::c_char; 8],
    pub bytesPerSector: uint16_t,
    pub sectorsPerCluster: uint8_t,
    pub reservedSectorCount: uint16_t,
    pub numFATs: uint8_t,
    pub rootEntryCount: uint16_t,
    pub totalSectors16: uint16_t,
    pub media: uint8_t,
    pub FATSize16: uint16_t,
    pub sectorsPerTrack: uint16_t,
    pub numHeads: uint16_t,
    pub hiddenSectors: uint32_t,
    pub totalSectors32: uint32_t,
    pub fatDescriptor: C2RustUnnamed,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed {
    pub fat16: fat16Descriptor_t,
    pub fat32: fat32Descriptor_t,
}
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct fatDirectoryEntry_t {
    pub filename: [libc::c_char; 11],
    pub attrib: uint8_t,
    pub ntReserved: uint8_t,
    pub creationTimeTenths: uint8_t,
    pub creationTime: uint16_t,
    pub creationDate: uint16_t,
    pub lastAccessDate: uint16_t,
    pub firstClusterHigh: uint16_t,
    pub lastWriteTime: uint16_t,
    pub lastWriteDate: uint16_t,
    pub firstClusterLow: uint16_t,
    pub fileSize: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct afatfsFile_t {
    pub type_0: afatfsFileType_e,
    pub cursorOffset: uint32_t,
    pub logicalSize: uint32_t,
    pub physicalSize: uint32_t,
    pub cursorCluster: uint32_t,
    pub cursorPreviousCluster: uint32_t,
    pub mode: uint8_t,
    pub attrib: uint8_t,
    pub writeLockedCacheIndex: int8_t,
    pub readRetainCacheIndex: int8_t,
    pub directoryEntryPos: afatfsDirEntryPointer_t,
    pub firstCluster: uint32_t,
    pub operation: afatfsFileOperation_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct afatfsFileOperation_t {
    pub operation: afatfsFileOperation_e,
    pub state: C2RustUnnamed_0,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_0 {
    pub createFile: afatfsCreateFile_t,
    pub seek: afatfsSeek_t,
    pub appendSupercluster: afatfsAppendSupercluster_t,
    pub appendFreeCluster: afatfsAppendFreeCluster_t,
    pub extendSubdirectory: afatfsExtendSubdirectory_t,
    pub unlinkFile: afatfsUnlinkFile_t,
    pub truncateFile: afatfsTruncateFile_t,
    pub closeFile: afatfsCloseFile_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct afatfsCloseFile_t {
    pub callback: afatfsCallback_t,
}
pub type afatfsCallback_t = Option<unsafe extern "C" fn() -> ()>;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct afatfsTruncateFile_t {
    pub startCluster: uint32_t,
    pub currentCluster: uint32_t,
    pub endCluster: uint32_t,
    pub callback: afatfsFileCallback_t,
    pub phase: afatfsTruncateFilePhase_e,
}
pub type afatfsTruncateFilePhase_e = libc::c_uint;
pub const AFATFS_TRUNCATE_FILE_SUCCESS: afatfsTruncateFilePhase_e = 4;
pub const AFATFS_TRUNCATE_FILE_PREPEND_TO_FREEFILE: afatfsTruncateFilePhase_e
          =
    3;
pub const AFATFS_TRUNCATE_FILE_ERASE_FAT_CHAIN_CONTIGUOUS:
          afatfsTruncateFilePhase_e =
    2;
pub const AFATFS_TRUNCATE_FILE_ERASE_FAT_CHAIN_NORMAL:
          afatfsTruncateFilePhase_e =
    1;
pub const AFATFS_TRUNCATE_FILE_UPDATE_DIRECTORY: afatfsTruncateFilePhase_e =
    0;
pub const AFATFS_TRUNCATE_FILE_INITIAL: afatfsTruncateFilePhase_e = 0;
pub type afatfsFileCallback_t
    =
    Option<unsafe extern "C" fn(_: afatfsFilePtr_t) -> ()>;
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
pub type afatfsFilePtr_t = *mut afatfsFile_t;
pub type afatfsUnlinkFile_t = afatfsDeleteFile_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct afatfsDeleteFile_t {
    pub truncateFile: afatfsTruncateFile_t,
    pub callback: afatfsCallback_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct afatfsExtendSubdirectory_t {
    pub appendFreeCluster: afatfsAppendFreeCluster_t,
    pub phase: afatfsExtendSubdirectoryPhase_e,
    pub parentDirectoryCluster: uint32_t,
    pub callback: afatfsFileCallback_t,
}
pub type afatfsExtendSubdirectoryPhase_e = libc::c_uint;
pub const AFATFS_EXTEND_SUBDIRECTORY_PHASE_FAILURE:
          afatfsExtendSubdirectoryPhase_e =
    3;
pub const AFATFS_EXTEND_SUBDIRECTORY_PHASE_SUCCESS:
          afatfsExtendSubdirectoryPhase_e =
    2;
pub const AFATFS_EXTEND_SUBDIRECTORY_PHASE_WRITE_SECTORS:
          afatfsExtendSubdirectoryPhase_e =
    1;
pub const AFATFS_EXTEND_SUBDIRECTORY_PHASE_ADD_FREE_CLUSTER:
          afatfsExtendSubdirectoryPhase_e =
    0;
pub const AFATFS_EXTEND_SUBDIRECTORY_PHASE_INITIAL:
          afatfsExtendSubdirectoryPhase_e =
    0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct afatfsAppendFreeCluster_t {
    pub previousCluster: uint32_t,
    pub searchCluster: uint32_t,
    pub phase: afatfsAppendFreeClusterPhase_e,
}
pub type afatfsAppendFreeClusterPhase_e = libc::c_uint;
pub const AFATFS_APPEND_FREE_CLUSTER_PHASE_FAILURE:
          afatfsAppendFreeClusterPhase_e =
    5;
pub const AFATFS_APPEND_FREE_CLUSTER_PHASE_COMPLETE:
          afatfsAppendFreeClusterPhase_e =
    4;
pub const AFATFS_APPEND_FREE_CLUSTER_PHASE_UPDATE_FILE_DIRECTORY:
          afatfsAppendFreeClusterPhase_e =
    3;
pub const AFATFS_APPEND_FREE_CLUSTER_PHASE_UPDATE_FAT2:
          afatfsAppendFreeClusterPhase_e =
    2;
pub const AFATFS_APPEND_FREE_CLUSTER_PHASE_UPDATE_FAT1:
          afatfsAppendFreeClusterPhase_e =
    1;
pub const AFATFS_APPEND_FREE_CLUSTER_PHASE_FIND_FREESPACE:
          afatfsAppendFreeClusterPhase_e =
    0;
pub const AFATFS_APPEND_FREE_CLUSTER_PHASE_INITIAL:
          afatfsAppendFreeClusterPhase_e =
    0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct afatfsAppendSupercluster_t {
    pub previousCluster: uint32_t,
    pub fatRewriteStartCluster: uint32_t,
    pub fatRewriteEndCluster: uint32_t,
    pub phase: afatfsAppendSuperclusterPhase_e,
}
pub type afatfsAppendSuperclusterPhase_e = libc::c_uint;
pub const AFATFS_APPEND_SUPERCLUSTER_PHASE_UPDATE_FILE_DIRECTORY:
          afatfsAppendSuperclusterPhase_e =
    3;
pub const AFATFS_APPEND_SUPERCLUSTER_PHASE_UPDATE_FAT:
          afatfsAppendSuperclusterPhase_e =
    2;
pub const AFATFS_APPEND_SUPERCLUSTER_PHASE_UPDATE_FREEFILE_DIRECTORY:
          afatfsAppendSuperclusterPhase_e =
    1;
pub const AFATFS_APPEND_SUPERCLUSTER_PHASE_INIT:
          afatfsAppendSuperclusterPhase_e =
    0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct afatfsSeek_t {
    pub callback: afatfsFileCallback_t,
    pub seekOffset: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct afatfsCreateFile_t {
    pub callback: afatfsFileCallback_t,
    pub phase: uint8_t,
    pub filename: [uint8_t; 11],
}
pub type afatfsFileOperation_e = libc::c_uint;
pub const AFATFS_FILE_OPERATION_EXTEND_SUBDIRECTORY: afatfsFileOperation_e =
    9;
pub const AFATFS_FILE_OPERATION_APPEND_FREE_CLUSTER: afatfsFileOperation_e =
    8;
pub const AFATFS_FILE_OPERATION_LOCKED: afatfsFileOperation_e = 7;
pub const AFATFS_FILE_OPERATION_APPEND_SUPERCLUSTER: afatfsFileOperation_e =
    6;
pub const AFATFS_FILE_OPERATION_UNLINK: afatfsFileOperation_e = 5;
pub const AFATFS_FILE_OPERATION_TRUNCATE: afatfsFileOperation_e = 4;
pub const AFATFS_FILE_OPERATION_CLOSE: afatfsFileOperation_e = 3;
pub const AFATFS_FILE_OPERATION_SEEK: afatfsFileOperation_e = 2;
pub const AFATFS_FILE_OPERATION_CREATE_FILE: afatfsFileOperation_e = 1;
pub const AFATFS_FILE_OPERATION_NONE: afatfsFileOperation_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct afatfsDirEntryPointer_t {
    pub sectorNumberPhysical: uint32_t,
    pub entryIndex: int16_t,
}
pub type afatfsFileType_e = libc::c_uint;
pub const AFATFS_FILE_TYPE_DIRECTORY: afatfsFileType_e = 3;
pub const AFATFS_FILE_TYPE_FAT16_ROOT_DIRECTORY: afatfsFileType_e = 2;
pub const AFATFS_FILE_TYPE_NORMAL: afatfsFileType_e = 1;
pub const AFATFS_FILE_TYPE_NONE: afatfsFileType_e = 0;
pub type afatfsFilesystemState_e = libc::c_uint;
pub const AFATFS_FILESYSTEM_STATE_READY: afatfsFilesystemState_e = 3;
pub const AFATFS_FILESYSTEM_STATE_INITIALIZATION: afatfsFilesystemState_e = 2;
pub const AFATFS_FILESYSTEM_STATE_FATAL: afatfsFilesystemState_e = 1;
pub const AFATFS_FILESYSTEM_STATE_UNKNOWN: afatfsFilesystemState_e = 0;
pub type afatfsOperationStatus_e = libc::c_uint;
pub const AFATFS_OPERATION_FAILURE: afatfsOperationStatus_e = 2;
pub const AFATFS_OPERATION_SUCCESS: afatfsOperationStatus_e = 1;
pub const AFATFS_OPERATION_IN_PROGRESS: afatfsOperationStatus_e = 0;
pub type afatfsError_e = libc::c_uint;
pub const AFATFS_ERROR_BAD_FILESYSTEM_HEADER: afatfsError_e = 3;
pub const AFATFS_ERROR_BAD_MBR: afatfsError_e = 2;
pub const AFATFS_ERROR_GENERIC: afatfsError_e = 1;
pub const AFATFS_ERROR_NONE: afatfsError_e = 0;
pub type afatfsFinder_t = afatfsDirEntryPointer_t;
pub type afatfsSeek_e = libc::c_uint;
pub const AFATFS_SEEK_END: afatfsSeek_e = 2;
pub const AFATFS_SEEK_CUR: afatfsSeek_e = 1;
pub const AFATFS_SEEK_SET: afatfsSeek_e = 0;
#[derive(Copy, Clone, BitfieldStruct)]
#[repr(C)]
pub struct afatfsCacheBlockDescriptor_t {
    pub sectorIndex: uint32_t,
    pub writeTimestamp: uint32_t,
    pub accessTimestamp: uint32_t,
    pub consecutiveEraseBlockCount: uint16_t,
    pub state: afatfsCacheBlockState_e,
    #[bitfield(name = "locked", ty = "libc::c_uint", bits = "0..=0")]
    #[bitfield(name = "retainCount", ty = "libc::c_uint", bits = "1..=6")]
    #[bitfield(name = "discardable", ty = "libc::c_uint", bits = "7..=7")]
    pub locked_retainCount_discardable: [u8; 1],
    #[bitfield(padding)]
    pub c2rust_padding: [u8; 3],
}
pub type afatfsCacheBlockState_e = libc::c_uint;
pub const AFATFS_CACHE_STATE_DIRTY: afatfsCacheBlockState_e = 4;
pub const AFATFS_CACHE_STATE_WRITING: afatfsCacheBlockState_e = 3;
pub const AFATFS_CACHE_STATE_READING: afatfsCacheBlockState_e = 2;
pub const AFATFS_CACHE_STATE_IN_SYNC: afatfsCacheBlockState_e = 1;
pub const AFATFS_CACHE_STATE_EMPTY: afatfsCacheBlockState_e = 0;
pub const AFATFS_CREATEFILE_PHASE_FAILURE: C2RustUnnamed_2 = 4;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct afatfs_t {
    pub filesystemType: fatFilesystemType_e,
    pub filesystemState: afatfsFilesystemState_e,
    pub initPhase: afatfsInitializationPhase_e,
    pub initState: C2RustUnnamed_1,
    pub cache: [uint8_t; 4096],
    pub cacheDescriptor: [afatfsCacheBlockDescriptor_t; 8],
    pub cacheTimer: uint32_t,
    pub cacheDirtyEntries: libc::c_int,
    pub cacheFlushInProgress: bool,
    pub openFiles: [afatfsFile_t; 3],
    pub freeFile: afatfsFile_t,
    pub lastError: afatfsError_e,
    pub filesystemFull: bool,
    pub currentDirectory: afatfsFile_t,
    pub partitionStartSector: uint32_t,
    pub fatStartSector: uint32_t,
    pub fatSectors: uint32_t,
    pub numClusters: uint32_t,
    pub clusterStartSector: uint32_t,
    pub sectorsPerCluster: uint32_t,
    pub lastClusterAllocated: uint32_t,
    pub byteInClusterMask: uint32_t,
    pub rootDirectoryCluster: uint32_t,
    pub rootDirectorySectors: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_1 {
    pub freeSpaceSearch: afatfsFreeSpaceSearch_t,
    pub freeSpaceFAT: afatfsFreeSpaceFAT_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct afatfsFreeSpaceFAT_t {
    pub startCluster: uint32_t,
    pub endCluster: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct afatfsFreeSpaceSearch_t {
    pub candidateStart: uint32_t,
    pub candidateEnd: uint32_t,
    pub bestGapStart: uint32_t,
    pub bestGapLength: uint32_t,
    pub phase: afatfsFreeSpaceSearchPhase_e,
}
pub type afatfsFreeSpaceSearchPhase_e = libc::c_uint;
pub const AFATFS_FREE_SPACE_SEARCH_PHASE_GROW_HOLE:
          afatfsFreeSpaceSearchPhase_e =
    1;
pub const AFATFS_FREE_SPACE_SEARCH_PHASE_FIND_HOLE:
          afatfsFreeSpaceSearchPhase_e =
    0;
pub type afatfsInitializationPhase_e = libc::c_uint;
pub const AFATFS_INITIALIZATION_DONE: afatfsInitializationPhase_e = 7;
pub const AFATFS_INITIALIZATION_FREEFILE_LAST: afatfsInitializationPhase_e =
    6;
pub const AFATFS_INITIALIZATION_FREEFILE_SAVE_DIR_ENTRY:
          afatfsInitializationPhase_e =
    6;
pub const AFATFS_INITIALIZATION_FREEFILE_UPDATE_FAT:
          afatfsInitializationPhase_e =
    5;
pub const AFATFS_INITIALIZATION_FREEFILE_FAT_SEARCH:
          afatfsInitializationPhase_e =
    4;
pub const AFATFS_INITIALIZATION_FREEFILE_CREATING: afatfsInitializationPhase_e
          =
    3;
pub const AFATFS_INITIALIZATION_FREEFILE_CREATE: afatfsInitializationPhase_e =
    2;
pub const AFATFS_INITIALIZATION_READ_VOLUME_ID: afatfsInitializationPhase_e =
    1;
pub const AFATFS_INITIALIZATION_READ_MBR: afatfsInitializationPhase_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub union afatfsFATSector_t {
    pub bytes: *mut uint8_t,
    pub fat16: *mut uint16_t,
    pub fat32: *mut uint32_t,
}
pub type sdcardBlockOperation_e = libc::c_uint;
pub const SDCARD_BLOCK_OPERATION_ERASE: sdcardBlockOperation_e = 2;
pub const SDCARD_BLOCK_OPERATION_WRITE: sdcardBlockOperation_e = 1;
pub const SDCARD_BLOCK_OPERATION_READ: sdcardBlockOperation_e = 0;
pub type sdcard_operationCompleteCallback_c
    =
    Option<unsafe extern "C" fn(_: sdcardBlockOperation_e, _: uint32_t,
                                _: *mut uint8_t, _: uint32_t) -> ()>;
pub const AFATFS_CREATEFILE_PHASE_SUCCESS: C2RustUnnamed_2 = 3;
pub type dateTime_t = _dateTime_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _dateTime_s {
    pub year: uint16_t,
    pub month: uint8_t,
    pub day: uint8_t,
    pub hours: uint8_t,
    pub minutes: uint8_t,
    pub seconds: uint8_t,
    pub millis: uint16_t,
}
pub type afatfsSaveDirectoryEntryMode_e = libc::c_uint;
pub const AFATFS_SAVE_DIRECTORY_DELETED: afatfsSaveDirectoryEntryMode_e = 2;
pub const AFATFS_SAVE_DIRECTORY_FOR_CLOSE: afatfsSaveDirectoryEntryMode_e = 1;
pub const AFATFS_SAVE_DIRECTORY_NORMAL: afatfsSaveDirectoryEntryMode_e = 0;
pub const AFATFS_FIND_CLUSTER_IN_PROGRESS: afatfsFindClusterStatus_e = 0;
pub const AFATFS_FIND_CLUSTER_NOT_FOUND: afatfsFindClusterStatus_e = 3;
pub const AFATFS_FIND_CLUSTER_FATAL: afatfsFindClusterStatus_e = 2;
pub const AFATFS_FIND_CLUSTER_FOUND: afatfsFindClusterStatus_e = 1;
pub type afatfsFindClusterStatus_e = libc::c_uint;
pub type afatfsClusterSearchCondition_e = libc::c_uint;
pub const CLUSTER_SEARCH_OCCUPIED: afatfsClusterSearchCondition_e = 2;
pub const CLUSTER_SEARCH_FREE: afatfsClusterSearchCondition_e = 1;
pub const CLUSTER_SEARCH_FREE_AT_BEGINNING_OF_FAT_SECTOR:
          afatfsClusterSearchCondition_e =
    0;
pub const AFATFS_CREATEFILE_PHASE_CREATE_NEW_FILE: C2RustUnnamed_2 = 2;
pub const AFATFS_CREATEFILE_PHASE_FIND_FILE: C2RustUnnamed_2 = 1;
pub const AFATFS_CREATEFILE_PHASE_INITIAL: C2RustUnnamed_2 = 0;
pub type afatfsFATPattern_e = libc::c_uint;
pub const AFATFS_FAT_PATTERN_FREE: afatfsFATPattern_e = 2;
pub const AFATFS_FAT_PATTERN_TERMINATED_CHAIN: afatfsFATPattern_e = 1;
pub const AFATFS_FAT_PATTERN_UNTERMINATED_CHAIN: afatfsFATPattern_e = 0;
pub const SDCARD_OPERATION_FAILURE: sdcardOperationStatus_e = 3;
pub const SDCARD_OPERATION_BUSY: sdcardOperationStatus_e = 1;
pub const SDCARD_OPERATION_SUCCESS: sdcardOperationStatus_e = 2;
pub const SDCARD_OPERATION_IN_PROGRESS: sdcardOperationStatus_e = 0;
pub type sdcardOperationStatus_e = libc::c_uint;
pub type C2RustUnnamed_2 = libc::c_uint;
static mut afatfs: afatfs_t =
    afatfs_t{filesystemType: FAT_FILESYSTEM_TYPE_INVALID,
             filesystemState: AFATFS_FILESYSTEM_STATE_UNKNOWN,
             initPhase: AFATFS_INITIALIZATION_READ_MBR,
             initState:
                 C2RustUnnamed_1{freeSpaceSearch:
                                     afatfsFreeSpaceSearch_t{candidateStart:
                                                                 0,
                                                             candidateEnd: 0,
                                                             bestGapStart: 0,
                                                             bestGapLength: 0,
                                                             phase:
                                                                 AFATFS_FREE_SPACE_SEARCH_PHASE_FIND_HOLE,},},
             cache: [0; 4096],
             cacheDescriptor:
                 [afatfsCacheBlockDescriptor_t{sectorIndex: 0,
                                               writeTimestamp: 0,
                                               accessTimestamp: 0,
                                               consecutiveEraseBlockCount: 0,
                                               state:
                                                   AFATFS_CACHE_STATE_EMPTY,
                                               locked_retainCount_discardable:
                                                   [0; 1],
                                               c2rust_padding: [0; 3],}; 8],
             cacheTimer: 0,
             cacheDirtyEntries: 0,
             cacheFlushInProgress: false,
             openFiles:
                 [afatfsFile_t{type_0: AFATFS_FILE_TYPE_NONE,
                               cursorOffset: 0,
                               logicalSize: 0,
                               physicalSize: 0,
                               cursorCluster: 0,
                               cursorPreviousCluster: 0,
                               mode: 0,
                               attrib: 0,
                               writeLockedCacheIndex: 0,
                               readRetainCacheIndex: 0,
                               directoryEntryPos:
                                   afatfsDirEntryPointer_t{sectorNumberPhysical:
                                                               0,
                                                           entryIndex: 0,},
                               firstCluster: 0,
                               operation:
                                   afatfsFileOperation_t{operation:
                                                             AFATFS_FILE_OPERATION_NONE,
                                                         state:
                                                             C2RustUnnamed_0{createFile:
                                                                                 afatfsCreateFile_t{callback:
                                                                                                        None,
                                                                                                    phase:
                                                                                                        0,
                                                                                                    filename:
                                                                                                        [0;
                                                                                                            11],},},},};
                     3],
             freeFile:
                 afatfsFile_t{type_0: AFATFS_FILE_TYPE_NONE,
                              cursorOffset: 0,
                              logicalSize: 0,
                              physicalSize: 0,
                              cursorCluster: 0,
                              cursorPreviousCluster: 0,
                              mode: 0,
                              attrib: 0,
                              writeLockedCacheIndex: 0,
                              readRetainCacheIndex: 0,
                              directoryEntryPos:
                                  afatfsDirEntryPointer_t{sectorNumberPhysical:
                                                              0,
                                                          entryIndex: 0,},
                              firstCluster: 0,
                              operation:
                                  afatfsFileOperation_t{operation:
                                                            AFATFS_FILE_OPERATION_NONE,
                                                        state:
                                                            C2RustUnnamed_0{createFile:
                                                                                afatfsCreateFile_t{callback:
                                                                                                       None,
                                                                                                   phase:
                                                                                                       0,
                                                                                                   filename:
                                                                                                       [0;
                                                                                                           11],},},},},
             lastError: AFATFS_ERROR_NONE,
             filesystemFull: false,
             currentDirectory:
                 afatfsFile_t{type_0: AFATFS_FILE_TYPE_NONE,
                              cursorOffset: 0,
                              logicalSize: 0,
                              physicalSize: 0,
                              cursorCluster: 0,
                              cursorPreviousCluster: 0,
                              mode: 0,
                              attrib: 0,
                              writeLockedCacheIndex: 0,
                              readRetainCacheIndex: 0,
                              directoryEntryPos:
                                  afatfsDirEntryPointer_t{sectorNumberPhysical:
                                                              0,
                                                          entryIndex: 0,},
                              firstCluster: 0,
                              operation:
                                  afatfsFileOperation_t{operation:
                                                            AFATFS_FILE_OPERATION_NONE,
                                                        state:
                                                            C2RustUnnamed_0{createFile:
                                                                                afatfsCreateFile_t{callback:
                                                                                                       None,
                                                                                                   phase:
                                                                                                       0,
                                                                                                   filename:
                                                                                                       [0;
                                                                                                           11],},},},},
             partitionStartSector: 0,
             fatStartSector: 0,
             fatSectors: 0,
             numClusters: 0,
             clusterStartSector: 0,
             sectorsPerCluster: 0,
             lastClusterAllocated: 0,
             byteInClusterMask: 0,
             rootDirectoryCluster: 0,
             rootDirectorySectors: 0,};
unsafe extern "C" fn roundUpTo(mut value: uint32_t, mut rounding: uint32_t)
 -> uint32_t {
    let mut remainder: uint32_t = value.wrapping_rem(rounding);
    if remainder > 0 as libc::c_int as libc::c_uint {
        value =
            (value as
                 libc::c_uint).wrapping_add(rounding.wrapping_sub(remainder))
                as uint32_t as uint32_t
    }
    return value;
}
unsafe extern "C" fn isPowerOfTwo(mut x: libc::c_uint) -> bool {
    return x != 0 as libc::c_int as libc::c_uint &&
               x & (!x).wrapping_add(1 as libc::c_int as libc::c_uint) == x;
}
// full year
// 1-12
// 1-31
// 0-23
// 0-59
// 0-59
// 0-999
/* *
 * Check for conditions that should always be true (and if otherwise mean a bug or a corrupt filesystem).
 *
 * If the condition is false, the filesystem is marked as being in a fatal state.
 *
 * Returns the value of the condition.
 */
unsafe extern "C" fn afatfs_assert(mut condition: bool) -> bool {
    if !condition {
        if afatfs.lastError as libc::c_uint ==
               AFATFS_ERROR_NONE as libc::c_int as libc::c_uint {
            afatfs.lastError = AFATFS_ERROR_GENERIC
        }
        afatfs.filesystemState = AFATFS_FILESYSTEM_STATE_FATAL
    }
    return condition;
}
unsafe extern "C" fn afatfs_fileIsBusy(mut file: afatfsFilePtr_t) -> bool {
    return (*file).operation.operation as libc::c_uint !=
               AFATFS_FILE_OPERATION_NONE as libc::c_int as libc::c_uint;
}
/* *
 * The number of FAT table entries that fit within one AFATFS sector size.
 *
 * Note that this is the same as the number of clusters in an AFATFS supercluster.
 */
unsafe extern "C" fn afatfs_fatEntriesPerSector() -> uint32_t {
    return if afatfs.filesystemType as libc::c_uint ==
                  FAT_FILESYSTEM_TYPE_FAT32 as libc::c_int as libc::c_uint {
               (512 as libc::c_int as
                    libc::c_ulong).wrapping_div(::core::mem::size_of::<uint32_t>()
                                                    as libc::c_ulong)
           } else {
               (512 as libc::c_int as
                    libc::c_ulong).wrapping_div(::core::mem::size_of::<uint16_t>()
                                                    as libc::c_ulong)
           } as uint32_t;
}
/* *
 * Size of a FAT cluster in bytes
 */
unsafe extern "C" fn afatfs_clusterSize() -> uint32_t {
    return afatfs.sectorsPerCluster.wrapping_mul(512 as libc::c_int as
                                                     libc::c_uint);
}
/* *
 * Given a byte offset within a file, return the byte offset of that position within the cluster it belongs to.
 */
unsafe extern "C" fn afatfs_byteIndexInCluster(mut byteOffset: uint32_t)
 -> uint32_t {
    return afatfs.byteInClusterMask & byteOffset;
}
/* *
 * Given a byte offset within a file, return the index of the sector within the cluster it belongs to.
 */
unsafe extern "C" fn afatfs_sectorIndexInCluster(mut byteOffset: uint32_t)
 -> uint32_t {
    return afatfs_byteIndexInCluster(byteOffset).wrapping_div(512 as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_uint);
}
// Get the buffer memory for the cache entry of the given index.
unsafe extern "C" fn afatfs_cacheSectorGetMemory(mut cacheEntryIndex:
                                                     libc::c_int)
 -> *mut uint8_t {
    return afatfs.cache.as_mut_ptr().offset((cacheEntryIndex *
                                                 512 as libc::c_int) as
                                                isize);
}
unsafe extern "C" fn afatfs_getCacheDescriptorIndexForBuffer(mut memory:
                                                                 *mut uint8_t)
 -> libc::c_int {
    let mut index: libc::c_int =
        (memory.wrapping_offset_from(afatfs.cache.as_mut_ptr()) as
             libc::c_long / 512 as libc::c_int as libc::c_long) as
            libc::c_int;
    if afatfs_assert(index >= 0 as libc::c_int && index < 8 as libc::c_int) {
        return index
    } else { return -(1 as libc::c_int) };
}
unsafe extern "C" fn afatfs_getCacheDescriptorForBuffer(mut memory:
                                                            *mut uint8_t)
 -> *mut afatfsCacheBlockDescriptor_t {
    return afatfs.cacheDescriptor.as_mut_ptr().offset(afatfs_getCacheDescriptorIndexForBuffer(memory)
                                                          as isize);
}
unsafe extern "C" fn afatfs_cacheSectorMarkDirty(mut descriptor:
                                                     *mut afatfsCacheBlockDescriptor_t) {
    if (*descriptor).state as libc::c_uint !=
           AFATFS_CACHE_STATE_DIRTY as libc::c_int as libc::c_uint {
        afatfs.cacheTimer = afatfs.cacheTimer.wrapping_add(1);
        (*descriptor).writeTimestamp = afatfs.cacheTimer;
        (*descriptor).state = AFATFS_CACHE_STATE_DIRTY;
        afatfs.cacheDirtyEntries += 1
    };
}
unsafe extern "C" fn afatfs_cacheSectorInit(mut descriptor:
                                                *mut afatfsCacheBlockDescriptor_t,
                                            mut sectorIndex: uint32_t,
                                            mut locked: bool) {
    (*descriptor).sectorIndex = sectorIndex;
    afatfs.cacheTimer = afatfs.cacheTimer.wrapping_add(1);
    (*descriptor).writeTimestamp = afatfs.cacheTimer;
    (*descriptor).accessTimestamp = (*descriptor).writeTimestamp;
    (*descriptor).consecutiveEraseBlockCount = 0 as libc::c_int as uint16_t;
    (*descriptor).state = AFATFS_CACHE_STATE_EMPTY;
    (*descriptor).set_locked(locked as libc::c_uint);
    (*descriptor).set_retainCount(0 as libc::c_int as libc::c_uint);
    (*descriptor).set_discardable(0 as libc::c_int as libc::c_uint);
}
/* *
 * Called by the SD card driver when one of our read operations completes.
 */
unsafe extern "C" fn afatfs_sdcardReadComplete(mut operation:
                                                   sdcardBlockOperation_e,
                                               mut sectorIndex: uint32_t,
                                               mut buffer: *mut uint8_t,
                                               mut callbackData: uint32_t) {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 8 as libc::c_int {
        if afatfs.cacheDescriptor[i as usize].state as libc::c_uint !=
               AFATFS_CACHE_STATE_EMPTY as libc::c_int as libc::c_uint &&
               afatfs.cacheDescriptor[i as usize].sectorIndex == sectorIndex {
            if buffer.is_null() {
                // Read failed, mark the sector as empty and whoever asked for it will ask for it again later to retry
                afatfs.cacheDescriptor[i as usize].state =
                    AFATFS_CACHE_STATE_EMPTY
            } else {
                afatfs_assert(afatfs_cacheSectorGetMemory(i) == buffer &&
                                  afatfs.cacheDescriptor[i as usize].state as
                                      libc::c_uint ==
                                      AFATFS_CACHE_STATE_READING as
                                          libc::c_int as libc::c_uint);
                afatfs.cacheDescriptor[i as usize].state =
                    AFATFS_CACHE_STATE_IN_SYNC
            }
            break ;
        } else { i += 1 }
    };
}
/* *
 * Called by the SD card driver when one of our write operations completes.
 */
unsafe extern "C" fn afatfs_sdcardWriteComplete(mut operation:
                                                    sdcardBlockOperation_e,
                                                mut sectorIndex: uint32_t,
                                                mut buffer: *mut uint8_t,
                                                mut callbackData: uint32_t) {
    afatfs.cacheFlushInProgress = 0 as libc::c_int != 0;
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 8 as libc::c_int {
        /* Keep in mind that someone may have marked the sector as dirty after writing had already begun. In this case we must leave
         * it marked as dirty because those modifications may have been made too late to make it to the disk!
         */
        if afatfs.cacheDescriptor[i as usize].sectorIndex == sectorIndex &&
               afatfs.cacheDescriptor[i as usize].state as libc::c_uint ==
                   AFATFS_CACHE_STATE_WRITING as libc::c_int as libc::c_uint {
            if buffer.is_null() {
                // Write failed, remark the sector as dirty
                afatfs.cacheDescriptor[i as usize].state =
                    AFATFS_CACHE_STATE_DIRTY;
                afatfs.cacheDirtyEntries += 1
            } else {
                afatfs_assert(afatfs_cacheSectorGetMemory(i) == buffer);
                afatfs.cacheDescriptor[i as usize].state =
                    AFATFS_CACHE_STATE_IN_SYNC
            }
            break ;
        } else { i += 1 }
    };
}
/* *
 * Attempt to flush the dirty cache entry with the given index to the SDcard.
 */
unsafe extern "C" fn afatfs_cacheFlushSector(mut cacheIndex: libc::c_int) {
    let mut cacheDescriptor: *mut afatfsCacheBlockDescriptor_t =
        &mut *afatfs.cacheDescriptor.as_mut_ptr().offset(cacheIndex as isize)
            as *mut afatfsCacheBlockDescriptor_t;
    if (*cacheDescriptor).consecutiveEraseBlockCount != 0 {
        sdcard_beginWriteBlocks((*cacheDescriptor).sectorIndex,
                                (*cacheDescriptor).consecutiveEraseBlockCount
                                    as uint32_t);
    }
    match sdcard_writeBlock((*cacheDescriptor).sectorIndex,
                            afatfs_cacheSectorGetMemory(cacheIndex),
                            Some(afatfs_sdcardWriteComplete as
                                     unsafe extern "C" fn(_:
                                                              sdcardBlockOperation_e,
                                                          _: uint32_t,
                                                          _: *mut uint8_t,
                                                          _: uint32_t) -> ()),
                            0 as libc::c_int as uint32_t) as libc::c_uint {
        0 => {
            // The card will call us back later when the buffer transmission finishes
            afatfs.cacheDirtyEntries -= 1;
            (*cacheDescriptor).state = AFATFS_CACHE_STATE_WRITING;
            afatfs.cacheFlushInProgress = 1 as libc::c_int != 0
        }
        2 => {
            // Buffer is already transmitted
            afatfs.cacheDirtyEntries -= 1;
            (*cacheDescriptor).state = AFATFS_CACHE_STATE_IN_SYNC
        }
        1 | 3 | _ => { }
    };
}
/* *
 * Find a sector in the cache which corresponds to the given physical sector index, or NULL if the sector isn't
 * cached. Note that the cached sector could be in any state including completely empty.
 */
unsafe extern "C" fn afatfs_findCacheSector(mut sectorIndex: uint32_t)
 -> *mut afatfsCacheBlockDescriptor_t {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 8 as libc::c_int {
        if afatfs.cacheDescriptor[i as usize].sectorIndex == sectorIndex {
            return &mut *afatfs.cacheDescriptor.as_mut_ptr().offset(i as
                                                                        isize)
                       as *mut afatfsCacheBlockDescriptor_t
        }
        i += 1
    }
    return 0 as *mut afatfsCacheBlockDescriptor_t;
}
/* *
 * Find or allocate a cache sector for the given sector index on disk. Returns a block which matches one of these
 * conditions (in descending order of preference):
 *
 * - The requested sector that already exists in the cache
 * - The index of an empty sector
 * - The index of a synced discardable sector
 * - The index of the oldest synced sector
 *
 * Otherwise it returns -1 to signal failure (cache is full!)
 */
unsafe extern "C" fn afatfs_allocateCacheSector(mut sectorIndex: uint32_t)
 -> libc::c_int {
    let mut allocateIndex: libc::c_int = 0;
    let mut emptyIndex: libc::c_int = -(1 as libc::c_int);
    let mut discardableIndex: libc::c_int = -(1 as libc::c_int);
    let mut oldestSyncedSectorLastUse: uint32_t = 0xffffffff as libc::c_uint;
    let mut oldestSyncedSectorIndex: libc::c_int = -(1 as libc::c_int);
    if !afatfs_assert(afatfs.numClusters == 0 as libc::c_int as libc::c_uint
                          ||
                          sectorIndex <
                              afatfs.clusterStartSector.wrapping_add(afatfs.numClusters.wrapping_mul(afatfs.sectorsPerCluster)))
       {
        return -(1 as libc::c_int)
    }
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 8 as libc::c_int {
        if afatfs.cacheDescriptor[i as usize].sectorIndex == sectorIndex {
            /*
             * If the sector is actually empty then do a complete re-init of it just like the standard
             * empty case. (Sectors marked as empty should be treated as if they don't have a block index assigned)
             */
            if afatfs.cacheDescriptor[i as usize].state as libc::c_uint ==
                   AFATFS_CACHE_STATE_EMPTY as libc::c_int as libc::c_uint {
                emptyIndex = i;
                break ;
            } else {
                // Bump the last access time
                afatfs.cacheTimer = afatfs.cacheTimer.wrapping_add(1);
                afatfs.cacheDescriptor[i as usize].accessTimestamp =
                    afatfs.cacheTimer;
                return i
            }
        } else {
            match afatfs.cacheDescriptor[i as usize].state as libc::c_uint {
                0 => { emptyIndex = i }
                1 => {
                    // Is this a synced sector that we could evict from the cache?
                    if afatfs.cacheDescriptor[i as usize].locked() == 0 &&
                           afatfs.cacheDescriptor[i as usize].retainCount() as
                               libc::c_int == 0 as libc::c_int {
                        if afatfs.cacheDescriptor[i as usize].discardable() !=
                               0 {
                            discardableIndex = i
                        } else if afatfs.cacheDescriptor[i as
                                                             usize].accessTimestamp
                                      < oldestSyncedSectorLastUse {
                            // This is older than last block we decided to evict, so evict this one in preference
                            oldestSyncedSectorLastUse =
                                afatfs.cacheDescriptor[i as
                                                           usize].accessTimestamp;
                            oldestSyncedSectorIndex = i
                        }
                    }
                }
                _ => { }
            }
            i += 1
        }
    }
    if emptyIndex > -(1 as libc::c_int) {
        allocateIndex = emptyIndex
    } else if discardableIndex > -(1 as libc::c_int) {
        allocateIndex = discardableIndex
    } else if oldestSyncedSectorIndex > -(1 as libc::c_int) {
        allocateIndex = oldestSyncedSectorIndex
    } else { allocateIndex = -(1 as libc::c_int) }
    if allocateIndex > -(1 as libc::c_int) {
        afatfs_cacheSectorInit(&mut *afatfs.cacheDescriptor.as_mut_ptr().offset(allocateIndex
                                                                                    as
                                                                                    isize),
                               sectorIndex, 0 as libc::c_int != 0);
    }
    return allocateIndex;
}
/* *
 * Attempt to flush dirty cache pages out to the sdcard, returning true if all flushable data has been flushed.
 */
#[no_mangle]
pub unsafe extern "C" fn afatfs_flush() -> bool {
    if afatfs.cacheDirtyEntries > 0 as libc::c_int {
        // Flush the oldest flushable sector
        let mut earliestSectorTime: uint32_t = 0xffffffff as libc::c_uint;
        let mut earliestSectorIndex: libc::c_int = -(1 as libc::c_int);
        let mut i: libc::c_int = 0 as libc::c_int;
        while i < 8 as libc::c_int {
            if afatfs.cacheDescriptor[i as usize].state as libc::c_uint ==
                   AFATFS_CACHE_STATE_DIRTY as libc::c_int as libc::c_uint &&
                   afatfs.cacheDescriptor[i as usize].locked() == 0 &&
                   (earliestSectorIndex == -(1 as libc::c_int) ||
                        afatfs.cacheDescriptor[i as usize].writeTimestamp <
                            earliestSectorTime) {
                earliestSectorIndex = i;
                earliestSectorTime =
                    afatfs.cacheDescriptor[i as usize].writeTimestamp
            }
            i += 1
        }
        if earliestSectorIndex > -(1 as libc::c_int) {
            afatfs_cacheFlushSector(earliestSectorIndex);
            // That flush will take time to complete so we may as well tell caller to come back later
            return 0 as libc::c_int != 0
        }
    }
    return 1 as libc::c_int != 0;
}
/* *
 * Returns true if either the freefile or the regular cluster pool has been exhausted during a previous write operation.
 */
#[no_mangle]
pub unsafe extern "C" fn afatfs_isFull() -> bool {
    return afatfs.filesystemFull;
}
/* *
 * Get the physical sector number that corresponds to the FAT sector of the given fatSectorIndex within the given
 * FAT (fatIndex may be 0 or 1). (0, 0) gives the first sector of the first FAT.
 */
unsafe extern "C" fn afatfs_fatSectorToPhysical(mut fatIndex: libc::c_int,
                                                mut fatSectorIndex: uint32_t)
 -> uint32_t {
    return afatfs.fatStartSector.wrapping_add((if fatIndex != 0 {
                                                   afatfs.fatSectors
                                               } else {
                                                   0 as libc::c_int as
                                                       libc::c_uint
                                               })).wrapping_add(fatSectorIndex);
}
unsafe extern "C" fn afatfs_fileClusterToPhysical(mut clusterNumber: uint32_t,
                                                  mut sectorIndex: uint32_t)
 -> uint32_t {
    return afatfs.clusterStartSector.wrapping_add(clusterNumber.wrapping_sub(2
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint).wrapping_mul(afatfs.sectorsPerCluster)).wrapping_add(sectorIndex);
}
unsafe extern "C" fn afatfs_fileGetCursorPhysicalSector(mut file:
                                                            afatfsFilePtr_t)
 -> uint32_t {
    if (*file).type_0 as libc::c_uint ==
           AFATFS_FILE_TYPE_FAT16_ROOT_DIRECTORY as libc::c_int as
               libc::c_uint {
        return afatfs.fatStartSector.wrapping_add((2 as libc::c_int as
                                                       libc::c_uint).wrapping_mul(afatfs.fatSectors)).wrapping_add((*file).cursorOffset.wrapping_div(512
                                                                                                                                                         as
                                                                                                                                                         libc::c_int
                                                                                                                                                         as
                                                                                                                                                         libc::c_uint))
    } else {
        let mut cursorSectorInCluster: uint32_t =
            afatfs_sectorIndexInCluster((*file).cursorOffset);
        return afatfs_fileClusterToPhysical((*file).cursorCluster,
                                            cursorSectorInCluster)
    };
}
/* *
 * Sector here is the sector index within the cluster.
 */
unsafe extern "C" fn afatfs_fileGetCursorClusterAndSector(mut file:
                                                              afatfsFilePtr_t,
                                                          mut cluster:
                                                              *mut uint32_t,
                                                          mut sector:
                                                              *mut uint16_t) {
    *cluster = (*file).cursorCluster;
    if (*file).type_0 as libc::c_uint ==
           AFATFS_FILE_TYPE_FAT16_ROOT_DIRECTORY as libc::c_int as
               libc::c_uint {
        *sector =
            (*file).cursorOffset.wrapping_div(512 as libc::c_int as
                                                  libc::c_uint) as uint16_t
    } else {
        *sector =
            afatfs_sectorIndexInCluster((*file).cursorOffset) as uint16_t
    };
}
/* *
 * Get a cache entry for the given sector and store a pointer to the cached memory in *buffer.
 *
 * physicalSectorIndex - The index of the sector in the SD card to cache
 * sectorflags         - A union of AFATFS_CACHE_* constants that says which operations the sector will be cached for.
 * buffer              - A pointer to the 512-byte memory buffer for the sector will be stored here upon success
 * eraseCount          - For write operations, set to a non-zero number to hint that we plan to write that many sectors
 *                       consecutively (including this sector)
 *
 * Returns:
 *     AFATFS_OPERATION_SUCCESS     - On success
 *     AFATFS_OPERATION_IN_PROGRESS - Card is busy, call again later
 *     AFATFS_OPERATION_FAILURE     - When the filesystem encounters a fatal error
 */
unsafe extern "C" fn afatfs_cacheSector(mut physicalSectorIndex: uint32_t,
                                        mut buffer: *mut *mut uint8_t,
                                        mut sectorFlags: uint8_t,
                                        mut eraseCount: uint32_t)
 -> afatfsOperationStatus_e {
    // We never write to the MBR, so any attempt to write there is an asyncfatfs bug
    if !afatfs_assert(sectorFlags as libc::c_int & 2 as libc::c_int ==
                          0 as libc::c_int ||
                          physicalSectorIndex !=
                              0 as libc::c_int as libc::c_uint) {
        return AFATFS_OPERATION_FAILURE
    }
    let mut cacheSectorIndex: libc::c_int =
        afatfs_allocateCacheSector(physicalSectorIndex);
    if cacheSectorIndex == -(1 as libc::c_int) {
        // We don't have enough free cache to service this request right now, try again later
        return AFATFS_OPERATION_IN_PROGRESS
    }
    let mut current_block_36: u64;
    match afatfs.cacheDescriptor[cacheSectorIndex as usize].state as
              libc::c_uint {
        2 => { return AFATFS_OPERATION_IN_PROGRESS }
        0 => {
            if sectorFlags as libc::c_int & 1 as libc::c_int !=
                   0 as libc::c_int {
                if sdcard_readBlock(physicalSectorIndex,
                                    afatfs_cacheSectorGetMemory(cacheSectorIndex),
                                    Some(afatfs_sdcardReadComplete as
                                             unsafe extern "C" fn(_:
                                                                      sdcardBlockOperation_e,
                                                                  _: uint32_t,
                                                                  _:
                                                                      *mut uint8_t,
                                                                  _: uint32_t)
                                                 -> ()),
                                    0 as libc::c_int as uint32_t) {
                    afatfs.cacheDescriptor[cacheSectorIndex as usize].state =
                        AFATFS_CACHE_STATE_READING
                }
                return AFATFS_OPERATION_IN_PROGRESS
            }
            // We only get to decide these fields if we're the first ones to cache the sector:
            afatfs.cacheDescriptor[cacheSectorIndex as
                                       usize].set_discardable(if sectorFlags
                                                                     as
                                                                     libc::c_int
                                                                     &
                                                                     8 as
                                                                         libc::c_int
                                                                     !=
                                                                     0 as
                                                                         libc::c_int
                                                                 {
                                                                  1 as
                                                                      libc::c_int
                                                              } else {
                                                                  0 as
                                                                      libc::c_int
                                                              } as
                                                                  libc::c_uint);
            // Don't bother pre-erasing for small block sequences
            if eraseCount < 4 as libc::c_int as libc::c_uint {
                eraseCount = 0 as libc::c_int as uint32_t
            } else {
                eraseCount =
                    ({
                         let mut _a: uint32_t = eraseCount;
                         let mut _b: uint32_t =
                             65535 as libc::c_int as uint32_t;
                         if _a < _b { _a } else { _b }
                     })
                // If caller asked for a longer chain of sectors we silently truncate that here
            }
            afatfs.cacheDescriptor[cacheSectorIndex as
                                       usize].consecutiveEraseBlockCount =
                eraseCount as uint16_t;
            current_block_36 = 3956267271606211338;
        }
        3 | 1 => { current_block_36 = 3956267271606211338; }
        4 => { current_block_36 = 2635160888021392008; }
        _ => {
            // Cache block in unknown state, should never happen
            afatfs_assert(0 as libc::c_int != 0);
            return AFATFS_OPERATION_FAILURE
        }
    }
    match current_block_36 {
        3956267271606211338 => {
            if sectorFlags as libc::c_int & 2 as libc::c_int !=
                   0 as libc::c_int {
                afatfs_cacheSectorMarkDirty(&mut *afatfs.cacheDescriptor.as_mut_ptr().offset(cacheSectorIndex
                                                                                                 as
                                                                                                 isize));
            }
        }
        _ => { }
    }
    if sectorFlags as libc::c_int & 4 as libc::c_int != 0 as libc::c_int {
        afatfs.cacheDescriptor[cacheSectorIndex as
                                   usize].set_locked(1 as libc::c_int as
                                                         libc::c_uint)
    }
    if sectorFlags as libc::c_int & 16 as libc::c_int != 0 as libc::c_int {
        afatfs.cacheDescriptor[cacheSectorIndex as
                                   usize].set_retainCount(afatfs.cacheDescriptor[cacheSectorIndex
                                                                                     as
                                                                                     usize].retainCount()
                                                              + 1)
    }
    *buffer = afatfs_cacheSectorGetMemory(cacheSectorIndex);
    return AFATFS_OPERATION_SUCCESS;
}
/* *
 * Parse the details out of the given MBR sector (512 bytes long). Return true if a compatible filesystem was found.
 */
unsafe extern "C" fn afatfs_parseMBR(mut sector: *const uint8_t) -> bool {
    // Check MBR signature
    if *sector.offset((512 as libc::c_int - 2 as libc::c_int) as isize) as
           libc::c_int != 0x55 as libc::c_int ||
           *sector.offset((512 as libc::c_int - 1 as libc::c_int) as isize) as
               libc::c_int != 0xaa as libc::c_int {
        return 0 as libc::c_int != 0
    }
    let mut partition: *mut mbrPartitionEntry_t =
        sector.offset(446 as libc::c_int as isize) as
            *mut mbrPartitionEntry_t;
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 4 as libc::c_int {
        if (*partition.offset(i as isize)).lbaBegin >
               0 as libc::c_int as libc::c_uint &&
               ((*partition.offset(i as isize)).type_0 as libc::c_int ==
                    0xb as libc::c_int ||
                    (*partition.offset(i as isize)).type_0 as libc::c_int ==
                        0xc as libc::c_int ||
                    (*partition.offset(i as isize)).type_0 as libc::c_int ==
                        0x6 as libc::c_int ||
                    (*partition.offset(i as isize)).type_0 as libc::c_int ==
                        0xe as libc::c_int) {
            afatfs.partitionStartSector =
                (*partition.offset(i as isize)).lbaBegin;
            return 1 as libc::c_int != 0
        }
        i += 1
    }
    return 0 as libc::c_int != 0;
}
unsafe extern "C" fn afatfs_parseVolumeID(mut sector: *const uint8_t)
 -> bool {
    let mut volume: *mut fatVolumeID_t = sector as *mut fatVolumeID_t;
    afatfs.filesystemType = FAT_FILESYSTEM_TYPE_INVALID;
    if (*volume).bytesPerSector as libc::c_int != 512 as libc::c_int ||
           (*volume).numFATs as libc::c_int != 2 as libc::c_int ||
           *sector.offset(510 as libc::c_int as isize) as libc::c_int !=
               0x55 as libc::c_int ||
           *sector.offset(511 as libc::c_int as isize) as libc::c_int !=
               0xaa as libc::c_int {
        return 0 as libc::c_int != 0
    }
    afatfs.fatStartSector =
        afatfs.partitionStartSector.wrapping_add((*volume).reservedSectorCount
                                                     as libc::c_uint);
    afatfs.sectorsPerCluster = (*volume).sectorsPerCluster as uint32_t;
    if afatfs.sectorsPerCluster < 1 as libc::c_int as libc::c_uint ||
           afatfs.sectorsPerCluster > 128 as libc::c_int as libc::c_uint ||
           !isPowerOfTwo(afatfs.sectorsPerCluster) {
        return 0 as libc::c_int != 0
    }
    afatfs.byteInClusterMask =
        (512 as libc::c_int as
             libc::c_uint).wrapping_mul(afatfs.sectorsPerCluster).wrapping_sub(1
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   libc::c_uint);
    afatfs.fatSectors =
        if (*volume).FATSize16 as libc::c_int != 0 as libc::c_int {
            (*volume).FATSize16 as libc::c_uint
        } else { (*volume).fatDescriptor.fat32.FATSize32 };
    // Always zero on FAT32 since rootEntryCount is always zero (this is non-zero on FAT16)
    afatfs.rootDirectorySectors =
        (((*volume).rootEntryCount as libc::c_int * 32 as libc::c_int +
              ((*volume).bytesPerSector as libc::c_int - 1 as libc::c_int)) /
             (*volume).bytesPerSector as libc::c_int) as uint32_t;
    let mut totalSectors: uint32_t =
        if (*volume).totalSectors16 as libc::c_int != 0 as libc::c_int {
            (*volume).totalSectors16 as libc::c_uint
        } else { (*volume).totalSectors32 };
    let mut dataSectors: uint32_t =
        totalSectors.wrapping_sub(((*volume).reservedSectorCount as
                                       libc::c_uint).wrapping_add((2 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_uint).wrapping_mul(afatfs.fatSectors)).wrapping_add(afatfs.rootDirectorySectors));
    afatfs.numClusters =
        dataSectors.wrapping_div((*volume).sectorsPerCluster as libc::c_uint);
    if afatfs.numClusters <= 4084 as libc::c_int as libc::c_uint {
        afatfs.filesystemType = FAT_FILESYSTEM_TYPE_FAT12;
        return 0 as libc::c_int != 0
        // FAT12 is not a supported filesystem
    } else {
        if afatfs.numClusters <= 65524 as libc::c_int as libc::c_uint {
            afatfs.filesystemType = FAT_FILESYSTEM_TYPE_FAT16
        } else { afatfs.filesystemType = FAT_FILESYSTEM_TYPE_FAT32 }
    }
    if afatfs.filesystemType as libc::c_uint ==
           FAT_FILESYSTEM_TYPE_FAT32 as libc::c_int as libc::c_uint {
        afatfs.rootDirectoryCluster =
            (*volume).fatDescriptor.fat32.rootCluster
    } else {
        // FAT16 doesn't store the root directory in clusters
        afatfs.rootDirectoryCluster = 0 as libc::c_int as uint32_t
    }
    let mut endOfFATs: uint32_t =
        afatfs.fatStartSector.wrapping_add((2 as libc::c_int as
                                                libc::c_uint).wrapping_mul(afatfs.fatSectors));
    afatfs.clusterStartSector =
        endOfFATs.wrapping_add(afatfs.rootDirectorySectors);
    return 1 as libc::c_int != 0;
}
/* *
 * Get the position of the FAT entry for the cluster with the given number.
 */
unsafe extern "C" fn afatfs_getFATPositionForCluster(mut cluster: uint32_t,
                                                     mut fatSectorIndex:
                                                         *mut uint32_t,
                                                     mut fatSectorEntryIndex:
                                                         *mut uint32_t) {
    if afatfs.filesystemType as libc::c_uint ==
           FAT_FILESYSTEM_TYPE_FAT16 as libc::c_int as libc::c_uint {
        let mut entriesPerFATSector: uint32_t =
            (512 as libc::c_int as
                 libc::c_ulong).wrapping_div(::core::mem::size_of::<uint16_t>()
                                                 as libc::c_ulong) as
                uint32_t;
        *fatSectorIndex = cluster.wrapping_div(entriesPerFATSector);
        *fatSectorEntryIndex =
            cluster &
                entriesPerFATSector.wrapping_sub(1 as libc::c_int as
                                                     libc::c_uint)
    } else {
        let mut entriesPerFATSector_0: uint32_t =
            (512 as libc::c_int as
                 libc::c_ulong).wrapping_div(::core::mem::size_of::<uint32_t>()
                                                 as libc::c_ulong) as
                uint32_t;
        *fatSectorIndex =
            fat32_decodeClusterNumber(cluster).wrapping_div(entriesPerFATSector_0);
        *fatSectorEntryIndex =
            cluster &
                entriesPerFATSector_0.wrapping_sub(1 as libc::c_int as
                                                       libc::c_uint)
    };
}
unsafe extern "C" fn afatfs_FATIsEndOfChainMarker(mut clusterNumber: uint32_t)
 -> bool {
    if afatfs.filesystemType as libc::c_uint ==
           FAT_FILESYSTEM_TYPE_FAT32 as libc::c_int as libc::c_uint {
        return fat32_isEndOfChainMarker(clusterNumber)
    } else { return fat16_isEndOfChainMarker(clusterNumber as uint16_t) };
}
/* *
 * Look up the FAT to find out which cluster follows the one with the given number and store it into *nextCluster.
 *
 * Use fat_isFreeSpace() and fat_isEndOfChainMarker() on nextCluster to distinguish those special values from regular
 * cluster numbers.
 *
 * Note that if you're trying to find the next cluster of a file, you should be calling afatfs_fileGetNextCluster()
 * instead, as that one supports contiguous freefile-based files (which needn't consult the FAT).
 *
 * Returns:
 *     AFATFS_OPERATION_IN_PROGRESS - FS is busy right now, call again later
 *     AFATFS_OPERATION_SUCCESS     - *nextCluster is set to the next cluster number
 */
unsafe extern "C" fn afatfs_FATGetNextCluster(mut fatIndex: libc::c_int,
                                              mut cluster: uint32_t,
                                              mut nextCluster: *mut uint32_t)
 -> afatfsOperationStatus_e {
    let mut fatSectorIndex: uint32_t = 0;
    let mut fatSectorEntryIndex: uint32_t = 0;
    let mut sector: afatfsFATSector_t =
        afatfsFATSector_t{bytes: 0 as *mut uint8_t,};
    afatfs_getFATPositionForCluster(cluster, &mut fatSectorIndex,
                                    &mut fatSectorEntryIndex);
    let mut result: afatfsOperationStatus_e =
        afatfs_cacheSector(afatfs_fatSectorToPhysical(fatIndex,
                                                      fatSectorIndex),
                           &mut sector.bytes, 1 as libc::c_int as uint8_t,
                           0 as libc::c_int as uint32_t);
    if result as libc::c_uint ==
           AFATFS_OPERATION_SUCCESS as libc::c_int as libc::c_uint {
        if afatfs.filesystemType as libc::c_uint ==
               FAT_FILESYSTEM_TYPE_FAT16 as libc::c_int as libc::c_uint {
            *nextCluster =
                *sector.fat16.offset(fatSectorEntryIndex as isize) as uint32_t
        } else {
            *nextCluster =
                fat32_decodeClusterNumber(*sector.fat32.offset(fatSectorEntryIndex
                                                                   as isize))
        }
    }
    return result;
}
/* *
 * Set the cluster number that follows the given cluster. Pass 0xFFFFFFFF for nextCluster to terminate the FAT chain.
 *
 * Returns:
 *     AFATFS_OPERATION_SUCCESS     - On success
 *     AFATFS_OPERATION_IN_PROGRESS - Card is busy, call again later
 *     AFATFS_OPERATION_FAILURE     - When the filesystem encounters a fatal error
 */
unsafe extern "C" fn afatfs_FATSetNextCluster(mut startCluster: uint32_t,
                                              mut nextCluster: uint32_t)
 -> afatfsOperationStatus_e {
    let mut sector: afatfsFATSector_t =
        afatfsFATSector_t{bytes: 0 as *mut uint8_t,};
    let mut fatSectorIndex: uint32_t = 0;
    let mut fatSectorEntryIndex: uint32_t = 0;
    let mut fatPhysicalSector: uint32_t = 0;
    let mut result: afatfsOperationStatus_e = AFATFS_OPERATION_IN_PROGRESS;
    afatfs_getFATPositionForCluster(startCluster, &mut fatSectorIndex,
                                    &mut fatSectorEntryIndex);
    fatPhysicalSector =
        afatfs_fatSectorToPhysical(0 as libc::c_int, fatSectorIndex);
    result =
        afatfs_cacheSector(fatPhysicalSector, &mut sector.bytes,
                           (1 as libc::c_int | 2 as libc::c_int) as uint8_t,
                           0 as libc::c_int as uint32_t);
    if result as libc::c_uint ==
           AFATFS_OPERATION_SUCCESS as libc::c_int as libc::c_uint {
        if afatfs.filesystemType as libc::c_uint ==
               FAT_FILESYSTEM_TYPE_FAT16 as libc::c_int as libc::c_uint {
            *sector.fat16.offset(fatSectorEntryIndex as isize) =
                nextCluster as uint16_t
        } else {
            *sector.fat32.offset(fatSectorEntryIndex as isize) = nextCluster
        }
    }
    return result;
}
/* *
 * Bring the logical filesize up to date with the current cursor position.
 */
unsafe extern "C" fn afatfs_fileUpdateFilesize(mut file: *mut afatfsFile_t) {
    (*file).logicalSize =
        ({
             let mut _a: uint32_t = (*file).logicalSize;
             let mut _b: uint32_t = (*file).cursorOffset;
             if _a > _b { _a } else { _b }
         });
}
unsafe extern "C" fn afatfs_fileUnlockCacheSector(mut file: afatfsFilePtr_t) {
    if (*file).writeLockedCacheIndex as libc::c_int != -(1 as libc::c_int) {
        afatfs.cacheDescriptor[(*file).writeLockedCacheIndex as
                                   usize].set_locked(0 as libc::c_int as
                                                         libc::c_uint);
        (*file).writeLockedCacheIndex = -(1 as libc::c_int) as int8_t
    }
    if (*file).readRetainCacheIndex as libc::c_int != -(1 as libc::c_int) {
        afatfs.cacheDescriptor[(*file).readRetainCacheIndex as
                                   usize].set_retainCount(({
                                                               let mut _a:
                                                                       libc::c_int =
                                                                   afatfs.cacheDescriptor[(*file).readRetainCacheIndex
                                                                                              as
                                                                                              usize].retainCount()
                                                                       as
                                                                       libc::c_int
                                                                       -
                                                                       1 as
                                                                           libc::c_int;
                                                               let mut _b:
                                                                       libc::c_int =
                                                                   0 as
                                                                       libc::c_int;
                                                               if _a > _b {
                                                                   _a
                                                               } else { _b }
                                                           }) as
                                                              libc::c_uint);
        (*file).readRetainCacheIndex = -(1 as libc::c_int) as int8_t
    };
}
/* *
 * Starting from and including the given cluster number, find the number of the first cluster which matches the given
 * condition.
 *
 * searchLimit - Last cluster to examine (exclusive). To search the entire volume, pass:
 *                   afatfs.numClusters + FAT_SMALLEST_LEGAL_CLUSTER_NUMBER
 *
 * Condition:
 *     CLUSTER_SEARCH_FREE_AT_BEGINNING_OF_FAT_SECTOR - Find a cluster marked as free in the FAT which lies at the
 *         beginning of its FAT sector. The passed initial search 'cluster' must correspond to the first entry of a FAT sector.
 *     CLUSTER_SEARCH_FREE            - Find a cluster marked as free in the FAT
 *     CLUSTER_SEARCH_OCCUPIED        - Find a cluster marked as occupied in the FAT.
 *
 * Returns:
 *     AFATFS_FIND_CLUSTER_FOUND       - A cluster matching the criteria was found and stored in *cluster
 *     AFATFS_FIND_CLUSTER_IN_PROGRESS - The search is not over, call this routine again later with the updated *cluster value to resume
 *     AFATFS_FIND_CLUSTER_FATAL       - An unexpected read error occurred, the volume should be abandoned
 *     AFATFS_FIND_CLUSTER_NOT_FOUND   - The entire device was searched without finding a suitable cluster (the
 *                                       *cluster points to just beyond the final cluster).
 */
unsafe extern "C" fn afatfs_findClusterWithCondition(mut condition:
                                                         afatfsClusterSearchCondition_e,
                                                     mut cluster:
                                                         *mut uint32_t,
                                                     mut searchLimit:
                                                         uint32_t)
 -> afatfsFindClusterStatus_e {
    let mut sector: afatfsFATSector_t =
        afatfsFATSector_t{bytes: 0 as *mut uint8_t,};
    let mut fatSectorIndex: uint32_t = 0;
    let mut fatSectorEntryIndex: uint32_t = 0;
    let mut fatEntriesPerSector: uint32_t = afatfs_fatEntriesPerSector();
    let mut lookingForFree: bool =
        condition as libc::c_uint ==
            CLUSTER_SEARCH_FREE_AT_BEGINNING_OF_FAT_SECTOR as libc::c_int as
                libc::c_uint ||
            condition as libc::c_uint ==
                CLUSTER_SEARCH_FREE as libc::c_int as libc::c_uint;
    let mut jump: libc::c_int = 0;
    // Get the FAT entry which corresponds to this cluster so we can begin our search there
    afatfs_getFATPositionForCluster(*cluster, &mut fatSectorIndex,
                                    &mut fatSectorEntryIndex);
    match condition as libc::c_uint {
        0 => {
            jump = fatEntriesPerSector as libc::c_int;
            // We're supposed to call this routine with the cluster properly aligned
            if !afatfs_assert(fatSectorEntryIndex ==
                                  0 as libc::c_int as libc::c_uint) {
                return AFATFS_FIND_CLUSTER_FATAL
            }
        }
        2 | 1 => { jump = 1 as libc::c_int }
        _ => {
            afatfs_assert(0 as libc::c_int != 0);
            return AFATFS_FIND_CLUSTER_FATAL
        }
    }
    while *cluster < searchLimit {
        // If we're looking inside the freefile, we won't find any free clusters! Skip it!
        if afatfs.freeFile.logicalSize > 0 as libc::c_int as libc::c_uint &&
               *cluster == afatfs.freeFile.firstCluster {
            *cluster =
                (*cluster as
                     libc::c_uint).wrapping_add(afatfs.freeFile.logicalSize.wrapping_add(afatfs_clusterSize()).wrapping_sub(1
                                                                                                                                as
                                                                                                                                libc::c_int
                                                                                                                                as
                                                                                                                                libc::c_uint).wrapping_div(afatfs_clusterSize()))
                    as uint32_t as uint32_t;
            // Go back to check that the new cluster number is within the volume
            *cluster = roundUpTo(*cluster, jump as uint32_t)
        } else {
            let mut status: afatfsOperationStatus_e =
                afatfs_cacheSector(afatfs_fatSectorToPhysical(0 as
                                                                  libc::c_int,
                                                              fatSectorIndex),
                                   &mut sector.bytes,
                                   (1 as libc::c_int | 8 as libc::c_int) as
                                       uint8_t, 0 as libc::c_int as uint32_t);
            match status as libc::c_uint {
                1 => {
                    loop  {
                        let mut clusterNumber: uint32_t = 0;
                        match afatfs.filesystemType as libc::c_uint {
                            2 => {
                                clusterNumber =
                                    *sector.fat16.offset(fatSectorEntryIndex
                                                             as isize) as
                                        uint32_t
                            }
                            3 => {
                                clusterNumber =
                                    fat32_decodeClusterNumber(*sector.fat32.offset(fatSectorEntryIndex
                                                                                       as
                                                                                       isize))
                            }
                            _ => { return AFATFS_FIND_CLUSTER_FATAL }
                        }
                        if fat_isFreeSpace(clusterNumber) as libc::c_int ==
                               lookingForFree as libc::c_int {
                            // Maintain alignment
                            /*
                         * The final FAT sector may have fewer than fatEntriesPerSector entries in it, so we need to
                         * check the cluster number is valid here before we report a bogus success!
                         */
                            if *cluster < searchLimit {
                                return AFATFS_FIND_CLUSTER_FOUND
                            } else {
                                *cluster = searchLimit;
                                return AFATFS_FIND_CLUSTER_NOT_FOUND
                            }
                        }
                        *cluster =
                            (*cluster as
                                 libc::c_uint).wrapping_add(jump as
                                                                libc::c_uint)
                                as uint32_t as uint32_t;
                        fatSectorEntryIndex =
                            (fatSectorEntryIndex as
                                 libc::c_uint).wrapping_add(jump as
                                                                libc::c_uint)
                                as uint32_t as uint32_t;
                        if !(fatSectorEntryIndex < fatEntriesPerSector) {
                            break ;
                        }
                    }
                    // Move on to the next FAT sector
                    fatSectorIndex = fatSectorIndex.wrapping_add(1);
                    fatSectorEntryIndex = 0 as libc::c_int as uint32_t
                }
                2 => { return AFATFS_FIND_CLUSTER_FATAL }
                0 => { return AFATFS_FIND_CLUSTER_IN_PROGRESS }
                _ => { }
            }
        }
    }
    // We looked at every available cluster and didn't find one matching the condition
    *cluster = searchLimit;
    return AFATFS_FIND_CLUSTER_NOT_FOUND;
}
/* *
 * Get the cluster that follows the currentCluster in the FAT chain for the given file.
 *
 * Returns:
 *     AFATFS_OPERATION_IN_PROGRESS - FS is busy right now, call again later
 *     AFATFS_OPERATION_SUCCESS     - *nextCluster is set to the next cluster number
 */
unsafe extern "C" fn afatfs_fileGetNextCluster(mut file: afatfsFilePtr_t,
                                               mut currentCluster: uint32_t,
                                               mut nextCluster: *mut uint32_t)
 -> afatfsOperationStatus_e {
    if (*file).mode as libc::c_int & 8 as libc::c_int != 0 as libc::c_int {
        let mut freeFileStart: uint32_t = afatfs.freeFile.firstCluster;
        afatfs_assert(currentCluster.wrapping_add(1 as libc::c_int as
                                                      libc::c_uint) <=
                          freeFileStart);
        // Would the next cluster lie outside the allocated file? (i.e. beyond the end of the file into the start of the freefile)
        if currentCluster.wrapping_add(1 as libc::c_int as libc::c_uint) ==
               freeFileStart {
            *nextCluster = 0 as libc::c_int as uint32_t
        } else {
            *nextCluster =
                currentCluster.wrapping_add(1 as libc::c_int as libc::c_uint)
        }
        return AFATFS_OPERATION_SUCCESS
    } else {
        return afatfs_FATGetNextCluster(0 as libc::c_int, currentCluster,
                                        nextCluster)
    };
}
/* *
 * Update the FAT to fill the contiguous series of clusters with indexes [*startCluster...endCluster) with the
 * specified pattern.
 *
 * AFATFS_FAT_PATTERN_TERMINATED_CHAIN - Chain the clusters together in linear sequence and terminate the final cluster
 * AFATFS_FAT_PATTERN_CHAIN            - Chain the clusters together without terminating the final entry
 * AFATFS_FAT_PATTERN_FREE             - Mark the clusters as free space
 *
 * Returns -
 *     AFATFS_OPERATION_SUCCESS        - When the entire chain has been written
 *     AFATFS_OPERATION_IN_PROGRESS    - Call again later with the updated *startCluster value in order to resume writing.
 */
unsafe extern "C" fn afatfs_FATFillWithPattern(mut pattern:
                                                   afatfsFATPattern_e,
                                               mut startCluster:
                                                   *mut uint32_t,
                                               mut endCluster: uint32_t)
 -> afatfsOperationStatus_e {
    let mut sector: afatfsFATSector_t =
        afatfsFATSector_t{bytes: 0 as *mut uint8_t,};
    let mut fatSectorIndex: uint32_t = 0;
    let mut firstEntryIndex: uint32_t = 0;
    let mut fatPhysicalSector: uint32_t = 0;
    let mut fatEntrySize: uint8_t = 0;
    let mut nextCluster: uint32_t = 0;
    let mut result: afatfsOperationStatus_e = AFATFS_OPERATION_IN_PROGRESS;
    let mut eraseSectorCount: uint32_t = 0;
    // Find the position of the initial cluster to begin our fill
    afatfs_getFATPositionForCluster(*startCluster, &mut fatSectorIndex,
                                    &mut firstEntryIndex);
    fatPhysicalSector =
        afatfs_fatSectorToPhysical(0 as libc::c_int, fatSectorIndex);
    // How many consecutive FAT sectors will we be overwriting?
    eraseSectorCount =
        endCluster.wrapping_sub(*startCluster).wrapping_add(firstEntryIndex).wrapping_add(afatfs_fatEntriesPerSector()).wrapping_sub(1
                                                                                                                                         as
                                                                                                                                         libc::c_int
                                                                                                                                         as
                                                                                                                                         libc::c_uint).wrapping_div(afatfs_fatEntriesPerSector());
    while *startCluster < endCluster {
        // The last entry we will fill inside this sector (exclusive):
        let mut lastEntryIndex: uint32_t =
            ({
                 let mut _a: libc::c_uint =
                     firstEntryIndex.wrapping_add(endCluster.wrapping_sub(*startCluster));
                 let mut _b: uint32_t = afatfs_fatEntriesPerSector();
                 if _a < _b { _a } else { _b }
             });
        let mut cacheFlags: uint8_t =
            (2 as libc::c_int | 8 as libc::c_int) as uint8_t;
        if firstEntryIndex > 0 as libc::c_int as libc::c_uint ||
               lastEntryIndex < afatfs_fatEntriesPerSector() {
            // We're not overwriting the entire FAT sector so we must read the existing contents
            cacheFlags =
                (cacheFlags as libc::c_int | 1 as libc::c_int) as uint8_t
        }
        result =
            afatfs_cacheSector(fatPhysicalSector, &mut sector.bytes,
                               cacheFlags, eraseSectorCount);
        if result as libc::c_uint !=
               AFATFS_OPERATION_SUCCESS as libc::c_int as libc::c_uint {
            return result
        }
        match pattern as libc::c_uint {
            1 | 0 => {
                nextCluster =
                    (*startCluster).wrapping_add(1 as libc::c_int as
                                                     libc::c_uint);
                // Write all the "next cluster" pointers
                if afatfs.filesystemType as libc::c_uint ==
                       FAT_FILESYSTEM_TYPE_FAT16 as libc::c_int as
                           libc::c_uint {
                    let mut i: uint32_t = firstEntryIndex;
                    while i < lastEntryIndex {
                        *sector.fat16.offset(i as isize) =
                            nextCluster as uint16_t;
                        i = i.wrapping_add(1);
                        nextCluster = nextCluster.wrapping_add(1)
                    }
                } else {
                    let mut i_0: uint32_t = firstEntryIndex;
                    while i_0 < lastEntryIndex {
                        *sector.fat32.offset(i_0 as isize) = nextCluster;
                        i_0 = i_0.wrapping_add(1);
                        nextCluster = nextCluster.wrapping_add(1)
                    }
                }
                *startCluster =
                    (*startCluster as
                         libc::c_uint).wrapping_add(lastEntryIndex.wrapping_sub(firstEntryIndex))
                        as uint32_t as uint32_t;
                if pattern as libc::c_uint ==
                       AFATFS_FAT_PATTERN_TERMINATED_CHAIN as libc::c_int as
                           libc::c_uint && *startCluster == endCluster {
                    // We completed the chain! Overwrite the last entry we wrote with the terminator for the end of the chain
                    if afatfs.filesystemType as libc::c_uint ==
                           FAT_FILESYSTEM_TYPE_FAT16 as libc::c_int as
                               libc::c_uint {
                        *sector.fat16.offset(lastEntryIndex.wrapping_sub(1 as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint)
                                                 as isize) =
                            0xffff as libc::c_int as uint16_t
                    } else {
                        *sector.fat32.offset(lastEntryIndex.wrapping_sub(1 as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint)
                                                 as isize) =
                            0xffffffff as libc::c_uint
                    }
                }
            }
            2 => {
                fatEntrySize =
                    if afatfs.filesystemType as libc::c_uint ==
                           FAT_FILESYSTEM_TYPE_FAT16 as libc::c_int as
                               libc::c_uint {
                        ::core::mem::size_of::<uint16_t>() as libc::c_ulong
                    } else {
                        ::core::mem::size_of::<uint32_t>() as libc::c_ulong
                    } as uint8_t;
                memset(sector.bytes.offset(firstEntryIndex.wrapping_mul(fatEntrySize
                                                                            as
                                                                            libc::c_uint)
                                               as isize) as *mut libc::c_void,
                       0 as libc::c_int,
                       lastEntryIndex.wrapping_sub(firstEntryIndex).wrapping_mul(fatEntrySize
                                                                                     as
                                                                                     libc::c_uint)
                           as libc::c_ulong);
                *startCluster =
                    (*startCluster as
                         libc::c_uint).wrapping_add(lastEntryIndex.wrapping_sub(firstEntryIndex))
                        as uint32_t as uint32_t
            }
            _ => { }
        }
        fatPhysicalSector = fatPhysicalSector.wrapping_add(1);
        eraseSectorCount = eraseSectorCount.wrapping_sub(1);
        firstEntryIndex = 0 as libc::c_int as uint32_t
    }
    return AFATFS_OPERATION_SUCCESS;
}
/* *
 * Write the directory entry for the file into its `directoryEntryPos` position in its containing directory.
 *
 * mode:
 *     AFATFS_SAVE_DIRECTORY_NORMAL    - Store the file's physical size, not the logical size, in the directory entry
 *     AFATFS_SAVE_DIRECTORY_FOR_CLOSE - We're done extending the file so we can write the logical size now.
 *     AFATFS_SAVE_DIRECTORY_DELETED   - Mark the directory entry as deleted
 *
 * Returns:
 *     AFATFS_OPERATION_SUCCESS - The directory entry has been stored into the directory sector in cache.
 *     AFATFS_OPERATION_IN_PROGRESS - Cache is too busy, retry later
 *     AFATFS_OPERATION_FAILURE - If the filesystem enters the fatal state
 */
unsafe extern "C" fn afatfs_saveDirectoryEntry(mut file: afatfsFilePtr_t,
                                               mut mode:
                                                   afatfsSaveDirectoryEntryMode_e)
 -> afatfsOperationStatus_e {
    let mut sector: *mut uint8_t = 0 as *mut uint8_t;
    let mut result: afatfsOperationStatus_e = AFATFS_OPERATION_IN_PROGRESS;
    if (*file).directoryEntryPos.sectorNumberPhysical ==
           0 as libc::c_int as libc::c_uint {
        return AFATFS_OPERATION_SUCCESS
        // Root directories don't have a directory entry
    }
    result =
        afatfs_cacheSector((*file).directoryEntryPos.sectorNumberPhysical,
                           &mut sector,
                           (1 as libc::c_int | 2 as libc::c_int) as uint8_t,
                           0 as libc::c_int as uint32_t);
    if result as libc::c_uint ==
           AFATFS_OPERATION_SUCCESS as libc::c_int as libc::c_uint {
        if afatfs_assert((*file).directoryEntryPos.entryIndex as libc::c_int
                             >= 0 as libc::c_int) {
            let mut entry: *mut fatDirectoryEntry_t =
                (sector as
                     *mut fatDirectoryEntry_t).offset((*file).directoryEntryPos.entryIndex
                                                          as libc::c_int as
                                                          isize);
            let mut current_block_8: u64;
            match mode as libc::c_uint {
                0 => {
                    /* We exaggerate the length of the written file so that if power is lost, the end of the file will
                    * still be readable (though the very tail of the file will be uninitialized data).
                    *
                    * This way we can avoid updating the directory entry too many times during fwrites() on the file.
                    */
                    (*entry).fileSize = (*file).physicalSize;
                    current_block_8 = 12599329904712511516;
                }
                2 => {
                    (*entry).filename[0 as libc::c_int as usize] =
                        0xe5 as libc::c_int as libc::c_char;
                    current_block_8 = 743387486898113897;
                }
                1 => { current_block_8 = 743387486898113897; }
                _ => { current_block_8 = 12599329904712511516; }
            }
            match current_block_8 {
                743387486898113897 => {
                    // We write the true length of the file on close.
                    (*entry).fileSize = (*file).logicalSize
                }
                _ => { }
            }
            // (sub)directories don't store a filesize in their directory entry:
            if (*file).type_0 as libc::c_uint ==
                   AFATFS_FILE_TYPE_DIRECTORY as libc::c_int as libc::c_uint {
                (*entry).fileSize = 0 as libc::c_int as uint32_t
            }
            (*entry).firstClusterHigh =
                ((*file).firstCluster >> 16 as libc::c_int) as uint16_t;
            (*entry).firstClusterLow =
                ((*file).firstCluster & 0xffff as libc::c_int as libc::c_uint)
                    as uint16_t
        } else { return AFATFS_OPERATION_FAILURE }
    }
    return result;
}
/* *
 * Attempt to add a free cluster to the end of the given file. If the file was previously empty, the directory entry
 * is updated to point to the new cluster.
 *
 * Returns:
 *     AFATFS_OPERATION_SUCCESS     - The cluster has been appended
 *     AFATFS_OPERATION_IN_PROGRESS - Cache was busy, so call again later to continue
 *     AFATFS_OPERATION_FAILURE     - Cluster could not be appended because the filesystem ran out of space
 *                                    (afatfs.filesystemFull is set to true)
 *
 * If the file's operation was AFATFS_FILE_OPERATION_APPEND_FREE_CLUSTER, the file operation is cleared upon completion,
 * otherwise it is left alone so that this operation can be called as a sub-operation of some other operation on the
 * file.
 */
unsafe extern "C" fn afatfs_appendRegularFreeClusterContinue(mut file:
                                                                 *mut afatfsFile_t)
 -> afatfsOperationStatus_e {
    let mut opState: *mut afatfsAppendFreeCluster_t =
        &mut (*file).operation.state.appendFreeCluster;
    let mut status: afatfsOperationStatus_e = AFATFS_OPERATION_IN_PROGRESS;
    loop  {
        match (*opState).phase as libc::c_uint {
            0 => {
                match afatfs_findClusterWithCondition(CLUSTER_SEARCH_FREE,
                                                      &mut (*opState).searchCluster,
                                                      afatfs.numClusters.wrapping_add(2
                                                                                          as
                                                                                          libc::c_int
                                                                                          as
                                                                                          libc::c_uint))
                          as libc::c_uint {
                    1 => {
                        afatfs.lastClusterAllocated =
                            (*opState).searchCluster;
                        // Make the cluster available for us to write in
                        (*file).cursorCluster = (*opState).searchCluster;
                        (*file).physicalSize =
                            ((*file).physicalSize as
                                 libc::c_uint).wrapping_add(afatfs_clusterSize())
                                as uint32_t as uint32_t;
                        if (*opState).previousCluster ==
                               0 as libc::c_int as libc::c_uint {
                            // This is the new first cluster in the file
                            (*file).firstCluster = (*opState).searchCluster
                        }
                        (*opState).phase =
                            AFATFS_APPEND_FREE_CLUSTER_PHASE_UPDATE_FAT1
                    }
                    2 | 3 => {
                        // We couldn't find an empty cluster to append to the file
                        (*opState).phase =
                            AFATFS_APPEND_FREE_CLUSTER_PHASE_FAILURE
                    }
                    0 | _ => { break ; }
                }
            }
            1 => {
                // Terminate the new cluster
                status =
                    afatfs_FATSetNextCluster((*opState).searchCluster,
                                             0xffffffff as libc::c_uint);
                if !(status as libc::c_uint ==
                         AFATFS_OPERATION_SUCCESS as libc::c_int as
                             libc::c_uint) {
                    break ;
                }
                if (*opState).previousCluster != 0 {
                    (*opState).phase =
                        AFATFS_APPEND_FREE_CLUSTER_PHASE_UPDATE_FAT2
                } else {
                    (*opState).phase =
                        AFATFS_APPEND_FREE_CLUSTER_PHASE_UPDATE_FILE_DIRECTORY
                }
            }
            2 => {
                // Add the new cluster to the pre-existing chain
                status =
                    afatfs_FATSetNextCluster((*opState).previousCluster,
                                             (*opState).searchCluster);
                if !(status as libc::c_uint ==
                         AFATFS_OPERATION_SUCCESS as libc::c_int as
                             libc::c_uint) {
                    break ;
                }
                (*opState).phase =
                    AFATFS_APPEND_FREE_CLUSTER_PHASE_UPDATE_FILE_DIRECTORY
            }
            3 => {
                if !(afatfs_saveDirectoryEntry(file,
                                               AFATFS_SAVE_DIRECTORY_NORMAL)
                         as libc::c_uint ==
                         AFATFS_OPERATION_SUCCESS as libc::c_int as
                             libc::c_uint) {
                    break ;
                }
                (*opState).phase = AFATFS_APPEND_FREE_CLUSTER_PHASE_COMPLETE
            }
            4 => {
                if (*file).operation.operation as libc::c_uint ==
                       AFATFS_FILE_OPERATION_APPEND_FREE_CLUSTER as
                           libc::c_int as libc::c_uint {
                    (*file).operation.operation = AFATFS_FILE_OPERATION_NONE
                }
                return AFATFS_OPERATION_SUCCESS
            }
            5 => {
                if (*file).operation.operation as libc::c_uint ==
                       AFATFS_FILE_OPERATION_APPEND_FREE_CLUSTER as
                           libc::c_int as libc::c_uint {
                    (*file).operation.operation = AFATFS_FILE_OPERATION_NONE
                }
                afatfs.filesystemFull = 1 as libc::c_int != 0;
                return AFATFS_OPERATION_FAILURE
            }
            _ => { break ; }
        }
    }
    return AFATFS_OPERATION_IN_PROGRESS;
}
unsafe extern "C" fn afatfs_appendRegularFreeClusterInitOperationState(mut state:
                                                                           *mut afatfsAppendFreeCluster_t,
                                                                       mut previousCluster:
                                                                           uint32_t) {
    (*state).phase = AFATFS_APPEND_FREE_CLUSTER_PHASE_INITIAL;
    (*state).previousCluster = previousCluster;
    (*state).searchCluster = afatfs.lastClusterAllocated;
}
/* *
 * Queue up an operation to append a free cluster to the file and update the file's cursorCluster to point to it.
 *
 * You must seek to the end of the file first, so file.cursorCluster will be 0 for the first call, and
 * `file.cursorPreviousCluster` will be the cluster to append after.
 *
 * Note that the cursorCluster will be updated before this operation is completely finished (i.e. before the FAT is
 * updated) but you can go ahead and write to it before the operation succeeds.
 *
 * Returns:
 *     AFATFS_OPERATION_SUCCESS     - The append completed successfully
 *     AFATFS_OPERATION_IN_PROGRESS - The operation was queued on the file and will complete later
 *     AFATFS_OPERATION_FAILURE     - Operation could not be queued or append failed, check afatfs.fileSystemFull
 */
unsafe extern "C" fn afatfs_appendRegularFreeCluster(mut file:
                                                         afatfsFilePtr_t)
 -> afatfsOperationStatus_e {
    if (*file).operation.operation as libc::c_uint ==
           AFATFS_FILE_OPERATION_APPEND_FREE_CLUSTER as libc::c_int as
               libc::c_uint {
        return AFATFS_OPERATION_IN_PROGRESS
    }
    if afatfs.filesystemFull as libc::c_int != 0 ||
           afatfs_fileIsBusy(file) as libc::c_int != 0 {
        return AFATFS_OPERATION_FAILURE
    }
    (*file).operation.operation = AFATFS_FILE_OPERATION_APPEND_FREE_CLUSTER;
    afatfs_appendRegularFreeClusterInitOperationState(&mut (*file).operation.state.appendFreeCluster,
                                                      (*file).cursorPreviousCluster);
    return afatfs_appendRegularFreeClusterContinue(file);
}
/* *
 * Size of a AFATFS supercluster in bytes
 */
unsafe extern "C" fn afatfs_superClusterSize() -> uint32_t {
    return afatfs_fatEntriesPerSector().wrapping_mul(afatfs_clusterSize());
}
/* *
 * Continue to attempt to add a supercluster to the end of the given file.
 *
 * If the file operation was set to AFATFS_FILE_OPERATION_APPEND_SUPERCLUSTER and the operation completes, the file's
 * operation is cleared.
 *
 * Returns:
 *     AFATFS_OPERATION_SUCCESS     - On completion
 *     AFATFS_OPERATION_IN_PROGRESS - Operation still in progress
 */
unsafe extern "C" fn afatfs_appendSuperclusterContinue(mut file:
                                                           *mut afatfsFile_t)
 -> afatfsOperationStatus_e {
    let mut opState: *mut afatfsAppendSupercluster_t =
        &mut (*file).operation.state.appendSupercluster;
    let mut status: afatfsOperationStatus_e = AFATFS_OPERATION_FAILURE;
    loop  {
        match (*opState).phase as libc::c_uint {
            0 => {
                // Our file steals the first cluster of the freefile
                // We can go ahead and write to that space before the FAT and directory are updated
                (*file).cursorCluster = afatfs.freeFile.firstCluster;
                (*file).physicalSize =
                    ((*file).physicalSize as
                         libc::c_uint).wrapping_add(afatfs_superClusterSize())
                        as uint32_t as uint32_t;
                /* Remove the first supercluster from the freefile
             *
             * Even if the freefile becomes empty, we still don't set its first cluster to zero. This is so that
             * afatfs_fileGetNextCluster() can tell where a contiguous file ends (at the start of the freefile).
             *
             * Note that normally the freefile can't become empty because it is allocated as a non-integer number
             * of superclusters to avoid precisely this situation.
             */
                afatfs.freeFile.firstCluster =
                    (afatfs.freeFile.firstCluster as
                         libc::c_uint).wrapping_add(afatfs_fatEntriesPerSector())
                        as uint32_t as uint32_t;
                afatfs.freeFile.logicalSize =
                    (afatfs.freeFile.logicalSize as
                         libc::c_uint).wrapping_sub(afatfs_superClusterSize())
                        as uint32_t as uint32_t;
                afatfs.freeFile.physicalSize =
                    (afatfs.freeFile.physicalSize as
                         libc::c_uint).wrapping_sub(afatfs_superClusterSize())
                        as uint32_t as uint32_t;
                // The new supercluster needs to have its clusters chained contiguously and marked with a terminator at the end
                (*opState).fatRewriteStartCluster = (*file).cursorCluster;
                (*opState).fatRewriteEndCluster =
                    (*opState).fatRewriteStartCluster.wrapping_add(afatfs_fatEntriesPerSector());
                if (*opState).previousCluster ==
                       0 as libc::c_int as libc::c_uint {
                    // This is the new first cluster in the file so we need to update the directory entry
                    (*file).firstCluster = (*file).cursorCluster
                } else {
                    /*
                 * We also need to update the FAT of the supercluster that used to end the file so that it no longer
                 * terminates there
                 */
                    (*opState).fatRewriteStartCluster =
                        ((*opState).fatRewriteStartCluster as
                             libc::c_uint).wrapping_sub(afatfs_fatEntriesPerSector())
                            as uint32_t as uint32_t
                }
                (*opState).phase =
                    AFATFS_APPEND_SUPERCLUSTER_PHASE_UPDATE_FREEFILE_DIRECTORY
            }
            1 => {
                // First update the freefile's directory entry to remove the first supercluster so we don't risk cross-linking the file
                status =
                    afatfs_saveDirectoryEntry(&mut afatfs.freeFile,
                                              AFATFS_SAVE_DIRECTORY_NORMAL);
                if !(status as libc::c_uint ==
                         AFATFS_OPERATION_SUCCESS as libc::c_int as
                             libc::c_uint) {
                    break ;
                }
                (*opState).phase = AFATFS_APPEND_SUPERCLUSTER_PHASE_UPDATE_FAT
            }
            2 => {
                status =
                    afatfs_FATFillWithPattern(AFATFS_FAT_PATTERN_TERMINATED_CHAIN,
                                              &mut (*opState).fatRewriteStartCluster,
                                              (*opState).fatRewriteEndCluster);
                if !(status as libc::c_uint ==
                         AFATFS_OPERATION_SUCCESS as libc::c_int as
                             libc::c_uint) {
                    break ;
                }
                (*opState).phase =
                    AFATFS_APPEND_SUPERCLUSTER_PHASE_UPDATE_FILE_DIRECTORY
            }
            3 => {
                // Update the fileSize/firstCluster in the directory entry for the file
                status =
                    afatfs_saveDirectoryEntry(file,
                                              AFATFS_SAVE_DIRECTORY_NORMAL);
                break ;
            }
            _ => { break ; }
        }
    }
    if (status as libc::c_uint ==
            AFATFS_OPERATION_FAILURE as libc::c_int as libc::c_uint ||
            status as libc::c_uint ==
                AFATFS_OPERATION_SUCCESS as libc::c_int as libc::c_uint) &&
           (*file).operation.operation as libc::c_uint ==
               AFATFS_FILE_OPERATION_APPEND_SUPERCLUSTER as libc::c_int as
                   libc::c_uint {
        (*file).operation.operation = AFATFS_FILE_OPERATION_NONE
    }
    return status;
}
/* *
 * Attempt to queue up an operation to append the first supercluster of the freefile to the given `file` (file's cursor
 * must be at end-of-file).
 *
 * The new cluster number will be set into the file's cursorCluster.
 *
 * Returns:
 *     AFATFS_OPERATION_SUCCESS     - The append completed successfully and the file's cursorCluster has been updated
 *     AFATFS_OPERATION_IN_PROGRESS - The operation was queued on the file and will complete later, or there is already an
 *                                    append in progress.
 *     AFATFS_OPERATION_FAILURE     - Operation could not be queued (file was busy) or append failed (filesystem is full).
 *                                    Check afatfs.fileSystemFull
 */
unsafe extern "C" fn afatfs_appendSupercluster(mut file: afatfsFilePtr_t)
 -> afatfsOperationStatus_e {
    let mut superClusterSize: uint32_t = afatfs_superClusterSize();
    if (*file).operation.operation as libc::c_uint ==
           AFATFS_FILE_OPERATION_APPEND_SUPERCLUSTER as libc::c_int as
               libc::c_uint {
        return AFATFS_OPERATION_IN_PROGRESS
    }
    if afatfs.freeFile.logicalSize < superClusterSize {
        afatfs.filesystemFull = 1 as libc::c_int != 0
    }
    if afatfs.filesystemFull as libc::c_int != 0 ||
           afatfs_fileIsBusy(file) as libc::c_int != 0 {
        return AFATFS_OPERATION_FAILURE
    }
    let mut opState: *mut afatfsAppendSupercluster_t =
        &mut (*file).operation.state.appendSupercluster;
    (*file).operation.operation = AFATFS_FILE_OPERATION_APPEND_SUPERCLUSTER;
    (*opState).phase = AFATFS_APPEND_SUPERCLUSTER_PHASE_INIT;
    (*opState).previousCluster = (*file).cursorPreviousCluster;
    return afatfs_appendSuperclusterContinue(file);
}
/* *
 * Queue an operation to add a cluster of free space to the end of the file. Must be called when the file's cursor
 * is beyond the last allocated cluster.
 */
unsafe extern "C" fn afatfs_appendFreeCluster(mut file: afatfsFilePtr_t)
 -> afatfsOperationStatus_e {
    let mut status: afatfsOperationStatus_e = AFATFS_OPERATION_IN_PROGRESS;
    if (*file).mode as libc::c_int & 8 as libc::c_int != 0 as libc::c_int {
        // Steal the first cluster from the beginning of the freefile if we can
        status = afatfs_appendSupercluster(file)
    } else { status = afatfs_appendRegularFreeCluster(file) }
    return status;
}
/* *
 * Returns true if the file's cursor is sitting beyond the end of the last allocated cluster (i.e. the logical fileSize
 * is not checked).
 */
unsafe extern "C" fn afatfs_isEndOfAllocatedFile(mut file: afatfsFilePtr_t)
 -> bool {
    if (*file).type_0 as libc::c_uint ==
           AFATFS_FILE_TYPE_FAT16_ROOT_DIRECTORY as libc::c_int as
               libc::c_uint {
        return (*file).cursorOffset >=
                   (512 as libc::c_int as
                        libc::c_uint).wrapping_mul(afatfs.rootDirectorySectors)
    } else {
        return (*file).cursorCluster == 0 as libc::c_int as libc::c_uint ||
                   afatfs_FATIsEndOfChainMarker((*file).cursorCluster) as
                       libc::c_int != 0
    };
}
/* *
 * Take a lock on the sector at the current file cursor position.
 *
 * Returns a pointer to the sector buffer if successful, or NULL if at the end of file (check afatfs_isEndOfAllocatedFile())
 * or the sector has not yet been read in from disk.
 */
unsafe extern "C" fn afatfs_fileRetainCursorSectorForRead(mut file:
                                                              afatfsFilePtr_t)
 -> *mut uint8_t {
    let mut result: *mut uint8_t = 0 as *mut uint8_t;
    let mut physicalSector: uint32_t =
        afatfs_fileGetCursorPhysicalSector(file);
    /* If we've already got a locked sector then we can assume that was the same one that's at the cursor (because this
     * cache is invalidated when crossing a sector boundary)
     */
    if (*file).readRetainCacheIndex as libc::c_int != -(1 as libc::c_int) {
        if !afatfs_assert(physicalSector ==
                              afatfs.cacheDescriptor[(*file).readRetainCacheIndex
                                                         as
                                                         usize].sectorIndex) {
            return 0 as *mut uint8_t
        } // We never read the root sector using files
        result =
            afatfs_cacheSectorGetMemory((*file).readRetainCacheIndex as
                                            libc::c_int)
    } else {
        if afatfs_isEndOfAllocatedFile(file) { return 0 as *mut uint8_t }
        afatfs_assert(physicalSector > 0 as libc::c_int as libc::c_uint);
        let mut status: afatfsOperationStatus_e =
            afatfs_cacheSector(physicalSector, &mut result,
                               (1 as libc::c_int | 16 as libc::c_int) as
                                   uint8_t, 0 as libc::c_int as uint32_t);
        if status as libc::c_uint !=
               AFATFS_OPERATION_SUCCESS as libc::c_int as libc::c_uint {
            // Sector not ready for read
            return 0 as *mut uint8_t
        }
        (*file).readRetainCacheIndex =
            afatfs_getCacheDescriptorIndexForBuffer(result) as int8_t
    }
    return result;
}
/* *
 * Lock the sector at the file's cursor position for write, and return a reference to the memory for that sector.
 *
 * Returns NULL if the cache was too busy, try again later.
 */
unsafe extern "C" fn afatfs_fileLockCursorSectorForWrite(mut file:
                                                             afatfsFilePtr_t)
 -> *mut uint8_t {
    let mut status: afatfsOperationStatus_e = AFATFS_OPERATION_IN_PROGRESS;
    let mut result: *mut uint8_t = 0 as *mut uint8_t;
    let mut eraseBlockCount: uint32_t = 0;
    // Do we already have a sector locked in our cache at the cursor position?
    if (*file).writeLockedCacheIndex as libc::c_int != -(1 as libc::c_int) {
        let mut physicalSector: uint32_t =
            afatfs_fileGetCursorPhysicalSector(file);
        if !afatfs_assert(physicalSector ==
                              afatfs.cacheDescriptor[(*file).writeLockedCacheIndex
                                                         as
                                                         usize].sectorIndex) {
            return 0 as *mut uint8_t
        }
        result =
            afatfs_cacheSectorGetMemory((*file).writeLockedCacheIndex as
                                            libc::c_int)
    } else {
        // Find / allocate a sector and lock it in the cache so we can rely on it sticking around
        // Are we at the start of an empty file or the end of a non-empty file? If so we need to add a cluster
        if afatfs_isEndOfAllocatedFile(file) as libc::c_int != 0 &&
               afatfs_appendFreeCluster(file) as libc::c_uint !=
                   AFATFS_OPERATION_SUCCESS as libc::c_int as libc::c_uint {
            // The extension of the file is in progress so please call us again later to try again
            return 0 as *mut uint8_t
        }
        let mut physicalSector_0: uint32_t =
            afatfs_fileGetCursorPhysicalSector(file);
        let mut cacheFlags: uint8_t =
            (2 as libc::c_int | 4 as libc::c_int) as uint8_t;
        let mut cursorOffsetInSector: uint32_t =
            (*file).cursorOffset.wrapping_rem(512 as libc::c_int as
                                                  libc::c_uint);
        let mut offsetOfStartOfSector: uint32_t =
            (*file).cursorOffset &
                !(512 as libc::c_int as
                      uint32_t).wrapping_sub(1 as libc::c_int as
                                                 libc::c_uint);
        let mut offsetOfEndOfSector: uint32_t =
            offsetOfStartOfSector.wrapping_add(512 as libc::c_int as
                                                   libc::c_uint);
        /*
         * If there is data before the write point in this sector, or there could be data after the write-point
         * then we need to have the original contents of the sector in the cache for us to merge into
         */
        if cursorOffsetInSector > 0 as libc::c_int as libc::c_uint ||
               offsetOfEndOfSector < (*file).logicalSize {
            cacheFlags =
                (cacheFlags as libc::c_int | 1 as libc::c_int) as uint8_t
        }
        // In contiguous append mode, we'll pre-erase the whole supercluster
        if (*file).mode as libc::c_int & (4 as libc::c_int | 8 as libc::c_int)
               == 4 as libc::c_int | 8 as libc::c_int {
            let mut cursorOffsetInSupercluster: uint32_t =
                (*file).cursorOffset &
                    afatfs_superClusterSize().wrapping_sub(1 as libc::c_int as
                                                               libc::c_uint);
            eraseBlockCount =
                afatfs_fatEntriesPerSector().wrapping_mul(afatfs.sectorsPerCluster).wrapping_sub(cursorOffsetInSupercluster.wrapping_div(512
                                                                                                                                             as
                                                                                                                                             libc::c_int
                                                                                                                                             as
                                                                                                                                             libc::c_uint))
        } else { eraseBlockCount = 0 as libc::c_int as uint32_t }
        status =
            afatfs_cacheSector(physicalSector_0, &mut result, cacheFlags,
                               eraseBlockCount);
        if status as libc::c_uint !=
               AFATFS_OPERATION_SUCCESS as libc::c_int as libc::c_uint {
            // Not enough cache available to accept this write / sector not ready for read
            return 0 as *mut uint8_t
        }
        (*file).writeLockedCacheIndex =
            afatfs_getCacheDescriptorIndexForBuffer(result) as int8_t
    }
    return result;
}
/* *
 * Attempt to seek the file pointer by the offset, relative to the current position.
 *
 * Returns true if the seek was completed, or false if you should try again later by calling this routine again (the
 * cursor is not moved and no seek operation is queued on the file for you).
 *
 * You can only seek forwards by the size of a cluster or less, or backwards to stay within the same cluster. Otherwise
 * false will always be returned (calling this routine again will never make progress on the seek).
 *
 * This amount of seek is special because we will have to wait on at most one read operation, so it's easy to make
 * the seek atomic.
 */
unsafe extern "C" fn afatfs_fseekAtomic(mut file: afatfsFilePtr_t,
                                        mut offset: int32_t) -> bool {
    // Seeks within a sector
    let mut newSectorOffset: uint32_t =
        (offset as
             libc::c_uint).wrapping_add((*file).cursorOffset.wrapping_rem(512
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint));
    // i.e. newSectorOffset is non-negative and smaller than AFATFS_SECTOR_SIZE, we're staying within the same sector
    if newSectorOffset < 512 as libc::c_int as libc::c_uint {
        (*file).cursorOffset =
            ((*file).cursorOffset as
                 libc::c_uint).wrapping_add(offset as libc::c_uint) as
                uint32_t as uint32_t;
        return 1 as libc::c_int != 0
    }
    // We're seeking outside the sector so unlock it if we were holding it
    afatfs_fileUnlockCacheSector(file);
    // FAT16 root directories are made up of contiguous sectors rather than clusters
    if (*file).type_0 as libc::c_uint ==
           AFATFS_FILE_TYPE_FAT16_ROOT_DIRECTORY as libc::c_int as
               libc::c_uint {
        (*file).cursorOffset =
            ((*file).cursorOffset as
                 libc::c_uint).wrapping_add(offset as libc::c_uint) as
                uint32_t as uint32_t;
        return 1 as libc::c_int != 0
    }
    let mut clusterSizeBytes: uint32_t = afatfs_clusterSize();
    let mut offsetInCluster: uint32_t =
        afatfs_byteIndexInCluster((*file).cursorOffset);
    let mut newOffsetInCluster: uint32_t =
        offsetInCluster.wrapping_add(offset as libc::c_uint);
    let mut status: afatfsOperationStatus_e = AFATFS_OPERATION_IN_PROGRESS;
    if offset > clusterSizeBytes as int32_t ||
           offset < -(offsetInCluster as int32_t) {
        return 0 as libc::c_int != 0
    }
    // Are we seeking outside the cluster? If so we'll need to find out the next cluster number
    if newOffsetInCluster >= clusterSizeBytes {
        let mut nextCluster: uint32_t = 0;
        status =
            afatfs_fileGetNextCluster(file, (*file).cursorCluster,
                                      &mut nextCluster);
        if status as libc::c_uint ==
               AFATFS_OPERATION_SUCCESS as libc::c_int as libc::c_uint {
            // Seek to the beginning of the next cluster
            let mut bytesToSeek: uint32_t =
                clusterSizeBytes.wrapping_sub(offsetInCluster);
            (*file).cursorPreviousCluster = (*file).cursorCluster;
            (*file).cursorCluster = nextCluster;
            (*file).cursorOffset =
                ((*file).cursorOffset as
                     libc::c_uint).wrapping_add(bytesToSeek) as uint32_t as
                    uint32_t;
            offset =
                (offset as libc::c_uint).wrapping_sub(bytesToSeek) as int32_t
                    as int32_t
        } else {
            // Try again later
            return 0 as libc::c_int != 0
        }
    }
    // If we didn't already hit the end of the file, add any remaining offset needed inside the cluster
    if !afatfs_isEndOfAllocatedFile(file) {
        (*file).cursorOffset =
            ((*file).cursorOffset as
                 libc::c_uint).wrapping_add(offset as libc::c_uint) as
                uint32_t as uint32_t
    }
    return 1 as libc::c_int != 0;
}
/* *
 * Returns true if the seek was completed, or false if it is still in progress.
 */
unsafe extern "C" fn afatfs_fseekInternalContinue(mut file: *mut afatfsFile_t)
 -> bool {
    let mut opState: *mut afatfsSeek_t = &mut (*file).operation.state.seek;
    let mut clusterSizeBytes: uint32_t = afatfs_clusterSize();
    let mut offsetInCluster: uint32_t =
        afatfs_byteIndexInCluster((*file).cursorOffset);
    let mut status: afatfsOperationStatus_e = AFATFS_OPERATION_IN_PROGRESS;
    // Keep advancing the cursor cluster forwards to consume seekOffset
    while offsetInCluster.wrapping_add((*opState).seekOffset) >=
              clusterSizeBytes && !afatfs_isEndOfAllocatedFile(file) {
        let mut nextCluster: uint32_t = 0;
        status =
            afatfs_fileGetNextCluster(file, (*file).cursorCluster,
                                      &mut nextCluster);
        if status as libc::c_uint ==
               AFATFS_OPERATION_SUCCESS as libc::c_int as libc::c_uint {
            // Seek to the beginning of the next cluster
            let mut bytesToSeek: uint32_t =
                clusterSizeBytes.wrapping_sub(offsetInCluster);
            (*file).cursorPreviousCluster = (*file).cursorCluster;
            (*file).cursorCluster = nextCluster;
            (*file).cursorOffset =
                ((*file).cursorOffset as
                     libc::c_uint).wrapping_add(bytesToSeek) as uint32_t as
                    uint32_t;
            (*opState).seekOffset =
                ((*opState).seekOffset as
                     libc::c_uint).wrapping_sub(bytesToSeek) as uint32_t as
                    uint32_t;
            offsetInCluster = 0 as libc::c_int as uint32_t
        } else {
            // Try again later
            return 0 as libc::c_int != 0
        }
    }
    // If we didn't already hit the end of the file, add any remaining offset needed inside the cluster
    if !afatfs_isEndOfAllocatedFile(file) {
        (*file).cursorOffset =
            ((*file).cursorOffset as
                 libc::c_uint).wrapping_add((*opState).seekOffset) as uint32_t
                as uint32_t
    } // TODO do we need this?
    afatfs_fileUpdateFilesize(file);
    (*file).operation.operation = AFATFS_FILE_OPERATION_NONE;
    if (*opState).callback.is_some() {
        (*opState).callback.expect("non-null function pointer")(file);
    }
    return 1 as libc::c_int != 0;
}
/* *
 * Seek the file pointer forwards by offset bytes. Calls the callback when the seek is complete.
 *
 * Will happily seek beyond the logical end of the file.
 *
 * Returns:
 *     AFATFS_OPERATION_SUCCESS     - The seek was completed immediately
 *     AFATFS_OPERATION_IN_PROGRESS - The seek was queued and will complete later
 *     AFATFS_OPERATION_FAILURE     - The seek could not be queued because the file was busy with another operation,
 *                                    try again later.
 */
unsafe extern "C" fn afatfs_fseekInternal(mut file: afatfsFilePtr_t,
                                          mut offset: uint32_t,
                                          mut callback: afatfsFileCallback_t)
 -> afatfsOperationStatus_e {
    // See if we can seek without queuing an operation
    if afatfs_fseekAtomic(file, offset as int32_t) {
        if callback.is_some() {
            callback.expect("non-null function pointer")(file);
        }
        return AFATFS_OPERATION_SUCCESS
    } else {
        // Our operation must queue
        if afatfs_fileIsBusy(file) { return AFATFS_OPERATION_FAILURE }
        let mut opState: *mut afatfsSeek_t =
            &mut (*file).operation.state.seek;
        (*file).operation.operation = AFATFS_FILE_OPERATION_SEEK;
        (*opState).callback = callback;
        (*opState).seekOffset = offset;
        return AFATFS_OPERATION_IN_PROGRESS
    };
}
/* *
 * Attempt to seek the file cursor from the given point (`whence`) by the given offset, just like C's fseek().
 *
 * AFATFS_SEEK_SET with offset 0 will always return AFATFS_OPERATION_SUCCESS.
 *
 * Returns:
 *     AFATFS_OPERATION_SUCCESS     - The seek was completed immediately
 *     AFATFS_OPERATION_IN_PROGRESS - The seek was queued and will complete later. Feel free to attempt read/write
 *                                    operations on the file, they'll fail until the seek is complete.
 *     AFATFS_OPERATION_FAILURE     - The seek could not be queued because the file was busy with another operation,
 *                                    try again later.
 */
#[no_mangle]
pub unsafe extern "C" fn afatfs_fseek(mut file: afatfsFilePtr_t,
                                      mut offset: int32_t,
                                      mut whence: afatfsSeek_e)
 -> afatfsOperationStatus_e {
    // We need an up-to-date logical filesize so we can clamp seeks to the EOF
    afatfs_fileUpdateFilesize(file);
    match whence as libc::c_uint {
        1 => {
            if offset >= 0 as libc::c_int {
                // Only forwards seeks are supported by this routine:
                return afatfs_fseekInternal(file,
                                            ({
                                                 let mut _a: libc::c_uint =
                                                     (*file).cursorOffset.wrapping_add(offset
                                                                                           as
                                                                                           libc::c_uint);
                                                 let mut _b: uint32_t =
                                                     (*file).logicalSize;
                                                 if _a < _b { _a } else { _b }
                                             }), None)
            }
            // Convert a backwards relative seek into a SEEK_SET. TODO considerable room for improvement if within the same cluster
            offset =
                (offset as libc::c_uint).wrapping_add((*file).cursorOffset) as
                    int32_t as int32_t
        }
        2 => {
            // Are we already at the right position?
            if (*file).logicalSize.wrapping_add(offset as libc::c_uint) ==
                   (*file).cursorOffset {
                return AFATFS_OPERATION_SUCCESS
            }
            // Convert into a SEEK_SET
            offset =
                (offset as libc::c_uint).wrapping_add((*file).logicalSize) as
                    int32_t as int32_t
        }
        0 | _ => { }
    }
    // Now we have a SEEK_SET with a positive offset. Begin by seeking to the start of the file
    afatfs_fileUnlockCacheSector(file);
    (*file).cursorPreviousCluster = 0 as libc::c_int as uint32_t;
    (*file).cursorCluster = (*file).firstCluster;
    (*file).cursorOffset = 0 as libc::c_int as uint32_t;
    // Then seek forwards by the offset
    return afatfs_fseekInternal(file,
                                ({
                                     let mut _a: uint32_t =
                                         offset as uint32_t;
                                     let mut _b: uint32_t =
                                         (*file).logicalSize;
                                     if _a < _b { _a } else { _b }
                                 }), None);
}
/* *
 * Get the byte-offset of the file's cursor from the start of the file.
 *
 * Returns true on success, or false if the file is busy (try again later).
 */
#[no_mangle]
pub unsafe extern "C" fn afatfs_ftell(mut file: afatfsFilePtr_t,
                                      mut position: *mut uint32_t) -> bool {
    if afatfs_fileIsBusy(file) {
        return 0 as libc::c_int != 0
    } else { *position = (*file).cursorOffset; return 1 as libc::c_int != 0 };
}
/* *
 * Attempt to advance the directory pointer `finder` to the next entry in the directory.
 *
 * Returns:
 *     AFATFS_OPERATION_SUCCESS -     A pointer to the next directory entry has been loaded into *dirEntry. If the
 *                                    directory was exhausted then *dirEntry will be set to NULL.
 *     AFATFS_OPERATION_IN_PROGRESS - The disk is busy. The pointer is not advanced, call again later to retry.
 */
#[no_mangle]
pub unsafe extern "C" fn afatfs_findNext(mut directory: afatfsFilePtr_t,
                                         mut finder: *mut afatfsFinder_t,
                                         mut dirEntry:
                                             *mut *mut fatDirectoryEntry_t)
 -> afatfsOperationStatus_e {
    let mut sector: *mut uint8_t = 0 as *mut uint8_t;
    if (*finder).entryIndex as libc::c_ulong ==
           (512 as libc::c_int as
                libc::c_ulong).wrapping_div(::core::mem::size_of::<fatDirectoryEntry_t>()
                                                as
                                                libc::c_ulong).wrapping_sub(1
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_ulong)
       {
        if afatfs_fseekAtomic(directory, 512 as libc::c_int) {
            (*finder).entryIndex = -(1 as libc::c_int) as int16_t
            // Fall through to read the first entry of that new sector
        } else { return AFATFS_OPERATION_IN_PROGRESS }
    }
    sector = afatfs_fileRetainCursorSectorForRead(directory);
    if !sector.is_null() {
        (*finder).entryIndex += 1;
        *dirEntry =
            (sector as
                 *mut fatDirectoryEntry_t).offset((*finder).entryIndex as
                                                      libc::c_int as isize);
        (*finder).sectorNumberPhysical =
            afatfs_fileGetCursorPhysicalSector(directory);
        return AFATFS_OPERATION_SUCCESS
    } else {
        if afatfs_isEndOfAllocatedFile(directory) {
            *dirEntry = 0 as *mut fatDirectoryEntry_t;
            return AFATFS_OPERATION_SUCCESS
        }
        return AFATFS_OPERATION_IN_PROGRESS
    };
}
/* *
 * Release resources associated with a find operation. Calling this more than once is harmless.
 */
#[no_mangle]
pub unsafe extern "C" fn afatfs_findLast(mut directory: afatfsFilePtr_t) {
    afatfs_fileUnlockCacheSector(directory);
}
/* *
 * Initialise the finder so that the first call with the directory to findNext() will return the first file in the
 * directory.
 */
#[no_mangle]
pub unsafe extern "C" fn afatfs_findFirst(mut directory: afatfsFilePtr_t,
                                          mut finder: *mut afatfsFinder_t) {
    afatfs_fseek(directory, 0 as libc::c_int, AFATFS_SEEK_SET);
    (*finder).entryIndex = -(1 as libc::c_int) as int16_t;
}
unsafe extern "C" fn afatfs_extendSubdirectoryContinue(mut directory:
                                                           *mut afatfsFile_t)
 -> afatfsOperationStatus_e {
    let mut opState: *mut afatfsExtendSubdirectory_t =
        &mut (*directory).operation.state.extendSubdirectory;
    let mut status: afatfsOperationStatus_e = AFATFS_OPERATION_IN_PROGRESS;
    let mut sectorBuffer: *mut uint8_t = 0 as *mut uint8_t;
    let mut clusterNumber: uint32_t = 0;
    let mut physicalSector: uint32_t = 0;
    let mut sectorInCluster: uint16_t = 0;
    loop  {
        match (*opState).phase as libc::c_uint {
            0 => {
                status = afatfs_appendRegularFreeClusterContinue(directory);
                if status as libc::c_uint ==
                       AFATFS_OPERATION_SUCCESS as libc::c_int as libc::c_uint
                   {
                    (*opState).phase =
                        AFATFS_EXTEND_SUBDIRECTORY_PHASE_WRITE_SECTORS
                } else {
                    if !(status as libc::c_uint ==
                             AFATFS_OPERATION_FAILURE as libc::c_int as
                                 libc::c_uint) {
                        break ;
                    }
                    (*opState).phase =
                        AFATFS_EXTEND_SUBDIRECTORY_PHASE_FAILURE
                }
            }
            1 => {
                // Now, zero out that cluster
                afatfs_fileGetCursorClusterAndSector(directory,
                                                     &mut clusterNumber,
                                                     &mut sectorInCluster);
                physicalSector =
                    afatfs_fileGetCursorPhysicalSector(directory);
                loop  {
                    status =
                        afatfs_cacheSector(physicalSector, &mut sectorBuffer,
                                           2 as libc::c_int as uint8_t,
                                           0 as libc::c_int as uint32_t);
                    if status as libc::c_uint !=
                           AFATFS_OPERATION_SUCCESS as libc::c_int as
                               libc::c_uint {
                        return status
                    }
                    memset(sectorBuffer as *mut libc::c_void,
                           0 as libc::c_int,
                           512 as libc::c_int as libc::c_ulong);
                    // If this is the first sector of a non-root directory, create the "." and ".." entries
                    if (*directory).directoryEntryPos.sectorNumberPhysical !=
                           0 as libc::c_int as libc::c_uint &&
                           (*directory).cursorOffset ==
                               0 as libc::c_int as libc::c_uint {
                        let mut dirEntries: *mut fatDirectoryEntry_t =
                            sectorBuffer as *mut fatDirectoryEntry_t;
                        memset((*dirEntries.offset(0 as libc::c_int as
                                                       isize)).filename.as_mut_ptr()
                                   as *mut libc::c_void, ' ' as i32,
                               ::core::mem::size_of::<[libc::c_char; 11]>() as
                                   libc::c_ulong);
                        (*dirEntries.offset(0 as libc::c_int as
                                                isize)).filename[0 as
                                                                     libc::c_int
                                                                     as usize]
                            = '.' as i32 as libc::c_char;
                        (*dirEntries.offset(0 as libc::c_int as
                                                isize)).firstClusterHigh =
                            ((*directory).firstCluster >> 16 as libc::c_int)
                                as uint16_t;
                        (*dirEntries.offset(0 as libc::c_int as
                                                isize)).firstClusterLow =
                            ((*directory).firstCluster &
                                 0xffff as libc::c_int as libc::c_uint) as
                                uint16_t;
                        (*dirEntries.offset(0 as libc::c_int as isize)).attrib
                            = 0x10 as libc::c_int as uint8_t;
                        memset((*dirEntries.offset(1 as libc::c_int as
                                                       isize)).filename.as_mut_ptr()
                                   as *mut libc::c_void, ' ' as i32,
                               ::core::mem::size_of::<[libc::c_char; 11]>() as
                                   libc::c_ulong);
                        (*dirEntries.offset(1 as libc::c_int as
                                                isize)).filename[0 as
                                                                     libc::c_int
                                                                     as usize]
                            = '.' as i32 as libc::c_char;
                        (*dirEntries.offset(1 as libc::c_int as
                                                isize)).filename[1 as
                                                                     libc::c_int
                                                                     as usize]
                            = '.' as i32 as libc::c_char;
                        (*dirEntries.offset(1 as libc::c_int as
                                                isize)).firstClusterHigh =
                            ((*opState).parentDirectoryCluster >>
                                 16 as libc::c_int) as uint16_t;
                        (*dirEntries.offset(1 as libc::c_int as
                                                isize)).firstClusterLow =
                            ((*opState).parentDirectoryCluster &
                                 0xffff as libc::c_int as libc::c_uint) as
                                uint16_t;
                        (*dirEntries.offset(1 as libc::c_int as isize)).attrib
                            = 0x10 as libc::c_int as uint8_t
                    }
                    if !((sectorInCluster as libc::c_uint) <
                             afatfs.sectorsPerCluster.wrapping_sub(1 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_uint))
                       {
                        break ;
                    }
                    // Move to next sector
                    afatfs_assert(afatfs_fseekAtomic(directory,
                                                     512 as libc::c_int));
                    sectorInCluster = sectorInCluster.wrapping_add(1);
                    physicalSector = physicalSector.wrapping_add(1)
                }
                // Seek back to the beginning of the cluster
                afatfs_assert(afatfs_fseekAtomic(directory,
                                                 (-(512 as libc::c_int) as
                                                      libc::c_uint).wrapping_mul(afatfs.sectorsPerCluster.wrapping_sub(1
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           libc::c_uint))
                                                     as int32_t));
                (*opState).phase = AFATFS_EXTEND_SUBDIRECTORY_PHASE_SUCCESS
            }
            2 => {
                (*directory).operation.operation = AFATFS_FILE_OPERATION_NONE;
                if (*opState).callback.is_some() {
                    (*opState).callback.expect("non-null function pointer")(directory);
                }
                return AFATFS_OPERATION_SUCCESS
            }
            3 => {
                (*directory).operation.operation = AFATFS_FILE_OPERATION_NONE;
                if (*opState).callback.is_some() {
                    (*opState).callback.expect("non-null function pointer")(0
                                                                                as
                                                                                afatfsFilePtr_t);
                }
                return AFATFS_OPERATION_FAILURE
            }
            _ => { break ; }
        }
    }
    return AFATFS_OPERATION_IN_PROGRESS;
}
/* *
 * Queue an operation to add a cluster to a sub-directory.
 *
 * Tthe new cluster is zero-filled. "." and ".." entries are added if it is the first cluster of a new subdirectory.
 *
 * The directory must not be busy, otherwise AFATFS_OPERATION_FAILURE is returned immediately.
 *
 * The directory's cursor must lie at the end of the directory file (i.e. isEndOfAllocatedFile() would return true).
 *
 * You must provide parentDirectory if this is the first extension to the subdirectory, otherwise pass NULL for that argument.
 */
unsafe extern "C" fn afatfs_extendSubdirectory(mut directory:
                                                   *mut afatfsFile_t,
                                               mut parentDirectory:
                                                   afatfsFilePtr_t,
                                               mut callback:
                                                   afatfsFileCallback_t)
 -> afatfsOperationStatus_e {
    // FAT16 root directories cannot be extended
    if (*directory).type_0 as libc::c_uint ==
           AFATFS_FILE_TYPE_FAT16_ROOT_DIRECTORY as libc::c_int as
               libc::c_uint ||
           afatfs_fileIsBusy(directory) as libc::c_int != 0 {
        return AFATFS_OPERATION_FAILURE
    }
    /*
     * We'll assume that we're never asked to append the first cluster of a root directory, since any
     * reasonably-formatted volume should have a root!
     */
    let mut opState: *mut afatfsExtendSubdirectory_t =
        &mut (*directory).operation.state.extendSubdirectory;
    (*directory).operation.operation =
        AFATFS_FILE_OPERATION_EXTEND_SUBDIRECTORY;
    (*opState).phase = AFATFS_EXTEND_SUBDIRECTORY_PHASE_INITIAL;
    (*opState).parentDirectoryCluster =
        if !parentDirectory.is_null() {
            (*parentDirectory).firstCluster
        } else { 0 as libc::c_int as libc::c_uint };
    (*opState).callback = callback;
    afatfs_appendRegularFreeClusterInitOperationState(&mut (*opState).appendFreeCluster,
                                                      (*directory).cursorPreviousCluster);
    return afatfs_extendSubdirectoryContinue(directory);
}
/* *
 * Allocate space for a new directory entry to be written, store the position of that entry in the finder, and set
 * the *dirEntry pointer to point to the entry within the cached FAT sector. This pointer's lifetime is only as good
 * as the life of the cache, so don't dawdle.
 *
 * Before the first call to this function, call afatfs_findFirst() on the directory.
 *
 * The directory sector in the cache is marked as dirty, so any changes written through to the entry will be flushed out
 * in a subsequent poll cycle.
 *
 * Returns:
 *     AFATFS_OPERATION_IN_PROGRESS - Call again later to continue
 *     AFATFS_OPERATION_SUCCESS     - Entry has been inserted and *dirEntry and *finder have been updated
 *     AFATFS_OPERATION_FAILURE     - When the directory is full.
 */
unsafe extern "C" fn afatfs_allocateDirectoryEntry(mut directory:
                                                       afatfsFilePtr_t,
                                                   mut dirEntry:
                                                       *mut *mut fatDirectoryEntry_t,
                                                   mut finder:
                                                       *mut afatfsFinder_t)
 -> afatfsOperationStatus_e {
    let mut result: afatfsOperationStatus_e = AFATFS_OPERATION_IN_PROGRESS;
    if afatfs_fileIsBusy(directory) { return AFATFS_OPERATION_IN_PROGRESS }
    loop 
         // Continue the search in the newly-extended directory
         {
        result = afatfs_findNext(directory, finder, dirEntry);
        if !(result as libc::c_uint ==
                 AFATFS_OPERATION_SUCCESS as libc::c_int as libc::c_uint) {
            break ;
        }
        if !(*dirEntry).is_null() {
            if fat_isDirectoryEntryEmpty(*dirEntry) as libc::c_int != 0 ||
                   fat_isDirectoryEntryTerminator(*dirEntry) as libc::c_int !=
                       0 {
                afatfs_cacheSectorMarkDirty(afatfs_getCacheDescriptorForBuffer(*dirEntry
                                                                                   as
                                                                                   *mut uint8_t));
                afatfs_findLast(directory);
                return AFATFS_OPERATION_SUCCESS
            }
        } else {
            // Need to extend directory size by adding a cluster
            result =
                afatfs_extendSubdirectory(directory, 0 as afatfsFilePtr_t,
                                          None);
            if !(result as libc::c_uint ==
                     AFATFS_OPERATION_SUCCESS as libc::c_int as libc::c_uint)
               {
                break ;
            }
        }
    }
    return result;
}
/* *
 * Return a pointer to a free entry in the open files table (a file whose type is "NONE"). You should initialize
 * the file afterwards with afatfs_initFileHandle().
 */
unsafe extern "C" fn afatfs_allocateFileHandle() -> afatfsFilePtr_t {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 3 as libc::c_int {
        if afatfs.openFiles[i as usize].type_0 as libc::c_uint ==
               AFATFS_FILE_TYPE_NONE as libc::c_int as libc::c_uint {
            return &mut *afatfs.openFiles.as_mut_ptr().offset(i as isize) as
                       *mut afatfsFile_t
        }
        i += 1
    }
    return 0 as afatfsFilePtr_t;
}
/* *
 * Continue the file truncation.
 *
 * When truncation finally succeeds or fails, the current operation is cleared on the file (if the current operation
 * was a truncate), then the truncate operation's callback is called. This allows the truncation to be called as a
 * sub-operation without it clearing the parent file operation.
 */
unsafe extern "C" fn afatfs_ftruncateContinue(mut file: afatfsFilePtr_t,
                                              mut markDeleted: bool)
 -> afatfsOperationStatus_e {
    let mut opState: *mut afatfsTruncateFile_t =
        &mut (*file).operation.state.truncateFile;
    let mut status: afatfsOperationStatus_e = AFATFS_OPERATION_FAILURE;
    let mut oldFreeFileStart: uint32_t = 0;
    let mut freeFileGrow: uint32_t = 0;
    loop  {
        match (*opState).phase as libc::c_uint {
            0 => {
                status =
                    afatfs_saveDirectoryEntry(file,
                                              if markDeleted as libc::c_int !=
                                                     0 {
                                                  AFATFS_SAVE_DIRECTORY_DELETED
                                                      as libc::c_int
                                              } else {
                                                  AFATFS_SAVE_DIRECTORY_NORMAL
                                                      as libc::c_int
                                              } as
                                                  afatfsSaveDirectoryEntryMode_e);
                if !(status as libc::c_uint ==
                         AFATFS_OPERATION_SUCCESS as libc::c_int as
                             libc::c_uint) {
                    break ;
                }
                if (*opState).endCluster != 0 {
                    (*opState).phase =
                        AFATFS_TRUNCATE_FILE_ERASE_FAT_CHAIN_CONTIGUOUS
                } else {
                    (*opState).phase =
                        AFATFS_TRUNCATE_FILE_ERASE_FAT_CHAIN_NORMAL
                }
            }
            2 => {
                // Prepare the clusters to be added back on to the beginning of the freefile
                status =
                    afatfs_FATFillWithPattern(AFATFS_FAT_PATTERN_UNTERMINATED_CHAIN,
                                              &mut (*opState).currentCluster,
                                              (*opState).endCluster);
                if !(status as libc::c_uint ==
                         AFATFS_OPERATION_SUCCESS as libc::c_int as
                             libc::c_uint) {
                    break ;
                }
                (*opState).phase = AFATFS_TRUNCATE_FILE_PREPEND_TO_FREEFILE
            }
            3 => {
                // Note, it's okay to run this code several times:
                oldFreeFileStart = afatfs.freeFile.firstCluster;
                afatfs.freeFile.firstCluster = (*opState).startCluster;
                freeFileGrow =
                    oldFreeFileStart.wrapping_sub((*opState).startCluster).wrapping_mul(afatfs_clusterSize());
                afatfs.freeFile.logicalSize =
                    (afatfs.freeFile.logicalSize as
                         libc::c_uint).wrapping_add(freeFileGrow) as uint32_t
                        as uint32_t;
                afatfs.freeFile.physicalSize =
                    (afatfs.freeFile.physicalSize as
                         libc::c_uint).wrapping_add(freeFileGrow) as uint32_t
                        as uint32_t;
                status =
                    afatfs_saveDirectoryEntry(&mut afatfs.freeFile,
                                              AFATFS_SAVE_DIRECTORY_NORMAL);
                if !(status as libc::c_uint ==
                         AFATFS_OPERATION_SUCCESS as libc::c_int as
                             libc::c_uint) {
                    break ;
                }
                (*opState).phase = AFATFS_TRUNCATE_FILE_SUCCESS
            }
            1 => {
                while !afatfs_FATIsEndOfChainMarker((*opState).currentCluster)
                      {
                    let mut nextCluster: uint32_t = 0;
                    status =
                        afatfs_FATGetNextCluster(0 as libc::c_int,
                                                 (*opState).currentCluster,
                                                 &mut nextCluster);
                    if status as libc::c_uint !=
                           AFATFS_OPERATION_SUCCESS as libc::c_int as
                               libc::c_uint {
                        return status
                    }
                    status =
                        afatfs_FATSetNextCluster((*opState).currentCluster,
                                                 0 as libc::c_int as
                                                     uint32_t);
                    if status as libc::c_uint !=
                           AFATFS_OPERATION_SUCCESS as libc::c_int as
                               libc::c_uint {
                        return status
                    }
                    (*opState).currentCluster = nextCluster;
                    // Searches for unallocated regular clusters should be told about this free cluster now
                    afatfs.lastClusterAllocated =
                        ({
                             let mut _a: uint32_t =
                                 afatfs.lastClusterAllocated;
                             let mut _b: libc::c_uint =
                                 (*opState).currentCluster.wrapping_sub(1 as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint);
                             if _a < _b { _a } else { _b }
                         })
                }
                (*opState).phase = AFATFS_TRUNCATE_FILE_SUCCESS
            }
            4 => {
                if (*file).operation.operation as libc::c_uint ==
                       AFATFS_FILE_OPERATION_TRUNCATE as libc::c_int as
                           libc::c_uint {
                    (*file).operation.operation = AFATFS_FILE_OPERATION_NONE
                }
                if (*opState).callback.is_some() {
                    (*opState).callback.expect("non-null function pointer")(file);
                }
                return AFATFS_OPERATION_SUCCESS
            }
            _ => { break ; }
        }
    }
    if status as libc::c_uint ==
           AFATFS_OPERATION_FAILURE as libc::c_int as libc::c_uint &&
           (*file).operation.operation as libc::c_uint ==
               AFATFS_FILE_OPERATION_TRUNCATE as libc::c_int as libc::c_uint {
        (*file).operation.operation = AFATFS_FILE_OPERATION_NONE
    }
    return status;
}
/* *
 * Queue an operation to truncate the file to zero bytes in length.
 *
 * Returns true if the operation was successfully queued or false if the file is busy (try again later).
 *
 * The callback is called once the file has been truncated (some time after this routine returns).
 */
#[no_mangle]
pub unsafe extern "C" fn afatfs_ftruncate(mut file: afatfsFilePtr_t,
                                          mut callback: afatfsFileCallback_t)
 -> bool {
    let mut opState: *mut afatfsTruncateFile_t =
        0 as *mut afatfsTruncateFile_t;
    if afatfs_fileIsBusy(file) { return 0 as libc::c_int != 0 }
    (*file).operation.operation = AFATFS_FILE_OPERATION_TRUNCATE;
    opState = &mut (*file).operation.state.truncateFile;
    (*opState).callback = callback;
    (*opState).phase = AFATFS_TRUNCATE_FILE_INITIAL;
    (*opState).startCluster = (*file).firstCluster;
    (*opState).currentCluster = (*opState).startCluster;
    if (*file).mode as libc::c_int & 8 as libc::c_int != 0 as libc::c_int {
        // The file is contiguous and ends where the freefile begins
        (*opState).endCluster = afatfs.freeFile.firstCluster
    } else {
        // The range of clusters to delete is not contiguous, so follow it as a linked-list instead
        (*opState).endCluster = 0 as libc::c_int as uint32_t
    }
    // We'll drop the cluster chain from the directory entry immediately
    (*file).firstCluster = 0 as libc::c_int as uint32_t;
    (*file).logicalSize = 0 as libc::c_int as uint32_t;
    (*file).physicalSize = 0 as libc::c_int as uint32_t;
    afatfs_fseek(file, 0 as libc::c_int, AFATFS_SEEK_SET);
    return 1 as libc::c_int != 0;
}
/* *
 * Load details from the given FAT directory entry into the file.
 */
unsafe extern "C" fn afatfs_fileLoadDirectoryEntry(mut file:
                                                       *mut afatfsFile_t,
                                                   mut entry:
                                                       *mut fatDirectoryEntry_t) {
    (*file).firstCluster =
        (((*entry).firstClusterHigh as libc::c_int) << 16 as libc::c_int) as
            uint32_t | (*entry).firstClusterLow as libc::c_uint;
    (*file).logicalSize = (*entry).fileSize;
    (*file).physicalSize = roundUpTo((*entry).fileSize, afatfs_clusterSize());
    (*file).attrib = (*entry).attrib;
}
unsafe extern "C" fn afatfs_createFileContinue(mut file: *mut afatfsFile_t) {
    let mut current_block: u64;
    let mut opState: *mut afatfsCreateFile_t =
        &mut (*file).operation.state.createFile;
    let mut entry: *mut fatDirectoryEntry_t = 0 as *mut fatDirectoryEntry_t;
    let mut status: afatfsOperationStatus_e = AFATFS_OPERATION_IN_PROGRESS;
    'c_32986:
        loop  {
            match (*opState).phase as libc::c_int {
                0 => {
                    afatfs_findFirst(&mut afatfs.currentDirectory,
                                     &mut (*file).directoryEntryPos);
                    (*opState).phase =
                        AFATFS_CREATEFILE_PHASE_FIND_FILE as libc::c_int as
                            uint8_t
                }
                1 => {
                    loop  {
                        status =
                            afatfs_findNext(&mut afatfs.currentDirectory,
                                            &mut (*file).directoryEntryPos,
                                            &mut entry);
                        match status as libc::c_uint {
                            1 => {
                                // Is this the last entry in the directory?
                                if entry.is_null() ||
                                       fat_isDirectoryEntryTerminator(entry)
                                           as libc::c_int != 0 {
                                    afatfs_findLast(&mut afatfs.currentDirectory);
                                    if (*file).mode as libc::c_int &
                                           16 as libc::c_int !=
                                           0 as libc::c_int {
                                        // The file didn't already exist, so we can create it. Allocate a new directory entry
                                        afatfs_findFirst(&mut afatfs.currentDirectory,
                                                         &mut (*file).directoryEntryPos);
                                        (*opState).phase =
                                            AFATFS_CREATEFILE_PHASE_CREATE_NEW_FILE
                                                as libc::c_int as uint8_t;
                                        break ;
                                    } else {
                                        // File not found.
                                        (*opState).phase =
                                            AFATFS_CREATEFILE_PHASE_FAILURE as
                                                libc::c_int as uint8_t;
                                        break ;
                                    }
                                } else if strncmp((*entry).filename.as_mut_ptr(),
                                                  (*opState).filename.as_mut_ptr()
                                                      as *mut libc::c_char,
                                                  11 as libc::c_int as
                                                      libc::c_ulong) ==
                                              0 as libc::c_int {
                                    // We found a file with this name!
                                    afatfs_fileLoadDirectoryEntry(file,
                                                                  entry);
                                    afatfs_findLast(&mut afatfs.currentDirectory);
                                    (*opState).phase =
                                        AFATFS_CREATEFILE_PHASE_SUCCESS as
                                            libc::c_int as uint8_t;
                                    break ;
                                }
                            }
                            2 => {
                                afatfs_findLast(&mut afatfs.currentDirectory);
                                (*opState).phase =
                                    AFATFS_CREATEFILE_PHASE_FAILURE as
                                        libc::c_int as uint8_t;
                                break ;
                            }
                            0 | _ => { }
                        }
                        if !(status as libc::c_uint ==
                                 AFATFS_OPERATION_SUCCESS as libc::c_int as
                                     libc::c_uint) {
                            current_block = 14298507163138330979;
                            break 'c_32986 ;
                        }
                    }
                }
                2 => {
                    status =
                        afatfs_allocateDirectoryEntry(&mut afatfs.currentDirectory,
                                                      &mut entry,
                                                      &mut (*file).directoryEntryPos);
                    if status as libc::c_uint ==
                           AFATFS_OPERATION_SUCCESS as libc::c_int as
                               libc::c_uint {
                        memset(entry as *mut libc::c_void, 0 as libc::c_int,
                               ::core::mem::size_of::<fatDirectoryEntry_t>()
                                   as libc::c_ulong);
                        memcpy((*entry).filename.as_mut_ptr() as
                                   *mut libc::c_void,
                               (*opState).filename.as_mut_ptr() as
                                   *const libc::c_void,
                               11 as libc::c_int as libc::c_ulong);
                        (*entry).attrib = (*file).attrib;
                        let mut fileDate: uint16_t =
                            (0o1 as libc::c_int |
                                 (12 as libc::c_int) << 5 as libc::c_int |
                                 (2015 as libc::c_int - 1980 as libc::c_int)
                                     << 9 as libc::c_int) as uint16_t;
                        let mut fileTime: uint16_t =
                            (0 as libc::c_int / 2 as libc::c_int |
                                 (0 as libc::c_int) << 5 as libc::c_int |
                                 (0 as libc::c_int) << 11 as libc::c_int) as
                                uint16_t;
                        // rtcGetDateTime will fill dt with 0000-01-01T00:00:00
                // when time is not known.
                        let mut dt: dateTime_t =
                            dateTime_t{year: 0,
                                       month: 0,
                                       day: 0,
                                       hours: 0,
                                       minutes: 0,
                                       seconds: 0,
                                       millis: 0,};
                        rtcGetDateTime(&mut dt);
                        if dt.year as libc::c_int != 0 as libc::c_int {
                            fileDate =
                                (dt.day as libc::c_int |
                                     (dt.month as libc::c_int) <<
                                         5 as libc::c_int |
                                     (dt.year as libc::c_int -
                                          1980 as libc::c_int) <<
                                         9 as libc::c_int) as uint16_t;
                            fileTime =
                                (dt.seconds as libc::c_int / 2 as libc::c_int
                                     |
                                     (dt.minutes as libc::c_int) <<
                                         5 as libc::c_int |
                                     (dt.hours as libc::c_int) <<
                                         11 as libc::c_int) as uint16_t
                        }
                        (*entry).creationDate = fileDate;
                        (*entry).creationTime = fileTime;
                        (*entry).lastWriteDate = fileDate;
                        (*entry).lastWriteTime = fileTime;
                        (*opState).phase =
                            AFATFS_CREATEFILE_PHASE_SUCCESS as libc::c_int as
                                uint8_t
                    } else {
                        if !(status as libc::c_uint ==
                                 AFATFS_OPERATION_FAILURE as libc::c_int as
                                     libc::c_uint) {
                            current_block = 14298507163138330979;
                            break ;
                        }
                        (*opState).phase =
                            AFATFS_CREATEFILE_PHASE_FAILURE as libc::c_int as
                                uint8_t
                    }
                }
                3 => {
                    if (*file).mode as libc::c_int & 32 as libc::c_int !=
                           0 as libc::c_int {
                        current_block = 2122094917359643297;
                        break ;
                    } else { current_block = 5891011138178424807; break ; }
                }
                4 => {
                    (*file).type_0 = AFATFS_FILE_TYPE_NONE;
                    (*opState).callback.expect("non-null function pointer")(0
                                                                                as
                                                                                afatfsFilePtr_t);
                    current_block = 14298507163138330979;
                    break ;
                }
                _ => { current_block = 14298507163138330979; break ; }
            }
        }
    match current_block {
        2122094917359643297 =>
        /*
                 * For this high performance file type, we require the directory entry for the file to be retained
                 * in the cache at all times.
                 */
        {
            let mut directorySector: *mut uint8_t = 0 as *mut uint8_t;
            status =
                afatfs_cacheSector((*file).directoryEntryPos.sectorNumberPhysical,
                                   &mut directorySector,
                                   (1 as libc::c_int | 16 as libc::c_int) as
                                       uint8_t, 0 as libc::c_int as uint32_t);
            if status as libc::c_uint !=
                   AFATFS_OPERATION_SUCCESS as libc::c_int as libc::c_uint {
                // Retry next time
                current_block = 14298507163138330979;
            } else { current_block = 5891011138178424807; }
        }
        _ => { }
    }
    match current_block {
        5891011138178424807 => {
            afatfs_fseek(file, 0 as libc::c_int, AFATFS_SEEK_SET);
            // Is file empty?
            if (*file).cursorCluster == 0 as libc::c_int as libc::c_uint {
                if (*file).mode as libc::c_int & 8 as libc::c_int !=
                       0 as libc::c_int {
                    if afatfs_fileIsBusy(&mut afatfs.freeFile) {
                        current_block = 14298507163138330979;
                    } else {
                        // Lock the freefile for our exclusive access
                        afatfs.freeFile.operation.operation =
                            AFATFS_FILE_OPERATION_LOCKED;
                        current_block = 1874315696050160458;
                    }
                } else { current_block = 1874315696050160458; }
            } else {
                // We can't guarantee that the existing file contents are contiguous
                (*file).mode =
                    ((*file).mode as libc::c_int & !(8 as libc::c_int)) as
                        uint8_t;
                // Seek to the end of the file if it is in append mode
                if (*file).mode as libc::c_int & 4 as libc::c_int !=
                       0 as libc::c_int {
                    // This replaces our open file operation
                    (*file).operation.operation = AFATFS_FILE_OPERATION_NONE;
                    afatfs_fseekInternal(file, (*file).logicalSize,
                                         (*opState).callback);
                    current_block = 14298507163138330979;
                } else if (*file).mode as libc::c_int ==
                              16 as libc::c_int | 2 as libc::c_int {
                    // If we're only writing (not reading) the file must be truncated
                    // This replaces our open file operation
                    (*file).operation.operation = AFATFS_FILE_OPERATION_NONE;
                    afatfs_ftruncate(file, (*opState).callback);
                    current_block = 14298507163138330979;
                } else { current_block = 1874315696050160458; }
            }
            match current_block {
                14298507163138330979 => { }
                _ => {
                    (*file).operation.operation = AFATFS_FILE_OPERATION_NONE;
                    (*opState).callback.expect("non-null function pointer")(file);
                }
            }
        }
        _ => { }
    };
}
/* *
 * Reset the in-memory data for the given handle back to the zeroed initial state
 */
unsafe extern "C" fn afatfs_initFileHandle(mut file: afatfsFilePtr_t) {
    memset(file as *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<afatfsFile_t>() as libc::c_ulong);
    (*file).writeLockedCacheIndex = -(1 as libc::c_int) as int8_t;
    (*file).readRetainCacheIndex = -(1 as libc::c_int) as int8_t;
}
unsafe extern "C" fn afatfs_funlinkContinue(mut file: afatfsFilePtr_t) {
    let mut opState: *mut afatfsUnlinkFile_t =
        &mut (*file).operation.state.unlinkFile;
    let mut status: afatfsOperationStatus_e = AFATFS_OPERATION_IN_PROGRESS;
    status = afatfs_ftruncateContinue(file, 1 as libc::c_int != 0);
    if status as libc::c_uint ==
           AFATFS_OPERATION_SUCCESS as libc::c_int as libc::c_uint {
        // Once the truncation is completed, we can close the file handle
        (*file).operation.operation = AFATFS_FILE_OPERATION_NONE;
        afatfs_fclose(file, (*opState).callback);
    };
}
/* *
 * Delete and close the file.
 *
 * Returns true if the operation was successfully queued (callback will be called some time after this routine returns)
 * or false if the file is busy and you should try again later.
 */
#[no_mangle]
pub unsafe extern "C" fn afatfs_funlink(mut file: afatfsFilePtr_t,
                                        mut callback: afatfsCallback_t)
 -> bool {
    let mut opState: *mut afatfsUnlinkFile_t =
        &mut (*file).operation.state.unlinkFile;
    if file.is_null() ||
           (*file).type_0 as libc::c_uint ==
               AFATFS_FILE_TYPE_NONE as libc::c_int as libc::c_uint {
        return 1 as libc::c_int != 0
    }
    /*
     * Internally an unlink is implemented by first doing a ftruncate(), marking the directory entry as deleted,
     * then doing a fclose() operation.
     */
    // Start the sub-operation of truncating the file
    if !afatfs_ftruncate(file, None) { return 0 as libc::c_int != 0 }
    /*
     * The unlink operation has its own private callback field so that the truncate suboperation doesn't end up
     * calling back early when it completes:
     */
    (*opState).callback = callback;
    (*file).operation.operation = AFATFS_FILE_OPERATION_UNLINK;
    return 1 as libc::c_int != 0;
}
/* *
 * Open (or create) a file in the CWD with the given filename.
 *
 * file             - Memory location to store the newly opened file details
 * name             - Filename in "name.ext" format. No path.
 * attrib           - FAT file attributes to give the file (if created)
 * fileMode         - Bitset of AFATFS_FILE_MODE_* constants. Include AFATFS_FILE_MODE_CREATE to create the file if
 *                    it does not exist.
 * callback         - Called when the operation is complete
 */
unsafe extern "C" fn afatfs_createFile(mut file: afatfsFilePtr_t,
                                       mut name: *const libc::c_char,
                                       mut attrib: uint8_t,
                                       mut fileMode: uint8_t,
                                       mut callback: afatfsFileCallback_t)
 -> afatfsFilePtr_t {
    let mut opState: *mut afatfsCreateFile_t =
        &mut (*file).operation.state.createFile;
    afatfs_initFileHandle(file);
    // Queue the operation to finish the file creation
    (*file).operation.operation = AFATFS_FILE_OPERATION_CREATE_FILE;
    (*file).mode = fileMode;
    if strcmp(name, b".\x00" as *const u8 as *const libc::c_char) ==
           0 as libc::c_int {
        (*file).firstCluster = afatfs.currentDirectory.firstCluster;
        (*file).physicalSize = afatfs.currentDirectory.physicalSize;
        (*file).logicalSize = afatfs.currentDirectory.logicalSize;
        (*file).attrib = afatfs.currentDirectory.attrib;
        (*file).type_0 = afatfs.currentDirectory.type_0
    } else {
        fat_convertFilenameToFATStyle(name, (*opState).filename.as_mut_ptr());
        (*file).attrib = attrib;
        if attrib as libc::c_int & 0x10 as libc::c_int != 0 as libc::c_int {
            (*file).type_0 = AFATFS_FILE_TYPE_DIRECTORY
        } else { (*file).type_0 = AFATFS_FILE_TYPE_NORMAL }
    }
    (*opState).callback = callback;
    if strcmp(name, b".\x00" as *const u8 as *const libc::c_char) ==
           0 as libc::c_int {
        // Since we already have the directory entry details, we can skip straight to the final operations requried
        (*opState).phase =
            AFATFS_CREATEFILE_PHASE_SUCCESS as libc::c_int as uint8_t
    } else {
        (*opState).phase =
            AFATFS_CREATEFILE_PHASE_INITIAL as libc::c_int as uint8_t
    }
    afatfs_createFileContinue(file);
    return file;
}
unsafe extern "C" fn afatfs_fcloseContinue(mut file: afatfsFilePtr_t) {
    let mut descriptor: *mut afatfsCacheBlockDescriptor_t =
        0 as *mut afatfsCacheBlockDescriptor_t;
    let mut opState: *mut afatfsCloseFile_t =
        &mut (*file).operation.state.closeFile;
    /*
     * Directories don't update their parent directory entries over time, because their fileSize field in the directory
     * never changes (when we add the first cluster to the directory we save the directory entry at that point and it
     * doesn't change afterwards). So don't bother trying to save their directory entries during fclose().
     *
     * Also if we only opened the file for read then we didn't change the directory entry either.
     */
    if (*file).type_0 as libc::c_uint !=
           AFATFS_FILE_TYPE_DIRECTORY as libc::c_int as libc::c_uint &&
           (*file).type_0 as libc::c_uint !=
               AFATFS_FILE_TYPE_FAT16_ROOT_DIRECTORY as libc::c_int as
                   libc::c_uint &&
           (*file).mode as libc::c_int & (4 as libc::c_int | 2 as libc::c_int)
               != 0 as libc::c_int {
        if afatfs_saveDirectoryEntry(file, AFATFS_SAVE_DIRECTORY_FOR_CLOSE) as
               libc::c_uint !=
               AFATFS_OPERATION_SUCCESS as libc::c_int as libc::c_uint {
            return
        }
    }
    // Release our reservation on the directory cache if needed
    if (*file).mode as libc::c_int & 32 as libc::c_int != 0 as libc::c_int {
        descriptor =
            afatfs_findCacheSector((*file).directoryEntryPos.sectorNumberPhysical);
        if !descriptor.is_null() {
            (*descriptor).set_retainCount(({
                                               let mut _a: libc::c_int =
                                                   (*descriptor).retainCount()
                                                       as libc::c_int -
                                                       1 as libc::c_int;
                                               let mut _b: libc::c_int =
                                                   0 as libc::c_int;
                                               if _a > _b { _a } else { _b }
                                           }) as libc::c_uint)
        }
    }
    // Release locks on the sector at the file cursor position
    afatfs_fileUnlockCacheSector(file);
    // Release our exclusive lock on the freefile if needed
    if (*file).mode as libc::c_int & 8 as libc::c_int != 0 as libc::c_int {
        afatfs_assert(afatfs.freeFile.operation.operation as libc::c_uint ==
                          AFATFS_FILE_OPERATION_LOCKED as libc::c_int as
                              libc::c_uint);
        afatfs.freeFile.operation.operation = AFATFS_FILE_OPERATION_NONE
    }
    (*file).type_0 = AFATFS_FILE_TYPE_NONE;
    (*file).operation.operation = AFATFS_FILE_OPERATION_NONE;
    if (*opState).callback.is_some() {
        (*opState).callback.expect("non-null function pointer")();
    };
}
/* *
 * Returns true if an operation was successfully queued to close the file and destroy the file handle. If the file is
 * currently busy, false is returned and you should retry later.
 *
 * If provided, the callback will be called after the operation completes (pass NULL for no callback).
 *
 * If this function returns true, you should not make any further calls to the file (as the handle might be reused for a
 * new file).
 */
#[no_mangle]
pub unsafe extern "C" fn afatfs_fclose(mut file: afatfsFilePtr_t,
                                       mut callback: afatfsCallback_t)
 -> bool {
    if file.is_null() ||
           (*file).type_0 as libc::c_uint ==
               AFATFS_FILE_TYPE_NONE as libc::c_int as libc::c_uint {
        return 1 as libc::c_int != 0
    } else if afatfs_fileIsBusy(file) {
        return 0 as libc::c_int != 0
    } else {
        afatfs_fileUpdateFilesize(file);
        (*file).operation.operation = AFATFS_FILE_OPERATION_CLOSE;
        (*file).operation.state.closeFile.callback = callback;
        afatfs_fcloseContinue(file);
        return 1 as libc::c_int != 0
    };
}
/* *
 * Create a new directory with the given name, or open the directory if it already exists.
 *
 * The directory will be passed to the callback, or NULL if the creation failed.
 *
 * Returns true if the directory creation was begun, or false if there are too many open files.
 */
#[no_mangle]
pub unsafe extern "C" fn afatfs_mkdir(mut filename: *const libc::c_char,
                                      mut callback: afatfsFileCallback_t)
 -> bool {
    let mut file: afatfsFilePtr_t = afatfs_allocateFileHandle();
    if !file.is_null() {
        afatfs_createFile(file, filename, 0x10 as libc::c_int as uint8_t,
                          (16 as libc::c_int | 1 as libc::c_int |
                               2 as libc::c_int) as uint8_t, callback);
    } else if callback.is_some() {
        callback.expect("non-null function pointer")(0 as afatfsFilePtr_t);
    }
    return !file.is_null();
}
/* *
 * Change the working directory to the directory with the given handle (use fopen). Pass NULL for `directory` in order to
 * change to the root directory.
 *
 * Returns true on success, false if you should call again later to retry. After changing into a directory, your handle
 * to that directory may be closed by fclose().
 */
#[no_mangle]
pub unsafe extern "C" fn afatfs_chdir(mut directory: afatfsFilePtr_t)
 -> bool {
    if afatfs_fileIsBusy(&mut afatfs.currentDirectory) {
        return 0 as libc::c_int != 0
    }
    if !directory.is_null() {
        if afatfs_fileIsBusy(directory) { return 0 as libc::c_int != 0 }
        memcpy(&mut afatfs.currentDirectory as *mut afatfsFile_t as
                   *mut libc::c_void, directory as *const libc::c_void,
               ::core::mem::size_of::<afatfsFile_t>() as libc::c_ulong);
        return 1 as libc::c_int != 0
    } else {
        afatfs_initFileHandle(&mut afatfs.currentDirectory);
        afatfs.currentDirectory.mode =
            (1 as libc::c_int | 2 as libc::c_int) as uint8_t;
        if afatfs.filesystemType as libc::c_uint ==
               FAT_FILESYSTEM_TYPE_FAT16 as libc::c_int as libc::c_uint {
            afatfs.currentDirectory.type_0 =
                AFATFS_FILE_TYPE_FAT16_ROOT_DIRECTORY
        } else { afatfs.currentDirectory.type_0 = AFATFS_FILE_TYPE_DIRECTORY }
        afatfs.currentDirectory.firstCluster = afatfs.rootDirectoryCluster;
        afatfs.currentDirectory.attrib = 0x10 as libc::c_int as uint8_t;
        // Root directories don't have a directory entry to represent themselves:
        afatfs.currentDirectory.directoryEntryPos.sectorNumberPhysical =
            0 as libc::c_int as uint32_t;
        afatfs_fseek(&mut afatfs.currentDirectory, 0 as libc::c_int,
                     AFATFS_SEEK_SET);
        return 1 as libc::c_int != 0
    };
}
/* *
 * Begin the process of opening a file with the given name in the current working directory (paths in the filename are
 * not supported) using the given mode.
 *
 * To open the current working directory, pass "." for filename.
 *
 * The complete() callback is called when finished with either a file handle (file was opened) or NULL upon failure.
 *
 * Supported file mode strings:
 *
 * r - Read from an existing file
 * w - Create a file for write access, if the file already exists then truncate it
 * a - Create a file for write access to the end of the file only, if the file already exists then append to it
 *
 * r+ - Read and write from an existing file
 * w+ - Read and write from an existing file, if the file doesn't already exist it is created
 * a+ - Read from or append to an existing file, if the file doesn't already exist it is created TODO
 *
 * as - Create a new file which is stored contiguously on disk (high performance mode/freefile) for append or write
 * ws   If the file is already non-empty or freefile support is not compiled in then it will fall back to non-contiguous
 *      operation.
 *
 * All other mode strings are illegal. In particular, don't add "b" to the end of the mode string.
 *
 * Returns false if the the open failed really early (out of file handles).
 */
#[no_mangle]
pub unsafe extern "C" fn afatfs_fopen(mut filename: *const libc::c_char,
                                      mut mode: *const libc::c_char,
                                      mut complete: afatfsFileCallback_t)
 -> bool {
    let mut fileMode: uint8_t = 0 as libc::c_int as uint8_t;
    let mut file: afatfsFilePtr_t = 0 as *mut afatfsFile_t;
    match *mode.offset(0 as libc::c_int as isize) as libc::c_int {
        114 => { fileMode = 1 as libc::c_int as uint8_t }
        119 => {
            fileMode = (2 as libc::c_int | 16 as libc::c_int) as uint8_t
        }
        97 => { fileMode = (4 as libc::c_int | 16 as libc::c_int) as uint8_t }
        _ => { }
    }
    match *mode.offset(1 as libc::c_int as isize) as libc::c_int {
        43 => {
            fileMode =
                (fileMode as libc::c_int | 1 as libc::c_int) as uint8_t;
            if fileMode as libc::c_int == 1 as libc::c_int {
                fileMode =
                    (fileMode as libc::c_int | 2 as libc::c_int) as uint8_t
            }
        }
        115 => {
            fileMode =
                (fileMode as libc::c_int |
                     (8 as libc::c_int | 32 as libc::c_int)) as uint8_t
        }
        _ => { }
    }
    file = afatfs_allocateFileHandle();
    if !file.is_null() {
        afatfs_createFile(file, filename, 0x20 as libc::c_int as uint8_t,
                          fileMode, complete);
    } else if complete.is_some() {
        complete.expect("non-null function pointer")(0 as afatfsFilePtr_t);
    }
    return !file.is_null();
}
/* *
 * Write a single character to the file at the current cursor position. If the cache is too busy to accept the write,
 * it is silently dropped.
 */
#[no_mangle]
pub unsafe extern "C" fn afatfs_fputc(mut file: afatfsFilePtr_t,
                                      mut c: uint8_t) {
    let mut cursorOffsetInSector: uint32_t =
        (*file).cursorOffset.wrapping_rem(512 as libc::c_int as libc::c_uint);
    let mut cacheIndex: libc::c_int =
        (*file).writeLockedCacheIndex as libc::c_int;
    /* If we've already locked the current sector in the cache, and we won't be completing the sector, we won't
     * be caching/uncaching/seeking, so we can just run this simpler fast case.
     */
    if cacheIndex != -(1 as libc::c_int) &&
           cursorOffsetInSector !=
               (512 as libc::c_int - 1 as libc::c_int) as libc::c_uint {
        *afatfs_cacheSectorGetMemory(cacheIndex).offset(cursorOffsetInSector
                                                            as isize) = c;
        (*file).cursorOffset = (*file).cursorOffset.wrapping_add(1)
    } else {
        // Slow path
        afatfs_fwrite(file, &mut c,
                      ::core::mem::size_of::<uint8_t>() as libc::c_ulong as
                          uint32_t);
    };
}
/* *
 * Attempt to write `len` bytes from `buffer` into the `file`.
 *
 * Returns the number of bytes actually written.
 *
 * 0 will be returned when:
 *     The filesystem is busy (try again later)
 *
 * Fewer bytes will be written than requested when:
 *     The write spanned a sector boundary and the next sector's contents/location was not yet available in the cache.
 *     Or you tried to extend the length of the file but the filesystem is full (check afatfs_isFull()).
 */
#[no_mangle]
pub unsafe extern "C" fn afatfs_fwrite(mut file: afatfsFilePtr_t,
                                       mut buffer: *const uint8_t,
                                       mut len: uint32_t) -> uint32_t {
    if (*file).mode as libc::c_int & (4 as libc::c_int | 2 as libc::c_int) ==
           0 as libc::c_int {
        return 0 as libc::c_int as uint32_t
    }
    if afatfs_fileIsBusy(file) {
        // There might be a seek pending
        return 0 as libc::c_int as uint32_t
    }
    let mut cursorOffsetInSector: uint32_t =
        (*file).cursorOffset.wrapping_rem(512 as libc::c_int as libc::c_uint);
    let mut writtenBytes: uint32_t = 0 as libc::c_int as uint32_t;
    while len > 0 as libc::c_int as libc::c_uint {
        let mut bytesToWriteThisSector: uint32_t =
            ({
                 let mut _a: libc::c_uint =
                     (512 as libc::c_int as
                          libc::c_uint).wrapping_sub(cursorOffsetInSector);
                 let mut _b: uint32_t = len;
                 if _a < _b { _a } else { _b }
             });
        let mut sectorBuffer: *mut uint8_t = 0 as *mut uint8_t;
        sectorBuffer = afatfs_fileLockCursorSectorForWrite(file);
        if sectorBuffer.is_null() { break ; }
        memcpy(sectorBuffer.offset(cursorOffsetInSector as isize) as
                   *mut libc::c_void, buffer as *const libc::c_void,
               bytesToWriteThisSector as libc::c_ulong);
        writtenBytes =
            (writtenBytes as
                 libc::c_uint).wrapping_add(bytesToWriteThisSector) as
                uint32_t as uint32_t;
        /*
         * If the seek doesn't complete immediately then we'll break and wait for that seek to complete by waiting for
         * the file to be non-busy on entry again.
         *
         * A seek operation should always be able to queue on the file since we have checked that the file wasn't busy
         * on entry (fseek will never return AFATFS_OPERATION_FAILURE).
         *
         * If the seek has to queue, when the seek completes, it'll update the fileSize for us to contain the cursor.
         */
        if afatfs_fseekInternal(file, bytesToWriteThisSector, None) as
               libc::c_uint ==
               AFATFS_OPERATION_IN_PROGRESS as libc::c_int as libc::c_uint {
            break ;
        }
        if (*file).mode as libc::c_int & 8 as libc::c_int != 0 as libc::c_int
           {
            afatfs_assert((*file).cursorCluster <
                              afatfs.freeFile.firstCluster);
        }
        len =
            (len as libc::c_uint).wrapping_sub(bytesToWriteThisSector) as
                uint32_t as uint32_t;
        buffer = buffer.offset(bytesToWriteThisSector as isize);
        cursorOffsetInSector = 0 as libc::c_int as uint32_t
    }
    return writtenBytes;
}
/* *
 * Attempt to read `len` bytes from `file` into the `buffer`.
 *
 * Returns the number of bytes actually read.
 *
 * 0 will be returned when:
 *     The filesystem is busy (try again later)
 *     EOF was reached (check afatfs_isEof())
 *
 * Fewer bytes than requested will be read when:
 *     The read spans a AFATFS_SECTOR_SIZE boundary and the following sector was not available in the cache yet.
 */
#[no_mangle]
pub unsafe extern "C" fn afatfs_fread(mut file: afatfsFilePtr_t,
                                      mut buffer: *mut uint8_t,
                                      mut len: uint32_t) -> uint32_t {
    if (*file).mode as libc::c_int & 1 as libc::c_int == 0 as libc::c_int {
        return 0 as libc::c_int as uint32_t
    }
    if afatfs_fileIsBusy(file) {
        // There might be a seek pending
        return 0 as libc::c_int as uint32_t
    }
    /*
     * If we've just previously fwritten() to extend the file, the logical filesize will be out of date and the cursor
     * will appear to be beyond the end of the file (but actually it's precisely at the end of the file, because if
     * we had seeked backwards to where we could read something with fseek(), we would have updated the filesize).
     */
    if (*file).cursorOffset >= (*file).logicalSize {
        return 0 as libc::c_int as uint32_t
    }
    len =
        ({
             let mut _a: libc::c_uint =
                 (*file).logicalSize.wrapping_sub((*file).cursorOffset);
             let mut _b: uint32_t = len;
             if _a < _b { _a } else { _b }
         });
    let mut readBytes: uint32_t = 0 as libc::c_int as uint32_t;
    let mut cursorOffsetInSector: uint32_t =
        (*file).cursorOffset.wrapping_rem(512 as libc::c_int as libc::c_uint);
    while len > 0 as libc::c_int as libc::c_uint {
        let mut bytesToReadThisSector: uint32_t =
            ({
                 let mut _a: libc::c_uint =
                     (512 as libc::c_int as
                          libc::c_uint).wrapping_sub(cursorOffsetInSector);
                 let mut _b: uint32_t = len;
                 if _a < _b { _a } else { _b }
             });
        let mut sectorBuffer: *mut uint8_t = 0 as *mut uint8_t;
        sectorBuffer = afatfs_fileRetainCursorSectorForRead(file);
        if sectorBuffer.is_null() {
            // Cache is currently busy
            return readBytes
        }
        memcpy(buffer as *mut libc::c_void,
               sectorBuffer.offset(cursorOffsetInSector as isize) as
                   *const libc::c_void,
               bytesToReadThisSector as libc::c_ulong);
        readBytes =
            (readBytes as libc::c_uint).wrapping_add(bytesToReadThisSector) as
                uint32_t as uint32_t;
        /*
         * If the seek doesn't complete immediately then we'll break and wait for that seek to complete by waiting for
         * the file to be non-busy on entry again.
         *
         * A seek operation should always be able to queue on the file since we have checked that the file wasn't busy
         * on entry (fseek will never return AFATFS_OPERATION_FAILURE).
         */
        if afatfs_fseekInternal(file, bytesToReadThisSector, None) as
               libc::c_uint ==
               AFATFS_OPERATION_IN_PROGRESS as libc::c_int as libc::c_uint {
            break ;
        }
        len =
            (len as libc::c_uint).wrapping_sub(bytesToReadThisSector) as
                uint32_t as uint32_t;
        buffer = buffer.offset(bytesToReadThisSector as isize);
        cursorOffsetInSector = 0 as libc::c_int as uint32_t
    }
    return readBytes;
}
/* *
 * Returns true if the file's pointer position currently lies at the end-of-file point (i.e. one byte beyond the last
 * byte in the file).
 */
#[no_mangle]
pub unsafe extern "C" fn afatfs_feof(mut file: afatfsFilePtr_t) -> bool {
    return (*file).cursorOffset >= (*file).logicalSize;
}
/* *
 * Continue any queued operations on the given file.
 */
unsafe extern "C" fn afatfs_fileOperationContinue(mut file:
                                                      *mut afatfsFile_t) {
    if (*file).type_0 as libc::c_uint ==
           AFATFS_FILE_TYPE_NONE as libc::c_int as libc::c_uint {
        return
    }
    match (*file).operation.operation as libc::c_uint {
        1 => { afatfs_createFileContinue(file); }
        2 => { afatfs_fseekInternalContinue(file); }
        3 => { afatfs_fcloseContinue(file); }
        5 => { afatfs_funlinkContinue(file); }
        4 => { afatfs_ftruncateContinue(file, 0 as libc::c_int != 0); }
        6 => { afatfs_appendSuperclusterContinue(file); }
        8 => { afatfs_appendRegularFreeClusterContinue(file); }
        9 => { afatfs_extendSubdirectoryContinue(file); }
        7 | 0 | _ => { }
    };
}
/* *
 * Check files for pending operations and execute them.
 */
unsafe extern "C" fn afatfs_fileOperationsPoll() {
    afatfs_fileOperationContinue(&mut afatfs.currentDirectory);
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 3 as libc::c_int {
        afatfs_fileOperationContinue(&mut *afatfs.openFiles.as_mut_ptr().offset(i
                                                                                    as
                                                                                    isize));
        i += 1
    };
}
/* *
 * Return the available size of the freefile (used for files in contiguous append mode)
 */
#[no_mangle]
pub unsafe extern "C" fn afatfs_getContiguousFreeSpace() -> uint32_t {
    return afatfs.freeFile.logicalSize;
}
/* *
 * Call to set up the initial state for finding the largest block of free space on the device whose corresponding FAT
 * sectors are themselves entirely free space (so the free space has dedicated FAT sectors of its own).
 */
unsafe extern "C" fn afatfs_findLargestContiguousFreeBlockBegin() {
    // The first FAT sector has two reserved entries, so it isn't eligible for this search. Start at the next FAT sector.
    afatfs.initState.freeSpaceSearch.candidateStart =
        afatfs_fatEntriesPerSector();
    afatfs.initState.freeSpaceSearch.candidateEnd =
        afatfs.initState.freeSpaceSearch.candidateStart;
    afatfs.initState.freeSpaceSearch.bestGapStart =
        0 as libc::c_int as uint32_t;
    afatfs.initState.freeSpaceSearch.bestGapLength =
        0 as libc::c_int as uint32_t;
    afatfs.initState.freeSpaceSearch.phase =
        AFATFS_FREE_SPACE_SEARCH_PHASE_FIND_HOLE;
}
/* *
 * Call to continue the search for the largest contiguous block of free space on the device.
 *
 * Returns:
 *     AFATFS_OPERATION_IN_PROGRESS - SD card is busy, call again later to resume
 *     AFATFS_OPERATION_SUCCESS - When the search has finished and afatfs.initState.freeSpaceSearch has been updated with the details of the best gap.
 *     AFATFS_OPERATION_FAILURE - When a read error occured
 */
unsafe extern "C" fn afatfs_findLargestContiguousFreeBlockContinue()
 -> afatfsOperationStatus_e {
    let mut opState: *mut afatfsFreeSpaceSearch_t =
        &mut afatfs.initState.freeSpaceSearch;
    let mut fatEntriesPerSector: uint32_t = afatfs_fatEntriesPerSector();
    let mut candidateGapLength: uint32_t = 0;
    let mut searchLimit: uint32_t = 0;
    let mut searchStatus: afatfsFindClusterStatus_e =
        AFATFS_FIND_CLUSTER_IN_PROGRESS;
    loop  {
        match (*opState).phase as libc::c_uint {
            0 => {
                // Find the first free cluster
                match afatfs_findClusterWithCondition(CLUSTER_SEARCH_FREE_AT_BEGINNING_OF_FAT_SECTOR,
                                                      &mut (*opState).candidateStart,
                                                      afatfs.numClusters.wrapping_add(2
                                                                                          as
                                                                                          libc::c_int
                                                                                          as
                                                                                          libc::c_uint))
                          as libc::c_uint {
                    1 => {
                        (*opState).candidateEnd =
                            (*opState).candidateStart.wrapping_add(1 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_uint);
                        (*opState).phase =
                            AFATFS_FREE_SPACE_SEARCH_PHASE_GROW_HOLE
                    }
                    2 => {
                        // Some sort of read error occured
                        return AFATFS_OPERATION_FAILURE
                    }
                    3 => {
                        // We finished searching the volume (didn't find any more holes to examine)
                        return AFATFS_OPERATION_SUCCESS
                    }
                    0 => { return AFATFS_OPERATION_IN_PROGRESS }
                    _ => { }
                }
            }
            1 => {
                // Find the first used cluster after the beginning of the hole (that signals the end of the hole)
                // Don't search beyond the end of the volume, or such that the freefile size would exceed the max filesize
                searchLimit =
                    ({
                         let mut _a: libc::c_ulong =
                             ((*opState).candidateStart as
                                  uint64_t).wrapping_add((0xffffffff as
                                                              libc::c_uint).wrapping_div(afatfs_clusterSize())
                                                             as
                                                             libc::c_ulong);
                         let mut _b: libc::c_uint =
                             afatfs.numClusters.wrapping_add(2 as libc::c_int
                                                                 as
                                                                 libc::c_uint);
                         if _a < _b as libc::c_ulong {
                             _a
                         } else { _b as libc::c_ulong }
                     }) as uint32_t;
                searchStatus =
                    afatfs_findClusterWithCondition(CLUSTER_SEARCH_OCCUPIED,
                                                    &mut (*opState).candidateEnd,
                                                    searchLimit);
                match searchStatus as libc::c_uint {
                    3 | 1 => {
                        // Either we found a used sector, or the search reached the end of the volume or exceeded the max filesize
                        candidateGapLength =
                            (*opState).candidateEnd.wrapping_sub((*opState).candidateStart);
                        if candidateGapLength > (*opState).bestGapLength {
                            (*opState).bestGapStart =
                                (*opState).candidateStart;
                            (*opState).bestGapLength = candidateGapLength
                        }
                        if searchStatus as libc::c_uint ==
                               AFATFS_FIND_CLUSTER_NOT_FOUND as libc::c_int as
                                   libc::c_uint {
                            // This is the best hole there can be
                            return AFATFS_OPERATION_SUCCESS
                        } else {
                            // Start a new search for a new hole
                            (*opState).candidateStart =
                                roundUpTo((*opState).candidateEnd.wrapping_add(1
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   libc::c_uint),
                                          fatEntriesPerSector);
                            (*opState).phase =
                                AFATFS_FREE_SPACE_SEARCH_PHASE_FIND_HOLE
                        }
                    }
                    2 => {
                        // Some sort of read error occured
                        return AFATFS_OPERATION_FAILURE
                    }
                    0 => { return AFATFS_OPERATION_IN_PROGRESS }
                    _ => { }
                }
            }
            _ => { }
        }
    };
}
unsafe extern "C" fn afatfs_freeFileCreated(mut file: *mut afatfsFile_t) {
    if !file.is_null() {
        // Did the freefile already have allocated space?
        if (*file).logicalSize > 0 as libc::c_int as libc::c_uint {
            // We've completed freefile init, move on to the next init phase
            afatfs.initPhase =
                (AFATFS_INITIALIZATION_FREEFILE_LAST as libc::c_int +
                     1 as libc::c_int) as afatfsInitializationPhase_e
        } else {
            // Allocate clusters for the freefile
            afatfs_findLargestContiguousFreeBlockBegin();
            afatfs.initPhase = AFATFS_INITIALIZATION_FREEFILE_FAT_SEARCH
        }
    } else {
        // Failed to allocate an entry
        afatfs.lastError = AFATFS_ERROR_GENERIC;
        afatfs.filesystemState = AFATFS_FILESYSTEM_STATE_FATAL
    };
}
unsafe extern "C" fn afatfs_initContinue() {
    let mut status: afatfsOperationStatus_e = AFATFS_OPERATION_IN_PROGRESS;
    let mut sector: *mut uint8_t = 0 as *mut uint8_t;
    loop  {
        match afatfs.initPhase as libc::c_uint {
            0 => {
                if !(afatfs_cacheSector(0 as libc::c_int as uint32_t,
                                        &mut sector,
                                        (1 as libc::c_int | 8 as libc::c_int)
                                            as uint8_t,
                                        0 as libc::c_int as uint32_t) as
                         libc::c_uint ==
                         AFATFS_OPERATION_SUCCESS as libc::c_int as
                             libc::c_uint) {
                    break ;
                }
                if afatfs_parseMBR(sector) {
                    afatfs.initPhase = AFATFS_INITIALIZATION_READ_VOLUME_ID
                } else {
                    afatfs.lastError = AFATFS_ERROR_BAD_MBR;
                    afatfs.filesystemState = AFATFS_FILESYSTEM_STATE_FATAL;
                    break ;
                }
            }
            1 => {
                if afatfs_cacheSector(afatfs.partitionStartSector,
                                      &mut sector,
                                      (1 as libc::c_int | 8 as libc::c_int) as
                                          uint8_t,
                                      0 as libc::c_int as uint32_t) as
                       libc::c_uint ==
                       AFATFS_OPERATION_SUCCESS as libc::c_int as libc::c_uint
                   {
                    if afatfs_parseVolumeID(sector) {
                        // Open the root directory
                        afatfs_chdir(0 as afatfsFilePtr_t);
                        afatfs.initPhase += 1
                    } else {
                        afatfs.lastError = AFATFS_ERROR_BAD_FILESYSTEM_HEADER;
                        afatfs.filesystemState = AFATFS_FILESYSTEM_STATE_FATAL
                    }
                }
                break ;
            }
            2 => {
                afatfs.initPhase = AFATFS_INITIALIZATION_FREEFILE_CREATING;
                afatfs_createFile(&mut afatfs.freeFile,
                                  b"FREESPAC.E\x00" as *const u8 as
                                      *const libc::c_char,
                                  (0x4 as libc::c_int | 0x1 as libc::c_int) as
                                      uint8_t,
                                  (16 as libc::c_int | 32 as libc::c_int) as
                                      uint8_t,
                                  Some(afatfs_freeFileCreated as
                                           unsafe extern "C" fn(_:
                                                                    *mut afatfsFile_t)
                                               -> ()));
                break ;
            }
            3 => {
                afatfs_fileOperationContinue(&mut afatfs.freeFile);
                break ;
            }
            4 => {
                if !(afatfs_findLargestContiguousFreeBlockContinue() as
                         libc::c_uint ==
                         AFATFS_OPERATION_SUCCESS as libc::c_int as
                             libc::c_uint) {
                    break ;
                }
                // If the freefile ends up being empty then we only have to save its directory entry:
                afatfs.initPhase =
                    AFATFS_INITIALIZATION_FREEFILE_SAVE_DIR_ENTRY;
                if afatfs.initState.freeSpaceSearch.bestGapLength >
                       (100 as libc::c_int + 1 as libc::c_int) as libc::c_uint
                   {
                    afatfs.initState.freeSpaceSearch.bestGapLength =
                        (afatfs.initState.freeSpaceSearch.bestGapLength as
                             libc::c_uint).wrapping_sub(100 as libc::c_int as
                                                            libc::c_uint) as
                            uint32_t as uint32_t;
                    // Else the freefile's FAT chain and filesize remains the default (empty)
                    afatfs.initState.freeSpaceSearch.bestGapLength =
                        (afatfs.initState.freeSpaceSearch.bestGapLength.wrapping_sub(1
                                                                                         as
                                                                                         libc::c_int
                                                                                         as
                                                                                         libc::c_uint)
                             &
                             !afatfs_fatEntriesPerSector().wrapping_sub(1 as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint)).wrapping_add(1
                                                                                                            as
                                                                                                            libc::c_int
                                                                                                            as
                                                                                                            libc::c_uint);
                    if afatfs.initState.freeSpaceSearch.bestGapLength >
                           afatfs_fatEntriesPerSector() {
                        let mut startCluster: uint32_t =
                            afatfs.initState.freeSpaceSearch.bestGapStart;
                        /* So that the freefile never becomes empty, we want it to occupy a non-integer number of
                     * superclusters. So its size mod the number of clusters in a supercluster should be 1.
                     */
                        // Anything useful left over?
                        // Points 1-beyond the final cluster of the freefile:
                        let mut endCluster: uint32_t =
                            afatfs.initState.freeSpaceSearch.bestGapStart.wrapping_add(afatfs.initState.freeSpaceSearch.bestGapLength);
                        afatfs_assert(endCluster <
                                          afatfs.numClusters.wrapping_add(2 as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint));
                        afatfs.initState.freeSpaceFAT.startCluster =
                            startCluster;
                        afatfs.initState.freeSpaceFAT.endCluster = endCluster;
                        afatfs.freeFile.firstCluster = startCluster;
                        afatfs.freeFile.logicalSize =
                            afatfs.initState.freeSpaceSearch.bestGapLength.wrapping_mul(afatfs_clusterSize());
                        afatfs.freeFile.physicalSize =
                            afatfs.freeFile.logicalSize;
                        // We can write the FAT table for the freefile now
                        afatfs.initPhase =
                            AFATFS_INITIALIZATION_FREEFILE_UPDATE_FAT
                    }
                }
            }
            5 => {
                status =
                    afatfs_FATFillWithPattern(AFATFS_FAT_PATTERN_TERMINATED_CHAIN,
                                              &mut afatfs.initState.freeSpaceFAT.startCluster,
                                              afatfs.initState.freeSpaceFAT.endCluster);
                if status as libc::c_uint ==
                       AFATFS_OPERATION_SUCCESS as libc::c_int as libc::c_uint
                   {
                    afatfs.initPhase =
                        AFATFS_INITIALIZATION_FREEFILE_SAVE_DIR_ENTRY
                } else {
                    if status as libc::c_uint ==
                           AFATFS_OPERATION_FAILURE as libc::c_int as
                               libc::c_uint {
                        afatfs.lastError = AFATFS_ERROR_GENERIC;
                        afatfs.filesystemState = AFATFS_FILESYSTEM_STATE_FATAL
                    }
                    break ;
                }
            }
            6 => {
                status =
                    afatfs_saveDirectoryEntry(&mut afatfs.freeFile,
                                              AFATFS_SAVE_DIRECTORY_NORMAL);
                if status as libc::c_uint ==
                       AFATFS_OPERATION_SUCCESS as libc::c_int as libc::c_uint
                   {
                    afatfs.initPhase += 1
                } else {
                    if status as libc::c_uint ==
                           AFATFS_OPERATION_FAILURE as libc::c_int as
                               libc::c_uint {
                        afatfs.lastError = AFATFS_ERROR_GENERIC;
                        afatfs.filesystemState = AFATFS_FILESYSTEM_STATE_FATAL
                    }
                    break ;
                }
            }
            7 => {
                afatfs.filesystemState = AFATFS_FILESYSTEM_STATE_READY;
                break ;
            }
            _ => { break ; }
        }
    };
}
/* *
 * Check to see if there are any pending operations on the filesystem and perform a little work (without waiting on the
 * sdcard). You must call this periodically.
 */
#[no_mangle]
pub unsafe extern "C" fn afatfs_poll() {
    // Only attempt to continue FS operations if the card is present & ready, otherwise we would just be wasting time
    if sdcard_poll() {
        afatfs_flush();
        match afatfs.filesystemState as libc::c_uint {
            2 => { afatfs_initContinue(); }
            3 => { afatfs_fileOperationsPoll(); }
            _ => { }
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn afatfs_getFilesystemState()
 -> afatfsFilesystemState_e {
    return afatfs.filesystemState;
}
#[no_mangle]
pub unsafe extern "C" fn afatfs_getLastError() -> afatfsError_e {
    return afatfs.lastError;
}
#[no_mangle]
pub unsafe extern "C" fn afatfs_init() {
    afatfs.filesystemState = AFATFS_FILESYSTEM_STATE_INITIALIZATION;
    afatfs.initPhase = AFATFS_INITIALIZATION_READ_MBR;
    afatfs.lastClusterAllocated = 2 as libc::c_int as uint32_t;
}
/* *
 * Shut down the filesystem, flushing all data to the disk. Keep calling until it returns true.
 *
 * dirty - Set to true to skip the flush operation and terminate immediately (buffered data will be lost!)
 */
#[no_mangle]
pub unsafe extern "C" fn afatfs_destroy(mut dirty: bool) -> bool {
    // Only attempt detailed cleanup if the filesystem is in reasonable looking state
    if !dirty &&
           afatfs.filesystemState as libc::c_uint ==
               AFATFS_FILESYSTEM_STATE_READY as libc::c_int as libc::c_uint {
        let mut openFileCount: libc::c_int = 0 as libc::c_int;
        let mut i: libc::c_int = 0 as libc::c_int;
        while i < 3 as libc::c_int {
            if afatfs.openFiles[i as usize].type_0 as libc::c_uint !=
                   AFATFS_FILE_TYPE_NONE as libc::c_int as libc::c_uint {
                afatfs_fclose(&mut *afatfs.openFiles.as_mut_ptr().offset(i as
                                                                             isize),
                              None);
                // The close operation might not finish right away, so count this file as still open for now
                openFileCount += 1
            }
            i += 1
        }
        if afatfs.freeFile.type_0 as libc::c_uint !=
               AFATFS_FILE_TYPE_NONE as libc::c_int as libc::c_uint {
            afatfs_fclose(&mut afatfs.freeFile, None);
            openFileCount += 1
        }
        if afatfs.currentDirectory.type_0 as libc::c_uint !=
               AFATFS_FILE_TYPE_NONE as libc::c_int as libc::c_uint {
            afatfs_fclose(&mut afatfs.currentDirectory, None);
            openFileCount += 1
        }
        afatfs_poll();
        if !afatfs_flush() { return 0 as libc::c_int != 0 }
        if afatfs.cacheFlushInProgress { return 0 as libc::c_int != 0 }
        if openFileCount > 0 as libc::c_int { return 0 as libc::c_int != 0 }
    }
    // Clear the afatfs so it's as if we never ran
    memset(&mut afatfs as *mut afatfs_t as *mut libc::c_void,
           0 as libc::c_int,
           ::core::mem::size_of::<afatfs_t>() as libc::c_ulong);
    return 1 as libc::c_int != 0;
}
/* *
 * Get a pessimistic estimate of the amount of buffer space that we have available to write to immediately.
 */
#[no_mangle]
pub unsafe extern "C" fn afatfs_getFreeBufferSpace() -> uint32_t {
    let mut result: uint32_t = 0 as libc::c_int as uint32_t;
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 8 as libc::c_int {
        if afatfs.cacheDescriptor[i as usize].locked() == 0 &&
               (afatfs.cacheDescriptor[i as usize].state as libc::c_uint ==
                    AFATFS_CACHE_STATE_EMPTY as libc::c_int as libc::c_uint ||
                    afatfs.cacheDescriptor[i as usize].state as libc::c_uint
                        ==
                        AFATFS_CACHE_STATE_IN_SYNC as libc::c_int as
                            libc::c_uint) {
            result =
                (result as
                     libc::c_uint).wrapping_add(512 as libc::c_int as
                                                    libc::c_uint) as uint32_t
                    as uint32_t
        }
        i += 1
    }
    return result;
}
