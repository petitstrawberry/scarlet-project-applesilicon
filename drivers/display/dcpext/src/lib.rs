#![no_std]

//! Apple external-display DCP driver for boot-time Type-C DisplayPort output.
//!
//! The initial implementation deliberately exposes one mirrored desktop:
//! DCPext selects the external timing (preferring 1920x1080 at up to 60 Hz),
//! while the internal DCP scans out the same two buffers with scaling.
//! Runtime Type-C hotplug and independent desktops are left for a later layer.
//!
//! # Provenance
//!
//! EPIC DPTX calls, callbacks, and iBoot swaps follow Asahi Linux's
//! `drivers/gpu/drm/apple/dptxep.c` and m1n1's `dptx_port_ep.c` and
//! `dcp_iboot.c`. See the repository `ATTRIBUTION.md`.

extern crate alloc;

// DCP and DCPext speak the same IOMFB shared-memory protocol.  Keep one
// implementation while the display protocol is still housed in the DCP
// driver; both drivers compile this module against their own RTKit instance.
#[path = "../../dcp/src/iomfb.rs"]
mod iomfb;

use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::mem;
use core::sync::atomic::{AtomicUsize, Ordering};

use scarlet::device::graphics::{FramebufferConfig, PixelFormat};
use scarlet::device::iommu::IommuStreamId;
use scarlet::device::manager::{DeviceManager, DriverPriority, is_probe_defer, probe_defer};
use scarlet::device::platform::resource::PlatformDeviceResourceType;
use scarlet::device::platform::{PlatformDeviceDriver, PlatformDeviceInfo};
use scarlet::device::remoteproc::{RemoteProcessor, RemoteprocDmaMapper, RemoteprocError};
use scarlet::device::usb::TypecOrientation;
use scarlet::println;
use scarlet::sync::{IrqSpinLock, Mutex};
use scarlet::{arch, time};
use scarlet_driver_apple_asc::get_apple_asc_by_phandle;
use scarlet_driver_apple_atcphy::{AppleAtcPhy, AtcPhyMode, get_atcphy_by_core_paddr};
use scarlet_driver_apple_cd321x::{
    Cd321xDisplayPortLaneMode, displayport_hpd_level, displayport_lane_mode,
    displayport_pin_assignment, get_cd321x_displayport_status_by_address,
    get_cd321x_status_by_address, has_displayport_connection,
};
use scarlet_driver_apple_dart::{DartDomain, DartInstance, DartPageTable, get_dart_by_phandle};
use scarlet_driver_apple_dpxbar::route_t8103_dpphy;
use scarlet_driver_apple_epic::EpicEndpoint;
use scarlet_driver_apple_rtkit::AppleRtkit;

use iomfb::{BandwidthRegisters, Iomfb};

const DCP_SYSTEM_EP: u8 = 0x20;
const DCP_DPTX_PORT_EP: u8 = 0x2a;
const DCP_IBOOT_EP: u8 = 0x23;
const DCP_IBOOT_SUBTYPE: u16 = 0xc0;
const DCP_SERVICE_TIMEOUT_US: u64 = 5_000_000;
const DCP_LINK_TIMEOUT_US: u64 = 2_000_000;
const DCP_STATUS_RETRIES: usize = 20;
const DCP_STATUS_RETRY_US: u64 = 100_000;

#[derive(Clone, Copy)]
struct J293TypecRoute {
    atc_index: u32,
    cd321x_address: u16,
    atc_core_paddr: usize,
}

const J293_TYPEC_ROUTES: [J293TypecRoute; 2] = [
    J293TypecRoute {
        atc_index: 0,
        cd321x_address: 0x38,
        atc_core_paddr: 0x383_000_000,
    },
    J293TypecRoute {
        atc_index: 1,
        cd321x_address: 0x3f,
        atc_core_paddr: 0x503_000_000,
    },
];
const T8103_ASC_DRAM_MASK: u64 = 0xf_0000_0000;
const DCP_DYNAMIC_IOVA_BASE: usize = 0x3000_0000;
const DCP_SCANOUT_IOVA_BASE: usize = 0x4000_0000;
const DCP_DART_FLAGS: u64 = 1;

const DPTX_CONNECT: u32 = 11;
const DPTX_REQUEST_DISPLAY: u32 = 6;
const DPTX_SET_HPD: u32 = 8;
const DPTX_CONNECTED: u32 = 1 << 15;
const DPTX_SERVICE_PORT: u32 = 0;

const DPTX_APCALL_ACTIVATE: u32 = 0;
const DPTX_APCALL_GET_MAX_DRIVE_SETTINGS: u32 = 2;
const DPTX_APCALL_SET_DRIVE_SETTINGS: u32 = 3;
const DPTX_APCALL_GET_DRIVE_SETTINGS: u32 = 4;
const DPTX_APCALL_WILL_CHANGE_LINK_CONFIG: u32 = 5;
const DPTX_APCALL_DID_CHANGE_LINK_CONFIG: u32 = 6;
const DPTX_APCALL_GET_MAX_LINK_RATE: u32 = 7;
const DPTX_APCALL_GET_LINK_RATE: u32 = 8;
const DPTX_APCALL_SET_LINK_RATE: u32 = 9;
const DPTX_APCALL_GET_MAX_LANE_COUNT: u32 = 10;
const DPTX_APCALL_GET_ACTIVE_LANE_COUNT: u32 = 11;
const DPTX_APCALL_SET_ACTIVE_LANE_COUNT: u32 = 12;
const DPTX_APCALL_GET_SUPPORTS_DOWN_SPREAD: u32 = 13;
const DPTX_APCALL_GET_DOWN_SPREAD: u32 = 14;
const DPTX_APCALL_SET_DOWN_SPREAD: u32 = 15;
const DPTX_APCALL_GET_SUPPORTS_LANE_MAPPING: u32 = 16;
const DPTX_APCALL_SET_LANE_MAP: u32 = 17;
const DPTX_APCALL_GET_SUPPORTS_HPD: u32 = 18;
const DPTX_APCALL_FORCE_HPD: u32 = 19;
const DPTX_APCALL_SET_TILED_DISPLAY_HINTS: u32 = 21;

const DP_LINK_RATE_RBR: u32 = 0x06;
const DP_LINK_RATE_HBR: u32 = 0x0a;
const DP_LINK_RATE_HBR2: u32 = 0x14;
const DP_LINK_RATE_HBR3: u32 = 0x1e;

const IBOOT_SET_SURFACE: u32 = 1;
const IBOOT_SET_POWER: u32 = 2;
const IBOOT_GET_HPD: u32 = 3;
const IBOOT_GET_TIMING_MODES: u32 = 4;
const IBOOT_GET_COLOR_MODES: u32 = 5;
const IBOOT_SET_MODE: u32 = 6;
const IBOOT_SWAP_BEGIN: u32 = 15;
const IBOOT_SWAP_SET_LAYER: u32 = 16;
const IBOOT_SWAP_END: u32 = 18;

const SURFACE_FMT_BGRA8888: u32 = 1;
const ADDR_FORMAT_PLANAR: u32 = 1;
const COLORSPACE_DISPLAY_P3: u32 = 2;
const EOTF_GAMMA_SDR: u32 = 1;

/// Timing selected by DCPext for the shared mirror canvas.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MirrorMode {
    pub width: u32,
    pub height: u32,
    /// 16.16 fixed-point refresh rate, as reported by DCP firmware.
    pub fps: u32,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
struct DcpTimingMode {
    valid: u32,
    width: u32,
    height: u32,
    fps: u32,
    _pad: [u8; 8],
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
struct DcpColorMode {
    valid: u32,
    colorimetry: u32,
    eotf: u32,
    encoding: u32,
    bpp: u32,
    _pad: [u8; 4],
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
struct DcpPlane {
    valid: u32,
    addr: u64,
    tile_size: u32,
    stride: u32,
    _unknown: [u32; 4],
    addr_format: u32,
    _unknown2: u32,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
struct DcpLayer {
    planes: [DcpPlane; 3],
    _unknown: u32,
    plane_count: u32,
    width: u32,
    height: u32,
    surface_format: u32,
    colorspace: u32,
    eotf: u32,
    transform: u8,
    _pad: [u8; 3],
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
struct DcpRect {
    width: u32,
    height: u32,
    x: u32,
    y: u32,
}

fn bytes_of<T>(value: &T) -> &[u8] {
    // SAFETY: all callers pass packed, plain-old-data wire structures.
    unsafe { core::slice::from_raw_parts(value as *const T as *const u8, mem::size_of::<T>()) }
}

fn read_wire<T: Copy>(bytes: &[u8], offset: usize) -> Option<T> {
    let end = offset.checked_add(mem::size_of::<T>())?;
    if end > bytes.len() {
        return None;
    }
    // SAFETY: bounds are checked; firmware data is permitted to be unaligned.
    Some(unsafe { core::ptr::read_unaligned(bytes.as_ptr().add(offset) as *const T) })
}

fn read_le_u32(bytes: &[u8], offset: usize) -> Option<u32> {
    Some(u32::from_le_bytes(
        bytes.get(offset..offset + 4)?.try_into().ok()?,
    ))
}

fn read_le_u64(bytes: &[u8], offset: usize) -> Option<u64> {
    Some(u64::from_le_bytes(
        bytes.get(offset..offset + 8)?.try_into().ok()?,
    ))
}

fn write_le_u32(bytes: &mut [u8], offset: usize, value: u32) -> bool {
    let Some(target) = bytes.get_mut(offset..offset + 4) else {
        return false;
    };
    target.copy_from_slice(&value.to_le_bytes());
    true
}

fn write_le_u64(bytes: &mut [u8], offset: usize, value: u64) -> bool {
    let Some(target) = bytes.get_mut(offset..offset + 8) else {
        return false;
    };
    target.copy_from_slice(&value.to_le_bytes());
    true
}

fn read_be_u32(bytes: &[u8], offset: usize) -> Option<u32> {
    Some(u32::from_be_bytes(
        bytes.get(offset..offset + 4)?.try_into().ok()?,
    ))
}

fn read_be_u64(bytes: &[u8], offset: usize) -> Option<u64> {
    Some(u64::from_be_bytes(
        bytes.get(offset..offset + 8)?.try_into().ok()?,
    ))
}

fn property_phandle(device: &PlatformDeviceInfo, name: &str) -> Option<u32> {
    read_be_u32(device.property(name)?.value(), 0)
}

fn device_phandle(device: &PlatformDeviceInfo) -> Option<u32> {
    property_phandle(device, "phandle").or_else(|| property_phandle(device, "linux,phandle"))
}

fn phandle_reg(phandle: u32, index: usize) -> Option<(usize, usize)> {
    let fdt = scarlet::device::fdt::FdtManager::get_manager().get_fdt()?;
    for node in fdt.all_nodes() {
        let node_phandle = node
            .property("phandle")
            .or_else(|| node.property("linux,phandle"))
            .and_then(|property| read_be_u32(property.value, 0));
        if node_phandle != Some(phandle) {
            continue;
        }
        let region = node.reg()?.nth(index)?;
        return Some((region.starting_address as usize, region.size.unwrap_or(0)));
    }
    None
}

fn device_clock_frequency(device: &PlatformDeviceInfo) -> u64 {
    let referenced = device
        .property("clocks")
        .and_then(|property| read_be_u32(property.value(), 0));
    if let Some(phandle) = referenced
        && let Some(fdt) = scarlet::device::fdt::FdtManager::get_manager().get_fdt()
    {
        for node in fdt.all_nodes() {
            let node_phandle = node
                .property("phandle")
                .or_else(|| node.property("linux,phandle"))
                .and_then(|property| read_be_u32(property.value, 0));
            if node_phandle == Some(phandle)
                && let Some(frequency) = node
                    .property("clock-frequency")
                    .and_then(|property| read_be_u32(property.value, 0))
            {
                return frequency as u64;
            }
        }
    }

    device
        .property("clock-frequency")
        .and_then(|property| read_be_u32(property.value(), 0))
        .unwrap_or(0) as u64
}

fn iomfb_registers(
    device: &PlatformDeviceInfo,
) -> Result<(Vec<(usize, usize)>, Option<BandwidthRegisters>), &'static str> {
    let mut registers = device
        .get_resources()
        .iter()
        .filter(|resource| matches!(resource.res_type, PlatformDeviceResourceType::MEM))
        .skip(1)
        .map(|resource| {
            let size = resource
                .end
                .checked_sub(resource.start)
                .and_then(|length| length.checked_add(1))
                .ok_or("apple-dcpext: invalid display register resource")?;
            Ok((resource.start, size))
        })
        .collect::<Result<Vec<_>, &'static str>>()?;

    let Some(scratch_property) = device.property("apple,bw-scratch") else {
        return Ok((registers, None));
    };
    let scratch = scratch_property.value();
    let scratch_phandle =
        read_be_u32(scratch, 0).ok_or("apple-dcpext: invalid bw-scratch phandle")?;
    let scratch_reg =
        read_be_u32(scratch, 4).ok_or("apple-dcpext: invalid bw-scratch reg")? as usize;
    let scratch_index =
        read_be_u32(scratch, 8).ok_or("apple-dcpext: invalid bw-scratch index")? as usize;
    let scratch_offset =
        read_be_u32(scratch, 12).ok_or("apple-dcpext: invalid bw-scratch offset")? as usize;
    if scratch_index != registers.len() {
        return Err("apple-dcpext: unexpected bw-scratch display index");
    }
    let scratch_resource = phandle_reg(scratch_phandle, scratch_reg)
        .ok_or("apple-dcpext: bw-scratch resource not found")?;
    registers.push(scratch_resource);

    let mut doorbell = 0;
    let mut doorbell_bit = 0;
    if let Some(doorbell_property) = device.property("apple,bw-doorbell") {
        let value = doorbell_property.value();
        let phandle = read_be_u32(value, 0).ok_or("apple-dcpext: invalid bw-doorbell phandle")?;
        let reg = read_be_u32(value, 4).ok_or("apple-dcpext: invalid bw-doorbell reg")? as usize;
        let index =
            read_be_u32(value, 8).ok_or("apple-dcpext: invalid bw-doorbell index")? as usize;
        if index != registers.len() {
            return Err("apple-dcpext: unexpected bw-doorbell display index");
        }
        let resource =
            phandle_reg(phandle, reg).ok_or("apple-dcpext: bw-doorbell resource not found")?;
        doorbell = resource.0 as u64;
        registers.push(resource);
        let dcp_index = device
            .property("apple,dcp-index")
            .and_then(|property| read_be_u32(property.value(), 0))
            .unwrap_or(0);
        doorbell_bit = 2 + dcp_index;
    }

    Ok((
        registers,
        Some(BandwidthRegisters {
            scratch: scratch_resource.0.saturating_add(scratch_offset) as u64,
            doorbell,
            doorbell_bit,
        }),
    ))
}

fn iommu_spec(device: &PlatformDeviceInfo) -> Option<(u32, usize)> {
    let value = device.property("iommus")?.value();
    Some((read_be_u32(value, 0)?, read_be_u32(value, 4)? as usize))
}

fn find_piodma_iommu(dcp_phandle: u32) -> Option<(u32, usize)> {
    let fdt = scarlet::device::fdt::FdtManager::get_manager().get_fdt()?;
    for node in fdt.all_nodes() {
        let phandle = node
            .property("phandle")
            .or_else(|| node.property("linux,phandle"))
            .and_then(|property| read_be_u32(property.value, 0));
        if phandle != Some(dcp_phandle) {
            continue;
        }
        let piodma = node.children().find(|child| child.name == "piodma")?;
        let iommus = piodma.property("iommus")?;
        return Some((
            read_be_u32(iommus.value, 0)?,
            read_be_u32(iommus.value, 4)? as usize,
        ));
    }
    None
}

fn map_handoff_regions(
    table: &mut DartPageTable,
    device_phandle: u32,
) -> Result<usize, &'static str> {
    let fdt = scarlet::device::fdt::FdtManager::get_manager()
        .get_fdt()
        .ok_or("apple-dcpext: FDT unavailable")?;
    let reserved = fdt
        .find_node("/reserved-memory")
        .ok_or("apple-dcpext: reserved-memory missing")?;
    let mut mapped = 0usize;

    for node in reserved.children() {
        let Some(reg) = node.property("reg") else {
            continue;
        };
        let Some(addresses) = node.property("iommu-addresses") else {
            continue;
        };
        let Some(paddr) = read_be_u64(reg.value, 0) else {
            continue;
        };
        for tuple in addresses.value.chunks_exact(20) {
            if read_be_u32(tuple, 0) != Some(device_phandle) {
                continue;
            }
            let iova = read_be_u64(tuple, 4).ok_or("apple-dcpext: malformed handoff IOVA")?;
            let size = read_be_u64(tuple, 12).ok_or("apple-dcpext: malformed handoff size")?;
            table.map_contiguous(iova as usize, paddr as usize, size as usize, DCP_DART_FLAGS)?;
            mapped += 1;
        }
    }
    Ok(mapped)
}

struct DcpDmaMapper {
    table: Arc<IrqSpinLock<DartPageTable>>,
    dart: Arc<DartInstance>,
    next_iova: AtomicUsize,
    page_size: usize,
    dva_base: u64,
}

impl DcpDmaMapper {
    fn new(
        table: Arc<IrqSpinLock<DartPageTable>>,
        dart: Arc<DartInstance>,
        page_size: usize,
        dva_base: u64,
    ) -> Self {
        Self {
            table,
            dart,
            next_iova: AtomicUsize::new(DCP_DYNAMIC_IOVA_BASE),
            page_size,
            dva_base,
        }
    }

    fn dva_from_iova(&self, iova: usize) -> u64 {
        self.dva_base | iova as u64
    }

    fn iova_from_dva(&self, dva: u64) -> usize {
        (dva & !self.dva_base) as usize
    }
}

impl RemoteprocDmaMapper for DcpDmaMapper {
    fn alignment(&self) -> usize {
        self.page_size
    }

    fn map(&self, paddr: usize, size: usize) -> Result<u64, RemoteprocError> {
        if size == 0 || !paddr.is_multiple_of(self.page_size) {
            return Err(RemoteprocError::LoadFailed);
        }
        let size = size
            .div_ceil(self.page_size)
            .checked_mul(self.page_size)
            .ok_or(RemoteprocError::LoadFailed)?;
        let mut iova = self.next_iova.load(Ordering::Relaxed);
        loop {
            let next = iova
                .checked_add(size)
                .filter(|next| *next <= DCP_SCANOUT_IOVA_BASE)
                .ok_or(RemoteprocError::LoadFailed)?;
            match self.next_iova.compare_exchange_weak(
                iova,
                next,
                Ordering::Relaxed,
                Ordering::Relaxed,
            ) {
                Ok(_) => break,
                Err(current) => iova = current,
            }
        }
        self.table
            .lock()
            .map_contiguous(iova, paddr, size, DCP_DART_FLAGS)
            .map_err(|_| RemoteprocError::LoadFailed)?;
        self.dart
            .sync_page_tables()
            .map_err(|_| RemoteprocError::LoadFailed)?;
        Ok(self.dva_from_iova(iova))
    }

    fn translate(&self, dva: u64) -> Option<usize> {
        self.table.lock().translate_iova(self.iova_from_dva(dva))
    }

    fn unmap(&self, dva: u64, size: usize) {
        let pages = size.div_ceil(self.page_size);
        let iova = self.iova_from_dva(dva);
        let mut table = self.table.lock();
        for page in 0..pages {
            if let Err(error) = table.unmap_page(iova + page * self.page_size) {
                println!("[apple-dcpext] failed to unmap DVA {:#x}: {}", dva, error);
                return;
            }
        }
        drop(table);
        let _ = self.dart.sync_page_tables();
    }
}

struct DptxCallbackState {
    phy: Arc<IrqSpinLock<AppleAtcPhy>>,
    max_lanes: u32,
    link_rate: u32,
    active_lanes: u32,
    drive_settings: [u32; 2],
}

static DPTX_CALLBACK_STATE: IrqSpinLock<Option<DptxCallbackState>> = IrqSpinLock::new(None);

fn dptx_service_call(
    _channel: u32,
    command: u32,
    request: &[u8],
    reply: &mut [u8],
) -> Result<(), &'static str> {
    println!(
        "[apple-dcpext] DPTX callback command={} request={} reply={}",
        command,
        request.len(),
        reply.len()
    );
    let copy_len = request.len().min(reply.len());
    reply[..copy_len].copy_from_slice(&request[..copy_len]);
    let _ = write_le_u32(reply, 0, 0);

    let mut state_guard = DPTX_CALLBACK_STATE.lock();
    let state = state_guard
        .as_mut()
        .ok_or("apple-dcpext: DPTX callback arrived without PHY state")?;

    match command {
        DPTX_APCALL_ACTIVATE
        | DPTX_APCALL_WILL_CHANGE_LINK_CONFIG
        | DPTX_APCALL_SET_DOWN_SPREAD
        | DPTX_APCALL_SET_LANE_MAP
        | DPTX_APCALL_FORCE_HPD => {}
        DPTX_APCALL_DID_CHANGE_LINK_CONFIG => {
            // Apple firmware expects the PHY link configuration to settle before
            // this callback is acknowledged. This matches m1n1's DPTX endpoint.
            time::udelay(100_000);
        }
        DPTX_APCALL_GET_MAX_DRIVE_SETTINGS => {
            write_le_u32(reply, 16, 3);
            write_le_u32(reply, 20, 3);
        }
        DPTX_APCALL_SET_DRIVE_SETTINGS => {
            let first = read_le_u32(request, 32).unwrap_or(0);
            let second = read_le_u32(request, 40).unwrap_or(0);
            state.drive_settings = [first, second];
        }
        DPTX_APCALL_GET_DRIVE_SETTINGS => {
            write_le_u32(reply, 0, state.max_lanes);
            write_le_u32(reply, 32, state.drive_settings[0]);
            write_le_u32(reply, 36, 0);
            write_le_u32(reply, 40, state.drive_settings[1]);
        }
        DPTX_APCALL_GET_MAX_LINK_RATE => {
            write_le_u32(reply, 16, DP_LINK_RATE_HBR3);
        }
        DPTX_APCALL_GET_LINK_RATE => {
            write_le_u32(reply, 16, state.link_rate);
        }
        DPTX_APCALL_SET_LINK_RATE => {
            let requested = read_le_u32(request, 16).unwrap_or(0);
            let mbps = match requested {
                0 => Some(0),
                DP_LINK_RATE_RBR => Some(1620),
                DP_LINK_RATE_HBR => Some(2700),
                DP_LINK_RATE_HBR2 => Some(5400),
                DP_LINK_RATE_HBR3 => Some(8100),
                _ => None,
            };
            let accepted = if let Some(mbps) = mbps {
                match state.phy.lock().configure_dp_link_rate(mbps) {
                    Ok(()) => requested,
                    Err(error) => {
                        println!("[apple-dcpext] failed to set DP link rate: {}", error);
                        write_le_u32(reply, 0, 1);
                        0
                    }
                }
            } else {
                write_le_u32(reply, 0, 1);
                0
            };
            state.link_rate = accepted;
            write_le_u32(reply, 16, accepted);
        }
        DPTX_APCALL_GET_MAX_LANE_COUNT => {
            write_le_u64(reply, 16, state.max_lanes as u64);
        }
        DPTX_APCALL_GET_ACTIVE_LANE_COUNT => {
            write_le_u64(reply, 16, state.active_lanes as u64);
        }
        DPTX_APCALL_SET_ACTIVE_LANE_COUNT => {
            let requested = read_le_u64(request, 16).unwrap_or(0) as u32;
            let accepted = match state.phy.lock().set_active_dp_lane_count(requested) {
                Ok(()) => requested,
                Err(error) => {
                    println!(
                        "[apple-dcpext] rejected DP lane count {}: {}",
                        requested, error
                    );
                    write_le_u32(reply, 0, 1);
                    0
                }
            };
            state.active_lanes = accepted;
            write_le_u64(reply, 16, accepted as u64);
        }
        DPTX_APCALL_GET_SUPPORTS_DOWN_SPREAD
        | DPTX_APCALL_GET_DOWN_SPREAD
        | DPTX_APCALL_GET_SUPPORTS_LANE_MAPPING
        | DPTX_APCALL_GET_SUPPORTS_HPD => {
            write_le_u32(reply, 16, 0);
        }
        DPTX_APCALL_SET_TILED_DISPLAY_HINTS => {
            write_le_u32(reply, 0, 1);
        }
        _ => {
            println!(
                "[apple-dcpext] acknowledging unimplemented DPTX callback {}",
                command
            );
        }
    }
    Ok(())
}

struct DcpIboot {
    endpoint: EpicEndpoint,
    channel: u32,
    firmware_13_3: bool,
}

impl DcpIboot {
    fn poll(&mut self) {
        self.endpoint.poll();
    }

    fn call(
        &mut self,
        operation: u32,
        payload: &[u8],
        expects_output: bool,
    ) -> Result<Vec<u8>, &'static str> {
        let total = 16usize
            .checked_add(payload.len())
            .ok_or("apple-dcpext: iBoot command too large")?;
        let mut command = alloc::vec![0u8; total];
        command[0..4].copy_from_slice(&operation.to_le_bytes());
        command[4..8].copy_from_slice(&(total as u32).to_le_bytes());
        command[16..].copy_from_slice(payload);
        let reply = self
            .endpoint
            .call_raw_by_channel(self.channel, DCP_IBOOT_SUBTYPE, &command)?;
        if reply.len() < 8 {
            return if expects_output {
                Err("apple-dcpext: short iBoot reply")
            } else {
                Ok(Vec::new())
            };
        }
        let reply_len = read_le_u32(&reply, 4).unwrap_or(0) as usize;
        if reply_len < 8 {
            return if expects_output {
                Err("apple-dcpext: invalid iBoot reply length")
            } else {
                Ok(Vec::new())
            };
        }
        Ok(reply[8..reply_len.min(reply.len())].to_vec())
    }

    fn set_power(&mut self, enabled: bool) -> Result<(), &'static str> {
        self.call(IBOOT_SET_POWER, &[enabled as u8], false)?;
        Ok(())
    }

    fn display_status(&mut self) -> Result<(bool, u32, u32), &'static str> {
        let reply = self.call(IBOOT_GET_HPD, &[], true)?;
        if reply.len() < 12 {
            return Err("apple-dcpext: short display status reply");
        }
        Ok((
            reply[0] != 0,
            read_le_u32(&reply, 4).unwrap_or(0),
            read_le_u32(&reply, 8).unwrap_or(0),
        ))
    }

    fn timing_modes(&mut self) -> Result<Vec<DcpTimingMode>, &'static str> {
        let reply = self.call(IBOOT_GET_TIMING_MODES, &[], true)?;
        let count = read_wire::<u32>(&reply, 0).ok_or("apple-dcpext: missing timing count")?;
        let mut modes = Vec::new();
        for index in 0..count as usize {
            modes.push(
                read_wire(&reply, 4 + index * mem::size_of::<DcpTimingMode>())
                    .ok_or("apple-dcpext: truncated timing modes")?,
            );
        }
        Ok(modes)
    }

    fn color_modes(&mut self) -> Result<Vec<DcpColorMode>, &'static str> {
        let reply = self.call(IBOOT_GET_COLOR_MODES, &[], true)?;
        let count = read_wire::<u32>(&reply, 0).ok_or("apple-dcpext: missing color count")?;
        let mut modes = Vec::new();
        for index in 0..count as usize {
            modes.push(
                read_wire(&reply, 4 + index * mem::size_of::<DcpColorMode>())
                    .ok_or("apple-dcpext: truncated color modes")?,
            );
        }
        Ok(modes)
    }

    fn set_mode(
        &mut self,
        timing: &DcpTimingMode,
        color: &DcpColorMode,
    ) -> Result<(), &'static str> {
        let timing_len = mem::size_of::<DcpTimingMode>();
        let mut payload = alloc::vec![0u8; timing_len + mem::size_of::<DcpColorMode>()];
        payload[..timing_len].copy_from_slice(bytes_of(timing));
        payload[timing_len..].copy_from_slice(bytes_of(color));
        self.call(IBOOT_SET_MODE, &payload, false)?;
        Ok(())
    }

    fn set_surface(&mut self, layer: &DcpLayer) -> Result<(), &'static str> {
        self.call(IBOOT_SET_SURFACE, bytes_of(layer), false)?;
        Ok(())
    }

    fn swap_begin(&mut self) -> Result<u32, &'static str> {
        let reply = self.call(IBOOT_SWAP_BEGIN, &[], true)?;
        read_le_u32(&reply, 12).ok_or("apple-dcpext: short swap-begin reply")
    }

    fn swap_set_layer(
        &mut self,
        layer_id: u32,
        layer: &DcpLayer,
        src: &DcpRect,
        dst: &DcpRect,
    ) -> Result<(), &'static str> {
        let extra = if self.firmware_13_3 { 8 } else { 0 };
        let layer_offset = 8;
        let rect_offset = layer_offset + mem::size_of::<DcpLayer>() + extra;
        let mut payload = alloc::vec![0u8; rect_offset + 2 * mem::size_of::<DcpRect>() + 4];
        payload[4..8].copy_from_slice(&layer_id.to_le_bytes());
        payload[layer_offset..layer_offset + mem::size_of::<DcpLayer>()]
            .copy_from_slice(bytes_of(layer));
        payload[rect_offset..rect_offset + mem::size_of::<DcpRect>()]
            .copy_from_slice(bytes_of(src));
        payload
            [rect_offset + mem::size_of::<DcpRect>()..rect_offset + 2 * mem::size_of::<DcpRect>()]
            .copy_from_slice(bytes_of(dst));
        self.call(IBOOT_SWAP_SET_LAYER, &payload, false)?;
        Ok(())
    }

    fn swap_end(&mut self) -> Result<(), &'static str> {
        self.call(IBOOT_SWAP_END, &[0u8; 12], false)?;
        Ok(())
    }
}

fn make_layer(config: &FramebufferConfig, dva: u64) -> DcpLayer {
    let mut layer = DcpLayer::default();
    layer.planes[0].addr = dva;
    layer.planes[0].stride = config.stride;
    layer.planes[0].addr_format = ADDR_FORMAT_PLANAR;
    layer.plane_count = 1;
    layer.width = config.width;
    layer.height = config.height;
    layer.surface_format = SURFACE_FMT_BGRA8888;
    layer.colorspace = COLORSPACE_DISPLAY_P3;
    layer.eotf = EOTF_GAMMA_SDR;
    layer
}

fn choose_timing(modes: &[DcpTimingMode]) -> Option<DcpTimingMode> {
    const MAX_60_HZ: u32 = 60 << 16;
    let valid = |mode: &&DcpTimingMode| {
        mode.valid != 0 && mode.width != 0 && mode.height != 0 && mode.fps <= MAX_60_HZ
    };

    modes
        .iter()
        .filter(valid)
        .copied()
        .filter(|mode| mode.width == 1920 && mode.height == 1080)
        .max_by_key(|mode| mode.fps)
        .or_else(|| {
            modes
                .iter()
                .filter(valid)
                .copied()
                .filter(|mode| mode.width <= 1920 && mode.height <= 1080)
                .max_by_key(|mode| (mode.width as u64 * mode.height as u64, mode.fps))
        })
        .or_else(|| {
            modes
                .iter()
                .filter(valid)
                .copied()
                .max_by_key(|mode| (mode.width as u64 * mode.height as u64, mode.fps))
        })
}

fn choose_color(modes: &[DcpColorMode]) -> Option<DcpColorMode> {
    modes
        .iter()
        .copied()
        .filter(|mode| mode.valid != 0)
        .max_by_key(|mode| (mode.bpp <= 32, mode.bpp))
}

struct MirrorScanouts {
    config: FramebufferConfig,
    dva: [u64; 2],
}

struct AppleDcpExt {
    mode: MirrorMode,
    iboot: DcpIboot,
    iomfb: Iomfb,
    iomfb_powered: bool,
    _system: EpicEndpoint,
    _dptx: EpicEndpoint,
    dcp_table: Arc<IrqSpinLock<DartPageTable>>,
    dcp_dart: Arc<DartInstance>,
    display_table: Arc<IrqSpinLock<DartPageTable>>,
    display_dart: Arc<DartInstance>,
    dva_base: u64,
    page_size: usize,
    scanouts: Option<MirrorScanouts>,
    _rtkit: Arc<AppleRtkit>,
}

impl AppleDcpExt {
    fn configure_scanouts(
        &mut self,
        config: &FramebufferConfig,
        buffers: [(usize, usize); 2],
    ) -> Result<(), &'static str> {
        if self.scanouts.is_some() {
            return Err("apple-dcpext: mirror scanouts are already configured");
        }
        if config.width != self.mode.width
            || config.height != self.mode.height
            || config.format != PixelFormat::BGRA8888
        {
            return Err("apple-dcpext: scanout configuration does not match external mode");
        }

        let mut dva = [0u64; 2];
        let mut next_iova = DCP_SCANOUT_IOVA_BASE;
        for (index, (paddr, size)) in buffers.into_iter().enumerate() {
            if size == 0 || !paddr.is_multiple_of(self.page_size) {
                return Err("apple-dcpext: invalid mirror scanout allocation");
            }
            let mapped_size = size.div_ceil(self.page_size) * self.page_size;
            self.dcp_table
                .lock()
                .map_contiguous(next_iova, paddr, mapped_size, DCP_DART_FLAGS)?;
            self.display_table.lock().map_contiguous(
                next_iova,
                paddr,
                mapped_size,
                DCP_DART_FLAGS,
            )?;
            dva[index] = self.dva_base | next_iova as u64;
            next_iova = next_iova
                .checked_add(mapped_size)
                .ok_or("apple-dcpext: scanout IOVA overflow")?;
        }
        self.dcp_dart
            .sync_page_tables()
            .map_err(|_| "apple-dcpext: scanout DART sync failed")?;
        self.display_dart
            .sync_page_tables()
            .map_err(|_| "apple-dcpext: display scanout DART sync failed")?;
        arch::io_wmb();
        self.iboot.set_surface(&make_layer(config, dva[0]))?;
        if !self.iomfb_powered {
            self.iomfb.power_on()?;
            self.iomfb_powered = true;
            println!("[apple-dcpext] IOMFB display client powered on");
        }
        self.scanouts = Some(MirrorScanouts {
            config: config.clone(),
            dva,
        });
        Ok(())
    }

    fn present(&mut self, index: usize) -> Result<(), &'static str> {
        let scanouts = self
            .scanouts
            .as_ref()
            .ok_or("apple-dcpext: mirror scanouts are not configured")?;
        let dva = *scanouts
            .dva
            .get(index)
            .ok_or("apple-dcpext: invalid mirror scanout index")?;
        let config = scanouts.config.clone();
        let layer = make_layer(&config, dva);
        let rect = DcpRect {
            width: config.width,
            height: config.height,
            x: 0,
            y: 0,
        };
        arch::io_wmb();
        let _swap_id = self.iboot.swap_begin()?;
        self.iboot.swap_set_layer(0, &layer, &rect, &rect)?;
        self.iboot.swap_end()?;
        Ok(())
    }
}

static DCP_EXT: Mutex<Option<AppleDcpExt>> = Mutex::new(None);

/// External timing selected during boot, if a Type-C DP sink was connected.
pub fn mirror_mode() -> Option<MirrorMode> {
    DCP_EXT.lock().as_ref().map(|dcp| dcp.mode)
}

/// Map the internal DCP's two shared scanout allocations into DCPext.
pub fn configure_mirror_scanouts(
    config: &FramebufferConfig,
    buffers: [(usize, usize); 2],
) -> Result<(), &'static str> {
    DCP_EXT
        .lock()
        .as_mut()
        .ok_or("apple-dcpext: no boot-time external display")?
        .configure_scanouts(config, buffers)
}

/// Present one of the shared scanout buffers on the external display.
pub fn present_mirror_buffer(index: usize) -> Result<(), &'static str> {
    DCP_EXT
        .lock()
        .as_mut()
        .ok_or("apple-dcpext: no boot-time external display")?
        .present(index)
}

fn probe_deferred(message: &'static str) -> Result<(), &'static str> {
    println!("[apple-dcpext] {}, deferring", message);
    let result = probe_defer();
    if let Err(error) = result {
        debug_assert!(is_probe_defer(error));
    }
    result
}

fn firmware_is_13_3_or_newer(device: &PlatformDeviceInfo) -> Result<bool, &'static str> {
    let value = device
        .property("apple,firmware-compat")
        .ok_or("apple-dcpext: missing firmware compatibility")?
        .value();
    let major = read_be_u32(value, 0).ok_or("apple-dcpext: invalid firmware compatibility")?;
    let minor = read_be_u32(value, 4).ok_or("apple-dcpext: invalid firmware compatibility")?;
    match (major, minor) {
        (12, 3) => Ok(false),
        (13, 3 | 5) => Ok(true),
        _ => Err("apple-dcpext: unsupported firmware compatibility"),
    }
}

fn dptx_channel(endpoint: &EpicEndpoint, port: u32) -> Result<u32, &'static str> {
    let service_name = match port {
        0 => "dcpdptx-port-epic:0",
        1 => "dcpdptx-port-epic:1",
        _ => return Err("apple-dcpext: invalid DPTX service port"),
    };

    endpoint
        .find_service_by_suffix(service_name)
        .or_else(|| endpoint.find_service(service_name))
        .or_else(|| endpoint.find_service("dcpdptx-port-epic"))
        .or_else(|| endpoint.find_service("AppleDCPDPTXRemotePort"))
        .map(|service| service.channel)
        .or_else(|| endpoint.first_service_channel())
        .ok_or("apple-dcpext: DPTX service not announced")
}

fn wait_for_named_service(
    endpoint: &mut EpicEndpoint,
    name_prefix: &str,
    timeout_us: u64,
) -> Result<(), &'static str> {
    let start = time::current_time();
    loop {
        if endpoint.find_service(name_prefix).is_some() {
            return Ok(());
        }
        if time::current_time().saturating_sub(start) >= timeout_us {
            println!(
                "[apple-dcpext] timed out waiting for service '{}' (announced={:?})",
                name_prefix,
                endpoint.service_names()
            );
            return Err("apple-dcpext: expected EPIC service not announced");
        }
        endpoint.poll();
        for _ in 0..100 {
            core::hint::spin_loop();
        }
    }
}

fn dptx_connect(
    endpoint: &mut EpicEndpoint,
    channel: u32,
    atc_index: u32,
) -> Result<(), &'static str> {
    let target = DPTX_CONNECTED | (atc_index << 4);
    let mut request = [0u8; 8];
    request[4..8].copy_from_slice(&target.to_le_bytes());
    let reply = endpoint.call_by_channel_sized(channel, 0, DPTX_CONNECT, &request, 32, 32)?;
    let reply_unk = read_le_u32(&reply, 0);
    let reply_target = read_le_u32(&reply, 4);
    println!(
        "[apple-dcpext] DPTX connect channel={} target={:#x} reply_unk={:?} reply_target={:?}",
        channel, target, reply_unk, reply_target
    );
    if reply_target != Some(target) {
        println!(
            "[apple-dcpext] DPTX connect mismatch: channel={} target={:#x} reply_unk={:?} reply_target={:?}",
            channel, target, reply_unk, reply_target
        );
        return Err("apple-dcpext: DPTX connect response mismatch");
    }
    // This field is 0 on current Asahi-tested firmware and has also been
    // observed as 0x100. It is not part of the connection identity, so keep
    // it diagnostic rather than rejecting an otherwise valid connection.
    if !matches!(reply_unk, Some(0 | 0x100)) {
        println!(
            "[apple-dcpext] DPTX connect returned unknown field {:?}",
            reply_unk
        );
    }
    let reply = endpoint.call_by_channel_sized(channel, 0, DPTX_REQUEST_DISPLAY, &[], 16, 16)?;
    println!(
        "[apple-dcpext] DPTX request-display reply={:#x}/{:#x}/{:#x}/{:#x}",
        read_le_u32(&reply, 0).unwrap_or(0),
        read_le_u32(&reply, 4).unwrap_or(0),
        read_le_u32(&reply, 8).unwrap_or(0),
        read_le_u32(&reply, 12).unwrap_or(0)
    );
    Ok(())
}

fn dptx_set_hpd(
    endpoint: &mut EpicEndpoint,
    channel: u32,
    enabled: bool,
) -> Result<(), &'static str> {
    let mut request = [0u8; 20];
    request[16..20].copy_from_slice(&(enabled as u32).to_le_bytes());
    let reply = endpoint.call_by_channel_sized(channel, 8, DPTX_SET_HPD, &request, 32, 32)?;
    if read_le_u32(&reply, 16) != Some(enabled as u32) {
        return Err("apple-dcpext: DPTX HPD response mismatch");
    }
    Ok(())
}

fn probe_fn(device: &PlatformDeviceInfo) -> Result<(), &'static str> {
    if DCP_EXT.lock().is_some() {
        return Ok(());
    }

    let preferred_atc_index = device
        .property("apple,dptx-phy")
        .and_then(|property| read_be_u32(property.value(), 0))
        .ok_or("apple-dcpext: missing DPTX PHY index")?;
    if preferred_atc_index >= J293_TYPEC_ROUTES.len() as u32 {
        return Err("apple-dcpext: invalid preferred J293 DPTX PHY index");
    }

    // `apple,dptx-phy` describes Fairydust's preferred boot route, not the
    // only ATC that DCPext can address.  Firmware commonly disables the unused
    // controller, so inspect both stable CD321x addresses and choose whichever
    // one actually negotiated DP Alt Mode.  Keep the DT route first when both
    // ports happen to be connected.
    let candidate_indices = [preferred_atc_index, preferred_atc_index ^ 1];
    let mut saw_controller = false;
    let mut saw_displayport = false;
    let mut first_read_error = None;
    let mut selected = None;
    for candidate_index in candidate_indices {
        let route = J293_TYPEC_ROUTES[candidate_index as usize];
        let typec_status = match get_cd321x_status_by_address(route.cd321x_address) {
            Some(Ok(status)) => {
                saw_controller = true;
                status
            }
            Some(Err(error)) => {
                saw_controller = true;
                first_read_error.get_or_insert(error);
                continue;
            }
            None => continue,
        };
        if !has_displayport_connection(&typec_status) {
            continue;
        }
        saw_displayport = true;
        if !displayport_hpd_level(&typec_status) {
            println!(
                "[apple-dcpext] DP Alt Mode on CD321x {:#x} has HPD low; checking other port",
                route.cd321x_address
            );
            continue;
        }
        let sid_status = match get_cd321x_displayport_status_by_address(route.cd321x_address) {
            Some(Ok(Some(status))) => status,
            Some(Ok(None)) => {
                first_read_error
                    .get_or_insert("apple-dcpext: DisplayPort SID status is unavailable");
                continue;
            }
            Some(Err(error)) => {
                first_read_error.get_or_insert(error);
                continue;
            }
            None => continue,
        };
        selected = Some((route, typec_status, sid_status));
        break;
    }

    let Some((route, typec_status, sid_status)) = selected else {
        if !saw_controller {
            return probe_deferred("Type-C controllers are not ready");
        }
        if let Some(error) = first_read_error {
            return Err(error);
        }
        if saw_displayport {
            return Err("apple-dcpext: DisplayPort sink HPD is low");
        }
        println!("[apple-dcpext] no boot-time DisplayPort sink on either Type-C port");
        return Err("apple-dcpext: no boot-time DisplayPort connection");
    };
    let atc_index = route.atc_index;
    let typec_address = route.cd321x_address;
    let Some(atcphy) = get_atcphy_by_core_paddr(route.atc_core_paddr) else {
        return probe_deferred("selected ATC PHY is not ready");
    };
    // `mux-index` selects a DPXBAR input; it is not a Type-C port number and
    // must not override the CD321x-negotiated pin assignment.  Fairydust's
    // unconditional State C switch is explicitly documented as a temporary
    // 14/16-inch MacBook Pro DP-to-HDMI bridge hack.  J293 Type-C hubs must
    // retain their negotiated State D (two DP lanes plus USB3).
    let mode = match displayport_lane_mode(&typec_status)? {
        Some(Cd321xDisplayPortLaneMode::DisplayPort) => AtcPhyMode::DisplayPort,
        Some(Cd321xDisplayPortLaneMode::Usb3DisplayPort) => AtcPhyMode::Usb3Dp,
        None => return Err("apple-dcpext: DisplayPort pin assignment is unavailable"),
    };
    let reverse = typec_status.orientation == TypecOrientation::Reverse;
    println!(
        "[apple-dcpext] Type-C route addr={:#x} atc={} preferred-atc={} assignment={:?} mode={:?} reversed={} hpd=true sid-status-rx={:#010x} sid-configure={:#010x}",
        typec_address,
        atc_index,
        preferred_atc_index,
        displayport_pin_assignment(&typec_status),
        mode,
        reverse,
        sid_status.status_rx,
        sid_status.configure,
    );
    {
        let mut phy = atcphy.lock();
        phy.configure_displayport(mode, reverse)?;
        route_t8103_dpphy(phy.display_crossbar_paddr())?;
    }

    let dcp_phandle = device_phandle(device).ok_or("apple-dcpext: missing phandle")?;
    let (dart_phandle, dart_stream) =
        iommu_spec(device).ok_or("apple-dcpext: missing DCP IOMMU")?;
    let Some(dcp_dart) = get_dart_by_phandle(dart_phandle) else {
        return probe_deferred("DCPext DART is not ready");
    };
    let root = dcp_dart
        .ttbr_paddr(dart_stream)
        .ok_or("apple-dcpext: DCP DART has no valid TTBR")?;
    let dcp_table = Arc::new(IrqSpinLock::new(DartPageTable::wrap_existing(
        root,
        dcp_dart.page_shift(),
    )?));
    let (display_dart_phandle, display_stream) =
        find_piodma_iommu(dcp_phandle).ok_or("apple-dcpext: PIODMA IOMMU missing")?;
    let Some(display_dart) = get_dart_by_phandle(display_dart_phandle) else {
        return probe_deferred("DCPext display DART is not ready");
    };
    if display_dart.page_shift() != dcp_dart.page_shift() {
        return Err("apple-dcpext: DCP and display DART page sizes differ");
    }
    let display_root = display_dart
        .ttbr_paddr(display_stream)
        .ok_or("apple-dcpext: display DART has no valid TTBR")?;
    let display_table = Arc::new(IrqSpinLock::new(DartPageTable::wrap_existing(
        display_root,
        display_dart.page_shift(),
    )?));
    let piodma_stream_id =
        u32::try_from(display_stream).map_err(|_| "apple-dcpext: PIODMA stream ID out of range")?;
    let piodma_domain = Arc::new(
        DartDomain::wrap_existing(
            Arc::clone(&display_dart),
            IommuStreamId {
                id: piodma_stream_id,
                substream_id: None,
            },
        )
        .map_err(|_| "apple-dcpext: PIODMA firmware page table unavailable")?,
    );
    let handoff_maps = map_handoff_regions(&mut dcp_table.lock(), dcp_phandle)?;
    if handoff_maps == 0 {
        return Err("apple-dcpext: no firmware handoff mappings");
    }
    dcp_dart
        .sync_page_tables()
        .map_err(|_| "apple-dcpext: DCP DART sync failed")?;

    let mailbox_phandle = property_phandle(device, "mboxes")
        .or_else(|| property_phandle(device, "mailboxes"))
        .ok_or("apple-dcpext: missing ASC mailbox")?;
    let Some(asc) = get_apple_asc_by_phandle(mailbox_phandle) else {
        return probe_deferred("DCPext ASC mailbox is not ready");
    };
    let page_size = 1usize << dcp_dart.page_shift();
    let dva_base = device
        .property("apple,asc-dram-mask")
        .or_else(|| device.property("asc-dram-mask"))
        .and_then(|property| read_be_u64(property.value(), 0))
        .unwrap_or(T8103_ASC_DRAM_MASK);
    let mapper = Arc::new(DcpDmaMapper::new(
        Arc::clone(&dcp_table),
        Arc::clone(&dcp_dart),
        page_size,
        dva_base,
    ));
    let rtkit = Arc::new(AppleRtkit::new_with_dma_mapper(asc, mapper));
    rtkit.wake()?;

    let remoteproc: Arc<dyn RemoteProcessor> = rtkit.clone();

    // DCP firmware publishes the system service before its display-facing
    // services.  The previous DCPext implementation retained and polled this
    // endpoint; dropping it during the driver rewrite leaves firmware only
    // partially initialized and stalls DPTX after ACTIVATE.
    rtkit.start_ep(DCP_SYSTEM_EP)?;
    let mut system = EpicEndpoint::new(remoteproc.clone(), DCP_SYSTEM_EP)?;
    wait_for_named_service(&mut system, "system", DCP_SERVICE_TIMEOUT_US)?;

    // Both m1n1 and Asahi Linux bring up the iBoot display service before
    // asking the DPTX port service to request a display.  DCP firmware does
    // not start the link-configuration callback sequence until the display
    // service is available, so waiting for lanes before starting this
    // endpoint deadlocks the negotiation.
    rtkit.start_ep(DCP_IBOOT_EP)?;
    let mut iboot_endpoint = EpicEndpoint::new(remoteproc.clone(), DCP_IBOOT_EP)?;
    wait_for_named_service(&mut iboot_endpoint, "disp0", DCP_SERVICE_TIMEOUT_US)?;
    let iboot_channel = iboot_endpoint
        .find_service("disp0")
        .map(|service| service.channel)
        .or_else(|| iboot_endpoint.first_service_channel())
        .ok_or("apple-dcpext: disp0-service not announced")?;
    let firmware_13_3 = firmware_is_13_3_or_newer(device)?;
    let mut iboot = DcpIboot {
        endpoint: iboot_endpoint,
        channel: iboot_channel,
        firmware_13_3,
    };

    rtkit.start_ep(DCP_DPTX_PORT_EP)?;
    let mut dptx = EpicEndpoint::new(remoteproc.clone(), DCP_DPTX_PORT_EP)?;
    dptx.set_service_call_handler(dptx_service_call);
    dptx.wait_for_services(2, DCP_SERVICE_TIMEOUT_US)?;
    let dptx_channel = dptx_channel(&dptx, DPTX_SERVICE_PORT)?;

    // Type-C hotplug reaches dcp_dptx_connect_oob() only after Asahi's IOMFB
    // endpoint has completed its A401 boot sequence.  Connecting DPTX before
    // this leaves firmware after ACTIVATE with no display server available,
    // so it never asks the AP to set a link rate or active lane count.
    let (registers, bandwidth) = iomfb_registers(device)?;
    let clock_frequency = device_clock_frequency(device);
    let mut iomfb = Iomfb::new(
        Arc::clone(&rtkit),
        registers,
        bandwidth,
        clock_frequency,
        Arc::clone(&piodma_domain),
        !firmware_13_3,
    )?;
    iomfb.start()?;
    println!(
        "[apple-dcpext] EPIC/IOMFB endpoints ready: system={:?} iboot={:?} dptx={:?}",
        system.service_names(),
        iboot.endpoint.service_names(),
        dptx.service_names()
    );
    let max_lanes = atcphy.lock().max_dp_lane_count();
    *DPTX_CALLBACK_STATE.lock() = Some(DptxCallbackState {
        phy: Arc::clone(&atcphy),
        max_lanes,
        link_rate: 0,
        active_lanes: 0,
        drive_settings: [0; 2],
    });
    dptx_connect(&mut dptx, dptx_channel, atc_index)?;

    let link_start = time::current_time();
    loop {
        let active = DPTX_CALLBACK_STATE
            .lock()
            .as_ref()
            .map(|state| state.active_lanes)
            .unwrap_or(0);
        if active != 0 {
            break;
        }
        if time::current_time().saturating_sub(link_start) >= DCP_LINK_TIMEOUT_US {
            let state = DPTX_CALLBACK_STATE.lock();
            let (link_rate, active_lanes) = state
                .as_ref()
                .map(|state| (state.link_rate, state.active_lanes))
                .unwrap_or((0, 0));
            println!(
                "[apple-dcpext] DPTX link timeout: rate={:#x} lanes={}",
                link_rate, active_lanes
            );
            return Err("apple-dcpext: DPTX link negotiation did not select lanes");
        }
        system.poll();
        iboot.poll();
        dptx.poll();
        for _ in 0..100 {
            core::hint::spin_loop();
        }
    }
    dptx_set_hpd(&mut dptx, dptx_channel, true)?;

    iboot.set_power(true)?;
    let mut status = (false, 0, 0);
    for _ in 0..DCP_STATUS_RETRIES {
        status = iboot.display_status()?;
        if status.0 && status.1 != 0 && status.2 != 0 {
            break;
        }
        time::udelay(DCP_STATUS_RETRY_US);
    }
    if !status.0 {
        return Err("apple-dcpext: external display did not assert HPD");
    }

    let timing = choose_timing(&iboot.timing_modes()?)
        .ok_or("apple-dcpext: no usable external timing at or below 60 Hz")?;
    let color =
        choose_color(&iboot.color_modes()?).ok_or("apple-dcpext: no usable external color mode")?;
    iboot.set_mode(&timing, &color)?;
    let selected_mode = MirrorMode {
        width: timing.width,
        height: timing.height,
        fps: timing.fps,
    };

    let negotiated = DPTX_CALLBACK_STATE
        .lock()
        .as_ref()
        .map(|state| (state.link_rate, state.active_lanes))
        .unwrap_or((0, 0));
    println!(
        "[apple-dcpext] external master {}x{} @ {}.{:02} Hz, DP rate={:#x} lanes={}, mode={:?}, reversed={}, handoff maps={}",
        selected_mode.width,
        selected_mode.height,
        selected_mode.fps >> 16,
        ((selected_mode.fps & 0xffff) * 100 + 0x7fff) >> 16,
        negotiated.0,
        negotiated.1,
        mode,
        reverse,
        handoff_maps
    );

    *DCP_EXT.lock() = Some(AppleDcpExt {
        mode: selected_mode,
        iboot,
        iomfb,
        iomfb_powered: false,
        _system: system,
        _dptx: dptx,
        dcp_table,
        dcp_dart,
        display_table,
        display_dart,
        dva_base,
        page_size,
        scanouts: None,
        _rtkit: rtkit,
    });
    Ok(())
}

fn remove_fn(_device: &PlatformDeviceInfo) -> Result<(), &'static str> {
    *DCP_EXT.lock() = None;
    *DPTX_CALLBACK_STATE.lock() = None;
    Ok(())
}

fn register_driver() {
    let driver = PlatformDeviceDriver::new(
        "apple-dcpext",
        probe_fn,
        remove_fn,
        alloc::vec!["apple,t8103-dcpext", "apple,dcpext"],
    );
    // Establish the external timing before the internal DCP chooses its shared
    // scanout canvas.
    DeviceManager::get_manager().register_driver(Box::new(driver), DriverPriority::Standard);
}

scarlet::driver_initcall!(register_driver);

#[used]
static SCARLET_DRIVER_DCPEXT_ANCHOR: fn() = force_link;

#[inline(never)]
pub fn force_link() {}
