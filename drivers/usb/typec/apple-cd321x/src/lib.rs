#![no_std]

extern crate alloc;

use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec::Vec;

use scarlet::{
    device::{
        DeviceInfo,
        i2c::{I2cAddress, I2cBus, I2cError, I2cMessage},
        manager::{DeviceManager, DriverPriority, probe_defer},
        platform::{PlatformDeviceDriver, PlatformDeviceInfo},
        usb::{TypecOrientation, TypecPort, TypecPortStatus, UsbDataRole},
    },
    early_println,
    sync::IrqSpinLock,
    time::udelay,
};

const CD321X_MAX_7BIT_ADDRESS: usize = 0x7f;
const TPS_MAX_LEN: usize = 64;
const I2C_SETTLE_US: u64 = 500;
const TPS_CMD_POLL_US: u64 = 1_000;
const TPS_CMD_TIMEOUT_US: u64 = 1_000_000;

const TPS_REG_VID: u8 = 0x00;
const TPS_REG_MODE: u8 = 0x03;
const TPS_REG_CMD1: u8 = 0x08;
const TPS_REG_DATA1: u8 = 0x09;
const TPS_REG_INT_EVENT1: u8 = 0x14;
const TPS_REG_STATUS: u8 = 0x1a;
const TPS_REG_SYSTEM_POWER_STATE: u8 = 0x20;
const TPS_REG_POWER_STATUS: u8 = 0x3f;
const TPS_REG_DP_SID_STATUS: u8 = 0x58;
const TPS_REG_DATA_STATUS: u8 = 0x5f;

const TPS_SYSTEM_POWER_STATE_S0: u8 = 0;
const TPS_INVALID_CMD: u32 = u32::from_le_bytes(*b"!CMD");
const TPS_TASK_TIMEOUT: u8 = 1;
const TPS_TASK_REJECTED: u8 = 3;

const TPS_STATUS_PLUG_PRESENT: u32 = 1 << 0;
const TPS_STATUS_PLUG_UPSIDE_DOWN: u32 = 1 << 4;
const TPS_DATA_STATUS_USB2_CONNECTION: u32 = 1 << 4;
const TPS_DATA_STATUS_USB3_CONNECTION: u32 = 1 << 5;
const TPS_DATA_STATUS_USB_DATA_ROLE: u32 = 1 << 7;
const TPS_DATA_STATUS_DP_CONNECTION: u32 = 1 << 8;
const TPS_DATA_STATUS_DP_PIN_ASSIGNMENT_MASK: u32 = 0x3 << 10;
const CD321X_DATA_STATUS_HPD_IRQ: u32 = 1 << 14;
const CD321X_DATA_STATUS_HPD_LEVEL: u32 = 1 << 15;

const TPS_DP_ASSIGNMENT_E: u32 = 0;
const TPS_DP_ASSIGNMENT_F: u32 = 1;
const TPS_DP_ASSIGNMENT_C: u32 = 2;
const TPS_DP_ASSIGNMENT_D: u32 = 3;
const TPS_DP_ASSIGNMENT_A: u32 = 4;
const TPS_DP_ASSIGNMENT_B: u32 = 6;

/// DisplayPort pin assignment reported by the CD321x DATA_STATUS register.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cd321xDisplayPortPinAssignment {
    A,
    B,
    C,
    D,
    E,
    F,
}

/// ATC lane layout selected by a negotiated DisplayPort pin assignment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cd321xDisplayPortLaneMode {
    /// Four DisplayPort lanes; SuperSpeed USB is unavailable.
    DisplayPort,
    /// Two DisplayPort lanes alongside two SuperSpeed USB lanes.
    Usb3DisplayPort,
}

/// DisplayPort SID status VDOs read from CD321x register 0x58.
#[derive(Debug, Clone, Copy)]
pub struct Cd321xDisplayPortStatus {
    pub mode_status: u8,
    pub status_tx: u32,
    pub status_rx: u32,
    pub configure: u32,
    pub mode_data: u32,
}

/// Read-only controller state used to diagnose Type-C/DisplayPort handoff.
#[derive(Debug, Clone, Copy)]
pub struct Cd321xDiagnosticSnapshot {
    pub interrupt_event1: u64,
    pub status: TypecPortStatus,
    pub displayport_status: Option<Cd321xDisplayPortStatus>,
}

struct AppleCd321x {
    bus: Arc<dyn I2cBus>,
    address: I2cAddress,
    bus_phandle: u32,
}

#[derive(Debug, Clone, Copy)]
struct Cd321xSnapshot {
    vendor_id: u32,
    mode: [u8; 4],
    status: u32,
    power_status: u32,
    data_status: u32,
    displayport_status: Option<Cd321xDisplayPortStatus>,
}

impl AppleCd321x {
    fn new(bus: Arc<dyn I2cBus>, address: I2cAddress, bus_phandle: u32) -> Self {
        Self {
            bus,
            address,
            bus_phandle,
        }
    }

    fn read_exact<const N: usize>(&self, register: u8) -> Result<[u8; N], I2cError> {
        if N > TPS_MAX_LEN {
            return Err(I2cError::InvalidArg);
        }

        let mut messages = alloc::vec![
            I2cMessage::write(self.address, &[register], false),
            I2cMessage::read(self.address, N + 1, true),
        ];
        self.bus.transfer(&mut messages)?;
        udelay(I2C_SETTLE_US);

        let data = messages[1].data.as_slice();
        let declared_len = *data.first().ok_or(I2cError::BusError)?;
        if usize::from(declared_len) < N {
            return Err(I2cError::BusError);
        }

        let mut out = [0u8; N];
        for (index, byte) in out.iter_mut().enumerate() {
            *byte = *data.get(index + 1).ok_or(I2cError::BusError)?;
        }
        Ok(out)
    }

    fn read_u32(&self, register: u8) -> Result<u32, I2cError> {
        Ok(u32::from_le_bytes(self.read_exact::<4>(register)?))
    }

    fn read_u8(&self, register: u8) -> Result<u8, I2cError> {
        Ok(self.read_exact::<1>(register)?[0])
    }

    fn write_block(&self, register: u8, payload: &[u8]) -> Result<(), I2cError> {
        if payload.len() > TPS_MAX_LEN {
            return Err(I2cError::InvalidArg);
        }

        let mut frame = [0u8; TPS_MAX_LEN + 2];
        frame[0] = register;
        frame[1] = u8::try_from(payload.len()).map_err(|_| I2cError::InvalidArg)?;
        frame[2..payload.len() + 2].copy_from_slice(payload);
        let message = I2cMessage::write(self.address, &frame[..payload.len() + 2], true);
        self.bus.transfer(&mut [message])?;
        udelay(I2C_SETTLE_US);
        Ok(())
    }

    fn execute_command(&self, command: [u8; 4], input: &[u8]) -> Result<(), I2cError> {
        let active_command = self.read_u32(TPS_REG_CMD1)?;
        if active_command != 0 && active_command != TPS_INVALID_CMD {
            return Err(I2cError::BusError);
        }

        if !input.is_empty() {
            self.write_block(TPS_REG_DATA1, input)?;
        }
        self.write_block(TPS_REG_CMD1, &command)?;

        let mut remaining = TPS_CMD_TIMEOUT_US;
        while remaining != 0 {
            let status = self.read_u32(TPS_REG_CMD1)?;
            if status == TPS_INVALID_CMD {
                return Err(I2cError::InvalidArg);
            }
            if status == 0 {
                return match self.read_u8(TPS_REG_DATA1)? {
                    TPS_TASK_TIMEOUT => Err(I2cError::Timeout),
                    TPS_TASK_REJECTED => Err(I2cError::BusError),
                    _ => Ok(()),
                };
            }
            udelay(TPS_CMD_POLL_US);
            remaining = remaining.saturating_sub(TPS_CMD_POLL_US);
        }

        Err(I2cError::Timeout)
    }

    fn switch_to_s0(&self) -> Result<bool, I2cError> {
        if self.read_u8(TPS_REG_SYSTEM_POWER_STATE)? == TPS_SYSTEM_POWER_STATE_S0 {
            return Ok(false);
        }

        self.execute_command(*b"SSPS", &[TPS_SYSTEM_POWER_STATE_S0])?;
        if self.read_u8(TPS_REG_SYSTEM_POWER_STATE)? != TPS_SYSTEM_POWER_STATE_S0 {
            return Err(I2cError::BusError);
        }
        Ok(true)
    }

    fn snapshot(&self) -> Result<Cd321xSnapshot, I2cError> {
        let data_status = self.read_u32(TPS_REG_DATA_STATUS)?;
        let displayport_status = if data_status & TPS_DATA_STATUS_DP_CONNECTION != 0 {
            let raw = self.read_exact::<17>(TPS_REG_DP_SID_STATUS)?;
            Some(Cd321xDisplayPortStatus {
                mode_status: raw[0],
                status_tx: u32::from_le_bytes(raw[1..5].try_into().unwrap_or([0; 4])),
                status_rx: u32::from_le_bytes(raw[5..9].try_into().unwrap_or([0; 4])),
                configure: u32::from_le_bytes(raw[9..13].try_into().unwrap_or([0; 4])),
                mode_data: u32::from_le_bytes(raw[13..17].try_into().unwrap_or([0; 4])),
            })
        } else {
            None
        };

        Ok(Cd321xSnapshot {
            vendor_id: self.read_u32(TPS_REG_VID)?,
            mode: self.read_exact::<4>(TPS_REG_MODE)?,
            status: self.read_u32(TPS_REG_STATUS)?,
            power_status: self.read_u32(TPS_REG_POWER_STATUS)?,
            data_status,
            displayport_status,
        })
    }

    fn status_from_snapshot(snapshot: Cd321xSnapshot) -> TypecPortStatus {
        let connected = snapshot.status & TPS_STATUS_PLUG_PRESENT != 0;
        let usb2 = snapshot.data_status & TPS_DATA_STATUS_USB2_CONNECTION != 0;
        let usb3 = snapshot.data_status & TPS_DATA_STATUS_USB3_CONNECTION != 0;
        let data_role = if usb2 || usb3 {
            if snapshot.data_status & TPS_DATA_STATUS_USB_DATA_ROLE != 0 {
                UsbDataRole::Device
            } else {
                UsbDataRole::Host
            }
        } else {
            UsbDataRole::None
        };
        let orientation = if !connected {
            TypecOrientation::None
        } else if snapshot.status & TPS_STATUS_PLUG_UPSIDE_DOWN != 0 {
            TypecOrientation::Reverse
        } else {
            TypecOrientation::Normal
        };

        TypecPortStatus {
            connected,
            usb2,
            usb3,
            data_role,
            orientation,
            raw_status: snapshot.status,
            raw_power_status: snapshot.power_status,
            raw_data_status: snapshot.data_status,
        }
    }
}

impl TypecPort for AppleCd321x {
    fn name(&self) -> &'static str {
        "apple-cd321x"
    }

    fn status(&self) -> Result<TypecPortStatus, &'static str> {
        self.snapshot()
            .map(Self::status_from_snapshot)
            .map_err(|_| "apple-cd321x: failed to read status")
    }
}

fn printable_ascii(byte: u8) -> char {
    if byte.is_ascii_graphic() || byte == b' ' {
        char::from(byte)
    } else {
        '.'
    }
}

fn read_i2c_address(device: &PlatformDeviceInfo) -> Result<I2cAddress, &'static str> {
    let address = device
        .property("reg")
        .and_then(|property| property.as_usize())
        .ok_or("apple-cd321x: missing I2C address")?;
    if address > CD321X_MAX_7BIT_ADDRESS {
        return Err("apple-cd321x: unsupported I2C address");
    }

    Ok(I2cAddress::SevenBit(
        u8::try_from(address).map_err(|_| "apple-cd321x: unsupported I2C address")?,
    ))
}

fn resolve_i2c_bus(device: &PlatformDeviceInfo) -> Result<(u32, Arc<dyn I2cBus>), &'static str> {
    let bus_phandle = device
        .parent_phandle()
        .ok_or("apple-cd321x: missing parent I2C bus")?;
    match DeviceManager::get_manager().get_i2c_bus(bus_phandle) {
        Some(bus) => Ok((bus_phandle, bus)),
        None => {
            early_println!(
                "[apple-cd321x] I2C bus phandle {:#x} is not ready, deferring",
                bus_phandle
            );
            probe_defer()
        }
    }
}

fn probe_fn(device: &PlatformDeviceInfo) -> Result<(), &'static str> {
    let (bus_phandle, bus) = resolve_i2c_bus(device)?;
    let address = read_i2c_address(device)?;
    let controller = Arc::new(AppleCd321x::new(bus, address, bus_phandle));
    let power_state_changed = controller
        .switch_to_s0()
        .map_err(|_| "apple-cd321x: failed to switch to S0")?;
    if power_state_changed {
        early_println!(
            "[apple-cd321x] {} switched system power state to S0",
            device.name()
        );
    }
    let snapshot = controller.snapshot().map_err(|_| {
        early_println!(
            "[apple-cd321x] failed to read status for {} bus-phandle={:#x} addr={:#x}",
            device.name(),
            bus_phandle,
            address.raw(),
        );
        "apple-cd321x: failed to read status"
    })?;

    early_println!(
        "[apple-cd321x] registered {} bus-phandle={:#x} addr={:#x} vid=0x{:08x} mode={}{}{}{} status=0x{:08x} power=0x{:08x} data=0x{:08x}",
        device.name(),
        controller.bus_phandle,
        controller.address.raw(),
        snapshot.vendor_id,
        printable_ascii(snapshot.mode[0]),
        printable_ascii(snapshot.mode[1]),
        printable_ascii(snapshot.mode[2]),
        printable_ascii(snapshot.mode[3]),
        snapshot.status,
        snapshot.power_status,
        snapshot.data_status,
    );
    if let Some(displayport) = snapshot.displayport_status {
        let port_status = AppleCd321x::status_from_snapshot(snapshot);
        early_println!(
            "[apple-cd321x] DisplayPort assignment={:?} hpd-level={} hpd-irq={} sid-mode={:#x} status-tx={:#010x} status-rx={:#010x} configure={:#010x} mode-data={:#010x}",
            displayport_pin_assignment(&port_status),
            displayport_hpd_level(&port_status),
            displayport_hpd_irq(&port_status),
            displayport.mode_status,
            displayport.status_tx,
            displayport.status_rx,
            displayport.configure,
            displayport.mode_data,
        );
    }

    let manager = DeviceManager::get_manager();
    for endpoint in manager.endpoint_phandles_for_platform_device(device) {
        let port: Arc<dyn TypecPort> = controller.clone();
        manager.register_typec_port_endpoint(endpoint, port);
        early_println!(
            "[apple-cd321x] endpoint {:#x} mapped to {} addr={:#x}",
            endpoint,
            device.name(),
            controller.address.raw(),
        );
    }

    APPLE_CD321X.lock().push(controller);
    Ok(())
}

fn remove_fn(_device: &PlatformDeviceInfo) -> Result<(), &'static str> {
    Ok(())
}

fn register_driver() {
    let driver = PlatformDeviceDriver::new(
        "apple-cd321x",
        probe_fn,
        remove_fn,
        alloc::vec!["apple,cd321x"],
    );

    DeviceManager::get_manager().register_driver(Box::new(driver), DriverPriority::Core);
}

static APPLE_CD321X: IrqSpinLock<Vec<Arc<AppleCd321x>>> = IrqSpinLock::new(Vec::new());

/// Return the current status of a CD321x controller in firmware probe order.
///
/// Local indices only describe the set of controllers firmware left enabled;
/// they are not stable hardware port numbers.
pub fn get_cd321x_status(index: usize) -> Option<Result<TypecPortStatus, &'static str>> {
    let controller = APPLE_CD321X.lock().get(index).cloned()?;
    Some(controller.status())
}

/// Return the current status of the CD321x at a specific I2C address.
///
/// Unlike [`get_cd321x_status`], this lookup is stable when firmware disables
/// an unused Type-C port and the remaining controller is registered at a
/// different local index.
pub fn get_cd321x_status_by_address(address: u16) -> Option<Result<TypecPortStatus, &'static str>> {
    let controller = APPLE_CD321X
        .lock()
        .iter()
        .find(|controller| controller.address.raw() == address)
        .cloned()?;
    Some(controller.status())
}

/// Whether a CD321x status snapshot reports an active DisplayPort alt mode.
pub fn has_displayport_connection(status: &TypecPortStatus) -> bool {
    status.connected && status.raw_data_status & TPS_DATA_STATUS_DP_CONNECTION != 0
}

/// Decode the DisplayPort pin assignment negotiated by the CD321x.
pub fn displayport_pin_assignment(
    status: &TypecPortStatus,
) -> Option<Cd321xDisplayPortPinAssignment> {
    if !has_displayport_connection(status) {
        return None;
    }

    // CD321x stores two assignment bits in DATA_STATUS[11:10].  The DP spec
    // assignment encoding appends the USB3-present bit as its least
    // significant bit, matching Fairydust's
    // TPS_DATA_STATUS_DP_SPEC_PIN_ASSIGNMENT().
    let encoded = ((status.raw_data_status & TPS_DATA_STATUS_DP_PIN_ASSIGNMENT_MASK) >> 9)
        | u32::from(status.raw_data_status & TPS_DATA_STATUS_USB3_CONNECTION != 0);
    match encoded {
        TPS_DP_ASSIGNMENT_A => Some(Cd321xDisplayPortPinAssignment::A),
        TPS_DP_ASSIGNMENT_B => Some(Cd321xDisplayPortPinAssignment::B),
        TPS_DP_ASSIGNMENT_C => Some(Cd321xDisplayPortPinAssignment::C),
        TPS_DP_ASSIGNMENT_D => Some(Cd321xDisplayPortPinAssignment::D),
        TPS_DP_ASSIGNMENT_E => Some(Cd321xDisplayPortPinAssignment::E),
        TPS_DP_ASSIGNMENT_F => Some(Cd321xDisplayPortPinAssignment::F),
        _ => None,
    }
}

/// Convert the negotiated pin assignment to the lane modes supported by the
/// Apple ATC PHY driver.
pub fn displayport_lane_mode(
    status: &TypecPortStatus,
) -> Result<Option<Cd321xDisplayPortLaneMode>, &'static str> {
    let Some(assignment) = displayport_pin_assignment(status) else {
        return if has_displayport_connection(status) {
            Err("apple-cd321x: invalid DisplayPort pin assignment")
        } else {
            Ok(None)
        };
    };

    match assignment {
        Cd321xDisplayPortPinAssignment::C | Cd321xDisplayPortPinAssignment::E => {
            Ok(Some(Cd321xDisplayPortLaneMode::DisplayPort))
        }
        Cd321xDisplayPortPinAssignment::D => Ok(Some(Cd321xDisplayPortLaneMode::Usb3DisplayPort)),
        Cd321xDisplayPortPinAssignment::A
        | Cd321xDisplayPortPinAssignment::B
        | Cd321xDisplayPortPinAssignment::F => {
            Err("apple-cd321x: unsupported DisplayPort pin assignment")
        }
    }
}

/// Whether the CD321x reports the DisplayPort sink's HPD level as asserted.
pub fn displayport_hpd_level(status: &TypecPortStatus) -> bool {
    has_displayport_connection(status) && status.raw_data_status & CD321X_DATA_STATUS_HPD_LEVEL != 0
}

/// Whether the CD321x reports a pending DisplayPort HPD IRQ pulse.
pub fn displayport_hpd_irq(status: &TypecPortStatus) -> bool {
    has_displayport_connection(status) && status.raw_data_status & CD321X_DATA_STATUS_HPD_IRQ != 0
}

/// Read the DisplayPort SID status for a CD321x at a stable I2C address.
pub fn get_cd321x_displayport_status_by_address(
    address: u16,
) -> Option<Result<Option<Cd321xDisplayPortStatus>, &'static str>> {
    let controller = APPLE_CD321X
        .lock()
        .iter()
        .find(|controller| controller.address.raw() == address)
        .cloned()?;
    Some(
        controller
            .snapshot()
            .map(|snapshot| snapshot.displayport_status)
            .map_err(|_| "apple-cd321x: failed to read DisplayPort SID status"),
    )
}

/// Read the latched interrupt event and current Type-C/DP state without
/// acknowledging or clearing any CD321x interrupt bits.
pub fn get_cd321x_diagnostic_snapshot_by_address(
    address: u16,
) -> Option<Result<Cd321xDiagnosticSnapshot, &'static str>> {
    let controller = APPLE_CD321X
        .lock()
        .iter()
        .find(|controller| controller.address.raw() == address)
        .cloned()?;
    Some((|| {
        let interrupt_event1 = u64::from_le_bytes(
            controller
                .read_exact::<8>(TPS_REG_INT_EVENT1)
                .map_err(|_| "apple-cd321x: failed to read INT_EVENT1")?,
        );
        let snapshot = controller
            .snapshot()
            .map_err(|_| "apple-cd321x: failed to read diagnostic state")?;
        Ok(Cd321xDiagnosticSnapshot {
            interrupt_event1,
            status: AppleCd321x::status_from_snapshot(snapshot),
            displayport_status: snapshot.displayport_status,
        })
    })())
}

scarlet::driver_initcall!(register_driver);

#[used]
static SCARLET_DRIVER_APPLE_CD321X_ANCHOR: fn() = force_link;

/// Keep the external driver object linked into Scarlet module builds.
#[inline(never)]
pub fn force_link() {}
