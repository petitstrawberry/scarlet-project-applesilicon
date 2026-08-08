//! DisplayPort audio control carried by DCP's AV EPIC endpoint.
//!
//! The RPC layout and firmware command tables follow Asahi Linux's
//! `drivers/gpu/drm/apple/av.c`, while the serialized sound-element parser
//! follows `drivers/gpu/drm/apple/parser.c`.

use alloc::sync::Arc;
use alloc::vec::Vec;

use scarlet::device::remoteproc::RemoteProcessor;
use scarlet::println;
use scarlet::time;
use scarlet_driver_apple_epic::EpicEndpoint;
use scarlet_driver_apple_rtkit::AppleRtkit;

mod parser;

pub use parser::DcpAvAudioCookie;
use parser::select_cookie;

const DCP_AV_EP: u8 = 0x29;
const DCP_AV_SERVICE: &str = "dcpav-audio-interface-epic";
const DCP_AV_SERVICE_TIMEOUT_US: u64 = 5_000_000;

// EpicEndpoint has one 0x4000-byte command buffer. A standard service call
// consumes 0x40 bytes, and the AV OSObject reply header consumes another
// 0x30 bytes, leaving this much room for the serialized object itself.
const AV_ELEMENTS_MAX_SIZE: usize = 0x3f90;
const AV_OSOBJECT_HEADER_SIZE: usize = 48;
const AV_LINK_CALL_SIZE: usize = 64;

#[derive(Clone, Copy)]
struct DcpAvAudioCommands {
    open: u32,
    close: u32,
    prepare: u32,
    start_link: u32,
    stop_link: u32,
    unprepare: u32,
    get_elements: u32,
}

const DCP_AV_AUDIO_COMMANDS_V12_3: DcpAvAudioCommands = DcpAvAudioCommands {
    open: 6,
    close: 7,
    prepare: 8,
    start_link: 9,
    stop_link: 12,
    unprepare: 13,
    get_elements: 18,
};

const DCP_AV_AUDIO_COMMANDS_V13_5: DcpAvAudioCommands = DcpAvAudioCommands {
    open: 4,
    close: 5,
    prepare: 6,
    start_link: 7,
    stop_link: 10,
    unprepare: 11,
    get_elements: 16,
};

pub(crate) struct DcpAvAudio {
    endpoint: EpicEndpoint,
    channel: u32,
    commands: DcpAvAudioCommands,
    elements: Vec<u8>,
    opened: bool,
    prepared: bool,
    started: bool,
}

impl DcpAvAudio {
    pub(crate) fn start(
        rtkit: &Arc<AppleRtkit>,
        remoteproc: Arc<dyn RemoteProcessor>,
        firmware_13_3_or_newer: bool,
    ) -> Result<Self, &'static str> {
        rtkit
            .start_ep(DCP_AV_EP)
            .map_err(|_| "apple-dcpext: failed to start AV RTKit endpoint")?;
        let mut endpoint = EpicEndpoint::new(remoteproc, DCP_AV_EP)?;
        let channel = wait_for_audio_service(&mut endpoint)?;
        let commands = if firmware_13_3_or_newer {
            DCP_AV_AUDIO_COMMANDS_V13_5
        } else {
            DCP_AV_AUDIO_COMMANDS_V12_3
        };

        // Opening publishes the current connection to the AV audio service.
        endpoint.call_by_channel_sized(channel, 0, commands.open, &[], 32, 32)?;

        let elements = match get_osobject(
            &mut endpoint,
            channel,
            1,
            commands.get_elements,
            AV_ELEMENTS_MAX_SIZE,
        ) {
            Ok(elements) => elements,
            Err(error) => {
                let _ = endpoint.call_by_channel_sized(channel, 0, commands.close, &[], 16, 16);
                return Err(error);
            }
        };

        // Parse a conservative concrete mode now rather than advertising an
        // endpoint that will fail only when userspace first opens it.
        let preferred_bits = if select_cookie(&elements, 48_000, 16, 2).is_ok() {
            16
        } else if select_cookie(&elements, 48_000, 32, 2).is_ok() {
            32
        } else {
            let _ = endpoint.call_by_channel_sized(channel, 0, commands.close, &[], 16, 16);
            return Err("apple-dcpext: sink has no 48 kHz stereo S16/S32 audio mode");
        };

        println!(
            "[apple-dcpext] DisplayPort audio ready: 48000 Hz, 2 channels, {}-bit",
            preferred_bits
        );
        Ok(Self {
            endpoint,
            channel,
            commands,
            elements,
            opened: true,
            prepared: false,
            started: false,
        })
    }

    pub(crate) fn cookie(
        &self,
        rate_hz: u32,
        sample_bits: u32,
        channels: u32,
    ) -> Result<DcpAvAudioCookie, &'static str> {
        if !self.opened {
            return Err("apple-dcpext: DisplayPort audio service is closed");
        }
        select_cookie(&self.elements, rate_hz, sample_bits, channels)
    }

    pub(crate) fn supports(&self, rate_hz: u32, sample_bits: u32, channels: u32) -> bool {
        self.cookie(rate_hz, sample_bits, channels).is_ok()
    }

    pub(crate) fn prepare(&mut self, cookie: &DcpAvAudioCookie) -> Result<(), &'static str> {
        if self.started {
            return Err("apple-dcpext: cannot prepare a running audio link");
        }
        self.endpoint.call_by_channel_sized(
            self.channel,
            0,
            self.commands.prepare,
            cookie,
            AV_LINK_CALL_SIZE,
            AV_LINK_CALL_SIZE,
        )?;
        self.prepared = true;
        Ok(())
    }

    pub(crate) fn start_link(&mut self, cookie: &DcpAvAudioCookie) -> Result<(), &'static str> {
        if !self.prepared {
            return Err("apple-dcpext: audio link has not been prepared");
        }
        if self.started {
            return Ok(());
        }
        self.endpoint.call_by_channel_sized(
            self.channel,
            0,
            self.commands.start_link,
            cookie,
            AV_LINK_CALL_SIZE,
            AV_LINK_CALL_SIZE,
        )?;
        self.started = true;
        Ok(())
    }

    pub(crate) fn stop_link(&mut self) -> Result<(), &'static str> {
        if !self.started {
            return Ok(());
        }
        self.endpoint.call_by_channel_sized(
            self.channel,
            0,
            self.commands.stop_link,
            &[],
            AV_LINK_CALL_SIZE,
            AV_LINK_CALL_SIZE,
        )?;
        self.started = false;
        Ok(())
    }

    pub(crate) fn unprepare(&mut self) -> Result<(), &'static str> {
        if self.started {
            return Err("apple-dcpext: stop audio before unpreparing it");
        }
        if !self.prepared {
            return Ok(());
        }
        self.endpoint.call_by_channel_sized(
            self.channel,
            0,
            self.commands.unprepare,
            &[],
            AV_LINK_CALL_SIZE,
            AV_LINK_CALL_SIZE,
        )?;
        self.prepared = false;
        Ok(())
    }

    fn shutdown(&mut self) {
        if self.started {
            if self
                .endpoint
                .call_by_channel_sized(
                    self.channel,
                    0,
                    self.commands.stop_link,
                    &[],
                    AV_LINK_CALL_SIZE,
                    AV_LINK_CALL_SIZE,
                )
                .is_err()
            {
                println!("[apple-dcpext] failed to stop DisplayPort audio during shutdown");
            }
            self.started = false;
        }

        if self.prepared {
            if self
                .endpoint
                .call_by_channel_sized(
                    self.channel,
                    0,
                    self.commands.unprepare,
                    &[],
                    AV_LINK_CALL_SIZE,
                    AV_LINK_CALL_SIZE,
                )
                .is_err()
            {
                println!("[apple-dcpext] failed to unprepare DisplayPort audio during shutdown");
            }
            self.prepared = false;
        }

        if self.opened {
            if self
                .endpoint
                .call_by_channel_sized(self.channel, 0, self.commands.close, &[], 16, 16)
                .is_err()
            {
                println!("[apple-dcpext] failed to close DisplayPort audio during shutdown");
            }
            self.opened = false;
        }
    }
}

impl Drop for DcpAvAudio {
    fn drop(&mut self) {
        self.shutdown();
    }
}

fn wait_for_audio_service(endpoint: &mut EpicEndpoint) -> Result<u32, &'static str> {
    let start = time::current_time();
    loop {
        if let Some(service) = endpoint
            .find_service(DCP_AV_SERVICE)
            .or_else(|| endpoint.find_service_by_suffix(DCP_AV_SERVICE))
            .or_else(|| endpoint.find_service("DCPAVAudioInterface"))
        {
            return Ok(service.channel);
        }
        if time::current_time().saturating_sub(start) >= DCP_AV_SERVICE_TIMEOUT_US {
            println!(
                "[apple-dcpext] timed out waiting for DP audio service (announced={:?})",
                endpoint.service_names()
            );
            return Err("apple-dcpext: DCP AV audio service not announced");
        }
        endpoint.poll();
        for _ in 0..100 {
            core::hint::spin_loop();
        }
    }
}

fn get_osobject(
    endpoint: &mut EpicEndpoint,
    channel: u32,
    group: u16,
    command: u32,
    output_max_size: usize,
) -> Result<Vec<u8>, &'static str> {
    let transfer_size = AV_OSOBJECT_HEADER_SIZE
        .checked_add(output_max_size)
        .ok_or("apple-dcpext: AV OSObject request overflow")?;
    let mut header = [0u8; AV_OSOBJECT_HEADER_SIZE];
    header[0..8].copy_from_slice(&(output_max_size as u64).to_le_bytes());
    let reply = endpoint.call_by_channel_sized(
        channel,
        group,
        command,
        &header,
        transfer_size,
        transfer_size,
    )?;
    if reply.len() < AV_OSOBJECT_HEADER_SIZE {
        return Err("apple-dcpext: short AV OSObject reply");
    }
    let used_size = u64::from_le_bytes(
        reply[32..40]
            .try_into()
            .map_err(|_| "apple-dcpext: invalid AV OSObject size")?,
    ) as usize;
    if used_size < 4 || used_size > output_max_size {
        return Err("apple-dcpext: invalid AV OSObject used size");
    }
    let end = AV_OSOBJECT_HEADER_SIZE
        .checked_add(used_size)
        .ok_or("apple-dcpext: AV OSObject reply overflow")?;
    let object = reply
        .get(AV_OSOBJECT_HEADER_SIZE..end)
        .ok_or("apple-dcpext: truncated AV OSObject reply")?;
    Ok(object.to_vec())
}
