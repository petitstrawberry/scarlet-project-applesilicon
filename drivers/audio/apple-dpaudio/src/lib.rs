#![no_std]

//! Apple DisplayPort audio playback frontend.
//!
//! PCM is copied from Scarlet's userspace-visible audio ring into a dedicated,
//! DART-granule-aligned bounce ring before it is submitted to the SIO DMA
//! engine. Link setup and teardown are delegated to the external DCP's AV
//! audio service.
//!
//! # Provenance
//!
//! The DPA DMA endpoint configuration and DCP AV link sequencing were
//! implemented with reference to Asahi Linux's
//! `drivers/gpu/drm/apple/audio.c`. See the repository `ATTRIBUTION.md`.

extern crate alloc;

use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec::Vec;

use scarlet::{
    arch,
    device::{
        DeviceInfo,
        audio::{
            AUDIO_DEVICE_KIND_UNKNOWN, AUDIO_PCM_FORMAT_S16LE, AUDIO_PCM_FORMAT_S32LE,
            AUDIO_PCM_MAX_RATES, AudioCompletionCallback, AudioDeviceInfo, AudioPcmBuffer,
            AudioPcmCapabilities, AudioPcmParams, AudioPcmPeriod, AudioPlaybackDevice,
            register_playback_device_with_info,
        },
        dma::{
            DmaBusWidth, DmaChannel, DmaController, DmaCyclicConfig, DmaDirection, DmaError,
            DmaPeripheralConfig, DmaSpec,
        },
        manager::{DeviceManager, DriverPriority, probe_defer},
        platform::{
            PlatformDeviceDriver, PlatformDeviceInfo, resource::PlatformDeviceResourceType,
        },
    },
    environment::PAGE_SIZE,
    mem::page::ContiguousPages,
    println,
    sync::{IrqGuard, IrqSpinLock},
};

use scarlet_driver_dcpext::{
    DcpAvAudioCookie, external_audio_available, external_audio_cookie,
    external_audio_format_supported, external_audio_prepare, external_audio_start,
    external_audio_stop, external_audio_unprepare,
};

const DPA_DMA_GRANULE: usize = 0x4000;
const DPA_RATE_HZ: u32 = 48_000;
const DPA_CHANNELS: u16 = 2;
const DPA_MIN_PERIOD_FRAMES: u32 = 64;
const DPA_MAX_PERIOD_FRAMES: u32 = 4_096;
const DPA_MIN_BUFFER_FRAMES: u32 = 128;
const DPA_MAX_BUFFER_FRAMES: u32 = 65_536;
const DPA_MAX_PERIODS: usize = 64;
const DPA_DMA_PERIPHERAL_ID: usize = 1;
const DPA_DMA_BURST_LEN: usize = 2;

struct AppleDpAudioDma {
    controller: Arc<dyn DmaController>,
    spec: DmaSpec,
}

struct AppleDpAudioStream {
    // Keep the channel before the allocation: struct fields drop in declaration
    // order, so the SIO/IOMMU mapping is released before its pages are freed.
    channel: Arc<dyn DmaChannel>,
    source_vaddr: usize,
    source_bytes: usize,
    bounce: ContiguousPages,
    dma_bytes: usize,
    period_bytes: usize,
    period_count: usize,
    in_flight_periods: usize,
    cookie: DcpAvAudioCookie,
    dma_running: bool,
    link_started: bool,
}

impl AppleDpAudioStream {
    fn queue_period(&mut self, period: AudioPcmPeriod) -> Result<(), &'static str> {
        if period.byte_len != self.period_bytes {
            return Err("apple-dpaudio: PCM period size mismatch");
        }
        if self.in_flight_periods >= self.period_count {
            return Err("apple-dpaudio: DMA ring is full");
        }
        let end = period
            .byte_offset
            .checked_add(period.byte_len)
            .ok_or("apple-dpaudio: PCM period range overflow")?;
        if end > self.source_bytes || end > self.dma_bytes {
            return Err("apple-dpaudio: PCM period is outside the ring");
        }

        let source = self
            .source_vaddr
            .checked_add(period.byte_offset)
            .ok_or("apple-dpaudio: PCM source address overflow")?;
        let destination = self
            .bounce
            .as_vaddr()
            .checked_add(period.byte_offset)
            .ok_or("apple-dpaudio: PCM bounce address overflow")?;

        // SAFETY: configure() verified both source and bounce ring bounds. The
        // allocations are distinct and this period lies wholly inside each.
        unsafe {
            core::ptr::copy_nonoverlapping(
                source as *const u8,
                destination as *mut u8,
                period.byte_len,
            );
        }
        arch::clean_dcache_to_poc_range(destination, period.byte_len);

        self.channel
            .queue_cyclic_period(period.byte_offset)
            .map_err(dma_error_to_str)?;
        self.in_flight_periods += 1;
        Ok(())
    }

    fn take_completions(&mut self) -> usize {
        let completed = self
            .channel
            .take_completed_periods()
            .min(self.in_flight_periods);
        self.in_flight_periods -= completed;
        completed
    }
}

struct AppleDpAudio {
    dma: AppleDpAudioDma,
    stream: IrqSpinLock<Option<AppleDpAudioStream>>,
    completion_callback: IrqSpinLock<Option<AudioCompletionCallback>>,
}

impl AppleDpAudio {
    fn new(dma: AppleDpAudioDma) -> Self {
        Self {
            dma,
            stream: IrqSpinLock::new(None),
            completion_callback: IrqSpinLock::new(None),
        }
    }

    fn sample_bits(params: &AudioPcmParams) -> Result<u32, &'static str> {
        match params.format {
            AUDIO_PCM_FORMAT_S16LE => Ok(16),
            AUDIO_PCM_FORMAT_S32LE => Ok(32),
            _ => Err("apple-dpaudio: unsupported PCM format"),
        }
    }

    fn validate_params(params: &AudioPcmParams) -> Result<(), &'static str> {
        if params.rate != DPA_RATE_HZ || params.channels != DPA_CHANNELS {
            return Err("apple-dpaudio: only 48 kHz stereo is supported");
        }
        let _ = Self::sample_bits(params)?;
        if params.period_frames == 0
            || params.buffer_frames == 0
            || params.buffer_frames % params.period_frames != 0
        {
            return Err("apple-dpaudio: invalid PCM ring geometry");
        }
        let period_count = (params.buffer_frames / params.period_frames) as usize;
        if period_count > DPA_MAX_PERIODS {
            return Err("apple-dpaudio: PCM ring exceeds SIO descriptor capacity");
        }
        Ok(())
    }

    fn request_dma_channel(&self) -> Result<Arc<dyn DmaChannel>, &'static str> {
        self.dma
            .controller
            .request_channel(&self.dma.spec)
            .map_err(dma_error_to_str)
    }
}

impl AudioPlaybackDevice for AppleDpAudio {
    fn capabilities(&self) -> AudioPcmCapabilities {
        let mut rates = [0u32; AUDIO_PCM_MAX_RATES];
        rates[0] = DPA_RATE_HZ;
        let mut formats = 0;
        if external_audio_format_supported(DPA_RATE_HZ, 16, u32::from(DPA_CHANNELS)) {
            formats |= 1 << AUDIO_PCM_FORMAT_S16LE;
        }
        if external_audio_format_supported(DPA_RATE_HZ, 32, u32::from(DPA_CHANNELS)) {
            formats |= 1 << AUDIO_PCM_FORMAT_S32LE;
        }
        AudioPcmCapabilities {
            formats,
            rate_count: 1,
            rates,
            min_channels: DPA_CHANNELS,
            max_channels: DPA_CHANNELS,
            min_period_frames: DPA_MIN_PERIOD_FRAMES,
            max_period_frames: DPA_MAX_PERIOD_FRAMES,
            min_buffer_frames: DPA_MIN_BUFFER_FRAMES,
            max_buffer_frames: DPA_MAX_BUFFER_FRAMES,
        }
    }

    fn configure(
        &self,
        params: &AudioPcmParams,
        buffer: AudioPcmBuffer,
    ) -> Result<(), &'static str> {
        self.release()?;
        Self::validate_params(params)?;
        if !external_audio_available() {
            return Err("apple-dpaudio: external DCP audio service is not ready");
        }

        let dma_bytes = params
            .buffer_bytes()
            .ok_or("apple-dpaudio: PCM buffer size overflow")?;
        let period_bytes = params
            .period_bytes()
            .ok_or("apple-dpaudio: PCM period size overflow")?;
        if dma_bytes > buffer.buffer_bytes {
            return Err("apple-dpaudio: PCM source buffer is too small");
        }
        let allocation_bytes = align_up_checked(dma_bytes, DPA_DMA_GRANULE)
            .ok_or("apple-dpaudio: bounce ring size overflow")?;
        let page_count = allocation_bytes / PAGE_SIZE;
        let bounce = ContiguousPages::new_aligned(page_count, DPA_DMA_GRANULE)
            .ok_or("apple-dpaudio: failed to allocate bounce ring")?;
        if !bounce.as_paddr().is_multiple_of(DPA_DMA_GRANULE) {
            return Err("apple-dpaudio: bounce ring is not DART-granule aligned");
        }

        let channel = self.request_dma_channel()?;
        let cookie = external_audio_cookie(
            params.rate,
            Self::sample_bits(params)?,
            u32::from(params.channels),
        )?;
        external_audio_prepare(&cookie)?;

        let prepare_result = channel
            .prepare_cyclic(DmaCyclicConfig {
                buffer_addr: bounce.as_paddr(),
                buffer_len: dma_bytes,
                period_len: period_bytes,
                direction: DmaDirection::MemToDev,
                peripheral: Some(DmaPeripheralConfig {
                    addr: DPA_DMA_PERIPHERAL_ID,
                    width: DmaBusWidth::Width4,
                    burst_len: DPA_DMA_BURST_LEN,
                }),
            })
            .map_err(dma_error_to_str);
        if let Err(error) = prepare_result {
            let _ = external_audio_unprepare();
            return Err(error);
        }

        let callback = {
            let _irq_guard = IrqGuard::new();
            self.completion_callback.lock().clone()
        };
        if let Err(error) = channel
            .set_completion_callback(callback)
            .map_err(dma_error_to_str)
        {
            let _ = external_audio_unprepare();
            return Err(error);
        }

        let period_count = (params.buffer_frames / params.period_frames) as usize;
        let stream = AppleDpAudioStream {
            channel,
            source_vaddr: buffer.vaddr,
            source_bytes: buffer.buffer_bytes,
            bounce,
            dma_bytes,
            period_bytes,
            period_count,
            in_flight_periods: 0,
            cookie,
            dma_running: false,
            link_started: false,
        };
        let _irq_guard = IrqGuard::new();
        *self.stream.lock() = Some(stream);
        Ok(())
    }

    fn start(&self) -> Result<(), &'static str> {
        let (channel, cookie, dma_running, link_started) = {
            let _irq_guard = IrqGuard::new();
            let guard = self.stream.lock();
            let stream = guard
                .as_ref()
                .ok_or("apple-dpaudio: stream is not configured")?;
            (
                stream.channel.clone(),
                stream.cookie,
                stream.dma_running,
                stream.link_started,
            )
        };
        if dma_running && link_started {
            return Ok(());
        }

        if !link_started {
            external_audio_start(&cookie)?;
            let _irq_guard = IrqGuard::new();
            if let Some(stream) = self.stream.lock().as_mut() {
                stream.link_started = true;
            }
        }
        if !dma_running {
            if let Err(error) = channel.start().map_err(dma_error_to_str) {
                if external_audio_stop().is_ok() {
                    let _irq_guard = IrqGuard::new();
                    if let Some(stream) = self.stream.lock().as_mut() {
                        stream.link_started = false;
                    }
                }
                return Err(error);
            }
            let _irq_guard = IrqGuard::new();
            if let Some(stream) = self.stream.lock().as_mut() {
                stream.dma_running = true;
            }
        }
        Ok(())
    }

    fn stop(&self) -> Result<(), &'static str> {
        let (channel, dma_running, link_started) = {
            let _irq_guard = IrqGuard::new();
            let guard = self.stream.lock();
            let Some(stream) = guard.as_ref() else {
                return Ok(());
            };
            (
                stream.channel.clone(),
                stream.dma_running,
                stream.link_started,
            )
        };

        // Do not release the bounce ring unless hardware has acknowledged the
        // stop. Tracking DMA and link state independently also lets a later
        // release/configure retry whichever half of the sequence failed.
        if dma_running {
            channel.stop().map_err(dma_error_to_str)?;
            let _irq_guard = IrqGuard::new();
            if let Some(stream) = self.stream.lock().as_mut() {
                stream.dma_running = false;
                stream.in_flight_periods = 0;
            }
        }
        if link_started {
            external_audio_stop()?;
            let _irq_guard = IrqGuard::new();
            if let Some(stream) = self.stream.lock().as_mut() {
                stream.link_started = false;
            }
        }
        Ok(())
    }

    fn release(&self) -> Result<(), &'static str> {
        let has_stream = {
            let _irq_guard = IrqGuard::new();
            self.stream.lock().is_some()
        };
        if !has_stream {
            return Ok(());
        }

        self.stop()?;
        let channel = {
            let _irq_guard = IrqGuard::new();
            self.stream
                .lock()
                .as_ref()
                .map(|stream| stream.channel.clone())
        };
        if let Some(channel) = channel {
            channel
                .set_completion_callback(None)
                .map_err(dma_error_to_str)?;
            external_audio_unprepare()?;
            let stream = {
                let _irq_guard = IrqGuard::new();
                self.stream.lock().take()
            };
            drop(stream);
        }
        Ok(())
    }

    fn submit_period(&self, period: AudioPcmPeriod) -> Result<(), &'static str> {
        let _irq_guard = IrqGuard::new();
        let mut guard = self.stream.lock();
        let stream = guard
            .as_mut()
            .ok_or("apple-dpaudio: stream is not configured")?;
        stream.queue_period(period)
    }

    fn process_completions(&self) -> usize {
        let _irq_guard = IrqGuard::new();
        let mut guard = self.stream.lock();
        guard
            .as_mut()
            .map(AppleDpAudioStream::take_completions)
            .unwrap_or(0)
    }

    fn max_in_flight_periods(&self) -> usize {
        let _irq_guard = IrqGuard::new();
        self.stream
            .lock()
            .as_ref()
            // SIO runs the prepared cyclic ring continuously, so fill several
            // periods before start rather than waiting for its first IRQ.
            .map(|stream| stream.period_count.min(4))
            .unwrap_or(4)
    }

    fn set_completion_callback(&self, callback: Option<AudioCompletionCallback>) {
        let channel = {
            let _irq_guard = IrqGuard::new();
            *self.completion_callback.lock() = callback.clone();
            self.stream
                .lock()
                .as_ref()
                .map(|stream| stream.channel.clone())
        };
        if let Some(channel) = channel {
            let _ = channel.set_completion_callback(callback);
        }
    }
}

fn align_up_checked(value: usize, alignment: usize) -> Option<usize> {
    debug_assert!(alignment.is_power_of_two());
    value
        .checked_add(alignment - 1)
        .map(|value| value & !(alignment - 1))
}

fn dma_error_to_str(error: DmaError) -> &'static str {
    match error {
        DmaError::InvalidSpec => "apple-dpaudio: invalid DMA spec",
        DmaError::ChannelNotFound => "apple-dpaudio: DMA channel not found",
        DmaError::ChannelBusy => "apple-dpaudio: DMA channel busy",
        DmaError::InvalidConfig => "apple-dpaudio: invalid DMA config",
        DmaError::Unsupported => "apple-dpaudio: unsupported DMA operation",
        DmaError::HardwareError => "apple-dpaudio: DMA hardware error",
        DmaError::NotPrepared => "apple-dpaudio: DMA channel is not prepared",
    }
}

fn read_be_u32_cells(bytes: &[u8]) -> Vec<u32> {
    bytes
        .chunks_exact(4)
        .map(|chunk| u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]))
        .collect()
}

fn resolve_tx_dma(device: &PlatformDeviceInfo) -> Result<AppleDpAudioDma, &'static str> {
    let names = device
        .property("dma-names")
        .ok_or("apple-dpaudio: missing dma-names")?
        .as_string_list()
        .ok_or("apple-dpaudio: malformed dma-names")?;
    let bytes = device
        .property("dmas")
        .ok_or("apple-dpaudio: missing dmas")?
        .value();
    if bytes.len() % 4 != 0 {
        return Err("apple-dpaudio: malformed dmas");
    }
    let cells = read_be_u32_cells(bytes);
    let manager = DeviceManager::get_manager();
    let mut cursor = 0usize;

    for name in names {
        let controller_phandle = *cells
            .get(cursor)
            .ok_or("apple-dpaudio: DMA names exceed specifiers")?;
        cursor += 1;
        let Some(controller) = manager.get_dma_controller_by_phandle(controller_phandle) else {
            return probe_defer();
        };
        let spec_cells = controller.dma_cells();
        let end = cursor
            .checked_add(spec_cells)
            .ok_or("apple-dpaudio: DMA specifier overflow")?;
        if end > cells.len() {
            return Err("apple-dpaudio: truncated DMA specifier");
        }
        if name == "tx" {
            return Ok(AppleDpAudioDma {
                controller,
                spec: DmaSpec {
                    controller_phandle,
                    cells: cells[cursor..end].to_vec(),
                },
            });
        }
        cursor = end;
    }

    Err("apple-dpaudio: missing tx DMA channel")
}

fn probe_fn(device: &PlatformDeviceInfo) -> Result<(), &'static str> {
    let resource = device
        .get_resources()
        .iter()
        .find(|resource| matches!(resource.res_type, PlatformDeviceResourceType::MEM))
        .ok_or("apple-dpaudio: missing DPA memory resource")?;
    let peripheral_paddr = resource.start;
    if peripheral_paddr == 0 || resource.end < resource.start {
        return Err("apple-dpaudio: invalid DPA memory resource");
    }

    let dma = resolve_tx_dma(device)?;
    let backend = Arc::new(AppleDpAudio::new(dma));
    let audio_backend: Arc<dyn AudioPlaybackDevice> = backend;
    let audio_name = register_playback_device_with_info(
        audio_backend,
        AudioDeviceInfo::new(
            AUDIO_DEVICE_KIND_UNKNOWN,
            "displayport",
            "DisplayPort Audio",
        ),
    );

    println!(
        "[apple-dpaudio] probed {} fifo={:#x} audio={}",
        device.name(),
        peripheral_paddr,
        audio_name
    );
    Ok(())
}

fn remove_fn(_device: &PlatformDeviceInfo) -> Result<(), &'static str> {
    Ok(())
}

fn register_driver() {
    let driver = PlatformDeviceDriver::new(
        "apple-dpaudio",
        probe_fn,
        remove_fn,
        alloc::vec!["apple,t8103-dpaudio", "apple,dpaudio"],
    );
    DeviceManager::get_manager().register_driver(Box::new(driver), DriverPriority::Standard);
}

scarlet::driver_initcall!(register_driver);

#[used]
static SCARLET_DRIVER_APPLE_DPAUDIO_ANCHOR: fn() = force_link;

/// Keep the external driver object linked into Scarlet module builds.
#[inline(never)]
pub fn force_link() {}
