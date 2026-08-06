#![no_std]

//! Apple SIO RTKit-backed DMA controller.
//!
//! # Provenance
//!
//! The mailbox protocol, shared descriptor format, and cyclic scheduling were
//! implemented with reference to Asahi Linux's `drivers/dma/apple-sio.c` on
//! the `fairydust` branch. See the repository `ATTRIBUTION.md`.

extern crate alloc;

use alloc::boxed::Box;
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use scarlet::device::DeviceInfo;
use scarlet::device::dma::{
    DmaBusWidth, DmaChannel, DmaCompletionCallback, DmaController, DmaCyclicConfig, DmaDirection,
    DmaError, DmaSpec,
};
use scarlet::device::iommu::{
    DmaContext, IommuAttachment, IommuDomain, IommuDomainConfig, IommuDomainType, IommuMapFlags,
    IommuStreamId,
};
use scarlet::device::manager::{DeviceManager, DriverPriority, probe_defer};
use scarlet::device::platform::{
    PlatformDeviceDriver, PlatformDeviceInfo, resource::PlatformDeviceResourceType,
};
use scarlet::device::remoteproc::{RemoteprocDmaMapper, RemoteprocError};
use scarlet::environment::PAGE_SIZE;
use scarlet::mem::pmm;
use scarlet::println;
use scarlet::sync::{IrqSpinLock, Mutex};
use scarlet::{arch, time, vm};

use scarlet_driver_apple_asc::{AppleAsc, AscRxReadyHandler, get_apple_asc_by_phandle};
use scarlet_driver_apple_dart::{DartDomain, DartPageTable, get_dart_by_phandle};
use scarlet_driver_apple_rtkit::{AppleRtkit, RtkitMessage};

const NCHANNELS_MAX: usize = 0x80;
const SIO_DMA_CELLS: usize = 1;

const EP_SIO: u8 = 0x20;

const MSG_START: u8 = 0x02;
const MSG_SETUP: u8 = 0x03;
const MSG_CONFIGURE: u8 = 0x05;
const MSG_ISSUE: u8 = 0x06;
const MSG_TERMINATE: u8 = 0x08;
const MSG_ACK: u8 = 0x65;
const MSG_NACK: u8 = 0x66;
const MSG_STARTED: u8 = 0x67;
const MSG_REPORT: u8 = 0x68;

const SIO_CALL_TIMEOUT_US: u64 = 100_000;
const SIO_TERMINATE_TIMEOUT_US: u64 = 500_000;
const SIO_POLL_DELAY_US: u64 = 10;

const SIO_SHMEM_SIZE: usize = 0x1000;
const SIO_DESC_BASE: usize = 56;
const SIO_DESC_SLOTS: usize = 64;
const SIO_MAX_INFLIGHT: usize = 4;

const SIO_TAGS: usize = 16;
const SIO_FIRST_TAG: usize = 1;

// Firmware mappings live at low IOVAs (m1n1 starts them at 0x30000). Keep
// Scarlet's dynamically allocated shared and PCM buffers in a disjoint range.
const SIO_DYNAMIC_IOVA_BASE: u64 = 0x4000_0000;
const SIO_DYNAMIC_IOVA_SIZE: u64 = 0x4000_0000;

const SIOMSG_EP_MASK: u64 = 0xff;
const SIOMSG_TAG_MASK: u64 = 0x3f << 8;
const SIOMSG_TYPE_MASK: u64 = 0xff << 16;
const SIOMSG_PARAM_MASK: u64 = 0xff << 24;
const SIOMSG_DATA_MASK: u64 = 0xffff_ffff << 32;

#[derive(Clone, Copy)]
#[repr(C)]
struct SioCoprocDesc {
    pad1: u32,
    flag: u32,
    unknown: u64,
    iova: u64,
    size: u64,
    pad2: u64,
    pad3: u64,
}

const _: () = assert!(core::mem::size_of::<SioCoprocDesc>() == 48);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(C)]
struct SioChannelConfig {
    datashape: u32,
    timeout: u32,
    fifo: u32,
    threshold: u32,
    limit: u32,
}

#[derive(Clone, Copy)]
enum SioTagKind {
    Free,
    Sync,
    Issue { channel: usize, generation: usize },
}

#[derive(Clone, Copy)]
struct SioTagEntry {
    kind: SioTagKind,
    result: Option<bool>,
}

struct SioTags {
    entries: [SioTagEntry; SIO_TAGS],
    last: usize,
}

impl SioTags {
    fn new() -> Self {
        Self {
            entries: [SioTagEntry {
                kind: SioTagKind::Free,
                result: None,
            }; SIO_TAGS],
            last: 0,
        }
    }

    fn allocate(&mut self, kind: SioTagKind) -> Option<usize> {
        let mut tag = self.last % (SIO_TAGS - 1) + SIO_FIRST_TAG;
        for _ in 0..(SIO_TAGS - 1) {
            if matches!(self.entries[tag].kind, SioTagKind::Free) {
                self.entries[tag] = SioTagEntry { kind, result: None };
                self.last = tag;
                return Some(tag);
            }
            tag = tag % (SIO_TAGS - 1) + SIO_FIRST_TAG;
        }
        None
    }

    fn free(&mut self, tag: usize) {
        if tag < SIO_TAGS {
            self.entries[tag] = SioTagEntry {
                kind: SioTagKind::Free,
                result: None,
            };
        }
    }
}

struct SioDmaMapper {
    dma: DmaContext,
    firmware_table: Arc<IrqSpinLock<DartPageTable>>,
}

impl RemoteprocDmaMapper for SioDmaMapper {
    fn alignment(&self) -> usize {
        self.dma.mapping_granule()
    }

    fn map(&self, paddr: usize, size: usize) -> Result<u64, RemoteprocError> {
        self.dma
            .map_phys(
                paddr,
                size,
                IommuMapFlags::READ | IommuMapFlags::WRITE | IommuMapFlags::COHERENT,
            )
            .map(|address| address as u64)
            .map_err(|_| RemoteprocError::LoadFailed)
    }

    fn translate(&self, dva: u64) -> Option<usize> {
        self.firmware_table
            .lock()
            .translate_iova(usize::try_from(dva).ok()?)
    }

    fn unmap(&self, dva: u64, size: usize) {
        let _ = self.dma.unmap(dva, size);
    }
}

struct SioSharedMemory {
    vaddr: usize,
    paddr: usize,
    iova: u64,
    allocation_size: usize,
    pages: usize,
    dma: DmaContext,
}

impl SioSharedMemory {
    fn allocate(dma: DmaContext) -> Result<Self, &'static str> {
        let granule = dma.mapping_granule();
        let allocation_size = SIO_SHMEM_SIZE.max(granule);
        let pages = allocation_size.div_ceil(PAGE_SIZE);
        let align_pages = granule.div_ceil(PAGE_SIZE);
        let paddr = pmm::alloc_contiguous_pages_aligned(pages, align_pages)
            .ok_or("apple-sio: shared-memory allocation failed")?;
        let vaddr = vm::phys_to_virt(paddr);
        // SAFETY: `paddr` owns `pages` contiguous pages in the direct map.
        unsafe {
            core::ptr::write_bytes(vaddr as *mut u8, 0, allocation_size);
        }
        arch::clean_dcache_to_poc_range(vaddr, allocation_size);

        let iova = match dma.map_phys(
            paddr,
            allocation_size,
            IommuMapFlags::READ | IommuMapFlags::WRITE | IommuMapFlags::COHERENT,
        ) {
            Ok(iova) => iova as u64,
            Err(_) => {
                pmm::free_contiguous_pages(paddr, pages);
                return Err("apple-sio: shared-memory IOMMU mapping failed");
            }
        };

        Ok(Self {
            vaddr,
            paddr,
            iova,
            allocation_size,
            pages,
            dma,
        })
    }

    fn write_config(&self, config: SioChannelConfig) {
        // SAFETY: the first 20 bytes of the SIO shared page are the channel
        // configuration scratch area consumed synchronously by CONFIGURE.
        unsafe {
            core::ptr::write_volatile(self.vaddr as *mut SioChannelConfig, config);
        }
        arch::clean_dcache_to_poc_range(self.vaddr, core::mem::size_of::<SioChannelConfig>());
    }

    fn write_descriptor(&self, slot: usize, descriptor: SioCoprocDesc) {
        let offset = SIO_DESC_BASE + slot * core::mem::size_of::<SioCoprocDesc>();
        // SAFETY: descriptor allocation restricts `slot` to the 64 entries
        // fitting inside the 4 KiB protocol-visible shared-memory page.
        unsafe {
            core::ptr::write_volatile((self.vaddr + offset) as *mut SioCoprocDesc, descriptor);
        }
        arch::clean_dcache_to_poc_range(self.vaddr + offset, core::mem::size_of::<SioCoprocDesc>());
    }
}

impl Drop for SioSharedMemory {
    fn drop(&mut self) {
        let _ = self.dma.unmap(self.iova, self.allocation_size);
        pmm::free_contiguous_pages(self.paddr, self.pages);
    }
}

struct SioTransfer {
    config: DmaCyclicConfig,
    dma_addr: u64,
    dma_len: usize,
    descriptor_slots: Vec<usize>,
    next_issue: usize,
    next_report: usize,
    inflight: usize,
    issue_pending: bool,
    /// Periods submitted by the client and not yet reported complete.
    ///
    /// A bit remains set across ISSUE/ACK so the client cannot recycle the
    /// corresponding buffer while firmware still owns it.
    committed: u64,
    /// Committed periods whose ISSUE request firmware has acknowledged.
    issued: u64,
    terminated: bool,
    generation: usize,
}

struct SioChannelState {
    in_use: AtomicBool,
    running: AtomicBool,
    error: AtomicBool,
    configured: IrqSpinLock<Option<SioChannelConfig>>,
    transfer: IrqSpinLock<Option<SioTransfer>>,
    completed_periods: AtomicUsize,
    completion_callback: IrqSpinLock<Option<DmaCompletionCallback>>,
    next_generation: AtomicUsize,
}

impl SioChannelState {
    fn new() -> Self {
        Self {
            in_use: AtomicBool::new(false),
            running: AtomicBool::new(false),
            error: AtomicBool::new(false),
            configured: IrqSpinLock::new(None),
            transfer: IrqSpinLock::new(None),
            completed_periods: AtomicUsize::new(0),
            completion_callback: IrqSpinLock::new(None),
            next_generation: AtomicUsize::new(1),
        }
    }
}

struct AppleSioInner {
    asc: Arc<AppleAsc>,
    rtkit: Arc<AppleRtkit>,
    dma: DmaContext,
    shared: SioSharedMemory,
    channel_count: usize,
    channels: Vec<SioChannelState>,
    tags: IrqSpinLock<SioTags>,
    descriptors: IrqSpinLock<u64>,
    tx_lock: Mutex<()>,
    configure_lock: Mutex<()>,
    issue_lock: Mutex<()>,
    draining: AtomicBool,
}

static SIO_WORKER_INNER: IrqSpinLock<Option<Weak<AppleSioInner>>> = IrqSpinLock::new(None);
static SIO_WORKER_PENDING: AtomicBool = AtomicBool::new(false);
static SIO_WORKER_STARTED: AtomicBool = AtomicBool::new(false);
static SIO_WORKER_WAKER: scarlet::sync::Waker =
    scarlet::sync::Waker::new_uninterruptible("apple-sio-worker");

fn queue_sio_worker() {
    if !SIO_WORKER_PENDING.swap(true, Ordering::AcqRel) {
        SIO_WORKER_WAKER.wake_one();
    }
}

fn process_deferred_sio_work() -> bool {
    if !SIO_WORKER_PENDING.swap(false, Ordering::AcqRel) {
        return false;
    }

    let inner = SIO_WORKER_INNER.lock().as_ref().and_then(Weak::upgrade);
    let Some(inner) = inner else {
        return true;
    };

    inner.process_messages();
    inner.issue_ready_transfers();
    true
}

fn sio_worker_entry() {
    loop {
        while process_deferred_sio_work() {}

        let Some(task) = scarlet::task::mytask() else {
            scarlet::arch::instruction::idle();
        };
        SIO_WORKER_WAKER.wait(task.get_id(), task.get_trapframe());
    }
}

fn start_sio_worker() {
    if SIO_WORKER_INNER
        .lock()
        .as_ref()
        .and_then(Weak::upgrade)
        .is_none()
        || SIO_WORKER_STARTED
            .compare_exchange(false, true, Ordering::SeqCst, Ordering::SeqCst)
            .is_err()
    {
        return;
    }

    let task = scarlet::task::new_kernel_task(
        alloc::string::String::from("apple-sio-worker"),
        1,
        sio_worker_entry,
    );
    task.init();
    scarlet::sched::scheduler::add_task(task, 0);
}

impl AppleSioInner {
    fn channel(&self, index: usize) -> Result<&SioChannelState, DmaError> {
        self.channels.get(index).ok_or(DmaError::ChannelNotFound)
    }

    fn alloc_descriptor(&self) -> Option<usize> {
        let mut allocated = self.descriptors.lock();
        for slot in 0..SIO_DESC_SLOTS {
            let bit = 1u64 << slot;
            if *allocated & bit == 0 {
                *allocated |= bit;
                return Some(slot);
            }
        }
        None
    }

    fn free_descriptor(&self, slot: usize) {
        if slot < SIO_DESC_SLOTS {
            *self.descriptors.lock() &= !(1u64 << slot);
        }
    }

    fn release_transfer(&self, transfer: SioTransfer) {
        for slot in transfer.descriptor_slots {
            self.free_descriptor(slot);
        }
        let _ = self.dma.unmap(transfer.dma_addr, transfer.dma_len);
    }

    fn send_tagged(&self, message: u64, kind: SioTagKind) -> Result<usize, DmaError> {
        // ASC's AP->IOP queue is a single MMIO FIFO. Serialize writers so a
        // synchronous control request cannot race a deferred ISSUE write.
        let _tx_guard = self.tx_lock.lock();
        let tag = self
            .tags
            .lock()
            .allocate(kind)
            .ok_or(DmaError::ChannelBusy)?;
        let tagged = (message & !SIOMSG_TAG_MASK) | ((tag as u64) << 8);
        if self
            .rtkit
            .send(&RtkitMessage {
                ep: EP_SIO,
                msg: tagged,
            })
            .is_err()
        {
            self.tags.lock().free(tag);
            return Err(DmaError::HardwareError);
        }
        Ok(tag)
    }

    fn call(&self, message: u64) -> Result<bool, DmaError> {
        let tag = self.send_tagged(message, SioTagKind::Sync)?;
        let start = time::current_time();
        loop {
            self.process_messages();
            let result = {
                let mut tags = self.tags.lock();
                let result = tags.entries[tag].result.take();
                if result.is_some() {
                    tags.free(tag);
                }
                result
            };
            if let Some(result) = result {
                return Ok(result);
            }
            if time::current_time().saturating_sub(start) >= SIO_CALL_TIMEOUT_US {
                self.tags.lock().free(tag);
                return Err(DmaError::HardwareError);
            }
            time::udelay(SIO_POLL_DELAY_US);
        }
    }

    fn process_messages(&self) {
        if self
            .draining
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return;
        }

        loop {
            let mut receive_failed = false;
            loop {
                let mut message = RtkitMessage { ep: 0, msg: 0 };
                match self.rtkit.recv_endpoint(EP_SIO, &mut message) {
                    Ok(true) => self.process_message(message.msg),
                    Ok(false) => {
                        if !self.asc.can_recv() {
                            break;
                        }
                    }
                    Err(error) => {
                        println!("[apple-sio] receive failed: {}", error);
                        receive_failed = true;
                        break;
                    }
                }
            }

            // Publish the idle state before the final availability check. If
            // an IRQ queued data while `draining` was true, either this path
            // reacquires ownership and drains it, or the IRQ callback wins
            // the CAS and becomes the new drainer. This closes the otherwise
            // possible lost-wakeup window between an empty check and clear.
            self.draining.store(false, Ordering::Release);
            if receive_failed
                || !self.asc.can_recv()
                || self
                    .draining
                    .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
                    .is_err()
            {
                return;
            }
        }
    }

    fn process_message(&self, message: u64) {
        let data = ((message & SIOMSG_DATA_MASK) >> 32) as u32;
        let mut message_type = ((message & SIOMSG_TYPE_MASK) >> 16) as u8;
        let tag = ((message & SIOMSG_TAG_MASK) >> 8) as usize;
        let endpoint = (message & SIOMSG_EP_MASK) as usize;

        if message_type == MSG_STARTED {
            println!("[apple-sio] protocol v{}", data);
            message_type = MSG_ACK;
        }

        match message_type {
            MSG_ACK | MSG_NACK => {
                if tag >= SIO_TAGS {
                    println!("[apple-sio] invalid reply tag {}", tag);
                    return;
                }
                let ok = message_type == MSG_ACK;
                let kind = {
                    let mut tags = self.tags.lock();
                    match tags.entries[tag].kind {
                        SioTagKind::Free => SioTagKind::Free,
                        SioTagKind::Sync => {
                            tags.entries[tag].result = Some(ok);
                            SioTagKind::Sync
                        }
                        issue @ SioTagKind::Issue { .. } => {
                            tags.free(tag);
                            issue
                        }
                    }
                };

                if let SioTagKind::Issue {
                    channel,
                    generation,
                } = kind
                {
                    self.process_issue_reply(channel, generation, ok);
                } else if matches!(kind, SioTagKind::Free) {
                    println!("[apple-sio] reply for unused tag {}", tag);
                }

                if !ok {
                    println!("[apple-sio] NACK channel={} tag={}", endpoint, tag);
                }
            }
            MSG_REPORT => self.process_report(endpoint),
            _ => println!("[apple-sio] unknown message {:#018x}", message),
        }
    }

    fn send_issue(&self, channel: usize, generation: usize) -> Result<(), DmaError> {
        let _issue_guard = self.issue_lock.lock();
        self.send_issue_locked(channel, generation)
    }

    fn send_issue_locked(&self, channel: usize, generation: usize) -> Result<(), DmaError> {
        let descriptor_slot = {
            let state = self.channel(channel)?;
            if !state.running.load(Ordering::Acquire) || state.error.load(Ordering::Acquire) {
                return Ok(());
            }
            let mut transfer = state.transfer.lock();
            let transfer = transfer.as_mut().ok_or(DmaError::NotPrepared)?;
            if transfer.generation != generation || transfer.terminated {
                return Ok(());
            }
            if transfer.issue_pending || transfer.inflight >= SIO_MAX_INFLIGHT {
                return Ok(());
            }
            let period = transfer.next_issue;
            let bit = 1u64 << period;
            // DMA descriptors are issued strictly in ring order. A period is
            // eligible only after queue_cyclic_period() commits its contents,
            // and never while a previous pass over that period is in flight.
            if transfer.committed & bit == 0 || transfer.issued & bit != 0 {
                return Ok(());
            }
            let descriptor_slot = transfer.descriptor_slots[transfer.next_issue];
            transfer.issue_pending = true;
            descriptor_slot
        };

        let message = sio_message(channel as u8, MSG_ISSUE, 0, (descriptor_slot * 4) as u32);
        if let Err(error) = self.send_tagged(
            message,
            SioTagKind::Issue {
                channel,
                generation,
            },
        ) {
            if let Ok(state) = self.channel(channel) {
                if let Some(transfer) = state.transfer.lock().as_mut()
                    && transfer.generation == generation
                {
                    transfer.issue_pending = false;
                    transfer.terminated = true;
                }
                state.error.store(true, Ordering::Release);
                state.running.store(false, Ordering::Release);
            }
            return Err(error);
        }
        Ok(())
    }

    fn issue_ready_transfers(&self) {
        for channel in 0..self.channel_count {
            let generation = {
                let transfer = self.channels[channel].transfer.lock();
                transfer.as_ref().map(|transfer| transfer.generation)
            };
            let Some(generation) = generation else {
                continue;
            };

            if let Err(error) = self.send_issue(channel, generation) {
                println!(
                    "[apple-sio] failed to issue channel {} from worker: {:?}",
                    channel, error
                );
            }
        }
    }

    fn process_issue_reply(&self, channel: usize, generation: usize, ok: bool) {
        let Some(state) = self.channels.get(channel) else {
            return;
        };
        let queue_more = {
            let mut transfer = state.transfer.lock();
            let Some(transfer) = transfer.as_mut() else {
                return;
            };
            if transfer.generation != generation {
                return;
            }
            transfer.issue_pending = false;
            if !ok {
                transfer.terminated = true;
                state.error.store(true, Ordering::Release);
                state.running.store(false, Ordering::Release);
                false
            } else {
                let period = transfer.next_issue;
                transfer.issued |= 1u64 << period;
                transfer.next_issue = (transfer.next_issue + 1) % transfer.descriptor_slots.len();
                transfer.inflight += 1;
                !transfer.terminated
                    && state.running.load(Ordering::Acquire)
                    && !state.error.load(Ordering::Acquire)
                    && transfer.inflight < SIO_MAX_INFLIGHT
            }
        };

        if queue_more {
            queue_sio_worker();
        }
    }

    fn process_report(&self, channel: usize) {
        let Some(state) = self.channels.get(channel) else {
            println!("[apple-sio] report for invalid channel {}", channel);
            return;
        };

        let outcome = {
            let mut transfer = state.transfer.lock();
            let Some(transfer) = transfer.as_mut() else {
                return;
            };
            if transfer.inflight == 0 {
                None
            } else {
                let period = transfer.next_report;
                let bit = 1u64 << period;
                if transfer.committed & bit == 0 || transfer.issued & bit == 0 {
                    transfer.terminated = true;
                    state.error.store(true, Ordering::Release);
                    state.running.store(false, Ordering::Release);
                    None
                } else {
                    transfer.inflight -= 1;
                    transfer.next_report =
                        (transfer.next_report + 1) % transfer.descriptor_slots.len();
                    transfer.committed &= !bit;
                    transfer.issued &= !bit;
                    Some((
                        !transfer.terminated,
                        !transfer.terminated
                            && state.running.load(Ordering::Acquire)
                            && !state.error.load(Ordering::Acquire)
                            && !transfer.issue_pending
                            && transfer.inflight < SIO_MAX_INFLIGHT,
                    ))
                }
            }
        };
        let Some((visible_completion, queue_more)) = outcome else {
            println!(
                "[apple-sio] unexpected report channel={} (no matching in-flight descriptor)",
                channel
            );
            return;
        };

        if visible_completion {
            state.completed_periods.fetch_add(1, Ordering::AcqRel);
        }
        if queue_more {
            queue_sio_worker();
        }

        // The audio callback may immediately re-enter take_completed_periods()
        // and queue_cyclic_period(). Invoke it only after every SIO lock above
        // has been released.
        if visible_completion {
            let callback = state.completion_callback.lock().clone();
            if let Some(callback) = callback {
                callback();
            }
        }
    }

    fn configure_channel(&self, index: usize, width: DmaBusWidth) -> Result<(), DmaError> {
        let config = SioChannelConfig {
            datashape: match width {
                DmaBusWidth::Width1 => 0,
                DmaBusWidth::Width2 => 1,
                DmaBusWidth::Width4 => 2,
                DmaBusWidth::Width8 => return Err(DmaError::InvalidConfig),
            },
            timeout: 0,
            fifo: 0x800,
            threshold: 0x800,
            limit: 0x800,
        };
        let state = self.channel(index)?;
        if let Some(existing) = *state.configured.lock() {
            return if existing == config {
                Ok(())
            } else {
                Err(DmaError::ChannelBusy)
            };
        }

        let _configure_guard = self.configure_lock.lock();
        self.shared.write_config(config);
        let acknowledged = self.call(sio_message(index as u8, MSG_CONFIGURE, 0, 0))?;
        if !acknowledged {
            return Err(DmaError::InvalidConfig);
        }
        *state.configured.lock() = Some(config);
        Ok(())
    }

    fn start_channel(&self, index: usize) -> Result<(), DmaError> {
        let _issue_guard = self.issue_lock.lock();
        let state = self.channel(index)?;
        self.process_messages();
        if state.error.load(Ordering::Acquire) {
            return Err(DmaError::HardwareError);
        }
        if state.running.load(Ordering::Acquire) {
            return Ok(());
        }
        let generation = {
            let mut transfer = state.transfer.lock();
            let transfer = transfer.as_mut().ok_or(DmaError::NotPrepared)?;
            transfer.terminated = false;
            transfer.generation
        };
        state.running.store(true, Ordering::Release);
        if let Err(error) = self.send_issue_locked(index, generation) {
            state.running.store(false, Ordering::Release);
            return Err(error);
        }
        Ok(())
    }

    fn stop_channel(&self, index: usize) -> Result<(), DmaError> {
        let _issue_guard = self.issue_lock.lock();
        let state = self.channel(index)?;
        self.process_messages();

        let needs_terminate = {
            let mut transfer = state.transfer.lock();
            let Some(transfer) = transfer.as_mut() else {
                state.running.store(false, Ordering::Release);
                return Ok(());
            };
            let needs = state.running.load(Ordering::Acquire)
                || transfer.terminated
                || transfer.inflight != 0
                || transfer.issue_pending;
            transfer.terminated = needs;
            needs
        };
        state.running.store(false, Ordering::Release);

        if needs_terminate {
            if !self.call(sio_message(index as u8, MSG_TERMINATE, 0, 0))? {
                return Err(DmaError::HardwareError);
            }

            let start = time::current_time();
            loop {
                self.process_messages();
                let done = {
                    let transfer = state.transfer.lock();
                    transfer
                        .as_ref()
                        .map(|transfer| transfer.inflight == 0 && !transfer.issue_pending)
                        .unwrap_or(true)
                };
                if done {
                    break;
                }
                if time::current_time().saturating_sub(start) >= SIO_TERMINATE_TIMEOUT_US {
                    return Err(DmaError::HardwareError);
                }
                time::udelay(SIO_POLL_DELAY_US);
            }
        }

        if let Some(transfer) = state.transfer.lock().as_mut() {
            transfer.next_issue = 0;
            transfer.next_report = 0;
            transfer.inflight = 0;
            transfer.issue_pending = false;
            transfer.committed = 0;
            transfer.issued = 0;
            transfer.terminated = false;
            transfer.generation = state.next_generation.fetch_add(1, Ordering::AcqRel) + 1;
        }
        state.completed_periods.store(0, Ordering::Release);
        state.error.store(false, Ordering::Release);
        Ok(())
    }
}

struct SioRxHandler;

impl AscRxReadyHandler for SioRxHandler {
    fn rx_ready(&self) {
        // ASC invokes this callback in hard-IRQ context. Keep it bounded: all
        // RTKit receive processing, ISSUE transmission, and DMA completion
        // callbacks run from the dedicated SIO kernel worker.
        queue_sio_worker();
    }
}

/// Apple SIO DMA controller.
#[derive(Clone)]
pub struct AppleSio {
    inner: Arc<AppleSioInner>,
}

impl DmaController for AppleSio {
    fn name(&self) -> &'static str {
        "apple-sio"
    }

    fn dma_cells(&self) -> usize {
        SIO_DMA_CELLS
    }

    fn request_channel(&self, spec: &DmaSpec) -> Result<Arc<dyn DmaChannel>, DmaError> {
        if spec.cells.len() != SIO_DMA_CELLS {
            return Err(DmaError::InvalidSpec);
        }
        let index = spec.cells[0] as usize;
        if index >= self.inner.channel_count {
            return Err(DmaError::ChannelNotFound);
        }
        let state = self.inner.channel(index)?;
        if state
            .in_use
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return Err(DmaError::ChannelBusy);
        }
        Ok(Arc::new(AppleSioChannel {
            controller: self.clone(),
            index,
        }))
    }
}

struct AppleSioChannel {
    controller: AppleSio,
    index: usize,
}

impl Drop for AppleSioChannel {
    fn drop(&mut self) {
        let state = match self.controller.inner.channel(self.index) {
            Ok(state) => state,
            Err(_) => return,
        };
        if let Err(error) = self.controller.inner.stop_channel(self.index) {
            // Keep the transfer and its IOMMU mapping pinned if firmware did
            // not prove that DMA stopped. Reusing or freeing it would create a
            // device-side use-after-free.
            println!(
                "[apple-sio] channel {} stop failed during drop: {:?}; resources pinned",
                self.index, error
            );
            return;
        }
        if let Some(transfer) = state.transfer.lock().take() {
            self.controller.inner.release_transfer(transfer);
        }
        state.completed_periods.store(0, Ordering::Release);
        *state.completion_callback.lock() = None;
        state.error.store(false, Ordering::Release);
        state.in_use.store(false, Ordering::Release);
    }
}

impl DmaChannel for AppleSioChannel {
    fn name(&self) -> &'static str {
        "apple-sio-channel"
    }

    fn prepare_cyclic(&self, config: DmaCyclicConfig) -> Result<(), DmaError> {
        config.validate()?;
        let state = self.controller.inner.channel(self.index)?;
        let expected_direction = if self.index & 1 == 0 {
            DmaDirection::MemToDev
        } else {
            DmaDirection::DevToMem
        };
        if config.direction != expected_direction {
            return Err(DmaError::InvalidConfig);
        }
        let peripheral = config.peripheral.ok_or(DmaError::InvalidConfig)?;
        let periods = config.buffer_len / config.period_len;
        if periods == 0 || periods > SIO_DESC_SLOTS {
            return Err(DmaError::InvalidConfig);
        }

        self.controller.inner.stop_channel(self.index)?;
        if let Some(old_transfer) = state.transfer.lock().take() {
            self.controller.inner.release_transfer(old_transfer);
        }
        self.controller
            .inner
            .configure_channel(self.index, peripheral.width)?;

        let map_flags = match config.direction {
            DmaDirection::MemToDev => IommuMapFlags::READ | IommuMapFlags::COHERENT,
            DmaDirection::DevToMem => IommuMapFlags::WRITE | IommuMapFlags::COHERENT,
            DmaDirection::MemToMem => return Err(DmaError::InvalidConfig),
        };
        let dma_addr = self
            .controller
            .inner
            .dma
            .map_phys(config.buffer_addr, config.buffer_len, map_flags)
            .map_err(|_| DmaError::HardwareError)?;

        let mut slots = Vec::with_capacity(periods);
        for period in 0..periods {
            let Some(slot) = self.controller.inner.alloc_descriptor() else {
                for slot in slots {
                    self.controller.inner.free_descriptor(slot);
                }
                let _ = self.controller.inner.dma.unmap(dma_addr, config.buffer_len);
                return Err(DmaError::ChannelBusy);
            };
            slots.push(slot);
            self.controller.inner.shared.write_descriptor(
                slot,
                SioCoprocDesc {
                    pad1: 0,
                    flag: 1,
                    unknown: 0,
                    iova: dma_addr as u64 + (period * config.period_len) as u64,
                    size: config.period_len as u64,
                    pad2: 0,
                    pad3: 0,
                },
            );
        }

        let generation = state.next_generation.fetch_add(1, Ordering::AcqRel);
        *state.transfer.lock() = Some(SioTransfer {
            config,
            dma_addr,
            dma_len: config.buffer_len,
            descriptor_slots: slots,
            next_issue: 0,
            next_report: 0,
            inflight: 0,
            issue_pending: false,
            committed: 0,
            issued: 0,
            terminated: false,
            generation,
        });
        state.completed_periods.store(0, Ordering::Release);
        state.error.store(false, Ordering::Release);
        Ok(())
    }

    fn start(&self) -> Result<(), DmaError> {
        self.controller.inner.start_channel(self.index)
    }

    fn stop(&self) -> Result<(), DmaError> {
        self.controller.inner.stop_channel(self.index)
    }

    fn pause(&self) -> Result<(), DmaError> {
        self.stop()
    }

    fn resume(&self) -> Result<(), DmaError> {
        self.start()
    }

    fn residue(&self) -> Result<usize, DmaError> {
        let state = self.controller.inner.channel(self.index)?;
        let transfer = state.transfer.lock();
        let transfer = transfer.as_ref().ok_or(DmaError::NotPrepared)?;
        // SIO reports completion at descriptor granularity and exposes no
        // byte-progress counter.
        Ok(transfer.config.period_len)
    }

    fn take_completed_periods(&self) -> usize {
        self.controller
            .inner
            .channel(self.index)
            .map(|state| state.completed_periods.swap(0, Ordering::AcqRel))
            .unwrap_or(0)
    }

    fn queue_cyclic_period(&self, byte_offset: usize) -> Result<(), DmaError> {
        let state = self.controller.inner.channel(self.index)?;
        if state.error.load(Ordering::Acquire) {
            return Err(DmaError::HardwareError);
        }
        let kick = {
            let mut transfer = state.transfer.lock();
            let transfer = transfer.as_mut().ok_or(DmaError::NotPrepared)?;
            if !byte_offset.is_multiple_of(transfer.config.period_len)
                || byte_offset >= transfer.config.buffer_len
            {
                return Err(DmaError::InvalidConfig);
            }
            let period = byte_offset / transfer.config.period_len;
            let bit = 1u64 << period;
            if transfer.committed & bit != 0 {
                return Err(DmaError::ChannelBusy);
            }
            transfer.committed |= bit;
            state.running.load(Ordering::Acquire)
                && !transfer.terminated
                && !transfer.issue_pending
                && transfer.inflight < SIO_MAX_INFLIGHT
        };
        if kick {
            queue_sio_worker();
        }
        Ok(())
    }

    fn set_completion_callback(
        &self,
        callback: Option<DmaCompletionCallback>,
    ) -> Result<(), DmaError> {
        let state = self.controller.inner.channel(self.index)?;
        *state.completion_callback.lock() = callback;
        Ok(())
    }

    fn is_running(&self) -> bool {
        self.controller
            .inner
            .channel(self.index)
            .map(|state| state.running.load(Ordering::Acquire))
            .unwrap_or(false)
    }
}

fn sio_message(endpoint: u8, message_type: u8, param: u8, data: u32) -> u64 {
    ((endpoint as u64) & SIOMSG_EP_MASK)
        | (((message_type as u64) << 16) & SIOMSG_TYPE_MASK)
        | (((param as u64) << 24) & SIOMSG_PARAM_MASK)
        | (((data as u64) << 32) & SIOMSG_DATA_MASK)
}

fn read_be_u32(bytes: &[u8], offset: usize) -> Option<u32> {
    let end = offset.checked_add(4)?;
    Some(u32::from_be_bytes(bytes.get(offset..end)?.try_into().ok()?))
}

fn property_phandle(device: &PlatformDeviceInfo, name: &str) -> Option<u32> {
    read_be_u32(device.property(name)?.value(), 0)
}

fn device_phandle(device: &PlatformDeviceInfo) -> Option<u32> {
    device
        .property("phandle")
        .or_else(|| device.property("linux,phandle"))
        .and_then(|property| property.as_usize())
        .map(|value| value as u32)
}

fn iommu_spec(device: &PlatformDeviceInfo) -> Option<(u32, u32)> {
    let value = device.property("iommus")?.value();
    Some((read_be_u32(value, 0)?, read_be_u32(value, 4)?))
}

fn send_firmware_params(
    inner: &AppleSioInner,
    device: &PlatformDeviceInfo,
) -> Result<(), DmaError> {
    let Some(property) = device.property("apple,sio-firmware-params") else {
        return Err(DmaError::InvalidConfig);
    };
    let values = property.value();
    if !values.len().is_multiple_of(8) {
        return Err(DmaError::InvalidConfig);
    }

    for offset in (0..values.len()).step_by(8) {
        let key = read_be_u32(values, offset).ok_or(DmaError::InvalidConfig)?;
        let value = read_be_u32(values, offset + 4).ok_or(DmaError::InvalidConfig)?;
        let acknowledged = inner.call(sio_message(
            (key >> 8) as u8,
            MSG_SETUP,
            (key & 0xff) as u8,
            value,
        ))?;
        if !acknowledged {
            return Err(DmaError::HardwareError);
        }
    }
    Ok(())
}

fn probe_fn(device: &PlatformDeviceInfo) -> Result<(), &'static str> {
    let resource = device
        .get_resources()
        .iter()
        .find(|resource| matches!(resource.res_type, PlatformDeviceResourceType::MEM))
        .ok_or("apple-sio: no memory resource")?;
    let phandle = device_phandle(device).ok_or("apple-sio: missing phandle")?;
    let channel_count = device
        .property("dma-channels")
        .and_then(|property| property.as_usize())
        .ok_or("apple-sio: missing dma-channels")?;
    if channel_count == 0 || channel_count > NCHANNELS_MAX {
        return Err("apple-sio: invalid dma-channels");
    }
    let dma_cells = device
        .property("#dma-cells")
        .and_then(|property| property.as_usize())
        .unwrap_or(SIO_DMA_CELLS);
    if dma_cells != SIO_DMA_CELLS {
        return Err("apple-sio: unsupported #dma-cells");
    }

    let mailbox_phandle = property_phandle(device, "mboxes")
        .or_else(|| property_phandle(device, "mailboxes"))
        .ok_or("apple-sio: missing ASC mailbox")?;
    let Some(asc) = get_apple_asc_by_phandle(mailbox_phandle) else {
        return probe_defer();
    };
    let (dart_phandle, stream_id) = iommu_spec(device).ok_or("apple-sio: missing IOMMU")?;
    let Some(dart) = get_dart_by_phandle(dart_phandle) else {
        return probe_defer();
    };
    let iommu_controller = DeviceManager::get_manager()
        .get_iommu_controller_by_phandle(dart_phandle)
        .ok_or("apple-sio: DART controller unavailable")?;
    let stream = IommuStreamId {
        id: stream_id,
        substream_id: None,
    };
    let root_paddr = dart
        .ttbr_paddr(stream_id as usize)
        .ok_or("apple-sio: firmware DART table unavailable")?;
    let firmware_table = Arc::new(IrqSpinLock::new(DartPageTable::wrap_existing(
        root_paddr,
        dart.page_shift(),
    )?));
    let domain: Arc<dyn IommuDomain> = Arc::new(
        DartDomain::wrap_existing(Arc::clone(&dart), stream)
            .map_err(|_| "apple-sio: failed to wrap firmware DART domain")?,
    );
    let dma_config = IommuDomainConfig {
        domain_type: IommuDomainType::Dma,
        iova_base: SIO_DYNAMIC_IOVA_BASE,
        iova_size: SIO_DYNAMIC_IOVA_SIZE,
    };
    let dma = DmaContext::from_iommu_attachments(
        Some(IommuAttachment {
            controller: iommu_controller,
            domain,
            streams: alloc::vec![stream],
        }),
        Vec::new(),
        dma_config,
    );
    let mapper = Arc::new(SioDmaMapper {
        dma: dma.clone(),
        firmware_table,
    });
    let rtkit = Arc::new(AppleRtkit::new_with_dma_mapper(Arc::clone(&asc), mapper));
    let shared = SioSharedMemory::allocate(dma.clone())?;

    let mut channels = Vec::with_capacity(channel_count);
    for _ in 0..channel_count {
        channels.push(SioChannelState::new());
    }
    let inner = Arc::new(AppleSioInner {
        asc: Arc::clone(&asc),
        rtkit,
        dma,
        shared,
        channel_count,
        channels,
        tags: IrqSpinLock::new(SioTags::new()),
        descriptors: IrqSpinLock::new(0),
        tx_lock: Mutex::new(()),
        configure_lock: Mutex::new(()),
        issue_lock: Mutex::new(()),
        draining: AtomicBool::new(false),
    });

    inner
        .rtkit
        .boot_with_endpoints(&[EP_SIO])
        .map_err(|_| "apple-sio: RTKit boot failed")?;
    if !inner
        .call(sio_message(0, MSG_START, 0, 0))
        .map_err(|_| "apple-sio: START failed")?
    {
        return Err("apple-sio: START was rejected");
    }
    send_firmware_params(&inner, device).map_err(|_| "apple-sio: firmware params failed")?;
    if !inner
        .call(sio_message(
            0,
            MSG_SETUP,
            1,
            (inner.shared.iova >> 12) as u32,
        ))
        .map_err(|_| "apple-sio: shared-memory address setup failed")?
    {
        return Err("apple-sio: shared-memory address was rejected");
    }
    if !inner
        .call(sio_message(0, MSG_SETUP, 2, SIO_SHMEM_SIZE as u32))
        .map_err(|_| "apple-sio: shared-memory size setup failed")?
    {
        return Err("apple-sio: shared-memory size was rejected");
    }

    // Register only after the boot and setup calls. During RTKit management
    // negotiation, an endpoint-specific callback must not consume HELLO/EPMAP.
    {
        let mut worker_inner = SIO_WORKER_INNER.lock();
        if worker_inner.as_ref().and_then(Weak::upgrade).is_some() {
            return Err("apple-sio: only one controller is supported");
        }
        *worker_inner = Some(Arc::downgrade(&inner));
    }
    asc.set_rx_ready_handler(Some(Arc::new(SioRxHandler)));

    let shared_iova = inner.shared.iova;
    let controller = Arc::new(AppleSio { inner });
    DeviceManager::get_manager().register_dma_controller(phandle, controller);
    println!(
        "[apple-sio] registered {} paddr={:#x} channels={} shared-iova={:#x}",
        device.name(),
        resource.start,
        channel_count,
        shared_iova
    );
    Ok(())
}

fn remove_fn(_device: &PlatformDeviceInfo) -> Result<(), &'static str> {
    Ok(())
}

fn register_driver() {
    let driver = PlatformDeviceDriver::new(
        "apple-sio",
        probe_fn,
        remove_fn,
        alloc::vec!["apple,t8103-sio", "apple,sio"],
    );
    DeviceManager::get_manager().register_driver(Box::new(driver), DriverPriority::Core);
}

scarlet::driver_initcall!(register_driver);
scarlet::late_initcall!(start_sio_worker);

#[used]
static SCARLET_DRIVER_APPLE_SIO_ANCHOR: fn() = force_link;

/// Keep the external driver object linked into Scarlet module builds.
#[inline(never)]
pub fn force_link() {}
