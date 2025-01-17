use core::arch::asm;

// PhytiumPi
pub const CORE0_AFF: u64 = 0x000;
pub const CORE1_AFF: u64 = 0x100;
pub const CORE2_AFF: u64 = 0x200;
pub const CORE3_AFF: u64 = 0x201;
pub const FCORE_NUM: u64 = 4;

/// Read reg: MPIDR_EL1
pub(crate) fn read_mpidr() -> u64 {
    let mut reg_r = 0; 
    unsafe {
        core::arch::asm!("mrs {}, MPIDR_EL1", out(reg) reg_r);
    }    
    reg_r
}

/// Converts MPIDR to CPU ID
pub(crate) fn mpidr2cpuid(mpidr: u64) -> usize {
    // RK3588
    //((mpidr >> 8) & 0xff) as usize

    // Qemu
    //(mpidr & 0xffffff & 0xff) as usize

    // PhytiumPi
    match (mpidr & 0xfff) {
        CORE0_AFF => 0,
        CORE1_AFF => 1,
        CORE2_AFF => 2,
        CORE3_AFF => 3,
        _ => {
            error!("Failed to get PhytiumPi CPU Id from mpidr={:#x}", mpidr);
            0
        }
    }
}

pub(crate) fn get_cpu_id() -> usize {
    let mpidr = read_mpidr();
    mpidr2cpuid(mpidr)
}

/// Data Synchronization Barrier
pub fn DSB() {
    unsafe {
        core::arch::asm!("dsb sy");
    }    
}

// pseudo assembler instructions 
pub fn MFCPSR() -> u32{
    let mut rval: u32 = 0;
unsafe {
    asm!("mrs {0:x}, DAIF", out(reg) rval);
}
    rval
}

    
pub fn MTCPSR(val: u32) {
unsafe {
    asm!("msr DAIF, {0:x}", in(reg) val);
}
}

pub fn MTCPDC_CIVAC(adr: u64) {
    unsafe {
        asm!("dc CIVAC, {}", in(reg) adr);
    }
}

/// CACHE of PhytiumPi
pub const CACHE_LINE_ADDR_MASK: u64 = 0x3F;
pub const CACHE_LINE: u64 = 64;

/// Mask IRQ and FIQ interrupts in cpsr
pub const IRQ_FIQ_MASK: u32 = 0xC0;

/// dc civac, virt_addr 通过虚拟地址清除和无效化cache
/// adr: 64bit start address of the range to be invalidated.
/// len: Length of the range to be invalidated in bytes.
pub fn FCacheDCacheInvalidateRange(mut adr: u64, len: u64)
{
   let end: u64 = adr + len;
   adr = adr & (!CACHE_LINE_ADDR_MASK);
   let currmask: u32 = MFCPSR();
   MTCPSR(currmask | IRQ_FIQ_MASK);
   if (len != 0) 
   {
       while adr < end {
           MTCPDC_CIVAC(adr); /* Clean and Invalidate data cache by address to Point of Coherency */
           adr += CACHE_LINE;
       }   
   }   
   /* Wait for invalidate to complete */
   DSB();
   MTCPSR(currmask);
}

use aarch64_cpu::registers::{CNTVCT_EL0, CNTFRQ_EL0, Readable};
use alloc::boxed::Box;

#[inline]
pub fn now_tsc() -> u64 {
    CNTVCT_EL0.get()
}

#[inline]
pub fn timer_freq() -> u64 {
    CNTFRQ_EL0.get() as u64
}

// 纳秒(ns)
#[inline]
pub fn now_ns() -> u64 {
    let freq = timer_freq();
    now_tsc() * (1_000_000_000 / freq)
}

pub fn ticks_to_nanos(ticks: u64) -> u64 {
    let freq = timer_freq();
    ticks * (1_000_000_000 / freq)
}

// 微秒(us)
pub fn usdelay(us: u64) {
    let mut current_ticks: u64 = now_tsc();
    let delay2 = current_ticks + us * (timer_freq() / 1000000);

    while delay2 >= current_ticks {
        core::hint::spin_loop();
        current_ticks = now_tsc();
    }

    trace!("usdelay current_ticks: {}", current_ticks);
}

// 毫秒(ms)
#[allow(unused)]
pub fn msdelay(ms: u64) {
    usdelay(ms * 1000);
}

#[linkage = "weak"]
#[export_name = "phys_to_virt"]
pub fn phys_to_virt(addr: usize) -> usize {
    addr
}

/// 申请DMA内存页
#[linkage = "weak"]
#[export_name = "dma_alloc_coherent"]
pub fn dma_alloc_coherent(pages: usize) -> (usize, usize) {
    let paddr: Box<[u32]> = if pages == 1 {
        Box::new([0; 1024]) // 4096
    } else if pages == 8 {
        Box::new([0; 1024 * 8]) // 4096
    } else {
        warn!("Alloc {} pages failed", pages);
        Box::new([0; 1024])
    };

    let len = paddr.len();

    let paddr = Box::into_raw(paddr) as *const u32 as usize;
    //let vaddr = phys_to_virt(paddr);
    let vaddr = paddr;
    trace!("dma alloc paddr: {:#x}, len={}", paddr, len);

    (vaddr, paddr)
}

/// 释放DMA内存页
#[linkage = "weak"]
#[export_name = "dma_free_coherent"]
pub fn dma_free_coherent(vaddr: usize, pages: usize) {
    let palloc = vaddr as *mut [u32; 1024];
    unsafe{ drop(Box::from_raw(palloc)); }
}

/// 请求分配irq
#[linkage = "weak"]
#[export_name = "dma_request_irq"]
pub fn dma_request_irq(irq: usize, handler: fn(u64)) {
    unimplemented!()
}

// 路由中断到指定的cpu，或所有的cpu
//pub(crate) fn InterruptSetTargetCpus() {}
