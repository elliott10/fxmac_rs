use core::cmp::min;
use core::ptr::{null_mut, null};
use core::slice::from_raw_parts_mut;

use alloc::boxed::Box;
use alloc::vec::Vec;
use log::*;
use crate::fxmac_const::*;
use crate::fxmac_phy::*;
use crate::fxmac::*;

// fxmac_lwip_port.h
pub const FXMAX_RX_BDSPACE_LENGTH: usize =   0x20000; /* default set 128KB*/
pub const FXMAX_TX_BDSPACE_LENGTH: usize =   0x20000; /* default set 128KB*/


pub const FXMAX_RX_PBUFS_LENGTH: usize = 128;
pub const FXMAX_TX_PBUFS_LENGTH: usize = 128;

pub const FXMAX_MAX_HARDWARE_ADDRESS_LENGTH: usize =6;

/* configuration */
pub const FXMAC_LWIP_PORT_CONFIG_JUMBO: u32 = BIT(0);
pub const FXMAC_LWIP_PORT_CONFIG_MULTICAST_ADDRESS_FILITER: u32 = BIT(1); /* Allow multicast address filtering  */
pub const FXMAC_LWIP_PORT_CONFIG_COPY_ALL_FRAMES: u32 =BIT(2); /* enable copy all frames */
pub const FXMAC_LWIP_PORT_CONFIG_CLOSE_FCS_CHECK: u32 =BIT(3); /* close fcs check */
pub const FXMAC_LWIP_PORT_CONFIG_UNICAST_ADDRESS_FILITER: u32 =BIT(5); /* Allow unicast address filtering  */

/* Phy */
pub const FXMAC_PHY_SPEED_10M: u32 =   10;
pub const FXMAC_PHY_SPEED_100M: u32 =   100;
pub const FXMAC_PHY_SPEED_1000M: u32 =   1000;
pub const FXMAC_PHY_SPEED_10G: u32 =   10000;


pub const FXMAC_PHY_HALF_DUPLEX: u32 =  0;
pub const FXMAC_PHY_FULL_DUPLEX: u32 =  1;

pub const FXMAC_RECV_MAX_COUNT: u32 =10;

/*  frame queue */
pub const PQ_QUEUE_SIZE: u32 =4096;

/// Transmit buffer descriptor status words offset
/// word 0/addr of BDs
pub const FXMAC_BD_ADDR_OFFSET: u32 = 0;
/// word 1/status of BDs, 4 bytes
pub const FXMAC_BD_STAT_OFFSET: u32 = 4;
/// word 2/addr of BDs
pub const FXMAC_BD_ADDR_HI_OFFSET: u32 = 1 << 3;

/// RX Used bit
pub const FXMAC_RXBUF_NEW_MASK: u32 = 1<<0;
/// RX Wrap bit, last BD
pub const FXMAC_RXBUF_WRAP_MASK: u32 = 1<<1;
/// Mask for address
pub const FXMAC_RXBUF_ADD_MASK: u32 = GENMASK(31, 2);
// FXMAC_RXBUF_ADD_MASK=0xff_ff_ff_fc

/// TX Used bit
pub const FXMAC_TXBUF_USED_MASK: u32 = 1<<31;
/// TX Wrap bit, last descriptor
pub const FXMAC_TXBUF_WRAP_MASK: u32 = 1<<30;

pub const ULONG64_HI_MASK: u64 = 0xFFFFFFFF_00000000;
pub const ULONG64_LO_MASK: u64 = !ULONG64_HI_MASK;

/// Byte alignment of BDs
pub const BD_ALIGNMENT: u64 = FXMAC_DMABD_MINIMUM_ALIGNMENT*2; // 128
pub const FXMAC_DMABD_MINIMUM_ALIGNMENT: u64 = 64;
pub const FXMAC_BD_NUM_WORDS: usize = 4;
pub type FXmacBd = [u32; FXMAC_BD_NUM_WORDS];
// sizeof(uintptr)=8
pub type uintptr = u64;

/// Enable tail Register
//pub const FXMAC_TAIL_ENABLE: u32 = 0xe7c;

/// send direction
pub const FXMAC_SEND: u32 = 1;
/// receive direction
pub const FXMAC_RECV: u32 = 2;

#[derive(Copy, Clone)]
#[repr(C)]
pub struct FXmacBdRing {
    pub phys_base_addr: uintptr,
    pub base_bd_addr: uintptr,
    pub high_bd_addr: uintptr,
    pub length: u32,
    pub run_state: u32,
    pub separation: u32,
    pub free_head: *mut FXmacBd,
    pub pre_head: *mut FXmacBd,
    pub hw_head: *mut FXmacBd,
    pub hw_tail: *mut FXmacBd,
    pub post_head: *mut FXmacBd,
    pub bda_restart: *mut FXmacBd,
    pub hw_cnt: u32,
    pub pre_cnt: u32,
    pub free_cnt: u32,
    pub post_cnt: u32,
    pub all_cnt: u32,
}

impl Default for FXmacBdRing {
    fn default() -> Self {
        Self {
            phys_base_addr: 0,
            base_bd_addr: 0,
            high_bd_addr: 0,
            length: 0,
            run_state: 0,
            separation: 0,
            free_head: null_mut(),
            pre_head: null_mut(),
            hw_head: null_mut(),
            hw_tail: null_mut(),
            post_head: null_mut(),
            bda_restart: null_mut(),
            hw_cnt: 0,
            pre_cnt: 0,
            free_cnt: 0,
            post_cnt: 0,
            all_cnt: 0,
        }
    }
}

pub struct FXmacNetifBuffer
{
    // 作为FXmacBdRing的基地址，并设置成一串多个BD
    pub rx_bdspace: Box<[u8; FXMAX_RX_BDSPACE_LENGTH]>, //  aligned(256); 接收bd 缓冲区
    pub tx_bdspace: Box<[u8; FXMAX_RX_BDSPACE_LENGTH]>, //  aligned(256); 发送bd 缓冲区

    // 保存收发数据包的内存基地址，收发数据包内存需要申请alloc
    pub rx_pbufs_storage: [uintptr; FXMAX_RX_PBUFS_LENGTH],
    pub tx_pbufs_storage: [uintptr; FXMAX_TX_PBUFS_LENGTH],
}

impl Default for FXmacNetifBuffer {
    fn default() -> Self {
        Self {
            rx_bdspace: Box::new([0; FXMAX_RX_BDSPACE_LENGTH]),
            tx_bdspace: Box::new([0; FXMAX_RX_BDSPACE_LENGTH]),
            rx_pbufs_storage: [0; FXMAX_RX_PBUFS_LENGTH],
            tx_pbufs_storage: [0; FXMAX_TX_PBUFS_LENGTH],
        }
    }
}

pub struct FXmacLwipPort
{
    pub buffer: FXmacNetifBuffer,
    // configuration
    pub feature: u32,
    pub hwaddr: [u8; FXMAX_MAX_HARDWARE_ADDRESS_LENGTH],
    // Indicating how many receive interrupts have been triggered
    pub recv_flg: u32,
}

pub fn fxmac_bd_read(bd_ptr: u64, offset: u32) -> u32 {
    read_reg((bd_ptr + offset as u64) as *const u32)
}
pub fn fxmac_bd_write(bd_ptr: u64, offset: u32, data: u32)
{
    // uintptr: u64
    write_reg((bd_ptr + offset as u64) as *mut u32, data);
}
    
/// FXmacBdSetRxWrap
/// Set this bit to mark the last descriptor in the receive buffer descriptor list.
fn FXmacBdSetRxWrap(mut bdptr: u64) {
   bdptr += FXMAC_BD_ADDR_OFFSET as u64;
   let temp_ptr = bdptr as *mut u32;
   if !temp_ptr.is_null() {
       let mut data_value_rx: u32 = unsafe{*temp_ptr};
       data_value_rx |= FXMAC_RXBUF_WRAP_MASK;
       unsafe {
        temp_ptr.write_volatile(data_value_rx);
       }
   }
}

/// FXmacBdSetTxWrap
/// Sets this bit to mark the last descriptor in the transmit buffer descriptor list.
fn FXmacBdSetTxWrap(mut bdptr: u64) {
   bdptr += FXMAC_BD_STAT_OFFSET as u64;
   let temp_ptr = bdptr as *mut u32;
   if !temp_ptr.is_null() {
       let mut data_value_tx: u32 = unsafe{*temp_ptr};
       data_value_tx |= FXMAC_TXBUF_WRAP_MASK;
       unsafe {
        temp_ptr.write_volatile(data_value_tx);
       }
   }
}

/// Reset BD ring head and tail pointers.
fn FXmacBdringPtrReset(ring_ptr: &mut FXmacBdRing, virtaddrloc: *mut FXmacBd)
{
   ring_ptr.free_head = virtaddrloc;
   ring_ptr.pre_head = virtaddrloc;
   ring_ptr.hw_head = virtaddrloc;
   ring_ptr.hw_tail = virtaddrloc;
   ring_ptr.post_head = virtaddrloc;
}

/// Set the BD's address field (word 0).
fn fxmac_bd_set_address_rx(bd_ptr: u64, addr: u64) {
    fxmac_bd_write((bd_ptr), FXMAC_BD_ADDR_OFFSET,
                   ((fxmac_bd_read(bd_ptr, FXMAC_BD_ADDR_OFFSET) & !FXMAC_RXBUF_ADD_MASK) |
                    (addr & ULONG64_LO_MASK) as u32));
                    
    fxmac_bd_write(bd_ptr, FXMAC_BD_ADDR_HI_OFFSET, ((addr & ULONG64_HI_MASK) >> 32) as u32);

    // For aarch64
}

/// Set the BD's address field (word 0).
fn fxmac_bd_set_address_tx(bd_ptr: u64, addr: u64) {
    fxmac_bd_write((bd_ptr), FXMAC_BD_ADDR_OFFSET, (addr & ULONG64_LO_MASK) as u32);
                   
    fxmac_bd_write(bd_ptr, FXMAC_BD_ADDR_HI_OFFSET, ((addr & ULONG64_HI_MASK) >> 32) as u32);

    // For aarch64
}

/// 将bdptr参数向前移动任意数量的bd，绕到环的开头。
fn FXMAC_RING_SEEKAHEAD(ring_ptr: &mut FXmacBdRing, mut bdptr: *mut FXmacBd, num_bd: u32)
{
        // 第一个free BD
        // bdptr = free_head

        let mut addr: u64 = bdptr as u64;
        addr += (ring_ptr.separation * num_bd) as u64;

        if (addr > ring_ptr.high_bd_addr) || (bdptr as u64 > addr)
        {
            addr -= ring_ptr.length as u64;
        }
        bdptr = addr as *mut FXmacBd;
}

/*
pub fn FXmacBdRingCreate() -> u32 {
    
    // 为DMA构建环形缓冲区内存

    let alloc_tx_ring_pages =
        ((MACB_TX_RING_SIZE * DMA_DESC_SIZE) + (M::PAGE_SIZE - 1)) / M::PAGE_SIZE;
    let alloc_rx_ring_pages =
        ((MACB_RX_RING_SIZE * DMA_DESC_SIZE) + (M::PAGE_SIZE - 1)) / M::PAGE_SIZE;
    let tx_ring_dma = M::dma_alloc_coherent(alloc_tx_ring_pages);
    let rx_ring_dma = M::dma_alloc_coherent(alloc_rx_ring_pages);

    let tx_ring = unsafe {
        slice::from_raw_parts_mut(
            phys_to_virt(tx_ring_dma) as *mut DmaDesc,
            MACB_TX_RING_SIZE * DMA_DESC_SIZE / size_of::<DmaDesc>(), // 4096/16 = 256 个 dma_desc ?
        )
    };

    let rx_ring = unsafe {
        slice::from_raw_parts_mut(
            phys_to_virt(rx_ring_dma) as *mut DmaDesc,
            MACB_RX_RING_SIZE * DMA_DESC_SIZE / size_of::<DmaDesc>(),
        )
    };

    let mut send_buffers = Vec::with_capacity(tx_ring.len());
    let mut recv_buffers = Vec::with_capacity(rx_ring.len());

    // 一起申请所有RX内存
    let alloc_rx_buffer_pages =
        ((MACB_RX_RING_SIZE * buffer_size) + (M::PAGE_SIZE - 1)) / M::PAGE_SIZE;
    let rx_buffer_dma: usize = M::dma_alloc_coherent(alloc_rx_buffer_pages);

    info!("Set ring desc buffer for RX");
    let mut count = 0;
    let mut paddr: u64 = rx_buffer_dma as u64;
    for i in 0..MACB_RX_RING_SIZE {
        if i == MACB_RX_RING_SIZE - 1 {
            paddr |= 1 << MACB_RX_WRAP_OFFSET;
        }

        if (config.hw_dma_cap & HW_DMA_CAP_64B) != 0 {
            count = i * 2;
            rx_ring[count + 1].addr = upper_32_bits(paddr); // Fill DmaDesc64.addrh
        } else {
            count = i;
        }
        rx_ring[count].ctrl = 0;
        rx_ring[count].addr = lower_32_bits(paddr);

        recv_buffers.push(phys_to_virt(paddr as usize));
        paddr += buffer_size as u64;

        // sync memery, fence指令？
    }
    flush_dcache_range(); // RX dma ring and buffer

    // 一起申请所有TX内存
    let alloc_tx_buffer_pages =
        ((MACB_TX_RING_SIZE * buffer_size) + (M::PAGE_SIZE - 1)) / M::PAGE_SIZE;
    let tx_buffer_dma: usize = M::dma_alloc_coherent(alloc_tx_buffer_pages);
    info!("Set ring desc buffer for TX");
    count = 0;
    paddr = tx_buffer_dma as u64;
    for i in 0..MACB_TX_RING_SIZE {
        if (config.hw_dma_cap & HW_DMA_CAP_64B) != 0 {
            count = i * 2;
            tx_ring[count + 1].addr = upper_32_bits(paddr); // Fill DmaDesc64.addrh
        } else {
            count = i;
        }
        tx_ring[count].addr = lower_32_bits(paddr);

        if i == MACB_TX_RING_SIZE - 1 {
            tx_ring[count].ctrl = (1 << MACB_TX_USED_OFFSET) | (1 << MACB_TX_WRAP_OFFSET);
        } else {
            tx_ring[count].ctrl = (1 << MACB_TX_USED_OFFSET);
        }
        // Used – must be zero for the controller to read data to the transmit buffer.
        // The controller sets this to one for the first buffer of a frame once it has been successfully transmitted.
        // Software must clear this bit before the buffer can be used again.

        send_buffers.push(phys_to_virt(paddr as usize));
        paddr += buffer_size as u64;
    }
    flush_dcache_range(); // TX dma ring

}
*/


pub fn FXmacInitDma(instance_p: &mut FXmac) -> u32
{
    //let mut rxbd: FXmacBd = [0; FXMAC_BD_NUM_WORDS];

    let rxringptr: &mut FXmacBdRing = &mut instance_p.rx_bd_queue.bdring;
    let txringptr: &mut FXmacBdRing = &mut instance_p.tx_bd_queue.bdring;
    info!("rxringptr: {:p}", rxringptr);
    info!("txringptr: {:p}", txringptr);
    info!("rx_bdspace: {:p}", &instance_p.lwipport.buffer.rx_bdspace);
    info!("tx_bdspace: {:p}", &instance_p.lwipport.buffer.tx_bdspace);

    // Setup RxBD space.
    // 对BD域清零
    //let mut bdtemplate: FXmacBd = unsafe { core::mem::zeroed() };

    // FXmacBd: The FXMAC_Bd is the type for buffer descriptors (BDs).
    let mut bdtemplate: FXmacBd = [0; FXMAC_BD_NUM_WORDS];

    // Create the RxBD ring, bdspace地址必须对齐128
    // 创建收包的环形缓冲区
    let mut status: u32 = FXmacBdRingCreate(rxringptr, &instance_p.lwipport.buffer.rx_bdspace as *const _ as u64, &instance_p.lwipport.buffer.rx_bdspace as *const _ as u64, BD_ALIGNMENT, FXMAX_RX_PBUFS_LENGTH as u32);

    // 将给定的BD, 克隆到list中的每个BD上
    status = FXmacBdRingClone(rxringptr, &mut bdtemplate, FXMAC_RECV);


    let mut bdtemplate: [u32; FXMAC_BD_NUM_WORDS] = [0; FXMAC_BD_NUM_WORDS];

    // FXMAC_BD_SET_STATUS()
    // Set the BD's Status field (word 1).
    fxmac_bd_write((&bdtemplate as *const _ as u64), FXMAC_BD_STAT_OFFSET, 
        fxmac_bd_read((&bdtemplate as *const _ as u64), FXMAC_BD_STAT_OFFSET) | (FXMAC_TXBUF_USED_MASK));

    /* Create the TxBD ring */
    status = FXmacBdRingCreate(txringptr, &instance_p.lwipport.buffer.tx_bdspace as *const _ as u64, &instance_p.lwipport.buffer.tx_bdspace as *const _ as u64, BD_ALIGNMENT, FXMAX_TX_PBUFS_LENGTH as u32);

    /* We reuse the bd template, as the same one will work for both rx and tx. */
    status = FXmacBdRingClone(txringptr, &mut bdtemplate, FXMAC_SEND);

    /*
     * Allocate RX descriptors, 1 RxBD at a time.
     */
    info!("Allocate RX descriptors, 1 RxBD at a time.");
    for i in 0..FXMAX_RX_PBUFS_LENGTH
    {
        let max_frame_size = if (instance_p.lwipport.feature & FXMAC_LWIP_PORT_CONFIG_JUMBO) != 0
        { info!("FXMAC_LWIP_PORT_CONFIG_JUMBO"); FXMAC_MAX_FRAME_SIZE_JUMBO } else { info!("NO CONFIG_JUMBO"); FXMAC_MAX_FRAME_SIZE };
        let alloc_rx_buffer_pages = (max_frame_size as usize + (PAGE_SIZE - 1)) / PAGE_SIZE;

        let (mut rx_mbufs_vaddr, mut rx_mbufs_dma) = crate::utils::dma_alloc_coherent(alloc_rx_buffer_pages);

        let rxbd: *mut FXmacBd = null_mut();
//let my_speed: Box<i32> = Box::new(88);
//rxbd = Box::into_raw(my_speed);
// OR
//let mut my_speed: i32 = 88;
//rxbd = &mut my_speed;

        // 在BD list中预留待设置的BD
        status = FXmacBdRingAlloc(rxringptr, 1, rxbd);
        if (status != 0)
        {
            error!("FXmacInitDma: Error allocating RxBD");
            return status;
        }

        // 将一组BD排队到之前由FXmacBdRingAlloc分配了的硬件上
        status = FXmacBdRingToHw(rxringptr, 1, rxbd);

        let bdindex = FXMAC_BD_TO_INDEX(rxringptr, rxbd as u64);

        let mut temp = rxbd as *mut u32;

        let mut v = 0;
        if bdindex == (FXMAX_RX_PBUFS_LENGTH - 1) as u32 {
            // Marks last descriptor in receive buffer descriptor list
            v |= FXMAC_RXBUF_WRAP_MASK;
        }
        unsafe{
        temp.write_volatile(v);
        // Clear word 1 in  descriptor
        temp.add(1).write_volatile(0);
        }
        crate::utils::DSB();

        // dc civac, virt_addr 通过虚拟地址清除和无效化cache
        crate::utils::FCacheDCacheInvalidateRange(rx_mbufs_vaddr as u64, max_frame_size as u64);

        // Set the BD's address field (word 0)
        // void *payload; 指向数据区域的指针，指向该pbuf管理的数据区域起始地址，可以是ROM或者RAM中的某个地址
        fxmac_bd_set_address_rx(rxbd as u64, rx_mbufs_dma as u64);

        instance_p.lwipport.buffer.rx_pbufs_storage[bdindex as usize] = rx_mbufs_vaddr as u64;
    }

    FXmacSetQueuePtr(instance_p.rx_bd_queue.bdring.phys_base_addr, 0, FXMAC_RECV);
    
    FXmacSetQueuePtr(instance_p.tx_bd_queue.bdring.phys_base_addr, 0, FXMAC_SEND);
    
    let FXMAC_TAIL_QUEUE = |queue: u64| 0x0e80 + (queue << 2);
    if (instance_p.config.caps & FXMAC_CAPS_TAILPTR) != 0
    {   
        write_reg((instance_p.config.base_address + FXMAC_TAIL_QUEUE(0)) as *mut u32, (1<<31) | 0);
    }

    0
}

fn FXMAC_BD_TO_INDEX (ringptr: &mut FXmacBdRing, bdptr: u64) -> u32 {
    ( (bdptr - ringptr.base_bd_addr as u64) / ringptr.separation as u64 ) as u32
}

/// 从bptr的列表中，获取下一个BD
fn FXMAC_BD_RING_NEXT(ring_ptr: &mut FXmacBdRing, bd_ptr: *mut FXmacBd) -> *mut FXmacBd {
        if bd_ptr as u64 >= ring_ptr.high_bd_addr {
            ring_ptr.base_bd_addr as *mut FXmacBd
        }else{
            (bd_ptr as u64 + ring_ptr.separation as u64) as *mut FXmacBd
        }
}

/// Create the RxBD ring
/// 创建收包的环形缓冲区
pub fn FXmacBdRingCreate(ring_ptr: &mut FXmacBdRing, phys_addr: u64, virt_addr: u64, alignment: u64, bd_count: u32) -> u32
    {
    // uintptr: u64

        // alignment=128, bd_count=128
       //let alignment = BD_ALIGNMENT;
       //let bd_count = FXMAX_RX_PBUFS_LENGTH;

       //let virt_addr_loc = instance_p.buffer.rx_bdspace;
       let virt_addr_loc: u64 = virt_addr;

 ring_ptr.all_cnt = 0;
 ring_ptr.free_cnt = 0;
 ring_ptr.hw_cnt = 0;
 ring_ptr.pre_cnt = 0;
 ring_ptr.post_cnt = 0;

       // 该地址必须对齐alignment=128
       assert!((virt_addr_loc % alignment) == 0);
       assert!(bd_count > 0);

       // 相邻BD之间隔多少bytes
       ring_ptr.separation = size_of::<FXmacBd>() as u32;

// Initial ring setup:
//  - Clear the entire space
//  - Setup each BD's BDA field with the physical address of the next BD
    let rxringptr = unsafe { from_raw_parts_mut(virt_addr_loc as *mut FXmacBd, bd_count as usize) };
    rxringptr.fill([0; FXMAC_BD_NUM_WORDS]);

    let mut bd_virt_addr = virt_addr_loc;
    for i in 1..bd_count {
        bd_virt_addr += ring_ptr.separation as u64;
    }

    // Setup and initialize pointers and counters

    // BD list中第一个的虚拟地址
    ring_ptr.base_bd_addr = virt_addr_loc;
    // BD list中最后一个的虚拟地址
    ring_ptr.high_bd_addr = bd_virt_addr;
    // ring的总大小bytes
    ring_ptr.length = (ring_ptr.high_bd_addr - ring_ptr.base_bd_addr) as u32 + ring_ptr.separation;
    // 第一个free BD
    ring_ptr.free_head = virt_addr_loc as *mut FXmacBd;
    // 第一个pre-work BD
    ring_ptr.pre_head = virt_addr_loc as *mut FXmacBd;

    // 可分配的free BD数量
    ring_ptr.free_cnt = bd_count;
    // 总共的BD数
    ring_ptr.all_cnt = bd_count;

    ring_ptr.run_state = FXMAC_DMA_SG_IS_STOPED as u32;
    ring_ptr.phys_base_addr = phys_addr;
    ring_ptr.hw_head = virt_addr_loc as *mut FXmacBd;
    ring_ptr.hw_tail = virt_addr_loc as *mut FXmacBd;
    ring_ptr.post_head = virt_addr_loc as *mut FXmacBd;
    ring_ptr.bda_restart = phys_addr as *mut FXmacBd;

    0
    }

/// 将给定的BD, 克隆到list中的每个BD上
pub fn FXmacBdRingClone(ring_ptr: &mut FXmacBdRing, src_bd_ptr: &mut FXmacBd, direction: u32) -> u32
{
    // Can't do this function with some of the BDs in use
    assert!(ring_ptr.free_cnt == ring_ptr.all_cnt);

    let mut cur_bd = ring_ptr.base_bd_addr;
    for i in 0..ring_ptr.all_cnt {
        // 将所有BD逐个复制成bdtemplate
        //let cur_bd_slice = unsafe { from_raw_parts_mut(cur_bd as *mut FXmacBd, 1) };
        //cur_bd_slice.copy_from_slice(src_bd_ptr);
        unsafe{ *(cur_bd as *mut FXmacBd) }.copy_from_slice(src_bd_ptr);
        
        cur_bd += ring_ptr.separation as u64;
    }
    cur_bd -= ring_ptr.separation as u64;

    // 对最后一个BD描述符进行位标记
    if direction == FXMAC_RECV {
        FXmacBdSetRxWrap(cur_bd);
    } else {
        FXmacBdSetTxWrap(cur_bd);
    }

    0
}


/// 在BD list中预留待设置的BD
pub fn FXmacBdRingAlloc(ring_ptr: &mut FXmacBdRing, num_bd: u32, mut bd_set_ptr: *mut FXmacBd) -> u32 {
/*
let num_bd = 1;
let bd_set_ptr = &rxbd;
*/

    if ring_ptr.free_cnt < num_bd {
        error!("No Enough free BDs available for the request: {}", num_bd);
        4
    } else {
        // 获取待设置的BD，并向前移动free BD
        bd_set_ptr = ring_ptr.free_head as *mut FXmacBd;
        
        let b = ring_ptr.free_head;
        FXMAC_RING_SEEKAHEAD(ring_ptr, ring_ptr.free_head, num_bd);
        assert!(b as usize != ring_ptr.free_head as usize);

        ring_ptr.free_cnt -= num_bd;
        ring_ptr.pre_cnt += num_bd;

        0
    }
}


/// 将一组BD排队到之前由FXmacBdRingAlloc分配了的硬件上
pub fn FXmacBdRingToHw(ring_ptr: &mut FXmacBdRing, num_bd: u32, bd_set_ptr: *mut FXmacBd) -> u32 {
    // uintptr: u64

    let mut cur_bd_ptr: *mut FXmacBd = bd_set_ptr;
    for i in 0..num_bd {
        cur_bd_ptr = FXMAC_BD_RING_NEXT(ring_ptr, cur_bd_ptr);
    }

    FXMAC_RING_SEEKAHEAD(ring_ptr, ring_ptr.pre_head, num_bd);

    ring_ptr.pre_cnt -= num_bd;
    ring_ptr.hw_tail = cur_bd_ptr;
    ring_ptr.hw_cnt += num_bd;

    0
}

pub fn FXmacBdRingFromHwRx(ring_ptr: &mut FXmacBdRing, bd_limit: usize, mut bd_set_ptr: *mut FXmacBd) -> u32 {

    let mut cur_bd_ptr: *mut FXmacBd = ring_ptr.hw_head;
    let mut status: u32 = 0;
    let mut bd_str: u32 = 0;
    let mut bd_count: u32 = 0;
    let mut bd_partial_count: u32 = 0;

    if ring_ptr.hw_cnt == 0 {
        warn!("No BDs in RX work group, there's nothing to search");

        bd_set_ptr = null_mut();
        
        status = 0;
    }else{
         /* Starting at hw_head, keep moving forward in the list until:
         *  - A BD is encountered with its new/used bit set which means hardware has completed processing of that BD.
         *  - ring_ptr->hw_tail is reached and ring_ptr->hw_cnt is reached.
         *  - The number of requested BDs has been processed
         */
        while (bd_count as usize) < bd_limit {

            // Read the status

            bd_str = fxmac_bd_read(cur_bd_ptr as u64, FXMAC_BD_STAT_OFFSET);

            // FXMAC_BD_IS_RX_NEW, Determine the new bit of the receive BD
            let bd_is_rx_new = (fxmac_bd_read(cur_bd_ptr as u64, FXMAC_BD_ADDR_OFFSET) & FXMAC_RXBUF_NEW_MASK) != 0;
            if !bd_is_rx_new == true {
                break;
            }

            bd_count += 1;

            /* hardware has processed this BD so check the "last" bit. If
             * it is clear, then there are more BDs for the current packet.
             * Keep a count of these partial packet BDs.
             */
            if (bd_str & FXMAC_RXBUF_EOF_MASK) != 0 {
                bd_partial_count = 0;
            }
            else
            {
                bd_partial_count+=1;
            }

            // Move on to next BD in work group
            cur_bd_ptr = FXMAC_BD_RING_NEXT(ring_ptr, cur_bd_ptr);

        }

        // 减去找到的任何partial packet BDs
        bd_count -= bd_partial_count;

        /* If bd_count is non-zero then BDs were found to return. Set return
         * parameters, update pointers and counters, return success
         */
        if bd_count > 0 {
            bd_set_ptr = ring_ptr.hw_head;

            ring_ptr.hw_cnt -= bd_count;
            ring_ptr.post_cnt += bd_count;
            FXMAC_RING_SEEKAHEAD(ring_ptr, ring_ptr.hw_head, bd_count);

            info!("FXmacBdRingFromHwRx, Found BD={}", bd_count);
            status = bd_count;
        } else {
            bd_set_ptr = null_mut();
            status = 0;
        }
    }
    status
}


/// @name: FXmacBdRingFromHwTx
/// @msg:  Returns a set of BD(s) that have been processed by hardware. The returned
/// BDs may be examined to determine the outcome of the DMA transaction(s).
/// Once the BDs have been examined, the user must call FXmacBdRingFree()
/// in the same order which they were retrieved here.
pub fn FXmacBdRingFromHwTx(ring_ptr: &mut FXmacBdRing, bd_limit: usize, mut bd_set_ptr: *mut FXmacBd) -> u32 {
    let mut bd_str: u32 = 0;
    let mut bd_count: u32 = 0;
    let mut bd_partial_count: u32 = 0;
    let mut status: u32 = 0;

    let mut bd_limitLoc: u32 = bd_limit as u32;
    let mut cur_bd_ptr: *mut FXmacBd = ring_ptr.hw_head;

    /* If no BDs in work group, then there's nothing to search */
    if ring_ptr.hw_cnt == 0
    {
        warn!("No BDs in TX work group, then there's nothing to search");
        bd_set_ptr = null_mut();
        status = 0;
    } else {

        if bd_limitLoc > ring_ptr.hw_cnt
        {
            bd_limitLoc = ring_ptr.hw_cnt;
        }
        /* Starting at hw_head, keep moving forward in the list until:
         *  - A BD is encountered with its new/used bit set which means
         *    hardware has not completed processing of that BD.
         *  - ring_ptr->hw_tail is reached and ring_ptr->hw_cnt is reached.
         *  - The number of requested BDs has been processed
         */
        while bd_count < bd_limitLoc {
            // Read the status
                bd_str = fxmac_bd_read(cur_bd_ptr as u64, FXMAC_BD_STAT_OFFSET);

            if (bd_str & FXMAC_TXBUF_USED_MASK) != 0
            {
                bd_count += 1;
                bd_partial_count += 1;
            }

            /* hardware has processed this BD so check the "last" bit.
             * If it is clear, then there are more BDs for the current
             * packet. Keep a count of these partial packet BDs.
             */
            if (bd_str & FXMAC_TXBUF_LAST_MASK) != 0
            {
                bd_partial_count = 0;
            }

            /* Move on to next BD in work group */
            cur_bd_ptr = FXMAC_BD_RING_NEXT(ring_ptr, cur_bd_ptr);
        }

        /* Subtract off any partial packet BDs found */
        bd_count -= bd_partial_count;

        /* If bd_count is non-zero then BDs were found to return. Set return
         * parameters, update pointers and counters, return success
         */
        if bd_count > 0 {
            bd_set_ptr = ring_ptr.hw_head;

            ring_ptr.hw_cnt -= bd_count;
            ring_ptr.post_cnt += bd_count;
            FXMAC_RING_SEEKAHEAD(ring_ptr, ring_ptr.hw_head, bd_count);

            info!("FXmacBdRingFromHwTx, Found BD={}", bd_count);
            status = bd_count;
        } else {
            bd_set_ptr = null_mut();
            status = 0;
        }
    }

    status
}

pub fn FXmacLwipPortTx(instance: &mut FXmac, pbuf: Vec<Vec<u8>>) -> i32
{
    info!("TX transmit packets");
    // 发送网络包时注意屏蔽下中断

    // check if space is available to send
    let freecnt = (instance.tx_bd_queue.bdring).free_cnt;
    if freecnt <= 5 {
        info!("TX freecnt={}, let's process sent BDs", freecnt);
        //let txring = &mut (instance.tx_bd_queue.bdring);
        FXmacProcessSentBds(instance);
    }

    if (instance.tx_bd_queue.bdring).free_cnt != 0 {
        FXmacSgsend(instance, pbuf) as i32
    }else{
        error!(" TX packets dropped, no space");
        -3 // FREERTOS_XMAC_NO_VALID_SPACE
    }
}

/// 发包函数
pub fn FXmacSgsend(instance_p: &mut FXmac, p: Vec<Vec<u8>>) -> u32 {
    let mut status: u32 = 0;
    let mut bdindex: u32 = 0;
    let mut max_fr_size: u32 = 0;

    let mut last_txbd: *mut FXmacBd = null_mut();
    let txbdset: *mut FXmacBd = null_mut();
    let txring: &mut FXmacBdRing = &mut instance_p.tx_bd_queue.bdring;

    // Count the number of pbufs: p
    let n_pbufs: u32 = p.len() as u32;

    /* obtain as many BD's */
    status = FXmacBdRingAlloc(txring, n_pbufs, txbdset);

    let mut txbd: *mut FXmacBd = txbdset;
    for q in &p {
        bdindex = FXMAC_BD_TO_INDEX(txring, txbd as u64);

        if (instance_p.lwipport.buffer.tx_pbufs_storage[bdindex as usize] != 0)
        {
            panic!("PBUFS not available");
        }

        /* Send the data from the pbuf to the interface, one pbuf at a
           time. The size of the data in each pbuf is kept in the ->len
           variable. */
        //FCacheDCacheFlushRange((uintptr)q->payload, (uintptr)q->len);

        fxmac_bd_set_address_tx(txbd as u64, q.as_ptr() as u64);

        if (instance_p.lwipport.feature & FXMAC_LWIP_PORT_CONFIG_JUMBO) != 0
        {
            max_fr_size = FXMAC_MAX_FRAME_SIZE_JUMBO;
        }
        else
        {
            max_fr_size = FXMAC_MAX_FRAME_SIZE;
        }

        let t_txbd = txbd as u64;
        if q.len() > max_fr_size as usize
        {
            // FXMAC_BD_SET_LENGTH: 设置BD的发送长度（以bytes为单位）。每次BD交给硬件时,都必须设置该长度
            // FXMAC_BD_SET_LENGTH(txbd, max_fr_size & 0x3FFF);
            fxmac_bd_write(t_txbd, FXMAC_BD_STAT_OFFSET,
                ((fxmac_bd_read(t_txbd, FXMAC_BD_STAT_OFFSET) & !FXMAC_TXBUF_LEN_MASK) | (max_fr_size & 0x3FFF)));
        } else {
            // FXMAC_BD_SET_LENGTH(txbd, q->len & 0x3FFF);
        fxmac_bd_write(t_txbd, FXMAC_BD_STAT_OFFSET,
            ((fxmac_bd_read(t_txbd, FXMAC_BD_STAT_OFFSET) & !FXMAC_TXBUF_LEN_MASK) | (q.len() as u32 & 0x3FFF)));
        }

        instance_p.lwipport.buffer.tx_pbufs_storage[bdindex as usize] = q.as_ptr() as u64;

        // 增加该pbuf的引用计数。
        //pbuf_ref(q);

        last_txbd = txbd;

        // 告诉DMA当前包不是以该BD结束
        //FXMAC_BD_CLEAR_LAST(txbd);
        let t_txbd = txbd as u64;
        fxmac_bd_write(t_txbd, FXMAC_BD_STAT_OFFSET,
            fxmac_bd_read(t_txbd, FXMAC_BD_STAT_OFFSET) & !FXMAC_TXBUF_LAST_MASK );

        txbd = FXMAC_BD_RING_NEXT(txring, txbd);
    }
    // 告诉DMA 该BD标志着当前数据包的结束
    //FXMAC_BD_SET_LAST(last_txbd);
    let t_txbd = last_txbd as u64;
    fxmac_bd_write(t_txbd, FXMAC_BD_STAT_OFFSET,
        fxmac_bd_read(t_txbd, FXMAC_BD_STAT_OFFSET) | FXMAC_TXBUF_LAST_MASK );

    /* The bdindex always points to the first free_head in tx_bdrings */
    if (instance_p.config.caps & FXMAC_CAPS_TAILPTR) != 0
    {   
         bdindex = FXMAC_BD_TO_INDEX(txring, txbd as u64);
    }

    // The used bit for the 1st BD should be cleared at the end after clearing out used bits for other fragments.
    let mut txbd = txbdset;
    let FXMAC_BD_CLEAR_TX_USED = |bd_ptr: u64|
            fxmac_bd_write(bd_ptr, FXMAC_BD_STAT_OFFSET, fxmac_bd_read(bd_ptr, FXMAC_BD_STAT_OFFSET) & (!FXMAC_TXBUF_USED_MASK));

    for q in 1..p.len() {
        txbd = FXMAC_BD_RING_NEXT(txring, txbd);
        FXMAC_BD_CLEAR_TX_USED(txbd as u64);
        crate::utils::DSB();
    }
    FXMAC_BD_CLEAR_TX_USED(txbdset as u64); // 最后清第一个BD
    crate::utils::DSB();

    status = FXmacBdRingToHw(txring, n_pbufs, txbdset);

    let FXMAC_TAIL_QUEUE = |queue: u64| 0x0e80 + (queue << 2);
    if (instance_p.config.caps & FXMAC_CAPS_TAILPTR) != 0
    {   
        write_reg((instance_p.config.base_address + FXMAC_TAIL_QUEUE(0)) as *mut u32, (1 << 31) | bdindex);
    }

    // Start transmit
    let value = read_reg((instance_p.config.base_address + FXMAC_NWCTRL_OFFSET) as *const u32) | FXMAC_NWCTRL_STARTTX_MASK;
    write_reg((instance_p.config.base_address + FXMAC_NWCTRL_OFFSET) as *mut u32, value);

    0
}

/// 收包函数
pub fn FXmacRecvHandler(instance_p: &mut FXmac) -> Option<Vec<Vec<u8>>> {
    info!("RX receive packets");
    let mut recv_packets = Vec::new();

    let mut rxbdset: *mut FXmacBd = null_mut();
    //let rxring: &mut FXmacBdRing = &mut (instance_p.rx_bd_queue.bdring);

    /* If Reception done interrupt is asserted, call RX call back function
     to handle the processed BDs and then raise the according flag.*/
    let regval: u32 = read_reg((instance_p.config.base_address + FXMAC_RXSR_OFFSET) as *const u32);
    write_reg((instance_p.config.base_address + FXMAC_RXSR_OFFSET) as *mut u32, regval);

    loop {
        // Returns a set of BD(s) that have been processed by hardware.
        let bd_processed: u32 = FXmacBdRingFromHwRx(&mut instance_p.rx_bd_queue.bdring, FXMAX_RX_PBUFS_LENGTH, rxbdset);
        assert!(!rxbdset.is_null());
        if bd_processed <= 0
        {
            break;
        }

        let mut curbdptr: *mut FXmacBd = rxbdset;
        for k in 0..bd_processed {

    let rxring: &mut FXmacBdRing = &mut instance_p.rx_bd_queue.bdring;

            // Adjust the buffer size to the actual number of bytes received.
            let rx_bytes: u32 =
            if (instance_p.lwipport.feature & FXMAC_LWIP_PORT_CONFIG_JUMBO) != 0
            {
                //FXMAC_GET_RX_FRAME_SIZE(curbdptr)
                fxmac_bd_read(curbdptr as u64, FXMAC_BD_STAT_OFFSET) & 0x00003FFF
           
            } else {
                //FXMAC_BD_GET_LENGTH(curbdptr)
                fxmac_bd_read(curbdptr as u64, FXMAC_BD_STAT_OFFSET) & FXMAC_RXBUF_LEN_MASK
           
            };

            let bdindex: u32 = FXMAC_BD_TO_INDEX(rxring, curbdptr as u64);
            let mbuf = unsafe { from_raw_parts_mut(instance_p.lwipport.buffer.rx_pbufs_storage[bdindex as usize] as *mut u8, rx_bytes as usize) };
            info!("RX PKT {} <<<<<<<<<", rx_bytes);

            // Copy mbuf into a new Vec
            recv_packets.push(mbuf.to_vec());

           /*  
            The value of hash_match indicates the hash result of the received packet 
               0: No hash match 
               1: Unicast hash match 
               2: Multicast hash match
               3: Reserved, the value is not legal
           */
           // FXMAC_BD_GET_HASH_MATCH(bd_ptr)
            let mut hash_match: u32 = (fxmac_bd_read(curbdptr as u64, FXMAC_BD_STAT_OFFSET) & FXMAC_RXBUF_HASH_MASK) >> 29;
            debug!("hash_match is {:#x}", hash_match);
    
            // Invalidate RX frame before queuing to handle
            // L1 cache prefetch conditions on any architecture.
            crate::utils::FCacheDCacheInvalidateRange(instance_p.lwipport.buffer.rx_pbufs_storage[bdindex as usize], rx_bytes as u64);

            /* store it in the receive queue,
             * where it'll be processed by a different handler
             */
            //if (FXmacPqEnqueue(&instance_p->recv_q, (void *)p) < 0)

            //instance_p.lwipport.buffer.rx_pbufs_storage[bdindex as usize] = 0;

            // Just clear 64 bits header
            mbuf[..min(64, rx_bytes as usize)].fill(0);

            curbdptr = FXMAC_BD_RING_NEXT(rxring, curbdptr);
        }

        // free up the BD's
        FXmacBdRingFree(&mut instance_p.rx_bd_queue.bdring, bd_processed);

        SetupRxBds(instance_p);
    }

    if recv_packets.len() > 0 {
        Some(recv_packets)
    } else {
        None
    }
}

pub fn SetupRxBds(instance_p: &mut FXmac) {
    let rxring: &mut FXmacBdRing = &mut instance_p.rx_bd_queue.bdring;

    let mut status: u32 = 0;
    let mut rxbd: *mut FXmacBd = null_mut();

    // let alloc_tx_buffer_pages: usize = ((TX_RING_SIZE * MBUF_SIZE) + (PAGE_SIZE - 1)) / PAGE_SIZE;
    // let alloc_rx_buffer_pages: usize = ((RX_RING_SIZE * MBUF_SIZE) + (PAGE_SIZE - 1)) / PAGE_SIZE;

    let mut freebds: u32 = rxring.free_cnt;

    while freebds > 0 {
        freebds -= 1;

        let max_frame_size = if (instance_p.lwipport.feature & FXMAC_LWIP_PORT_CONFIG_JUMBO) != 0 {
            FXMAC_MAX_FRAME_SIZE_JUMBO
        } else {
            FXMAC_MAX_FRAME_SIZE
        };
        let alloc_rx_buffer_pages: usize = (max_frame_size as usize + (PAGE_SIZE - 1)) / PAGE_SIZE;

        let (mut rx_mbufs_vaddr, mut rx_mbufs_dma) = crate::utils::dma_alloc_coherent(alloc_rx_buffer_pages);

        status = FXmacBdRingAlloc(rxring, 1, rxbd);

        status = FXmacBdRingToHw(rxring, 1, rxbd);


        crate::utils::FCacheDCacheInvalidateRange(rx_mbufs_vaddr as u64, max_frame_size as u64);

        let bdindex: u32 = FXMAC_BD_TO_INDEX(rxring, rxbd as u64);
        let mut temp = rxbd as *mut u32;
        let mut v = 0;
        if bdindex == (FXMAX_RX_PBUFS_LENGTH - 1) as u32 {
            // Mask last descriptor in receive buffer list
            v = 0x2;
        }
        unsafe {
        temp.write_volatile(v);
        // Clear word 1 in  descriptor
        temp.add(1).write_volatile(0);
        }
        crate::utils::DSB();

        // 设置BD的地址字段(word 0)
        fxmac_bd_set_address_rx(rxbd as u64, rx_mbufs_dma as u64);

        instance_p.lwipport.buffer.rx_pbufs_storage[bdindex as usize] = rx_mbufs_vaddr as u64;
    }
}

pub fn ethernetif_input_to_recv_packets(instance_p: &mut FXmac)
{
    if(instance_p.lwipport.recv_flg > 0)
        {
      info!("ethernetif_input_to_recv_packets, fxmac_port->recv_flg={}", instance_p.lwipport.recv_flg);

      // 也许需要屏蔽中断的临界区来保护
      instance_p.lwipport.recv_flg -= 1;

      write_reg((instance_p.config.base_address + FXMAC_IER_OFFSET) as *mut u32, instance_p.mask);

            FXmacRecvHandler(instance_p);
        }

    {
        // move received packet into a new pbuf
        //p = low_level_input(netif);
        // IP or ARP packet
        // full packet send to tcpip thread to process
    }
}

/// FXmacBdRingFree, Frees a set of BDs that had been previously retrieved with
pub fn FXmacBdRingFree(ring_ptr: &mut FXmacBdRing, num_bd: u32) -> u32 {

    // if no bds to process, simply return.
    if 0 == num_bd {
        0
    } else {
        /* Update pointers and counters */
        ring_ptr.free_cnt += num_bd;
        ring_ptr.post_cnt -= num_bd;
        FXMAC_RING_SEEKAHEAD(ring_ptr, ring_ptr.post_head, num_bd);
        
        0
    }
}

/// Reset Tx and Rx DMA pointers after FXmacStop
pub fn ResetDma(instance_p: &mut FXmac)
{
    info!("Resetting DMA");

    let txringptr: &mut FXmacBdRing = &mut instance_p.tx_bd_queue.bdring;
    let rxringptr: &mut FXmacBdRing = &mut instance_p.rx_bd_queue.bdring;

    FXmacBdringPtrReset(txringptr, &instance_p.lwipport.buffer.tx_bdspace as *const _ as *mut FXmacBd);
    FXmacBdringPtrReset(rxringptr, &instance_p.lwipport.buffer.rx_bdspace as *const _ as *mut FXmacBd);

    FXmacSetQueuePtr(instance_p.tx_bd_queue.bdring.phys_base_addr, 0, FXMAC_SEND);
    FXmacSetQueuePtr(instance_p.rx_bd_queue.bdring.phys_base_addr, 0, FXMAC_RECV);
}

/// Handle DMA interrupt error
pub fn FXmacHandleDmaTxError(instance_p: &mut FXmac)
{
    panic!("Failed to handle DMA interrupt error");
    /*
    FreeTxRxPbufs(instance_p);
    FXmacCfgInitialize(&instance_p->instance, &instance_p->instance.config); // -> FXmacReset()

    /* initialize the mac */
    FXmacInitOnError(instance_p); /* need to set mac filter address */
    let mut dmacrreg: u32 = read_reg((instance_p.config.base_address + FXMAC_DMACR_OFFSET) as *const u32);
    dmacrreg = dmacrreg | FXMAC_DMACR_ORCE_DISCARD_ON_ERR_MASK; /* force_discard_on_err */
    write_reg((instance_p.config.base_address + FXMAC_DMACR_OFFSET) as *mut u32, dmacrreg);
    FXmacSetupIsr(instance_p);
    FXmacInitDma(instance_p);

    FXmacStart(&instance_p->instance);
    */
}

pub fn FXmacHandleTxErrors(instance_p: &mut FXmac)
{
    let mut netctrlreg: u32 = read_reg((instance_p.config.base_address + FXMAC_NWCTRL_OFFSET) as *const u32);
    netctrlreg = netctrlreg & !FXMAC_NWCTRL_TXEN_MASK;
    write_reg((instance_p.config.base_address + FXMAC_NWCTRL_OFFSET) as *mut u32, netctrlreg);
    FreeOnlyTxPbufs(instance_p);

    CleanDmaTxdescs(instance_p);
    netctrlreg = read_reg((instance_p.config.base_address + FXMAC_NWCTRL_OFFSET) as *const u32);
    netctrlreg = netctrlreg | FXMAC_NWCTRL_TXEN_MASK;
    write_reg((instance_p.config.base_address + FXMAC_NWCTRL_OFFSET) as *mut u32, netctrlreg);
}

fn CleanDmaTxdescs(instance_p: &mut FXmac)
{
    info!("Clean DMA TX DESCs");
    let txringptr: &mut FXmacBdRing = &mut instance_p.tx_bd_queue.bdring;

    let mut bdtemplate: FXmacBd = [0; FXMAC_BD_NUM_WORDS];

    //FXMAC_BD_SET_STATUS(&bdtemplate, FXMAC_TXBUF_USED_MASK);
    fxmac_bd_write((&mut bdtemplate as *mut _ as u64), FXMAC_BD_STAT_OFFSET, 
        fxmac_bd_read((&mut bdtemplate as *mut _ as u64), FXMAC_BD_STAT_OFFSET) | (FXMAC_TXBUF_USED_MASK));

    let tx_bdspace_ptr = &mut instance_p.lwipport.buffer.tx_bdspace as *mut _ as u64;
    FXmacBdRingCreate(txringptr, tx_bdspace_ptr, tx_bdspace_ptr, BD_ALIGNMENT, FXMAX_TX_BDSPACE_LENGTH as u32);
    FXmacBdRingClone(txringptr, &mut bdtemplate, FXMAC_SEND);
}

fn FreeOnlyTxPbufs(instance_p: &mut FXmac)
{
    for index in  0..FXMAX_TX_PBUFS_LENGTH
    {
        if (instance_p.lwipport.buffer.tx_pbufs_storage[index] != 0)
        {
            let pbuf = instance_p.lwipport.buffer.tx_pbufs_storage[index];
            let pages = (FXMAC_MAX_FRAME_SIZE as usize + (PAGE_SIZE - 1)) / PAGE_SIZE;
            crate::utils::dma_free_coherent(pbuf as usize, pages);

            instance_p.lwipport.buffer.tx_pbufs_storage[index] = 0;
        }
    }
}

/// FXmacProcessSentBds, 释放发送队列q参数
pub fn FXmacProcessSentBds(instance_p: &mut FXmac)
{
    let txring: &mut FXmacBdRing = &mut (instance_p.tx_bd_queue.bdring);
    let txbdset: *mut FXmacBd = null_mut();
    loop {
        /* obtain processed BD's */
        let n_bds: u32 = FXmacBdRingFromHwTx(txring, FXMAX_TX_PBUFS_LENGTH, txbdset);
        if n_bds == 0 {
            error!("FXmacProcessSentBds have not found BD");
            return;
        }
        /* free the processed BD's */
        let mut n_pbufs_freed: u32 = n_bds;
        let mut curbdpntr: *mut FXmacBd = txbdset;
        while n_pbufs_freed > 0 {
            let bdindex = FXMAC_BD_TO_INDEX(txring, curbdpntr as u64) as usize;

            let mut v = 0;
            if bdindex == (FXMAX_TX_PBUFS_LENGTH - 1) {
                v = 0xC0000000; /* Word 1 ,used/Wrap – marks last descriptor in transmit buffer descriptor list.*/
            } else {
                v = 0x80000000; /* Word 1 , Used – must be zero for GEM to read data to the transmit buffer.*/
            }

            let mut temp = curbdpntr as *mut u32;
            // Word 0
            unsafe{
            temp.write_volatile(0);
            temp.add(1).write_volatile(v);
            }
            crate::utils::DSB();

            let pbuf = instance_p.lwipport.buffer.tx_pbufs_storage[bdindex];
            if pbuf != 0 {
                // pbuf_free(p);
                // ?todo xiaoluoyuan@163.com
                let pages = (FXMAC_MAX_FRAME_SIZE as usize + (PAGE_SIZE - 1)) / PAGE_SIZE;
                // Deallocate DMA memory by virtual address
                crate::utils::dma_free_coherent(pbuf as usize, pages);
            }

            instance_p.lwipport.buffer.tx_pbufs_storage[bdindex] = 0;

            let b = curbdpntr;
            curbdpntr = FXMAC_BD_RING_NEXT(txring, curbdpntr);

            assert!(curbdpntr as usize != b as usize);

            n_pbufs_freed -= 1;
            crate::utils::DSB();
        }

        FXmacBdRingFree(txring, n_bds);
    }
}

pub fn FXmacSendHandler(instance: &mut FXmac)
{
    //let txringptr: FXmacBdRing = instance.tx_bd_queue.bdring;
    let regval: u32 = read_reg((instance.config.base_address + FXMAC_TXSR_OFFSET) as *const u32);

    // 清除中断状态位来停止中断
    write_reg((instance.config.base_address + FXMAC_TXSR_OFFSET) as *mut u32, regval);

    // If Transmit done interrupt is asserted, process completed BD's
    // 释放发送队列q参数
    FXmacProcessSentBds(instance);
}

pub fn FXmacLinkChange(instance: &mut FXmac)
{
    if instance.config.interface == FXmacPhyInterface::FXMAC_PHY_INTERFACE_MODE_SGMII
    {
        let mut link: u32 = 0;
        let mut link_status: u32 = 0;

        let ctrl: u32 = read_reg((instance.config.base_address + FXMAC_PCS_AN_LP_OFFSET) as *const u32);
        let link: u32 = (ctrl & FXMAC_PCS_LINK_PARTNER_NEXT_PAGE_STATUS) >> FXMAC_PCS_LINK_PARTNER_NEXT_PAGE_OFFSET;

        match link {
         0 => {
            info!("link status is down");
            link_status = FXMAC_LINKDOWN;
        }
         1 => {
            info!("link status is up");
            link_status = FXMAC_LINKUP;
        }
        _ => {
            error!("link status is error {:#x}", link);
        }
    }

        if link_status == FXMAC_LINKUP
        {
            if link_status != instance.link_status
            {
                instance.link_status = FXMAC_NEGOTIATING;
                info!("need NEGOTIATING");
            }
        }
        else
        {
            instance.link_status = FXMAC_LINKDOWN;
        }
    }
}

/* phy */

/**
 * @name: phy_link_detect
 * @msg:  获取当前link status
 * @note:
 * @param {FXmac} *fxmac_p
 * @param {u32} phy_addr
 * @return {*} 1 is link up , 0 is link down
 */
pub fn phy_link_detect(xmac_p: &mut FXmac, phy_addr: u32) -> u32
{
    let mut status: u16 = 0;

    // Read Phy Status register twice to get the confirmation of the current link status.
    let mut ret: u32 = FXmacPhyRead(xmac_p, phy_addr, PHY_STATUS_REG_OFFSET, &mut status);

    if status & PHY_STAT_LINK_STATUS != 0 {
        return 1;
    }

    0
}

pub fn phy_autoneg_status(xmac_p: &mut FXmac, phy_addr: u32) -> u32
{
    let mut status: u16 = 0;

    // Read Phy Status register twice to get the confirmation of the current link status.
    FXmacPhyRead(xmac_p, phy_addr, PHY_STATUS_REG_OFFSET, &mut status);


    if status & PHY_STATUS_AUTONEGOTIATE_COMPLETE != 0 {
        return 1;
    }

    0
}
