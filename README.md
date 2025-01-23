# FXMAC ethernet driver
fxmac ethernet driver in Rust on PhytiumPi board.

## Quick Start

* Initialize ethernet driver
```
/// 虚拟地址转换成物理地址
#[no_mangle]
pub fn virt_to_phys_fxmac(addr: usize) -> usize {
}

/// 物理地址转换成虚拟地址
#[no_mangle]
pub fn phys_to_virt_fxmac(addr: usize) -> usize {
}

/// 使能并注册网卡中断
#[no_mangle]
pub fn dma_request_irq_fxmac(irq: usize, handler: fn()) {
}

/// 申请页对齐的DMA连续内存页
/// 返回((cpu virtual address, dma physical address))
#[no_mangle]
pub fn dma_alloc_coherent_fxmac(pages: usize) -> (usize, usize) {
}

/// 释放DMA内存页
#[no_mangle]
fn dma_free_coherent_fxmac(vaddr: usize, pages: usize) {
}

let hwaddr: [u8; 6] = [0x55, 0x44, 0x33, 0x22, 0x11, 0x00];
let fxmac_device: &'static mut FXmac = fxmac_rs::fxmac::xmac_init(&hwaddr);
```

* Sending network packets
```
let mut tx_vec = Vec::new();
tx_vec.push(packet.to_vec());
FXmacLwipPortTx(fxmac_device, tx_vec);
```

* Receiving network packets
```
let recv_packets = FXmacRecvHandler(fxmac_device);

```
## About ethernet
PHY: Motorcomm YT8521

![yt8521](doc/phtpi-eth.jpg)

## Reference
* [phytium-standalone-sdk](https://gitee.com/phytium_embedded/phytium-standalone-sdk/tree/master) *sdk驱动的代码质量及逻辑写得一言难尽啊 *
* [Linux 5.10](https://gitee.com/phytium_embedded/phytium-linux-kernel/blob/linux-5.10/drivers/net/ethernet/cadence/macb_main.c)
