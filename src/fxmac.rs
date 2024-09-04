
pub struct FXmac {
    FXmacConfig config;
    is_ready: u32, /* Device is ininitialized and ready*/
    is_started: u32,
    link_status: u32, /* indicates link status ,FXMAC_LINKUP is link up ,FXMAC_LINKDOWN is link down,FXMAC_NEGOTIATING is need to negotiating*/
    options: u32, 
    mask: u32, /* indicates intr mask */
    caps: u32, /*  Capability mask bits */

    FXmacQueue tx_bd_queue; /* Transmit Queue */
    FXmacQueue rx_bd_queue; /* Receive Queue */

    /*
    FXmacIrqHandler send_irq_handler;
    void *send_args;

    FXmacIrqHandler recv_irq_handler;
    void *recv_args;

    FXmacErrorIrqHandler error_irq_handler;
    void *error_args;

    FXmacIrqHandler link_change_handler;
    void *link_change_args;

    FXmacIrqHandler restart_handler;
    void *restart_args;
    */

    moudle_id: u32, /* Module identification number */
    max_mtu_size: u32,
    max_frame_size: u32,

    phy_address: u32,   /* phy address */
    rxbuf_mask: u32,    /* 1000,100,10 */

}

pub struct FXmacConfig {
    instance_id: u32, /* Id of device*/
    base_address: u64,
    extral_mode_base: u64,
    extral_loopback_base: u64,
    interface: FXmacPhyInterface;
    speed: u32,    /* FXMAC_SPEED_XXX */
    duplex: u32,   /* 1 is full-duplex , 0 is half-duplex */
    auto_neg: u32, /* Enable auto-negotiation - when set active high, autonegotiation operation is enabled. */
    pclk_hz: u32,
    max_queue_num: u32, /* Number of Xmac Controller Queues  */
    tx_queue_id: u32,   /* 0 ~ FXMAC_QUEUE_MAX_NUM ,Index queue number */
    rx_queue_id: u32,   /* 0 ~ FXMAC_QUEUE_MAX_NUM ,Index queue number */
    hotplug_irq_num: u32,
    dma_brust_length: u32, /*  burst length */
    network_default_config: u32,
    queue_irq_num[u32; FXMAC_QUEUE_MAX_NUM]; /* mac0 8个 ，其他的 4个 */
    caps: u32, /* used to configure tail ptr feature */
}

/// Interface Mode definitions
pub enum FXmacPhyInterface
{
    FXMAC_PHY_INTERFACE_MODE_SGMII = 0,
    FXMAC_PHY_INTERFACE_MODE_RMII = 1,
    FXMAC_PHY_INTERFACE_MODE_RGMII = 2,
    FXMAC_PHY_INTERFACE_MODE_XGMII = 3,
    FXMAC_PHY_INTERFACE_MODE_USXGMII = 4,
    FXMAC_PHY_INTERFACE_MODE_5GBASER = 5,
    FXMAC_PHY_INTERFACE_MODE_2500BASEX = 6,
}

pub fn xmac_init() -> i32 {

    let xmac = FXmac{ };

    //mii_interface = FXMAC_LWIP_PORT_INTERFACE_SGMII;



/* step 1: initialize instance */
/* step 2: depend on config set some options : JUMBO / IGMP */
/* step 3: FXmacSelectClk */
/* step 4: FXmacInitInterface */
/* step 5: initialize phy */
/* step 6: initialize dma */
/* step 7: initialize interrupt */
/* step 8: start mac */

// FXmacLwipPortInit()


// fxmac_cfg_tbl[0].base_address = 0x3200c000
// xmac_config: interface=FXMAC_PHY_INTERFACE_MODE_SGMII, autonegotiation=0, phy_speed=FXMAC_PHY_SPEED_100M, phy_duplex=FXMAC_PHY_FULL_DUPLEX

// Reset the hardware and set default options
let link_status = FXMAC_LINKDOWN;
let is_ready = FT_COMPONENT_IS_READY;
FXmacReset();

// irq_handler = (FXmacIrqHandler)FXmacIrqStubHandler;
let mask = FXMAC_INTR_MASK;

if (config.caps & FXMAC_CAPS_TAILPTR) != 0
{    
    FXmacSetOptions(xmac_p, FXMAC_TAIL_PTR_OPTION, 0);
    xmac_p->mask &= !FXMAC_IXR_TXUSED_MASK;
}

FxmacFeatureSetOptions(instance_p->feature,xmac_p);

status = FXmacSetMacAddress(xmac_p, (void *)(instance_p->hwaddr), 0);


if(mac_config.interface != FXMAC_PHY_INTERFACE_MODE_USXGMII)
{    
    /* initialize phy */
    status = FXmacPhyInit(xmac_p, xmac_p->config.speed, xmac_p->config.duplex, xmac_p->config.auto_neg, XMAC_PHY_RESET_ENABLE);
    if (status != FT_SUCCESS)
    {    
        warn!("FXmacPhyInit is error");
    }    
} else {
    info!("interface == FXMAC_PHY_INTERFACE_MODE_USXGMII");    
}    

FXmacSelectClk(xmac_p);
FXmacInitInterface(xmac_p);

// initialize dma
dmacrreg = FXMAC_READREG32(xmac_p->config.base_address, FXMAC_DMACR_OFFSET);
dmacrreg &= !(FXMAC_DMACR_BLENGTH_MASK);
dmacrreg = dmacrreg | FXMAC_DMACR_INCR16_AHB_AXI_BURST; /* Attempt to use bursts of up to 16. */
write_reg((FXMAC_IOBASE + FXMAC_DMACR_OFFSET) as *mut u32, dmacrreg);

// TODO
FXmacInitDma(instance_p);

/* initialize interrupt */
FXmacSetupIsr(instance_p);

return FT_SUCCESS;






// Set unicast hash table

FXmac_SetHash()

/*

ethernetif_link_detect()

ethernetif_input()

ethernetif_deinit()

ethernetif_start()

ethernetif_debug()

*/


}

/*
 * Perform a graceful reset of the Ethernet MAC. Resets the DMA channels, the
 * transmitter, and the receiver.
 *
 * Steps to reset
 * - Stops transmit and receive channels
 * - Stops DMA
 * - Configure transmit and receive buffer size to default
 * - Clear transmit and receive status register and counters
 * - Clear all interrupt sources
 * - Clear phy (if there is any previously detected) address
 * - Clear MAC addresses (1-4) as well as Type IDs and hash value
 *
 */

fn FXmacReset() {
    let mac_addr: [u8; 6] = [0; 6];
    let mut reg_val: u32 = 0
    let mut write_reg: u32 = 0;
 
    /* Stop the device and reset hardware */
    FXmacStop();

    // Module identification number
    let moudle_id = ( read_reg((FXMAC_IOBASE + FXMAC_REVISION_REG_OFFSET) as *const u32)
                    & FXMAC_IDENTIFICATION_MASK ) >> 16;
    let mut max_mtu_size = FXMAC_MTU;
    let mut max_frame_size = FXMAC_MAX_FRAME_SIZE;
    let max_queue_num = 16;
    let network_default_config = FXMAC_DEFAULT_OPTIONS;

    let netctrl = (FXMAC_NWCTRL_STATCLR_MASK & !(FXMAC_NWCTRL_LOOPBACK_LOCAL_MASK as u32)) | FXMAC_NWCTRL_MDEN_MASK;
    write_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *mut u32, netctrl);

    FXmacConfigureCaps();

    // mdio clock division
    let mut write_reg = FXmacClkDivGet(moudle_id);
    // DMA bus width
    write_reg |= FXmacDmaWidth(moudle_id);
    write_reg((FXMAC_IOBASE + FXMAC_NWCFG_OFFSET) as *mut u32, write_reg);

    FXmacDmaReset(moudle_id, max_frame_size, max_queue_num);

    // This register, when read provides details of the status of the receive path.
    write_reg((FXMAC_IOBASE + FXMAC_RXSR_OFFSET) as *mut u32, FXMAC_SR_ALL_MASK);

    // write 1 ro the relavant bit location disable that particular interrupt
    write_reg((FXMAC_IOBASE + FXMAC_IDR_OFFSET) as *mut u32, FXMAC_IXR_ALL_MASK);

    reg_val = read_reg((FXMAC_IOBASE + FXMAC_ISR_OFFSET) as *const u32);
    write_reg((FXMAC_IOBASE + FXMAC_ISR_OFFSET) as *mut u32, reg_val);

    write_reg((FXMAC_IOBASE + FXMAC_TXSR_OFFSET) as *mut u32, FXMAC_SR_ALL_MASK);
    
    FXmacClearHash();

    // set default mac address
    for i in 0..4 {
        FXmacSetMacAddress(mac_addr, i);
        FXmacGetMacAddress(mac_addr, i);
        FXmacSetTypeIdCheck(0, i);
    }

    /* clear all counters */
    for i in  0..((FXMAC_LAST_OFFSET - FXMAC_OCTTXL_OFFSET) / 4) {
        read_reg((FXMAC_IOBASE + FXMAC_OCTTXL_OFFSET + (i * 4) as u32) as *mut u32);
    }    

    /* Sync default options with hardware but leave receiver and
     * transmitter disabled. They get enabled with FXmacStart() if
     * FXMAC_TRANSMITTER_ENABLE_OPTION and FXMAC_RECEIVER_ENABLE_OPTION are set.
     */
     FXmacSetOptions(max_mtu_size, max_frame_size, max_queue_num, network_default_config & !((FXMAC_TRANSMITTER_ENABLE_OPTION | FXMAC_RECEIVER_ENABLE_OPTION) as u32), 0);
     FXmacClearOptions(max_mtu_size, max_frame_size, max_queue_num, !network_default_config, 0);
}
/*
 * Gracefully stop the Ethernet MAC as follows:
 *   - Disable all interrupts from this device
 *   - Stop DMA channels
 *   - Disable the tansmitter and receiver
 */
 pub fn FXmacStop() {
    // Disable all interrupts
    write_reg((FXMAC_IOBASE + FXMAC_IDR_OFFSET) as *mut u32, FXMAC_IXR_ALL_MASK);

    // Disable the receiver & transmitter
    let reg_val = read_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *const u32);
    reg_val &= (!FXMAC_NWCTRL_RXEN_MASK) as u32;
    reg_val &= (!FXMAC_NWCTRL_TXEN_MASK) as u32;
    write_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *mut u32, reg_val);
 }

 fn FXmacDmaReset(moudle_id: u32, max_frame_size: u32, max_queue_num: u32)
 {
     let mut dmacfg: u32 = 0;
     //let max_queue_num = 16;
     let dma_brust_length = 16;

     let mut rx_buf_size: u32 = max_frame_size / FXMAC_RX_BUF_UNIT;
     rx_buf_size += ((max_frame_size % FXMAC_RX_BUF_UNIT) != 0) ? 1 : 0; /* roundup */
 
     let FXMAC_RXBUFQX_SIZE_OFFSET = |value: u32| (FXMAC_RXBUFQ1_SIZE_OFFSET + (value << 2));

     // moudle_id=12
     if (moudle_id >= 2)
     {
         for queue in 0..max_queue_num {
             dmacfg = 0;
             FXmacSetQueuePtr(0, queue, FXMAC_SEND);
             FXmacSetQueuePtr(0, queue, FXMAC_RECV);
 
             if queue != 0
             {
    write_reg((FXMAC_IOBASE + FXMAC_RXBUFQX_SIZE_OFFSET(queue)) as *mut u32, rx_buf_size);
             }
             else /* queue is 0 */
             {
                 dmacfg |= (FXMAC_DMACR_RXBUF_MASK & (rx_buf_size << FXMAC_DMACR_RXBUF_SHIFT));
             }
         }
 
         dmacfg |= (dma_brust_length & FXMAC_DMACR_BLENGTH_MASK);
 
         dmacfg &= !FXMAC_DMACR_ENDIAN_MASK;
         dmacfg &= !FXMAC_DMACR_SWAP_MANAGEMENT_MASK; /* 选择小端 */
 
         dmacfg &= !FXMAC_DMACR_TCPCKSUM_MASK; /* close  transmitter checksum generation engine */
 
         dmacfg &= !FXMAC_DMACR_ADDR_WIDTH_64;
         dmacfg |= FXMAC_DMACR_RXSIZE_MASK | FXMAC_DMACR_TXSIZE_MASK;
         /*  
             set this bit can enable auto discard rx frame when lack of receive source,
             which avoid endless rx buffer not available error intrrupts.
         */
         dmacfg |= FXMAC_DMACR_ORCE_DISCARD_ON_ERR_MASK; /* force_discard_on_rx_err */
         dmacfg |= FXMAC_DMACR_ADDR_WIDTH_64; // Just for aarch64
     }
     else
     {
         FXmacSetQueuePtr(0, 0, FXMAC_SEND);
         FXmacSetQueuePtr(0, 0, FXMAC_RECV);
         dmacfg |= (FXMAC_DMACR_RXBUF_MASK & (rx_buf_size << FXMAC_DMACR_RXBUF_SHIFT));
         dmacfg |= (dma_brust_length & FXMAC_DMACR_BLENGTH_MASK);
 
         dmacfg &= !FXMAC_DMACR_ENDIAN_MASK;
         dmacfg &= !FXMAC_DMACR_SWAP_MANAGEMENT_MASK; /* 选择小端 */
 
         dmacfg &= !FXMAC_DMACR_TCPCKSUM_MASK; /* close  transmitter checksum generation engine */
 
         dmacfg &= !FXMAC_DMACR_ADDR_WIDTH_64;
         dmacfg |= FXMAC_DMACR_RXSIZE_MASK | FXMAC_DMACR_TXSIZE_MASK;
         /*  
             set this bit can enable auto discard rx frame when lack of receive source,
             which avoid endless rx buffer not available error intrrupts.
         */
         dmacfg |= FXMAC_DMACR_ORCE_DISCARD_ON_ERR_MASK; /* force_discard_on_rx_err */
         dmacfg |= FXMAC_DMACR_ADDR_WIDTH_64; // Just for aarch64
     }
 
    write_reg((FXMAC_IOBASE + FXMAC_DMACR_OFFSET) as *mut u32, dmacfg);
 }


fn FXmacDmaWidth(moudle_id: u32) -> u32 {
    if moudle_id < 2 {
        return FXMAC_NWCFG_BUS_WIDTH_32_MASK;
    }

    let read_regs = read_reg(FXMAC_IOBASE, FXMAC_DESIGNCFG_DEBUG1_OFFSET);
    match ((read_regs & FXMAC_DESIGNCFG_DEBUG1_BUS_WIDTH_MASK) >> 25) {
        4 => {
            info!("bus width is 128");
            FXMAC_NWCFG_BUS_WIDTH_128_MASK
        }
        2 => {
            info!("bus width is 64");
            FXMAC_NWCFG_BUS_WIDTH_64_MASK
        }
        _ => {
            info!("bus width is 32");
            FXMAC_NWCFG_BUS_WIDTH_32_MASK
        }
    }
}



fn FxmacFeatureSetOptions(feature: u32, FXmac* xmac_p)
{
    let mut options: u32 = 0;

    if (feature & FXMAC_LWIP_PORT_CONFIG_JUMBO) != 0
    {
        info!("FXMAC_JUMBO_ENABLE_OPTION is ok");
        options |= FXMAC_JUMBO_ENABLE_OPTION;
    }

    if (feature & FXMAC_LWIP_PORT_CONFIG_UNICAST_ADDRESS_FILITER) !=0
    {
        info!("FXMAC_UNICAST_OPTION is ok");
        options |= FXMAC_UNICAST_OPTION;
    }

    if (feature & FXMAC_LWIP_PORT_CONFIG_MULTICAST_ADDRESS_FILITER) !=0
    {
        info!("FXMAC_MULTICAST_OPTION is ok");
        options |= FXMAC_MULTICAST_OPTION;
    }
    /* enable copy all frames */
    if (feature & FXMAC_LWIP_PORT_CONFIG_COPY_ALL_FRAMES) != 0
    {
        info!("FXMAC_PROMISC_OPTION is ok");
        options |= FXMAC_PROMISC_OPTION;
    }
      /* close fcs check */
    if (feature & FXMAC_LWIP_PORT_CONFIG_CLOSE_FCS_CHECK) != 0
    {
        info!("FXMAC_FCS_STRIP_OPTION is ok");
        options |= FXMAC_FCS_STRIP_OPTION;
    }

    FXmacSetOptions(xmac_p, options, 0);
}

/**
 * This function sets the start address of the transmit/receive buffer queue.
 *
 * @param   instance_p is a pointer to the instance to be worked on.
 * @param   queue_p is the address of the Queue to be written
 * @param   queue_num is the Buffer Queue Index
 * @param   direction indicates Transmit/Receive
 *
 * @note
 * The buffer queue addresses has to be set before starting the transfer, so
 * this function has to be called in prior to FXmacStart()
 */
 fn FXmacSetQueuePtr(queue_p: u64, queue_num: u8, direction: u32) {

    let flag_queue_p = if queue_p == 0 { 1 }else{ 0 };

    let FXMAC_QUEUE_REGISTER_OFFSET = |base_addr: u32, queue_id: u32| (base_addr + (queue_id - 1) * 4);

    if queue_num == 0x00U {
        if direction == FXMAC_SEND
        {
            /* set base start address of TX buffer queue (tx buffer descriptor list) */
    write_reg((FXMAC_IOBASE + FXMAC_TXQBASE_OFFSET) as *mut u32,
                             (queue_p & ULONG64_LO_MASK) | flag_queue_p); 
        }
        else
        {
            /* set base start address of RX buffer queue (rx buffer descriptor list) */
    write_reg((FXMAC_IOBASE + FXMAC_RXQBASE_OFFSET) as *mut u32,
                             (queue_p & ULONG64_LO_MASK) | flag_queue_p); 
        }
    }    
    else 
    {    
        if direction == FXMAC_SEND
        {
    write_reg((FXMAC_IOBASE + FXMAC_QUEUE_REGISTER_OFFSET(FXMAC_TXQ1BASE_OFFSET, queue_num),) as *mut u32,
                             (queue_p & ULONG64_LO_MASK) | flag_queue_p); 
        }
        else
        {
    write_reg((FXMAC_IOBASE + FXMAC_QUEUE_REGISTER_OFFSET(FXMAC_RXQ1BASE_OFFSET, queue_num)) as *mut u32,
                             (queue_p & ULONG64_LO_MASK) | flag_queue_p); 
        }
    }    

    if direction == FXMAC_SEND // Only for aarch64
    {    
        /* Set the MSB of TX Queue start address */
    write_reg((FXMAC_IOBASE + FXMAC_MSBBUF_TXQBASE_OFFSET) as *mut u32,
                        ((queue_p & ULONG64_HI_MASK) >> 32U) as u32);
    } else {    
        /* Set the MSB of RX Queue start address */
    write_reg((FXMAC_IOBASE + FXMAC_MSBBUF_RXQBASE_OFFSET) as *mut u32,
                        ((queue_p & ULONG64_HI_MASK) >> 32U) as u32);
    }    
}

fn FXmacConfigureCaps() {
    let read_regs = read_reg((FXMAC_IOBASE + FXMAC_DESIGNCFG_DEBUG1_OFFSET) as *const u32);
    if (read_regs & FXMAC_DESIGNCFG_DEBUG1_BUS_IRQCOR_MASK) == 0 {
        // let caps |= FXMAC_CAPS_ISR_CLEAR_ON_WRITE;
        info!("Design ConfigReg1: {:#x} Has FXMAC_CAPS_ISR_CLEAR_ON_WRITE feature", read_regs);
    }
}


fn FXmacClkDivGet(moudle_id: u32) -> u32 {
    // moudle_id=12
    // let pclk_hz = 50000000;
    let pclk_hz = FXMAC0_PCLK;

    if (pclk_hz <= 20000000)
    {
        return FXMAC_NWCFG_CLOCK_DIV8_MASK;
    }
    else if (pclk_hz <= 40000000)
    {
        return FXMAC_NWCFG_CLOCK_DIV16_MASK;
    }
    else if (pclk_hz <= 80000000)
    {
        return FXMAC_NWCFG_CLOCK_DIV32_MASK;
    }
    else if (moudle_id >= 2)
    {
        if (pclk_hz <= 120000000)
        {
            return FXMAC_NWCFG_CLOCK_DIV48_MASK;
        }
        else if (pclk_hz <= 160000000)
        {
            return FXMAC_NWCFG_CLOCK_DIV64_MASK;
        }
        else if (pclk_hz <= 240000000)
        {
            return FXMAC_NWCFG_CLOCK_DIV96_MASK;
        }
        else if (pclk_hz <= 320000000)
        {
            return FXMAC_NWCFG_CLOCK_DIV128_MASK;
        }
        else
        {
            return FXMAC_NWCFG_CLOCK_DIV224_MASK;
        }
    }
    else
    {
        return FXMAC_NWCFG_CLOCK_DIV64_MASK;
    }
}


/**
 * Set options for the driver/device. The driver should be stopped with
 * FXmacStop() before changing options.
 */
 fn FXmacSetOptions(max_mtu_size: mut u32, max_frame_size: mut u32, max_queue_num: u32, options: mut u32, queue_num: u32) -> u32 {
     let mut reg: u32 = 0;            /* Generic register contents */
     let mut reg_netcfg: u32 = 0;     /* Reflects original contents of NET_CONFIG */
     let mut reg_new_netcfg: u32 = 0; /* Reflects new contents of NET_CONFIG */
     let mut status: u32 = 0;

     let FXMAC_RXBUFQX_SIZE_OFFSET = |value: u32| (FXMAC_RXBUFQ1_SIZE_OFFSET + (value << 2));
     let is_started = 0;
 
     info!("FXmacSetOptions, is_started={}, options={}, queue_num={}, max_queue_num={}", is_started, options, queue_num, max_queue_num);
 
     /* Be sure device has been stopped */
     if is_started == FT_COMPONENT_IS_STARTED
     {
         status = FXMAC_ERR_MAC_IS_PROCESSING;
        error!("FXMAC is processing when calling FXmacSetOptions function");
     } else {
 
         /* Many of these options will change the NET_CONFIG registers.
          * To reduce the amount of IO to the device, group these options here
          * and change them all at once.
          */
 
         /* Grab current register contents */
         reg_netcfg = read_reg((FXMAC_IOBASE + FXMAC_NWCFG_OFFSET) as *const u32);

         reg_new_netcfg = reg_netcfg;
 
         /*
          * It is configured to max 1536.
          */
         if (options & FXMAC_FRAME1536_OPTION) != 0
         {
             reg_new_netcfg |= FXMAC_NWCFG_1536RXEN_MASK;
         }
 
         /* Turn on VLAN packet only, only VLAN tagged will be accepted */
         if (options & FXMAC_VLAN_OPTION) != 0
         {
             reg_new_netcfg |= FXMAC_NWCFG_NVLANDISC_MASK;
         }
 
         /* Turn on FCS stripping on receive packets */
         if (options & FXMAC_FCS_STRIP_OPTION) != 0
         {
             reg_new_netcfg |= FXMAC_NWCFG_FCS_REMOVE_MASK;
         }
 
         /* Turn on length/type field checking on receive packets */
         if (options & FXMAC_LENTYPE_ERR_OPTION) != 0
         {
             reg_new_netcfg |= FXMAC_NWCFG_LENGTH_FIELD_ERROR_FRAME_DISCARD_MASK;
         }
 
         /* Turn on flow control */
         if (options & FXMAC_FLOW_CONTROL_OPTION) != 0
         {
             reg_new_netcfg |= FXMAC_NWCFG_PAUSE_ENABLE_MASK;
         }
 
         /* Turn on promiscuous frame filtering (all frames are received) */
         if (options & FXMAC_PROMISC_OPTION) != 0
         {
             reg_new_netcfg |= FXMAC_NWCFG_COPYALLEN_MASK;
         }
 
         /* Allow broadcast address reception */
         if (options & FXMAC_BROADCAST_OPTION) != 0
         {
             reg_new_netcfg &= !(FXMAC_NWCFG_BCASTDI_MASK as u32);
         }
 
         /* Allow multicast address filtering */
         if (options & FXMAC_MULTICAST_OPTION) != 0
         {
             reg_new_netcfg |= FXMAC_NWCFG_MCASTHASHEN_MASK;
         }
 
         if (options & FXMAC_UNICAST_OPTION) != 0
         {
             reg_new_netcfg |= FXMAC_NWCFG_UCASTHASHEN_MASK;
         }
 
         if options & FXMAC_TAIL_PTR_OPTION
         {
    write_reg((FXMAC_IOBASE + FXMAC_TAIL_ENABLE) as *mut u32, 0x80000001);
         }
 
 
         /* enable RX checksum offload */
         if (options & FXMAC_RX_CHKSUM_ENABLE_OPTION) != 0
         {
             reg_new_netcfg |= FXMAC_NWCFG_RXCHKSUMEN_MASK;
         }
 
         /* Enable jumbo frames */
         if (options & FXMAC_JUMBO_ENABLE_OPTION) != 0
         {
             max_mtu_size = FXMAC_MTU_JUMBO;
             max_frame_size = FXMAC_MAX_FRAME_SIZE_JUMBO;

             reg_new_netcfg |= FXMAC_NWCFG_JUMBO_MASK;

    write_reg((FXMAC_IOBASE + FXMAC_JUMBOMAXLEN_OFFSET) as *mut u32, FXMAC_MAX_FRAME_SIZE_JUMBO);

    write_reg((FXMAC_IOBASE + FXMAC_TXQSEGALLOC_QLOWER_OFFSET) as *mut u32, FXMAC_TXQSEGALLOC_QLOWER_JUMBO_MASK);

            if queue_num == 0 {
                 let mut rx_buf_size: u32 = 0;
                 reg = read_reg((FXMAC_IOBASE + FXMAC_DMACR_OFFSET) as *const u32);

                 reg &= !FXMAC_DMACR_RXBUF_MASK;
 
                 rx_buf_size = max_frame_size / FXMAC_RX_BUF_UNIT;
                 rx_buf_size += if (max_frame_size % FXMAC_RX_BUF_UNIT) != 0 { 1 } else { 0 };
 
                 reg |= (rx_buf_size << FXMAC_DMACR_RXBUF_SHIFT) & FXMAC_DMACR_RXBUF_MASK;
    write_reg((FXMAC_IOBASE + FXMAC_DMACR_OFFSET) as *mut u32, reg);
             } else if queue_num < max_queue_num {
                 let mut rx_buf_size: u32 = 0;
                 rx_buf_size = max_frame_size / FXMAC_RX_BUF_UNIT;
                 rx_buf_size += if (max_frame_size % FXMAC_RX_BUF_UNIT) != 0 { 1 } else { 0 };
 
    write_reg((FXMAC_IOBASE + FXMAC_RXBUFQX_SIZE_OFFSET(queue_num)) as *mut u32, rx_buf_size & FXMAC_RXBUFQX_SIZE_MASK);
             }
         }
 
         if (options & FXMAC_SGMII_ENABLE_OPTION) != 0
         {
             reg_new_netcfg |= (FXMAC_NWCFG_SGMII_MODE_ENABLE_MASK |
                                FXMAC_NWCFG_PCSSEL_MASK);
         }
 
         if (options & FXMAC_LOOPBACK_NO_MII_OPTION) != 0
         {
             reg = read_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *const u32);
             reg |= FXMAC_NWCTRL_LOOPBACK_LOCAL_MASK;
    write_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *mut u32, reg);
         }
 
         if (options & FXMAC_LOOPBACK_USXGMII_OPTION) != 0
         {
    write_reg((FXMAC_IOBASE + FXMAC_TEST_CONTROL_OFFSET) as *mut u32, 2);
         }
 
         /* Officially change the NET_CONFIG registers if it needs to be
          * modified.
          */
         if (reg_netcfg != reg_new_netcfg)
         {
    write_reg((FXMAC_IOBASE + FXMAC_NWCFG_OFFSET) as *mut u32, reg_new_netcfg);
         }
 
         /* Enable TX checksum offload */
         if (options & FXMAC_TX_CHKSUM_ENABLE_OPTION) != 0
         {
             reg = read_reg((FXMAC_IOBASE + FXMAC_DMACR_OFFSET) as *const u32);
             reg |= FXMAC_DMACR_TCPCKSUM_MASK;
    write_reg((FXMAC_IOBASE + FXMAC_DMACR_OFFSET) as *mut u32, reg);
         }
 
         /* Enable transmitter */
         if (options & FXMAC_TRANSMITTER_ENABLE_OPTION) != 0
         {
             reg = read_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *const u32);
             reg |= FXMAC_NWCTRL_TXEN_MASK;
    write_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *mut u32, reg);

         }
 
         /* Enable receiver */
         if (options & FXMAC_RECEIVER_ENABLE_OPTION) != 0
         {
             reg = read_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *const u32);
             reg |= FXMAC_NWCTRL_RXEN_MASK;

    write_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *mut u32, reg);

         }
 
         /* The remaining options not handled here are managed elsewhere in the
          * driver. No register modifications are needed at this time. Reflecting
          * the option in instance_p->options is good enough for now.
          */
 
         /* Set options word to its new value */
         options |= options;
 
         status = FT_SUCCESS;
     }

     status
 }
 
/// Clear options for the driver/device
fn FXmacClearOptions(max_mtu_size: mut u32, max_frame_size: mut u32, max_queue_num: u32, options: mut u32, queue_num: u32) -> u32
 {
     let mut reg: u32 = 0;             /* Generic */
     let mut reg_net_cfg: u32 = 0;     /* Reflects original contents of NET_CONFIG */
     let mut reg_new_net_cfg: u32 = 0; /* Reflects new contents of NET_CONFIG */
     let mut status: u32 = 0;

     let is_started = 0;
     /* Be sure device has been stopped */
     if (is_started == FT_COMPONENT_IS_STARTED)
     {
         status = FXMAC_ERR_MAC_IS_PROCESSING;
         error!("FXMAC is processing when calling FXmacClearOptions function");

     } else {
         /* Many of these options will change the NET_CONFIG registers.
          * To reduce the amount of IO to the device, group these options here
          * and change them all at once.
          */
         /* Grab current register contents */
         reg_net_cfg = read_reg((FXMAC_IOBASE + FXMAC_NWCFG_OFFSET) as *const u32);
         reg_new_net_cfg = reg_net_cfg;
         /* There is only RX configuration!?
          * It is configured in two different length, up to 1536 and 10240 bytes
          */
         if (options & FXMAC_FRAME1536_OPTION) != 0
         {
             reg_new_net_cfg &= !(FXMAC_NWCFG_1536RXEN_MASK as u32);
         }
 
         /* Turn off VLAN packet only */
         if (options & FXMAC_VLAN_OPTION) != 0
         {
             reg_new_net_cfg &= !(FXMAC_NWCFG_NVLANDISC_MASK as u32);
         }
 
         /* Turn off FCS stripping on receive packets */
         if (options & FXMAC_FCS_STRIP_OPTION) != 0
         {
             reg_new_net_cfg &= !(FXMAC_NWCFG_FCS_REMOVE_MASK as u32);
         }
 
         /* Turn off length/type field checking on receive packets */
         if (options & FXMAC_LENTYPE_ERR_OPTION) != 0
         {
             reg_new_net_cfg &= !(FXMAC_NWCFG_LENGTH_FIELD_ERROR_FRAME_DISCARD_MASK as u32);
         }
 
         /* Turn off flow control */
         if (options & FXMAC_FLOW_CONTROL_OPTION) != 0
         {
             reg_new_net_cfg &= !(FXMAC_NWCFG_PAUSE_ENABLE_MASK as u32);
         }
 
         /* Turn off promiscuous frame filtering (all frames are received) */
         if (options & FXMAC_PROMISC_OPTION) != 0
         {
             reg_new_net_cfg &= !(FXMAC_NWCFG_COPYALLEN_MASK as u32);
         }
 
         /* Disallow broadcast address filtering => broadcast reception */
         if (options & FXMAC_BROADCAST_OPTION) != 0
         {
             reg_new_net_cfg |= FXMAC_NWCFG_BCASTDI_MASK;
         }
 
         /* Disallow unicast address filtering */
         if (options & FXMAC_UNICAST_OPTION) != 0
         {
             reg_new_net_cfg &= !(FXMAC_NWCFG_UCASTHASHEN_MASK as u32);
         }
         
         /* Disallow multicast address filtering */
         if (options & FXMAC_MULTICAST_OPTION) != 0
         {
             reg_new_net_cfg &= !(FXMAC_NWCFG_MCASTHASHEN_MASK as u32);
         }
 
         if (options & FXMAC_TAIL_PTR_OPTION) != 0
         {
    write_reg((FXMAC_IOBASE + FXMAC_TAIL_ENABLE) as *mut u32, 0);
         }
 
         /* Disable RX checksum offload */
         if (options & FXMAC_RX_CHKSUM_ENABLE_OPTION) != 0
         {
             reg_new_net_cfg &= !(FXMAC_NWCFG_RXCHKSUMEN_MASK as u32);
         }
 
         /* Disable jumbo frames */
         if (options & FXMAC_JUMBO_ENABLE_OPTION) != 0 /* 恢复之前buffer 容量 */
         {
             max_mtu_size = FXMAC_MTU;
             max_frame_size = FXMAC_MAX_FRAME_SIZE;
 
             reg_new_net_cfg &= !(FXMAC_NWCFG_JUMBO_MASK as u32);

             reg = read_reg((FXMAC_IOBASE + FXMAC_DMACR_OFFSET) as *const u32);

             reg &= !FXMAC_DMACR_RXBUF_MASK;
 
             if queue_num == 0
             {
                 u32 rx_buf_size = 0;
 
                 reg = read_reg((FXMAC_IOBASE + FXMAC_DMACR_OFFSET) as *const u32);
                 reg &= !FXMAC_DMACR_RXBUF_MASK;
 
                 rx_buf_size = max_frame_size / FXMAC_RX_BUF_UNIT;
                 rx_buf_size += if max_frame_size % FXMAC_RX_BUF_UNIT != 0 {1} else {0};
 
                 reg |= (rx_buf_size << FXMAC_DMACR_RXBUF_SHIFT) & FXMAC_DMACR_RXBUF_MASK;
 
    write_reg((FXMAC_IOBASE + FXMAC_DMACR_OFFSET) as *mut u32, reg);
             }
             else if (queue_num < max_queue_num)
             {
                 let rx_buf_size: u32 = 0;
                 rx_buf_size = max_frame_size / FXMAC_RX_BUF_UNIT;
                 rx_buf_size += if (max_frame_size % FXMAC_RX_BUF_UNIT) != 0 {1} else {0};
 
    write_reg((FXMAC_IOBASE + FXMAC_RXBUFQX_SIZE_OFFSET(queue_num)) as *mut u32, rx_buf_size & FXMAC_RXBUFQX_SIZE_MASK);
             }
         }
 
         if (options & FXMAC_SGMII_ENABLE_OPTION) != 0
         {
             reg_new_net_cfg &= !((FXMAC_NWCFG_SGMII_MODE_ENABLE_MASK |
                                        FXMAC_NWCFG_PCSSEL_MASK) as u32);
         }
 
         if (options & FXMAC_LOOPBACK_NO_MII_OPTION) != 0
         {
             reg = read_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *const u32);
             reg &= !FXMAC_NWCTRL_LOOPBACK_LOCAL_MASK;
    write_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *mut u32, reg);
         }
 
         if (options & FXMAC_LOOPBACK_USXGMII_OPTION) != 0
         {
    write_reg((FXMAC_IOBASE + FXMAC_TEST_CONTROL_OFFSET) as *mut u32,
        read_reg((FXMAC_IOBASE + FXMAC_TEST_CONTROL_OFFSET) as *const u32) & !2 );
         }
 
         /* Officially change the NET_CONFIG registers if it needs to be
          * modified.
          */
         if reg_net_cfg != reg_new_net_cfg
         {
    write_reg((FXMAC_IOBASE + FXMAC_NWCFG_OFFSET) as *mut u32, reg_new_net_cfg);
         }
 
         /* Disable TX checksum offload */
         if (options & FXMAC_TX_CHKSUM_ENABLE_OPTION) != 0
         {
             reg = read_reg((FXMAC_IOBASE + FXMAC_DMACR_OFFSET) as *const u32);
             reg &= !FXMAC_DMACR_TCPCKSUM_MASK;
    write_reg((FXMAC_IOBASE + FXMAC_DMACR_OFFSET) as *mut u32, reg);
         }
 
         /* Disable transmitter */
         if (options & FXMAC_TRANSMITTER_ENABLE_OPTION) != 0
         {
             reg = read_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *const u32);
             reg &= !FXMAC_NWCTRL_TXEN_MASK;
    write_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *mut u32, reg);
         }
 
         /* Disable receiver */
         if (options & FXMAC_RECEIVER_ENABLE_OPTION) != 0
         {
             reg = read_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *const u32);
             reg &= !FXMAC_NWCTRL_RXEN_MASK;
    write_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *mut u32, reg);
         }
 
         /* The remaining options not handled here are managed elsewhere in the
          * driver. No register modifications are needed at this time. Reflecting
          * option in instance_p->options is good enough for now.
          */
 
         /* Set options word to its new value */
         options &= !options;
 
         status = FT_SUCCESS;
     }
     status
 }

///  Clear the Hash registers for the mac address pointed by address_ptr.
fn FXmacClearHash() {
    write_reg((FXMAC_IOBASE + FXMAC_HASHL_OFFSET) as *mut u32, 0);

    /* write bits [63:32] in TOP */
    write_reg((FXMAC_IOBASE + FXMAC_HASHH_OFFSET) as *mut u32, 0);
}

/// Set the MAC address for this driver/device.  The address is a 48-bit value.
/// The device must be stopped before calling this function.
fn FXmacSetMacAddress(address_ptr: &[u8; 6], index: u8) -> u32 {
    let mut mac_addr: u32 = 0; 
    let aptr = address_ptr;
    let index_loc: u8 = index;
    let mut status: u32 = 0;
    assert!((index_loc < FXMAC_MAX_MAC_ADDR as u8), "index of Mac Address exceed {}", FXMAC_MAX_MAC_ADDR);

    let is_started = 0;
    /* Be sure device has been stopped */
    if is_started == (u32)FT_COMPONENT_IS_STARTED
    {        
        status = FXMAC_ERR_MAC_IS_PROCESSING;
        error!("FXMAC is processing when calling FXmacSetMacAddress function");
    } else {
        /* Set the MAC bits [31:0] in BOT */
        mac_addr = aptr[0];
        mac_addr |= aptr[1] as u32 << 8;
        mac_addr |= aptr[2] as u32 << 16;
        mac_addr |= aptr[3] as u32 << 24;
    write_reg((FXMAC_IOBASE + FXMAC_GEM_SA1B + (index_loc * 8) as u32) as *mut u32, mac_addr);

        /* There are reserved bits in TOP so don't affect them */
        mac_addr = read_reg((FXMAC_IOBASE + FXMAC_GEM_SA1T + (index_loc * 8) as u32) as *const u32);
        mac_addr &= !FXMAC_GEM_SAB_MASK;

        /* Set MAC bits [47:32] in TOP */
        mac_addr |= aptr[4] as u32;
        mac_addr |= aptr[5] as u32 << 8;

        write_reg((FXMAC_IOBASE + FXMAC_GEM_SA1T + (index_loc * 8) as u32) as *mut u32, mac_addr);

        status = FT_SUCCESS;
    }

    status
}

/// Set the Type ID match for this driver/device.  The register is a 32-bit value. 
/// The device must be stopped before calling this function.
fn FXmacSetTypeIdCheck(u32 id_check: u32, index: u8) -> u32 {
    let status: u32 = 0;
    assert!((index < FXMAC_MAX_TYPE_ID as u8), "index of Type ID exceed {}", FXMAC_MAX_TYPE_ID);

    let is_started = 0;
    /* Be sure device has been stopped */
    if is_started == FT_COMPONENT_IS_STARTED
    {   
        status = FXMAC_ERR_MAC_IS_PROCESSING;
        error!("FXMAC is processing when calling FXmacSetTypeIdCheck function");
    } else {   

        /* Set the ID bits in MATCHx register */
    write_reg((FXMAC_IOBASE + FXMAC_MATCH1_OFFSET + (index * 4) as u32) as *mut u32, id_check);

        status = FT_SUCCESS;
    }

    status;
}

/// FXmacSelectClk
/// Determine the driver clock configuration based on the media independent interface
/// FXMAC_CLK_TYPE_0
 fn FXmacSelectClk(instance_p: &mut FXmac)
 {
     let speed: u32 = instance_p.config.speed;
     let FXMAC_WRITEREG32 = |base_address: u64, offset: u64, reg_value: u32| write_reg((base_address + offset) as *mut u32, reg_value);

     assert!((speed == FXMAC_SPEED_10) || (speed == FXMAC_SPEED_100) || (speed == FXMAC_SPEED_1000) || (speed == FXMAC_SPEED_2500) || (speed == FXMAC_SPEED_10000));

     if (instance_p.config.interface == FXMAC_PHY_INTERFACE_MODE_USXGMII) || (instance_p.config.interface == FXMAC_PHY_INTERFACE_MODE_XGMII)
     {
         if speed == FXMAC_SPEED_10000
         {
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_SRC_SEL_LN, 0x1); /*0x1c04*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_DIV_SEL0_LN, 0x4); /*0x1c08*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_DIV_SEL1_LN, 0x1); /*0x1c0c*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_PMA_XCVR_POWER_STATE, 0x1); /*0x1c10*/
         }
         else if speed == FXMAC_SPEED_5000
         {
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_SRC_SEL_LN, 0x1); /*0x1c04*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_DIV_SEL0_LN, 0x8); /*0x1c08*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_DIV_SEL1_LN, 0x2); /*0x1c0c*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_PMA_XCVR_POWER_STATE, 0); /*0x1c10*/
         }
     }
     else if instance_p.config.interface == FXMAC_PHY_INTERFACE_MODE_5GBASER
     {
         if speed == FXMAC_SPEED_5000
         {
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_SRC_SEL_LN, 0x1); /*0x1c04*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_DIV_SEL0_LN, 0x8); /*0x1c08*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_DIV_SEL1_LN, 0x2); /*0x1c0c*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_PMA_XCVR_POWER_STATE, 0x0); /*0x1c10*/
         }
     }
     else if instance_p.config.interface == FXMAC_PHY_INTERFACE_MODE_2500BASEX
     {
         if speed == FXMAC_SPEED_25000
         {
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_SRC_SEL_LN, 0x1); /*0x1c04*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_DIV_SEL0_LN, 0x1); /*0x1c08*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_DIV_SEL1_LN, 0x2); /*0x1c0c*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_PMA_XCVR_POWER_STATE, 0x1); /*0x1c10*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL0, 0); /*0x1c20*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL1, 0x1); /*0x1c24*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL2, 0x1); /*0x1c28*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL3, 0x1); /*0x1c2c*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL0, 0x1); /*0x1c30*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL1, 0x0); /*0x1c34*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL3_0, 0x0); /*0x1c70*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL4_0, 0x0); /*0x1c74*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL3_0, 0x0); /*0x1c78*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL4_0, 0x0); /*0x1c7c*/
         }
     }
     else if instance_p.config.interface == FXMAC_PHY_INTERFACE_MODE_SGMII
     {
         info!("FXMAC_PHY_INTERFACE_MODE_SGMII init");
         if speed == FXMAC_SPEED_2500
         {
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_SRC_SEL_LN, 0);
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_DIV_SEL0_LN, 0x1);
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_DIV_SEL1_LN, 0x2);
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_PMA_XCVR_POWER_STATE, 0x1);
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL0, 0);
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL1, 0x1);
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL2, 0x1);
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL3, 0x1);
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL0, 0x1);
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL1, 0x0);
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL3_0, 0x0); /*0x1c70*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL4_0, 0x0); /*0x1c74*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL3_0, 0x0); /*0x1c78*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL4_0, 0x0); /*0x1c7c*/
         }
         else if speed == FXMAC_SPEED_1000
         {
             info!("sgmii FXMAC_SPEED_1000");
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_SRC_SEL_LN, 0x1); /*0x1c04*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_DIV_SEL0_LN, 0x4); /*0x1c08*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_DIV_SEL1_LN, 0x8); /*0x1c0c*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_PMA_XCVR_POWER_STATE, 0x1); /*0x1c10*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL0, 0x0); /*0x1c20*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL1, 0x0); /*0x1c24*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL2, 0x0); /*0x1c28*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL3, 0x1); /*0x1c2c*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL0, 0x1); /*0x1c30*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL1, 0x0); /*0x1c34*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL3_0, 0x0); /*0x1c70*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL4_0, 0x0); /*0x1c74*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL3_0, 0x0); /*0x1c78*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL4_0, 0x0); /*0x1c7c*/
         }
         else if (speed == FXMAC_SPEED_100) || (speed == FXMAC_SPEED_10)
         {
            info!("sgmii FXMAC_SPEED_{}", speed);

             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_SRC_SEL_LN, 0x1); /*0x1c04*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_DIV_SEL0_LN, 0x4); /*0x1c08*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_DIV_SEL1_LN, 0x8); /*0x1c0c*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_PMA_XCVR_POWER_STATE, 0x1); /*0x1c10*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL0, 0x0); /*0x1c20*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL1, 0x0); /*0x1c24*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL2, 0x1); /*0x1c28*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL3, 0x1); /*0x1c2c*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL0, 0x1); /*0x1c30*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL1, 0x0); /*0x1c34*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL3_0, 0x1); /*0x1c70*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL4_0, 0x0); /*0x1c74*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL3_0, 0x0); /*0x1c78*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL4_0, 0x1); /*0x1c7c*/
         }
     }
     else if instance_p.config.interface == FXMAC_PHY_INTERFACE_MODE_RGMII
     {
         info!("FXMAC_PHY_INTERFACE_MODE_RGMII init");
         if speed == FXMAC_SPEED_1000
         {
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_MII_SELECT, 0x1);       /*0x1c18*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_SEL_MII_ON_RGMII, 0x0); /*0x1c1c*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL0, 0x0);      /*0x1c20*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL1, 0x1);      /*0x1c24*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL2, 0x0);      /*0x1c28*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL3, 0x0);      /*0x1c2c*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL0, 0x0);      /*0x1c30*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL1, 0x1);      /*0x1c34*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_CLK_250M_DIV10_DIV100_SEL,
                              0x0);                                                               /*0x1c38*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL5, 0x1);       /*0x1c48*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RGMII_TX_CLK_SEL0, 0x1); /*0x1c80*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RGMII_TX_CLK_SEL1, 0x0); /*0x1c84*/
         }
         else if speed == FXMAC_SPEED_100
         {
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_MII_SELECT, 0x1);       /*0x1c18*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_SEL_MII_ON_RGMII, 0x0); /*0x1c1c*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL0, 0x0);      /*0x1c20*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL1, 0x1);      /*0x1c24*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL2, 0x0);      /*0x1c28*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL3, 0x0);      /*0x1c2c*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL0, 0x0);      /*0x1c30*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL1, 0x1);      /*0x1c34*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_CLK_250M_DIV10_DIV100_SEL,
                              0x0);                                                               /*0x1c38*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL5, 0x1);       /*0x1c48*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RGMII_TX_CLK_SEL0, 0x0); /*0x1c80*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RGMII_TX_CLK_SEL1, 0x0); /*0x1c84*/
         }
         else
         {
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_MII_SELECT, 0x1);       /*0x1c18*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_SEL_MII_ON_RGMII, 0x0); /*0x1c1c*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL0, 0x0);      /*0x1c20*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL1, 0x1);      /*0x1c24*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL2, 0x0);      /*0x1c28*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_TX_CLK_SEL3, 0x0);      /*0x1c2c*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL0, 0x0);      /*0x1c30*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL1, 0x1);      /*0x1c34*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_CLK_250M_DIV10_DIV100_SEL,
                              0x1);                                                               /*0x1c38*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL5, 0x1);       /*0x1c48*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RGMII_TX_CLK_SEL0, 0x0); /*0x1c80*/
             FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RGMII_TX_CLK_SEL1, 0x0); /*0x1c84*/
         }
     }
     else if instance_p.config.interface == FXMAC_PHY_INTERFACE_MODE_RMII
     {
         FXMAC_WRITEREG32(instance_p.config.base_address, FXMAC_GEM_RX_CLK_SEL5, 0x1); /*0x1c48*/
     }
 
     FXmacHighSpeedConfiguration(instance_p, speed);
 }

 fn FXmacHighSpeedConfiguration(instance_p: &mut FXmac, speed: u32)
{
    let reg_value: mut u32 = 0;
    let set_speed: mut i32 = 0; 
    match speed
    {
         FXMAC_SPEED_25000 => {
            set_speed = 2; 
         }
         FXMAC_SPEED_10000 => {
            set_speed = 4; 
         }
         FXMAC_SPEED_5000 => {
            set_speed = 3; 
         }
         FXMAC_SPEED_2500 => {
            set_speed = 2; 
         }
         FXMAC_SPEED_1000 => {
            set_speed = 1; 
         }
         _ => {
            set_speed = 0; 
        }
    }

    /*GEM_HSMAC(0x0050) provide rate to the external*/
    reg_value = read_reg((FXMAC_IOBASE + FXMAC_GEM_HSMAC) as *const u32);
    reg_value &= !FXMAC_GEM_HSMACSPEED_MASK;
    reg_value |= (set_speed) & FXMAC_GEM_HSMACSPEED_MASK;
    write_reg((FXMAC_IOBASE + FXMAC_GEM_HSMAC) as *mut u32, reg_value);

    reg_value = read_reg((FXMAC_IOBASE + FXMAC_GEM_HSMAC) as *const u32);

    info!("FXMAC_GEM_HSMAC is {:#x}", reg_value);
}


/// FXmacInitInterface
/// Initialize the MAC controller configuration based on the PHY interface type
 fn FXmacInitInterface(instance_p: &mut FXmac)
 {
     let mut config: u32 = 0, 
     let mut control: u32 = 0;
 
     info!("FXmacInitInterface, PHY MODE:{}", instance_p.config.interface);
 
     if (instance_p.config.interface == FXMAC_PHY_INTERFACE_MODE_XGMII )
     {
         config = read_reg((FXMAC_IOBASE + FXMAC_NWCFG_OFFSET) as *const u32);
         config &= !FXMAC_NWCFG_PCSSEL_MASK;
         write_reg((FXMAC_IOBASE + FXMAC_NWCFG_OFFSET) as *mut u32, config);
 
         control = read_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *const u32);
         control |= FXMAC_NWCTRL_ENABLE_HS_MAC_MASK; /* Use high speed MAC */
         write_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *mut u32, control);
 
         instance_p.config.duplex = 1;
     }
     else if (instance_p.config.interface == FXMAC_PHY_INTERFACE_MODE_USXGMII || instance_p.config.interface == FXMAC_PHY_INTERFACE_MODE_5GBASER)
     {
         info!("usx interface is {}",instance_p.config.interface);
         /*  network_config */
         instance_p.config.duplex = 1;
         config = read_reg((FXMAC_IOBASE + FXMAC_NWCFG_OFFSET) as *const u32);
         config |= FXMAC_NWCFG_PCSSEL_MASK;
         config &= !FXMAC_NWCFG_100_MASK;
         config &= !FXMAC_NWCFG_SGMII_MODE_ENABLE_MASK;
         if (instance_p.config.duplex == 1)
         {
             info!("is duplex");
             config |= FXMAC_NWCFG_FDEN_MASK;
         }
     
         write_reg((FXMAC_IOBASE + FXMAC_NWCFG_OFFSET) as *mut u32, config);
 
         /* network_control */
         control = read_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *const u32);
         control |= FXMAC_NWCTRL_ENABLE_HS_MAC_MASK; /* Use high speed MAC */
         write_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *mut u32, control);
         
         
         /* High speed PCS control register */
         control = read_reg((FXMAC_IOBASE + FXMAC_GEM_USX_CONTROL_OFFSET) as *const u32);
         
         if (instance_p.config.speed == FXMAC_SPEED_10000)
         {
             info!("is 10G");
             control |= FXMAC_GEM_USX_HS_MAC_SPEED_10G;
             control |= FXMAC_GEM_USX_SERDES_RATE_10G;
         }
         else if (instance_p.config.speed == FXMAC_SPEED_25000)
         {
             control |= FXMAC_GEM_USX_HS_MAC_SPEED_2_5G;
         }
         else if (instance_p.config.speed == FXMAC_SPEED_1000)
         {
             control |= FXMAC_GEM_USX_HS_MAC_SPEED_1G;
         }
         else if (instance_p.config.speed == FXMAC_SPEED_100)
         {
             control |= FXMAC_GEM_USX_HS_MAC_SPEED_100M;
         }
         else if(instance_p.config.speed == FXMAC_SPEED_5000)
         {
             control |= FXMAC_GEM_USX_HS_MAC_SPEED_5G;
             control |= FXMAC_GEM_USX_SERDES_RATE_5G;
         }
         
         control &= !(FXMAC_GEM_USX_TX_SCR_BYPASS | FXMAC_GEM_USX_RX_SCR_BYPASS);
         control |= FXMAC_GEM_USX_RX_SYNC_RESET;
         write_reg((FXMAC_IOBASE + FXMAC_GEM_USX_CONTROL_OFFSET) as *mut u32, control);
 
         control = read_reg((FXMAC_IOBASE + FXMAC_GEM_USX_CONTROL_OFFSET) as *const u32);
         control &= !FXMAC_GEM_USX_RX_SYNC_RESET;
         control |= FXMAC_GEM_USX_TX_DATAPATH_EN;
         control |= FXMAC_GEM_USX_SIGNAL_OK;
 
         write_reg((FXMAC_IOBASE + FXMAC_GEM_USX_CONTROL_OFFSET) as *mut u32, control);
 
     }
     else if(instance_p.config.interface == FXMAC_PHY_INTERFACE_MODE_2500BASEX)
     {
         /*  network_config */
         instance_p.config.duplex = 1;
         config = read_reg((FXMAC_IOBASE + FXMAC_NWCFG_OFFSET) as *const u32);
         config |= FXMAC_NWCFG_PCSSEL_MASK | FXMAC_NWCFG_SGMII_MODE_ENABLE_MASK;
         config &= !FXMAC_NWCFG_100_MASK;
         
         if (instance_p.config.duplex == 1)
         {
             config |= FXMAC_NWCFG_FDEN_MASK;
         }
         write_reg((FXMAC_IOBASE + FXMAC_NWCFG_OFFSET) as *mut u32, config);
 
         /* network_control */
         control = read_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *const u32);
         control &= !FXMAC_NWCTRL_ENABLE_HS_MAC_MASK;
         control |= FXMAC_NWCTRL_TWO_PT_FIVE_GIG_MASK; /* Use high speed MAC */
         write_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *mut u32, control);
 
         /* High speed PCS control register */
         control = read_reg((FXMAC_IOBASE + FXMAC_GEM_USX_CONTROL_OFFSET) as *const u32);
         
         if (instance_p.config.speed == FXMAC_SPEED_25000)
         {
             control |= FXMAC_GEM_USX_HS_MAC_SPEED_2_5G;
         }
         
         control &= !(FXMAC_GEM_USX_TX_SCR_BYPASS | FXMAC_GEM_USX_RX_SCR_BYPASS);
         control |= FXMAC_GEM_USX_RX_SYNC_RESET;
         write_reg((FXMAC_IOBASE + FXMAC_GEM_USX_CONTROL_OFFSET) as *mut u32, control);
 
         control = read_reg((FXMAC_IOBASE + FXMAC_GEM_USX_CONTROL_OFFSET) as *const u32);
         control &= !FXMAC_GEM_USX_RX_SYNC_RESET;
         control |= FXMAC_GEM_USX_TX_DATAPATH_EN;
         control |= FXMAC_GEM_USX_SIGNAL_OK;
 
         write_reg((FXMAC_IOBASE + FXMAC_GEM_USX_CONTROL_OFFSET) as *mut u32, control);
 
     }
     else if (instance_p.config.interface == FXMAC_PHY_INTERFACE_MODE_SGMII)
     {
         config = read_reg((FXMAC_IOBASE + FXMAC_NWCFG_OFFSET) as *const u32);
         config |= FXMAC_NWCFG_PCSSEL_MASK | FXMAC_NWCFG_SGMII_MODE_ENABLE_MASK;
 
         config &= !(FXMAC_NWCFG_100_MASK | FXMAC_NWCFG_FDEN_MASK|FXMAC_NWCFG_LENGTH_FIELD_ERROR_FRAME_DISCARD_MASK);
 
         if instance_p->moudle_id >= 2
         {
             config &= !FXMAC_NWCFG_1000_MASK;
         }
 
         if instance_p.config.duplex != 0
         {
             config |= FXMAC_NWCFG_FDEN_MASK;
         }
 
         if instance_p.config.speed == FXMAC_SPEED_100
         {
             config |= FXMAC_NWCFG_100_MASK;
         }
         else if instance_p.config.speed == FXMAC_SPEED_1000
         {
             config |= FXMAC_NWCFG_1000_MASK;
         }
 
         write_reg((FXMAC_IOBASE + FXMAC_NWCFG_OFFSET) as *mut u32, config);
 
         if instance_p.config.speed == FXMAC_SPEED_2500
         {
             control = read_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *const u32);
             control |= FXMAC_NWCTRL_TWO_PT_FIVE_GIG_MASK;
             write_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *mut u32, control);
         }
         else
         {
             control = read_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *const u32);
             control &= !FXMAC_NWCTRL_TWO_PT_FIVE_GIG_MASK;
             write_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *mut u32, control);
         }
 
         control = read_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *const u32);
         control &= !FXMAC_NWCTRL_ENABLE_HS_MAC_MASK;
         write_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *mut u32, control);
 
         control = read_reg((FXMAC_IOBASE + FXMAC_PCS_CONTROL_OFFSET) as *const u32);
         control |= FXMAC_PCS_CONTROL_ENABLE_AUTO_NEG;
         write_reg((FXMAC_IOBASE + FXMAC_PCS_CONTROL_OFFSET) as *mut u32, control);
     }
     else
     {
         config = read_reg((FXMAC_IOBASE + FXMAC_NWCFG_OFFSET) as *const u32);
 
         info!("select rgmii");
 
         config &= !FXMAC_NWCFG_PCSSEL_MASK;
         config &= !(FXMAC_NWCFG_100_MASK | FXMAC_NWCFG_FDEN_MASK);
 
         if instance_p->moudle_id >= 2
         {
             config &= !FXMAC_NWCFG_1000_MASK;
         }
 
         if instance_p.config.duplex != 0
         {
             config |= FXMAC_NWCFG_FDEN_MASK;
         }
 
         if instance_p.config.speed == FXMAC_SPEED_100
         {
             config |= FXMAC_NWCFG_100_MASK;
         }
         else if instance_p.config.speed == FXMAC_SPEED_1000
         {
             config |= FXMAC_NWCFG_1000_MASK;
         }
 
         write_reg((FXMAC_IOBASE + FXMAC_NWCFG_OFFSET) as *mut u32, config);
 
         control = read_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *const u32);
         control &= !FXMAC_NWCTRL_ENABLE_HS_MAC_MASK; /* Use high speed MAC */
         write_reg((FXMAC_IOBASE + FXMAC_NWCTRL_OFFSET) as *mut u32, control);
     }
 }
