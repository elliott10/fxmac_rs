/*
 * Write data to the specified PHY register. The Ethernet driver does not
 * require the device to be stopped before writing to the PHY.  Although it is
 * probably a good idea to stop the device, it is the responsibility of the
 * application to deem this necessary. The MAC provides the driver with the
 * ability to talk to a PHY that adheres to the Media Independent Interface
 * (MII) as defined in the IEEE 802.3 standard.
 *
 * Prior to PHY access with this function, the user should have setup the MDIO
 * clock with FXmacSetMdioDivisor().
 */
fn FXmacPhyWrite(instance_p &mut FXmac, phy_address: u32, register_num: u32, phy_data: u16) -> u32
{
    let mgtcr: mut u32 = 0;
    let ipisr: mut u32 = 0;
    let ip_write_temp: mut u32 = 0;
    let status: mut u32 = 0;

    debug!("FXmacPhyWrite, phy_address={:#x}, register_num={}, phy_data={:#x}", phy_address, register_num, phy_data);

    /* Make sure no other PHY operation is currently in progress */
    if (!(read_reg((instance_p.config.base_address + FXMAC_NWSR_OFFSET) as *const u32) &
           FXMAC_NWSR_MDIOIDLE_MASK)) == 1
    {   
        status = FXMAC_ERR_PHY_BUSY;
        error!("FXmacPhyRead error: PHY busy!");
    }else{   
        /* Construct mgtcr mask for the operation */
        mgtcr = FXMAC_PHYMNTNC_OP_MASK | FXMAC_PHYMNTNC_OP_W_MASK |
                (phy_address << FXMAC_PHYMNTNC_PHAD_SHFT_MSK) |
                (register_num << FXMAC_PHYMNTNC_PREG_SHFT_MSK) | phy_data as u32;

        /* Write mgtcr and wait for completion */
        write_reg((instance_p.config.base_address + FXMAC_PHYMNTNC_OFFSET) as *mut u32, mgtcr);

        loop{
            ipisr = read_reg((instance_p.config.base_address + FXMAC_NWSR_OFFSET) as *const u32);
            ip_write_temp = ipisr;

            if !(ip_write_temp & FXMAC_NWSR_MDIOIDLE_MASK) == 0 {
                break;
            }
        }

        status = FT_SUCCESS;
    }   
    
    status
}

fn FXmacPhyRead(instance_p &mut FXmac, phy_address: u32, register_num: u32, phydat_aptr: &mut u16) -> u32
{
    let mgtcr: mut u32 = 0;
    let ipisr: mut u32 = 0;
    let IpReadTemp: mut u32 = 0;
    let status: mut u32 = 0;

    /* Make sure no other PHY operation is currently in progress */
    if (!(read_reg((instance_p.config.base_address + FXMAC_NWSR_OFFSET) as *const u32) & FXMAC_NWSR_MDIOIDLE_MASK)) == 1
    {   
        status = FXMAC_ERR_PHY_BUSY;
        error!("FXmacPhyRead error: PHY busy!");
    }else{   
        /* Construct mgtcr mask for the operation */
        mgtcr = FXMAC_PHYMNTNC_OP_MASK | FXMAC_PHYMNTNC_OP_R_MASK |
        (phy_address << FXMAC_PHYMNTNC_PHAD_SHFT_MSK) |
        (register_num << FXMAC_PHYMNTNC_PREG_SHFT_MSK);

        /* Write mgtcr and wait for completion */
        write_reg((instance_p.config.base_address + FXMAC_PHYMNTNC_OFFSET) as *mut u32, mgtcr);

        loop{
            ipisr = read_reg((instance_p.config.base_address + FXMAC_NWSR_OFFSET) as *const u32);
            IpReadTemp = ipisr;

            if !(IpReadTemp & FXMAC_NWSR_MDIOIDLE_MASK) == 0 {
                break;
            }
        }

        // Read data
        phydat_aptr = read_reg((instance_p.config.base_address + FXMAC_PHYMNTNC_OFFSET) as *const u32) as u16;
    debug!("FXmacPhyRead, phy_address={:#x}, register_num={}, phydat_aptr={:#x}", phy_address, register_num, phydat_aptr);

        status = FT_SUCCESS;
    }   
    
    status
}


/// FXmacPhyInit
/// Setup the PHYs for proper speed setting.
pub fn FXmacPhyInit(
    mut instance_p: &mut FXmac,
    mut speed: u32,
    mut duplex_mode: u32,
    mut autonegotiation_en: u32,
    mut reset_flag: u32,
) -> u32 {
    info!("FXmacPhyInit, speed={}, duplex_mode={}, autonegotiation_en={}, reset_flag={}",
        duplex_mode,
        autonegotiation_en,
        reset_flag,
    );
    let mut ret: u32 = 0;
    let mut phy_addr: u32 = 0;
    if FXmacDetect(instance_p, &mut phy_addr) != 0 {
        error!("Phy is not found.");
        return FXMAC_PHY_IS_NOT_FOUND;
    }
    info!("Setting phy addr is {}", phy_addr);
    instance_p.phy_address = phy_addr;
    if reset_flag != 0 {
        FXmacPhyReset(instance_p, phy_addr);
    }
    if autonegotiation_en != 0 {
        ret = FXmacGetIeeePhySpeed(instance_p, phy_addr);
        if ret != 0 {
            return ret;
        }
    } else {
        info!("Set the communication speed manually.");
    assert!(speed != FXMAC_SPEED_1000, "The speed must be 100M or 10M!");

        ret = FXmacConfigureIeeePhySpeed(instance_p, phy_addr, speed, duplex_mode);
        if ret != 0 {
            error!("Failed to manually set the phy.");
            return ret;
        }
    }
    instance_p.link_status = FXMAC_LINKUP;

    FT_SUCCESS
}

fn FXmacDetect(instance_p: &mut FXmac, phy_addr_p: &mut u32) -> u32
{
    let phy_addr: mut u32 = 0;
    let phy_reg: mut u16 = 0;
    let phy_id1_reg: mut u16 = 0;
    let phy_id2_reg: mut u16 = 0;

    for phy_addr in 0..FXMAC_PHY_MAX_NUM
    {   
        let mut ret: u32 = FXmacPhyRead(instance_p, phy_addr, PHY_STATUS_REG_OFFSET, &mut phy_reg);
        if (ret != FT_SUCCESS)
        {
            error!("Phy operation is busy.");
            return ret;
        }
        info!("Phy status reg is {:#x}", phy_reg);

        if (phy_reg != 0xffff)
        {
            ret = FXmacPhyRead(instance_p, phy_addr, PHY_IDENTIFIER_1_REG, &phy_id1_reg);
            ret |= FXmacPhyRead(instance_p, phy_addr, PHY_IDENTIFIER_2_REG, &phy_id2_reg);
            info!("Phy id1 reg is {:#x}, Phy id2 reg is {:#x}", phy_id1_reg , phy_id2_reg);

            if ((ret == FT_SUCCESS) && (phy_id2_reg != 0) && (phy_id2_reg != 0xffff) && (phy_id1_reg != 0xffff))
            {
                *phy_addr_p = phy_addr;
                phy_addr_b = phy_addr;
                info!("Phy addr is {:#x}", phy_addr);
                return FT_SUCCESS;
            }
        }
    }

    FXMAC_PHY_IS_NOT_FOUND
}

/// FXmacPhyReset: Perform phy software reset
 fn FXmacPhyReset(instance_p: &mut FXmac, phy_addr: u32) -> u32
 {
     let control: mut u16 = 0;
 
     let ret: mut u32 = FXmacPhyRead(instance_p, phy_addr, PHY_CONTROL_REG_OFFSET, &control);
     if (ret != FT_SUCCESS)
     {
         error!("FXmacPhyReset, read PHY_CONTROL_REG_OFFSET is error");
         return ret;
     }
 
     control |= PHY_CONTROL_RESET_MASK;
 
     ret = FXmacPhyWrite(instance_p, phy_addr, PHY_CONTROL_REG_OFFSET, control);
     if (ret != FT_SUCCESS)
     {
         error!("FXmacPhyReset, write PHY_CONTROL_REG_OFFSET is error");
         return ret;
     }
 
     loop
     {
         ret = FXmacPhyRead(instance_p, phy_addr, PHY_CONTROL_REG_OFFSET, &control);
         if (ret != FT_SUCCESS)
         {
             error!("FXmacPhyReset, read PHY_CONTROL_REG_OFFSET is error");
             return ret;
         }
         if (control & PHY_CONTROL_RESET_MASK) == 0 {
            break;
         }
     }
 
     info!("Phy reset end.");
     ret
 }


fn FXmacGetIeeePhySpeed(instance_p: &mut FXmac, phy_addr: u32) -> u32
{
    let temp: mut u16 = 0;
    let temp2: mut u16 = 0;
    let control: mut u16 = 0;
    let status: mut u16 = 0;
    let negotitation_timeout_cnt: mut u32 = 0;

    info!("Start phy auto negotiation.");

    let mut ret: u32 = FXmacPhyRead(instance_p, phy_addr, PHY_CONTROL_REG_OFFSET, &control);
    if (ret != FT_SUCCESS)
    {
        error!("FXmacGetIeeePhySpeed,read PHY_CONTROL_REG_OFFSET is error");
        return ret;
    }

    control |= PHY_CONTROL_AUTONEGOTIATE_ENABLE;
    control |= PHY_CONTROL_AUTONEGOTIATE_RESTART;
    ret = FXmacPhyWrite(instance_p, phy_addr, PHY_CONTROL_REG_OFFSET, control);
    if (ret != FT_SUCCESS)
    {
        error!("FXmacGetIeeePhySpeed,write PHY_CONTROL_REG_OFFSET is error");
        return ret;
    }

    info!("Waiting for phy to complete auto negotiation.");
    do{
        FDriverMdelay(50);

        ret = FXmacPhyRead(instance_p, phy_addr, PHY_STATUS_REG_OFFSET, &status);
        if (ret != FT_SUCCESS)
        {
            error!("FXmacGetIeeePhySpeed,read PHY_STATUS_REG_OFFSET is error");
            return ret;
        }


        if (negotitation_timeout_cnt++ >= 0xff)
        {
            error!("Auto negotiation is error.");
            return FXMAC_PHY_AUTO_AUTONEGOTIATION_FAILED;
        }

        if (status & PHY_STATUS_AUTONEGOTIATE_COMPLETE) != 0 {
            break
        }
    }
    
    info!("Auto negotiation complete.");

    ret = FXmacPhyRead(instance_p, phy_addr, PHY_SPECIFIC_STATUS_REG, &temp);
    if (ret != FT_SUCCESS)
    {
        error!("FXmacGetIeeePhySpeed,read PHY_SPECIFIC_STATUS_REG is error");
        return ret;
    }

    info!("Temp is {:#x}", temp);
    ret = FXmacPhyRead(instance_p, phy_addr, PHY_STATUS_REG_OFFSET, &temp2);
    if (ret != FT_SUCCESS)
    {
        error!("FXmacGetIeeePhySpeed,read PHY_STATUS_REG_OFFSET is error");
        return ret;
    }

    info!("Temp2 is {:#x}", temp2);

    if (temp & (1 << 13)) != 0
    {
        info!("Duplex is full.");
        instance_p.config.duplex = 1;
    }
    else
    {
        info!("Duplex is half.");
        instance_p.config.duplex = 0;
    }

    if (temp & 0xC000) == PHY_SPECIFIC_STATUS_SPEED_1000M
    {
        info!("Speed is 1000M.");
        instance_p.config.speed = 1000;
    }
    else if (temp & 0xC000) == PHY_SPECIFIC_STATUS_SPEED_100M
    {
        info!("Speed is 100M.");
        instance_p.config.speed = 100;
    }
    else
    {
        info!("Speed is 10M.");
        instance_p.config.speed = 10;
    }

    FT_SUCCESS
}

fn FXmacConfigureIeeePhySpeed(instance_p: &mut FXmac, phy_addr: u32, speed: u32, duplex_mode: u32) -> u32
{
    let control: mut u16 = 0;;
    let autonereg: mut u16 = 0;;
    let specific_reg: mut u16 = 0;

    info("Manual setting, phy_addr is {:#x},speed {}, duplex_mode is {}.", phy_addr, speed, duplex_mode);

    let mut ret: u32 = FXmacPhyRead(instance_p, phy_addr, PHY_AUTONEGO_ADVERTISE_REG, &autonereg);
    if (ret != FT_SUCCESS)
    {
        error!("FXmacConfigureIeeePhySpeed, read PHY_AUTONEGO_ADVERTISE_REG is error.");
        return ret;
    }

    autonereg |= PHY_AUTOADVERTISE_ASYMMETRIC_PAUSE_MASK;
    autonereg |= PHY_AUTOADVERTISE_PAUSE_MASK;
    ret = FXmacPhyWrite(instance_p, phy_addr, PHY_AUTONEGO_ADVERTISE_REG, autonereg);
    if (ret != FT_SUCCESS)
    {
        error!("FXmacConfigureIeeePhySpeed, write PHY_AUTONEGO_ADVERTISE_REG is error.");
        return ret;
    }

    ret = FXmacPhyRead(instance_p, phy_addr, PHY_CONTROL_REG_OFFSET, &control);
    if (ret != FT_SUCCESS)
    {
        error!("FXmacConfigureIeeePhySpeed, read PHY_AUTONEGO_ADVERTISE_REG is error.");
        return ret;
    }
    info!("PHY_CONTROL_REG_OFFSET is {:#x}.", control);

    control &= !PHY_CONTROL_LINKSPEED_1000M;
    control &= !PHY_CONTROL_LINKSPEED_100M;
    control &= !PHY_CONTROL_LINKSPEED_10M;

    if speed == 100
    {
        control |= PHY_CONTROL_LINKSPEED_100M;
    }
    else if speed == 10
    {
        control |= PHY_CONTROL_LINKSPEED_10M;
    }

    if duplex_mode == 1
    {
        control |= PHY_CONTROL_FULL_DUPLEX_MASK;
    }
    else
    {
        control &= !PHY_CONTROL_FULL_DUPLEX_MASK;
    }

    /* disable auto-negotiation */
    control &= !PHY_CONTROL_AUTONEGOTIATE_ENABLE;
    control &= !PHY_CONTROL_AUTONEGOTIATE_RESTART;

    ret = FXmacPhyWrite(instance_p, phy_addr, PHY_CONTROL_REG_OFFSET, control); /* Technology Ability Field */
    if (ret != FT_SUCCESS)
    {
        error!("FXmacConfigureIeeePhySpeed, write PHY_AUTONEGO_ADVERTISE_REG is error.");
        return ret;
    }

    FDriverMdelay(1500);

    info!("Manual selection completed.");

    ret = FXmacPhyRead(instance_p, phy_addr, PHY_SPECIFIC_STATUS_REG, &specific_reg);
    if (ret != FT_SUCCESS)
    {
        error!("FXmacConfigureIeeePhySpeed, read PHY_SPECIFIC_STATUS_REG is error.");
        return ret;
    }

    info!("Specific reg is 0x%x.", specific_reg);

    if (specific_reg & (1 << 13)) != 0
    {
        info!("Duplex is full.");
        instance_p.config.duplex = 1;
    }
    else
    {
        info!("Duplex is half.");
        instance_p.config.duplex = 0;
    }

    if (specific_reg & 0xC000) == PHY_SPECIFIC_STATUS_SPEED_100M
    {
        info!("Speed is 100M.");
        instance_p.config.speed = 100;
    }
    else
    {
        info!("Speed is 10M.");
        instance_p.config.speed = 10;
    }

    FT_SUCCESS
}