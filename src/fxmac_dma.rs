


FError FXmacInitDma(FXmacLwipPort *instance_p)
{
    FXmacBd bdtemplate;
    FXmacBdRing *rxringptr, *txringptr;
    FXmacBd *rxbd;
    struct pbuf *p;
    FError status;
    int i;
    u32 bdindex = 0;
    u32 *temp;

        FXMAC_LWIP_PORT_XMAC_PRINT_I("%s,\r\n", __func__);

    /*
     * The BDs need to be allocated in uncached memory. Hence the 1 MB
     * address range allocated for Bd_Space is made uncached
     * by setting appropriate attributes in the translation table.
     * The Bd_Space is aligned to 1MB and has a size of 1 MB. This ensures
     * a reserved uncached area used only for BDs.
     */

    rxringptr = &FXMAC_GET_RXRING(instance_p->instance);
    txringptr = &FXMAC_GET_TXRING(instance_p->instance);
    FXMAC_LWIP_PORT_XMAC_PRINT_I("rxringptr: 0x%08x\r\n", rxringptr);
    FXMAC_LWIP_PORT_XMAC_PRINT_I("txringptr: 0x%08x\r\n", txringptr);

    FXMAC_LWIP_PORT_XMAC_PRINT_I("rx_bdspace: %p \r\n", instance_p->buffer.rx_bdspace);
    FXMAC_LWIP_PORT_XMAC_PRINT_I("tx_bdspace: %p \r\n", instance_p->buffer.tx_bdspace);

    /* Setup RxBD space. */
    FXMAC_BD_CLEAR(&bdtemplate);

    /* Create the RxBD ring */
    status = FXmacBdRingCreate(rxringptr, (uintptr)instance_p->buffer.rx_bdspace,
                               (uintptr)instance_p->buffer.rx_bdspace, BD_ALIGNMENT,
                               FXMAX_RX_PBUFS_LENGTH);

    if (status != FT_SUCCESS)
    {
        FXMAC_LWIP_PORT_XMAC_PRINT_E("Error setting up RxBD space\r\n");
        return ERR_IF;
    }

    status = FXmacBdRingClone(rxringptr, &bdtemplate, FXMAC_RECV);
    if (status != FT_SUCCESS)
    {
        FXMAC_LWIP_PORT_XMAC_PRINT_E("Error initializing RxBD space\r\n");
        return ERR_IF;
    }

    FXMAC_BD_CLEAR(&bdtemplate);
    FXMAC_BD_SET_STATUS(&bdtemplate, FXMAC_TXBUF_USED_MASK);

    /* Create the TxBD ring */
    status = FXmacBdRingCreate(txringptr, (uintptr)instance_p->buffer.tx_bdspace,
                               (uintptr)instance_p->buffer.tx_bdspace, BD_ALIGNMENT,
                               FXMAX_TX_PBUFS_LENGTH);

    if (status != FT_SUCCESS)
    {
        return ERR_IF;
    }

    /* We reuse the bd template, as the same one will work for both rx and tx. */
    status = FXmacBdRingClone(txringptr, &bdtemplate, FXMAC_SEND);
    if (status != FT_SUCCESS)
    {
        return ERR_IF;
    }

    /*
     * Allocate RX descriptors, 1 RxBD at a time.
     */
    FXMAC_LWIP_PORT_XMAC_PRINT_I("Allocate RX descriptors, 1 RxBD at a time.");
    for (i = 0; i < FXMAX_RX_PBUFS_LENGTH; i++)
    {
        if (instance_p->feature & FXMAC_LWIP_PORT_CONFIG_JUMBO)
        {
            p = pbuf_alloc(PBUF_RAW, FXMAC_MAX_FRAME_SIZE_JUMBO, PBUF_RAM);
        }
        else
        {
            p = pbuf_alloc(PBUF_RAW, FXMAC_MAX_FRAME_SIZE, PBUF_POOL);
        }

        if (!p)
        {
#if LINK_STATS
            lwip_stats.link.memerr++;
            lwip_stats.link.drop++;
#endif
            FXMAC_LWIP_PORT_XMAC_PRINT_E("unable to alloc pbuf in InitDma\r\n");
            return ERR_IF;
        }
        status = FXmacBdRingAlloc(rxringptr, 1, &rxbd);
        if (status != FT_SUCCESS)
        {
            FXMAC_LWIP_PORT_XMAC_PRINT_E("InitDma: Error allocating RxBD\r\n");
            pbuf_free(p);
            return ERR_IF;
        }
        /* Enqueue to HW */
        status = FXmacBdRingToHw(rxringptr, 1, rxbd);
        if (status != FT_SUCCESS)
        {
            FXMAC_LWIP_PORT_XMAC_PRINT_E("Error: committing RxBD to HW\r\n");
            pbuf_free(p);
            FXmacBdRingUnAlloc(rxringptr, 1, rxbd);
            return ERR_IF;
        }

        bdindex = FXMAC_BD_TO_INDEX(rxringptr, rxbd);
        temp = (u32 *)rxbd;
        *temp = 0;
        if (bdindex == (FXMAX_RX_PBUFS_LENGTH - 1))
        {
            *temp = 0x00000002; /* Marks last descriptor in receive buffer descriptor list */
        }
        temp++;
        *temp = 0; /* Clear word 1 in  descriptor */
        DSB();

        if (instance_p->feature & FXMAC_LWIP_PORT_CONFIG_JUMBO)
        {
            FCacheDCacheInvalidateRange((uintptr)p->payload, (uintptr)FXMAC_MAX_FRAME_SIZE_JUMBO);
        }
        else
        {
            FCacheDCacheInvalidateRange((uintptr)p->payload, (uintptr)FXMAC_MAX_FRAME_SIZE);
        }
        FXMAC_BD_SET_ADDRESS_RX(rxbd, (uintptr)p->payload);

        instance_p->buffer.rx_pbufs_storage[bdindex] = (uintptr)p;
    }
    
    FXmacSetQueuePtr(&(instance_p->instance), instance_p->instance.tx_bd_queue.bdring.phys_base_addr, 0, (u16)FXMAC_SEND);
    FXmacSetQueuePtr(&(instance_p->instance), instance_p->instance.rx_bd_queue.bdring.phys_base_addr, 0, (u16)FXMAC_RECV);
    
    if ((instance_p->instance).config.caps & FXMAC_CAPS_TAILPTR)
    {   
        FXMAC_WRITEREG32((instance_p->instance).config.base_address, FXMAC_TAIL_QUEUE(0), BIT(31)|0);
    }

    return 0;
}

/* Reset Tx and Rx DMA pointers after FXmacStop */
void ResetDma(FXmacLwipPort *instance_p)
{
        FXMAC_LWIP_PORT_XMAC_PRINT_I("%s,\r\n", __func__);

    FXmacBdRing *txringptr = &FXMAC_GET_TXRING(instance_p->instance);
    FXmacBdRing *rxringptr = &FXMAC_GET_RXRING(instance_p->instance);

    FXmacBdringPtrReset(txringptr, instance_p->buffer.tx_bdspace);
    FXmacBdringPtrReset(rxringptr, instance_p->buffer.rx_bdspace);

    FXmacSetQueuePtr(&(instance_p->instance), instance_p->instance.tx_bd_queue.bdring.phys_base_addr, 0, (u16)FXMAC_SEND);
    FXmacSetQueuePtr(&(instance_p->instance), instance_p->instance.rx_bd_queue.bdring.phys_base_addr, 0, (u16)FXMAC_RECV);
}


static void FXmacSetupIsr(FXmacLwipPort *instance_p)
{
        FXMAC_LWIP_PORT_XMAC_PRINT_I("%s,\r\n", __func__);

    u32 cpu_id;
    GetCpuId(&cpu_id);
    InterruptSetTargetCpus(instance_p->instance.config.queue_irq_num[0], cpu_id);
    /* Setup callbacks */
    FXmacSetHandler(&instance_p->instance, FXMAC_HANDLER_DMASEND, FXmacSendHandler, instance_p);
    FXmacSetHandler(&instance_p->instance, FXMAC_HANDLER_DMARECV, FXmacRecvIsrHandler, instance_p);
    FXmacSetHandler(&instance_p->instance, FXMAC_HANDLER_ERROR, FXmacErrorHandler, instance_p);
    FXmacSetHandler(&instance_p->instance, FXMAC_HANDLER_LINKCHANGE, FXmacLinkChange, instance_p);

    InterruptSetPriority(instance_p->instance.config.queue_irq_num[0], IRQ_PRIORITY_VALUE_12);
    // setup interrupt handler
    InterruptInstall(instance_p->instance.config.queue_irq_num[0], FXmacLwipPortIntrHandler, &instance_p->instance, "fxmac");
    InterruptUmask(instance_p->instance.config.queue_irq_num[0]);
}

void CleanDmaTxdescs(FXmacLwipPort *instance_p)
{
    FXmacBd bdtemplate;
    FXmacBdRing *txringptr;

        FXMAC_LWIP_PORT_XMAC_PRINT_I("%s,\r\n", __func__);

    txringptr = &FXMAC_GET_TXRING((instance_p->instance));
    FXMAC_BD_CLEAR(&bdtemplate);
    FXMAC_BD_SET_STATUS(&bdtemplate, FXMAC_TXBUF_USED_MASK);

    FXmacBdRingCreate(txringptr, (uintptr)instance_p->buffer.tx_bdspace,
                      (uintptr)instance_p->buffer.tx_bdspace, BD_ALIGNMENT,
                      sizeof(instance_p->buffer.tx_bdspace));

    FXmacBdRingClone(txringptr, &bdtemplate, FXMAC_SEND);
}

