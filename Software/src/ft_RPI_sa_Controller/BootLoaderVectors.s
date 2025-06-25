    .syntax unified
    .arch armv7-m

    .section .BootLoader.Vectors, "a"
    .align 2

    .long   __StackTop                                      /* Top of Stack */
    .long   BL_DefaultISR                                   /* Reset Handler */
    .long   BL_DefaultISR                                     /* NMI Handler*/
    .long   BL_DefaultISR                               /* Hard Fault Handler*/
    .long   BL_DefaultISR                               /* MPU Fault Handler*/
    .long   BL_DefaultISR                                /* Bus Fault Handler*/
    .long   BL_DefaultISR                              /* Usage Fault Handler*/
    .long   0                                               /* Reserved*/
    .long   0                                               /* Reserved*/
    .long   0                                               /* Reserved*/
    .long   0                                               /* Reserved*/
    .long   BL_DefaultISR                                     /* SVCall Handler*/
    .long   BL_DefaultISR                                /* Debug Monitor Handler*/
    .long   0                                               /* Reserved*/
    .long   BL_DefaultISR                                  /* PendSV Handler*/
    .long   BL_DefaultISR                                 /* SysTick Handler*/

                                                            /* External Interrupts*/
    .long   BL_DefaultISR                                 /* DMA channel 0 transfer complete*/
    .long   BL_DefaultISR                                 /* DMA channel 1 transfer complete*/
    .long   BL_DefaultISR                                 /* DMA channel 2 transfer complete*/
    .long   BL_DefaultISR                                 /* DMA channel 3 transfer complete*/
    .long   BL_DefaultISR                                 /* DMA channel 4 transfer complete*/
    .long   BL_DefaultISR                                 /* DMA channel 5 transfer complete*/
    .long   BL_DefaultISR                                 /* DMA channel 6 transfer complete*/
    .long   BL_DefaultISR                                 /* DMA channel 7 transfer complete*/
    .long   BL_DefaultISR                                 /* DMA channel 8 transfer complete*/
    .long   BL_DefaultISR                                 /* DMA channel 9 transfer complete*/
    .long   BL_DefaultISR                                /* DMA channel 10 transfer complete*/
    .long   BL_DefaultISR                                /* DMA channel 11 transfer complete*/
    .long   BL_DefaultISR                                /* DMA channel 12 transfer complete*/
    .long   BL_DefaultISR                                /* DMA channel 13 transfer complete*/
    .long   BL_DefaultISR                                /* DMA channel 14 transfer complete*/
    .long   BL_DefaultISR                                /* DMA channel 15 transfer complete*/
    .long   BL_DefaultISR                            /* DMA error interrupt channels 0-15*/
    .long   BL_DefaultISR                                  /* FPU sources*/
    .long   BL_DefaultISR                                 /* FTFC Command complete*/
    .long   BL_DefaultISR                       /* FTFC Read collision*/
    .long   BL_DefaultISR                              /* PMC Low voltage detect interrupt*/
    .long   BL_DefaultISR                           /* FTFC Double bit fault detect*/
    .long   BL_DefaultISR                             /* Single interrupt vector for WDOG and EWM*/
    .long   BL_DefaultISR                                  /* RCM Asynchronous Interrupt*/
    .long   BL_DefaultISR                        /* LPI2C0 Master Interrupt*/
    .long   BL_DefaultISR                         /* LPI2C0 Slave Interrupt*/
    .long   BL_DefaultISR                               /* LPSPI0 Interrupt*/
    .long   BL_DefaultISR                               /* LPSPI1 Interrupt*/
    .long   BL_DefaultISR                               /* LPSPI2 Interrupt*/
    .long   BL_DefaultISR                           /* Reserved Interrupt 45*/
    .long   BL_DefaultISR                           /* Reserved Interrupt 46*/
    .long   BL_DefaultISR                         /* LPUART0 Transmit / Receive Interrupt*/
    .long   BL_DefaultISR                           /* Reserved Interrupt 48*/
    .long   BL_LPUART1_RxTx_IRQHandler                         /* LPUART1 Transmit / Receive  Interrupt*/

	.end
