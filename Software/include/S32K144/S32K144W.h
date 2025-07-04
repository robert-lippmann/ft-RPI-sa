/*
** ###################################################################
**     Processor:           S32K144W
**     Reference manual:    S32K1XX RM Rev.12.1
**     Version:             rev. 1.2, 2020-03-09
**     Build:               b200309
**
**     Abstract:
**         Peripheral Access Layer for S32K144W
**
**     Copyright 1997-2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2020 NXP
**
**     THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
**     IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
**     OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
**     IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
**     INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
**     SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
**     HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
**     STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
**     IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
**     THE POSSIBILITY OF SUCH DAMAGE.
**
**     http:                 www.nxp.com
**     mail:                 support@nxp.com
**
**     Revisions:
**     - rev. 1.0 Taru Shree 23-Oct-19  Initial release based on 
**                                      RM Rev.12 Draft B.
**     - rev. 1.1 Taru Shree 12-Feb-20  1)Initial release based on
**                                      RM Rev.12.
**                                      2)CAN- 
**                                      CAN_RAMn_COUNT is changed from
**                                      128 to 256.
**                                      3)DMA-
**                                      DCHMID register array is removed.
**                                      4)GPIO-
**                                      Following peripheral instance names are 
**                                      changed-
**                                      -from GPIOA to PTA.
**                                      -from GPIOB to PTB.
**                                      -from GPIOC to PTC.
**                                      -from GPIOD to PTD.
**                                      -from GPIOE to PTE.
**                                      5)TRGMUX-
**                                      -ADCx_REG is changed to ADCx.
**                                      -CMP0_REG is changed to CMP0.
**                                      -FTMx_REG is changed to FTMx.
**                                      -PDBx_REG is changed to PDBx.
**                                      -FLEXIO_REG is changed to FLEXIO.
**                                      -LPIT0_REG is changed to LPIT0.
**                                      -LPUARTx_REG is changed to LPUARTx.
**                                      -LPI2C0_REG is changed to LPI2C0.
**                                      -LPSPIx_REG is changed to LPSPIx.
**                                      -LPTMR0_REG is changed to LPTMR0. 
**     - rev.1.2 Taru Shree 09-March-20 1)Initial release based on
**                                      RM Rev.12.1. 
**                                      2)FTFM Interrupts are updated.   
**                                      3)PCC-
**                                      PCC_ATX register is removed.                            
** ###################################################################
*/

/*!
 * @file S32K144W.h
 * @version 1.2
 * @date 2020-03-09
 * @brief Peripheral Access Layer for S32K144W_M4
 *
 * This file contains register definitions and macros for easy access to their
 * bit fields.
 *
 * This file assumes LITTLE endian system.
 */

/**
* @page misra_violations MISRA-C:2012 violations
*
* @section [global]
* Violates MISRA 2012 Advisory Rule 2.3, local typedef not referenced
* The SoC header defines typedef for all modules.
*
* @section [global]
* Violates MISRA 2012 Advisory Rule 2.5, local macro not referenced
* The SoC header defines macros for all modules and registers.
*
* @section [global]
* Violates MISRA 2012 Advisory Directive 4.9, Function-like macro
* These are generated macros used for accessing the bit-fields from registers.
*
* @section [global]
* Violates MISRA 2012 Required Rule 5.1, identifier clash
* The supported compilers use more than 31 significant characters for identifiers.
*
* @section [global]
* Violates MISRA 2012 Required Rule 5.2, identifier clash
* The supported compilers use more than 31 significant characters for identifiers.
*
* @section [global]
* Violates MISRA 2012 Required Rule 5.4, identifier clash
* The supported compilers use more than 31 significant characters for identifiers.
*
* @section [global]
* Violates MISRA 2012 Required Rule 5.5, identifier clash
* The supported compilers use more than 31 significant characters for identifiers.
*
* @section [global]
* Violates MISRA 2012 Required Rule 21.1, defined macro '__I' is reserved to the compiler
* This type qualifier is needed to ensure correct I/O access and addressing.
*/


/* ----------------------------------------------------------------------------
   -- MCU activation
   ---------------------------------------------------------------------------- */

/* Prevention from multiple including the same memory map */
#if !defined(S32K144W_H_)  /* Check if memory map has not been already included */
#define S32K144W_H_
#define MCU_S32K144W

/* Check if another memory map has not been also included */
#if (defined(MCU_ACTIVE))
  #error S32K144W memory map: There is already included another memory map. Only one memory map can be included.
#endif /* (defined(MCU_ACTIVE)) */
#define MCU_ACTIVE

#include <stdint.h>

/** Memory map major version (memory maps with equal major version number are
 * compatible) */
#define MCU_MEM_MAP_VERSION 0x0100U
/** Memory map minor version */
#define MCU_MEM_MAP_VERSION_MINOR 0x0002U

/* ----------------------------------------------------------------------------
   -- Generic macros
   ---------------------------------------------------------------------------- */

/* IO definitions (access restrictions to peripheral registers) */
/**
*   IO Type Qualifiers are used
*   \li to specify the access to peripheral variables.
*   \li for automatic generation of peripheral register debug information.
*/
#ifndef __IO
#ifdef __cplusplus
  #define   __I     volatile             /*!< Defines 'read only' permissions                 */
#else
  #define   __I     volatile const       /*!< Defines 'read only' permissions                 */
#endif
#define     __O     volatile             /*!< Defines 'write only' permissions                */
#define     __IO    volatile             /*!< Defines 'read / write' permissions              */
#endif


/**
* @brief 32 bits memory read macro.
*/
#if !defined(REG_READ32)
  #define REG_READ32(address)               (*(volatile uint32_t*)(address))
#endif

/**
* @brief 32 bits memory write macro.
*/
#if !defined(REG_WRITE32)
  #define REG_WRITE32(address, value)       ((*(volatile uint32_t*)(address))= (uint32_t)(value))
#endif

/**
* @brief 32 bits bits setting macro.
*/
#if !defined(REG_BIT_SET32)
  #define REG_BIT_SET32(address, mask)      ((*(volatile uint32_t*)(address))|= (uint32_t)(mask))
#endif

/**
* @brief 32 bits bits clearing macro.
*/
#if !defined(REG_BIT_CLEAR32)
  #define REG_BIT_CLEAR32(address, mask)    ((*(volatile uint32_t*)(address))&= ((uint32_t)~((uint32_t)(mask))))
#endif

/**
* @brief 32 bit clear bits and set with new value
* @note It is user's responsability to make sure that value has only "mask" bits set - (value&~mask)==0
*/
#if !defined(REG_RMW32)
  #define REG_RMW32(address, mask, value)   (REG_WRITE32((address), ((REG_READ32(address)& ((uint32_t)~((uint32_t)(mask))))| ((uint32_t)(value)))))
#endif


/* ----------------------------------------------------------------------------
   -- Interrupt vector numbers
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Interrupt_vector_numbers Interrupt vector numbers
 * @{
 */

/** Interrupt Number Definitions */
#define NUMBER_OF_INT_VECTORS 170                /**< Number of interrupts in the Vector table */

typedef enum {
  /* Auxiliary constants */
  NotAvail_IRQn                = -128,             /**< Not available device specific interrupt */

  /* Core interrupts */
  NonMaskableInt_IRQn          = -14,              /**< Non Maskable Interrupt */
  HardFault_IRQn               = -13,              /**< Cortex-M4 SV Hard Fault Interrupt */
  MemoryManagement_IRQn        = -12,              /**< Cortex-M4 Memory Management Interrupt */
  BusFault_IRQn                = -11,              /**< Cortex-M4 Bus Fault Interrupt */
  UsageFault_IRQn              = -10,              /**< Cortex-M4 Usage Fault Interrupt */
  SVCall_IRQn                  = -5,               /**< Cortex-M4 SV Call Interrupt */
  DebugMonitor_IRQn            = -4,               /**< Cortex-M4 Debug Monitor Interrupt */
  PendSV_IRQn                  = -2,               /**< Cortex-M4 Pend SV Interrupt */
  SysTick_IRQn                 = -1,               /**< Cortex-M4 System Tick Interrupt */

  /* Device specific interrupts */
  DMA0_IRQn                    = 0,                /**< DMA channel 0 transfer complete */
  DMA1_IRQn                    = 1,                /**< DMA channel 1 transfer complete */
  DMA2_IRQn                    = 2,                /**< DMA channel 2 transfer complete */
  DMA3_IRQn                    = 3,                /**< DMA channel 3 transfer complete */
  DMA4_IRQn                    = 4,                /**< DMA channel 4 transfer complete */
  DMA5_IRQn                    = 5,                /**< DMA channel 5 transfer complete */
  DMA6_IRQn                    = 6,                /**< DMA channel 6 transfer complete */
  DMA7_IRQn                    = 7,                /**< DMA channel 7 transfer complete */
  DMA8_IRQn                    = 8,                /**< DMA channel 8 transfer complete */
  DMA9_IRQn                    = 9,                /**< DMA channel 9 transfer complete */
  DMA10_IRQn                   = 10,               /**< DMA channel 10 transfer complete */
  DMA11_IRQn                   = 11,               /**< DMA channel 11 transfer complete */
  DMA12_IRQn                   = 12,               /**< DMA channel 12 transfer complete */
  DMA13_IRQn                   = 13,               /**< DMA channel 13 transfer complete */
  DMA14_IRQn                   = 14,               /**< DMA channel 14 transfer complete */
  DMA15_IRQn                   = 15,               /**< DMA channel 15 transfer complete */
  DMA_Error_IRQn               = 16,               /**< DMA error interrupt channels 0-15 */
  MCM_IRQn                     = 17,               /**< FPU sources */
  FTFM_IRQn                    = 18,               /**< FTFM Command complete */
  Read_Collision_IRQn          = 19,               /**< FTFM Read collision */
  LVD_LVW_IRQn                 = 20,               /**< PMC Low voltage detect interrupt */
  FTFM_Fault_IRQn              = 21,               /**< FTFM Double bit fault detect */
  WDOG_EWM_IRQn                = 22,               /**< Interrupt request out before WDOG reset out, EWM output as interrupt */
  RCM_IRQn                     = 23,               /**< RCM Asynchronous Interrupt */
  LPI2C0_Master_IRQn           = 24,               /**< LPI2C0 Master Interrupt */
  LPI2C0_Slave_IRQn            = 25,               /**< LPI2C0 Slave Interrupt */
  LPSPI0_IRQn                  = 26,               /**< LPSPI0 Interrupt */
  LPSPI1_IRQn                  = 27,               /**< LPSPI1 Interrupt */
  LPSPI2_IRQn                  = 28,               /**< LPSPI2 Interrupt */
  LPUART0_RxTx_IRQn            = 31,               /**< LPUART0 Transmit / Receive Interrupt / Error / Overrun */
  LPUART1_RxTx_IRQn            = 33,               /**< LPUART1 Transmit / Receive Interrupt / Error / Overrun */
  LPUART2_RxTx_IRQn            = 35,               /**< LPUART2 Transmit / Receive Interrupt / Error / Overrun */
  ADC0_IRQn                    = 39,               /**< ADC0 interrupt request */
  ADC1_IRQn                    = 40,               /**< ADC1 interrupt request */
  CMP0_IRQn                    = 41,               /**< CMP0 interrupt request */
  ERM_single_fault_IRQn        = 44,               /**< ERM single bit error correction */
  ERM_double_fault_IRQn        = 45,               /**< ERM double bit error non-correctable */
  RTC_IRQn                     = 46,               /**< RTC alarm interrupt */
  RTC_Seconds_IRQn             = 47,               /**< RTC seconds interrupt */
  LPIT0_Ch0_IRQn               = 48,               /**< LPIT0 channel 0 overflow interrupt */
  LPIT0_Ch1_IRQn               = 49,               /**< LPIT0 channel 1 overflow interrupt */
  LPIT0_Ch2_IRQn               = 50,               /**< LPIT0 channel 2 overflow interrupt */
  LPIT0_Ch3_IRQn               = 51,               /**< LPIT0 channel 3 overflow interrupt */
  PDB0_IRQn                    = 52,               /**< PDB0 interrupt */
  SCG_IRQn                     = 57,               /**< SCG bus interrupt request */
  LPTMR0_IRQn                  = 58,               /**< LPTIMER interrupt request */
  PORTA_IRQn                   = 59,               /**< Port A pin detect interrupt */
  PORTB_IRQn                   = 60,               /**< Port B pin detect interrupt */
  PORTC_IRQn                   = 61,               /**< Port C pin detect interrupt */
  PORTD_IRQn                   = 62,               /**< Port D pin detect interrupt */
  PORTE_IRQn                   = 63,               /**< Port E pin detect interrupt */
  SWI_IRQn                     = 64,               /**< Software interrupt */
  PDB1_IRQn                    = 68,               /**< PDB1 interrupt */
  FLEXIO_IRQn                  = 69,               /**< Software interrupt */
  CAN0_ORed_IRQn               = 78,               /**< CAN0 OR'ed Bus in Off State. */
  CAN0_Error_IRQn              = 79,               /**< CAN0 Interrupt indicating that errors were detected on the CAN bus */
  CAN0_Wake_Up_IRQn            = 80,               /**< CAN0 Interrupt asserted when Pretended Networking operation is enabled, and a valid message matches the selected filter criteria during Low Power mode */
  CAN0_ORed_0_15_MB_IRQn       = 81,               /**< CAN0 OR'ed Message buffer (0-15) */
  CAN0_ORed_16_31_MB_IRQn      = 82,               /**< CAN0 OR'ed Message buffer (16-31) */
  CAN0_ORed_32_47_MB_IRQn      = 83,               /**< CAN0 OR'ed Message buffer (32-47) */
  CAN0_ORed_48_63_MB_IRQn      = 84,               /**< CAN0 OR'ed Message buffer (32-47) */
  CAN1_ORed_IRQn               = 85,               /**< CAN1 OR'ed Bus in Off State. */
  CAN1_Error_IRQn              = 86,               /**< CAN1 Interrupt indicating that errors were detected on the CAN bus */
  CAN1_ORed_0_15_MB_IRQn       = 88,               /**< CAN1 OR'ed Message buffer (0-15) */
  CAN1_ORed_16_31_MB_IRQn      = 89,               /**< CAN1 OR'ed Message buffer (16-31) */
  CAN1_ORed_32_47_MB_IRQn      = 90,               /**< CAN1 OR'ed Message buffer (32-47) */
  CAN1_ORed_48_63_MB_IRQn      = 91,               /**< CAN1 OR'ed Message buffer (32-47) */
  FTM0_Ch0_Ch1_IRQn            = 99,               /**< FTM0 Channel 0 and 1 interrupt */
  FTM0_Ch2_Ch3_IRQn            = 100,              /**< FTM0 Channel 2 and 3 interrupt */
  FTM0_Ch4_Ch5_IRQn            = 101,              /**< FTM0 Channel 4 and 5 interrupt */
  FTM0_Ch6_Ch7_IRQn            = 102,              /**< FTM0 Channel 6 and 7 interrupt */
  FTM0_Fault_IRQn              = 103,              /**< FTM0 Fault interrupt */
  FTM0_Ovf_Reload_IRQn         = 104,              /**< FTM0 Counter overflow and Reload interrupt */
  FTM1_Ch0_Ch1_IRQn            = 105,              /**< FTM1 Channel 0 and 1 interrupt */
  FTM1_Ch2_Ch3_IRQn            = 106,              /**< FTM1 Channel 2 and 3 interrupt */
  FTM1_Ch4_Ch5_IRQn            = 107,              /**< FTM1 Channel 4 and 5 interrupt */
  FTM1_Ch6_Ch7_IRQn            = 108,              /**< FTM1 Channel 6 and 7 interrupt */
  FTM1_Fault_IRQn              = 109,              /**< FTM1 Fault interrupt */
  FTM1_Ovf_Reload_IRQn         = 110,              /**< FTM1 Counter overflow and Reload interrupt */
  FTM2_Ch0_Ch1_IRQn            = 111,              /**< FTM2 Channel 0 and 1 interrupt */
  FTM2_Ch2_Ch3_IRQn            = 112,              /**< FTM2 Channel 2 and 3 interrupt */
  FTM2_Ch4_Ch5_IRQn            = 113,              /**< FTM2 Channel 4 and 5 interrupt */
  FTM2_Ch6_Ch7_IRQn            = 114,              /**< FTM2 Channel 6 and 7 interrupt */
  FTM2_Fault_IRQn              = 115,              /**< FTM2 Fault interrupt */
  FTM2_Ovf_Reload_IRQn         = 116,              /**< FTM2 Counter overflow and Reload interrupt */
  FTM3_Ch0_Ch1_IRQn            = 117,              /**< FTM3 Channel 0 and 1 interrupt */
  FTM3_Ch2_Ch3_IRQn            = 118,              /**< FTM3 Channel 2 and 3 interrupt */
  FTM3_Ch4_Ch5_IRQn            = 119,              /**< FTM3 Channel 4 and 5 interrupt */
  FTM3_Ch6_Ch7_IRQn            = 120,              /**< FTM3 Channel 6 and 7 interrupt */
  FTM3_Fault_IRQn              = 121,              /**< FTM3 Fault interrupt */
  FTM3_Ovf_Reload_IRQn         = 122,              /**< FTM3 Counter overflow and Reload interrupt */
  FTFM_EEERAM_ECC_ERR_IRQn     = 152,              /**< FFTM EEERAM ECC error */
  FTFM_Mgate_ECC_ERR_IRQn      = 153               /**< FFTM Mgate ECC error */
} IRQn_Type;

/*!
 * @}
 */ /* end of group Interrupt_vector_numbers */

/* ----------------------------------------------------------------------------
   -- S32_SCB Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup S32_SCB_Peripheral_Access_Layer S32_SCB Peripheral Access Layer
 * @{
 */


/** S32_SCB - Size of Registers Arrays */

/** S32_SCB - Register Layout Typedef */
typedef struct {
       uint8_t RESERVED_0[8];
  __IO uint32_t ACTLR;                             /**< Auxiliary Control Register,, offset: 0x8 */
       uint8_t RESERVED_1[3316];
  __I  uint32_t CPUID;                             /**< CPUID Base Register, offset: 0xD00 */
  __IO uint32_t ICSR;                              /**< Interrupt Control and State Register, offset: 0xD04 */
  __IO uint32_t VTOR;                              /**< Vector Table Offset Register, offset: 0xD08 */
  __IO uint32_t AIRCR;                             /**< Application Interrupt and Reset Control Register, offset: 0xD0C */
  __IO uint32_t SCR;                               /**< System Control Register, offset: 0xD10 */
  __IO uint32_t CCR;                               /**< Configuration and Control Register, offset: 0xD14 */
  __IO uint32_t SHPR1;                             /**< System Handler Priority Register 1, offset: 0xD18 */
  __IO uint32_t SHPR2;                             /**< System Handler Priority Register 2, offset: 0xD1C */
  __IO uint32_t SHPR3;                             /**< System Handler Priority Register 3, offset: 0xD20 */
  __IO uint32_t SHCSR;                             /**< System Handler Control and State Register, offset: 0xD24 */
  __IO uint32_t CFSR;                              /**< Configurable Fault Status Registers, offset: 0xD28 */
  __IO uint32_t HFSR;                              /**< HardFault Status register, offset: 0xD2C */
  __IO uint32_t DFSR;                              /**< Debug Fault Status Register, offset: 0xD30 */
  __IO uint32_t MMFAR;                             /**< MemManage Address Register, offset: 0xD34 */
  __IO uint32_t BFAR;                              /**< BusFault Address Register, offset: 0xD38 */
  __IO uint32_t AFSR;                              /**< Auxiliary Fault Status Register, offset: 0xD3C */
       uint8_t RESERVED_2[72];
  __IO uint32_t CPACR;                             /**< Coprocessor Access Control Register, offset: 0xD88 */
       uint8_t RESERVED_3[424];
  __IO uint32_t FPCCR;                             /**< Floating-point Context Control Register, offset: 0xF34 */
  __IO uint32_t FPCAR;                             /**< Floating-point Context Address Register, offset: 0xF38 */
  __IO uint32_t FPDSCR;                            /**< Floating-point Default Status Control Register, offset: 0xF3C */
} S32_SCB_Type, *S32_SCB_MemMapPtr;

 /** Number of instances of the S32_SCB module. */
#define S32_SCB_INSTANCE_COUNT                   (1u)


/* S32_SCB - Peripheral instance base addresses */
/** Peripheral S32_SCB base address */
#define S32_SCB_BASE                             (0xE000E000u)
/** Peripheral S32_SCB base pointer */
#define S32_SCB                                  ((S32_SCB_Type *)S32_SCB_BASE)
/** Array initializer of S32_SCB peripheral base addresses */
#define S32_SCB_BASE_ADDRS                       { S32_SCB_BASE }
/** Array initializer of S32_SCB peripheral base pointers */
#define S32_SCB_BASE_PTRS                        { S32_SCB }

/* ----------------------------------------------------------------------------
   -- S32_SCB Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup S32_SCB_Register_Masks S32_SCB Register Masks
 * @{
 */

/* ACTLR Bit Fields */
#define S32_SCB_ACTLR_DISMCYCINT_MASK            0x1u
#define S32_SCB_ACTLR_DISMCYCINT_SHIFT           0u
#define S32_SCB_ACTLR_DISMCYCINT_WIDTH           1u
#define S32_SCB_ACTLR_DISMCYCINT(x)              (((uint32_t)(((uint32_t)(x))<<S32_SCB_ACTLR_DISMCYCINT_SHIFT))&S32_SCB_ACTLR_DISMCYCINT_MASK)
#define S32_SCB_ACTLR_DISDEFWBUF_MASK            0x2u
#define S32_SCB_ACTLR_DISDEFWBUF_SHIFT           1u
#define S32_SCB_ACTLR_DISDEFWBUF_WIDTH           1u
#define S32_SCB_ACTLR_DISDEFWBUF(x)              (((uint32_t)(((uint32_t)(x))<<S32_SCB_ACTLR_DISDEFWBUF_SHIFT))&S32_SCB_ACTLR_DISDEFWBUF_MASK)
#define S32_SCB_ACTLR_DISFOLD_MASK               0x4u
#define S32_SCB_ACTLR_DISFOLD_SHIFT              2u
#define S32_SCB_ACTLR_DISFOLD_WIDTH              1u
#define S32_SCB_ACTLR_DISFOLD(x)                 (((uint32_t)(((uint32_t)(x))<<S32_SCB_ACTLR_DISFOLD_SHIFT))&S32_SCB_ACTLR_DISFOLD_MASK)
#define S32_SCB_ACTLR_DISFPCA_MASK               0x100u
#define S32_SCB_ACTLR_DISFPCA_SHIFT              8u
#define S32_SCB_ACTLR_DISFPCA_WIDTH              1u
#define S32_SCB_ACTLR_DISFPCA(x)                 (((uint32_t)(((uint32_t)(x))<<S32_SCB_ACTLR_DISFPCA_SHIFT))&S32_SCB_ACTLR_DISFPCA_MASK)
#define S32_SCB_ACTLR_DISOOFP_MASK               0x200u
#define S32_SCB_ACTLR_DISOOFP_SHIFT              9u
#define S32_SCB_ACTLR_DISOOFP_WIDTH              1u
#define S32_SCB_ACTLR_DISOOFP(x)                 (((uint32_t)(((uint32_t)(x))<<S32_SCB_ACTLR_DISOOFP_SHIFT))&S32_SCB_ACTLR_DISOOFP_MASK)
/* CPUID Bit Fields */
#define S32_SCB_CPUID_REVISION_MASK              0xFu
#define S32_SCB_CPUID_REVISION_SHIFT             0u
#define S32_SCB_CPUID_REVISION_WIDTH             4u
#define S32_SCB_CPUID_REVISION(x)                (((uint32_t)(((uint32_t)(x))<<S32_SCB_CPUID_REVISION_SHIFT))&S32_SCB_CPUID_REVISION_MASK)
#define S32_SCB_CPUID_PARTNO_MASK                0xFFF0u
#define S32_SCB_CPUID_PARTNO_SHIFT               4u
#define S32_SCB_CPUID_PARTNO_WIDTH               12u
#define S32_SCB_CPUID_PARTNO(x)                  (((uint32_t)(((uint32_t)(x))<<S32_SCB_CPUID_PARTNO_SHIFT))&S32_SCB_CPUID_PARTNO_MASK)
#define S32_SCB_CPUID_VARIANT_MASK               0xF00000u
#define S32_SCB_CPUID_VARIANT_SHIFT              20u
#define S32_SCB_CPUID_VARIANT_WIDTH              4u
#define S32_SCB_CPUID_VARIANT(x)                 (((uint32_t)(((uint32_t)(x))<<S32_SCB_CPUID_VARIANT_SHIFT))&S32_SCB_CPUID_VARIANT_MASK)
#define S32_SCB_CPUID_IMPLEMENTER_MASK           0xFF000000u
#define S32_SCB_CPUID_IMPLEMENTER_SHIFT          24u
#define S32_SCB_CPUID_IMPLEMENTER_WIDTH          8u
#define S32_SCB_CPUID_IMPLEMENTER(x)             (((uint32_t)(((uint32_t)(x))<<S32_SCB_CPUID_IMPLEMENTER_SHIFT))&S32_SCB_CPUID_IMPLEMENTER_MASK)
/* ICSR Bit Fields */
#define S32_SCB_ICSR_VECTACTIVE_MASK             0x1FFu
#define S32_SCB_ICSR_VECTACTIVE_SHIFT            0u
#define S32_SCB_ICSR_VECTACTIVE_WIDTH            9u
#define S32_SCB_ICSR_VECTACTIVE(x)               (((uint32_t)(((uint32_t)(x))<<S32_SCB_ICSR_VECTACTIVE_SHIFT))&S32_SCB_ICSR_VECTACTIVE_MASK)
#define S32_SCB_ICSR_RETTOBASE_MASK              0x800u
#define S32_SCB_ICSR_RETTOBASE_SHIFT             11u
#define S32_SCB_ICSR_RETTOBASE_WIDTH             1u
#define S32_SCB_ICSR_RETTOBASE(x)                (((uint32_t)(((uint32_t)(x))<<S32_SCB_ICSR_RETTOBASE_SHIFT))&S32_SCB_ICSR_RETTOBASE_MASK)
#define S32_SCB_ICSR_VECTPENDING_MASK            0x3F000u
#define S32_SCB_ICSR_VECTPENDING_SHIFT           12u
#define S32_SCB_ICSR_VECTPENDING_WIDTH           6u
#define S32_SCB_ICSR_VECTPENDING(x)              (((uint32_t)(((uint32_t)(x))<<S32_SCB_ICSR_VECTPENDING_SHIFT))&S32_SCB_ICSR_VECTPENDING_MASK)
#define S32_SCB_ICSR_ISRPENDING_MASK             0x400000u
#define S32_SCB_ICSR_ISRPENDING_SHIFT            22u
#define S32_SCB_ICSR_ISRPENDING_WIDTH            1u
#define S32_SCB_ICSR_ISRPENDING(x)               (((uint32_t)(((uint32_t)(x))<<S32_SCB_ICSR_ISRPENDING_SHIFT))&S32_SCB_ICSR_ISRPENDING_MASK)
#define S32_SCB_ICSR_ISRPREEMPT_MASK             0x800000u
#define S32_SCB_ICSR_ISRPREEMPT_SHIFT            23u
#define S32_SCB_ICSR_ISRPREEMPT_WIDTH            1u
#define S32_SCB_ICSR_ISRPREEMPT(x)               (((uint32_t)(((uint32_t)(x))<<S32_SCB_ICSR_ISRPREEMPT_SHIFT))&S32_SCB_ICSR_ISRPREEMPT_MASK)
#define S32_SCB_ICSR_PENDSTCLR_MASK              0x2000000u
#define S32_SCB_ICSR_PENDSTCLR_SHIFT             25u
#define S32_SCB_ICSR_PENDSTCLR_WIDTH             1u
#define S32_SCB_ICSR_PENDSTCLR(x)                (((uint32_t)(((uint32_t)(x))<<S32_SCB_ICSR_PENDSTCLR_SHIFT))&S32_SCB_ICSR_PENDSTCLR_MASK)
#define S32_SCB_ICSR_PENDSTSET_MASK              0x4000000u
#define S32_SCB_ICSR_PENDSTSET_SHIFT             26u
#define S32_SCB_ICSR_PENDSTSET_WIDTH             1u
#define S32_SCB_ICSR_PENDSTSET(x)                (((uint32_t)(((uint32_t)(x))<<S32_SCB_ICSR_PENDSTSET_SHIFT))&S32_SCB_ICSR_PENDSTSET_MASK)
#define S32_SCB_ICSR_PENDSVCLR_MASK              0x8000000u
#define S32_SCB_ICSR_PENDSVCLR_SHIFT             27u
#define S32_SCB_ICSR_PENDSVCLR_WIDTH             1u
#define S32_SCB_ICSR_PENDSVCLR(x)                (((uint32_t)(((uint32_t)(x))<<S32_SCB_ICSR_PENDSVCLR_SHIFT))&S32_SCB_ICSR_PENDSVCLR_MASK)
#define S32_SCB_ICSR_PENDSVSET_MASK              0x10000000u
#define S32_SCB_ICSR_PENDSVSET_SHIFT             28u
#define S32_SCB_ICSR_PENDSVSET_WIDTH             1u
#define S32_SCB_ICSR_PENDSVSET(x)                (((uint32_t)(((uint32_t)(x))<<S32_SCB_ICSR_PENDSVSET_SHIFT))&S32_SCB_ICSR_PENDSVSET_MASK)
#define S32_SCB_ICSR_NMIPENDSET_MASK             0x80000000u
#define S32_SCB_ICSR_NMIPENDSET_SHIFT            31u
#define S32_SCB_ICSR_NMIPENDSET_WIDTH            1u
#define S32_SCB_ICSR_NMIPENDSET(x)               (((uint32_t)(((uint32_t)(x))<<S32_SCB_ICSR_NMIPENDSET_SHIFT))&S32_SCB_ICSR_NMIPENDSET_MASK)
/* VTOR Bit Fields */
#define S32_SCB_VTOR_TBLOFF_MASK                 0xFFFFFF80u
#define S32_SCB_VTOR_TBLOFF_SHIFT                7u
#define S32_SCB_VTOR_TBLOFF_WIDTH                25u
#define S32_SCB_VTOR_TBLOFF(x)                   (((uint32_t)(((uint32_t)(x))<<S32_SCB_VTOR_TBLOFF_SHIFT))&S32_SCB_VTOR_TBLOFF_MASK)
/* AIRCR Bit Fields */
#define S32_SCB_AIRCR_VECTRESET_MASK             0x1u
#define S32_SCB_AIRCR_VECTRESET_SHIFT            0u
#define S32_SCB_AIRCR_VECTRESET_WIDTH            1u
#define S32_SCB_AIRCR_VECTRESET(x)               (((uint32_t)(((uint32_t)(x))<<S32_SCB_AIRCR_VECTRESET_SHIFT))&S32_SCB_AIRCR_VECTRESET_MASK)
#define S32_SCB_AIRCR_VECTCLRACTIVE_MASK         0x2u
#define S32_SCB_AIRCR_VECTCLRACTIVE_SHIFT        1u
#define S32_SCB_AIRCR_VECTCLRACTIVE_WIDTH        1u
#define S32_SCB_AIRCR_VECTCLRACTIVE(x)           (((uint32_t)(((uint32_t)(x))<<S32_SCB_AIRCR_VECTCLRACTIVE_SHIFT))&S32_SCB_AIRCR_VECTCLRACTIVE_MASK)
#define S32_SCB_AIRCR_SYSRESETREQ_MASK           0x4u
#define S32_SCB_AIRCR_SYSRESETREQ_SHIFT          2u
#define S32_SCB_AIRCR_SYSRESETREQ_WIDTH          1u
#define S32_SCB_AIRCR_SYSRESETREQ(x)             (((uint32_t)(((uint32_t)(x))<<S32_SCB_AIRCR_SYSRESETREQ_SHIFT))&S32_SCB_AIRCR_SYSRESETREQ_MASK)
#define S32_SCB_AIRCR_PRIGROUP_MASK              0x700u
#define S32_SCB_AIRCR_PRIGROUP_SHIFT             8u
#define S32_SCB_AIRCR_PRIGROUP_WIDTH             3u
#define S32_SCB_AIRCR_PRIGROUP(x)                (((uint32_t)(((uint32_t)(x))<<S32_SCB_AIRCR_PRIGROUP_SHIFT))&S32_SCB_AIRCR_PRIGROUP_MASK)
#define S32_SCB_AIRCR_ENDIANNESS_MASK            0x8000u
#define S32_SCB_AIRCR_ENDIANNESS_SHIFT           15u
#define S32_SCB_AIRCR_ENDIANNESS_WIDTH           1u
#define S32_SCB_AIRCR_ENDIANNESS(x)              (((uint32_t)(((uint32_t)(x))<<S32_SCB_AIRCR_ENDIANNESS_SHIFT))&S32_SCB_AIRCR_ENDIANNESS_MASK)
#define S32_SCB_AIRCR_VECTKEY_MASK               0xFFFF0000u
#define S32_SCB_AIRCR_VECTKEY_SHIFT              16u
#define S32_SCB_AIRCR_VECTKEY_WIDTH              16u
#define S32_SCB_AIRCR_VECTKEY(x)                 (((uint32_t)(((uint32_t)(x))<<S32_SCB_AIRCR_VECTKEY_SHIFT))&S32_SCB_AIRCR_VECTKEY_MASK)
/* SCR Bit Fields */
#define S32_SCB_SCR_SLEEPONEXIT_MASK             0x2u
#define S32_SCB_SCR_SLEEPONEXIT_SHIFT            1u
#define S32_SCB_SCR_SLEEPONEXIT_WIDTH            1u
#define S32_SCB_SCR_SLEEPONEXIT(x)               (((uint32_t)(((uint32_t)(x))<<S32_SCB_SCR_SLEEPONEXIT_SHIFT))&S32_SCB_SCR_SLEEPONEXIT_MASK)
#define S32_SCB_SCR_SLEEPDEEP_MASK               0x4u
#define S32_SCB_SCR_SLEEPDEEP_SHIFT              2u
#define S32_SCB_SCR_SLEEPDEEP_WIDTH              1u
#define S32_SCB_SCR_SLEEPDEEP(x)                 (((uint32_t)(((uint32_t)(x))<<S32_SCB_SCR_SLEEPDEEP_SHIFT))&S32_SCB_SCR_SLEEPDEEP_MASK)
#define S32_SCB_SCR_SEVONPEND_MASK               0x10u
#define S32_SCB_SCR_SEVONPEND_SHIFT              4u
#define S32_SCB_SCR_SEVONPEND_WIDTH              1u
#define S32_SCB_SCR_SEVONPEND(x)                 (((uint32_t)(((uint32_t)(x))<<S32_SCB_SCR_SEVONPEND_SHIFT))&S32_SCB_SCR_SEVONPEND_MASK)
/* CCR Bit Fields */
#define S32_SCB_CCR_NONBASETHRDENA_MASK          0x1u
#define S32_SCB_CCR_NONBASETHRDENA_SHIFT         0u
#define S32_SCB_CCR_NONBASETHRDENA_WIDTH         1u
#define S32_SCB_CCR_NONBASETHRDENA(x)            (((uint32_t)(((uint32_t)(x))<<S32_SCB_CCR_NONBASETHRDENA_SHIFT))&S32_SCB_CCR_NONBASETHRDENA_MASK)
#define S32_SCB_CCR_USERSETMPEND_MASK            0x2u
#define S32_SCB_CCR_USERSETMPEND_SHIFT           1u
#define S32_SCB_CCR_USERSETMPEND_WIDTH           1u
#define S32_SCB_CCR_USERSETMPEND(x)              (((uint32_t)(((uint32_t)(x))<<S32_SCB_CCR_USERSETMPEND_SHIFT))&S32_SCB_CCR_USERSETMPEND_MASK)
#define S32_SCB_CCR_UNALIGN_TRP_MASK             0x8u
#define S32_SCB_CCR_UNALIGN_TRP_SHIFT            3u
#define S32_SCB_CCR_UNALIGN_TRP_WIDTH            1u
#define S32_SCB_CCR_UNALIGN_TRP(x)               (((uint32_t)(((uint32_t)(x))<<S32_SCB_CCR_UNALIGN_TRP_SHIFT))&S32_SCB_CCR_UNALIGN_TRP_MASK)
#define S32_SCB_CCR_DIV_0_TRP_MASK               0x10u
#define S32_SCB_CCR_DIV_0_TRP_SHIFT              4u
#define S32_SCB_CCR_DIV_0_TRP_WIDTH              1u
#define S32_SCB_CCR_DIV_0_TRP(x)                 (((uint32_t)(((uint32_t)(x))<<S32_SCB_CCR_DIV_0_TRP_SHIFT))&S32_SCB_CCR_DIV_0_TRP_MASK)
#define S32_SCB_CCR_BFHFNMIGN_MASK               0x100u
#define S32_SCB_CCR_BFHFNMIGN_SHIFT              8u
#define S32_SCB_CCR_BFHFNMIGN_WIDTH              1u
#define S32_SCB_CCR_BFHFNMIGN(x)                 (((uint32_t)(((uint32_t)(x))<<S32_SCB_CCR_BFHFNMIGN_SHIFT))&S32_SCB_CCR_BFHFNMIGN_MASK)
#define S32_SCB_CCR_STKALIGN_MASK                0x200u
#define S32_SCB_CCR_STKALIGN_SHIFT               9u
#define S32_SCB_CCR_STKALIGN_WIDTH               1u
#define S32_SCB_CCR_STKALIGN(x)                  (((uint32_t)(((uint32_t)(x))<<S32_SCB_CCR_STKALIGN_SHIFT))&S32_SCB_CCR_STKALIGN_MASK)
/* SHPR1 Bit Fields */
#define S32_SCB_SHPR1_PRI_4_MASK                 0xFFu
#define S32_SCB_SHPR1_PRI_4_SHIFT                0u
#define S32_SCB_SHPR1_PRI_4_WIDTH                8u
#define S32_SCB_SHPR1_PRI_4(x)                   (((uint32_t)(((uint32_t)(x))<<S32_SCB_SHPR1_PRI_4_SHIFT))&S32_SCB_SHPR1_PRI_4_MASK)
#define S32_SCB_SHPR1_PRI_5_MASK                 0xFF00u
#define S32_SCB_SHPR1_PRI_5_SHIFT                8u
#define S32_SCB_SHPR1_PRI_5_WIDTH                8u
#define S32_SCB_SHPR1_PRI_5(x)                   (((uint32_t)(((uint32_t)(x))<<S32_SCB_SHPR1_PRI_5_SHIFT))&S32_SCB_SHPR1_PRI_5_MASK)
#define S32_SCB_SHPR1_PRI_6_MASK                 0xFF0000u
#define S32_SCB_SHPR1_PRI_6_SHIFT                16u
#define S32_SCB_SHPR1_PRI_6_WIDTH                8u
#define S32_SCB_SHPR1_PRI_6(x)                   (((uint32_t)(((uint32_t)(x))<<S32_SCB_SHPR1_PRI_6_SHIFT))&S32_SCB_SHPR1_PRI_6_MASK)
/* SHPR2 Bit Fields */
#define S32_SCB_SHPR2_PRI_11_MASK                0xFF000000u
#define S32_SCB_SHPR2_PRI_11_SHIFT               24u
#define S32_SCB_SHPR2_PRI_11_WIDTH               8u
#define S32_SCB_SHPR2_PRI_11(x)                  (((uint32_t)(((uint32_t)(x))<<S32_SCB_SHPR2_PRI_11_SHIFT))&S32_SCB_SHPR2_PRI_11_MASK)
/* SHPR3 Bit Fields */
#define S32_SCB_SHPR3_PRI_12_MASK                0xFFu
#define S32_SCB_SHPR3_PRI_12_SHIFT               0u
#define S32_SCB_SHPR3_PRI_12_WIDTH               8u
#define S32_SCB_SHPR3_PRI_12(x)                  (((uint32_t)(((uint32_t)(x))<<S32_SCB_SHPR3_PRI_12_SHIFT))&S32_SCB_SHPR3_PRI_12_MASK)
#define S32_SCB_SHPR3_PRI_14_MASK                0xFF0000u
#define S32_SCB_SHPR3_PRI_14_SHIFT               16u
#define S32_SCB_SHPR3_PRI_14_WIDTH               8u
#define S32_SCB_SHPR3_PRI_14(x)                  (((uint32_t)(((uint32_t)(x))<<S32_SCB_SHPR3_PRI_14_SHIFT))&S32_SCB_SHPR3_PRI_14_MASK)
#define S32_SCB_SHPR3_PRI_15_MASK                0xFF000000u
#define S32_SCB_SHPR3_PRI_15_SHIFT               24u
#define S32_SCB_SHPR3_PRI_15_WIDTH               8u
#define S32_SCB_SHPR3_PRI_15(x)                  (((uint32_t)(((uint32_t)(x))<<S32_SCB_SHPR3_PRI_15_SHIFT))&S32_SCB_SHPR3_PRI_15_MASK)
/* SHCSR Bit Fields */
#define S32_SCB_SHCSR_MEMFAULTACT_MASK           0x1u
#define S32_SCB_SHCSR_MEMFAULTACT_SHIFT          0u
#define S32_SCB_SHCSR_MEMFAULTACT_WIDTH          1u
#define S32_SCB_SHCSR_MEMFAULTACT(x)             (((uint32_t)(((uint32_t)(x))<<S32_SCB_SHCSR_MEMFAULTACT_SHIFT))&S32_SCB_SHCSR_MEMFAULTACT_MASK)
#define S32_SCB_SHCSR_BUSFAULTACT_MASK           0x2u
#define S32_SCB_SHCSR_BUSFAULTACT_SHIFT          1u
#define S32_SCB_SHCSR_BUSFAULTACT_WIDTH          1u
#define S32_SCB_SHCSR_BUSFAULTACT(x)             (((uint32_t)(((uint32_t)(x))<<S32_SCB_SHCSR_BUSFAULTACT_SHIFT))&S32_SCB_SHCSR_BUSFAULTACT_MASK)
#define S32_SCB_SHCSR_USGFAULTACT_MASK           0x8u
#define S32_SCB_SHCSR_USGFAULTACT_SHIFT          3u
#define S32_SCB_SHCSR_USGFAULTACT_WIDTH          1u
#define S32_SCB_SHCSR_USGFAULTACT(x)             (((uint32_t)(((uint32_t)(x))<<S32_SCB_SHCSR_USGFAULTACT_SHIFT))&S32_SCB_SHCSR_USGFAULTACT_MASK)
#define S32_SCB_SHCSR_SVCALLACT_MASK             0x80u
#define S32_SCB_SHCSR_SVCALLACT_SHIFT            7u
#define S32_SCB_SHCSR_SVCALLACT_WIDTH            1u
#define S32_SCB_SHCSR_SVCALLACT(x)               (((uint32_t)(((uint32_t)(x))<<S32_SCB_SHCSR_SVCALLACT_SHIFT))&S32_SCB_SHCSR_SVCALLACT_MASK)
#define S32_SCB_SHCSR_MONITORACT_MASK            0x100u
#define S32_SCB_SHCSR_MONITORACT_SHIFT           8u
#define S32_SCB_SHCSR_MONITORACT_WIDTH           1u
#define S32_SCB_SHCSR_MONITORACT(x)              (((uint32_t)(((uint32_t)(x))<<S32_SCB_SHCSR_MONITORACT_SHIFT))&S32_SCB_SHCSR_MONITORACT_MASK)
#define S32_SCB_SHCSR_PENDSVACT_MASK             0x400u
#define S32_SCB_SHCSR_PENDSVACT_SHIFT            10u
#define S32_SCB_SHCSR_PENDSVACT_WIDTH            1u
#define S32_SCB_SHCSR_PENDSVACT(x)               (((uint32_t)(((uint32_t)(x))<<S32_SCB_SHCSR_PENDSVACT_SHIFT))&S32_SCB_SHCSR_PENDSVACT_MASK)
#define S32_SCB_SHCSR_SYSTICKACT_MASK            0x800u
#define S32_SCB_SHCSR_SYSTICKACT_SHIFT           11u
#define S32_SCB_SHCSR_SYSTICKACT_WIDTH           1u
#define S32_SCB_SHCSR_SYSTICKACT(x)              (((uint32_t)(((uint32_t)(x))<<S32_SCB_SHCSR_SYSTICKACT_SHIFT))&S32_SCB_SHCSR_SYSTICKACT_MASK)
#define S32_SCB_SHCSR_USGFAULTPENDED_MASK        0x1000u
#define S32_SCB_SHCSR_USGFAULTPENDED_SHIFT       12u
#define S32_SCB_SHCSR_USGFAULTPENDED_WIDTH       1u
#define S32_SCB_SHCSR_USGFAULTPENDED(x)          (((uint32_t)(((uint32_t)(x))<<S32_SCB_SHCSR_USGFAULTPENDED_SHIFT))&S32_SCB_SHCSR_USGFAULTPENDED_MASK)
#define S32_SCB_SHCSR_MEMFAULTPENDED_MASK        0x2000u
#define S32_SCB_SHCSR_MEMFAULTPENDED_SHIFT       13u
#define S32_SCB_SHCSR_MEMFAULTPENDED_WIDTH       1u
#define S32_SCB_SHCSR_MEMFAULTPENDED(x)          (((uint32_t)(((uint32_t)(x))<<S32_SCB_SHCSR_MEMFAULTPENDED_SHIFT))&S32_SCB_SHCSR_MEMFAULTPENDED_MASK)
#define S32_SCB_SHCSR_BUSFAULTPENDED_MASK        0x4000u
#define S32_SCB_SHCSR_BUSFAULTPENDED_SHIFT       14u
#define S32_SCB_SHCSR_BUSFAULTPENDED_WIDTH       1u
#define S32_SCB_SHCSR_BUSFAULTPENDED(x)          (((uint32_t)(((uint32_t)(x))<<S32_SCB_SHCSR_BUSFAULTPENDED_SHIFT))&S32_SCB_SHCSR_BUSFAULTPENDED_MASK)
#define S32_SCB_SHCSR_SVCALLPENDED_MASK          0x8000u
#define S32_SCB_SHCSR_SVCALLPENDED_SHIFT         15u
#define S32_SCB_SHCSR_SVCALLPENDED_WIDTH         1u
#define S32_SCB_SHCSR_SVCALLPENDED(x)            (((uint32_t)(((uint32_t)(x))<<S32_SCB_SHCSR_SVCALLPENDED_SHIFT))&S32_SCB_SHCSR_SVCALLPENDED_MASK)
#define S32_SCB_SHCSR_MEMFAULTENA_MASK           0x10000u
#define S32_SCB_SHCSR_MEMFAULTENA_SHIFT          16u
#define S32_SCB_SHCSR_MEMFAULTENA_WIDTH          1u
#define S32_SCB_SHCSR_MEMFAULTENA(x)             (((uint32_t)(((uint32_t)(x))<<S32_SCB_SHCSR_MEMFAULTENA_SHIFT))&S32_SCB_SHCSR_MEMFAULTENA_MASK)
#define S32_SCB_SHCSR_BUSFAULTENA_MASK           0x20000u
#define S32_SCB_SHCSR_BUSFAULTENA_SHIFT          17u
#define S32_SCB_SHCSR_BUSFAULTENA_WIDTH          1u
#define S32_SCB_SHCSR_BUSFAULTENA(x)             (((uint32_t)(((uint32_t)(x))<<S32_SCB_SHCSR_BUSFAULTENA_SHIFT))&S32_SCB_SHCSR_BUSFAULTENA_MASK)
#define S32_SCB_SHCSR_USGFAULTENA_MASK           0x40000u
#define S32_SCB_SHCSR_USGFAULTENA_SHIFT          18u
#define S32_SCB_SHCSR_USGFAULTENA_WIDTH          1u
#define S32_SCB_SHCSR_USGFAULTENA(x)             (((uint32_t)(((uint32_t)(x))<<S32_SCB_SHCSR_USGFAULTENA_SHIFT))&S32_SCB_SHCSR_USGFAULTENA_MASK)
/* CFSR Bit Fields */
#define S32_SCB_CFSR_IACCVIOL_MASK               0x1u
#define S32_SCB_CFSR_IACCVIOL_SHIFT              0u
#define S32_SCB_CFSR_IACCVIOL_WIDTH              1u
#define S32_SCB_CFSR_IACCVIOL(x)                 (((uint32_t)(((uint32_t)(x))<<S32_SCB_CFSR_IACCVIOL_SHIFT))&S32_SCB_CFSR_IACCVIOL_MASK)
#define S32_SCB_CFSR_DACCVIOL_MASK               0x2u
#define S32_SCB_CFSR_DACCVIOL_SHIFT              1u
#define S32_SCB_CFSR_DACCVIOL_WIDTH              1u
#define S32_SCB_CFSR_DACCVIOL(x)                 (((uint32_t)(((uint32_t)(x))<<S32_SCB_CFSR_DACCVIOL_SHIFT))&S32_SCB_CFSR_DACCVIOL_MASK)
#define S32_SCB_CFSR_MUNSTKERR_MASK              0x8u
#define S32_SCB_CFSR_MUNSTKERR_SHIFT             3u
#define S32_SCB_CFSR_MUNSTKERR_WIDTH             1u
#define S32_SCB_CFSR_MUNSTKERR(x)                (((uint32_t)(((uint32_t)(x))<<S32_SCB_CFSR_MUNSTKERR_SHIFT))&S32_SCB_CFSR_MUNSTKERR_MASK)
#define S32_SCB_CFSR_MSTKERR_MASK                0x10u
#define S32_SCB_CFSR_MSTKERR_SHIFT               4u
#define S32_SCB_CFSR_MSTKERR_WIDTH               1u
#define S32_SCB_CFSR_MSTKERR(x)                  (((uint32_t)(((uint32_t)(x))<<S32_SCB_CFSR_MSTKERR_SHIFT))&S32_SCB_CFSR_MSTKERR_MASK)
#define S32_SCB_CFSR_MLSPERR_MASK                0x20u
#define S32_SCB_CFSR_MLSPERR_SHIFT               5u
#define S32_SCB_CFSR_MLSPERR_WIDTH               1u
#define S32_SCB_CFSR_MLSPERR(x)                  (((uint32_t)(((uint32_t)(x))<<S32_SCB_CFSR_MLSPERR_SHIFT))&S32_SCB_CFSR_MLSPERR_MASK)
#define S32_SCB_CFSR_MMARVALID_MASK              0x80u
#define S32_SCB_CFSR_MMARVALID_SHIFT             7u
#define S32_SCB_CFSR_MMARVALID_WIDTH             1u
#define S32_SCB_CFSR_MMARVALID(x)                (((uint32_t)(((uint32_t)(x))<<S32_SCB_CFSR_MMARVALID_SHIFT))&S32_SCB_CFSR_MMARVALID_MASK)
#define S32_SCB_CFSR_IBUSERR_MASK                0x100u
#define S32_SCB_CFSR_IBUSERR_SHIFT               8u
#define S32_SCB_CFSR_IBUSERR_WIDTH               1u
#define S32_SCB_CFSR_IBUSERR(x)                  (((uint32_t)(((uint32_t)(x))<<S32_SCB_CFSR_IBUSERR_SHIFT))&S32_SCB_CFSR_IBUSERR_MASK)
#define S32_SCB_CFSR_PRECISERR_MASK              0x200u
#define S32_SCB_CFSR_PRECISERR_SHIFT             9u
#define S32_SCB_CFSR_PRECISERR_WIDTH             1u
#define S32_SCB_CFSR_PRECISERR(x)                (((uint32_t)(((uint32_t)(x))<<S32_SCB_CFSR_PRECISERR_SHIFT))&S32_SCB_CFSR_PRECISERR_MASK)
#define S32_SCB_CFSR_IMPRECISERR_MASK            0x400u
#define S32_SCB_CFSR_IMPRECISERR_SHIFT           10u
#define S32_SCB_CFSR_IMPRECISERR_WIDTH           1u
#define S32_SCB_CFSR_IMPRECISERR(x)              (((uint32_t)(((uint32_t)(x))<<S32_SCB_CFSR_IMPRECISERR_SHIFT))&S32_SCB_CFSR_IMPRECISERR_MASK)
#define S32_SCB_CFSR_UNSTKERR_MASK               0x800u
#define S32_SCB_CFSR_UNSTKERR_SHIFT              11u
#define S32_SCB_CFSR_UNSTKERR_WIDTH              1u
#define S32_SCB_CFSR_UNSTKERR(x)                 (((uint32_t)(((uint32_t)(x))<<S32_SCB_CFSR_UNSTKERR_SHIFT))&S32_SCB_CFSR_UNSTKERR_MASK)
#define S32_SCB_CFSR_STKERR_MASK                 0x1000u
#define S32_SCB_CFSR_STKERR_SHIFT                12u
#define S32_SCB_CFSR_STKERR_WIDTH                1u
#define S32_SCB_CFSR_STKERR(x)                   (((uint32_t)(((uint32_t)(x))<<S32_SCB_CFSR_STKERR_SHIFT))&S32_SCB_CFSR_STKERR_MASK)
#define S32_SCB_CFSR_LSPERR_MASK                 0x2000u
#define S32_SCB_CFSR_LSPERR_SHIFT                13u
#define S32_SCB_CFSR_LSPERR_WIDTH                1u
#define S32_SCB_CFSR_LSPERR(x)                   (((uint32_t)(((uint32_t)(x))<<S32_SCB_CFSR_LSPERR_SHIFT))&S32_SCB_CFSR_LSPERR_MASK)
#define S32_SCB_CFSR_BFARVALID_MASK              0x8000u
#define S32_SCB_CFSR_BFARVALID_SHIFT             15u
#define S32_SCB_CFSR_BFARVALID_WIDTH             1u
#define S32_SCB_CFSR_BFARVALID(x)                (((uint32_t)(((uint32_t)(x))<<S32_SCB_CFSR_BFARVALID_SHIFT))&S32_SCB_CFSR_BFARVALID_MASK)
#define S32_SCB_CFSR_UNDEFINSTR_MASK             0x10000u
#define S32_SCB_CFSR_UNDEFINSTR_SHIFT            16u
#define S32_SCB_CFSR_UNDEFINSTR_WIDTH            1u
#define S32_SCB_CFSR_UNDEFINSTR(x)               (((uint32_t)(((uint32_t)(x))<<S32_SCB_CFSR_UNDEFINSTR_SHIFT))&S32_SCB_CFSR_UNDEFINSTR_MASK)
#define S32_SCB_CFSR_INVSTATE_MASK               0x20000u
#define S32_SCB_CFSR_INVSTATE_SHIFT              17u
#define S32_SCB_CFSR_INVSTATE_WIDTH              1u
#define S32_SCB_CFSR_INVSTATE(x)                 (((uint32_t)(((uint32_t)(x))<<S32_SCB_CFSR_INVSTATE_SHIFT))&S32_SCB_CFSR_INVSTATE_MASK)
#define S32_SCB_CFSR_INVPC_MASK                  0x40000u
#define S32_SCB_CFSR_INVPC_SHIFT                 18u
#define S32_SCB_CFSR_INVPC_WIDTH                 1u
#define S32_SCB_CFSR_INVPC(x)                    (((uint32_t)(((uint32_t)(x))<<S32_SCB_CFSR_INVPC_SHIFT))&S32_SCB_CFSR_INVPC_MASK)
#define S32_SCB_CFSR_NOCP_MASK                   0x80000u
#define S32_SCB_CFSR_NOCP_SHIFT                  19u
#define S32_SCB_CFSR_NOCP_WIDTH                  1u
#define S32_SCB_CFSR_NOCP(x)                     (((uint32_t)(((uint32_t)(x))<<S32_SCB_CFSR_NOCP_SHIFT))&S32_SCB_CFSR_NOCP_MASK)
#define S32_SCB_CFSR_UNALIGNED_MASK              0x1000000u
#define S32_SCB_CFSR_UNALIGNED_SHIFT             24u
#define S32_SCB_CFSR_UNALIGNED_WIDTH             1u
#define S32_SCB_CFSR_UNALIGNED(x)                (((uint32_t)(((uint32_t)(x))<<S32_SCB_CFSR_UNALIGNED_SHIFT))&S32_SCB_CFSR_UNALIGNED_MASK)
#define S32_SCB_CFSR_DIVBYZERO_MASK              0x2000000u
#define S32_SCB_CFSR_DIVBYZERO_SHIFT             25u
#define S32_SCB_CFSR_DIVBYZERO_WIDTH             1u
#define S32_SCB_CFSR_DIVBYZERO(x)                (((uint32_t)(((uint32_t)(x))<<S32_SCB_CFSR_DIVBYZERO_SHIFT))&S32_SCB_CFSR_DIVBYZERO_MASK)
/* HFSR Bit Fields */
#define S32_SCB_HFSR_VECTTBL_MASK                0x2u
#define S32_SCB_HFSR_VECTTBL_SHIFT               1u
#define S32_SCB_HFSR_VECTTBL_WIDTH               1u
#define S32_SCB_HFSR_VECTTBL(x)                  (((uint32_t)(((uint32_t)(x))<<S32_SCB_HFSR_VECTTBL_SHIFT))&S32_SCB_HFSR_VECTTBL_MASK)
#define S32_SCB_HFSR_FORCED_MASK                 0x40000000u
#define S32_SCB_HFSR_FORCED_SHIFT                30u
#define S32_SCB_HFSR_FORCED_WIDTH                1u
#define S32_SCB_HFSR_FORCED(x)                   (((uint32_t)(((uint32_t)(x))<<S32_SCB_HFSR_FORCED_SHIFT))&S32_SCB_HFSR_FORCED_MASK)
#define S32_SCB_HFSR_DEBUGEVT_MASK               0x80000000u
#define S32_SCB_HFSR_DEBUGEVT_SHIFT              31u
#define S32_SCB_HFSR_DEBUGEVT_WIDTH              1u
#define S32_SCB_HFSR_DEBUGEVT(x)                 (((uint32_t)(((uint32_t)(x))<<S32_SCB_HFSR_DEBUGEVT_SHIFT))&S32_SCB_HFSR_DEBUGEVT_MASK)
/* DFSR Bit Fields */
#define S32_SCB_DFSR_HALTED_MASK                 0x1u
#define S32_SCB_DFSR_HALTED_SHIFT                0u
#define S32_SCB_DFSR_HALTED_WIDTH                1u
#define S32_SCB_DFSR_HALTED(x)                   (((uint32_t)(((uint32_t)(x))<<S32_SCB_DFSR_HALTED_SHIFT))&S32_SCB_DFSR_HALTED_MASK)
#define S32_SCB_DFSR_BKPT_MASK                   0x2u
#define S32_SCB_DFSR_BKPT_SHIFT                  1u
#define S32_SCB_DFSR_BKPT_WIDTH                  1u
#define S32_SCB_DFSR_BKPT(x)                     (((uint32_t)(((uint32_t)(x))<<S32_SCB_DFSR_BKPT_SHIFT))&S32_SCB_DFSR_BKPT_MASK)
#define S32_SCB_DFSR_DWTTRAP_MASK                0x4u
#define S32_SCB_DFSR_DWTTRAP_SHIFT               2u
#define S32_SCB_DFSR_DWTTRAP_WIDTH               1u
#define S32_SCB_DFSR_DWTTRAP(x)                  (((uint32_t)(((uint32_t)(x))<<S32_SCB_DFSR_DWTTRAP_SHIFT))&S32_SCB_DFSR_DWTTRAP_MASK)
#define S32_SCB_DFSR_VCATCH_MASK                 0x8u
#define S32_SCB_DFSR_VCATCH_SHIFT                3u
#define S32_SCB_DFSR_VCATCH_WIDTH                1u
#define S32_SCB_DFSR_VCATCH(x)                   (((uint32_t)(((uint32_t)(x))<<S32_SCB_DFSR_VCATCH_SHIFT))&S32_SCB_DFSR_VCATCH_MASK)
#define S32_SCB_DFSR_EXTERNAL_MASK               0x10u
#define S32_SCB_DFSR_EXTERNAL_SHIFT              4u
#define S32_SCB_DFSR_EXTERNAL_WIDTH              1u
#define S32_SCB_DFSR_EXTERNAL(x)                 (((uint32_t)(((uint32_t)(x))<<S32_SCB_DFSR_EXTERNAL_SHIFT))&S32_SCB_DFSR_EXTERNAL_MASK)
/* MMFAR Bit Fields */
#define S32_SCB_MMFAR_ADDRESS_MASK               0xFFFFFFFFu
#define S32_SCB_MMFAR_ADDRESS_SHIFT              0u
#define S32_SCB_MMFAR_ADDRESS_WIDTH              32u
#define S32_SCB_MMFAR_ADDRESS(x)                 (((uint32_t)(((uint32_t)(x))<<S32_SCB_MMFAR_ADDRESS_SHIFT))&S32_SCB_MMFAR_ADDRESS_MASK)
/* BFAR Bit Fields */
#define S32_SCB_BFAR_ADDRESS_MASK                0xFFFFFFFFu
#define S32_SCB_BFAR_ADDRESS_SHIFT               0u
#define S32_SCB_BFAR_ADDRESS_WIDTH               32u
#define S32_SCB_BFAR_ADDRESS(x)                  (((uint32_t)(((uint32_t)(x))<<S32_SCB_BFAR_ADDRESS_SHIFT))&S32_SCB_BFAR_ADDRESS_MASK)
/* AFSR Bit Fields */
#define S32_SCB_AFSR_AUXFAULT_MASK               0xFFFFFFFFu
#define S32_SCB_AFSR_AUXFAULT_SHIFT              0u
#define S32_SCB_AFSR_AUXFAULT_WIDTH              32u
#define S32_SCB_AFSR_AUXFAULT(x)                 (((uint32_t)(((uint32_t)(x))<<S32_SCB_AFSR_AUXFAULT_SHIFT))&S32_SCB_AFSR_AUXFAULT_MASK)
/* CPACR Bit Fields */
#define S32_SCB_CPACR_CP10_MASK                  0x300000u
#define S32_SCB_CPACR_CP10_SHIFT                 20u
#define S32_SCB_CPACR_CP10_WIDTH                 2u
#define S32_SCB_CPACR_CP10(x)                    (((uint32_t)(((uint32_t)(x))<<S32_SCB_CPACR_CP10_SHIFT))&S32_SCB_CPACR_CP10_MASK)
#define S32_SCB_CPACR_CP11_MASK                  0xC00000u
#define S32_SCB_CPACR_CP11_SHIFT                 22u
#define S32_SCB_CPACR_CP11_WIDTH                 2u
#define S32_SCB_CPACR_CP11(x)                    (((uint32_t)(((uint32_t)(x))<<S32_SCB_CPACR_CP11_SHIFT))&S32_SCB_CPACR_CP11_MASK)
/* FPCCR Bit Fields */
#define S32_SCB_FPCCR_LSPACT_MASK                0x1u
#define S32_SCB_FPCCR_LSPACT_SHIFT               0u
#define S32_SCB_FPCCR_LSPACT_WIDTH               1u
#define S32_SCB_FPCCR_LSPACT(x)                  (((uint32_t)(((uint32_t)(x))<<S32_SCB_FPCCR_LSPACT_SHIFT))&S32_SCB_FPCCR_LSPACT_MASK)
#define S32_SCB_FPCCR_USER_MASK                  0x2u
#define S32_SCB_FPCCR_USER_SHIFT                 1u
#define S32_SCB_FPCCR_USER_WIDTH                 1u
#define S32_SCB_FPCCR_USER(x)                    (((uint32_t)(((uint32_t)(x))<<S32_SCB_FPCCR_USER_SHIFT))&S32_SCB_FPCCR_USER_MASK)
#define S32_SCB_FPCCR_THREAD_MASK                0x8u
#define S32_SCB_FPCCR_THREAD_SHIFT               3u
#define S32_SCB_FPCCR_THREAD_WIDTH               1u
#define S32_SCB_FPCCR_THREAD(x)                  (((uint32_t)(((uint32_t)(x))<<S32_SCB_FPCCR_THREAD_SHIFT))&S32_SCB_FPCCR_THREAD_MASK)
#define S32_SCB_FPCCR_HFRDY_MASK                 0x10u
#define S32_SCB_FPCCR_HFRDY_SHIFT                4u
#define S32_SCB_FPCCR_HFRDY_WIDTH                1u
#define S32_SCB_FPCCR_HFRDY(x)                   (((uint32_t)(((uint32_t)(x))<<S32_SCB_FPCCR_HFRDY_SHIFT))&S32_SCB_FPCCR_HFRDY_MASK)
#define S32_SCB_FPCCR_MMRDY_MASK                 0x20u
#define S32_SCB_FPCCR_MMRDY_SHIFT                5u
#define S32_SCB_FPCCR_MMRDY_WIDTH                1u
#define S32_SCB_FPCCR_MMRDY(x)                   (((uint32_t)(((uint32_t)(x))<<S32_SCB_FPCCR_MMRDY_SHIFT))&S32_SCB_FPCCR_MMRDY_MASK)
#define S32_SCB_FPCCR_BFRDY_MASK                 0x40u
#define S32_SCB_FPCCR_BFRDY_SHIFT                6u
#define S32_SCB_FPCCR_BFRDY_WIDTH                1u
#define S32_SCB_FPCCR_BFRDY(x)                   (((uint32_t)(((uint32_t)(x))<<S32_SCB_FPCCR_BFRDY_SHIFT))&S32_SCB_FPCCR_BFRDY_MASK)
#define S32_SCB_FPCCR_MONRDY_MASK                0x100u
#define S32_SCB_FPCCR_MONRDY_SHIFT               8u
#define S32_SCB_FPCCR_MONRDY_WIDTH               1u
#define S32_SCB_FPCCR_MONRDY(x)                  (((uint32_t)(((uint32_t)(x))<<S32_SCB_FPCCR_MONRDY_SHIFT))&S32_SCB_FPCCR_MONRDY_MASK)
#define S32_SCB_FPCCR_LSPEN_MASK                 0x40000000u
#define S32_SCB_FPCCR_LSPEN_SHIFT                30u
#define S32_SCB_FPCCR_LSPEN_WIDTH                1u
#define S32_SCB_FPCCR_LSPEN(x)                   (((uint32_t)(((uint32_t)(x))<<S32_SCB_FPCCR_LSPEN_SHIFT))&S32_SCB_FPCCR_LSPEN_MASK)
#define S32_SCB_FPCCR_ASPEN_MASK                 0x80000000u
#define S32_SCB_FPCCR_ASPEN_SHIFT                31u
#define S32_SCB_FPCCR_ASPEN_WIDTH                1u
#define S32_SCB_FPCCR_ASPEN(x)                   (((uint32_t)(((uint32_t)(x))<<S32_SCB_FPCCR_ASPEN_SHIFT))&S32_SCB_FPCCR_ASPEN_MASK)
/* FPCAR Bit Fields */
#define S32_SCB_FPCAR_ADDRESS_MASK               0xFFFFFFF8u
#define S32_SCB_FPCAR_ADDRESS_SHIFT              3u
#define S32_SCB_FPCAR_ADDRESS_WIDTH              29u
#define S32_SCB_FPCAR_ADDRESS(x)                 (((uint32_t)(((uint32_t)(x))<<S32_SCB_FPCAR_ADDRESS_SHIFT))&S32_SCB_FPCAR_ADDRESS_MASK)
/* FPDSCR Bit Fields */
#define S32_SCB_FPDSCR_RMode_MASK                0xC00000u
#define S32_SCB_FPDSCR_RMode_SHIFT               22u
#define S32_SCB_FPDSCR_RMode_WIDTH               2u
#define S32_SCB_FPDSCR_RMode(x)                  (((uint32_t)(((uint32_t)(x))<<S32_SCB_FPDSCR_RMode_SHIFT))&S32_SCB_FPDSCR_RMode_MASK)
#define S32_SCB_FPDSCR_FZ_MASK                   0x1000000u
#define S32_SCB_FPDSCR_FZ_SHIFT                  24u
#define S32_SCB_FPDSCR_FZ_WIDTH                  1u
#define S32_SCB_FPDSCR_FZ(x)                     (((uint32_t)(((uint32_t)(x))<<S32_SCB_FPDSCR_FZ_SHIFT))&S32_SCB_FPDSCR_FZ_MASK)
#define S32_SCB_FPDSCR_DN_MASK                   0x2000000u
#define S32_SCB_FPDSCR_DN_SHIFT                  25u
#define S32_SCB_FPDSCR_DN_WIDTH                  1u
#define S32_SCB_FPDSCR_DN(x)                     (((uint32_t)(((uint32_t)(x))<<S32_SCB_FPDSCR_DN_SHIFT))&S32_SCB_FPDSCR_DN_MASK)
#define S32_SCB_FPDSCR_AHP_MASK                  0x4000000u
#define S32_SCB_FPDSCR_AHP_SHIFT                 26u
#define S32_SCB_FPDSCR_AHP_WIDTH                 1u
#define S32_SCB_FPDSCR_AHP(x)                    (((uint32_t)(((uint32_t)(x))<<S32_SCB_FPDSCR_AHP_SHIFT))&S32_SCB_FPDSCR_AHP_MASK)

/*!
 * @}
 */ /* end of group S32_SCB_Register_Masks */


/*!
 * @}
 */ /* end of group S32_SCB_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- S32_SysTick Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup S32_SysTick_Peripheral_Access_Layer S32_SysTick Peripheral Access Layer
 * @{
 */


/** S32_SysTick - Size of Registers Arrays */

/** S32_SysTick - Register Layout Typedef */
typedef struct {
  __IO uint32_t CSR;                               /**< SysTick Control and Status Register, offset: 0x0 */
  __IO uint32_t RVR;                               /**< SysTick Reload Value Register, offset: 0x4 */
  __IO uint32_t CVR;                               /**< SysTick Current Value Register, offset: 0x8 */
  __I  uint32_t CALIB;                             /**< SysTick Calibration Value Register, offset: 0xC */
} S32_SysTick_Type, *S32_SysTick_MemMapPtr;

 /** Number of instances of the S32_SysTick module. */
#define S32_SysTick_INSTANCE_COUNT               (1u)


/* S32_SysTick - Peripheral instance base addresses */
/** Peripheral S32_SysTick base address */
#define S32_SysTick_BASE                         (0xE000E010u)
/** Peripheral S32_SysTick base pointer */
#define S32_SysTick                              ((S32_SysTick_Type *)S32_SysTick_BASE)
/** Array initializer of S32_SysTick peripheral base addresses */
#define S32_SysTick_BASE_ADDRS                   { S32_SysTick_BASE }
/** Array initializer of S32_SysTick peripheral base pointers */
#define S32_SysTick_BASE_PTRS                    { S32_SysTick }

/* ----------------------------------------------------------------------------
   -- S32_SysTick Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup S32_SysTick_Register_Masks S32_SysTick Register Masks
 * @{
 */

/* CSR Bit Fields */
#define S32_SysTick_CSR_ENABLE_MASK              0x1u
#define S32_SysTick_CSR_ENABLE_SHIFT             0u
#define S32_SysTick_CSR_ENABLE_WIDTH             1u
#define S32_SysTick_CSR_ENABLE(x)                (((uint32_t)(((uint32_t)(x))<<S32_SysTick_CSR_ENABLE_SHIFT))&S32_SysTick_CSR_ENABLE_MASK)
#define S32_SysTick_CSR_TICKINT_MASK             0x2u
#define S32_SysTick_CSR_TICKINT_SHIFT            1u
#define S32_SysTick_CSR_TICKINT_WIDTH            1u
#define S32_SysTick_CSR_TICKINT(x)               (((uint32_t)(((uint32_t)(x))<<S32_SysTick_CSR_TICKINT_SHIFT))&S32_SysTick_CSR_TICKINT_MASK)
#define S32_SysTick_CSR_CLKSOURCE_MASK           0x4u
#define S32_SysTick_CSR_CLKSOURCE_SHIFT          2u
#define S32_SysTick_CSR_CLKSOURCE_WIDTH          1u
#define S32_SysTick_CSR_CLKSOURCE(x)             (((uint32_t)(((uint32_t)(x))<<S32_SysTick_CSR_CLKSOURCE_SHIFT))&S32_SysTick_CSR_CLKSOURCE_MASK)
#define S32_SysTick_CSR_COUNTFLAG_MASK           0x10000u
#define S32_SysTick_CSR_COUNTFLAG_SHIFT          16u
#define S32_SysTick_CSR_COUNTFLAG_WIDTH          1u
#define S32_SysTick_CSR_COUNTFLAG(x)             (((uint32_t)(((uint32_t)(x))<<S32_SysTick_CSR_COUNTFLAG_SHIFT))&S32_SysTick_CSR_COUNTFLAG_MASK)
/* RVR Bit Fields */
#define S32_SysTick_RVR_RELOAD_MASK              0xFFFFFFu
#define S32_SysTick_RVR_RELOAD_SHIFT             0u
#define S32_SysTick_RVR_RELOAD_WIDTH             24u
#define S32_SysTick_RVR_RELOAD(x)                (((uint32_t)(((uint32_t)(x))<<S32_SysTick_RVR_RELOAD_SHIFT))&S32_SysTick_RVR_RELOAD_MASK)
/* CVR Bit Fields */
#define S32_SysTick_CVR_CURRENT_MASK             0xFFFFFFu
#define S32_SysTick_CVR_CURRENT_SHIFT            0u
#define S32_SysTick_CVR_CURRENT_WIDTH            24u
#define S32_SysTick_CVR_CURRENT(x)               (((uint32_t)(((uint32_t)(x))<<S32_SysTick_CVR_CURRENT_SHIFT))&S32_SysTick_CVR_CURRENT_MASK)
/* CALIB Bit Fields */
#define S32_SysTick_CALIB_TENMS_MASK             0xFFFFFFu
#define S32_SysTick_CALIB_TENMS_SHIFT            0u
#define S32_SysTick_CALIB_TENMS_WIDTH            24u
#define S32_SysTick_CALIB_TENMS(x)               (((uint32_t)(((uint32_t)(x))<<S32_SysTick_CALIB_TENMS_SHIFT))&S32_SysTick_CALIB_TENMS_MASK)
#define S32_SysTick_CALIB_SKEW_MASK              0x40000000u
#define S32_SysTick_CALIB_SKEW_SHIFT             30u
#define S32_SysTick_CALIB_SKEW_WIDTH             1u
#define S32_SysTick_CALIB_SKEW(x)                (((uint32_t)(((uint32_t)(x))<<S32_SysTick_CALIB_SKEW_SHIFT))&S32_SysTick_CALIB_SKEW_MASK)
#define S32_SysTick_CALIB_NOREF_MASK             0x80000000u
#define S32_SysTick_CALIB_NOREF_SHIFT            31u
#define S32_SysTick_CALIB_NOREF_WIDTH            1u
#define S32_SysTick_CALIB_NOREF(x)               (((uint32_t)(((uint32_t)(x))<<S32_SysTick_CALIB_NOREF_SHIFT))&S32_SysTick_CALIB_NOREF_MASK)

/*!
 * @}
 */ /* end of group S32_SysTick_Register_Masks */


/*!
 * @}
 */ /* end of group S32_SysTick_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- Cortex M4 Core Configuration
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Cortex_Core_Configuration Cortex M4 Core Configuration
 * @{
 */

#define __MPU_PRESENT                  1         /**< Defines if an MPU is present or not */
#define __NVIC_PRIO_BITS               4         /**< Number of priority bits implemented in the NVIC */
#define __Vendor_SysTickConfig         0         /**< Vendor specific implementation of SysTickConfig is defined */
#define __FPU_PRESENT                  1         /**< Defines if an FPU is present or not */


/*!
 * @}
 */ /* end of group Cortex_Core_Configuration */


/* ----------------------------------------------------------------------------
   -- Device Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Peripheral_access_layer Device Peripheral Access Layer
 * @{
 */


/* ----------------------------------------------------------------------------
   -- S32_NVIC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup S32_NVIC_Peripheral_Access_Layer S32_NVIC Peripheral Access Layer
 * @{
 */


/** S32_NVIC - Size of Registers Arrays */
#define S32_NVIC_ISER_COUNT                      8u
#define S32_NVIC_ICER_COUNT                      8u
#define S32_NVIC_ISPR_COUNT                      8u
#define S32_NVIC_ICPR_COUNT                      8u
#define S32_NVIC_IABR_COUNT                      8u
#define S32_NVIC_IP_COUNT                        240u

/** S32_NVIC - Register Layout Typedef */
typedef struct {
  __IO uint32_t ISER[S32_NVIC_ISER_COUNT];         /**< Interrupt Set Enable Register n, array offset: 0x0, array step: 0x4 */
       uint8_t RESERVED_0[96];
  __IO uint32_t ICER[S32_NVIC_ICER_COUNT];         /**< Interrupt Clear Enable Register n, array offset: 0x80, array step: 0x4 */
       uint8_t RESERVED_1[96];
  __IO uint32_t ISPR[S32_NVIC_ISPR_COUNT];         /**< Interrupt Set Pending Register n, array offset: 0x100, array step: 0x4 */
       uint8_t RESERVED_2[96];
  __IO uint32_t ICPR[S32_NVIC_ICPR_COUNT];         /**< Interrupt Clear Pending Register n, array offset: 0x180, array step: 0x4 */
       uint8_t RESERVED_3[96];
  __IO uint32_t IABR[S32_NVIC_IABR_COUNT];         /**< Interrupt Active bit Register n, array offset: 0x200, array step: 0x4 */
       uint8_t RESERVED_4[224];
  __IO uint8_t IP[S32_NVIC_IP_COUNT];              /**< Interrupt Priority Register n, array offset: 0x300, array step: 0x1 */
       uint8_t RESERVED_5[2576];
  __O  uint32_t STIR;                              /**< Software Trigger Interrupt Register, offset: 0xE00 */
} S32_NVIC_Type, *S32_NVIC_MemMapPtr;

 /** Number of instances of the S32_NVIC module. */
#define S32_NVIC_INSTANCE_COUNT                  (1u)


/* S32_NVIC - Peripheral instance base addresses */
/** Peripheral S32_NVIC base address */
#define S32_NVIC_BASE                            (0xE000E100u)
/** Peripheral S32_NVIC base pointer */
#define S32_NVIC                                 ((S32_NVIC_Type *)S32_NVIC_BASE)
/** Array initializer of S32_NVIC peripheral base addresses */
#define S32_NVIC_BASE_ADDRS                      { S32_NVIC_BASE }
/** Array initializer of S32_NVIC peripheral base pointers */
#define S32_NVIC_BASE_PTRS                       { S32_NVIC }
 /** Number of interrupt vector arrays for the S32_NVIC module. */
#define S32_NVIC_IRQS_ARR_COUNT                  (1u)
 /** Number of interrupt channels for the S32_NVIC module. */
#define S32_NVIC_IRQS_CH_COUNT                   (1u)
/** Interrupt vectors for the S32_NVIC peripheral type */
#define S32_NVIC_IRQS                            { SWI_IRQn }

/* ----------------------------------------------------------------------------
   -- S32_NVIC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup S32_NVIC_Register_Masks S32_NVIC Register Masks
 * @{
 */

/* ISER Bit Fields */
#define S32_NVIC_ISER_SETENA_MASK                0xFFFFFFFFu
#define S32_NVIC_ISER_SETENA_SHIFT               0u
#define S32_NVIC_ISER_SETENA_WIDTH               32u
#define S32_NVIC_ISER_SETENA(x)                  (((uint32_t)(((uint32_t)(x))<<S32_NVIC_ISER_SETENA_SHIFT))&S32_NVIC_ISER_SETENA_MASK)
/* ICER Bit Fields */
#define S32_NVIC_ICER_CLRENA_MASK                0xFFFFFFFFu
#define S32_NVIC_ICER_CLRENA_SHIFT               0u
#define S32_NVIC_ICER_CLRENA_WIDTH               32u
#define S32_NVIC_ICER_CLRENA(x)                  (((uint32_t)(((uint32_t)(x))<<S32_NVIC_ICER_CLRENA_SHIFT))&S32_NVIC_ICER_CLRENA_MASK)
/* ISPR Bit Fields */
#define S32_NVIC_ISPR_SETPEND_MASK               0xFFFFFFFFu
#define S32_NVIC_ISPR_SETPEND_SHIFT              0u
#define S32_NVIC_ISPR_SETPEND_WIDTH              32u
#define S32_NVIC_ISPR_SETPEND(x)                 (((uint32_t)(((uint32_t)(x))<<S32_NVIC_ISPR_SETPEND_SHIFT))&S32_NVIC_ISPR_SETPEND_MASK)
/* ICPR Bit Fields */
#define S32_NVIC_ICPR_CLRPEND_MASK               0xFFFFFFFFu
#define S32_NVIC_ICPR_CLRPEND_SHIFT              0u
#define S32_NVIC_ICPR_CLRPEND_WIDTH              32u
#define S32_NVIC_ICPR_CLRPEND(x)                 (((uint32_t)(((uint32_t)(x))<<S32_NVIC_ICPR_CLRPEND_SHIFT))&S32_NVIC_ICPR_CLRPEND_MASK)
/* IABR Bit Fields */
#define S32_NVIC_IABR_ACTIVE_MASK                0xFFFFFFFFu
#define S32_NVIC_IABR_ACTIVE_SHIFT               0u
#define S32_NVIC_IABR_ACTIVE_WIDTH               32u
#define S32_NVIC_IABR_ACTIVE(x)                  (((uint32_t)(((uint32_t)(x))<<S32_NVIC_IABR_ACTIVE_SHIFT))&S32_NVIC_IABR_ACTIVE_MASK)
/* IP Bit Fields */
#define S32_NVIC_IP_PRI0_MASK                    0xFFu
#define S32_NVIC_IP_PRI0_SHIFT                   0u
#define S32_NVIC_IP_PRI0_WIDTH                   8u
#define S32_NVIC_IP_PRI0(x)                      (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI0_SHIFT))&S32_NVIC_IP_PRI0_MASK)
#define S32_NVIC_IP_PRI1_MASK                    0xFFu
#define S32_NVIC_IP_PRI1_SHIFT                   0u
#define S32_NVIC_IP_PRI1_WIDTH                   8u
#define S32_NVIC_IP_PRI1(x)                      (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI1_SHIFT))&S32_NVIC_IP_PRI1_MASK)
#define S32_NVIC_IP_PRI2_MASK                    0xFFu
#define S32_NVIC_IP_PRI2_SHIFT                   0u
#define S32_NVIC_IP_PRI2_WIDTH                   8u
#define S32_NVIC_IP_PRI2(x)                      (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI2_SHIFT))&S32_NVIC_IP_PRI2_MASK)
#define S32_NVIC_IP_PRI3_MASK                    0xFFu
#define S32_NVIC_IP_PRI3_SHIFT                   0u
#define S32_NVIC_IP_PRI3_WIDTH                   8u
#define S32_NVIC_IP_PRI3(x)                      (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI3_SHIFT))&S32_NVIC_IP_PRI3_MASK)
#define S32_NVIC_IP_PRI4_MASK                    0xFFu
#define S32_NVIC_IP_PRI4_SHIFT                   0u
#define S32_NVIC_IP_PRI4_WIDTH                   8u
#define S32_NVIC_IP_PRI4(x)                      (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI4_SHIFT))&S32_NVIC_IP_PRI4_MASK)
#define S32_NVIC_IP_PRI5_MASK                    0xFFu
#define S32_NVIC_IP_PRI5_SHIFT                   0u
#define S32_NVIC_IP_PRI5_WIDTH                   8u
#define S32_NVIC_IP_PRI5(x)                      (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI5_SHIFT))&S32_NVIC_IP_PRI5_MASK)
#define S32_NVIC_IP_PRI6_MASK                    0xFFu
#define S32_NVIC_IP_PRI6_SHIFT                   0u
#define S32_NVIC_IP_PRI6_WIDTH                   8u
#define S32_NVIC_IP_PRI6(x)                      (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI6_SHIFT))&S32_NVIC_IP_PRI6_MASK)
#define S32_NVIC_IP_PRI7_MASK                    0xFFu
#define S32_NVIC_IP_PRI7_SHIFT                   0u
#define S32_NVIC_IP_PRI7_WIDTH                   8u
#define S32_NVIC_IP_PRI7(x)                      (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI7_SHIFT))&S32_NVIC_IP_PRI7_MASK)
#define S32_NVIC_IP_PRI8_MASK                    0xFFu
#define S32_NVIC_IP_PRI8_SHIFT                   0u
#define S32_NVIC_IP_PRI8_WIDTH                   8u
#define S32_NVIC_IP_PRI8(x)                      (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI8_SHIFT))&S32_NVIC_IP_PRI8_MASK)
#define S32_NVIC_IP_PRI9_MASK                    0xFFu
#define S32_NVIC_IP_PRI9_SHIFT                   0u
#define S32_NVIC_IP_PRI9_WIDTH                   8u
#define S32_NVIC_IP_PRI9(x)                      (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI9_SHIFT))&S32_NVIC_IP_PRI9_MASK)
#define S32_NVIC_IP_PRI10_MASK                   0xFFu
#define S32_NVIC_IP_PRI10_SHIFT                  0u
#define S32_NVIC_IP_PRI10_WIDTH                  8u
#define S32_NVIC_IP_PRI10(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI10_SHIFT))&S32_NVIC_IP_PRI10_MASK)
#define S32_NVIC_IP_PRI11_MASK                   0xFFu
#define S32_NVIC_IP_PRI11_SHIFT                  0u
#define S32_NVIC_IP_PRI11_WIDTH                  8u
#define S32_NVIC_IP_PRI11(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI11_SHIFT))&S32_NVIC_IP_PRI11_MASK)
#define S32_NVIC_IP_PRI12_MASK                   0xFFu
#define S32_NVIC_IP_PRI12_SHIFT                  0u
#define S32_NVIC_IP_PRI12_WIDTH                  8u
#define S32_NVIC_IP_PRI12(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI12_SHIFT))&S32_NVIC_IP_PRI12_MASK)
#define S32_NVIC_IP_PRI13_MASK                   0xFFu
#define S32_NVIC_IP_PRI13_SHIFT                  0u
#define S32_NVIC_IP_PRI13_WIDTH                  8u
#define S32_NVIC_IP_PRI13(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI13_SHIFT))&S32_NVIC_IP_PRI13_MASK)
#define S32_NVIC_IP_PRI14_MASK                   0xFFu
#define S32_NVIC_IP_PRI14_SHIFT                  0u
#define S32_NVIC_IP_PRI14_WIDTH                  8u
#define S32_NVIC_IP_PRI14(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI14_SHIFT))&S32_NVIC_IP_PRI14_MASK)
#define S32_NVIC_IP_PRI15_MASK                   0xFFu
#define S32_NVIC_IP_PRI15_SHIFT                  0u
#define S32_NVIC_IP_PRI15_WIDTH                  8u
#define S32_NVIC_IP_PRI15(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI15_SHIFT))&S32_NVIC_IP_PRI15_MASK)
#define S32_NVIC_IP_PRI16_MASK                   0xFFu
#define S32_NVIC_IP_PRI16_SHIFT                  0u
#define S32_NVIC_IP_PRI16_WIDTH                  8u
#define S32_NVIC_IP_PRI16(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI16_SHIFT))&S32_NVIC_IP_PRI16_MASK)
#define S32_NVIC_IP_PRI17_MASK                   0xFFu
#define S32_NVIC_IP_PRI17_SHIFT                  0u
#define S32_NVIC_IP_PRI17_WIDTH                  8u
#define S32_NVIC_IP_PRI17(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI17_SHIFT))&S32_NVIC_IP_PRI17_MASK)
#define S32_NVIC_IP_PRI18_MASK                   0xFFu
#define S32_NVIC_IP_PRI18_SHIFT                  0u
#define S32_NVIC_IP_PRI18_WIDTH                  8u
#define S32_NVIC_IP_PRI18(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI18_SHIFT))&S32_NVIC_IP_PRI18_MASK)
#define S32_NVIC_IP_PRI19_MASK                   0xFFu
#define S32_NVIC_IP_PRI19_SHIFT                  0u
#define S32_NVIC_IP_PRI19_WIDTH                  8u
#define S32_NVIC_IP_PRI19(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI19_SHIFT))&S32_NVIC_IP_PRI19_MASK)
#define S32_NVIC_IP_PRI20_MASK                   0xFFu
#define S32_NVIC_IP_PRI20_SHIFT                  0u
#define S32_NVIC_IP_PRI20_WIDTH                  8u
#define S32_NVIC_IP_PRI20(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI20_SHIFT))&S32_NVIC_IP_PRI20_MASK)
#define S32_NVIC_IP_PRI21_MASK                   0xFFu
#define S32_NVIC_IP_PRI21_SHIFT                  0u
#define S32_NVIC_IP_PRI21_WIDTH                  8u
#define S32_NVIC_IP_PRI21(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI21_SHIFT))&S32_NVIC_IP_PRI21_MASK)
#define S32_NVIC_IP_PRI22_MASK                   0xFFu
#define S32_NVIC_IP_PRI22_SHIFT                  0u
#define S32_NVIC_IP_PRI22_WIDTH                  8u
#define S32_NVIC_IP_PRI22(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI22_SHIFT))&S32_NVIC_IP_PRI22_MASK)
#define S32_NVIC_IP_PRI23_MASK                   0xFFu
#define S32_NVIC_IP_PRI23_SHIFT                  0u
#define S32_NVIC_IP_PRI23_WIDTH                  8u
#define S32_NVIC_IP_PRI23(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI23_SHIFT))&S32_NVIC_IP_PRI23_MASK)
#define S32_NVIC_IP_PRI24_MASK                   0xFFu
#define S32_NVIC_IP_PRI24_SHIFT                  0u
#define S32_NVIC_IP_PRI24_WIDTH                  8u
#define S32_NVIC_IP_PRI24(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI24_SHIFT))&S32_NVIC_IP_PRI24_MASK)
#define S32_NVIC_IP_PRI25_MASK                   0xFFu
#define S32_NVIC_IP_PRI25_SHIFT                  0u
#define S32_NVIC_IP_PRI25_WIDTH                  8u
#define S32_NVIC_IP_PRI25(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI25_SHIFT))&S32_NVIC_IP_PRI25_MASK)
#define S32_NVIC_IP_PRI26_MASK                   0xFFu
#define S32_NVIC_IP_PRI26_SHIFT                  0u
#define S32_NVIC_IP_PRI26_WIDTH                  8u
#define S32_NVIC_IP_PRI26(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI26_SHIFT))&S32_NVIC_IP_PRI26_MASK)
#define S32_NVIC_IP_PRI27_MASK                   0xFFu
#define S32_NVIC_IP_PRI27_SHIFT                  0u
#define S32_NVIC_IP_PRI27_WIDTH                  8u
#define S32_NVIC_IP_PRI27(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI27_SHIFT))&S32_NVIC_IP_PRI27_MASK)
#define S32_NVIC_IP_PRI28_MASK                   0xFFu
#define S32_NVIC_IP_PRI28_SHIFT                  0u
#define S32_NVIC_IP_PRI28_WIDTH                  8u
#define S32_NVIC_IP_PRI28(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI28_SHIFT))&S32_NVIC_IP_PRI28_MASK)
#define S32_NVIC_IP_PRI29_MASK                   0xFFu
#define S32_NVIC_IP_PRI29_SHIFT                  0u
#define S32_NVIC_IP_PRI29_WIDTH                  8u
#define S32_NVIC_IP_PRI29(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI29_SHIFT))&S32_NVIC_IP_PRI29_MASK)
#define S32_NVIC_IP_PRI30_MASK                   0xFFu
#define S32_NVIC_IP_PRI30_SHIFT                  0u
#define S32_NVIC_IP_PRI30_WIDTH                  8u
#define S32_NVIC_IP_PRI30(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI30_SHIFT))&S32_NVIC_IP_PRI30_MASK)
#define S32_NVIC_IP_PRI31_MASK                   0xFFu
#define S32_NVIC_IP_PRI31_SHIFT                  0u
#define S32_NVIC_IP_PRI31_WIDTH                  8u
#define S32_NVIC_IP_PRI31(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI31_SHIFT))&S32_NVIC_IP_PRI31_MASK)
#define S32_NVIC_IP_PRI32_MASK                   0xFFu
#define S32_NVIC_IP_PRI32_SHIFT                  0u
#define S32_NVIC_IP_PRI32_WIDTH                  8u
#define S32_NVIC_IP_PRI32(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI32_SHIFT))&S32_NVIC_IP_PRI32_MASK)
#define S32_NVIC_IP_PRI33_MASK                   0xFFu
#define S32_NVIC_IP_PRI33_SHIFT                  0u
#define S32_NVIC_IP_PRI33_WIDTH                  8u
#define S32_NVIC_IP_PRI33(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI33_SHIFT))&S32_NVIC_IP_PRI33_MASK)
#define S32_NVIC_IP_PRI34_MASK                   0xFFu
#define S32_NVIC_IP_PRI34_SHIFT                  0u
#define S32_NVIC_IP_PRI34_WIDTH                  8u
#define S32_NVIC_IP_PRI34(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI34_SHIFT))&S32_NVIC_IP_PRI34_MASK)
#define S32_NVIC_IP_PRI35_MASK                   0xFFu
#define S32_NVIC_IP_PRI35_SHIFT                  0u
#define S32_NVIC_IP_PRI35_WIDTH                  8u
#define S32_NVIC_IP_PRI35(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI35_SHIFT))&S32_NVIC_IP_PRI35_MASK)
#define S32_NVIC_IP_PRI36_MASK                   0xFFu
#define S32_NVIC_IP_PRI36_SHIFT                  0u
#define S32_NVIC_IP_PRI36_WIDTH                  8u
#define S32_NVIC_IP_PRI36(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI36_SHIFT))&S32_NVIC_IP_PRI36_MASK)
#define S32_NVIC_IP_PRI37_MASK                   0xFFu
#define S32_NVIC_IP_PRI37_SHIFT                  0u
#define S32_NVIC_IP_PRI37_WIDTH                  8u
#define S32_NVIC_IP_PRI37(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI37_SHIFT))&S32_NVIC_IP_PRI37_MASK)
#define S32_NVIC_IP_PRI38_MASK                   0xFFu
#define S32_NVIC_IP_PRI38_SHIFT                  0u
#define S32_NVIC_IP_PRI38_WIDTH                  8u
#define S32_NVIC_IP_PRI38(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI38_SHIFT))&S32_NVIC_IP_PRI38_MASK)
#define S32_NVIC_IP_PRI39_MASK                   0xFFu
#define S32_NVIC_IP_PRI39_SHIFT                  0u
#define S32_NVIC_IP_PRI39_WIDTH                  8u
#define S32_NVIC_IP_PRI39(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI39_SHIFT))&S32_NVIC_IP_PRI39_MASK)
#define S32_NVIC_IP_PRI40_MASK                   0xFFu
#define S32_NVIC_IP_PRI40_SHIFT                  0u
#define S32_NVIC_IP_PRI40_WIDTH                  8u
#define S32_NVIC_IP_PRI40(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI40_SHIFT))&S32_NVIC_IP_PRI40_MASK)
#define S32_NVIC_IP_PRI41_MASK                   0xFFu
#define S32_NVIC_IP_PRI41_SHIFT                  0u
#define S32_NVIC_IP_PRI41_WIDTH                  8u
#define S32_NVIC_IP_PRI41(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI41_SHIFT))&S32_NVIC_IP_PRI41_MASK)
#define S32_NVIC_IP_PRI42_MASK                   0xFFu
#define S32_NVIC_IP_PRI42_SHIFT                  0u
#define S32_NVIC_IP_PRI42_WIDTH                  8u
#define S32_NVIC_IP_PRI42(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI42_SHIFT))&S32_NVIC_IP_PRI42_MASK)
#define S32_NVIC_IP_PRI43_MASK                   0xFFu
#define S32_NVIC_IP_PRI43_SHIFT                  0u
#define S32_NVIC_IP_PRI43_WIDTH                  8u
#define S32_NVIC_IP_PRI43(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI43_SHIFT))&S32_NVIC_IP_PRI43_MASK)
#define S32_NVIC_IP_PRI44_MASK                   0xFFu
#define S32_NVIC_IP_PRI44_SHIFT                  0u
#define S32_NVIC_IP_PRI44_WIDTH                  8u
#define S32_NVIC_IP_PRI44(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI44_SHIFT))&S32_NVIC_IP_PRI44_MASK)
#define S32_NVIC_IP_PRI45_MASK                   0xFFu
#define S32_NVIC_IP_PRI45_SHIFT                  0u
#define S32_NVIC_IP_PRI45_WIDTH                  8u
#define S32_NVIC_IP_PRI45(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI45_SHIFT))&S32_NVIC_IP_PRI45_MASK)
#define S32_NVIC_IP_PRI46_MASK                   0xFFu
#define S32_NVIC_IP_PRI46_SHIFT                  0u
#define S32_NVIC_IP_PRI46_WIDTH                  8u
#define S32_NVIC_IP_PRI46(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI46_SHIFT))&S32_NVIC_IP_PRI46_MASK)
#define S32_NVIC_IP_PRI47_MASK                   0xFFu
#define S32_NVIC_IP_PRI47_SHIFT                  0u
#define S32_NVIC_IP_PRI47_WIDTH                  8u
#define S32_NVIC_IP_PRI47(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI47_SHIFT))&S32_NVIC_IP_PRI47_MASK)
#define S32_NVIC_IP_PRI48_MASK                   0xFFu
#define S32_NVIC_IP_PRI48_SHIFT                  0u
#define S32_NVIC_IP_PRI48_WIDTH                  8u
#define S32_NVIC_IP_PRI48(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI48_SHIFT))&S32_NVIC_IP_PRI48_MASK)
#define S32_NVIC_IP_PRI49_MASK                   0xFFu
#define S32_NVIC_IP_PRI49_SHIFT                  0u
#define S32_NVIC_IP_PRI49_WIDTH                  8u
#define S32_NVIC_IP_PRI49(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI49_SHIFT))&S32_NVIC_IP_PRI49_MASK)
#define S32_NVIC_IP_PRI50_MASK                   0xFFu
#define S32_NVIC_IP_PRI50_SHIFT                  0u
#define S32_NVIC_IP_PRI50_WIDTH                  8u
#define S32_NVIC_IP_PRI50(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI50_SHIFT))&S32_NVIC_IP_PRI50_MASK)
#define S32_NVIC_IP_PRI51_MASK                   0xFFu
#define S32_NVIC_IP_PRI51_SHIFT                  0u
#define S32_NVIC_IP_PRI51_WIDTH                  8u
#define S32_NVIC_IP_PRI51(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI51_SHIFT))&S32_NVIC_IP_PRI51_MASK)
#define S32_NVIC_IP_PRI52_MASK                   0xFFu
#define S32_NVIC_IP_PRI52_SHIFT                  0u
#define S32_NVIC_IP_PRI52_WIDTH                  8u
#define S32_NVIC_IP_PRI52(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI52_SHIFT))&S32_NVIC_IP_PRI52_MASK)
#define S32_NVIC_IP_PRI53_MASK                   0xFFu
#define S32_NVIC_IP_PRI53_SHIFT                  0u
#define S32_NVIC_IP_PRI53_WIDTH                  8u
#define S32_NVIC_IP_PRI53(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI53_SHIFT))&S32_NVIC_IP_PRI53_MASK)
#define S32_NVIC_IP_PRI54_MASK                   0xFFu
#define S32_NVIC_IP_PRI54_SHIFT                  0u
#define S32_NVIC_IP_PRI54_WIDTH                  8u
#define S32_NVIC_IP_PRI54(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI54_SHIFT))&S32_NVIC_IP_PRI54_MASK)
#define S32_NVIC_IP_PRI55_MASK                   0xFFu
#define S32_NVIC_IP_PRI55_SHIFT                  0u
#define S32_NVIC_IP_PRI55_WIDTH                  8u
#define S32_NVIC_IP_PRI55(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI55_SHIFT))&S32_NVIC_IP_PRI55_MASK)
#define S32_NVIC_IP_PRI56_MASK                   0xFFu
#define S32_NVIC_IP_PRI56_SHIFT                  0u
#define S32_NVIC_IP_PRI56_WIDTH                  8u
#define S32_NVIC_IP_PRI56(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI56_SHIFT))&S32_NVIC_IP_PRI56_MASK)
#define S32_NVIC_IP_PRI57_MASK                   0xFFu
#define S32_NVIC_IP_PRI57_SHIFT                  0u
#define S32_NVIC_IP_PRI57_WIDTH                  8u
#define S32_NVIC_IP_PRI57(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI57_SHIFT))&S32_NVIC_IP_PRI57_MASK)
#define S32_NVIC_IP_PRI58_MASK                   0xFFu
#define S32_NVIC_IP_PRI58_SHIFT                  0u
#define S32_NVIC_IP_PRI58_WIDTH                  8u
#define S32_NVIC_IP_PRI58(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI58_SHIFT))&S32_NVIC_IP_PRI58_MASK)
#define S32_NVIC_IP_PRI59_MASK                   0xFFu
#define S32_NVIC_IP_PRI59_SHIFT                  0u
#define S32_NVIC_IP_PRI59_WIDTH                  8u
#define S32_NVIC_IP_PRI59(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI59_SHIFT))&S32_NVIC_IP_PRI59_MASK)
#define S32_NVIC_IP_PRI60_MASK                   0xFFu
#define S32_NVIC_IP_PRI60_SHIFT                  0u
#define S32_NVIC_IP_PRI60_WIDTH                  8u
#define S32_NVIC_IP_PRI60(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI60_SHIFT))&S32_NVIC_IP_PRI60_MASK)
#define S32_NVIC_IP_PRI61_MASK                   0xFFu
#define S32_NVIC_IP_PRI61_SHIFT                  0u
#define S32_NVIC_IP_PRI61_WIDTH                  8u
#define S32_NVIC_IP_PRI61(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI61_SHIFT))&S32_NVIC_IP_PRI61_MASK)
#define S32_NVIC_IP_PRI62_MASK                   0xFFu
#define S32_NVIC_IP_PRI62_SHIFT                  0u
#define S32_NVIC_IP_PRI62_WIDTH                  8u
#define S32_NVIC_IP_PRI62(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI62_SHIFT))&S32_NVIC_IP_PRI62_MASK)
#define S32_NVIC_IP_PRI63_MASK                   0xFFu
#define S32_NVIC_IP_PRI63_SHIFT                  0u
#define S32_NVIC_IP_PRI63_WIDTH                  8u
#define S32_NVIC_IP_PRI63(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI63_SHIFT))&S32_NVIC_IP_PRI63_MASK)
#define S32_NVIC_IP_PRI64_MASK                   0xFFu
#define S32_NVIC_IP_PRI64_SHIFT                  0u
#define S32_NVIC_IP_PRI64_WIDTH                  8u
#define S32_NVIC_IP_PRI64(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI64_SHIFT))&S32_NVIC_IP_PRI64_MASK)
#define S32_NVIC_IP_PRI65_MASK                   0xFFu
#define S32_NVIC_IP_PRI65_SHIFT                  0u
#define S32_NVIC_IP_PRI65_WIDTH                  8u
#define S32_NVIC_IP_PRI65(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI65_SHIFT))&S32_NVIC_IP_PRI65_MASK)
#define S32_NVIC_IP_PRI66_MASK                   0xFFu
#define S32_NVIC_IP_PRI66_SHIFT                  0u
#define S32_NVIC_IP_PRI66_WIDTH                  8u
#define S32_NVIC_IP_PRI66(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI66_SHIFT))&S32_NVIC_IP_PRI66_MASK)
#define S32_NVIC_IP_PRI67_MASK                   0xFFu
#define S32_NVIC_IP_PRI67_SHIFT                  0u
#define S32_NVIC_IP_PRI67_WIDTH                  8u
#define S32_NVIC_IP_PRI67(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI67_SHIFT))&S32_NVIC_IP_PRI67_MASK)
#define S32_NVIC_IP_PRI68_MASK                   0xFFu
#define S32_NVIC_IP_PRI68_SHIFT                  0u
#define S32_NVIC_IP_PRI68_WIDTH                  8u
#define S32_NVIC_IP_PRI68(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI68_SHIFT))&S32_NVIC_IP_PRI68_MASK)
#define S32_NVIC_IP_PRI69_MASK                   0xFFu
#define S32_NVIC_IP_PRI69_SHIFT                  0u
#define S32_NVIC_IP_PRI69_WIDTH                  8u
#define S32_NVIC_IP_PRI69(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI69_SHIFT))&S32_NVIC_IP_PRI69_MASK)
#define S32_NVIC_IP_PRI70_MASK                   0xFFu
#define S32_NVIC_IP_PRI70_SHIFT                  0u
#define S32_NVIC_IP_PRI70_WIDTH                  8u
#define S32_NVIC_IP_PRI70(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI70_SHIFT))&S32_NVIC_IP_PRI70_MASK)
#define S32_NVIC_IP_PRI71_MASK                   0xFFu
#define S32_NVIC_IP_PRI71_SHIFT                  0u
#define S32_NVIC_IP_PRI71_WIDTH                  8u
#define S32_NVIC_IP_PRI71(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI71_SHIFT))&S32_NVIC_IP_PRI71_MASK)
#define S32_NVIC_IP_PRI72_MASK                   0xFFu
#define S32_NVIC_IP_PRI72_SHIFT                  0u
#define S32_NVIC_IP_PRI72_WIDTH                  8u
#define S32_NVIC_IP_PRI72(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI72_SHIFT))&S32_NVIC_IP_PRI72_MASK)
#define S32_NVIC_IP_PRI73_MASK                   0xFFu
#define S32_NVIC_IP_PRI73_SHIFT                  0u
#define S32_NVIC_IP_PRI73_WIDTH                  8u
#define S32_NVIC_IP_PRI73(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI73_SHIFT))&S32_NVIC_IP_PRI73_MASK)
#define S32_NVIC_IP_PRI74_MASK                   0xFFu
#define S32_NVIC_IP_PRI74_SHIFT                  0u
#define S32_NVIC_IP_PRI74_WIDTH                  8u
#define S32_NVIC_IP_PRI74(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI74_SHIFT))&S32_NVIC_IP_PRI74_MASK)
#define S32_NVIC_IP_PRI75_MASK                   0xFFu
#define S32_NVIC_IP_PRI75_SHIFT                  0u
#define S32_NVIC_IP_PRI75_WIDTH                  8u
#define S32_NVIC_IP_PRI75(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI75_SHIFT))&S32_NVIC_IP_PRI75_MASK)
#define S32_NVIC_IP_PRI76_MASK                   0xFFu
#define S32_NVIC_IP_PRI76_SHIFT                  0u
#define S32_NVIC_IP_PRI76_WIDTH                  8u
#define S32_NVIC_IP_PRI76(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI76_SHIFT))&S32_NVIC_IP_PRI76_MASK)
#define S32_NVIC_IP_PRI77_MASK                   0xFFu
#define S32_NVIC_IP_PRI77_SHIFT                  0u
#define S32_NVIC_IP_PRI77_WIDTH                  8u
#define S32_NVIC_IP_PRI77(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI77_SHIFT))&S32_NVIC_IP_PRI77_MASK)
#define S32_NVIC_IP_PRI78_MASK                   0xFFu
#define S32_NVIC_IP_PRI78_SHIFT                  0u
#define S32_NVIC_IP_PRI78_WIDTH                  8u
#define S32_NVIC_IP_PRI78(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI78_SHIFT))&S32_NVIC_IP_PRI78_MASK)
#define S32_NVIC_IP_PRI79_MASK                   0xFFu
#define S32_NVIC_IP_PRI79_SHIFT                  0u
#define S32_NVIC_IP_PRI79_WIDTH                  8u
#define S32_NVIC_IP_PRI79(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI79_SHIFT))&S32_NVIC_IP_PRI79_MASK)
#define S32_NVIC_IP_PRI80_MASK                   0xFFu
#define S32_NVIC_IP_PRI80_SHIFT                  0u
#define S32_NVIC_IP_PRI80_WIDTH                  8u
#define S32_NVIC_IP_PRI80(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI80_SHIFT))&S32_NVIC_IP_PRI80_MASK)
#define S32_NVIC_IP_PRI81_MASK                   0xFFu
#define S32_NVIC_IP_PRI81_SHIFT                  0u
#define S32_NVIC_IP_PRI81_WIDTH                  8u
#define S32_NVIC_IP_PRI81(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI81_SHIFT))&S32_NVIC_IP_PRI81_MASK)
#define S32_NVIC_IP_PRI82_MASK                   0xFFu
#define S32_NVIC_IP_PRI82_SHIFT                  0u
#define S32_NVIC_IP_PRI82_WIDTH                  8u
#define S32_NVIC_IP_PRI82(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI82_SHIFT))&S32_NVIC_IP_PRI82_MASK)
#define S32_NVIC_IP_PRI83_MASK                   0xFFu
#define S32_NVIC_IP_PRI83_SHIFT                  0u
#define S32_NVIC_IP_PRI83_WIDTH                  8u
#define S32_NVIC_IP_PRI83(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI83_SHIFT))&S32_NVIC_IP_PRI83_MASK)
#define S32_NVIC_IP_PRI84_MASK                   0xFFu
#define S32_NVIC_IP_PRI84_SHIFT                  0u
#define S32_NVIC_IP_PRI84_WIDTH                  8u
#define S32_NVIC_IP_PRI84(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI84_SHIFT))&S32_NVIC_IP_PRI84_MASK)
#define S32_NVIC_IP_PRI85_MASK                   0xFFu
#define S32_NVIC_IP_PRI85_SHIFT                  0u
#define S32_NVIC_IP_PRI85_WIDTH                  8u
#define S32_NVIC_IP_PRI85(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI85_SHIFT))&S32_NVIC_IP_PRI85_MASK)
#define S32_NVIC_IP_PRI86_MASK                   0xFFu
#define S32_NVIC_IP_PRI86_SHIFT                  0u
#define S32_NVIC_IP_PRI86_WIDTH                  8u
#define S32_NVIC_IP_PRI86(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI86_SHIFT))&S32_NVIC_IP_PRI86_MASK)
#define S32_NVIC_IP_PRI87_MASK                   0xFFu
#define S32_NVIC_IP_PRI87_SHIFT                  0u
#define S32_NVIC_IP_PRI87_WIDTH                  8u
#define S32_NVIC_IP_PRI87(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI87_SHIFT))&S32_NVIC_IP_PRI87_MASK)
#define S32_NVIC_IP_PRI88_MASK                   0xFFu
#define S32_NVIC_IP_PRI88_SHIFT                  0u
#define S32_NVIC_IP_PRI88_WIDTH                  8u
#define S32_NVIC_IP_PRI88(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI88_SHIFT))&S32_NVIC_IP_PRI88_MASK)
#define S32_NVIC_IP_PRI89_MASK                   0xFFu
#define S32_NVIC_IP_PRI89_SHIFT                  0u
#define S32_NVIC_IP_PRI89_WIDTH                  8u
#define S32_NVIC_IP_PRI89(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI89_SHIFT))&S32_NVIC_IP_PRI89_MASK)
#define S32_NVIC_IP_PRI90_MASK                   0xFFu
#define S32_NVIC_IP_PRI90_SHIFT                  0u
#define S32_NVIC_IP_PRI90_WIDTH                  8u
#define S32_NVIC_IP_PRI90(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI90_SHIFT))&S32_NVIC_IP_PRI90_MASK)
#define S32_NVIC_IP_PRI91_MASK                   0xFFu
#define S32_NVIC_IP_PRI91_SHIFT                  0u
#define S32_NVIC_IP_PRI91_WIDTH                  8u
#define S32_NVIC_IP_PRI91(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI91_SHIFT))&S32_NVIC_IP_PRI91_MASK)
#define S32_NVIC_IP_PRI92_MASK                   0xFFu
#define S32_NVIC_IP_PRI92_SHIFT                  0u
#define S32_NVIC_IP_PRI92_WIDTH                  8u
#define S32_NVIC_IP_PRI92(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI92_SHIFT))&S32_NVIC_IP_PRI92_MASK)
#define S32_NVIC_IP_PRI93_MASK                   0xFFu
#define S32_NVIC_IP_PRI93_SHIFT                  0u
#define S32_NVIC_IP_PRI93_WIDTH                  8u
#define S32_NVIC_IP_PRI93(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI93_SHIFT))&S32_NVIC_IP_PRI93_MASK)
#define S32_NVIC_IP_PRI94_MASK                   0xFFu
#define S32_NVIC_IP_PRI94_SHIFT                  0u
#define S32_NVIC_IP_PRI94_WIDTH                  8u
#define S32_NVIC_IP_PRI94(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI94_SHIFT))&S32_NVIC_IP_PRI94_MASK)
#define S32_NVIC_IP_PRI95_MASK                   0xFFu
#define S32_NVIC_IP_PRI95_SHIFT                  0u
#define S32_NVIC_IP_PRI95_WIDTH                  8u
#define S32_NVIC_IP_PRI95(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI95_SHIFT))&S32_NVIC_IP_PRI95_MASK)
#define S32_NVIC_IP_PRI96_MASK                   0xFFu
#define S32_NVIC_IP_PRI96_SHIFT                  0u
#define S32_NVIC_IP_PRI96_WIDTH                  8u
#define S32_NVIC_IP_PRI96(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI96_SHIFT))&S32_NVIC_IP_PRI96_MASK)
#define S32_NVIC_IP_PRI97_MASK                   0xFFu
#define S32_NVIC_IP_PRI97_SHIFT                  0u
#define S32_NVIC_IP_PRI97_WIDTH                  8u
#define S32_NVIC_IP_PRI97(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI97_SHIFT))&S32_NVIC_IP_PRI97_MASK)
#define S32_NVIC_IP_PRI98_MASK                   0xFFu
#define S32_NVIC_IP_PRI98_SHIFT                  0u
#define S32_NVIC_IP_PRI98_WIDTH                  8u
#define S32_NVIC_IP_PRI98(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI98_SHIFT))&S32_NVIC_IP_PRI98_MASK)
#define S32_NVIC_IP_PRI99_MASK                   0xFFu
#define S32_NVIC_IP_PRI99_SHIFT                  0u
#define S32_NVIC_IP_PRI99_WIDTH                  8u
#define S32_NVIC_IP_PRI99(x)                     (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI99_SHIFT))&S32_NVIC_IP_PRI99_MASK)
#define S32_NVIC_IP_PRI100_MASK                  0xFFu
#define S32_NVIC_IP_PRI100_SHIFT                 0u
#define S32_NVIC_IP_PRI100_WIDTH                 8u
#define S32_NVIC_IP_PRI100(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI100_SHIFT))&S32_NVIC_IP_PRI100_MASK)
#define S32_NVIC_IP_PRI101_MASK                  0xFFu
#define S32_NVIC_IP_PRI101_SHIFT                 0u
#define S32_NVIC_IP_PRI101_WIDTH                 8u
#define S32_NVIC_IP_PRI101(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI101_SHIFT))&S32_NVIC_IP_PRI101_MASK)
#define S32_NVIC_IP_PRI102_MASK                  0xFFu
#define S32_NVIC_IP_PRI102_SHIFT                 0u
#define S32_NVIC_IP_PRI102_WIDTH                 8u
#define S32_NVIC_IP_PRI102(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI102_SHIFT))&S32_NVIC_IP_PRI102_MASK)
#define S32_NVIC_IP_PRI103_MASK                  0xFFu
#define S32_NVIC_IP_PRI103_SHIFT                 0u
#define S32_NVIC_IP_PRI103_WIDTH                 8u
#define S32_NVIC_IP_PRI103(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI103_SHIFT))&S32_NVIC_IP_PRI103_MASK)
#define S32_NVIC_IP_PRI104_MASK                  0xFFu
#define S32_NVIC_IP_PRI104_SHIFT                 0u
#define S32_NVIC_IP_PRI104_WIDTH                 8u
#define S32_NVIC_IP_PRI104(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI104_SHIFT))&S32_NVIC_IP_PRI104_MASK)
#define S32_NVIC_IP_PRI105_MASK                  0xFFu
#define S32_NVIC_IP_PRI105_SHIFT                 0u
#define S32_NVIC_IP_PRI105_WIDTH                 8u
#define S32_NVIC_IP_PRI105(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI105_SHIFT))&S32_NVIC_IP_PRI105_MASK)
#define S32_NVIC_IP_PRI106_MASK                  0xFFu
#define S32_NVIC_IP_PRI106_SHIFT                 0u
#define S32_NVIC_IP_PRI106_WIDTH                 8u
#define S32_NVIC_IP_PRI106(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI106_SHIFT))&S32_NVIC_IP_PRI106_MASK)
#define S32_NVIC_IP_PRI107_MASK                  0xFFu
#define S32_NVIC_IP_PRI107_SHIFT                 0u
#define S32_NVIC_IP_PRI107_WIDTH                 8u
#define S32_NVIC_IP_PRI107(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI107_SHIFT))&S32_NVIC_IP_PRI107_MASK)
#define S32_NVIC_IP_PRI108_MASK                  0xFFu
#define S32_NVIC_IP_PRI108_SHIFT                 0u
#define S32_NVIC_IP_PRI108_WIDTH                 8u
#define S32_NVIC_IP_PRI108(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI108_SHIFT))&S32_NVIC_IP_PRI108_MASK)
#define S32_NVIC_IP_PRI109_MASK                  0xFFu
#define S32_NVIC_IP_PRI109_SHIFT                 0u
#define S32_NVIC_IP_PRI109_WIDTH                 8u
#define S32_NVIC_IP_PRI109(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI109_SHIFT))&S32_NVIC_IP_PRI109_MASK)
#define S32_NVIC_IP_PRI110_MASK                  0xFFu
#define S32_NVIC_IP_PRI110_SHIFT                 0u
#define S32_NVIC_IP_PRI110_WIDTH                 8u
#define S32_NVIC_IP_PRI110(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI110_SHIFT))&S32_NVIC_IP_PRI110_MASK)
#define S32_NVIC_IP_PRI111_MASK                  0xFFu
#define S32_NVIC_IP_PRI111_SHIFT                 0u
#define S32_NVIC_IP_PRI111_WIDTH                 8u
#define S32_NVIC_IP_PRI111(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI111_SHIFT))&S32_NVIC_IP_PRI111_MASK)
#define S32_NVIC_IP_PRI112_MASK                  0xFFu
#define S32_NVIC_IP_PRI112_SHIFT                 0u
#define S32_NVIC_IP_PRI112_WIDTH                 8u
#define S32_NVIC_IP_PRI112(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI112_SHIFT))&S32_NVIC_IP_PRI112_MASK)
#define S32_NVIC_IP_PRI113_MASK                  0xFFu
#define S32_NVIC_IP_PRI113_SHIFT                 0u
#define S32_NVIC_IP_PRI113_WIDTH                 8u
#define S32_NVIC_IP_PRI113(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI113_SHIFT))&S32_NVIC_IP_PRI113_MASK)
#define S32_NVIC_IP_PRI114_MASK                  0xFFu
#define S32_NVIC_IP_PRI114_SHIFT                 0u
#define S32_NVIC_IP_PRI114_WIDTH                 8u
#define S32_NVIC_IP_PRI114(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI114_SHIFT))&S32_NVIC_IP_PRI114_MASK)
#define S32_NVIC_IP_PRI115_MASK                  0xFFu
#define S32_NVIC_IP_PRI115_SHIFT                 0u
#define S32_NVIC_IP_PRI115_WIDTH                 8u
#define S32_NVIC_IP_PRI115(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI115_SHIFT))&S32_NVIC_IP_PRI115_MASK)
#define S32_NVIC_IP_PRI116_MASK                  0xFFu
#define S32_NVIC_IP_PRI116_SHIFT                 0u
#define S32_NVIC_IP_PRI116_WIDTH                 8u
#define S32_NVIC_IP_PRI116(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI116_SHIFT))&S32_NVIC_IP_PRI116_MASK)
#define S32_NVIC_IP_PRI117_MASK                  0xFFu
#define S32_NVIC_IP_PRI117_SHIFT                 0u
#define S32_NVIC_IP_PRI117_WIDTH                 8u
#define S32_NVIC_IP_PRI117(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI117_SHIFT))&S32_NVIC_IP_PRI117_MASK)
#define S32_NVIC_IP_PRI118_MASK                  0xFFu
#define S32_NVIC_IP_PRI118_SHIFT                 0u
#define S32_NVIC_IP_PRI118_WIDTH                 8u
#define S32_NVIC_IP_PRI118(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI118_SHIFT))&S32_NVIC_IP_PRI118_MASK)
#define S32_NVIC_IP_PRI119_MASK                  0xFFu
#define S32_NVIC_IP_PRI119_SHIFT                 0u
#define S32_NVIC_IP_PRI119_WIDTH                 8u
#define S32_NVIC_IP_PRI119(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI119_SHIFT))&S32_NVIC_IP_PRI119_MASK)
#define S32_NVIC_IP_PRI120_MASK                  0xFFu
#define S32_NVIC_IP_PRI120_SHIFT                 0u
#define S32_NVIC_IP_PRI120_WIDTH                 8u
#define S32_NVIC_IP_PRI120(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI120_SHIFT))&S32_NVIC_IP_PRI120_MASK)
#define S32_NVIC_IP_PRI121_MASK                  0xFFu
#define S32_NVIC_IP_PRI121_SHIFT                 0u
#define S32_NVIC_IP_PRI121_WIDTH                 8u
#define S32_NVIC_IP_PRI121(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI121_SHIFT))&S32_NVIC_IP_PRI121_MASK)
#define S32_NVIC_IP_PRI122_MASK                  0xFFu
#define S32_NVIC_IP_PRI122_SHIFT                 0u
#define S32_NVIC_IP_PRI122_WIDTH                 8u
#define S32_NVIC_IP_PRI122(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI122_SHIFT))&S32_NVIC_IP_PRI122_MASK)
#define S32_NVIC_IP_PRI123_MASK                  0xFFu
#define S32_NVIC_IP_PRI123_SHIFT                 0u
#define S32_NVIC_IP_PRI123_WIDTH                 8u
#define S32_NVIC_IP_PRI123(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI123_SHIFT))&S32_NVIC_IP_PRI123_MASK)
#define S32_NVIC_IP_PRI124_MASK                  0xFFu
#define S32_NVIC_IP_PRI124_SHIFT                 0u
#define S32_NVIC_IP_PRI124_WIDTH                 8u
#define S32_NVIC_IP_PRI124(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI124_SHIFT))&S32_NVIC_IP_PRI124_MASK)
#define S32_NVIC_IP_PRI125_MASK                  0xFFu
#define S32_NVIC_IP_PRI125_SHIFT                 0u
#define S32_NVIC_IP_PRI125_WIDTH                 8u
#define S32_NVIC_IP_PRI125(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI125_SHIFT))&S32_NVIC_IP_PRI125_MASK)
#define S32_NVIC_IP_PRI126_MASK                  0xFFu
#define S32_NVIC_IP_PRI126_SHIFT                 0u
#define S32_NVIC_IP_PRI126_WIDTH                 8u
#define S32_NVIC_IP_PRI126(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI126_SHIFT))&S32_NVIC_IP_PRI126_MASK)
#define S32_NVIC_IP_PRI127_MASK                  0xFFu
#define S32_NVIC_IP_PRI127_SHIFT                 0u
#define S32_NVIC_IP_PRI127_WIDTH                 8u
#define S32_NVIC_IP_PRI127(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI127_SHIFT))&S32_NVIC_IP_PRI127_MASK)
#define S32_NVIC_IP_PRI128_MASK                  0xFFu
#define S32_NVIC_IP_PRI128_SHIFT                 0u
#define S32_NVIC_IP_PRI128_WIDTH                 8u
#define S32_NVIC_IP_PRI128(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI128_SHIFT))&S32_NVIC_IP_PRI128_MASK)
#define S32_NVIC_IP_PRI129_MASK                  0xFFu
#define S32_NVIC_IP_PRI129_SHIFT                 0u
#define S32_NVIC_IP_PRI129_WIDTH                 8u
#define S32_NVIC_IP_PRI129(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI129_SHIFT))&S32_NVIC_IP_PRI129_MASK)
#define S32_NVIC_IP_PRI130_MASK                  0xFFu
#define S32_NVIC_IP_PRI130_SHIFT                 0u
#define S32_NVIC_IP_PRI130_WIDTH                 8u
#define S32_NVIC_IP_PRI130(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI130_SHIFT))&S32_NVIC_IP_PRI130_MASK)
#define S32_NVIC_IP_PRI131_MASK                  0xFFu
#define S32_NVIC_IP_PRI131_SHIFT                 0u
#define S32_NVIC_IP_PRI131_WIDTH                 8u
#define S32_NVIC_IP_PRI131(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI131_SHIFT))&S32_NVIC_IP_PRI131_MASK)
#define S32_NVIC_IP_PRI132_MASK                  0xFFu
#define S32_NVIC_IP_PRI132_SHIFT                 0u
#define S32_NVIC_IP_PRI132_WIDTH                 8u
#define S32_NVIC_IP_PRI132(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI132_SHIFT))&S32_NVIC_IP_PRI132_MASK)
#define S32_NVIC_IP_PRI133_MASK                  0xFFu
#define S32_NVIC_IP_PRI133_SHIFT                 0u
#define S32_NVIC_IP_PRI133_WIDTH                 8u
#define S32_NVIC_IP_PRI133(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI133_SHIFT))&S32_NVIC_IP_PRI133_MASK)
#define S32_NVIC_IP_PRI134_MASK                  0xFFu
#define S32_NVIC_IP_PRI134_SHIFT                 0u
#define S32_NVIC_IP_PRI134_WIDTH                 8u
#define S32_NVIC_IP_PRI134(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI134_SHIFT))&S32_NVIC_IP_PRI134_MASK)
#define S32_NVIC_IP_PRI135_MASK                  0xFFu
#define S32_NVIC_IP_PRI135_SHIFT                 0u
#define S32_NVIC_IP_PRI135_WIDTH                 8u
#define S32_NVIC_IP_PRI135(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI135_SHIFT))&S32_NVIC_IP_PRI135_MASK)
#define S32_NVIC_IP_PRI136_MASK                  0xFFu
#define S32_NVIC_IP_PRI136_SHIFT                 0u
#define S32_NVIC_IP_PRI136_WIDTH                 8u
#define S32_NVIC_IP_PRI136(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI136_SHIFT))&S32_NVIC_IP_PRI136_MASK)
#define S32_NVIC_IP_PRI137_MASK                  0xFFu
#define S32_NVIC_IP_PRI137_SHIFT                 0u
#define S32_NVIC_IP_PRI137_WIDTH                 8u
#define S32_NVIC_IP_PRI137(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI137_SHIFT))&S32_NVIC_IP_PRI137_MASK)
#define S32_NVIC_IP_PRI138_MASK                  0xFFu
#define S32_NVIC_IP_PRI138_SHIFT                 0u
#define S32_NVIC_IP_PRI138_WIDTH                 8u
#define S32_NVIC_IP_PRI138(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI138_SHIFT))&S32_NVIC_IP_PRI138_MASK)
#define S32_NVIC_IP_PRI139_MASK                  0xFFu
#define S32_NVIC_IP_PRI139_SHIFT                 0u
#define S32_NVIC_IP_PRI139_WIDTH                 8u
#define S32_NVIC_IP_PRI139(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI139_SHIFT))&S32_NVIC_IP_PRI139_MASK)
#define S32_NVIC_IP_PRI140_MASK                  0xFFu
#define S32_NVIC_IP_PRI140_SHIFT                 0u
#define S32_NVIC_IP_PRI140_WIDTH                 8u
#define S32_NVIC_IP_PRI140(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI140_SHIFT))&S32_NVIC_IP_PRI140_MASK)
#define S32_NVIC_IP_PRI141_MASK                  0xFFu
#define S32_NVIC_IP_PRI141_SHIFT                 0u
#define S32_NVIC_IP_PRI141_WIDTH                 8u
#define S32_NVIC_IP_PRI141(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI141_SHIFT))&S32_NVIC_IP_PRI141_MASK)
#define S32_NVIC_IP_PRI142_MASK                  0xFFu
#define S32_NVIC_IP_PRI142_SHIFT                 0u
#define S32_NVIC_IP_PRI142_WIDTH                 8u
#define S32_NVIC_IP_PRI142(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI142_SHIFT))&S32_NVIC_IP_PRI142_MASK)
#define S32_NVIC_IP_PRI143_MASK                  0xFFu
#define S32_NVIC_IP_PRI143_SHIFT                 0u
#define S32_NVIC_IP_PRI143_WIDTH                 8u
#define S32_NVIC_IP_PRI143(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI143_SHIFT))&S32_NVIC_IP_PRI143_MASK)
#define S32_NVIC_IP_PRI144_MASK                  0xFFu
#define S32_NVIC_IP_PRI144_SHIFT                 0u
#define S32_NVIC_IP_PRI144_WIDTH                 8u
#define S32_NVIC_IP_PRI144(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI144_SHIFT))&S32_NVIC_IP_PRI144_MASK)
#define S32_NVIC_IP_PRI145_MASK                  0xFFu
#define S32_NVIC_IP_PRI145_SHIFT                 0u
#define S32_NVIC_IP_PRI145_WIDTH                 8u
#define S32_NVIC_IP_PRI145(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI145_SHIFT))&S32_NVIC_IP_PRI145_MASK)
#define S32_NVIC_IP_PRI146_MASK                  0xFFu
#define S32_NVIC_IP_PRI146_SHIFT                 0u
#define S32_NVIC_IP_PRI146_WIDTH                 8u
#define S32_NVIC_IP_PRI146(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI146_SHIFT))&S32_NVIC_IP_PRI146_MASK)
#define S32_NVIC_IP_PRI147_MASK                  0xFFu
#define S32_NVIC_IP_PRI147_SHIFT                 0u
#define S32_NVIC_IP_PRI147_WIDTH                 8u
#define S32_NVIC_IP_PRI147(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI147_SHIFT))&S32_NVIC_IP_PRI147_MASK)
#define S32_NVIC_IP_PRI148_MASK                  0xFFu
#define S32_NVIC_IP_PRI148_SHIFT                 0u
#define S32_NVIC_IP_PRI148_WIDTH                 8u
#define S32_NVIC_IP_PRI148(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI148_SHIFT))&S32_NVIC_IP_PRI148_MASK)
#define S32_NVIC_IP_PRI149_MASK                  0xFFu
#define S32_NVIC_IP_PRI149_SHIFT                 0u
#define S32_NVIC_IP_PRI149_WIDTH                 8u
#define S32_NVIC_IP_PRI149(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI149_SHIFT))&S32_NVIC_IP_PRI149_MASK)
#define S32_NVIC_IP_PRI150_MASK                  0xFFu
#define S32_NVIC_IP_PRI150_SHIFT                 0u
#define S32_NVIC_IP_PRI150_WIDTH                 8u
#define S32_NVIC_IP_PRI150(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI150_SHIFT))&S32_NVIC_IP_PRI150_MASK)
#define S32_NVIC_IP_PRI151_MASK                  0xFFu
#define S32_NVIC_IP_PRI151_SHIFT                 0u
#define S32_NVIC_IP_PRI151_WIDTH                 8u
#define S32_NVIC_IP_PRI151(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI151_SHIFT))&S32_NVIC_IP_PRI151_MASK)
#define S32_NVIC_IP_PRI152_MASK                  0xFFu
#define S32_NVIC_IP_PRI152_SHIFT                 0u
#define S32_NVIC_IP_PRI152_WIDTH                 8u
#define S32_NVIC_IP_PRI152(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI152_SHIFT))&S32_NVIC_IP_PRI152_MASK)
#define S32_NVIC_IP_PRI153_MASK                  0xFFu
#define S32_NVIC_IP_PRI153_SHIFT                 0u
#define S32_NVIC_IP_PRI153_WIDTH                 8u
#define S32_NVIC_IP_PRI153(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI153_SHIFT))&S32_NVIC_IP_PRI153_MASK)
#define S32_NVIC_IP_PRI154_MASK                  0xFFu
#define S32_NVIC_IP_PRI154_SHIFT                 0u
#define S32_NVIC_IP_PRI154_WIDTH                 8u
#define S32_NVIC_IP_PRI154(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI154_SHIFT))&S32_NVIC_IP_PRI154_MASK)
#define S32_NVIC_IP_PRI155_MASK                  0xFFu
#define S32_NVIC_IP_PRI155_SHIFT                 0u
#define S32_NVIC_IP_PRI155_WIDTH                 8u
#define S32_NVIC_IP_PRI155(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI155_SHIFT))&S32_NVIC_IP_PRI155_MASK)
#define S32_NVIC_IP_PRI156_MASK                  0xFFu
#define S32_NVIC_IP_PRI156_SHIFT                 0u
#define S32_NVIC_IP_PRI156_WIDTH                 8u
#define S32_NVIC_IP_PRI156(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI156_SHIFT))&S32_NVIC_IP_PRI156_MASK)
#define S32_NVIC_IP_PRI157_MASK                  0xFFu
#define S32_NVIC_IP_PRI157_SHIFT                 0u
#define S32_NVIC_IP_PRI157_WIDTH                 8u
#define S32_NVIC_IP_PRI157(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI157_SHIFT))&S32_NVIC_IP_PRI157_MASK)
#define S32_NVIC_IP_PRI158_MASK                  0xFFu
#define S32_NVIC_IP_PRI158_SHIFT                 0u
#define S32_NVIC_IP_PRI158_WIDTH                 8u
#define S32_NVIC_IP_PRI158(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI158_SHIFT))&S32_NVIC_IP_PRI158_MASK)
#define S32_NVIC_IP_PRI159_MASK                  0xFFu
#define S32_NVIC_IP_PRI159_SHIFT                 0u
#define S32_NVIC_IP_PRI159_WIDTH                 8u
#define S32_NVIC_IP_PRI159(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI159_SHIFT))&S32_NVIC_IP_PRI159_MASK)
#define S32_NVIC_IP_PRI160_MASK                  0xFFu
#define S32_NVIC_IP_PRI160_SHIFT                 0u
#define S32_NVIC_IP_PRI160_WIDTH                 8u
#define S32_NVIC_IP_PRI160(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI160_SHIFT))&S32_NVIC_IP_PRI160_MASK)
#define S32_NVIC_IP_PRI161_MASK                  0xFFu
#define S32_NVIC_IP_PRI161_SHIFT                 0u
#define S32_NVIC_IP_PRI161_WIDTH                 8u
#define S32_NVIC_IP_PRI161(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI161_SHIFT))&S32_NVIC_IP_PRI161_MASK)
#define S32_NVIC_IP_PRI162_MASK                  0xFFu
#define S32_NVIC_IP_PRI162_SHIFT                 0u
#define S32_NVIC_IP_PRI162_WIDTH                 8u
#define S32_NVIC_IP_PRI162(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI162_SHIFT))&S32_NVIC_IP_PRI162_MASK)
#define S32_NVIC_IP_PRI163_MASK                  0xFFu
#define S32_NVIC_IP_PRI163_SHIFT                 0u
#define S32_NVIC_IP_PRI163_WIDTH                 8u
#define S32_NVIC_IP_PRI163(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI163_SHIFT))&S32_NVIC_IP_PRI163_MASK)
#define S32_NVIC_IP_PRI164_MASK                  0xFFu
#define S32_NVIC_IP_PRI164_SHIFT                 0u
#define S32_NVIC_IP_PRI164_WIDTH                 8u
#define S32_NVIC_IP_PRI164(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI164_SHIFT))&S32_NVIC_IP_PRI164_MASK)
#define S32_NVIC_IP_PRI165_MASK                  0xFFu
#define S32_NVIC_IP_PRI165_SHIFT                 0u
#define S32_NVIC_IP_PRI165_WIDTH                 8u
#define S32_NVIC_IP_PRI165(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI165_SHIFT))&S32_NVIC_IP_PRI165_MASK)
#define S32_NVIC_IP_PRI166_MASK                  0xFFu
#define S32_NVIC_IP_PRI166_SHIFT                 0u
#define S32_NVIC_IP_PRI166_WIDTH                 8u
#define S32_NVIC_IP_PRI166(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI166_SHIFT))&S32_NVIC_IP_PRI166_MASK)
#define S32_NVIC_IP_PRI167_MASK                  0xFFu
#define S32_NVIC_IP_PRI167_SHIFT                 0u
#define S32_NVIC_IP_PRI167_WIDTH                 8u
#define S32_NVIC_IP_PRI167(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI167_SHIFT))&S32_NVIC_IP_PRI167_MASK)
#define S32_NVIC_IP_PRI168_MASK                  0xFFu
#define S32_NVIC_IP_PRI168_SHIFT                 0u
#define S32_NVIC_IP_PRI168_WIDTH                 8u
#define S32_NVIC_IP_PRI168(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI168_SHIFT))&S32_NVIC_IP_PRI168_MASK)
#define S32_NVIC_IP_PRI169_MASK                  0xFFu
#define S32_NVIC_IP_PRI169_SHIFT                 0u
#define S32_NVIC_IP_PRI169_WIDTH                 8u
#define S32_NVIC_IP_PRI169(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI169_SHIFT))&S32_NVIC_IP_PRI169_MASK)
#define S32_NVIC_IP_PRI170_MASK                  0xFFu
#define S32_NVIC_IP_PRI170_SHIFT                 0u
#define S32_NVIC_IP_PRI170_WIDTH                 8u
#define S32_NVIC_IP_PRI170(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI170_SHIFT))&S32_NVIC_IP_PRI170_MASK)
#define S32_NVIC_IP_PRI171_MASK                  0xFFu
#define S32_NVIC_IP_PRI171_SHIFT                 0u
#define S32_NVIC_IP_PRI171_WIDTH                 8u
#define S32_NVIC_IP_PRI171(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI171_SHIFT))&S32_NVIC_IP_PRI171_MASK)
#define S32_NVIC_IP_PRI172_MASK                  0xFFu
#define S32_NVIC_IP_PRI172_SHIFT                 0u
#define S32_NVIC_IP_PRI172_WIDTH                 8u
#define S32_NVIC_IP_PRI172(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI172_SHIFT))&S32_NVIC_IP_PRI172_MASK)
#define S32_NVIC_IP_PRI173_MASK                  0xFFu
#define S32_NVIC_IP_PRI173_SHIFT                 0u
#define S32_NVIC_IP_PRI173_WIDTH                 8u
#define S32_NVIC_IP_PRI173(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI173_SHIFT))&S32_NVIC_IP_PRI173_MASK)
#define S32_NVIC_IP_PRI174_MASK                  0xFFu
#define S32_NVIC_IP_PRI174_SHIFT                 0u
#define S32_NVIC_IP_PRI174_WIDTH                 8u
#define S32_NVIC_IP_PRI174(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI174_SHIFT))&S32_NVIC_IP_PRI174_MASK)
#define S32_NVIC_IP_PRI175_MASK                  0xFFu
#define S32_NVIC_IP_PRI175_SHIFT                 0u
#define S32_NVIC_IP_PRI175_WIDTH                 8u
#define S32_NVIC_IP_PRI175(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI175_SHIFT))&S32_NVIC_IP_PRI175_MASK)
#define S32_NVIC_IP_PRI176_MASK                  0xFFu
#define S32_NVIC_IP_PRI176_SHIFT                 0u
#define S32_NVIC_IP_PRI176_WIDTH                 8u
#define S32_NVIC_IP_PRI176(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI176_SHIFT))&S32_NVIC_IP_PRI176_MASK)
#define S32_NVIC_IP_PRI177_MASK                  0xFFu
#define S32_NVIC_IP_PRI177_SHIFT                 0u
#define S32_NVIC_IP_PRI177_WIDTH                 8u
#define S32_NVIC_IP_PRI177(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI177_SHIFT))&S32_NVIC_IP_PRI177_MASK)
#define S32_NVIC_IP_PRI178_MASK                  0xFFu
#define S32_NVIC_IP_PRI178_SHIFT                 0u
#define S32_NVIC_IP_PRI178_WIDTH                 8u
#define S32_NVIC_IP_PRI178(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI178_SHIFT))&S32_NVIC_IP_PRI178_MASK)
#define S32_NVIC_IP_PRI179_MASK                  0xFFu
#define S32_NVIC_IP_PRI179_SHIFT                 0u
#define S32_NVIC_IP_PRI179_WIDTH                 8u
#define S32_NVIC_IP_PRI179(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI179_SHIFT))&S32_NVIC_IP_PRI179_MASK)
#define S32_NVIC_IP_PRI180_MASK                  0xFFu
#define S32_NVIC_IP_PRI180_SHIFT                 0u
#define S32_NVIC_IP_PRI180_WIDTH                 8u
#define S32_NVIC_IP_PRI180(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI180_SHIFT))&S32_NVIC_IP_PRI180_MASK)
#define S32_NVIC_IP_PRI181_MASK                  0xFFu
#define S32_NVIC_IP_PRI181_SHIFT                 0u
#define S32_NVIC_IP_PRI181_WIDTH                 8u
#define S32_NVIC_IP_PRI181(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI181_SHIFT))&S32_NVIC_IP_PRI181_MASK)
#define S32_NVIC_IP_PRI182_MASK                  0xFFu
#define S32_NVIC_IP_PRI182_SHIFT                 0u
#define S32_NVIC_IP_PRI182_WIDTH                 8u
#define S32_NVIC_IP_PRI182(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI182_SHIFT))&S32_NVIC_IP_PRI182_MASK)
#define S32_NVIC_IP_PRI183_MASK                  0xFFu
#define S32_NVIC_IP_PRI183_SHIFT                 0u
#define S32_NVIC_IP_PRI183_WIDTH                 8u
#define S32_NVIC_IP_PRI183(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI183_SHIFT))&S32_NVIC_IP_PRI183_MASK)
#define S32_NVIC_IP_PRI184_MASK                  0xFFu
#define S32_NVIC_IP_PRI184_SHIFT                 0u
#define S32_NVIC_IP_PRI184_WIDTH                 8u
#define S32_NVIC_IP_PRI184(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI184_SHIFT))&S32_NVIC_IP_PRI184_MASK)
#define S32_NVIC_IP_PRI185_MASK                  0xFFu
#define S32_NVIC_IP_PRI185_SHIFT                 0u
#define S32_NVIC_IP_PRI185_WIDTH                 8u
#define S32_NVIC_IP_PRI185(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI185_SHIFT))&S32_NVIC_IP_PRI185_MASK)
#define S32_NVIC_IP_PRI186_MASK                  0xFFu
#define S32_NVIC_IP_PRI186_SHIFT                 0u
#define S32_NVIC_IP_PRI186_WIDTH                 8u
#define S32_NVIC_IP_PRI186(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI186_SHIFT))&S32_NVIC_IP_PRI186_MASK)
#define S32_NVIC_IP_PRI187_MASK                  0xFFu
#define S32_NVIC_IP_PRI187_SHIFT                 0u
#define S32_NVIC_IP_PRI187_WIDTH                 8u
#define S32_NVIC_IP_PRI187(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI187_SHIFT))&S32_NVIC_IP_PRI187_MASK)
#define S32_NVIC_IP_PRI188_MASK                  0xFFu
#define S32_NVIC_IP_PRI188_SHIFT                 0u
#define S32_NVIC_IP_PRI188_WIDTH                 8u
#define S32_NVIC_IP_PRI188(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI188_SHIFT))&S32_NVIC_IP_PRI188_MASK)
#define S32_NVIC_IP_PRI189_MASK                  0xFFu
#define S32_NVIC_IP_PRI189_SHIFT                 0u
#define S32_NVIC_IP_PRI189_WIDTH                 8u
#define S32_NVIC_IP_PRI189(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI189_SHIFT))&S32_NVIC_IP_PRI189_MASK)
#define S32_NVIC_IP_PRI190_MASK                  0xFFu
#define S32_NVIC_IP_PRI190_SHIFT                 0u
#define S32_NVIC_IP_PRI190_WIDTH                 8u
#define S32_NVIC_IP_PRI190(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI190_SHIFT))&S32_NVIC_IP_PRI190_MASK)
#define S32_NVIC_IP_PRI191_MASK                  0xFFu
#define S32_NVIC_IP_PRI191_SHIFT                 0u
#define S32_NVIC_IP_PRI191_WIDTH                 8u
#define S32_NVIC_IP_PRI191(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI191_SHIFT))&S32_NVIC_IP_PRI191_MASK)
#define S32_NVIC_IP_PRI192_MASK                  0xFFu
#define S32_NVIC_IP_PRI192_SHIFT                 0u
#define S32_NVIC_IP_PRI192_WIDTH                 8u
#define S32_NVIC_IP_PRI192(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI192_SHIFT))&S32_NVIC_IP_PRI192_MASK)
#define S32_NVIC_IP_PRI193_MASK                  0xFFu
#define S32_NVIC_IP_PRI193_SHIFT                 0u
#define S32_NVIC_IP_PRI193_WIDTH                 8u
#define S32_NVIC_IP_PRI193(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI193_SHIFT))&S32_NVIC_IP_PRI193_MASK)
#define S32_NVIC_IP_PRI194_MASK                  0xFFu
#define S32_NVIC_IP_PRI194_SHIFT                 0u
#define S32_NVIC_IP_PRI194_WIDTH                 8u
#define S32_NVIC_IP_PRI194(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI194_SHIFT))&S32_NVIC_IP_PRI194_MASK)
#define S32_NVIC_IP_PRI195_MASK                  0xFFu
#define S32_NVIC_IP_PRI195_SHIFT                 0u
#define S32_NVIC_IP_PRI195_WIDTH                 8u
#define S32_NVIC_IP_PRI195(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI195_SHIFT))&S32_NVIC_IP_PRI195_MASK)
#define S32_NVIC_IP_PRI196_MASK                  0xFFu
#define S32_NVIC_IP_PRI196_SHIFT                 0u
#define S32_NVIC_IP_PRI196_WIDTH                 8u
#define S32_NVIC_IP_PRI196(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI196_SHIFT))&S32_NVIC_IP_PRI196_MASK)
#define S32_NVIC_IP_PRI197_MASK                  0xFFu
#define S32_NVIC_IP_PRI197_SHIFT                 0u
#define S32_NVIC_IP_PRI197_WIDTH                 8u
#define S32_NVIC_IP_PRI197(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI197_SHIFT))&S32_NVIC_IP_PRI197_MASK)
#define S32_NVIC_IP_PRI198_MASK                  0xFFu
#define S32_NVIC_IP_PRI198_SHIFT                 0u
#define S32_NVIC_IP_PRI198_WIDTH                 8u
#define S32_NVIC_IP_PRI198(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI198_SHIFT))&S32_NVIC_IP_PRI198_MASK)
#define S32_NVIC_IP_PRI199_MASK                  0xFFu
#define S32_NVIC_IP_PRI199_SHIFT                 0u
#define S32_NVIC_IP_PRI199_WIDTH                 8u
#define S32_NVIC_IP_PRI199(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI199_SHIFT))&S32_NVIC_IP_PRI199_MASK)
#define S32_NVIC_IP_PRI200_MASK                  0xFFu
#define S32_NVIC_IP_PRI200_SHIFT                 0u
#define S32_NVIC_IP_PRI200_WIDTH                 8u
#define S32_NVIC_IP_PRI200(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI200_SHIFT))&S32_NVIC_IP_PRI200_MASK)
#define S32_NVIC_IP_PRI201_MASK                  0xFFu
#define S32_NVIC_IP_PRI201_SHIFT                 0u
#define S32_NVIC_IP_PRI201_WIDTH                 8u
#define S32_NVIC_IP_PRI201(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI201_SHIFT))&S32_NVIC_IP_PRI201_MASK)
#define S32_NVIC_IP_PRI202_MASK                  0xFFu
#define S32_NVIC_IP_PRI202_SHIFT                 0u
#define S32_NVIC_IP_PRI202_WIDTH                 8u
#define S32_NVIC_IP_PRI202(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI202_SHIFT))&S32_NVIC_IP_PRI202_MASK)
#define S32_NVIC_IP_PRI203_MASK                  0xFFu
#define S32_NVIC_IP_PRI203_SHIFT                 0u
#define S32_NVIC_IP_PRI203_WIDTH                 8u
#define S32_NVIC_IP_PRI203(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI203_SHIFT))&S32_NVIC_IP_PRI203_MASK)
#define S32_NVIC_IP_PRI204_MASK                  0xFFu
#define S32_NVIC_IP_PRI204_SHIFT                 0u
#define S32_NVIC_IP_PRI204_WIDTH                 8u
#define S32_NVIC_IP_PRI204(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI204_SHIFT))&S32_NVIC_IP_PRI204_MASK)
#define S32_NVIC_IP_PRI205_MASK                  0xFFu
#define S32_NVIC_IP_PRI205_SHIFT                 0u
#define S32_NVIC_IP_PRI205_WIDTH                 8u
#define S32_NVIC_IP_PRI205(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI205_SHIFT))&S32_NVIC_IP_PRI205_MASK)
#define S32_NVIC_IP_PRI206_MASK                  0xFFu
#define S32_NVIC_IP_PRI206_SHIFT                 0u
#define S32_NVIC_IP_PRI206_WIDTH                 8u
#define S32_NVIC_IP_PRI206(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI206_SHIFT))&S32_NVIC_IP_PRI206_MASK)
#define S32_NVIC_IP_PRI207_MASK                  0xFFu
#define S32_NVIC_IP_PRI207_SHIFT                 0u
#define S32_NVIC_IP_PRI207_WIDTH                 8u
#define S32_NVIC_IP_PRI207(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI207_SHIFT))&S32_NVIC_IP_PRI207_MASK)
#define S32_NVIC_IP_PRI208_MASK                  0xFFu
#define S32_NVIC_IP_PRI208_SHIFT                 0u
#define S32_NVIC_IP_PRI208_WIDTH                 8u
#define S32_NVIC_IP_PRI208(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI208_SHIFT))&S32_NVIC_IP_PRI208_MASK)
#define S32_NVIC_IP_PRI209_MASK                  0xFFu
#define S32_NVIC_IP_PRI209_SHIFT                 0u
#define S32_NVIC_IP_PRI209_WIDTH                 8u
#define S32_NVIC_IP_PRI209(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI209_SHIFT))&S32_NVIC_IP_PRI209_MASK)
#define S32_NVIC_IP_PRI210_MASK                  0xFFu
#define S32_NVIC_IP_PRI210_SHIFT                 0u
#define S32_NVIC_IP_PRI210_WIDTH                 8u
#define S32_NVIC_IP_PRI210(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI210_SHIFT))&S32_NVIC_IP_PRI210_MASK)
#define S32_NVIC_IP_PRI211_MASK                  0xFFu
#define S32_NVIC_IP_PRI211_SHIFT                 0u
#define S32_NVIC_IP_PRI211_WIDTH                 8u
#define S32_NVIC_IP_PRI211(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI211_SHIFT))&S32_NVIC_IP_PRI211_MASK)
#define S32_NVIC_IP_PRI212_MASK                  0xFFu
#define S32_NVIC_IP_PRI212_SHIFT                 0u
#define S32_NVIC_IP_PRI212_WIDTH                 8u
#define S32_NVIC_IP_PRI212(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI212_SHIFT))&S32_NVIC_IP_PRI212_MASK)
#define S32_NVIC_IP_PRI213_MASK                  0xFFu
#define S32_NVIC_IP_PRI213_SHIFT                 0u
#define S32_NVIC_IP_PRI213_WIDTH                 8u
#define S32_NVIC_IP_PRI213(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI213_SHIFT))&S32_NVIC_IP_PRI213_MASK)
#define S32_NVIC_IP_PRI214_MASK                  0xFFu
#define S32_NVIC_IP_PRI214_SHIFT                 0u
#define S32_NVIC_IP_PRI214_WIDTH                 8u
#define S32_NVIC_IP_PRI214(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI214_SHIFT))&S32_NVIC_IP_PRI214_MASK)
#define S32_NVIC_IP_PRI215_MASK                  0xFFu
#define S32_NVIC_IP_PRI215_SHIFT                 0u
#define S32_NVIC_IP_PRI215_WIDTH                 8u
#define S32_NVIC_IP_PRI215(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI215_SHIFT))&S32_NVIC_IP_PRI215_MASK)
#define S32_NVIC_IP_PRI216_MASK                  0xFFu
#define S32_NVIC_IP_PRI216_SHIFT                 0u
#define S32_NVIC_IP_PRI216_WIDTH                 8u
#define S32_NVIC_IP_PRI216(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI216_SHIFT))&S32_NVIC_IP_PRI216_MASK)
#define S32_NVIC_IP_PRI217_MASK                  0xFFu
#define S32_NVIC_IP_PRI217_SHIFT                 0u
#define S32_NVIC_IP_PRI217_WIDTH                 8u
#define S32_NVIC_IP_PRI217(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI217_SHIFT))&S32_NVIC_IP_PRI217_MASK)
#define S32_NVIC_IP_PRI218_MASK                  0xFFu
#define S32_NVIC_IP_PRI218_SHIFT                 0u
#define S32_NVIC_IP_PRI218_WIDTH                 8u
#define S32_NVIC_IP_PRI218(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI218_SHIFT))&S32_NVIC_IP_PRI218_MASK)
#define S32_NVIC_IP_PRI219_MASK                  0xFFu
#define S32_NVIC_IP_PRI219_SHIFT                 0u
#define S32_NVIC_IP_PRI219_WIDTH                 8u
#define S32_NVIC_IP_PRI219(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI219_SHIFT))&S32_NVIC_IP_PRI219_MASK)
#define S32_NVIC_IP_PRI220_MASK                  0xFFu
#define S32_NVIC_IP_PRI220_SHIFT                 0u
#define S32_NVIC_IP_PRI220_WIDTH                 8u
#define S32_NVIC_IP_PRI220(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI220_SHIFT))&S32_NVIC_IP_PRI220_MASK)
#define S32_NVIC_IP_PRI221_MASK                  0xFFu
#define S32_NVIC_IP_PRI221_SHIFT                 0u
#define S32_NVIC_IP_PRI221_WIDTH                 8u
#define S32_NVIC_IP_PRI221(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI221_SHIFT))&S32_NVIC_IP_PRI221_MASK)
#define S32_NVIC_IP_PRI222_MASK                  0xFFu
#define S32_NVIC_IP_PRI222_SHIFT                 0u
#define S32_NVIC_IP_PRI222_WIDTH                 8u
#define S32_NVIC_IP_PRI222(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI222_SHIFT))&S32_NVIC_IP_PRI222_MASK)
#define S32_NVIC_IP_PRI223_MASK                  0xFFu
#define S32_NVIC_IP_PRI223_SHIFT                 0u
#define S32_NVIC_IP_PRI223_WIDTH                 8u
#define S32_NVIC_IP_PRI223(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI223_SHIFT))&S32_NVIC_IP_PRI223_MASK)
#define S32_NVIC_IP_PRI224_MASK                  0xFFu
#define S32_NVIC_IP_PRI224_SHIFT                 0u
#define S32_NVIC_IP_PRI224_WIDTH                 8u
#define S32_NVIC_IP_PRI224(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI224_SHIFT))&S32_NVIC_IP_PRI224_MASK)
#define S32_NVIC_IP_PRI225_MASK                  0xFFu
#define S32_NVIC_IP_PRI225_SHIFT                 0u
#define S32_NVIC_IP_PRI225_WIDTH                 8u
#define S32_NVIC_IP_PRI225(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI225_SHIFT))&S32_NVIC_IP_PRI225_MASK)
#define S32_NVIC_IP_PRI226_MASK                  0xFFu
#define S32_NVIC_IP_PRI226_SHIFT                 0u
#define S32_NVIC_IP_PRI226_WIDTH                 8u
#define S32_NVIC_IP_PRI226(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI226_SHIFT))&S32_NVIC_IP_PRI226_MASK)
#define S32_NVIC_IP_PRI227_MASK                  0xFFu
#define S32_NVIC_IP_PRI227_SHIFT                 0u
#define S32_NVIC_IP_PRI227_WIDTH                 8u
#define S32_NVIC_IP_PRI227(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI227_SHIFT))&S32_NVIC_IP_PRI227_MASK)
#define S32_NVIC_IP_PRI228_MASK                  0xFFu
#define S32_NVIC_IP_PRI228_SHIFT                 0u
#define S32_NVIC_IP_PRI228_WIDTH                 8u
#define S32_NVIC_IP_PRI228(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI228_SHIFT))&S32_NVIC_IP_PRI228_MASK)
#define S32_NVIC_IP_PRI229_MASK                  0xFFu
#define S32_NVIC_IP_PRI229_SHIFT                 0u
#define S32_NVIC_IP_PRI229_WIDTH                 8u
#define S32_NVIC_IP_PRI229(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI229_SHIFT))&S32_NVIC_IP_PRI229_MASK)
#define S32_NVIC_IP_PRI230_MASK                  0xFFu
#define S32_NVIC_IP_PRI230_SHIFT                 0u
#define S32_NVIC_IP_PRI230_WIDTH                 8u
#define S32_NVIC_IP_PRI230(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI230_SHIFT))&S32_NVIC_IP_PRI230_MASK)
#define S32_NVIC_IP_PRI231_MASK                  0xFFu
#define S32_NVIC_IP_PRI231_SHIFT                 0u
#define S32_NVIC_IP_PRI231_WIDTH                 8u
#define S32_NVIC_IP_PRI231(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI231_SHIFT))&S32_NVIC_IP_PRI231_MASK)
#define S32_NVIC_IP_PRI232_MASK                  0xFFu
#define S32_NVIC_IP_PRI232_SHIFT                 0u
#define S32_NVIC_IP_PRI232_WIDTH                 8u
#define S32_NVIC_IP_PRI232(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI232_SHIFT))&S32_NVIC_IP_PRI232_MASK)
#define S32_NVIC_IP_PRI233_MASK                  0xFFu
#define S32_NVIC_IP_PRI233_SHIFT                 0u
#define S32_NVIC_IP_PRI233_WIDTH                 8u
#define S32_NVIC_IP_PRI233(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI233_SHIFT))&S32_NVIC_IP_PRI233_MASK)
#define S32_NVIC_IP_PRI234_MASK                  0xFFu
#define S32_NVIC_IP_PRI234_SHIFT                 0u
#define S32_NVIC_IP_PRI234_WIDTH                 8u
#define S32_NVIC_IP_PRI234(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI234_SHIFT))&S32_NVIC_IP_PRI234_MASK)
#define S32_NVIC_IP_PRI235_MASK                  0xFFu
#define S32_NVIC_IP_PRI235_SHIFT                 0u
#define S32_NVIC_IP_PRI235_WIDTH                 8u
#define S32_NVIC_IP_PRI235(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI235_SHIFT))&S32_NVIC_IP_PRI235_MASK)
#define S32_NVIC_IP_PRI236_MASK                  0xFFu
#define S32_NVIC_IP_PRI236_SHIFT                 0u
#define S32_NVIC_IP_PRI236_WIDTH                 8u
#define S32_NVIC_IP_PRI236(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI236_SHIFT))&S32_NVIC_IP_PRI236_MASK)
#define S32_NVIC_IP_PRI237_MASK                  0xFFu
#define S32_NVIC_IP_PRI237_SHIFT                 0u
#define S32_NVIC_IP_PRI237_WIDTH                 8u
#define S32_NVIC_IP_PRI237(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI237_SHIFT))&S32_NVIC_IP_PRI237_MASK)
#define S32_NVIC_IP_PRI238_MASK                  0xFFu
#define S32_NVIC_IP_PRI238_SHIFT                 0u
#define S32_NVIC_IP_PRI238_WIDTH                 8u
#define S32_NVIC_IP_PRI238(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI238_SHIFT))&S32_NVIC_IP_PRI238_MASK)
#define S32_NVIC_IP_PRI239_MASK                  0xFFu
#define S32_NVIC_IP_PRI239_SHIFT                 0u
#define S32_NVIC_IP_PRI239_WIDTH                 8u
#define S32_NVIC_IP_PRI239(x)                    (((uint8_t)(((uint8_t)(x))<<S32_NVIC_IP_PRI239_SHIFT))&S32_NVIC_IP_PRI239_MASK)
/* STIR Bit Fields */
#define S32_NVIC_STIR_INTID_MASK                 0x1FFu
#define S32_NVIC_STIR_INTID_SHIFT                0u
#define S32_NVIC_STIR_INTID_WIDTH                9u
#define S32_NVIC_STIR_INTID(x)                   (((uint32_t)(((uint32_t)(x))<<S32_NVIC_STIR_INTID_SHIFT))&S32_NVIC_STIR_INTID_MASK)

/*!
 * @}
 */ /* end of group S32_NVIC_Register_Masks */


/*!
 * @}
 */ /* end of group S32_NVIC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- ADC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Peripheral_Access_Layer ADC Peripheral Access Layer
 * @{
 */

/** ADC - Size of Registers Arrays */
#define ADC_SC1_COUNT                             16u
#define ADC_R_COUNT                               16u
#define ADC_CV_COUNT                              2u

/** ADC - Register Layout Typedef */
typedef struct {
  __IO uint32_t SC1[ADC_SC1_COUNT];                /**< ADC Status and Control Register 1, array offset: 0x0, array step: 0x4 */
  __IO uint32_t CFG1;                              /**< ADC Configuration Register 1, offset: 0x40 */
  __IO uint32_t CFG2;                              /**< ADC Configuration Register 2, offset: 0x44 */
  __I  uint32_t R[ADC_R_COUNT];                    /**< ADC Data Result Registers, array offset: 0x48, array step: 0x4 */
  __IO uint32_t CV[ADC_CV_COUNT];                  /**< Compare Value Registers, array offset: 0x88, array step: 0x4 */
  __IO uint32_t SC2;                               /**< Status and Control Register 2, offset: 0x90 */
  __IO uint32_t SC3;                               /**< Status and Control Register 3, offset: 0x94 */
  __IO uint32_t BASE_OFS;                          /**< BASE Offset Register, offset: 0x98 */
  __IO uint32_t OFS;                               /**< ADC Offset Correction Register, offset: 0x9C */
  __IO uint32_t USR_OFS;                           /**< USER Offset Correction Register, offset: 0xA0 */
  __IO uint32_t XOFS;                              /**< ADC X Offset Correction Register, offset: 0xA4 */
  __IO uint32_t YOFS;                              /**< ADC Y Offset Correction Register, offset: 0xA8 */
  __IO uint32_t G;                                 /**< ADC Gain Register, offset: 0xAC */
  __IO uint32_t UG;                                /**< ADC User Gain Register, offset: 0xB0 */
  __IO uint32_t CLPS;                              /**< ADC General Calibration Value Register S, offset: 0xB4 */
  __IO uint32_t CLP3;                              /**< ADC Plus-Side General Calibration Value Register 3, offset: 0xB8 */
  __IO uint32_t CLP2;                              /**< ADC Plus-Side General Calibration Value Register 2, offset: 0xBC */
  __IO uint32_t CLP1;                              /**< ADC Plus-Side General Calibration Value Register 1, offset: 0xC0 */
  __IO uint32_t CLP0;                              /**< ADC Plus-Side General Calibration Value Register 0, offset: 0xC4 */
  __IO uint32_t CLPX;                              /**< ADC Plus-Side General Calibration Value Register X, offset: 0xC8 */
  __IO uint32_t CLP9;                              /**< ADC Plus-Side General Calibration Value Register 9, offset: 0xCC */
  __IO uint32_t CLPS_OFS;                          /**< ADC General Calibration Offset Value Register S, offset: 0xD0 */
  __IO uint32_t CLP3_OFS;                          /**< ADC Plus-Side General Calibration Offset Value Register 3, offset: 0xD4 */
  __IO uint32_t CLP2_OFS;                          /**< ADC Plus-Side General Calibration Offset Value Register 2, offset: 0xD8 */
  __IO uint32_t CLP1_OFS;                          /**< ADC Plus-Side General Calibration Offset Value Register 1, offset: 0xDC */
  __IO uint32_t CLP0_OFS;                          /**< ADC Plus-Side General Calibration Offset Value Register 0, offset: 0xE0 */
  __IO uint32_t CLPX_OFS;                          /**< ADC Plus-Side General Calibration Offset Value Register X, offset: 0xE4 */
  __IO uint32_t CLP9_OFS;                          /**< ADC Plus-Side General Calibration Offset Value Register 9, offset: 0xE8 */
} ADC_Type, *ADC_MemMapPtr;

/** Number of instances of the ADC module. */
#define ADC_INSTANCE_COUNT                       (2u)

/* ADC - Peripheral instance base addresses */
/** Peripheral ADC0 base address */
#define ADC0_BASE                                (0x4003B000u)
/** Peripheral ADC0 base pointer */
#define ADC0                                     ((ADC_Type *)ADC0_BASE)
/** Peripheral ADC1 base address */
#define ADC1_BASE                                (0x40027000u)
/** Peripheral ADC1 base pointer */
#define ADC1                                     ((ADC_Type *)ADC1_BASE)
/** Array initializer of ADC peripheral base addresses */
#define ADC_BASE_ADDRS                           { ADC0_BASE, ADC1_BASE }
/** Array initializer of ADC peripheral base pointers */
#define ADC_BASE_PTRS                            { ADC0, ADC1 }

/* ----------------------------------------------------------------------------
   -- ADC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Register_Masks ADC Register Masks
 * @{
 */

/*! @name SC1 - ADC Status and Control Register 1 */
/*! @{ */
#define ADC_SC1_ADCH_MASK                        (0x1FU)
#define ADC_SC1_ADCH_SHIFT                       (0U)
#define ADC_SC1_ADCH_WIDTH                       (5U)
#define ADC_SC1_ADCH(x)                          (((uint32_t)(((uint32_t)(x)) << ADC_SC1_ADCH_SHIFT)) & ADC_SC1_ADCH_MASK)
#define ADC_SC1_AIEN_MASK                        (0x40U)
#define ADC_SC1_AIEN_SHIFT                       (6U)
#define ADC_SC1_AIEN_WIDTH                       (1U)
#define ADC_SC1_AIEN(x)                          (((uint32_t)(((uint32_t)(x)) << ADC_SC1_AIEN_SHIFT)) & ADC_SC1_AIEN_MASK)
#define ADC_SC1_COCO_MASK                        (0x80U)
#define ADC_SC1_COCO_SHIFT                       (7U)
#define ADC_SC1_COCO_WIDTH                       (1U)
#define ADC_SC1_COCO(x)                          (((uint32_t)(((uint32_t)(x)) << ADC_SC1_COCO_SHIFT)) & ADC_SC1_COCO_MASK)
/*! @} */

/*! @name CFG1 - ADC Configuration Register 1 */
/*! @{ */
#define ADC_CFG1_ADICLK_MASK                     (0x3U)
#define ADC_CFG1_ADICLK_SHIFT                    (0U)
#define ADC_CFG1_ADICLK_WIDTH                    (2U)
#define ADC_CFG1_ADICLK(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_CFG1_ADICLK_SHIFT)) & ADC_CFG1_ADICLK_MASK)
#define ADC_CFG1_MODE_MASK                       (0xCU)
#define ADC_CFG1_MODE_SHIFT                      (2U)
#define ADC_CFG1_MODE_WIDTH                      (2U)
#define ADC_CFG1_MODE(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CFG1_MODE_SHIFT)) & ADC_CFG1_MODE_MASK)
#define ADC_CFG1_ADIV_MASK                       (0x60U)
#define ADC_CFG1_ADIV_SHIFT                      (5U)
#define ADC_CFG1_ADIV_WIDTH                      (2U)
#define ADC_CFG1_ADIV(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CFG1_ADIV_SHIFT)) & ADC_CFG1_ADIV_MASK)
#define ADC_CFG1_CLRLTRG_MASK                    (0x100U)
#define ADC_CFG1_CLRLTRG_SHIFT                   (8U)
#define ADC_CFG1_CLRLTRG_WIDTH                   (1U)
#define ADC_CFG1_CLRLTRG(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_CFG1_CLRLTRG_SHIFT)) & ADC_CFG1_CLRLTRG_MASK)
/*! @} */

/*! @name CFG2 - ADC Configuration Register 2 */
/*! @{ */
#define ADC_CFG2_SMPLTS_MASK                     (0xFFU)
#define ADC_CFG2_SMPLTS_SHIFT                    (0U)
#define ADC_CFG2_SMPLTS_WIDTH                    (8U)
#define ADC_CFG2_SMPLTS(x)                       (((uint32_t)(((uint32_t)(x)) << ADC_CFG2_SMPLTS_SHIFT)) & ADC_CFG2_SMPLTS_MASK)
/*! @} */

/*! @name R - ADC Data Result Registers */
/*! @{ */
#define ADC_R_D_MASK                             (0xFFFU)
#define ADC_R_D_SHIFT                            (0U)
#define ADC_R_D_WIDTH                            (12U)
#define ADC_R_D(x)                               (((uint32_t)(((uint32_t)(x)) << ADC_R_D_SHIFT)) & ADC_R_D_MASK)
/*! @} */

/*! @name CV - Compare Value Registers */
/*! @{ */
#define ADC_CV_CV_MASK                           (0xFFFFU)
#define ADC_CV_CV_SHIFT                          (0U)
#define ADC_CV_CV_WIDTH                          (16U)
#define ADC_CV_CV(x)                             (((uint32_t)(((uint32_t)(x)) << ADC_CV_CV_SHIFT)) & ADC_CV_CV_MASK)
/*! @} */

/*! @name SC2 - Status and Control Register 2 */
/*! @{ */
#define ADC_SC2_REFSEL_MASK                      (0x3U)
#define ADC_SC2_REFSEL_SHIFT                     (0U)
#define ADC_SC2_REFSEL_WIDTH                     (2U)
#define ADC_SC2_REFSEL(x)                        (((uint32_t)(((uint32_t)(x)) << ADC_SC2_REFSEL_SHIFT)) & ADC_SC2_REFSEL_MASK)
#define ADC_SC2_DMAEN_MASK                       (0x4U)
#define ADC_SC2_DMAEN_SHIFT                      (2U)
#define ADC_SC2_DMAEN_WIDTH                      (1U)
#define ADC_SC2_DMAEN(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_SC2_DMAEN_SHIFT)) & ADC_SC2_DMAEN_MASK)
#define ADC_SC2_ACREN_MASK                       (0x8U)
#define ADC_SC2_ACREN_SHIFT                      (3U)
#define ADC_SC2_ACREN_WIDTH                      (1U)
#define ADC_SC2_ACREN(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_SC2_ACREN_SHIFT)) & ADC_SC2_ACREN_MASK)
#define ADC_SC2_ACFGT_MASK                       (0x10U)
#define ADC_SC2_ACFGT_SHIFT                      (4U)
#define ADC_SC2_ACFGT_WIDTH                      (1U)
#define ADC_SC2_ACFGT(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_SC2_ACFGT_SHIFT)) & ADC_SC2_ACFGT_MASK)
#define ADC_SC2_ACFE_MASK                        (0x20U)
#define ADC_SC2_ACFE_SHIFT                       (5U)
#define ADC_SC2_ACFE_WIDTH                       (1U)
#define ADC_SC2_ACFE(x)                          (((uint32_t)(((uint32_t)(x)) << ADC_SC2_ACFE_SHIFT)) & ADC_SC2_ACFE_MASK)
#define ADC_SC2_ADTRG_MASK                       (0x40U)
#define ADC_SC2_ADTRG_SHIFT                      (6U)
#define ADC_SC2_ADTRG_WIDTH                      (1U)
#define ADC_SC2_ADTRG(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_SC2_ADTRG_SHIFT)) & ADC_SC2_ADTRG_MASK)
#define ADC_SC2_ADACT_MASK                       (0x80U)
#define ADC_SC2_ADACT_SHIFT                      (7U)
#define ADC_SC2_ADACT_WIDTH                      (1U)
#define ADC_SC2_ADACT(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_SC2_ADACT_SHIFT)) & ADC_SC2_ADACT_MASK)
#define ADC_SC2_TRGPRNUM_MASK                    (0x6000U)
#define ADC_SC2_TRGPRNUM_SHIFT                   (13U)
#define ADC_SC2_TRGPRNUM_WIDTH                   (2U)
#define ADC_SC2_TRGPRNUM(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_SC2_TRGPRNUM_SHIFT)) & ADC_SC2_TRGPRNUM_MASK)
#define ADC_SC2_TRGSTLAT_MASK                    (0xF0000U)
#define ADC_SC2_TRGSTLAT_SHIFT                   (16U)
#define ADC_SC2_TRGSTLAT_WIDTH                   (4U)
#define ADC_SC2_TRGSTLAT(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_SC2_TRGSTLAT_SHIFT)) & ADC_SC2_TRGSTLAT_MASK)
#define ADC_SC2_TRGSTERR_MASK                    (0xF000000U)
#define ADC_SC2_TRGSTERR_SHIFT                   (24U)
#define ADC_SC2_TRGSTERR_WIDTH                   (4U)
#define ADC_SC2_TRGSTERR(x)                      (((uint32_t)(((uint32_t)(x)) << ADC_SC2_TRGSTERR_SHIFT)) & ADC_SC2_TRGSTERR_MASK)
/*! @} */

/*! @name SC3 - Status and Control Register 3 */
/*! @{ */
#define ADC_SC3_AVGS_MASK                        (0x3U)
#define ADC_SC3_AVGS_SHIFT                       (0U)
#define ADC_SC3_AVGS_WIDTH                       (2U)
#define ADC_SC3_AVGS(x)                          (((uint32_t)(((uint32_t)(x)) << ADC_SC3_AVGS_SHIFT)) & ADC_SC3_AVGS_MASK)
#define ADC_SC3_AVGE_MASK                        (0x4U)
#define ADC_SC3_AVGE_SHIFT                       (2U)
#define ADC_SC3_AVGE_WIDTH                       (1U)
#define ADC_SC3_AVGE(x)                          (((uint32_t)(((uint32_t)(x)) << ADC_SC3_AVGE_SHIFT)) & ADC_SC3_AVGE_MASK)
#define ADC_SC3_ADCO_MASK                        (0x8U)
#define ADC_SC3_ADCO_SHIFT                       (3U)
#define ADC_SC3_ADCO_WIDTH                       (1U)
#define ADC_SC3_ADCO(x)                          (((uint32_t)(((uint32_t)(x)) << ADC_SC3_ADCO_SHIFT)) & ADC_SC3_ADCO_MASK)
#define ADC_SC3_CAL_MASK                         (0x80U)
#define ADC_SC3_CAL_SHIFT                        (7U)
#define ADC_SC3_CAL_WIDTH                        (1U)
#define ADC_SC3_CAL(x)                           (((uint32_t)(((uint32_t)(x)) << ADC_SC3_CAL_SHIFT)) & ADC_SC3_CAL_MASK)
/*! @} */

/*! @name BASE_OFS - BASE Offset Register */
/*! @{ */
#define ADC_BASE_OFS_BA_OFS_MASK                 (0xFFU)
#define ADC_BASE_OFS_BA_OFS_SHIFT                (0U)
#define ADC_BASE_OFS_BA_OFS_WIDTH                (8U)
#define ADC_BASE_OFS_BA_OFS(x)                   (((uint32_t)(((uint32_t)(x)) << ADC_BASE_OFS_BA_OFS_SHIFT)) & ADC_BASE_OFS_BA_OFS_MASK)
/*! @} */

/*! @name OFS - ADC Offset Correction Register */
/*! @{ */
#define ADC_OFS_OFS_MASK                         (0xFFFFU)
#define ADC_OFS_OFS_SHIFT                        (0U)
#define ADC_OFS_OFS_WIDTH                        (16U)
#define ADC_OFS_OFS(x)                           (((uint32_t)(((uint32_t)(x)) << ADC_OFS_OFS_SHIFT)) & ADC_OFS_OFS_MASK)
/*! @} */

/*! @name USR_OFS - USER Offset Correction Register */
/*! @{ */
#define ADC_USR_OFS_USR_OFS_MASK                 (0xFFU)
#define ADC_USR_OFS_USR_OFS_SHIFT                (0U)
#define ADC_USR_OFS_USR_OFS_WIDTH                (8U)
#define ADC_USR_OFS_USR_OFS(x)                   (((uint32_t)(((uint32_t)(x)) << ADC_USR_OFS_USR_OFS_SHIFT)) & ADC_USR_OFS_USR_OFS_MASK)
/*! @} */

/*! @name XOFS - ADC X Offset Correction Register */
/*! @{ */
#define ADC_XOFS_XOFS_MASK                       (0x3FU)
#define ADC_XOFS_XOFS_SHIFT                      (0U)
#define ADC_XOFS_XOFS_WIDTH                      (6U)
#define ADC_XOFS_XOFS(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_XOFS_XOFS_SHIFT)) & ADC_XOFS_XOFS_MASK)
/*! @} */

/*! @name YOFS - ADC Y Offset Correction Register */
/*! @{ */
#define ADC_YOFS_YOFS_MASK                       (0xFFU)
#define ADC_YOFS_YOFS_SHIFT                      (0U)
#define ADC_YOFS_YOFS_WIDTH                      (8U)
#define ADC_YOFS_YOFS(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_YOFS_YOFS_SHIFT)) & ADC_YOFS_YOFS_MASK)
/*! @} */

/*! @name G - ADC Gain Register */
/*! @{ */
#define ADC_G_G_MASK                             (0x7FFU)
#define ADC_G_G_SHIFT                            (0U)
#define ADC_G_G_WIDTH                            (11U)
#define ADC_G_G(x)                               (((uint32_t)(((uint32_t)(x)) << ADC_G_G_SHIFT)) & ADC_G_G_MASK)
/*! @} */

/*! @name UG - ADC User Gain Register */
/*! @{ */
#define ADC_UG_UG_MASK                           (0x3FFU)
#define ADC_UG_UG_SHIFT                          (0U)
#define ADC_UG_UG_WIDTH                          (10U)
#define ADC_UG_UG(x)                             (((uint32_t)(((uint32_t)(x)) << ADC_UG_UG_SHIFT)) & ADC_UG_UG_MASK)
/*! @} */

/*! @name CLPS - ADC General Calibration Value Register S */
/*! @{ */
#define ADC_CLPS_CLPS_MASK                       (0x7FU)
#define ADC_CLPS_CLPS_SHIFT                      (0U)
#define ADC_CLPS_CLPS_WIDTH                      (7U)
#define ADC_CLPS_CLPS(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLPS_CLPS_SHIFT)) & ADC_CLPS_CLPS_MASK)
/*! @} */

/*! @name CLP3 - ADC Plus-Side General Calibration Value Register 3 */
/*! @{ */
#define ADC_CLP3_CLP3_MASK                       (0x3FFU)
#define ADC_CLP3_CLP3_SHIFT                      (0U)
#define ADC_CLP3_CLP3_WIDTH                      (10U)
#define ADC_CLP3_CLP3(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLP3_CLP3_SHIFT)) & ADC_CLP3_CLP3_MASK)
/*! @} */

/*! @name CLP2 - ADC Plus-Side General Calibration Value Register 2 */
/*! @{ */
#define ADC_CLP2_CLP2_MASK                       (0x3FFU)
#define ADC_CLP2_CLP2_SHIFT                      (0U)
#define ADC_CLP2_CLP2_WIDTH                      (10U)
#define ADC_CLP2_CLP2(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLP2_CLP2_SHIFT)) & ADC_CLP2_CLP2_MASK)
/*! @} */

/*! @name CLP1 - ADC Plus-Side General Calibration Value Register 1 */
/*! @{ */
#define ADC_CLP1_CLP1_MASK                       (0x1FFU)
#define ADC_CLP1_CLP1_SHIFT                      (0U)
#define ADC_CLP1_CLP1_WIDTH                      (9U)
#define ADC_CLP1_CLP1(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLP1_CLP1_SHIFT)) & ADC_CLP1_CLP1_MASK)
/*! @} */

/*! @name CLP0 - ADC Plus-Side General Calibration Value Register 0 */
/*! @{ */
#define ADC_CLP0_CLP0_MASK                       (0xFFU)
#define ADC_CLP0_CLP0_SHIFT                      (0U)
#define ADC_CLP0_CLP0_WIDTH                      (8U)
#define ADC_CLP0_CLP0(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLP0_CLP0_SHIFT)) & ADC_CLP0_CLP0_MASK)
/*! @} */

/*! @name CLPX - ADC Plus-Side General Calibration Value Register X */
/*! @{ */
#define ADC_CLPX_CLPX_MASK                       (0x7FU)
#define ADC_CLPX_CLPX_SHIFT                      (0U)
#define ADC_CLPX_CLPX_WIDTH                      (7U)
#define ADC_CLPX_CLPX(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLPX_CLPX_SHIFT)) & ADC_CLPX_CLPX_MASK)
/*! @} */

/*! @name CLP9 - ADC Plus-Side General Calibration Value Register 9 */
/*! @{ */
#define ADC_CLP9_CLP9_MASK                       (0x7FU)
#define ADC_CLP9_CLP9_SHIFT                      (0U)
#define ADC_CLP9_CLP9_WIDTH                      (7U)
#define ADC_CLP9_CLP9(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CLP9_CLP9_SHIFT)) & ADC_CLP9_CLP9_MASK)
/*! @} */

/*! @name CLPS_OFS - ADC General Calibration Offset Value Register S */
/*! @{ */
#define ADC_CLPS_OFS_CLPS_OFS_MASK               (0xFU)
#define ADC_CLPS_OFS_CLPS_OFS_SHIFT              (0U)
#define ADC_CLPS_OFS_CLPS_OFS_WIDTH              (4U)
#define ADC_CLPS_OFS_CLPS_OFS(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_CLPS_OFS_CLPS_OFS_SHIFT)) & ADC_CLPS_OFS_CLPS_OFS_MASK)
/*! @} */

/*! @name CLP3_OFS - ADC Plus-Side General Calibration Offset Value Register 3 */
/*! @{ */
#define ADC_CLP3_OFS_CLP3_OFS_MASK               (0xFU)
#define ADC_CLP3_OFS_CLP3_OFS_SHIFT              (0U)
#define ADC_CLP3_OFS_CLP3_OFS_WIDTH              (4U)
#define ADC_CLP3_OFS_CLP3_OFS(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_CLP3_OFS_CLP3_OFS_SHIFT)) & ADC_CLP3_OFS_CLP3_OFS_MASK)
/*! @} */

/*! @name CLP2_OFS - ADC Plus-Side General Calibration Offset Value Register 2 */
/*! @{ */
#define ADC_CLP2_OFS_CLP2_OFS_MASK               (0xFU)
#define ADC_CLP2_OFS_CLP2_OFS_SHIFT              (0U)
#define ADC_CLP2_OFS_CLP2_OFS_WIDTH              (4U)
#define ADC_CLP2_OFS_CLP2_OFS(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_CLP2_OFS_CLP2_OFS_SHIFT)) & ADC_CLP2_OFS_CLP2_OFS_MASK)
/*! @} */

/*! @name CLP1_OFS - ADC Plus-Side General Calibration Offset Value Register 1 */
/*! @{ */
#define ADC_CLP1_OFS_CLP1_OFS_MASK               (0xFU)
#define ADC_CLP1_OFS_CLP1_OFS_SHIFT              (0U)
#define ADC_CLP1_OFS_CLP1_OFS_WIDTH              (4U)
#define ADC_CLP1_OFS_CLP1_OFS(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_CLP1_OFS_CLP1_OFS_SHIFT)) & ADC_CLP1_OFS_CLP1_OFS_MASK)
/*! @} */

/*! @name CLP0_OFS - ADC Plus-Side General Calibration Offset Value Register 0 */
/*! @{ */
#define ADC_CLP0_OFS_CLP0_OFS_MASK               (0xFU)
#define ADC_CLP0_OFS_CLP0_OFS_SHIFT              (0U)
#define ADC_CLP0_OFS_CLP0_OFS_WIDTH              (4U)
#define ADC_CLP0_OFS_CLP0_OFS(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_CLP0_OFS_CLP0_OFS_SHIFT)) & ADC_CLP0_OFS_CLP0_OFS_MASK)
/*! @} */

/*! @name CLPX_OFS - ADC Plus-Side General Calibration Offset Value Register X */
/*! @{ */
#define ADC_CLPX_OFS_CLPX_OFS_MASK               (0xFFFU)
#define ADC_CLPX_OFS_CLPX_OFS_SHIFT              (0U)
#define ADC_CLPX_OFS_CLPX_OFS_WIDTH              (12U)
#define ADC_CLPX_OFS_CLPX_OFS(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_CLPX_OFS_CLPX_OFS_SHIFT)) & ADC_CLPX_OFS_CLPX_OFS_MASK)
/*! @} */

/*! @name CLP9_OFS - ADC Plus-Side General Calibration Offset Value Register 9 */
/*! @{ */
#define ADC_CLP9_OFS_CLP9_OFS_MASK               (0xFFFU)
#define ADC_CLP9_OFS_CLP9_OFS_SHIFT              (0U)
#define ADC_CLP9_OFS_CLP9_OFS_WIDTH              (12U)
#define ADC_CLP9_OFS_CLP9_OFS(x)                 (((uint32_t)(((uint32_t)(x)) << ADC_CLP9_OFS_CLP9_OFS_SHIFT)) & ADC_CLP9_OFS_CLP9_OFS_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group ADC_Register_Masks */

/*!
 * @}
 */ /* end of group ADC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- AIPS Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup AIPS_Peripheral_Access_Layer AIPS Peripheral Access Layer
 * @{
 */

/** AIPS - Size of Registers Arrays */
#define AIPS_OPACR_COUNT                          12u

/** AIPS - Register Layout Typedef */
typedef struct {
  __IO uint32_t MPRA;                              /**< Master Privilege Register A, offset: 0x0 */
  uint8_t RESERVED_0[28];
  __IO uint32_t PACRA;                             /**< Peripheral Access Control Register, offset: 0x20 */
  __IO uint32_t PACRB;                             /**< Peripheral Access Control Register, offset: 0x24 */
  uint8_t RESERVED_1[4];
  __IO uint32_t PACRD;                             /**< Peripheral Access Control Register, offset: 0x2C */
  uint8_t RESERVED_2[16];
  __IO uint32_t OPACR[AIPS_OPACR_COUNT];           /**< Off-Platform Peripheral Access Control Register, array offset: 0x40, array step: 0x4 */
} AIPS_Type, *AIPS_MemMapPtr;

/** Number of instances of the AIPS module. */
#define AIPS_INSTANCE_COUNT                      (1u)

/* AIPS - Peripheral instance base addresses */
/** Peripheral AIPS base address */
#define AIPS_BASE                                (0x40000000u)
/** Peripheral AIPS base pointer */
#define AIPS                                     ((AIPS_Type *)AIPS_BASE)
/** Array initializer of AIPS peripheral base addresses */
#define AIPS_BASE_ADDRS                          { AIPS_BASE }
/** Array initializer of AIPS peripheral base pointers */
#define AIPS_BASE_PTRS                           { AIPS }

/* ----------------------------------------------------------------------------
   -- AIPS Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup AIPS_Register_Masks AIPS Register Masks
 * @{
 */

/*! @name MPRA - Master Privilege Register A */
/*! @{ */
#define AIPS_MPRA_MPL2_MASK                      (0x100000U)
#define AIPS_MPRA_MPL2_SHIFT                     (20U)
#define AIPS_MPRA_MPL2_WIDTH                     (1U)
#define AIPS_MPRA_MPL2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MPL2_SHIFT)) & AIPS_MPRA_MPL2_MASK)
#define AIPS_MPRA_MTW2_MASK                      (0x200000U)
#define AIPS_MPRA_MTW2_SHIFT                     (21U)
#define AIPS_MPRA_MTW2_WIDTH                     (1U)
#define AIPS_MPRA_MTW2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MTW2_SHIFT)) & AIPS_MPRA_MTW2_MASK)
#define AIPS_MPRA_MTR2_MASK                      (0x400000U)
#define AIPS_MPRA_MTR2_SHIFT                     (22U)
#define AIPS_MPRA_MTR2_WIDTH                     (1U)
#define AIPS_MPRA_MTR2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MTR2_SHIFT)) & AIPS_MPRA_MTR2_MASK)
#define AIPS_MPRA_MPL1_MASK                      (0x1000000U)
#define AIPS_MPRA_MPL1_SHIFT                     (24U)
#define AIPS_MPRA_MPL1_WIDTH                     (1U)
#define AIPS_MPRA_MPL1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MPL1_SHIFT)) & AIPS_MPRA_MPL1_MASK)
#define AIPS_MPRA_MTW1_MASK                      (0x2000000U)
#define AIPS_MPRA_MTW1_SHIFT                     (25U)
#define AIPS_MPRA_MTW1_WIDTH                     (1U)
#define AIPS_MPRA_MTW1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MTW1_SHIFT)) & AIPS_MPRA_MTW1_MASK)
#define AIPS_MPRA_MTR1_MASK                      (0x4000000U)
#define AIPS_MPRA_MTR1_SHIFT                     (26U)
#define AIPS_MPRA_MTR1_WIDTH                     (1U)
#define AIPS_MPRA_MTR1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MTR1_SHIFT)) & AIPS_MPRA_MTR1_MASK)
#define AIPS_MPRA_MPL0_MASK                      (0x10000000U)
#define AIPS_MPRA_MPL0_SHIFT                     (28U)
#define AIPS_MPRA_MPL0_WIDTH                     (1U)
#define AIPS_MPRA_MPL0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MPL0_SHIFT)) & AIPS_MPRA_MPL0_MASK)
#define AIPS_MPRA_MTW0_MASK                      (0x20000000U)
#define AIPS_MPRA_MTW0_SHIFT                     (29U)
#define AIPS_MPRA_MTW0_WIDTH                     (1U)
#define AIPS_MPRA_MTW0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MTW0_SHIFT)) & AIPS_MPRA_MTW0_MASK)
#define AIPS_MPRA_MTR0_MASK                      (0x40000000U)
#define AIPS_MPRA_MTR0_SHIFT                     (30U)
#define AIPS_MPRA_MTR0_WIDTH                     (1U)
#define AIPS_MPRA_MTR0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_MPRA_MTR0_SHIFT)) & AIPS_MPRA_MTR0_MASK)
/*! @} */

/*! @name PACRA - Peripheral Access Control Register */
/*! @{ */
#define AIPS_PACRA_TP1_MASK                      (0x1000000U)
#define AIPS_PACRA_TP1_SHIFT                     (24U)
#define AIPS_PACRA_TP1_WIDTH                     (1U)
#define AIPS_PACRA_TP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_TP1_SHIFT)) & AIPS_PACRA_TP1_MASK)
#define AIPS_PACRA_WP1_MASK                      (0x2000000U)
#define AIPS_PACRA_WP1_SHIFT                     (25U)
#define AIPS_PACRA_WP1_WIDTH                     (1U)
#define AIPS_PACRA_WP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_WP1_SHIFT)) & AIPS_PACRA_WP1_MASK)
#define AIPS_PACRA_SP1_MASK                      (0x4000000U)
#define AIPS_PACRA_SP1_SHIFT                     (26U)
#define AIPS_PACRA_SP1_WIDTH                     (1U)
#define AIPS_PACRA_SP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_SP1_SHIFT)) & AIPS_PACRA_SP1_MASK)
#define AIPS_PACRA_TP0_MASK                      (0x10000000U)
#define AIPS_PACRA_TP0_SHIFT                     (28U)
#define AIPS_PACRA_TP0_WIDTH                     (1U)
#define AIPS_PACRA_TP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_TP0_SHIFT)) & AIPS_PACRA_TP0_MASK)
#define AIPS_PACRA_WP0_MASK                      (0x20000000U)
#define AIPS_PACRA_WP0_SHIFT                     (29U)
#define AIPS_PACRA_WP0_WIDTH                     (1U)
#define AIPS_PACRA_WP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_WP0_SHIFT)) & AIPS_PACRA_WP0_MASK)
#define AIPS_PACRA_SP0_MASK                      (0x40000000U)
#define AIPS_PACRA_SP0_SHIFT                     (30U)
#define AIPS_PACRA_SP0_WIDTH                     (1U)
#define AIPS_PACRA_SP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRA_SP0_SHIFT)) & AIPS_PACRA_SP0_MASK)
/*! @} */

/*! @name PACRB - Peripheral Access Control Register */
/*! @{ */
#define AIPS_PACRB_TP5_MASK                      (0x100U)
#define AIPS_PACRB_TP5_SHIFT                     (8U)
#define AIPS_PACRB_TP5_WIDTH                     (1U)
#define AIPS_PACRB_TP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_TP5_SHIFT)) & AIPS_PACRB_TP5_MASK)
#define AIPS_PACRB_WP5_MASK                      (0x200U)
#define AIPS_PACRB_WP5_SHIFT                     (9U)
#define AIPS_PACRB_WP5_WIDTH                     (1U)
#define AIPS_PACRB_WP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_WP5_SHIFT)) & AIPS_PACRB_WP5_MASK)
#define AIPS_PACRB_SP5_MASK                      (0x400U)
#define AIPS_PACRB_SP5_SHIFT                     (10U)
#define AIPS_PACRB_SP5_WIDTH                     (1U)
#define AIPS_PACRB_SP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_SP5_SHIFT)) & AIPS_PACRB_SP5_MASK)
#define AIPS_PACRB_TP1_MASK                      (0x1000000U)
#define AIPS_PACRB_TP1_SHIFT                     (24U)
#define AIPS_PACRB_TP1_WIDTH                     (1U)
#define AIPS_PACRB_TP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_TP1_SHIFT)) & AIPS_PACRB_TP1_MASK)
#define AIPS_PACRB_WP1_MASK                      (0x2000000U)
#define AIPS_PACRB_WP1_SHIFT                     (25U)
#define AIPS_PACRB_WP1_WIDTH                     (1U)
#define AIPS_PACRB_WP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_WP1_SHIFT)) & AIPS_PACRB_WP1_MASK)
#define AIPS_PACRB_SP1_MASK                      (0x4000000U)
#define AIPS_PACRB_SP1_SHIFT                     (26U)
#define AIPS_PACRB_SP1_WIDTH                     (1U)
#define AIPS_PACRB_SP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_SP1_SHIFT)) & AIPS_PACRB_SP1_MASK)
#define AIPS_PACRB_TP0_MASK                      (0x10000000U)
#define AIPS_PACRB_TP0_SHIFT                     (28U)
#define AIPS_PACRB_TP0_WIDTH                     (1U)
#define AIPS_PACRB_TP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_TP0_SHIFT)) & AIPS_PACRB_TP0_MASK)
#define AIPS_PACRB_WP0_MASK                      (0x20000000U)
#define AIPS_PACRB_WP0_SHIFT                     (29U)
#define AIPS_PACRB_WP0_WIDTH                     (1U)
#define AIPS_PACRB_WP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_WP0_SHIFT)) & AIPS_PACRB_WP0_MASK)
#define AIPS_PACRB_SP0_MASK                      (0x40000000U)
#define AIPS_PACRB_SP0_SHIFT                     (30U)
#define AIPS_PACRB_SP0_WIDTH                     (1U)
#define AIPS_PACRB_SP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRB_SP0_SHIFT)) & AIPS_PACRB_SP0_MASK)
/*! @} */

/*! @name PACRD - Peripheral Access Control Register */
/*! @{ */
#define AIPS_PACRD_TP1_MASK                      (0x1000000U)
#define AIPS_PACRD_TP1_SHIFT                     (24U)
#define AIPS_PACRD_TP1_WIDTH                     (1U)
#define AIPS_PACRD_TP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_TP1_SHIFT)) & AIPS_PACRD_TP1_MASK)
#define AIPS_PACRD_WP1_MASK                      (0x2000000U)
#define AIPS_PACRD_WP1_SHIFT                     (25U)
#define AIPS_PACRD_WP1_WIDTH                     (1U)
#define AIPS_PACRD_WP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_WP1_SHIFT)) & AIPS_PACRD_WP1_MASK)
#define AIPS_PACRD_SP1_MASK                      (0x4000000U)
#define AIPS_PACRD_SP1_SHIFT                     (26U)
#define AIPS_PACRD_SP1_WIDTH                     (1U)
#define AIPS_PACRD_SP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_SP1_SHIFT)) & AIPS_PACRD_SP1_MASK)
#define AIPS_PACRD_TP0_MASK                      (0x10000000U)
#define AIPS_PACRD_TP0_SHIFT                     (28U)
#define AIPS_PACRD_TP0_WIDTH                     (1U)
#define AIPS_PACRD_TP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_TP0_SHIFT)) & AIPS_PACRD_TP0_MASK)
#define AIPS_PACRD_WP0_MASK                      (0x20000000U)
#define AIPS_PACRD_WP0_SHIFT                     (29U)
#define AIPS_PACRD_WP0_WIDTH                     (1U)
#define AIPS_PACRD_WP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_WP0_SHIFT)) & AIPS_PACRD_WP0_MASK)
#define AIPS_PACRD_SP0_MASK                      (0x40000000U)
#define AIPS_PACRD_SP0_SHIFT                     (30U)
#define AIPS_PACRD_SP0_WIDTH                     (1U)
#define AIPS_PACRD_SP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_PACRD_SP0_SHIFT)) & AIPS_PACRD_SP0_MASK)
/*! @} */

/*! @name OPACR - Off-Platform Peripheral Access Control Register */
/*! @{ */
#define AIPS_OPACR_TP7_MASK                      (0x1U)
#define AIPS_OPACR_TP7_SHIFT                     (0U)
#define AIPS_OPACR_TP7_WIDTH                     (1U)
#define AIPS_OPACR_TP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_TP7_SHIFT)) & AIPS_OPACR_TP7_MASK)
#define AIPS_OPACR_WP7_MASK                      (0x2U)
#define AIPS_OPACR_WP7_SHIFT                     (1U)
#define AIPS_OPACR_WP7_WIDTH                     (1U)
#define AIPS_OPACR_WP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_WP7_SHIFT)) & AIPS_OPACR_WP7_MASK)
#define AIPS_OPACR_SP7_MASK                      (0x4U)
#define AIPS_OPACR_SP7_SHIFT                     (2U)
#define AIPS_OPACR_SP7_WIDTH                     (1U)
#define AIPS_OPACR_SP7(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_SP7_SHIFT)) & AIPS_OPACR_SP7_MASK)
#define AIPS_OPACR_TP6_MASK                      (0x10U)
#define AIPS_OPACR_TP6_SHIFT                     (4U)
#define AIPS_OPACR_TP6_WIDTH                     (1U)
#define AIPS_OPACR_TP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_TP6_SHIFT)) & AIPS_OPACR_TP6_MASK)
#define AIPS_OPACR_WP6_MASK                      (0x20U)
#define AIPS_OPACR_WP6_SHIFT                     (5U)
#define AIPS_OPACR_WP6_WIDTH                     (1U)
#define AIPS_OPACR_WP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_WP6_SHIFT)) & AIPS_OPACR_WP6_MASK)
#define AIPS_OPACR_SP6_MASK                      (0x40U)
#define AIPS_OPACR_SP6_SHIFT                     (6U)
#define AIPS_OPACR_SP6_WIDTH                     (1U)
#define AIPS_OPACR_SP6(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_SP6_SHIFT)) & AIPS_OPACR_SP6_MASK)
#define AIPS_OPACR_TP5_MASK                      (0x100U)
#define AIPS_OPACR_TP5_SHIFT                     (8U)
#define AIPS_OPACR_TP5_WIDTH                     (1U)
#define AIPS_OPACR_TP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_TP5_SHIFT)) & AIPS_OPACR_TP5_MASK)
#define AIPS_OPACR_WP5_MASK                      (0x200U)
#define AIPS_OPACR_WP5_SHIFT                     (9U)
#define AIPS_OPACR_WP5_WIDTH                     (1U)
#define AIPS_OPACR_WP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_WP5_SHIFT)) & AIPS_OPACR_WP5_MASK)
#define AIPS_OPACR_SP5_MASK                      (0x400U)
#define AIPS_OPACR_SP5_SHIFT                     (10U)
#define AIPS_OPACR_SP5_WIDTH                     (1U)
#define AIPS_OPACR_SP5(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_SP5_SHIFT)) & AIPS_OPACR_SP5_MASK)
#define AIPS_OPACR_TP4_MASK                      (0x1000U)
#define AIPS_OPACR_TP4_SHIFT                     (12U)
#define AIPS_OPACR_TP4_WIDTH                     (1U)
#define AIPS_OPACR_TP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_TP4_SHIFT)) & AIPS_OPACR_TP4_MASK)
#define AIPS_OPACR_WP4_MASK                      (0x2000U)
#define AIPS_OPACR_WP4_SHIFT                     (13U)
#define AIPS_OPACR_WP4_WIDTH                     (1U)
#define AIPS_OPACR_WP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_WP4_SHIFT)) & AIPS_OPACR_WP4_MASK)
#define AIPS_OPACR_SP4_MASK                      (0x4000U)
#define AIPS_OPACR_SP4_SHIFT                     (14U)
#define AIPS_OPACR_SP4_WIDTH                     (1U)
#define AIPS_OPACR_SP4(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_SP4_SHIFT)) & AIPS_OPACR_SP4_MASK)
#define AIPS_OPACR_TP3_MASK                      (0x10000U)
#define AIPS_OPACR_TP3_SHIFT                     (16U)
#define AIPS_OPACR_TP3_WIDTH                     (1U)
#define AIPS_OPACR_TP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_TP3_SHIFT)) & AIPS_OPACR_TP3_MASK)
#define AIPS_OPACR_WP3_MASK                      (0x20000U)
#define AIPS_OPACR_WP3_SHIFT                     (17U)
#define AIPS_OPACR_WP3_WIDTH                     (1U)
#define AIPS_OPACR_WP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_WP3_SHIFT)) & AIPS_OPACR_WP3_MASK)
#define AIPS_OPACR_SP3_MASK                      (0x40000U)
#define AIPS_OPACR_SP3_SHIFT                     (18U)
#define AIPS_OPACR_SP3_WIDTH                     (1U)
#define AIPS_OPACR_SP3(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_SP3_SHIFT)) & AIPS_OPACR_SP3_MASK)
#define AIPS_OPACR_TP2_MASK                      (0x100000U)
#define AIPS_OPACR_TP2_SHIFT                     (20U)
#define AIPS_OPACR_TP2_WIDTH                     (1U)
#define AIPS_OPACR_TP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_TP2_SHIFT)) & AIPS_OPACR_TP2_MASK)
#define AIPS_OPACR_WP2_MASK                      (0x200000U)
#define AIPS_OPACR_WP2_SHIFT                     (21U)
#define AIPS_OPACR_WP2_WIDTH                     (1U)
#define AIPS_OPACR_WP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_WP2_SHIFT)) & AIPS_OPACR_WP2_MASK)
#define AIPS_OPACR_SP2_MASK                      (0x400000U)
#define AIPS_OPACR_SP2_SHIFT                     (22U)
#define AIPS_OPACR_SP2_WIDTH                     (1U)
#define AIPS_OPACR_SP2(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_SP2_SHIFT)) & AIPS_OPACR_SP2_MASK)
#define AIPS_OPACR_TP1_MASK                      (0x1000000U)
#define AIPS_OPACR_TP1_SHIFT                     (24U)
#define AIPS_OPACR_TP1_WIDTH                     (1U)
#define AIPS_OPACR_TP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_TP1_SHIFT)) & AIPS_OPACR_TP1_MASK)
#define AIPS_OPACR_WP1_MASK                      (0x2000000U)
#define AIPS_OPACR_WP1_SHIFT                     (25U)
#define AIPS_OPACR_WP1_WIDTH                     (1U)
#define AIPS_OPACR_WP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_WP1_SHIFT)) & AIPS_OPACR_WP1_MASK)
#define AIPS_OPACR_SP1_MASK                      (0x4000000U)
#define AIPS_OPACR_SP1_SHIFT                     (26U)
#define AIPS_OPACR_SP1_WIDTH                     (1U)
#define AIPS_OPACR_SP1(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_SP1_SHIFT)) & AIPS_OPACR_SP1_MASK)
#define AIPS_OPACR_TP0_MASK                      (0x10000000U)
#define AIPS_OPACR_TP0_SHIFT                     (28U)
#define AIPS_OPACR_TP0_WIDTH                     (1U)
#define AIPS_OPACR_TP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_TP0_SHIFT)) & AIPS_OPACR_TP0_MASK)
#define AIPS_OPACR_WP0_MASK                      (0x20000000U)
#define AIPS_OPACR_WP0_SHIFT                     (29U)
#define AIPS_OPACR_WP0_WIDTH                     (1U)
#define AIPS_OPACR_WP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_WP0_SHIFT)) & AIPS_OPACR_WP0_MASK)
#define AIPS_OPACR_SP0_MASK                      (0x40000000U)
#define AIPS_OPACR_SP0_SHIFT                     (30U)
#define AIPS_OPACR_SP0_WIDTH                     (1U)
#define AIPS_OPACR_SP0(x)                        (((uint32_t)(((uint32_t)(x)) << AIPS_OPACR_SP0_SHIFT)) & AIPS_OPACR_SP0_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group AIPS_Register_Masks */

/*!
 * @}
 */ /* end of group AIPS_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- CAN Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CAN_Peripheral_Access_Layer CAN Peripheral Access Layer
 * @{
 */

/** CAN - Size of Registers Arrays */
#define CAN_RAMn_COUNT                            256u
#define CAN_RXIMR_COUNT                           64u
#define CAN_WMB_COUNT                             4u

/** CAN - Register Layout Typedef */
typedef struct {
  __IO uint32_t MCR;                               /**< Module Configuration register, offset: 0x0 */
  __IO uint32_t CTRL1;                             /**< Control 1 register, offset: 0x4 */
  __IO uint32_t TIMER;                             /**< Free Running Timer, offset: 0x8 */
  uint8_t RESERVED_0[4];
  __IO uint32_t RXMGMASK;                          /**< Rx Mailboxes Global Mask register, offset: 0x10 */
  __IO uint32_t RX14MASK;                          /**< Rx 14 Mask register, offset: 0x14 */
  __IO uint32_t RX15MASK;                          /**< Rx 15 Mask register, offset: 0x18 */
  __IO uint32_t ECR;                               /**< Error Counter, offset: 0x1C */
  __IO uint32_t ESR1;                              /**< Error and Status 1 register, offset: 0x20 */
  __IO uint32_t IMASK2;                            /**< Interrupt Masks 2 register, offset: 0x24 */
  __IO uint32_t IMASK1;                            /**< Interrupt Masks 1 register, offset: 0x28 */
  __IO uint32_t IFLAG2;                            /**< Interrupt Flags 2 register, offset: 0x2C */
  __IO uint32_t IFLAG1;                            /**< Interrupt Flags 1 register, offset: 0x30 */
  __IO uint32_t CTRL2;                             /**< Control 2 register, offset: 0x34 */
  __I  uint32_t ESR2;                              /**< Error and Status 2 register, offset: 0x38 */
  uint8_t RESERVED_1[8];
  __I  uint32_t CRCR;                              /**< CRC register, offset: 0x44 */
  __IO uint32_t RXFGMASK;                          /**< Rx FIFO Global Mask register, offset: 0x48 */
  __I  uint32_t RXFIR;                             /**< Rx FIFO Information register, offset: 0x4C */
  __IO uint32_t CBT;                               /**< CAN Bit Timing register, offset: 0x50 */
  uint8_t RESERVED_2[44];
  __IO uint32_t RAMn[CAN_RAMn_COUNT];              /**< Embedded RAM, array offset: 0x80, array step: 0x4 */
  uint8_t RESERVED_3[1024];
  __IO uint32_t RXIMR[CAN_RXIMR_COUNT];            /**< Rx Individual Mask registers, array offset: 0x880, array step: 0x4 */
  uint8_t RESERVED_4[384];
  __IO uint32_t CTRL1_PN;                          /**< Pretended Networking Control 1 register, offset: 0xB00 */
  __IO uint32_t CTRL2_PN;                          /**< Pretended Networking Control 2 register, offset: 0xB04 */
  __IO uint32_t WU_MTC;                            /**< Pretended Networking Wake Up Match register, offset: 0xB08 */
  __IO uint32_t FLT_ID1;                           /**< Pretended Networking ID Filter 1 register, offset: 0xB0C */
  __IO uint32_t FLT_DLC;                           /**< Pretended Networking DLC Filter register, offset: 0xB10 */
  __IO uint32_t PL1_LO;                            /**< Pretended Networking Payload Low Filter 1 register, offset: 0xB14 */
  __IO uint32_t PL1_HI;                            /**< Pretended Networking Payload High Filter 1 register, offset: 0xB18 */
  __IO uint32_t FLT_ID2_IDMASK;                    /**< Pretended Networking ID Filter 2 Register / ID Mask register, offset: 0xB1C */
  __IO uint32_t PL2_PLMASK_LO;                     /**< Pretended Networking Payload Low Filter 2 Register / Payload Low Mask register, offset: 0xB20 */
  __IO uint32_t PL2_PLMASK_HI;                     /**< Pretended Networking Payload High Filter 2 low order bits / Payload High Mask register, offset: 0xB24 */
  uint8_t RESERVED_5[24];
  struct {                                         /* offset: 0xB40, array step: 0x10 */
    __I  uint32_t WMBn_CS;                           /**< Wake Up Message Buffer register for C/S, array offset: 0xB40, array step: 0x10 */
    __I  uint32_t WMBn_ID;                           /**< Wake Up Message Buffer Register for ID, array offset: 0xB44, array step: 0x10 */
    __I  uint32_t WMBn_D03;                          /**< Wake Up Message Buffer Register for Data 0-3, array offset: 0xB48, array step: 0x10 */
    __I  uint32_t WMBn_D47;                          /**< Wake Up Message Buffer Register Data 4-7, array offset: 0xB4C, array step: 0x10 */
  } WMB[CAN_WMB_COUNT];
  uint8_t RESERVED_6[128];
  __IO uint32_t FDCTRL;                            /**< CAN FD Control register, offset: 0xC00 */
  __IO uint32_t FDCBT;                             /**< CAN FD Bit Timing register, offset: 0xC04 */
  __I  uint32_t FDCRC;                             /**< CAN FD CRC register, offset: 0xC08 */
} CAN_Type, *CAN_MemMapPtr;

/** Number of instances of the CAN module. */
#define CAN_INSTANCE_COUNT                       (2u)

/* CAN - Peripheral instance base addresses */
/** Peripheral CAN0 base address */
#define CAN0_BASE                                (0x40024000u)
/** Peripheral CAN0 base pointer */
#define CAN0                                     ((CAN_Type *)CAN0_BASE)
/** Peripheral CAN1 base address */
#define CAN1_BASE                                (0x40025000u)
/** Peripheral CAN1 base pointer */
#define CAN1                                     ((CAN_Type *)CAN1_BASE)
/** Array initializer of CAN peripheral base addresses */
#define CAN_BASE_ADDRS                           { CAN0_BASE, CAN1_BASE }
/** Array initializer of CAN peripheral base pointers */
#define CAN_BASE_PTRS                            { CAN0, CAN1 }

/* ----------------------------------------------------------------------------
   -- CAN Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CAN_Register_Masks CAN Register Masks
 * @{
 */

/*! @name MCR - Module Configuration register */
/*! @{ */
#define CAN_MCR_MAXMB_MASK                       (0x7FU)
#define CAN_MCR_MAXMB_SHIFT                      (0U)
#define CAN_MCR_MAXMB_WIDTH                      (7U)
#define CAN_MCR_MAXMB(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_MCR_MAXMB_SHIFT)) & CAN_MCR_MAXMB_MASK)
#define CAN_MCR_IDAM_MASK                        (0x300U)
#define CAN_MCR_IDAM_SHIFT                       (8U)
#define CAN_MCR_IDAM_WIDTH                       (2U)
#define CAN_MCR_IDAM(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_MCR_IDAM_SHIFT)) & CAN_MCR_IDAM_MASK)
#define CAN_MCR_FDEN_MASK                        (0x800U)
#define CAN_MCR_FDEN_SHIFT                       (11U)
#define CAN_MCR_FDEN_WIDTH                       (1U)
#define CAN_MCR_FDEN(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_MCR_FDEN_SHIFT)) & CAN_MCR_FDEN_MASK)
#define CAN_MCR_AEN_MASK                         (0x1000U)
#define CAN_MCR_AEN_SHIFT                        (12U)
#define CAN_MCR_AEN_WIDTH                        (1U)
#define CAN_MCR_AEN(x)                           (((uint32_t)(((uint32_t)(x)) << CAN_MCR_AEN_SHIFT)) & CAN_MCR_AEN_MASK)
#define CAN_MCR_LPRIOEN_MASK                     (0x2000U)
#define CAN_MCR_LPRIOEN_SHIFT                    (13U)
#define CAN_MCR_LPRIOEN_WIDTH                    (1U)
#define CAN_MCR_LPRIOEN(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_MCR_LPRIOEN_SHIFT)) & CAN_MCR_LPRIOEN_MASK)
#define CAN_MCR_PNET_EN_MASK                     (0x4000U)
#define CAN_MCR_PNET_EN_SHIFT                    (14U)
#define CAN_MCR_PNET_EN_WIDTH                    (1U)
#define CAN_MCR_PNET_EN(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_MCR_PNET_EN_SHIFT)) & CAN_MCR_PNET_EN_MASK)
#define CAN_MCR_DMA_MASK                         (0x8000U)
#define CAN_MCR_DMA_SHIFT                        (15U)
#define CAN_MCR_DMA_WIDTH                        (1U)
#define CAN_MCR_DMA(x)                           (((uint32_t)(((uint32_t)(x)) << CAN_MCR_DMA_SHIFT)) & CAN_MCR_DMA_MASK)
#define CAN_MCR_IRMQ_MASK                        (0x10000U)
#define CAN_MCR_IRMQ_SHIFT                       (16U)
#define CAN_MCR_IRMQ_WIDTH                       (1U)
#define CAN_MCR_IRMQ(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_MCR_IRMQ_SHIFT)) & CAN_MCR_IRMQ_MASK)
#define CAN_MCR_SRXDIS_MASK                      (0x20000U)
#define CAN_MCR_SRXDIS_SHIFT                     (17U)
#define CAN_MCR_SRXDIS_WIDTH                     (1U)
#define CAN_MCR_SRXDIS(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_MCR_SRXDIS_SHIFT)) & CAN_MCR_SRXDIS_MASK)
#define CAN_MCR_LPMACK_MASK                      (0x100000U)
#define CAN_MCR_LPMACK_SHIFT                     (20U)
#define CAN_MCR_LPMACK_WIDTH                     (1U)
#define CAN_MCR_LPMACK(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_MCR_LPMACK_SHIFT)) & CAN_MCR_LPMACK_MASK)
#define CAN_MCR_WRNEN_MASK                       (0x200000U)
#define CAN_MCR_WRNEN_SHIFT                      (21U)
#define CAN_MCR_WRNEN_WIDTH                      (1U)
#define CAN_MCR_WRNEN(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_MCR_WRNEN_SHIFT)) & CAN_MCR_WRNEN_MASK)
#define CAN_MCR_SUPV_MASK                        (0x800000U)
#define CAN_MCR_SUPV_SHIFT                       (23U)
#define CAN_MCR_SUPV_WIDTH                       (1U)
#define CAN_MCR_SUPV(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_MCR_SUPV_SHIFT)) & CAN_MCR_SUPV_MASK)
#define CAN_MCR_FRZACK_MASK                      (0x1000000U)
#define CAN_MCR_FRZACK_SHIFT                     (24U)
#define CAN_MCR_FRZACK_WIDTH                     (1U)
#define CAN_MCR_FRZACK(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_MCR_FRZACK_SHIFT)) & CAN_MCR_FRZACK_MASK)
#define CAN_MCR_SOFTRST_MASK                     (0x2000000U)
#define CAN_MCR_SOFTRST_SHIFT                    (25U)
#define CAN_MCR_SOFTRST_WIDTH                    (1U)
#define CAN_MCR_SOFTRST(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_MCR_SOFTRST_SHIFT)) & CAN_MCR_SOFTRST_MASK)
#define CAN_MCR_NOTRDY_MASK                      (0x8000000U)
#define CAN_MCR_NOTRDY_SHIFT                     (27U)
#define CAN_MCR_NOTRDY_WIDTH                     (1U)
#define CAN_MCR_NOTRDY(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_MCR_NOTRDY_SHIFT)) & CAN_MCR_NOTRDY_MASK)
#define CAN_MCR_HALT_MASK                        (0x10000000U)
#define CAN_MCR_HALT_SHIFT                       (28U)
#define CAN_MCR_HALT_WIDTH                       (1U)
#define CAN_MCR_HALT(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_MCR_HALT_SHIFT)) & CAN_MCR_HALT_MASK)
#define CAN_MCR_RFEN_MASK                        (0x20000000U)
#define CAN_MCR_RFEN_SHIFT                       (29U)
#define CAN_MCR_RFEN_WIDTH                       (1U)
#define CAN_MCR_RFEN(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_MCR_RFEN_SHIFT)) & CAN_MCR_RFEN_MASK)
#define CAN_MCR_FRZ_MASK                         (0x40000000U)
#define CAN_MCR_FRZ_SHIFT                        (30U)
#define CAN_MCR_FRZ_WIDTH                        (1U)
#define CAN_MCR_FRZ(x)                           (((uint32_t)(((uint32_t)(x)) << CAN_MCR_FRZ_SHIFT)) & CAN_MCR_FRZ_MASK)
#define CAN_MCR_MDIS_MASK                        (0x80000000U)
#define CAN_MCR_MDIS_SHIFT                       (31U)
#define CAN_MCR_MDIS_WIDTH                       (1U)
#define CAN_MCR_MDIS(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_MCR_MDIS_SHIFT)) & CAN_MCR_MDIS_MASK)
/*! @} */

/*! @name CTRL1 - Control 1 register */
/*! @{ */
#define CAN_CTRL1_PROPSEG_MASK                   (0x7U)
#define CAN_CTRL1_PROPSEG_SHIFT                  (0U)
#define CAN_CTRL1_PROPSEG_WIDTH                  (3U)
#define CAN_CTRL1_PROPSEG(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PROPSEG_SHIFT)) & CAN_CTRL1_PROPSEG_MASK)
#define CAN_CTRL1_LOM_MASK                       (0x8U)
#define CAN_CTRL1_LOM_SHIFT                      (3U)
#define CAN_CTRL1_LOM_WIDTH                      (1U)
#define CAN_CTRL1_LOM(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_LOM_SHIFT)) & CAN_CTRL1_LOM_MASK)
#define CAN_CTRL1_LBUF_MASK                      (0x10U)
#define CAN_CTRL1_LBUF_SHIFT                     (4U)
#define CAN_CTRL1_LBUF_WIDTH                     (1U)
#define CAN_CTRL1_LBUF(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_LBUF_SHIFT)) & CAN_CTRL1_LBUF_MASK)
#define CAN_CTRL1_TSYN_MASK                      (0x20U)
#define CAN_CTRL1_TSYN_SHIFT                     (5U)
#define CAN_CTRL1_TSYN_WIDTH                     (1U)
#define CAN_CTRL1_TSYN(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_TSYN_SHIFT)) & CAN_CTRL1_TSYN_MASK)
#define CAN_CTRL1_BOFFREC_MASK                   (0x40U)
#define CAN_CTRL1_BOFFREC_SHIFT                  (6U)
#define CAN_CTRL1_BOFFREC_WIDTH                  (1U)
#define CAN_CTRL1_BOFFREC(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_BOFFREC_SHIFT)) & CAN_CTRL1_BOFFREC_MASK)
#define CAN_CTRL1_SMP_MASK                       (0x80U)
#define CAN_CTRL1_SMP_SHIFT                      (7U)
#define CAN_CTRL1_SMP_WIDTH                      (1U)
#define CAN_CTRL1_SMP(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_SMP_SHIFT)) & CAN_CTRL1_SMP_MASK)
#define CAN_CTRL1_RWRNMSK_MASK                   (0x400U)
#define CAN_CTRL1_RWRNMSK_SHIFT                  (10U)
#define CAN_CTRL1_RWRNMSK_WIDTH                  (1U)
#define CAN_CTRL1_RWRNMSK(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_RWRNMSK_SHIFT)) & CAN_CTRL1_RWRNMSK_MASK)
#define CAN_CTRL1_TWRNMSK_MASK                   (0x800U)
#define CAN_CTRL1_TWRNMSK_SHIFT                  (11U)
#define CAN_CTRL1_TWRNMSK_WIDTH                  (1U)
#define CAN_CTRL1_TWRNMSK(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_TWRNMSK_SHIFT)) & CAN_CTRL1_TWRNMSK_MASK)
#define CAN_CTRL1_LPB_MASK                       (0x1000U)
#define CAN_CTRL1_LPB_SHIFT                      (12U)
#define CAN_CTRL1_LPB_WIDTH                      (1U)
#define CAN_CTRL1_LPB(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_LPB_SHIFT)) & CAN_CTRL1_LPB_MASK)
#define CAN_CTRL1_CLKSRC_MASK                    (0x2000U)
#define CAN_CTRL1_CLKSRC_SHIFT                   (13U)
#define CAN_CTRL1_CLKSRC_WIDTH                   (1U)
#define CAN_CTRL1_CLKSRC(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_CLKSRC_SHIFT)) & CAN_CTRL1_CLKSRC_MASK)
#define CAN_CTRL1_ERRMSK_MASK                    (0x4000U)
#define CAN_CTRL1_ERRMSK_SHIFT                   (14U)
#define CAN_CTRL1_ERRMSK_WIDTH                   (1U)
#define CAN_CTRL1_ERRMSK(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_ERRMSK_SHIFT)) & CAN_CTRL1_ERRMSK_MASK)
#define CAN_CTRL1_BOFFMSK_MASK                   (0x8000U)
#define CAN_CTRL1_BOFFMSK_SHIFT                  (15U)
#define CAN_CTRL1_BOFFMSK_WIDTH                  (1U)
#define CAN_CTRL1_BOFFMSK(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_BOFFMSK_SHIFT)) & CAN_CTRL1_BOFFMSK_MASK)
#define CAN_CTRL1_PSEG2_MASK                     (0x70000U)
#define CAN_CTRL1_PSEG2_SHIFT                    (16U)
#define CAN_CTRL1_PSEG2_WIDTH                    (3U)
#define CAN_CTRL1_PSEG2(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PSEG2_SHIFT)) & CAN_CTRL1_PSEG2_MASK)
#define CAN_CTRL1_PSEG1_MASK                     (0x380000U)
#define CAN_CTRL1_PSEG1_SHIFT                    (19U)
#define CAN_CTRL1_PSEG1_WIDTH                    (3U)
#define CAN_CTRL1_PSEG1(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PSEG1_SHIFT)) & CAN_CTRL1_PSEG1_MASK)
#define CAN_CTRL1_RJW_MASK                       (0xC00000U)
#define CAN_CTRL1_RJW_SHIFT                      (22U)
#define CAN_CTRL1_RJW_WIDTH                      (2U)
#define CAN_CTRL1_RJW(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_RJW_SHIFT)) & CAN_CTRL1_RJW_MASK)
#define CAN_CTRL1_PRESDIV_MASK                   (0xFF000000U)
#define CAN_CTRL1_PRESDIV_SHIFT                  (24U)
#define CAN_CTRL1_PRESDIV_WIDTH                  (8U)
#define CAN_CTRL1_PRESDIV(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PRESDIV_SHIFT)) & CAN_CTRL1_PRESDIV_MASK)
/*! @} */

/*! @name TIMER - Free Running Timer */
/*! @{ */
#define CAN_TIMER_TIMER_MASK                     (0xFFFFU)
#define CAN_TIMER_TIMER_SHIFT                    (0U)
#define CAN_TIMER_TIMER_WIDTH                    (16U)
#define CAN_TIMER_TIMER(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_TIMER_TIMER_SHIFT)) & CAN_TIMER_TIMER_MASK)
/*! @} */

/*! @name RXMGMASK - Rx Mailboxes Global Mask register */
/*! @{ */
#define CAN_RXMGMASK_MG_MASK                     (0xFFFFFFFFU)
#define CAN_RXMGMASK_MG_SHIFT                    (0U)
#define CAN_RXMGMASK_MG_WIDTH                    (32U)
#define CAN_RXMGMASK_MG(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_RXMGMASK_MG_SHIFT)) & CAN_RXMGMASK_MG_MASK)
/*! @} */

/*! @name RX14MASK - Rx 14 Mask register */
/*! @{ */
#define CAN_RX14MASK_RX14M_MASK                  (0xFFFFFFFFU)
#define CAN_RX14MASK_RX14M_SHIFT                 (0U)
#define CAN_RX14MASK_RX14M_WIDTH                 (32U)
#define CAN_RX14MASK_RX14M(x)                    (((uint32_t)(((uint32_t)(x)) << CAN_RX14MASK_RX14M_SHIFT)) & CAN_RX14MASK_RX14M_MASK)
/*! @} */

/*! @name RX15MASK - Rx 15 Mask register */
/*! @{ */
#define CAN_RX15MASK_RX15M_MASK                  (0xFFFFFFFFU)
#define CAN_RX15MASK_RX15M_SHIFT                 (0U)
#define CAN_RX15MASK_RX15M_WIDTH                 (32U)
#define CAN_RX15MASK_RX15M(x)                    (((uint32_t)(((uint32_t)(x)) << CAN_RX15MASK_RX15M_SHIFT)) & CAN_RX15MASK_RX15M_MASK)
/*! @} */

/*! @name ECR - Error Counter */
/*! @{ */
#define CAN_ECR_TXERRCNT_MASK                    (0xFFU)
#define CAN_ECR_TXERRCNT_SHIFT                   (0U)
#define CAN_ECR_TXERRCNT_WIDTH                   (8U)
#define CAN_ECR_TXERRCNT(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ECR_TXERRCNT_SHIFT)) & CAN_ECR_TXERRCNT_MASK)
#define CAN_ECR_RXERRCNT_MASK                    (0xFF00U)
#define CAN_ECR_RXERRCNT_SHIFT                   (8U)
#define CAN_ECR_RXERRCNT_WIDTH                   (8U)
#define CAN_ECR_RXERRCNT(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ECR_RXERRCNT_SHIFT)) & CAN_ECR_RXERRCNT_MASK)
#define CAN_ECR_TXERRCNT_FAST_MASK               (0xFF0000U)
#define CAN_ECR_TXERRCNT_FAST_SHIFT              (16U)
#define CAN_ECR_TXERRCNT_FAST_WIDTH              (8U)
#define CAN_ECR_TXERRCNT_FAST(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ECR_TXERRCNT_FAST_SHIFT)) & CAN_ECR_TXERRCNT_FAST_MASK)
#define CAN_ECR_RXERRCNT_FAST_MASK               (0xFF000000U)
#define CAN_ECR_RXERRCNT_FAST_SHIFT              (24U)
#define CAN_ECR_RXERRCNT_FAST_WIDTH              (8U)
#define CAN_ECR_RXERRCNT_FAST(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ECR_RXERRCNT_FAST_SHIFT)) & CAN_ECR_RXERRCNT_FAST_MASK)
/*! @} */

/*! @name ESR1 - Error and Status 1 register */
/*! @{ */
#define CAN_ESR1_ERRINT_MASK                     (0x2U)
#define CAN_ESR1_ERRINT_SHIFT                    (1U)
#define CAN_ESR1_ERRINT_WIDTH                    (1U)
#define CAN_ESR1_ERRINT(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_ERRINT_SHIFT)) & CAN_ESR1_ERRINT_MASK)
#define CAN_ESR1_BOFFINT_MASK                    (0x4U)
#define CAN_ESR1_BOFFINT_SHIFT                   (2U)
#define CAN_ESR1_BOFFINT_WIDTH                   (1U)
#define CAN_ESR1_BOFFINT(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_BOFFINT_SHIFT)) & CAN_ESR1_BOFFINT_MASK)
#define CAN_ESR1_RX_MASK                         (0x8U)
#define CAN_ESR1_RX_SHIFT                        (3U)
#define CAN_ESR1_RX_WIDTH                        (1U)
#define CAN_ESR1_RX(x)                           (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_RX_SHIFT)) & CAN_ESR1_RX_MASK)
#define CAN_ESR1_FLTCONF_MASK                    (0x30U)
#define CAN_ESR1_FLTCONF_SHIFT                   (4U)
#define CAN_ESR1_FLTCONF_WIDTH                   (2U)
#define CAN_ESR1_FLTCONF(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_FLTCONF_SHIFT)) & CAN_ESR1_FLTCONF_MASK)
#define CAN_ESR1_TX_MASK                         (0x40U)
#define CAN_ESR1_TX_SHIFT                        (6U)
#define CAN_ESR1_TX_WIDTH                        (1U)
#define CAN_ESR1_TX(x)                           (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_TX_SHIFT)) & CAN_ESR1_TX_MASK)
#define CAN_ESR1_IDLE_MASK                       (0x80U)
#define CAN_ESR1_IDLE_SHIFT                      (7U)
#define CAN_ESR1_IDLE_WIDTH                      (1U)
#define CAN_ESR1_IDLE(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_IDLE_SHIFT)) & CAN_ESR1_IDLE_MASK)
#define CAN_ESR1_RXWRN_MASK                      (0x100U)
#define CAN_ESR1_RXWRN_SHIFT                     (8U)
#define CAN_ESR1_RXWRN_WIDTH                     (1U)
#define CAN_ESR1_RXWRN(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_RXWRN_SHIFT)) & CAN_ESR1_RXWRN_MASK)
#define CAN_ESR1_TXWRN_MASK                      (0x200U)
#define CAN_ESR1_TXWRN_SHIFT                     (9U)
#define CAN_ESR1_TXWRN_WIDTH                     (1U)
#define CAN_ESR1_TXWRN(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_TXWRN_SHIFT)) & CAN_ESR1_TXWRN_MASK)
#define CAN_ESR1_STFERR_MASK                     (0x400U)
#define CAN_ESR1_STFERR_SHIFT                    (10U)
#define CAN_ESR1_STFERR_WIDTH                    (1U)
#define CAN_ESR1_STFERR(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_STFERR_SHIFT)) & CAN_ESR1_STFERR_MASK)
#define CAN_ESR1_FRMERR_MASK                     (0x800U)
#define CAN_ESR1_FRMERR_SHIFT                    (11U)
#define CAN_ESR1_FRMERR_WIDTH                    (1U)
#define CAN_ESR1_FRMERR(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_FRMERR_SHIFT)) & CAN_ESR1_FRMERR_MASK)
#define CAN_ESR1_CRCERR_MASK                     (0x1000U)
#define CAN_ESR1_CRCERR_SHIFT                    (12U)
#define CAN_ESR1_CRCERR_WIDTH                    (1U)
#define CAN_ESR1_CRCERR(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_CRCERR_SHIFT)) & CAN_ESR1_CRCERR_MASK)
#define CAN_ESR1_ACKERR_MASK                     (0x2000U)
#define CAN_ESR1_ACKERR_SHIFT                    (13U)
#define CAN_ESR1_ACKERR_WIDTH                    (1U)
#define CAN_ESR1_ACKERR(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_ACKERR_SHIFT)) & CAN_ESR1_ACKERR_MASK)
#define CAN_ESR1_BIT0ERR_MASK                    (0x4000U)
#define CAN_ESR1_BIT0ERR_SHIFT                   (14U)
#define CAN_ESR1_BIT0ERR_WIDTH                   (1U)
#define CAN_ESR1_BIT0ERR(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_BIT0ERR_SHIFT)) & CAN_ESR1_BIT0ERR_MASK)
#define CAN_ESR1_BIT1ERR_MASK                    (0x8000U)
#define CAN_ESR1_BIT1ERR_SHIFT                   (15U)
#define CAN_ESR1_BIT1ERR_WIDTH                   (1U)
#define CAN_ESR1_BIT1ERR(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_BIT1ERR_SHIFT)) & CAN_ESR1_BIT1ERR_MASK)
#define CAN_ESR1_RWRNINT_MASK                    (0x10000U)
#define CAN_ESR1_RWRNINT_SHIFT                   (16U)
#define CAN_ESR1_RWRNINT_WIDTH                   (1U)
#define CAN_ESR1_RWRNINT(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_RWRNINT_SHIFT)) & CAN_ESR1_RWRNINT_MASK)
#define CAN_ESR1_TWRNINT_MASK                    (0x20000U)
#define CAN_ESR1_TWRNINT_SHIFT                   (17U)
#define CAN_ESR1_TWRNINT_WIDTH                   (1U)
#define CAN_ESR1_TWRNINT(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_TWRNINT_SHIFT)) & CAN_ESR1_TWRNINT_MASK)
#define CAN_ESR1_SYNCH_MASK                      (0x40000U)
#define CAN_ESR1_SYNCH_SHIFT                     (18U)
#define CAN_ESR1_SYNCH_WIDTH                     (1U)
#define CAN_ESR1_SYNCH(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_SYNCH_SHIFT)) & CAN_ESR1_SYNCH_MASK)
#define CAN_ESR1_BOFFDONEINT_MASK                (0x80000U)
#define CAN_ESR1_BOFFDONEINT_SHIFT               (19U)
#define CAN_ESR1_BOFFDONEINT_WIDTH               (1U)
#define CAN_ESR1_BOFFDONEINT(x)                  (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_BOFFDONEINT_SHIFT)) & CAN_ESR1_BOFFDONEINT_MASK)
#define CAN_ESR1_ERRINT_FAST_MASK                (0x100000U)
#define CAN_ESR1_ERRINT_FAST_SHIFT               (20U)
#define CAN_ESR1_ERRINT_FAST_WIDTH               (1U)
#define CAN_ESR1_ERRINT_FAST(x)                  (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_ERRINT_FAST_SHIFT)) & CAN_ESR1_ERRINT_FAST_MASK)
#define CAN_ESR1_ERROVR_MASK                     (0x200000U)
#define CAN_ESR1_ERROVR_SHIFT                    (21U)
#define CAN_ESR1_ERROVR_WIDTH                    (1U)
#define CAN_ESR1_ERROVR(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_ERROVR_SHIFT)) & CAN_ESR1_ERROVR_MASK)
#define CAN_ESR1_STFERR_FAST_MASK                (0x4000000U)
#define CAN_ESR1_STFERR_FAST_SHIFT               (26U)
#define CAN_ESR1_STFERR_FAST_WIDTH               (1U)
#define CAN_ESR1_STFERR_FAST(x)                  (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_STFERR_FAST_SHIFT)) & CAN_ESR1_STFERR_FAST_MASK)
#define CAN_ESR1_FRMERR_FAST_MASK                (0x8000000U)
#define CAN_ESR1_FRMERR_FAST_SHIFT               (27U)
#define CAN_ESR1_FRMERR_FAST_WIDTH               (1U)
#define CAN_ESR1_FRMERR_FAST(x)                  (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_FRMERR_FAST_SHIFT)) & CAN_ESR1_FRMERR_FAST_MASK)
#define CAN_ESR1_CRCERR_FAST_MASK                (0x10000000U)
#define CAN_ESR1_CRCERR_FAST_SHIFT               (28U)
#define CAN_ESR1_CRCERR_FAST_WIDTH               (1U)
#define CAN_ESR1_CRCERR_FAST(x)                  (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_CRCERR_FAST_SHIFT)) & CAN_ESR1_CRCERR_FAST_MASK)
#define CAN_ESR1_BIT0ERR_FAST_MASK               (0x40000000U)
#define CAN_ESR1_BIT0ERR_FAST_SHIFT              (30U)
#define CAN_ESR1_BIT0ERR_FAST_WIDTH              (1U)
#define CAN_ESR1_BIT0ERR_FAST(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_BIT0ERR_FAST_SHIFT)) & CAN_ESR1_BIT0ERR_FAST_MASK)
#define CAN_ESR1_BIT1ERR_FAST_MASK               (0x80000000U)
#define CAN_ESR1_BIT1ERR_FAST_SHIFT              (31U)
#define CAN_ESR1_BIT1ERR_FAST_WIDTH              (1U)
#define CAN_ESR1_BIT1ERR_FAST(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_BIT1ERR_FAST_SHIFT)) & CAN_ESR1_BIT1ERR_FAST_MASK)
/*! @} */

/*! @name IMASK2 - Interrupt Masks 2 register */
/*! @{ */
#define CAN_IMASK2_BUF63TO32M_MASK               (0xFFFFFFFFU)
#define CAN_IMASK2_BUF63TO32M_SHIFT              (0U)
#define CAN_IMASK2_BUF63TO32M_WIDTH              (32U)
#define CAN_IMASK2_BUF63TO32M(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_IMASK2_BUF63TO32M_SHIFT)) & CAN_IMASK2_BUF63TO32M_MASK)
/*! @} */

/*! @name IMASK1 - Interrupt Masks 1 register */
/*! @{ */
#define CAN_IMASK1_BUF31TO0M_MASK                (0xFFFFFFFFU)
#define CAN_IMASK1_BUF31TO0M_SHIFT               (0U)
#define CAN_IMASK1_BUF31TO0M_WIDTH               (32U)
#define CAN_IMASK1_BUF31TO0M(x)                  (((uint32_t)(((uint32_t)(x)) << CAN_IMASK1_BUF31TO0M_SHIFT)) & CAN_IMASK1_BUF31TO0M_MASK)
/*! @} */

/*! @name IFLAG2 - Interrupt Flags 2 register */
/*! @{ */
#define CAN_IFLAG2_BUF63TO32I_MASK               (0xFFFFFFFFU)
#define CAN_IFLAG2_BUF63TO32I_SHIFT              (0U)
#define CAN_IFLAG2_BUF63TO32I_WIDTH              (32U)
#define CAN_IFLAG2_BUF63TO32I(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_IFLAG2_BUF63TO32I_SHIFT)) & CAN_IFLAG2_BUF63TO32I_MASK)
/*! @} */

/*! @name IFLAG1 - Interrupt Flags 1 register */
/*! @{ */
#define CAN_IFLAG1_BUF0I_MASK                    (0x1U)
#define CAN_IFLAG1_BUF0I_SHIFT                   (0U)
#define CAN_IFLAG1_BUF0I_WIDTH                   (1U)
#define CAN_IFLAG1_BUF0I(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_IFLAG1_BUF0I_SHIFT)) & CAN_IFLAG1_BUF0I_MASK)
#define CAN_IFLAG1_BUF4TO1I_MASK                 (0x1EU)
#define CAN_IFLAG1_BUF4TO1I_SHIFT                (1U)
#define CAN_IFLAG1_BUF4TO1I_WIDTH                (4U)
#define CAN_IFLAG1_BUF4TO1I(x)                   (((uint32_t)(((uint32_t)(x)) << CAN_IFLAG1_BUF4TO1I_SHIFT)) & CAN_IFLAG1_BUF4TO1I_MASK)
#define CAN_IFLAG1_BUF5I_MASK                    (0x20U)
#define CAN_IFLAG1_BUF5I_SHIFT                   (5U)
#define CAN_IFLAG1_BUF5I_WIDTH                   (1U)
#define CAN_IFLAG1_BUF5I(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_IFLAG1_BUF5I_SHIFT)) & CAN_IFLAG1_BUF5I_MASK)
#define CAN_IFLAG1_BUF6I_MASK                    (0x40U)
#define CAN_IFLAG1_BUF6I_SHIFT                   (6U)
#define CAN_IFLAG1_BUF6I_WIDTH                   (1U)
#define CAN_IFLAG1_BUF6I(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_IFLAG1_BUF6I_SHIFT)) & CAN_IFLAG1_BUF6I_MASK)
#define CAN_IFLAG1_BUF7I_MASK                    (0x80U)
#define CAN_IFLAG1_BUF7I_SHIFT                   (7U)
#define CAN_IFLAG1_BUF7I_WIDTH                   (1U)
#define CAN_IFLAG1_BUF7I(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_IFLAG1_BUF7I_SHIFT)) & CAN_IFLAG1_BUF7I_MASK)
#define CAN_IFLAG1_BUF31TO8I_MASK                (0xFFFFFF00U)
#define CAN_IFLAG1_BUF31TO8I_SHIFT               (8U)
#define CAN_IFLAG1_BUF31TO8I_WIDTH               (24U)
#define CAN_IFLAG1_BUF31TO8I(x)                  (((uint32_t)(((uint32_t)(x)) << CAN_IFLAG1_BUF31TO8I_SHIFT)) & CAN_IFLAG1_BUF31TO8I_MASK)
/*! @} */

/*! @name CTRL2 - Control 2 register */
/*! @{ */
#define CAN_CTRL2_EDFLTDIS_MASK                  (0x800U)
#define CAN_CTRL2_EDFLTDIS_SHIFT                 (11U)
#define CAN_CTRL2_EDFLTDIS_WIDTH                 (1U)
#define CAN_CTRL2_EDFLTDIS(x)                    (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_EDFLTDIS_SHIFT)) & CAN_CTRL2_EDFLTDIS_MASK)
#define CAN_CTRL2_ISOCANFDEN_MASK                (0x1000U)
#define CAN_CTRL2_ISOCANFDEN_SHIFT               (12U)
#define CAN_CTRL2_ISOCANFDEN_WIDTH               (1U)
#define CAN_CTRL2_ISOCANFDEN(x)                  (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_ISOCANFDEN_SHIFT)) & CAN_CTRL2_ISOCANFDEN_MASK)
#define CAN_CTRL2_PREXCEN_MASK                   (0x4000U)
#define CAN_CTRL2_PREXCEN_SHIFT                  (14U)
#define CAN_CTRL2_PREXCEN_WIDTH                  (1U)
#define CAN_CTRL2_PREXCEN(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_PREXCEN_SHIFT)) & CAN_CTRL2_PREXCEN_MASK)
#define CAN_CTRL2_TIMER_SRC_MASK                 (0x8000U)
#define CAN_CTRL2_TIMER_SRC_SHIFT                (15U)
#define CAN_CTRL2_TIMER_SRC_WIDTH                (1U)
#define CAN_CTRL2_TIMER_SRC(x)                   (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_TIMER_SRC_SHIFT)) & CAN_CTRL2_TIMER_SRC_MASK)
#define CAN_CTRL2_EACEN_MASK                     (0x10000U)
#define CAN_CTRL2_EACEN_SHIFT                    (16U)
#define CAN_CTRL2_EACEN_WIDTH                    (1U)
#define CAN_CTRL2_EACEN(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_EACEN_SHIFT)) & CAN_CTRL2_EACEN_MASK)
#define CAN_CTRL2_RRS_MASK                       (0x20000U)
#define CAN_CTRL2_RRS_SHIFT                      (17U)
#define CAN_CTRL2_RRS_WIDTH                      (1U)
#define CAN_CTRL2_RRS(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_RRS_SHIFT)) & CAN_CTRL2_RRS_MASK)
#define CAN_CTRL2_MRP_MASK                       (0x40000U)
#define CAN_CTRL2_MRP_SHIFT                      (18U)
#define CAN_CTRL2_MRP_WIDTH                      (1U)
#define CAN_CTRL2_MRP(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_MRP_SHIFT)) & CAN_CTRL2_MRP_MASK)
#define CAN_CTRL2_TASD_MASK                      (0xF80000U)
#define CAN_CTRL2_TASD_SHIFT                     (19U)
#define CAN_CTRL2_TASD_WIDTH                     (5U)
#define CAN_CTRL2_TASD(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_TASD_SHIFT)) & CAN_CTRL2_TASD_MASK)
#define CAN_CTRL2_RFFN_MASK                      (0xF000000U)
#define CAN_CTRL2_RFFN_SHIFT                     (24U)
#define CAN_CTRL2_RFFN_WIDTH                     (4U)
#define CAN_CTRL2_RFFN(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_RFFN_SHIFT)) & CAN_CTRL2_RFFN_MASK)
#define CAN_CTRL2_BOFFDONEMSK_MASK               (0x40000000U)
#define CAN_CTRL2_BOFFDONEMSK_SHIFT              (30U)
#define CAN_CTRL2_BOFFDONEMSK_WIDTH              (1U)
#define CAN_CTRL2_BOFFDONEMSK(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_BOFFDONEMSK_SHIFT)) & CAN_CTRL2_BOFFDONEMSK_MASK)
#define CAN_CTRL2_ERRMSK_FAST_MASK               (0x80000000U)
#define CAN_CTRL2_ERRMSK_FAST_SHIFT              (31U)
#define CAN_CTRL2_ERRMSK_FAST_WIDTH              (1U)
#define CAN_CTRL2_ERRMSK_FAST(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_ERRMSK_FAST_SHIFT)) & CAN_CTRL2_ERRMSK_FAST_MASK)
/*! @} */

/*! @name ESR2 - Error and Status 2 register */
/*! @{ */
#define CAN_ESR2_IMB_MASK                        (0x2000U)
#define CAN_ESR2_IMB_SHIFT                       (13U)
#define CAN_ESR2_IMB_WIDTH                       (1U)
#define CAN_ESR2_IMB(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_ESR2_IMB_SHIFT)) & CAN_ESR2_IMB_MASK)
#define CAN_ESR2_VPS_MASK                        (0x4000U)
#define CAN_ESR2_VPS_SHIFT                       (14U)
#define CAN_ESR2_VPS_WIDTH                       (1U)
#define CAN_ESR2_VPS(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_ESR2_VPS_SHIFT)) & CAN_ESR2_VPS_MASK)
#define CAN_ESR2_LPTM_MASK                       (0x7F0000U)
#define CAN_ESR2_LPTM_SHIFT                      (16U)
#define CAN_ESR2_LPTM_WIDTH                      (7U)
#define CAN_ESR2_LPTM(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_ESR2_LPTM_SHIFT)) & CAN_ESR2_LPTM_MASK)
/*! @} */

/*! @name CRCR - CRC register */
/*! @{ */
#define CAN_CRCR_TXCRC_MASK                      (0x7FFFU)
#define CAN_CRCR_TXCRC_SHIFT                     (0U)
#define CAN_CRCR_TXCRC_WIDTH                     (15U)
#define CAN_CRCR_TXCRC(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_CRCR_TXCRC_SHIFT)) & CAN_CRCR_TXCRC_MASK)
#define CAN_CRCR_MBCRC_MASK                      (0x7F0000U)
#define CAN_CRCR_MBCRC_SHIFT                     (16U)
#define CAN_CRCR_MBCRC_WIDTH                     (7U)
#define CAN_CRCR_MBCRC(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_CRCR_MBCRC_SHIFT)) & CAN_CRCR_MBCRC_MASK)
/*! @} */

/*! @name RXFGMASK - Rx FIFO Global Mask register */
/*! @{ */
#define CAN_RXFGMASK_FGM_MASK                    (0xFFFFFFFFU)
#define CAN_RXFGMASK_FGM_SHIFT                   (0U)
#define CAN_RXFGMASK_FGM_WIDTH                   (32U)
#define CAN_RXFGMASK_FGM(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_RXFGMASK_FGM_SHIFT)) & CAN_RXFGMASK_FGM_MASK)
/*! @} */

/*! @name RXFIR - Rx FIFO Information register */
/*! @{ */
#define CAN_RXFIR_IDHIT_MASK                     (0x1FFU)
#define CAN_RXFIR_IDHIT_SHIFT                    (0U)
#define CAN_RXFIR_IDHIT_WIDTH                    (9U)
#define CAN_RXFIR_IDHIT(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_RXFIR_IDHIT_SHIFT)) & CAN_RXFIR_IDHIT_MASK)
/*! @} */

/*! @name CBT - CAN Bit Timing register */
/*! @{ */
#define CAN_CBT_EPSEG2_MASK                      (0x1FU)
#define CAN_CBT_EPSEG2_SHIFT                     (0U)
#define CAN_CBT_EPSEG2_WIDTH                     (5U)
#define CAN_CBT_EPSEG2(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_CBT_EPSEG2_SHIFT)) & CAN_CBT_EPSEG2_MASK)
#define CAN_CBT_EPSEG1_MASK                      (0x3E0U)
#define CAN_CBT_EPSEG1_SHIFT                     (5U)
#define CAN_CBT_EPSEG1_WIDTH                     (5U)
#define CAN_CBT_EPSEG1(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_CBT_EPSEG1_SHIFT)) & CAN_CBT_EPSEG1_MASK)
#define CAN_CBT_EPROPSEG_MASK                    (0xFC00U)
#define CAN_CBT_EPROPSEG_SHIFT                   (10U)
#define CAN_CBT_EPROPSEG_WIDTH                   (6U)
#define CAN_CBT_EPROPSEG(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_CBT_EPROPSEG_SHIFT)) & CAN_CBT_EPROPSEG_MASK)
#define CAN_CBT_ERJW_MASK                        (0x1F0000U)
#define CAN_CBT_ERJW_SHIFT                       (16U)
#define CAN_CBT_ERJW_WIDTH                       (5U)
#define CAN_CBT_ERJW(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_CBT_ERJW_SHIFT)) & CAN_CBT_ERJW_MASK)
#define CAN_CBT_EPRESDIV_MASK                    (0x7FE00000U)
#define CAN_CBT_EPRESDIV_SHIFT                   (21U)
#define CAN_CBT_EPRESDIV_WIDTH                   (10U)
#define CAN_CBT_EPRESDIV(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_CBT_EPRESDIV_SHIFT)) & CAN_CBT_EPRESDIV_MASK)
#define CAN_CBT_BTF_MASK                         (0x80000000U)
#define CAN_CBT_BTF_SHIFT                        (31U)
#define CAN_CBT_BTF_WIDTH                        (1U)
#define CAN_CBT_BTF(x)                           (((uint32_t)(((uint32_t)(x)) << CAN_CBT_BTF_SHIFT)) & CAN_CBT_BTF_MASK)
/*! @} */

/*! @name RAMn - Embedded RAM */
/*! @{ */
#define CAN_RAMn_DATA_BYTE_3_MASK                (0xFFU)
#define CAN_RAMn_DATA_BYTE_3_SHIFT               (0U)
#define CAN_RAMn_DATA_BYTE_3_WIDTH               (8U)
#define CAN_RAMn_DATA_BYTE_3(x)                  (((uint32_t)(((uint32_t)(x)) << CAN_RAMn_DATA_BYTE_3_SHIFT)) & CAN_RAMn_DATA_BYTE_3_MASK)
#define CAN_RAMn_DATA_BYTE_2_MASK                (0xFF00U)
#define CAN_RAMn_DATA_BYTE_2_SHIFT               (8U)
#define CAN_RAMn_DATA_BYTE_2_WIDTH               (8U)
#define CAN_RAMn_DATA_BYTE_2(x)                  (((uint32_t)(((uint32_t)(x)) << CAN_RAMn_DATA_BYTE_2_SHIFT)) & CAN_RAMn_DATA_BYTE_2_MASK)
#define CAN_RAMn_DATA_BYTE_1_MASK                (0xFF0000U)
#define CAN_RAMn_DATA_BYTE_1_SHIFT               (16U)
#define CAN_RAMn_DATA_BYTE_1_WIDTH               (8U)
#define CAN_RAMn_DATA_BYTE_1(x)                  (((uint32_t)(((uint32_t)(x)) << CAN_RAMn_DATA_BYTE_1_SHIFT)) & CAN_RAMn_DATA_BYTE_1_MASK)
#define CAN_RAMn_DATA_BYTE_0_MASK                (0xFF000000U)
#define CAN_RAMn_DATA_BYTE_0_SHIFT               (24U)
#define CAN_RAMn_DATA_BYTE_0_WIDTH               (8U)
#define CAN_RAMn_DATA_BYTE_0(x)                  (((uint32_t)(((uint32_t)(x)) << CAN_RAMn_DATA_BYTE_0_SHIFT)) & CAN_RAMn_DATA_BYTE_0_MASK)
/*! @} */

/*! @name RXIMR - Rx Individual Mask registers */
/*! @{ */
#define CAN_RXIMR_MI_MASK                        (0xFFFFFFFFU)
#define CAN_RXIMR_MI_SHIFT                       (0U)
#define CAN_RXIMR_MI_WIDTH                       (32U)
#define CAN_RXIMR_MI(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_RXIMR_MI_SHIFT)) & CAN_RXIMR_MI_MASK)
/*! @} */

/*! @name CTRL1_PN - Pretended Networking Control 1 register */
/*! @{ */
#define CAN_CTRL1_PN_FCS_MASK                    (0x3U)
#define CAN_CTRL1_PN_FCS_SHIFT                   (0U)
#define CAN_CTRL1_PN_FCS_WIDTH                   (2U)
#define CAN_CTRL1_PN_FCS(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PN_FCS_SHIFT)) & CAN_CTRL1_PN_FCS_MASK)
#define CAN_CTRL1_PN_IDFS_MASK                   (0xCU)
#define CAN_CTRL1_PN_IDFS_SHIFT                  (2U)
#define CAN_CTRL1_PN_IDFS_WIDTH                  (2U)
#define CAN_CTRL1_PN_IDFS(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PN_IDFS_SHIFT)) & CAN_CTRL1_PN_IDFS_MASK)
#define CAN_CTRL1_PN_PLFS_MASK                   (0x30U)
#define CAN_CTRL1_PN_PLFS_SHIFT                  (4U)
#define CAN_CTRL1_PN_PLFS_WIDTH                  (2U)
#define CAN_CTRL1_PN_PLFS(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PN_PLFS_SHIFT)) & CAN_CTRL1_PN_PLFS_MASK)
#define CAN_CTRL1_PN_NMATCH_MASK                 (0xFF00U)
#define CAN_CTRL1_PN_NMATCH_SHIFT                (8U)
#define CAN_CTRL1_PN_NMATCH_WIDTH                (8U)
#define CAN_CTRL1_PN_NMATCH(x)                   (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PN_NMATCH_SHIFT)) & CAN_CTRL1_PN_NMATCH_MASK)
#define CAN_CTRL1_PN_WUMF_MSK_MASK               (0x10000U)
#define CAN_CTRL1_PN_WUMF_MSK_SHIFT              (16U)
#define CAN_CTRL1_PN_WUMF_MSK_WIDTH              (1U)
#define CAN_CTRL1_PN_WUMF_MSK(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PN_WUMF_MSK_SHIFT)) & CAN_CTRL1_PN_WUMF_MSK_MASK)
#define CAN_CTRL1_PN_WTOF_MSK_MASK               (0x20000U)
#define CAN_CTRL1_PN_WTOF_MSK_SHIFT              (17U)
#define CAN_CTRL1_PN_WTOF_MSK_WIDTH              (1U)
#define CAN_CTRL1_PN_WTOF_MSK(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PN_WTOF_MSK_SHIFT)) & CAN_CTRL1_PN_WTOF_MSK_MASK)
/*! @} */

/*! @name CTRL2_PN - Pretended Networking Control 2 register */
/*! @{ */
#define CAN_CTRL2_PN_MATCHTO_MASK                (0xFFFFU)
#define CAN_CTRL2_PN_MATCHTO_SHIFT               (0U)
#define CAN_CTRL2_PN_MATCHTO_WIDTH               (16U)
#define CAN_CTRL2_PN_MATCHTO(x)                  (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_PN_MATCHTO_SHIFT)) & CAN_CTRL2_PN_MATCHTO_MASK)
/*! @} */

/*! @name WU_MTC - Pretended Networking Wake Up Match register */
/*! @{ */
#define CAN_WU_MTC_MCOUNTER_MASK                 (0xFF00U)
#define CAN_WU_MTC_MCOUNTER_SHIFT                (8U)
#define CAN_WU_MTC_MCOUNTER_WIDTH                (8U)
#define CAN_WU_MTC_MCOUNTER(x)                   (((uint32_t)(((uint32_t)(x)) << CAN_WU_MTC_MCOUNTER_SHIFT)) & CAN_WU_MTC_MCOUNTER_MASK)
#define CAN_WU_MTC_WUMF_MASK                     (0x10000U)
#define CAN_WU_MTC_WUMF_SHIFT                    (16U)
#define CAN_WU_MTC_WUMF_WIDTH                    (1U)
#define CAN_WU_MTC_WUMF(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_WU_MTC_WUMF_SHIFT)) & CAN_WU_MTC_WUMF_MASK)
#define CAN_WU_MTC_WTOF_MASK                     (0x20000U)
#define CAN_WU_MTC_WTOF_SHIFT                    (17U)
#define CAN_WU_MTC_WTOF_WIDTH                    (1U)
#define CAN_WU_MTC_WTOF(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_WU_MTC_WTOF_SHIFT)) & CAN_WU_MTC_WTOF_MASK)
/*! @} */

/*! @name FLT_ID1 - Pretended Networking ID Filter 1 register */
/*! @{ */
#define CAN_FLT_ID1_FLT_ID1_MASK                 (0x1FFFFFFFU)
#define CAN_FLT_ID1_FLT_ID1_SHIFT                (0U)
#define CAN_FLT_ID1_FLT_ID1_WIDTH                (29U)
#define CAN_FLT_ID1_FLT_ID1(x)                   (((uint32_t)(((uint32_t)(x)) << CAN_FLT_ID1_FLT_ID1_SHIFT)) & CAN_FLT_ID1_FLT_ID1_MASK)
#define CAN_FLT_ID1_FLT_RTR_MASK                 (0x20000000U)
#define CAN_FLT_ID1_FLT_RTR_SHIFT                (29U)
#define CAN_FLT_ID1_FLT_RTR_WIDTH                (1U)
#define CAN_FLT_ID1_FLT_RTR(x)                   (((uint32_t)(((uint32_t)(x)) << CAN_FLT_ID1_FLT_RTR_SHIFT)) & CAN_FLT_ID1_FLT_RTR_MASK)
#define CAN_FLT_ID1_FLT_IDE_MASK                 (0x40000000U)
#define CAN_FLT_ID1_FLT_IDE_SHIFT                (30U)
#define CAN_FLT_ID1_FLT_IDE_WIDTH                (1U)
#define CAN_FLT_ID1_FLT_IDE(x)                   (((uint32_t)(((uint32_t)(x)) << CAN_FLT_ID1_FLT_IDE_SHIFT)) & CAN_FLT_ID1_FLT_IDE_MASK)
/*! @} */

/*! @name FLT_DLC - Pretended Networking DLC Filter register */
/*! @{ */
#define CAN_FLT_DLC_FLT_DLC_HI_MASK              (0xFU)
#define CAN_FLT_DLC_FLT_DLC_HI_SHIFT             (0U)
#define CAN_FLT_DLC_FLT_DLC_HI_WIDTH             (4U)
#define CAN_FLT_DLC_FLT_DLC_HI(x)                (((uint32_t)(((uint32_t)(x)) << CAN_FLT_DLC_FLT_DLC_HI_SHIFT)) & CAN_FLT_DLC_FLT_DLC_HI_MASK)
#define CAN_FLT_DLC_FLT_DLC_LO_MASK              (0xF0000U)
#define CAN_FLT_DLC_FLT_DLC_LO_SHIFT             (16U)
#define CAN_FLT_DLC_FLT_DLC_LO_WIDTH             (4U)
#define CAN_FLT_DLC_FLT_DLC_LO(x)                (((uint32_t)(((uint32_t)(x)) << CAN_FLT_DLC_FLT_DLC_LO_SHIFT)) & CAN_FLT_DLC_FLT_DLC_LO_MASK)
/*! @} */

/*! @name PL1_LO - Pretended Networking Payload Low Filter 1 register */
/*! @{ */
#define CAN_PL1_LO_Data_byte_3_MASK              (0xFFU)
#define CAN_PL1_LO_Data_byte_3_SHIFT             (0U)
#define CAN_PL1_LO_Data_byte_3_WIDTH             (8U)
#define CAN_PL1_LO_Data_byte_3(x)                (((uint32_t)(((uint32_t)(x)) << CAN_PL1_LO_Data_byte_3_SHIFT)) & CAN_PL1_LO_Data_byte_3_MASK)
#define CAN_PL1_LO_Data_byte_2_MASK              (0xFF00U)
#define CAN_PL1_LO_Data_byte_2_SHIFT             (8U)
#define CAN_PL1_LO_Data_byte_2_WIDTH             (8U)
#define CAN_PL1_LO_Data_byte_2(x)                (((uint32_t)(((uint32_t)(x)) << CAN_PL1_LO_Data_byte_2_SHIFT)) & CAN_PL1_LO_Data_byte_2_MASK)
#define CAN_PL1_LO_Data_byte_1_MASK              (0xFF0000U)
#define CAN_PL1_LO_Data_byte_1_SHIFT             (16U)
#define CAN_PL1_LO_Data_byte_1_WIDTH             (8U)
#define CAN_PL1_LO_Data_byte_1(x)                (((uint32_t)(((uint32_t)(x)) << CAN_PL1_LO_Data_byte_1_SHIFT)) & CAN_PL1_LO_Data_byte_1_MASK)
#define CAN_PL1_LO_Data_byte_0_MASK              (0xFF000000U)
#define CAN_PL1_LO_Data_byte_0_SHIFT             (24U)
#define CAN_PL1_LO_Data_byte_0_WIDTH             (8U)
#define CAN_PL1_LO_Data_byte_0(x)                (((uint32_t)(((uint32_t)(x)) << CAN_PL1_LO_Data_byte_0_SHIFT)) & CAN_PL1_LO_Data_byte_0_MASK)
/*! @} */

/*! @name PL1_HI - Pretended Networking Payload High Filter 1 register */
/*! @{ */
#define CAN_PL1_HI_Data_byte_7_MASK              (0xFFU)
#define CAN_PL1_HI_Data_byte_7_SHIFT             (0U)
#define CAN_PL1_HI_Data_byte_7_WIDTH             (8U)
#define CAN_PL1_HI_Data_byte_7(x)                (((uint32_t)(((uint32_t)(x)) << CAN_PL1_HI_Data_byte_7_SHIFT)) & CAN_PL1_HI_Data_byte_7_MASK)
#define CAN_PL1_HI_Data_byte_6_MASK              (0xFF00U)
#define CAN_PL1_HI_Data_byte_6_SHIFT             (8U)
#define CAN_PL1_HI_Data_byte_6_WIDTH             (8U)
#define CAN_PL1_HI_Data_byte_6(x)                (((uint32_t)(((uint32_t)(x)) << CAN_PL1_HI_Data_byte_6_SHIFT)) & CAN_PL1_HI_Data_byte_6_MASK)
#define CAN_PL1_HI_Data_byte_5_MASK              (0xFF0000U)
#define CAN_PL1_HI_Data_byte_5_SHIFT             (16U)
#define CAN_PL1_HI_Data_byte_5_WIDTH             (8U)
#define CAN_PL1_HI_Data_byte_5(x)                (((uint32_t)(((uint32_t)(x)) << CAN_PL1_HI_Data_byte_5_SHIFT)) & CAN_PL1_HI_Data_byte_5_MASK)
#define CAN_PL1_HI_Data_byte_4_MASK              (0xFF000000U)
#define CAN_PL1_HI_Data_byte_4_SHIFT             (24U)
#define CAN_PL1_HI_Data_byte_4_WIDTH             (8U)
#define CAN_PL1_HI_Data_byte_4(x)                (((uint32_t)(((uint32_t)(x)) << CAN_PL1_HI_Data_byte_4_SHIFT)) & CAN_PL1_HI_Data_byte_4_MASK)
/*! @} */

/*! @name FLT_ID2_IDMASK - Pretended Networking ID Filter 2 Register / ID Mask register */
/*! @{ */
#define CAN_FLT_ID2_IDMASK_FLT_ID2_IDMASK_MASK   (0x1FFFFFFFU)
#define CAN_FLT_ID2_IDMASK_FLT_ID2_IDMASK_SHIFT  (0U)
#define CAN_FLT_ID2_IDMASK_FLT_ID2_IDMASK_WIDTH  (29U)
#define CAN_FLT_ID2_IDMASK_FLT_ID2_IDMASK(x)     (((uint32_t)(((uint32_t)(x)) << CAN_FLT_ID2_IDMASK_FLT_ID2_IDMASK_SHIFT)) & CAN_FLT_ID2_IDMASK_FLT_ID2_IDMASK_MASK)
#define CAN_FLT_ID2_IDMASK_RTR_MSK_MASK          (0x20000000U)
#define CAN_FLT_ID2_IDMASK_RTR_MSK_SHIFT         (29U)
#define CAN_FLT_ID2_IDMASK_RTR_MSK_WIDTH         (1U)
#define CAN_FLT_ID2_IDMASK_RTR_MSK(x)            (((uint32_t)(((uint32_t)(x)) << CAN_FLT_ID2_IDMASK_RTR_MSK_SHIFT)) & CAN_FLT_ID2_IDMASK_RTR_MSK_MASK)
#define CAN_FLT_ID2_IDMASK_IDE_MSK_MASK          (0x40000000U)
#define CAN_FLT_ID2_IDMASK_IDE_MSK_SHIFT         (30U)
#define CAN_FLT_ID2_IDMASK_IDE_MSK_WIDTH         (1U)
#define CAN_FLT_ID2_IDMASK_IDE_MSK(x)            (((uint32_t)(((uint32_t)(x)) << CAN_FLT_ID2_IDMASK_IDE_MSK_SHIFT)) & CAN_FLT_ID2_IDMASK_IDE_MSK_MASK)
/*! @} */

/*! @name PL2_PLMASK_LO - Pretended Networking Payload Low Filter 2 Register / Payload Low Mask register */
/*! @{ */
#define CAN_PL2_PLMASK_LO_Data_byte_3_MASK       (0xFFU)
#define CAN_PL2_PLMASK_LO_Data_byte_3_SHIFT      (0U)
#define CAN_PL2_PLMASK_LO_Data_byte_3_WIDTH      (8U)
#define CAN_PL2_PLMASK_LO_Data_byte_3(x)         (((uint32_t)(((uint32_t)(x)) << CAN_PL2_PLMASK_LO_Data_byte_3_SHIFT)) & CAN_PL2_PLMASK_LO_Data_byte_3_MASK)
#define CAN_PL2_PLMASK_LO_Data_byte_2_MASK       (0xFF00U)
#define CAN_PL2_PLMASK_LO_Data_byte_2_SHIFT      (8U)
#define CAN_PL2_PLMASK_LO_Data_byte_2_WIDTH      (8U)
#define CAN_PL2_PLMASK_LO_Data_byte_2(x)         (((uint32_t)(((uint32_t)(x)) << CAN_PL2_PLMASK_LO_Data_byte_2_SHIFT)) & CAN_PL2_PLMASK_LO_Data_byte_2_MASK)
#define CAN_PL2_PLMASK_LO_Data_byte_1_MASK       (0xFF0000U)
#define CAN_PL2_PLMASK_LO_Data_byte_1_SHIFT      (16U)
#define CAN_PL2_PLMASK_LO_Data_byte_1_WIDTH      (8U)
#define CAN_PL2_PLMASK_LO_Data_byte_1(x)         (((uint32_t)(((uint32_t)(x)) << CAN_PL2_PLMASK_LO_Data_byte_1_SHIFT)) & CAN_PL2_PLMASK_LO_Data_byte_1_MASK)
#define CAN_PL2_PLMASK_LO_Data_byte_0_MASK       (0xFF000000U)
#define CAN_PL2_PLMASK_LO_Data_byte_0_SHIFT      (24U)
#define CAN_PL2_PLMASK_LO_Data_byte_0_WIDTH      (8U)
#define CAN_PL2_PLMASK_LO_Data_byte_0(x)         (((uint32_t)(((uint32_t)(x)) << CAN_PL2_PLMASK_LO_Data_byte_0_SHIFT)) & CAN_PL2_PLMASK_LO_Data_byte_0_MASK)
/*! @} */

/*! @name PL2_PLMASK_HI - Pretended Networking Payload High Filter 2 low order bits / Payload High Mask register */
/*! @{ */
#define CAN_PL2_PLMASK_HI_Data_byte_7_MASK       (0xFFU)
#define CAN_PL2_PLMASK_HI_Data_byte_7_SHIFT      (0U)
#define CAN_PL2_PLMASK_HI_Data_byte_7_WIDTH      (8U)
#define CAN_PL2_PLMASK_HI_Data_byte_7(x)         (((uint32_t)(((uint32_t)(x)) << CAN_PL2_PLMASK_HI_Data_byte_7_SHIFT)) & CAN_PL2_PLMASK_HI_Data_byte_7_MASK)
#define CAN_PL2_PLMASK_HI_Data_byte_6_MASK       (0xFF00U)
#define CAN_PL2_PLMASK_HI_Data_byte_6_SHIFT      (8U)
#define CAN_PL2_PLMASK_HI_Data_byte_6_WIDTH      (8U)
#define CAN_PL2_PLMASK_HI_Data_byte_6(x)         (((uint32_t)(((uint32_t)(x)) << CAN_PL2_PLMASK_HI_Data_byte_6_SHIFT)) & CAN_PL2_PLMASK_HI_Data_byte_6_MASK)
#define CAN_PL2_PLMASK_HI_Data_byte_5_MASK       (0xFF0000U)
#define CAN_PL2_PLMASK_HI_Data_byte_5_SHIFT      (16U)
#define CAN_PL2_PLMASK_HI_Data_byte_5_WIDTH      (8U)
#define CAN_PL2_PLMASK_HI_Data_byte_5(x)         (((uint32_t)(((uint32_t)(x)) << CAN_PL2_PLMASK_HI_Data_byte_5_SHIFT)) & CAN_PL2_PLMASK_HI_Data_byte_5_MASK)
#define CAN_PL2_PLMASK_HI_Data_byte_4_MASK       (0xFF000000U)
#define CAN_PL2_PLMASK_HI_Data_byte_4_SHIFT      (24U)
#define CAN_PL2_PLMASK_HI_Data_byte_4_WIDTH      (8U)
#define CAN_PL2_PLMASK_HI_Data_byte_4(x)         (((uint32_t)(((uint32_t)(x)) << CAN_PL2_PLMASK_HI_Data_byte_4_SHIFT)) & CAN_PL2_PLMASK_HI_Data_byte_4_MASK)
/*! @} */

/*! @name WMBn_CS - Wake Up Message Buffer register for C/S */
/*! @{ */
#define CAN_WMBn_CS_DLC_MASK                     (0xF0000U)
#define CAN_WMBn_CS_DLC_SHIFT                    (16U)
#define CAN_WMBn_CS_DLC_WIDTH                    (4U)
#define CAN_WMBn_CS_DLC(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_WMBn_CS_DLC_SHIFT)) & CAN_WMBn_CS_DLC_MASK)
#define CAN_WMBn_CS_RTR_MASK                     (0x100000U)
#define CAN_WMBn_CS_RTR_SHIFT                    (20U)
#define CAN_WMBn_CS_RTR_WIDTH                    (1U)
#define CAN_WMBn_CS_RTR(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_WMBn_CS_RTR_SHIFT)) & CAN_WMBn_CS_RTR_MASK)
#define CAN_WMBn_CS_IDE_MASK                     (0x200000U)
#define CAN_WMBn_CS_IDE_SHIFT                    (21U)
#define CAN_WMBn_CS_IDE_WIDTH                    (1U)
#define CAN_WMBn_CS_IDE(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_WMBn_CS_IDE_SHIFT)) & CAN_WMBn_CS_IDE_MASK)
#define CAN_WMBn_CS_SRR_MASK                     (0x400000U)
#define CAN_WMBn_CS_SRR_SHIFT                    (22U)
#define CAN_WMBn_CS_SRR_WIDTH                    (1U)
#define CAN_WMBn_CS_SRR(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_WMBn_CS_SRR_SHIFT)) & CAN_WMBn_CS_SRR_MASK)
/*! @} */

/*! @name WMBn_ID - Wake Up Message Buffer Register for ID */
/*! @{ */
#define CAN_WMBn_ID_ID_MASK                      (0x1FFFFFFFU)
#define CAN_WMBn_ID_ID_SHIFT                     (0U)
#define CAN_WMBn_ID_ID_WIDTH                     (29U)
#define CAN_WMBn_ID_ID(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_WMBn_ID_ID_SHIFT)) & CAN_WMBn_ID_ID_MASK)
/*! @} */

/*! @name WMBn_D03 - Wake Up Message Buffer Register for Data 0-3 */
/*! @{ */
#define CAN_WMBn_D03_Data_byte_3_MASK            (0xFFU)
#define CAN_WMBn_D03_Data_byte_3_SHIFT           (0U)
#define CAN_WMBn_D03_Data_byte_3_WIDTH           (8U)
#define CAN_WMBn_D03_Data_byte_3(x)              (((uint32_t)(((uint32_t)(x)) << CAN_WMBn_D03_Data_byte_3_SHIFT)) & CAN_WMBn_D03_Data_byte_3_MASK)
#define CAN_WMBn_D03_Data_byte_2_MASK            (0xFF00U)
#define CAN_WMBn_D03_Data_byte_2_SHIFT           (8U)
#define CAN_WMBn_D03_Data_byte_2_WIDTH           (8U)
#define CAN_WMBn_D03_Data_byte_2(x)              (((uint32_t)(((uint32_t)(x)) << CAN_WMBn_D03_Data_byte_2_SHIFT)) & CAN_WMBn_D03_Data_byte_2_MASK)
#define CAN_WMBn_D03_Data_byte_1_MASK            (0xFF0000U)
#define CAN_WMBn_D03_Data_byte_1_SHIFT           (16U)
#define CAN_WMBn_D03_Data_byte_1_WIDTH           (8U)
#define CAN_WMBn_D03_Data_byte_1(x)              (((uint32_t)(((uint32_t)(x)) << CAN_WMBn_D03_Data_byte_1_SHIFT)) & CAN_WMBn_D03_Data_byte_1_MASK)
#define CAN_WMBn_D03_Data_byte_0_MASK            (0xFF000000U)
#define CAN_WMBn_D03_Data_byte_0_SHIFT           (24U)
#define CAN_WMBn_D03_Data_byte_0_WIDTH           (8U)
#define CAN_WMBn_D03_Data_byte_0(x)              (((uint32_t)(((uint32_t)(x)) << CAN_WMBn_D03_Data_byte_0_SHIFT)) & CAN_WMBn_D03_Data_byte_0_MASK)
/*! @} */

/*! @name WMBn_D47 - Wake Up Message Buffer Register Data 4-7 */
/*! @{ */
#define CAN_WMBn_D47_Data_byte_7_MASK            (0xFFU)
#define CAN_WMBn_D47_Data_byte_7_SHIFT           (0U)
#define CAN_WMBn_D47_Data_byte_7_WIDTH           (8U)
#define CAN_WMBn_D47_Data_byte_7(x)              (((uint32_t)(((uint32_t)(x)) << CAN_WMBn_D47_Data_byte_7_SHIFT)) & CAN_WMBn_D47_Data_byte_7_MASK)
#define CAN_WMBn_D47_Data_byte_6_MASK            (0xFF00U)
#define CAN_WMBn_D47_Data_byte_6_SHIFT           (8U)
#define CAN_WMBn_D47_Data_byte_6_WIDTH           (8U)
#define CAN_WMBn_D47_Data_byte_6(x)              (((uint32_t)(((uint32_t)(x)) << CAN_WMBn_D47_Data_byte_6_SHIFT)) & CAN_WMBn_D47_Data_byte_6_MASK)
#define CAN_WMBn_D47_Data_byte_5_MASK            (0xFF0000U)
#define CAN_WMBn_D47_Data_byte_5_SHIFT           (16U)
#define CAN_WMBn_D47_Data_byte_5_WIDTH           (8U)
#define CAN_WMBn_D47_Data_byte_5(x)              (((uint32_t)(((uint32_t)(x)) << CAN_WMBn_D47_Data_byte_5_SHIFT)) & CAN_WMBn_D47_Data_byte_5_MASK)
#define CAN_WMBn_D47_Data_byte_4_MASK            (0xFF000000U)
#define CAN_WMBn_D47_Data_byte_4_SHIFT           (24U)
#define CAN_WMBn_D47_Data_byte_4_WIDTH           (8U)
#define CAN_WMBn_D47_Data_byte_4(x)              (((uint32_t)(((uint32_t)(x)) << CAN_WMBn_D47_Data_byte_4_SHIFT)) & CAN_WMBn_D47_Data_byte_4_MASK)
/*! @} */

/*! @name FDCTRL - CAN FD Control register */
/*! @{ */
#define CAN_FDCTRL_TDCVAL_MASK                   (0x3FU)
#define CAN_FDCTRL_TDCVAL_SHIFT                  (0U)
#define CAN_FDCTRL_TDCVAL_WIDTH                  (6U)
#define CAN_FDCTRL_TDCVAL(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_TDCVAL_SHIFT)) & CAN_FDCTRL_TDCVAL_MASK)
#define CAN_FDCTRL_TDCOFF_MASK                   (0x1F00U)
#define CAN_FDCTRL_TDCOFF_SHIFT                  (8U)
#define CAN_FDCTRL_TDCOFF_WIDTH                  (5U)
#define CAN_FDCTRL_TDCOFF(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_TDCOFF_SHIFT)) & CAN_FDCTRL_TDCOFF_MASK)
#define CAN_FDCTRL_TDCFAIL_MASK                  (0x4000U)
#define CAN_FDCTRL_TDCFAIL_SHIFT                 (14U)
#define CAN_FDCTRL_TDCFAIL_WIDTH                 (1U)
#define CAN_FDCTRL_TDCFAIL(x)                    (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_TDCFAIL_SHIFT)) & CAN_FDCTRL_TDCFAIL_MASK)
#define CAN_FDCTRL_TDCEN_MASK                    (0x8000U)
#define CAN_FDCTRL_TDCEN_SHIFT                   (15U)
#define CAN_FDCTRL_TDCEN_WIDTH                   (1U)
#define CAN_FDCTRL_TDCEN(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_TDCEN_SHIFT)) & CAN_FDCTRL_TDCEN_MASK)
#define CAN_FDCTRL_MBDSR0_MASK                   (0x30000U)
#define CAN_FDCTRL_MBDSR0_SHIFT                  (16U)
#define CAN_FDCTRL_MBDSR0_WIDTH                  (2U)
#define CAN_FDCTRL_MBDSR0(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_MBDSR0_SHIFT)) & CAN_FDCTRL_MBDSR0_MASK)
#define CAN_FDCTRL_MBDSR1_MASK                   (0x180000U)
#define CAN_FDCTRL_MBDSR1_SHIFT                  (19U)
#define CAN_FDCTRL_MBDSR1_WIDTH                  (2U)
#define CAN_FDCTRL_MBDSR1(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_MBDSR1_SHIFT)) & CAN_FDCTRL_MBDSR1_MASK)
#define CAN_FDCTRL_FDRATE_MASK                   (0x80000000U)
#define CAN_FDCTRL_FDRATE_SHIFT                  (31U)
#define CAN_FDCTRL_FDRATE_WIDTH                  (1U)
#define CAN_FDCTRL_FDRATE(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_FDRATE_SHIFT)) & CAN_FDCTRL_FDRATE_MASK)
/*! @} */

/*! @name FDCBT - CAN FD Bit Timing register */
/*! @{ */
#define CAN_FDCBT_FPSEG2_MASK                    (0x7U)
#define CAN_FDCBT_FPSEG2_SHIFT                   (0U)
#define CAN_FDCBT_FPSEG2_WIDTH                   (3U)
#define CAN_FDCBT_FPSEG2(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_FDCBT_FPSEG2_SHIFT)) & CAN_FDCBT_FPSEG2_MASK)
#define CAN_FDCBT_FPSEG1_MASK                    (0xE0U)
#define CAN_FDCBT_FPSEG1_SHIFT                   (5U)
#define CAN_FDCBT_FPSEG1_WIDTH                   (3U)
#define CAN_FDCBT_FPSEG1(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_FDCBT_FPSEG1_SHIFT)) & CAN_FDCBT_FPSEG1_MASK)
#define CAN_FDCBT_FPROPSEG_MASK                  (0x7C00U)
#define CAN_FDCBT_FPROPSEG_SHIFT                 (10U)
#define CAN_FDCBT_FPROPSEG_WIDTH                 (5U)
#define CAN_FDCBT_FPROPSEG(x)                    (((uint32_t)(((uint32_t)(x)) << CAN_FDCBT_FPROPSEG_SHIFT)) & CAN_FDCBT_FPROPSEG_MASK)
#define CAN_FDCBT_FRJW_MASK                      (0x70000U)
#define CAN_FDCBT_FRJW_SHIFT                     (16U)
#define CAN_FDCBT_FRJW_WIDTH                     (3U)
#define CAN_FDCBT_FRJW(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_FDCBT_FRJW_SHIFT)) & CAN_FDCBT_FRJW_MASK)
#define CAN_FDCBT_FPRESDIV_MASK                  (0x3FF00000U)
#define CAN_FDCBT_FPRESDIV_SHIFT                 (20U)
#define CAN_FDCBT_FPRESDIV_WIDTH                 (10U)
#define CAN_FDCBT_FPRESDIV(x)                    (((uint32_t)(((uint32_t)(x)) << CAN_FDCBT_FPRESDIV_SHIFT)) & CAN_FDCBT_FPRESDIV_MASK)
/*! @} */

/*! @name FDCRC - CAN FD CRC register */
/*! @{ */
#define CAN_FDCRC_FD_TXCRC_MASK                  (0x1FFFFFU)
#define CAN_FDCRC_FD_TXCRC_SHIFT                 (0U)
#define CAN_FDCRC_FD_TXCRC_WIDTH                 (21U)
#define CAN_FDCRC_FD_TXCRC(x)                    (((uint32_t)(((uint32_t)(x)) << CAN_FDCRC_FD_TXCRC_SHIFT)) & CAN_FDCRC_FD_TXCRC_MASK)
#define CAN_FDCRC_FD_MBCRC_MASK                  (0x7F000000U)
#define CAN_FDCRC_FD_MBCRC_SHIFT                 (24U)
#define CAN_FDCRC_FD_MBCRC_WIDTH                 (7U)
#define CAN_FDCRC_FD_MBCRC(x)                    (((uint32_t)(((uint32_t)(x)) << CAN_FDCRC_FD_MBCRC_SHIFT)) & CAN_FDCRC_FD_MBCRC_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group CAN_Register_Masks */

/*!
 * @}
 */ /* end of group CAN_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- CMP Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMP_Peripheral_Access_Layer CMP Peripheral Access Layer
 * @{
 */

/** CMP - Register Layout Typedef */
typedef struct {
  __IO uint32_t C0;                                /**< CMP Control Register 0, offset: 0x0 */
  __IO uint32_t C1;                                /**< CMP Control Register 1, offset: 0x4 */
  __IO uint32_t C2;                                /**< CMP Control Register 2, offset: 0x8 */
} CMP_Type, *CMP_MemMapPtr;

/** Number of instances of the CMP module. */
#define CMP_INSTANCE_COUNT                       (1u)

/* CMP - Peripheral instance base addresses */
/** Peripheral CMP0 base address */
#define CMP0_BASE                                (0x40073000u)
/** Peripheral CMP0 base pointer */
#define CMP0                                     ((CMP_Type *)CMP0_BASE)
/** Array initializer of CMP peripheral base addresses */
#define CMP_BASE_ADDRS                           { CMP0_BASE }
/** Array initializer of CMP peripheral base pointers */
#define CMP_BASE_PTRS                            { CMP0 }

/* ----------------------------------------------------------------------------
   -- CMP Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMP_Register_Masks CMP Register Masks
 * @{
 */

/*! @name C0 - CMP Control Register 0 */
/*! @{ */
#define CMP_C0_HYSTCTR_MASK                      (0x3U)
#define CMP_C0_HYSTCTR_SHIFT                     (0U)
#define CMP_C0_HYSTCTR_WIDTH                     (2U)
#define CMP_C0_HYSTCTR(x)                        (((uint32_t)(((uint32_t)(x)) << CMP_C0_HYSTCTR_SHIFT)) & CMP_C0_HYSTCTR_MASK)
#define CMP_C0_OFFSET_MASK                       (0x4U)
#define CMP_C0_OFFSET_SHIFT                      (2U)
#define CMP_C0_OFFSET_WIDTH                      (1U)
#define CMP_C0_OFFSET(x)                         (((uint32_t)(((uint32_t)(x)) << CMP_C0_OFFSET_SHIFT)) & CMP_C0_OFFSET_MASK)
#define CMP_C0_FILTER_CNT_MASK                   (0x70U)
#define CMP_C0_FILTER_CNT_SHIFT                  (4U)
#define CMP_C0_FILTER_CNT_WIDTH                  (3U)
#define CMP_C0_FILTER_CNT(x)                     (((uint32_t)(((uint32_t)(x)) << CMP_C0_FILTER_CNT_SHIFT)) & CMP_C0_FILTER_CNT_MASK)
#define CMP_C0_EN_MASK                           (0x100U)
#define CMP_C0_EN_SHIFT                          (8U)
#define CMP_C0_EN_WIDTH                          (1U)
#define CMP_C0_EN(x)                             (((uint32_t)(((uint32_t)(x)) << CMP_C0_EN_SHIFT)) & CMP_C0_EN_MASK)
#define CMP_C0_OPE_MASK                          (0x200U)
#define CMP_C0_OPE_SHIFT                         (9U)
#define CMP_C0_OPE_WIDTH                         (1U)
#define CMP_C0_OPE(x)                            (((uint32_t)(((uint32_t)(x)) << CMP_C0_OPE_SHIFT)) & CMP_C0_OPE_MASK)
#define CMP_C0_COS_MASK                          (0x400U)
#define CMP_C0_COS_SHIFT                         (10U)
#define CMP_C0_COS_WIDTH                         (1U)
#define CMP_C0_COS(x)                            (((uint32_t)(((uint32_t)(x)) << CMP_C0_COS_SHIFT)) & CMP_C0_COS_MASK)
#define CMP_C0_INVT_MASK                         (0x800U)
#define CMP_C0_INVT_SHIFT                        (11U)
#define CMP_C0_INVT_WIDTH                        (1U)
#define CMP_C0_INVT(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C0_INVT_SHIFT)) & CMP_C0_INVT_MASK)
#define CMP_C0_PMODE_MASK                        (0x1000U)
#define CMP_C0_PMODE_SHIFT                       (12U)
#define CMP_C0_PMODE_WIDTH                       (1U)
#define CMP_C0_PMODE(x)                          (((uint32_t)(((uint32_t)(x)) << CMP_C0_PMODE_SHIFT)) & CMP_C0_PMODE_MASK)
#define CMP_C0_WE_MASK                           (0x4000U)
#define CMP_C0_WE_SHIFT                          (14U)
#define CMP_C0_WE_WIDTH                          (1U)
#define CMP_C0_WE(x)                             (((uint32_t)(((uint32_t)(x)) << CMP_C0_WE_SHIFT)) & CMP_C0_WE_MASK)
#define CMP_C0_SE_MASK                           (0x8000U)
#define CMP_C0_SE_SHIFT                          (15U)
#define CMP_C0_SE_WIDTH                          (1U)
#define CMP_C0_SE(x)                             (((uint32_t)(((uint32_t)(x)) << CMP_C0_SE_SHIFT)) & CMP_C0_SE_MASK)
#define CMP_C0_FPR_MASK                          (0xFF0000U)
#define CMP_C0_FPR_SHIFT                         (16U)
#define CMP_C0_FPR_WIDTH                         (8U)
#define CMP_C0_FPR(x)                            (((uint32_t)(((uint32_t)(x)) << CMP_C0_FPR_SHIFT)) & CMP_C0_FPR_MASK)
#define CMP_C0_COUT_MASK                         (0x1000000U)
#define CMP_C0_COUT_SHIFT                        (24U)
#define CMP_C0_COUT_WIDTH                        (1U)
#define CMP_C0_COUT(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C0_COUT_SHIFT)) & CMP_C0_COUT_MASK)
#define CMP_C0_CFF_MASK                          (0x2000000U)
#define CMP_C0_CFF_SHIFT                         (25U)
#define CMP_C0_CFF_WIDTH                         (1U)
#define CMP_C0_CFF(x)                            (((uint32_t)(((uint32_t)(x)) << CMP_C0_CFF_SHIFT)) & CMP_C0_CFF_MASK)
#define CMP_C0_CFR_MASK                          (0x4000000U)
#define CMP_C0_CFR_SHIFT                         (26U)
#define CMP_C0_CFR_WIDTH                         (1U)
#define CMP_C0_CFR(x)                            (((uint32_t)(((uint32_t)(x)) << CMP_C0_CFR_SHIFT)) & CMP_C0_CFR_MASK)
#define CMP_C0_IEF_MASK                          (0x8000000U)
#define CMP_C0_IEF_SHIFT                         (27U)
#define CMP_C0_IEF_WIDTH                         (1U)
#define CMP_C0_IEF(x)                            (((uint32_t)(((uint32_t)(x)) << CMP_C0_IEF_SHIFT)) & CMP_C0_IEF_MASK)
#define CMP_C0_IER_MASK                          (0x10000000U)
#define CMP_C0_IER_SHIFT                         (28U)
#define CMP_C0_IER_WIDTH                         (1U)
#define CMP_C0_IER(x)                            (((uint32_t)(((uint32_t)(x)) << CMP_C0_IER_SHIFT)) & CMP_C0_IER_MASK)
#define CMP_C0_DMAEN_MASK                        (0x40000000U)
#define CMP_C0_DMAEN_SHIFT                       (30U)
#define CMP_C0_DMAEN_WIDTH                       (1U)
#define CMP_C0_DMAEN(x)                          (((uint32_t)(((uint32_t)(x)) << CMP_C0_DMAEN_SHIFT)) & CMP_C0_DMAEN_MASK)
/*! @} */

/*! @name C1 - CMP Control Register 1 */
/*! @{ */
#define CMP_C1_VOSEL_MASK                        (0xFFU)
#define CMP_C1_VOSEL_SHIFT                       (0U)
#define CMP_C1_VOSEL_WIDTH                       (8U)
#define CMP_C1_VOSEL(x)                          (((uint32_t)(((uint32_t)(x)) << CMP_C1_VOSEL_SHIFT)) & CMP_C1_VOSEL_MASK)
#define CMP_C1_MSEL_MASK                         (0x700U)
#define CMP_C1_MSEL_SHIFT                        (8U)
#define CMP_C1_MSEL_WIDTH                        (3U)
#define CMP_C1_MSEL(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C1_MSEL_SHIFT)) & CMP_C1_MSEL_MASK)
#define CMP_C1_PSEL_MASK                         (0x3800U)
#define CMP_C1_PSEL_SHIFT                        (11U)
#define CMP_C1_PSEL_WIDTH                        (3U)
#define CMP_C1_PSEL(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C1_PSEL_SHIFT)) & CMP_C1_PSEL_MASK)
#define CMP_C1_VRSEL_MASK                        (0x4000U)
#define CMP_C1_VRSEL_SHIFT                       (14U)
#define CMP_C1_VRSEL_WIDTH                       (1U)
#define CMP_C1_VRSEL(x)                          (((uint32_t)(((uint32_t)(x)) << CMP_C1_VRSEL_SHIFT)) & CMP_C1_VRSEL_MASK)
#define CMP_C1_DACEN_MASK                        (0x8000U)
#define CMP_C1_DACEN_SHIFT                       (15U)
#define CMP_C1_DACEN_WIDTH                       (1U)
#define CMP_C1_DACEN(x)                          (((uint32_t)(((uint32_t)(x)) << CMP_C1_DACEN_SHIFT)) & CMP_C1_DACEN_MASK)
#define CMP_C1_CHN0_MASK                         (0x10000U)
#define CMP_C1_CHN0_SHIFT                        (16U)
#define CMP_C1_CHN0_WIDTH                        (1U)
#define CMP_C1_CHN0(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C1_CHN0_SHIFT)) & CMP_C1_CHN0_MASK)
#define CMP_C1_CHN1_MASK                         (0x20000U)
#define CMP_C1_CHN1_SHIFT                        (17U)
#define CMP_C1_CHN1_WIDTH                        (1U)
#define CMP_C1_CHN1(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C1_CHN1_SHIFT)) & CMP_C1_CHN1_MASK)
#define CMP_C1_CHN2_MASK                         (0x40000U)
#define CMP_C1_CHN2_SHIFT                        (18U)
#define CMP_C1_CHN2_WIDTH                        (1U)
#define CMP_C1_CHN2(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C1_CHN2_SHIFT)) & CMP_C1_CHN2_MASK)
#define CMP_C1_CHN3_MASK                         (0x80000U)
#define CMP_C1_CHN3_SHIFT                        (19U)
#define CMP_C1_CHN3_WIDTH                        (1U)
#define CMP_C1_CHN3(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C1_CHN3_SHIFT)) & CMP_C1_CHN3_MASK)
#define CMP_C1_CHN4_MASK                         (0x100000U)
#define CMP_C1_CHN4_SHIFT                        (20U)
#define CMP_C1_CHN4_WIDTH                        (1U)
#define CMP_C1_CHN4(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C1_CHN4_SHIFT)) & CMP_C1_CHN4_MASK)
#define CMP_C1_CHN5_MASK                         (0x200000U)
#define CMP_C1_CHN5_SHIFT                        (21U)
#define CMP_C1_CHN5_WIDTH                        (1U)
#define CMP_C1_CHN5(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C1_CHN5_SHIFT)) & CMP_C1_CHN5_MASK)
#define CMP_C1_CHN6_MASK                         (0x400000U)
#define CMP_C1_CHN6_SHIFT                        (22U)
#define CMP_C1_CHN6_WIDTH                        (1U)
#define CMP_C1_CHN6(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C1_CHN6_SHIFT)) & CMP_C1_CHN6_MASK)
#define CMP_C1_CHN7_MASK                         (0x800000U)
#define CMP_C1_CHN7_SHIFT                        (23U)
#define CMP_C1_CHN7_WIDTH                        (1U)
#define CMP_C1_CHN7(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C1_CHN7_SHIFT)) & CMP_C1_CHN7_MASK)
#define CMP_C1_INNSEL_MASK                       (0x3000000U)
#define CMP_C1_INNSEL_SHIFT                      (24U)
#define CMP_C1_INNSEL_WIDTH                      (2U)
#define CMP_C1_INNSEL(x)                         (((uint32_t)(((uint32_t)(x)) << CMP_C1_INNSEL_SHIFT)) & CMP_C1_INNSEL_MASK)
#define CMP_C1_INPSEL_MASK                       (0x18000000U)
#define CMP_C1_INPSEL_SHIFT                      (27U)
#define CMP_C1_INPSEL_WIDTH                      (2U)
#define CMP_C1_INPSEL(x)                         (((uint32_t)(((uint32_t)(x)) << CMP_C1_INPSEL_SHIFT)) & CMP_C1_INPSEL_MASK)
/*! @} */

/*! @name C2 - CMP Control Register 2 */
/*! @{ */
#define CMP_C2_ACOn_MASK                         (0xFFU)
#define CMP_C2_ACOn_SHIFT                        (0U)
#define CMP_C2_ACOn_WIDTH                        (8U)
#define CMP_C2_ACOn(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C2_ACOn_SHIFT)) & CMP_C2_ACOn_MASK)
#define CMP_C2_INITMOD_MASK                      (0x3F00U)
#define CMP_C2_INITMOD_SHIFT                     (8U)
#define CMP_C2_INITMOD_WIDTH                     (6U)
#define CMP_C2_INITMOD(x)                        (((uint32_t)(((uint32_t)(x)) << CMP_C2_INITMOD_SHIFT)) & CMP_C2_INITMOD_MASK)
#define CMP_C2_NSAM_MASK                         (0xC000U)
#define CMP_C2_NSAM_SHIFT                        (14U)
#define CMP_C2_NSAM_WIDTH                        (2U)
#define CMP_C2_NSAM(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C2_NSAM_SHIFT)) & CMP_C2_NSAM_MASK)
#define CMP_C2_CH0F_MASK                         (0x10000U)
#define CMP_C2_CH0F_SHIFT                        (16U)
#define CMP_C2_CH0F_WIDTH                        (1U)
#define CMP_C2_CH0F(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C2_CH0F_SHIFT)) & CMP_C2_CH0F_MASK)
#define CMP_C2_CH1F_MASK                         (0x20000U)
#define CMP_C2_CH1F_SHIFT                        (17U)
#define CMP_C2_CH1F_WIDTH                        (1U)
#define CMP_C2_CH1F(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C2_CH1F_SHIFT)) & CMP_C2_CH1F_MASK)
#define CMP_C2_CH2F_MASK                         (0x40000U)
#define CMP_C2_CH2F_SHIFT                        (18U)
#define CMP_C2_CH2F_WIDTH                        (1U)
#define CMP_C2_CH2F(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C2_CH2F_SHIFT)) & CMP_C2_CH2F_MASK)
#define CMP_C2_CH3F_MASK                         (0x80000U)
#define CMP_C2_CH3F_SHIFT                        (19U)
#define CMP_C2_CH3F_WIDTH                        (1U)
#define CMP_C2_CH3F(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C2_CH3F_SHIFT)) & CMP_C2_CH3F_MASK)
#define CMP_C2_CH4F_MASK                         (0x100000U)
#define CMP_C2_CH4F_SHIFT                        (20U)
#define CMP_C2_CH4F_WIDTH                        (1U)
#define CMP_C2_CH4F(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C2_CH4F_SHIFT)) & CMP_C2_CH4F_MASK)
#define CMP_C2_CH5F_MASK                         (0x200000U)
#define CMP_C2_CH5F_SHIFT                        (21U)
#define CMP_C2_CH5F_WIDTH                        (1U)
#define CMP_C2_CH5F(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C2_CH5F_SHIFT)) & CMP_C2_CH5F_MASK)
#define CMP_C2_CH6F_MASK                         (0x400000U)
#define CMP_C2_CH6F_SHIFT                        (22U)
#define CMP_C2_CH6F_WIDTH                        (1U)
#define CMP_C2_CH6F(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C2_CH6F_SHIFT)) & CMP_C2_CH6F_MASK)
#define CMP_C2_CH7F_MASK                         (0x800000U)
#define CMP_C2_CH7F_SHIFT                        (23U)
#define CMP_C2_CH7F_WIDTH                        (1U)
#define CMP_C2_CH7F(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C2_CH7F_SHIFT)) & CMP_C2_CH7F_MASK)
#define CMP_C2_FXMXCH_MASK                       (0xE000000U)
#define CMP_C2_FXMXCH_SHIFT                      (25U)
#define CMP_C2_FXMXCH_WIDTH                      (3U)
#define CMP_C2_FXMXCH(x)                         (((uint32_t)(((uint32_t)(x)) << CMP_C2_FXMXCH_SHIFT)) & CMP_C2_FXMXCH_MASK)
#define CMP_C2_FXMP_MASK                         (0x20000000U)
#define CMP_C2_FXMP_SHIFT                        (29U)
#define CMP_C2_FXMP_WIDTH                        (1U)
#define CMP_C2_FXMP(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C2_FXMP_SHIFT)) & CMP_C2_FXMP_MASK)
#define CMP_C2_RRIE_MASK                         (0x40000000U)
#define CMP_C2_RRIE_SHIFT                        (30U)
#define CMP_C2_RRIE_WIDTH                        (1U)
#define CMP_C2_RRIE(x)                           (((uint32_t)(((uint32_t)(x)) << CMP_C2_RRIE_SHIFT)) & CMP_C2_RRIE_MASK)
#define CMP_C2_RRE_MASK                          (0x80000000U)
#define CMP_C2_RRE_SHIFT                         (31U)
#define CMP_C2_RRE_WIDTH                         (1U)
#define CMP_C2_RRE(x)                            (((uint32_t)(((uint32_t)(x)) << CMP_C2_RRE_SHIFT)) & CMP_C2_RRE_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group CMP_Register_Masks */

/*!
 * @}
 */ /* end of group CMP_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- CRC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CRC_Peripheral_Access_Layer CRC Peripheral Access Layer
 * @{
 */

/** CRC - Register Layout Typedef */
typedef struct {
  union {                                          /* offset: 0x0 */
    struct {                                         /* offset: 0x0 */
      __IO uint8_t LL;                             /**< CRC_LL register, offset: 0x0 */
      __IO uint8_t LU;                             /**< CRC_LU register, offset: 0x1 */
      __IO uint8_t HL;                             /**< CRC_HL register, offset: 0x2 */
      __IO uint8_t HU;                             /**< CRC_HU register, offset: 0x3 */
    } DATA_8;
    struct {                                         /* offset: 0x0 */
      __IO uint16_t L;                             /**< CRC_L register, offset: 0x0 */
      __IO uint16_t H;                             /**< CRC_H register, offset: 0x2 */
    } DATA_16;
    __IO uint32_t DATA;                              /**< CRC Data register, offset: 0x0 */
  } DATAu;
  __IO uint32_t GPOLY;                             /**< CRC Polynomial register, offset: 0x4 */
  __IO uint32_t CTRL;                              /**< CRC Control register, offset: 0x8 */
} CRC_Type, *CRC_MemMapPtr;

/** Number of instances of the CRC module. */
#define CRC_INSTANCE_COUNT                       (1u)

/* CRC - Peripheral instance base addresses */
/** Peripheral CRC base address */
#define CRC_BASE                                 (0x40032000u)
/** Peripheral CRC base pointer */
#define CRC                                      ((CRC_Type *)CRC_BASE)
/** Array initializer of CRC peripheral base addresses */
#define CRC_BASE_ADDRS                           { CRC_BASE }
/** Array initializer of CRC peripheral base pointers */
#define CRC_BASE_PTRS                            { CRC }

/* ----------------------------------------------------------------------------
   -- CRC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CRC_Register_Masks CRC Register Masks
 * @{
 */

/*! @name DATA_8_LL - CRC_LL register */
/*! @{ */
#define CRC_DATAu_DATA_8_LL_DATALL_MASK            (0xFFU)
#define CRC_DATAu_DATA_8_LL_DATALL_SHIFT           (0U)
#define CRC_DATAu_DATA_8_LL_DATALL_WIDTH           (8U)
#define CRC_DATAu_DATA_8_LL_DATALL(x)              (((uint8_t)(((uint8_t)(x)) << CRC_DATAu_DATA_8_LL_DATALL_SHIFT)) & CRC_DATAu_DATA_8_LL_DATALL_MASK)
/*! @} */

/*! @name DATA_8_LU - CRC_LU register */
/*! @{ */
#define CRC_DATAu_DATA_8_LU_DATALU_MASK            (0xFFU)
#define CRC_DATAu_DATA_8_LU_DATALU_SHIFT           (0U)
#define CRC_DATAu_DATA_8_LU_DATALU_WIDTH           (8U)
#define CRC_DATAu_DATA_8_LU_DATALU(x)              (((uint8_t)(((uint8_t)(x)) << CRC_DATAu_DATA_8_LU_DATALU_SHIFT)) & CRC_DATAu_DATA_8_LU_DATALU_MASK)
/*! @} */

/*! @name DATA_8_HL - CRC_HL register */
/*! @{ */
#define CRC_DATAu_DATA_8_HL_DATAHL_MASK            (0xFFU)
#define CRC_DATAu_DATA_8_HL_DATAHL_SHIFT           (0U)
#define CRC_DATAu_DATA_8_HL_DATAHL_WIDTH           (8U)
#define CRC_DATAu_DATA_8_HL_DATAHL(x)              (((uint8_t)(((uint8_t)(x)) << CRC_DATAu_DATA_8_HL_DATAHL_SHIFT)) & CRC_DATAu_DATA_8_HL_DATAHL_MASK)
/*! @} */

/*! @name DATA_8_HU - CRC_HU register */
/*! @{ */
#define CRC_DATAu_DATA_8_HU_DATAHU_MASK            (0xFFU)
#define CRC_DATAu_DATA_8_HU_DATAHU_SHIFT           (0U)
#define CRC_DATAu_DATA_8_HU_DATAHU_WIDTH           (8U)
#define CRC_DATAu_DATA_8_HU_DATAHU(x)              (((uint8_t)(((uint8_t)(x)) << CRC_DATAu_DATA_8_HU_DATAHU_SHIFT)) & CRC_DATAu_DATA_8_HU_DATAHU_MASK)
/*! @} */

/*! @name DATA_16_L - CRC_L register */
/*! @{ */
#define CRC_DATAu_DATA_16_L_DATAL_MASK             (0xFFFFU)
#define CRC_DATAu_DATA_16_L_DATAL_SHIFT            (0U)
#define CRC_DATAu_DATA_16_L_DATAL_WIDTH            (16U)
#define CRC_DATAu_DATA_16_L_DATAL(x)               (((uint16_t)(((uint16_t)(x)) << CRC_DATAu_DATA_16_L_DATAL_SHIFT)) & CRC_DATAu_DATA_16_L_DATAL_MASK)
/*! @} */

/*! @name DATA_16_H - CRC_H register */
/*! @{ */
#define CRC_DATAu_DATA_16_H_DATAH_MASK             (0xFFFFU)
#define CRC_DATAu_DATA_16_H_DATAH_SHIFT            (0U)
#define CRC_DATAu_DATA_16_H_DATAH_WIDTH            (16U)
#define CRC_DATAu_DATA_16_H_DATAH(x)               (((uint16_t)(((uint16_t)(x)) << CRC_DATAu_DATA_16_H_DATAH_SHIFT)) & CRC_DATAu_DATA_16_H_DATAH_MASK)
/*! @} */

/*! @name DATA - CRC Data register */
/*! @{ */
#define CRC_DATAu_DATA_LL_MASK                         (0xFFU)
#define CRC_DATAu_DATA_LL_SHIFT                        (0U)
#define CRC_DATAu_DATA_LL_WIDTH                        (8U)
#define CRC_DATAu_DATA_LL(x)                           (((uint32_t)(((uint32_t)(x)) << CRC_DATAu_DATA_LL_SHIFT)) & CRC_DATAu_DATA_LL_MASK)
#define CRC_DATAu_DATA_LU_MASK                         (0xFF00U)
#define CRC_DATAu_DATA_LU_SHIFT                        (8U)
#define CRC_DATAu_DATA_LU_WIDTH                        (8U)
#define CRC_DATAu_DATA_LU(x)                           (((uint32_t)(((uint32_t)(x)) << CRC_DATAu_DATA_LU_SHIFT)) & CRC_DATAu_DATA_LU_MASK)
#define CRC_DATAu_DATA_HL_MASK                         (0xFF0000U)
#define CRC_DATAu_DATA_HL_SHIFT                        (16U)
#define CRC_DATAu_DATA_HL_WIDTH                        (8U)
#define CRC_DATAu_DATA_HL(x)                           (((uint32_t)(((uint32_t)(x)) << CRC_DATAu_DATA_HL_SHIFT)) & CRC_DATAu_DATA_HL_MASK)
#define CRC_DATAu_DATA_HU_MASK                         (0xFF000000U)
#define CRC_DATAu_DATA_HU_SHIFT                        (24U)
#define CRC_DATAu_DATA_HU_WIDTH                        (8U)
#define CRC_DATAu_DATA_HU(x)                           (((uint32_t)(((uint32_t)(x)) << CRC_DATAu_DATA_HU_SHIFT)) & CRC_DATAu_DATA_HU_MASK)
/*! @} */

/*! @name GPOLY - CRC Polynomial register */
/*! @{ */
#define CRC_GPOLY_LOW_MASK                       (0xFFFFU)
#define CRC_GPOLY_LOW_SHIFT                      (0U)
#define CRC_GPOLY_LOW_WIDTH                      (16U)
#define CRC_GPOLY_LOW(x)                         (((uint32_t)(((uint32_t)(x)) << CRC_GPOLY_LOW_SHIFT)) & CRC_GPOLY_LOW_MASK)
#define CRC_GPOLY_HIGH_MASK                      (0xFFFF0000U)
#define CRC_GPOLY_HIGH_SHIFT                     (16U)
#define CRC_GPOLY_HIGH_WIDTH                     (16U)
#define CRC_GPOLY_HIGH(x)                        (((uint32_t)(((uint32_t)(x)) << CRC_GPOLY_HIGH_SHIFT)) & CRC_GPOLY_HIGH_MASK)
/*! @} */

/*! @name CTRL - CRC Control register */
/*! @{ */
#define CRC_CTRL_TCRC_MASK                       (0x1000000U)
#define CRC_CTRL_TCRC_SHIFT                      (24U)
#define CRC_CTRL_TCRC_WIDTH                      (1U)
#define CRC_CTRL_TCRC(x)                         (((uint32_t)(((uint32_t)(x)) << CRC_CTRL_TCRC_SHIFT)) & CRC_CTRL_TCRC_MASK)
#define CRC_CTRL_WAS_MASK                        (0x2000000U)
#define CRC_CTRL_WAS_SHIFT                       (25U)
#define CRC_CTRL_WAS_WIDTH                       (1U)
#define CRC_CTRL_WAS(x)                          (((uint32_t)(((uint32_t)(x)) << CRC_CTRL_WAS_SHIFT)) & CRC_CTRL_WAS_MASK)
#define CRC_CTRL_FXOR_MASK                       (0x4000000U)
#define CRC_CTRL_FXOR_SHIFT                      (26U)
#define CRC_CTRL_FXOR_WIDTH                      (1U)
#define CRC_CTRL_FXOR(x)                         (((uint32_t)(((uint32_t)(x)) << CRC_CTRL_FXOR_SHIFT)) & CRC_CTRL_FXOR_MASK)
#define CRC_CTRL_TOTR_MASK                       (0x30000000U)
#define CRC_CTRL_TOTR_SHIFT                      (28U)
#define CRC_CTRL_TOTR_WIDTH                      (2U)
#define CRC_CTRL_TOTR(x)                         (((uint32_t)(((uint32_t)(x)) << CRC_CTRL_TOTR_SHIFT)) & CRC_CTRL_TOTR_MASK)
#define CRC_CTRL_TOT_MASK                        (0xC0000000U)
#define CRC_CTRL_TOT_SHIFT                       (30U)
#define CRC_CTRL_TOT_WIDTH                       (2U)
#define CRC_CTRL_TOT(x)                          (((uint32_t)(((uint32_t)(x)) << CRC_CTRL_TOT_SHIFT)) & CRC_CTRL_TOT_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group CRC_Register_Masks */

/*!
 * @}
 */ /* end of group CRC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- DMA Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Peripheral_Access_Layer DMA Peripheral Access Layer
 * @{
 */

/** DMA - Size of Registers Arrays */
#define DMA_DCHPRI_COUNT                          16u
#define DMA_TCD_COUNT                             16u

/** DMA - Register Layout Typedef */
typedef struct {
  __IO uint32_t CR;                                /**< Control Register, offset: 0x0 */
  __I  uint32_t ES;                                /**< Error Status Register, offset: 0x4 */
  uint8_t RESERVED_0[4];
  __IO uint32_t ERQ;                               /**< Enable Request Register, offset: 0xC */
  uint8_t RESERVED_1[4];
  __IO uint32_t EEI;                               /**< Enable Error Interrupt Register, offset: 0x14 */
  __O  uint8_t CEEI;                               /**< Clear Enable Error Interrupt Register, offset: 0x18 */
  __O  uint8_t SEEI;                               /**< Set Enable Error Interrupt Register, offset: 0x19 */
  __O  uint8_t CERQ;                               /**< Clear Enable Request Register, offset: 0x1A */
  __O  uint8_t SERQ;                               /**< Set Enable Request Register, offset: 0x1B */
  __O  uint8_t CDNE;                               /**< Clear DONE Status Bit Register, offset: 0x1C */
  __O  uint8_t SSRT;                               /**< Set START Bit Register, offset: 0x1D */
  __O  uint8_t CERR;                               /**< Clear Error Register, offset: 0x1E */
  __O  uint8_t CINT;                               /**< Clear Interrupt Request Register, offset: 0x1F */
  uint8_t RESERVED_2[4];
  __IO uint32_t INT;                               /**< Interrupt Request Register, offset: 0x24 */
  uint8_t RESERVED_3[4];
  __IO uint32_t ERR;                               /**< Error Register, offset: 0x2C */
  uint8_t RESERVED_4[4];
  __I  uint32_t HRS;                               /**< Hardware Request Status Register, offset: 0x34 */
  uint8_t RESERVED_5[12];
  __IO uint32_t EARS;                              /**< Enable Asynchronous Request in Stop Register, offset: 0x44 */
  uint8_t RESERVED_6[184];
  __IO uint8_t DCHPRI[DMA_DCHPRI_COUNT];           /**< Channel Priority Register, array offset: 0x100, array step: 0x1 */
  uint8_t RESERVED_7[3824];
  struct {                                         /* offset: 0x1000, array step: 0x20 */
    __IO uint32_t SADDR;                             /**< TCD Source Address, array offset: 0x1000, array step: 0x20 */
    __IO uint16_t SOFF;                              /**< TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20 */
    __IO uint16_t ATTR;                              /**< TCD Transfer Attributes, array offset: 0x1006, array step: 0x20 */
    union {                                          /* offset: 0x1008, array step: 0x20 */
      __IO uint32_t MLNO;                       /**< TCD Minor Byte Count (Minor Loop Mapping Disabled), array offset: 0x1008, array step: 0x20 */
      __IO uint32_t MLOFFNO;                    /**< TCD Signed Minor Loop Offset (Minor Loop Mapping Enabled and Offset Disabled), array offset: 0x1008, array step: 0x20 */
      __IO uint32_t MLOFFYES;                   /**< TCD Signed Minor Loop Offset (Minor Loop Mapping and Offset Enabled), array offset: 0x1008, array step: 0x20 */
    } NBYTES;
    __IO uint32_t SLAST;                             /**< TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20 */
    __IO uint32_t DADDR;                             /**< TCD Destination Address, array offset: 0x1010, array step: 0x20 */
    __IO uint16_t DOFF;                              /**< TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20 */
    union {                                          /* offset: 0x1016, array step: 0x20 */
      __IO uint16_t ELINKNO;                     /**< TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x1016, array step: 0x20 */
      __IO uint16_t ELINKYES;                    /**< TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x1016, array step: 0x20 */
    } CITER;
    __IO uint32_t DLASTSGA;                          /**< TCD Last Destination Address Adjustment/Scatter Gather Address, array offset: 0x1018, array step: 0x20 */
    __IO uint16_t CSR;                               /**< TCD Control and Status, array offset: 0x101C, array step: 0x20 */
    union {                                          /* offset: 0x101E, array step: 0x20 */
      __IO uint16_t ELINKNO;                     /**< TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x101E, array step: 0x20 */
      __IO uint16_t ELINKYES;                    /**< TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x101E, array step: 0x20 */
    } BITER;
  } TCD[DMA_TCD_COUNT];
} DMA_Type, *DMA_MemMapPtr;

/** Number of instances of the DMA module. */
#define DMA_INSTANCE_COUNT                       (1u)

/* DMA - Peripheral instance base addresses */
/** Peripheral DMA base address */
#define DMA_BASE                                 (0x40008000u)
/** Peripheral DMA base pointer */
#define DMA                                      ((DMA_Type *)DMA_BASE)
/** Array initializer of DMA peripheral base addresses */
#define DMA_BASE_ADDRS                           { DMA_BASE }
/** Array initializer of DMA peripheral base pointers */
#define DMA_BASE_PTRS                            { DMA }

/* ----------------------------------------------------------------------------
   -- DMA Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Register_Masks DMA Register Masks
 * @{
 */

/*! @name CR - Control Register */
/*! @{ */
#define DMA_CR_EDBG_MASK                         (0x2U)
#define DMA_CR_EDBG_SHIFT                        (1U)
#define DMA_CR_EDBG_WIDTH                        (1U)
#define DMA_CR_EDBG(x)                           (((uint32_t)(((uint32_t)(x)) << DMA_CR_EDBG_SHIFT)) & DMA_CR_EDBG_MASK)
#define DMA_CR_ERCA_MASK                         (0x4U)
#define DMA_CR_ERCA_SHIFT                        (2U)
#define DMA_CR_ERCA_WIDTH                        (1U)
#define DMA_CR_ERCA(x)                           (((uint32_t)(((uint32_t)(x)) << DMA_CR_ERCA_SHIFT)) & DMA_CR_ERCA_MASK)
#define DMA_CR_HOE_MASK                          (0x10U)
#define DMA_CR_HOE_SHIFT                         (4U)
#define DMA_CR_HOE_WIDTH                         (1U)
#define DMA_CR_HOE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_CR_HOE_SHIFT)) & DMA_CR_HOE_MASK)
#define DMA_CR_HALT_MASK                         (0x20U)
#define DMA_CR_HALT_SHIFT                        (5U)
#define DMA_CR_HALT_WIDTH                        (1U)
#define DMA_CR_HALT(x)                           (((uint32_t)(((uint32_t)(x)) << DMA_CR_HALT_SHIFT)) & DMA_CR_HALT_MASK)
#define DMA_CR_CLM_MASK                          (0x40U)
#define DMA_CR_CLM_SHIFT                         (6U)
#define DMA_CR_CLM_WIDTH                         (1U)
#define DMA_CR_CLM(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_CR_CLM_SHIFT)) & DMA_CR_CLM_MASK)
#define DMA_CR_EMLM_MASK                         (0x80U)
#define DMA_CR_EMLM_SHIFT                        (7U)
#define DMA_CR_EMLM_WIDTH                        (1U)
#define DMA_CR_EMLM(x)                           (((uint32_t)(((uint32_t)(x)) << DMA_CR_EMLM_SHIFT)) & DMA_CR_EMLM_MASK)
#define DMA_CR_ECX_MASK                          (0x10000U)
#define DMA_CR_ECX_SHIFT                         (16U)
#define DMA_CR_ECX_WIDTH                         (1U)
#define DMA_CR_ECX(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_CR_ECX_SHIFT)) & DMA_CR_ECX_MASK)
#define DMA_CR_CX_MASK                           (0x20000U)
#define DMA_CR_CX_SHIFT                          (17U)
#define DMA_CR_CX_WIDTH                          (1U)
#define DMA_CR_CX(x)                             (((uint32_t)(((uint32_t)(x)) << DMA_CR_CX_SHIFT)) & DMA_CR_CX_MASK)
#define DMA_CR_ACTIVE_MASK                       (0x80000000U)
#define DMA_CR_ACTIVE_SHIFT                      (31U)
#define DMA_CR_ACTIVE_WIDTH                      (1U)
#define DMA_CR_ACTIVE(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_CR_ACTIVE_SHIFT)) & DMA_CR_ACTIVE_MASK)
/*! @} */

/*! @name ES - Error Status Register */
/*! @{ */
#define DMA_ES_DBE_MASK                          (0x1U)
#define DMA_ES_DBE_SHIFT                         (0U)
#define DMA_ES_DBE_WIDTH                         (1U)
#define DMA_ES_DBE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_DBE_SHIFT)) & DMA_ES_DBE_MASK)
#define DMA_ES_SBE_MASK                          (0x2U)
#define DMA_ES_SBE_SHIFT                         (1U)
#define DMA_ES_SBE_WIDTH                         (1U)
#define DMA_ES_SBE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_SBE_SHIFT)) & DMA_ES_SBE_MASK)
#define DMA_ES_SGE_MASK                          (0x4U)
#define DMA_ES_SGE_SHIFT                         (2U)
#define DMA_ES_SGE_WIDTH                         (1U)
#define DMA_ES_SGE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_SGE_SHIFT)) & DMA_ES_SGE_MASK)
#define DMA_ES_NCE_MASK                          (0x8U)
#define DMA_ES_NCE_SHIFT                         (3U)
#define DMA_ES_NCE_WIDTH                         (1U)
#define DMA_ES_NCE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_NCE_SHIFT)) & DMA_ES_NCE_MASK)
#define DMA_ES_DOE_MASK                          (0x10U)
#define DMA_ES_DOE_SHIFT                         (4U)
#define DMA_ES_DOE_WIDTH                         (1U)
#define DMA_ES_DOE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_DOE_SHIFT)) & DMA_ES_DOE_MASK)
#define DMA_ES_DAE_MASK                          (0x20U)
#define DMA_ES_DAE_SHIFT                         (5U)
#define DMA_ES_DAE_WIDTH                         (1U)
#define DMA_ES_DAE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_DAE_SHIFT)) & DMA_ES_DAE_MASK)
#define DMA_ES_SOE_MASK                          (0x40U)
#define DMA_ES_SOE_SHIFT                         (6U)
#define DMA_ES_SOE_WIDTH                         (1U)
#define DMA_ES_SOE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_SOE_SHIFT)) & DMA_ES_SOE_MASK)
#define DMA_ES_SAE_MASK                          (0x80U)
#define DMA_ES_SAE_SHIFT                         (7U)
#define DMA_ES_SAE_WIDTH                         (1U)
#define DMA_ES_SAE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_SAE_SHIFT)) & DMA_ES_SAE_MASK)
#define DMA_ES_ERRCHN_MASK                       (0xF00U)
#define DMA_ES_ERRCHN_SHIFT                      (8U)
#define DMA_ES_ERRCHN_WIDTH                      (4U)
#define DMA_ES_ERRCHN(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ES_ERRCHN_SHIFT)) & DMA_ES_ERRCHN_MASK)
#define DMA_ES_CPE_MASK                          (0x4000U)
#define DMA_ES_CPE_SHIFT                         (14U)
#define DMA_ES_CPE_WIDTH                         (1U)
#define DMA_ES_CPE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_CPE_SHIFT)) & DMA_ES_CPE_MASK)
#define DMA_ES_ECX_MASK                          (0x10000U)
#define DMA_ES_ECX_SHIFT                         (16U)
#define DMA_ES_ECX_WIDTH                         (1U)
#define DMA_ES_ECX(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_ECX_SHIFT)) & DMA_ES_ECX_MASK)
#define DMA_ES_VLD_MASK                          (0x80000000U)
#define DMA_ES_VLD_SHIFT                         (31U)
#define DMA_ES_VLD_WIDTH                         (1U)
#define DMA_ES_VLD(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_VLD_SHIFT)) & DMA_ES_VLD_MASK)
/*! @} */

/*! @name ERQ - Enable Request Register */
/*! @{ */
#define DMA_ERQ_ERQ0_MASK                        (0x1U)
#define DMA_ERQ_ERQ0_SHIFT                       (0U)
#define DMA_ERQ_ERQ0_WIDTH                       (1U)
#define DMA_ERQ_ERQ0(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ0_SHIFT)) & DMA_ERQ_ERQ0_MASK)
#define DMA_ERQ_ERQ1_MASK                        (0x2U)
#define DMA_ERQ_ERQ1_SHIFT                       (1U)
#define DMA_ERQ_ERQ1_WIDTH                       (1U)
#define DMA_ERQ_ERQ1(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ1_SHIFT)) & DMA_ERQ_ERQ1_MASK)
#define DMA_ERQ_ERQ2_MASK                        (0x4U)
#define DMA_ERQ_ERQ2_SHIFT                       (2U)
#define DMA_ERQ_ERQ2_WIDTH                       (1U)
#define DMA_ERQ_ERQ2(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ2_SHIFT)) & DMA_ERQ_ERQ2_MASK)
#define DMA_ERQ_ERQ3_MASK                        (0x8U)
#define DMA_ERQ_ERQ3_SHIFT                       (3U)
#define DMA_ERQ_ERQ3_WIDTH                       (1U)
#define DMA_ERQ_ERQ3(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ3_SHIFT)) & DMA_ERQ_ERQ3_MASK)
#define DMA_ERQ_ERQ4_MASK                        (0x10U)
#define DMA_ERQ_ERQ4_SHIFT                       (4U)
#define DMA_ERQ_ERQ4_WIDTH                       (1U)
#define DMA_ERQ_ERQ4(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ4_SHIFT)) & DMA_ERQ_ERQ4_MASK)
#define DMA_ERQ_ERQ5_MASK                        (0x20U)
#define DMA_ERQ_ERQ5_SHIFT                       (5U)
#define DMA_ERQ_ERQ5_WIDTH                       (1U)
#define DMA_ERQ_ERQ5(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ5_SHIFT)) & DMA_ERQ_ERQ5_MASK)
#define DMA_ERQ_ERQ6_MASK                        (0x40U)
#define DMA_ERQ_ERQ6_SHIFT                       (6U)
#define DMA_ERQ_ERQ6_WIDTH                       (1U)
#define DMA_ERQ_ERQ6(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ6_SHIFT)) & DMA_ERQ_ERQ6_MASK)
#define DMA_ERQ_ERQ7_MASK                        (0x80U)
#define DMA_ERQ_ERQ7_SHIFT                       (7U)
#define DMA_ERQ_ERQ7_WIDTH                       (1U)
#define DMA_ERQ_ERQ7(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ7_SHIFT)) & DMA_ERQ_ERQ7_MASK)
#define DMA_ERQ_ERQ8_MASK                        (0x100U)
#define DMA_ERQ_ERQ8_SHIFT                       (8U)
#define DMA_ERQ_ERQ8_WIDTH                       (1U)
#define DMA_ERQ_ERQ8(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ8_SHIFT)) & DMA_ERQ_ERQ8_MASK)
#define DMA_ERQ_ERQ9_MASK                        (0x200U)
#define DMA_ERQ_ERQ9_SHIFT                       (9U)
#define DMA_ERQ_ERQ9_WIDTH                       (1U)
#define DMA_ERQ_ERQ9(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ9_SHIFT)) & DMA_ERQ_ERQ9_MASK)
#define DMA_ERQ_ERQ10_MASK                       (0x400U)
#define DMA_ERQ_ERQ10_SHIFT                      (10U)
#define DMA_ERQ_ERQ10_WIDTH                      (1U)
#define DMA_ERQ_ERQ10(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ10_SHIFT)) & DMA_ERQ_ERQ10_MASK)
#define DMA_ERQ_ERQ11_MASK                       (0x800U)
#define DMA_ERQ_ERQ11_SHIFT                      (11U)
#define DMA_ERQ_ERQ11_WIDTH                      (1U)
#define DMA_ERQ_ERQ11(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ11_SHIFT)) & DMA_ERQ_ERQ11_MASK)
#define DMA_ERQ_ERQ12_MASK                       (0x1000U)
#define DMA_ERQ_ERQ12_SHIFT                      (12U)
#define DMA_ERQ_ERQ12_WIDTH                      (1U)
#define DMA_ERQ_ERQ12(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ12_SHIFT)) & DMA_ERQ_ERQ12_MASK)
#define DMA_ERQ_ERQ13_MASK                       (0x2000U)
#define DMA_ERQ_ERQ13_SHIFT                      (13U)
#define DMA_ERQ_ERQ13_WIDTH                      (1U)
#define DMA_ERQ_ERQ13(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ13_SHIFT)) & DMA_ERQ_ERQ13_MASK)
#define DMA_ERQ_ERQ14_MASK                       (0x4000U)
#define DMA_ERQ_ERQ14_SHIFT                      (14U)
#define DMA_ERQ_ERQ14_WIDTH                      (1U)
#define DMA_ERQ_ERQ14(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ14_SHIFT)) & DMA_ERQ_ERQ14_MASK)
#define DMA_ERQ_ERQ15_MASK                       (0x8000U)
#define DMA_ERQ_ERQ15_SHIFT                      (15U)
#define DMA_ERQ_ERQ15_WIDTH                      (1U)
#define DMA_ERQ_ERQ15(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ15_SHIFT)) & DMA_ERQ_ERQ15_MASK)
/*! @} */

/*! @name EEI - Enable Error Interrupt Register */
/*! @{ */
#define DMA_EEI_EEI0_MASK                        (0x1U)
#define DMA_EEI_EEI0_SHIFT                       (0U)
#define DMA_EEI_EEI0_WIDTH                       (1U)
#define DMA_EEI_EEI0(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI0_SHIFT)) & DMA_EEI_EEI0_MASK)
#define DMA_EEI_EEI1_MASK                        (0x2U)
#define DMA_EEI_EEI1_SHIFT                       (1U)
#define DMA_EEI_EEI1_WIDTH                       (1U)
#define DMA_EEI_EEI1(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI1_SHIFT)) & DMA_EEI_EEI1_MASK)
#define DMA_EEI_EEI2_MASK                        (0x4U)
#define DMA_EEI_EEI2_SHIFT                       (2U)
#define DMA_EEI_EEI2_WIDTH                       (1U)
#define DMA_EEI_EEI2(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI2_SHIFT)) & DMA_EEI_EEI2_MASK)
#define DMA_EEI_EEI3_MASK                        (0x8U)
#define DMA_EEI_EEI3_SHIFT                       (3U)
#define DMA_EEI_EEI3_WIDTH                       (1U)
#define DMA_EEI_EEI3(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI3_SHIFT)) & DMA_EEI_EEI3_MASK)
#define DMA_EEI_EEI4_MASK                        (0x10U)
#define DMA_EEI_EEI4_SHIFT                       (4U)
#define DMA_EEI_EEI4_WIDTH                       (1U)
#define DMA_EEI_EEI4(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI4_SHIFT)) & DMA_EEI_EEI4_MASK)
#define DMA_EEI_EEI5_MASK                        (0x20U)
#define DMA_EEI_EEI5_SHIFT                       (5U)
#define DMA_EEI_EEI5_WIDTH                       (1U)
#define DMA_EEI_EEI5(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI5_SHIFT)) & DMA_EEI_EEI5_MASK)
#define DMA_EEI_EEI6_MASK                        (0x40U)
#define DMA_EEI_EEI6_SHIFT                       (6U)
#define DMA_EEI_EEI6_WIDTH                       (1U)
#define DMA_EEI_EEI6(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI6_SHIFT)) & DMA_EEI_EEI6_MASK)
#define DMA_EEI_EEI7_MASK                        (0x80U)
#define DMA_EEI_EEI7_SHIFT                       (7U)
#define DMA_EEI_EEI7_WIDTH                       (1U)
#define DMA_EEI_EEI7(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI7_SHIFT)) & DMA_EEI_EEI7_MASK)
#define DMA_EEI_EEI8_MASK                        (0x100U)
#define DMA_EEI_EEI8_SHIFT                       (8U)
#define DMA_EEI_EEI8_WIDTH                       (1U)
#define DMA_EEI_EEI8(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI8_SHIFT)) & DMA_EEI_EEI8_MASK)
#define DMA_EEI_EEI9_MASK                        (0x200U)
#define DMA_EEI_EEI9_SHIFT                       (9U)
#define DMA_EEI_EEI9_WIDTH                       (1U)
#define DMA_EEI_EEI9(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI9_SHIFT)) & DMA_EEI_EEI9_MASK)
#define DMA_EEI_EEI10_MASK                       (0x400U)
#define DMA_EEI_EEI10_SHIFT                      (10U)
#define DMA_EEI_EEI10_WIDTH                      (1U)
#define DMA_EEI_EEI10(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI10_SHIFT)) & DMA_EEI_EEI10_MASK)
#define DMA_EEI_EEI11_MASK                       (0x800U)
#define DMA_EEI_EEI11_SHIFT                      (11U)
#define DMA_EEI_EEI11_WIDTH                      (1U)
#define DMA_EEI_EEI11(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI11_SHIFT)) & DMA_EEI_EEI11_MASK)
#define DMA_EEI_EEI12_MASK                       (0x1000U)
#define DMA_EEI_EEI12_SHIFT                      (12U)
#define DMA_EEI_EEI12_WIDTH                      (1U)
#define DMA_EEI_EEI12(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI12_SHIFT)) & DMA_EEI_EEI12_MASK)
#define DMA_EEI_EEI13_MASK                       (0x2000U)
#define DMA_EEI_EEI13_SHIFT                      (13U)
#define DMA_EEI_EEI13_WIDTH                      (1U)
#define DMA_EEI_EEI13(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI13_SHIFT)) & DMA_EEI_EEI13_MASK)
#define DMA_EEI_EEI14_MASK                       (0x4000U)
#define DMA_EEI_EEI14_SHIFT                      (14U)
#define DMA_EEI_EEI14_WIDTH                      (1U)
#define DMA_EEI_EEI14(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI14_SHIFT)) & DMA_EEI_EEI14_MASK)
#define DMA_EEI_EEI15_MASK                       (0x8000U)
#define DMA_EEI_EEI15_SHIFT                      (15U)
#define DMA_EEI_EEI15_WIDTH                      (1U)
#define DMA_EEI_EEI15(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI15_SHIFT)) & DMA_EEI_EEI15_MASK)
/*! @} */

/*! @name CEEI - Clear Enable Error Interrupt Register */
/*! @{ */
#define DMA_CEEI_CEEI_MASK                       (0xFU)
#define DMA_CEEI_CEEI_SHIFT                      (0U)
#define DMA_CEEI_CEEI_WIDTH                      (4U)
#define DMA_CEEI_CEEI(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CEEI_CEEI_SHIFT)) & DMA_CEEI_CEEI_MASK)
#define DMA_CEEI_CAEE_MASK                       (0x40U)
#define DMA_CEEI_CAEE_SHIFT                      (6U)
#define DMA_CEEI_CAEE_WIDTH                      (1U)
#define DMA_CEEI_CAEE(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CEEI_CAEE_SHIFT)) & DMA_CEEI_CAEE_MASK)
#define DMA_CEEI_NOP_MASK                        (0x80U)
#define DMA_CEEI_NOP_SHIFT                       (7U)
#define DMA_CEEI_NOP_WIDTH                       (1U)
#define DMA_CEEI_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_CEEI_NOP_SHIFT)) & DMA_CEEI_NOP_MASK)
/*! @} */

/*! @name SEEI - Set Enable Error Interrupt Register */
/*! @{ */
#define DMA_SEEI_SEEI_MASK                       (0xFU)
#define DMA_SEEI_SEEI_SHIFT                      (0U)
#define DMA_SEEI_SEEI_WIDTH                      (4U)
#define DMA_SEEI_SEEI(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_SEEI_SEEI_SHIFT)) & DMA_SEEI_SEEI_MASK)
#define DMA_SEEI_SAEE_MASK                       (0x40U)
#define DMA_SEEI_SAEE_SHIFT                      (6U)
#define DMA_SEEI_SAEE_WIDTH                      (1U)
#define DMA_SEEI_SAEE(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_SEEI_SAEE_SHIFT)) & DMA_SEEI_SAEE_MASK)
#define DMA_SEEI_NOP_MASK                        (0x80U)
#define DMA_SEEI_NOP_SHIFT                       (7U)
#define DMA_SEEI_NOP_WIDTH                       (1U)
#define DMA_SEEI_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_SEEI_NOP_SHIFT)) & DMA_SEEI_NOP_MASK)
/*! @} */

/*! @name CERQ - Clear Enable Request Register */
/*! @{ */
#define DMA_CERQ_CERQ_MASK                       (0xFU)
#define DMA_CERQ_CERQ_SHIFT                      (0U)
#define DMA_CERQ_CERQ_WIDTH                      (4U)
#define DMA_CERQ_CERQ(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CERQ_CERQ_SHIFT)) & DMA_CERQ_CERQ_MASK)
#define DMA_CERQ_CAER_MASK                       (0x40U)
#define DMA_CERQ_CAER_SHIFT                      (6U)
#define DMA_CERQ_CAER_WIDTH                      (1U)
#define DMA_CERQ_CAER(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CERQ_CAER_SHIFT)) & DMA_CERQ_CAER_MASK)
#define DMA_CERQ_NOP_MASK                        (0x80U)
#define DMA_CERQ_NOP_SHIFT                       (7U)
#define DMA_CERQ_NOP_WIDTH                       (1U)
#define DMA_CERQ_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_CERQ_NOP_SHIFT)) & DMA_CERQ_NOP_MASK)
/*! @} */

/*! @name SERQ - Set Enable Request Register */
/*! @{ */
#define DMA_SERQ_SERQ_MASK                       (0xFU)
#define DMA_SERQ_SERQ_SHIFT                      (0U)
#define DMA_SERQ_SERQ_WIDTH                      (4U)
#define DMA_SERQ_SERQ(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_SERQ_SERQ_SHIFT)) & DMA_SERQ_SERQ_MASK)
#define DMA_SERQ_SAER_MASK                       (0x40U)
#define DMA_SERQ_SAER_SHIFT                      (6U)
#define DMA_SERQ_SAER_WIDTH                      (1U)
#define DMA_SERQ_SAER(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_SERQ_SAER_SHIFT)) & DMA_SERQ_SAER_MASK)
#define DMA_SERQ_NOP_MASK                        (0x80U)
#define DMA_SERQ_NOP_SHIFT                       (7U)
#define DMA_SERQ_NOP_WIDTH                       (1U)
#define DMA_SERQ_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_SERQ_NOP_SHIFT)) & DMA_SERQ_NOP_MASK)
/*! @} */

/*! @name CDNE - Clear DONE Status Bit Register */
/*! @{ */
#define DMA_CDNE_CDNE_MASK                       (0xFU)
#define DMA_CDNE_CDNE_SHIFT                      (0U)
#define DMA_CDNE_CDNE_WIDTH                      (4U)
#define DMA_CDNE_CDNE(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CDNE_CDNE_SHIFT)) & DMA_CDNE_CDNE_MASK)
#define DMA_CDNE_CADN_MASK                       (0x40U)
#define DMA_CDNE_CADN_SHIFT                      (6U)
#define DMA_CDNE_CADN_WIDTH                      (1U)
#define DMA_CDNE_CADN(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CDNE_CADN_SHIFT)) & DMA_CDNE_CADN_MASK)
#define DMA_CDNE_NOP_MASK                        (0x80U)
#define DMA_CDNE_NOP_SHIFT                       (7U)
#define DMA_CDNE_NOP_WIDTH                       (1U)
#define DMA_CDNE_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_CDNE_NOP_SHIFT)) & DMA_CDNE_NOP_MASK)
/*! @} */

/*! @name SSRT - Set START Bit Register */
/*! @{ */
#define DMA_SSRT_SSRT_MASK                       (0xFU)
#define DMA_SSRT_SSRT_SHIFT                      (0U)
#define DMA_SSRT_SSRT_WIDTH                      (4U)
#define DMA_SSRT_SSRT(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_SSRT_SSRT_SHIFT)) & DMA_SSRT_SSRT_MASK)
#define DMA_SSRT_SAST_MASK                       (0x40U)
#define DMA_SSRT_SAST_SHIFT                      (6U)
#define DMA_SSRT_SAST_WIDTH                      (1U)
#define DMA_SSRT_SAST(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_SSRT_SAST_SHIFT)) & DMA_SSRT_SAST_MASK)
#define DMA_SSRT_NOP_MASK                        (0x80U)
#define DMA_SSRT_NOP_SHIFT                       (7U)
#define DMA_SSRT_NOP_WIDTH                       (1U)
#define DMA_SSRT_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_SSRT_NOP_SHIFT)) & DMA_SSRT_NOP_MASK)
/*! @} */

/*! @name CERR - Clear Error Register */
/*! @{ */
#define DMA_CERR_CERR_MASK                       (0xFU)
#define DMA_CERR_CERR_SHIFT                      (0U)
#define DMA_CERR_CERR_WIDTH                      (4U)
#define DMA_CERR_CERR(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CERR_CERR_SHIFT)) & DMA_CERR_CERR_MASK)
#define DMA_CERR_CAEI_MASK                       (0x40U)
#define DMA_CERR_CAEI_SHIFT                      (6U)
#define DMA_CERR_CAEI_WIDTH                      (1U)
#define DMA_CERR_CAEI(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CERR_CAEI_SHIFT)) & DMA_CERR_CAEI_MASK)
#define DMA_CERR_NOP_MASK                        (0x80U)
#define DMA_CERR_NOP_SHIFT                       (7U)
#define DMA_CERR_NOP_WIDTH                       (1U)
#define DMA_CERR_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_CERR_NOP_SHIFT)) & DMA_CERR_NOP_MASK)
/*! @} */

/*! @name CINT - Clear Interrupt Request Register */
/*! @{ */
#define DMA_CINT_CINT_MASK                       (0xFU)
#define DMA_CINT_CINT_SHIFT                      (0U)
#define DMA_CINT_CINT_WIDTH                      (4U)
#define DMA_CINT_CINT(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CINT_CINT_SHIFT)) & DMA_CINT_CINT_MASK)
#define DMA_CINT_CAIR_MASK                       (0x40U)
#define DMA_CINT_CAIR_SHIFT                      (6U)
#define DMA_CINT_CAIR_WIDTH                      (1U)
#define DMA_CINT_CAIR(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CINT_CAIR_SHIFT)) & DMA_CINT_CAIR_MASK)
#define DMA_CINT_NOP_MASK                        (0x80U)
#define DMA_CINT_NOP_SHIFT                       (7U)
#define DMA_CINT_NOP_WIDTH                       (1U)
#define DMA_CINT_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_CINT_NOP_SHIFT)) & DMA_CINT_NOP_MASK)
/*! @} */

/*! @name INT - Interrupt Request Register */
/*! @{ */
#define DMA_INT_INT0_MASK                        (0x1U)
#define DMA_INT_INT0_SHIFT                       (0U)
#define DMA_INT_INT0_WIDTH                       (1U)
#define DMA_INT_INT0(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT0_SHIFT)) & DMA_INT_INT0_MASK)
#define DMA_INT_INT1_MASK                        (0x2U)
#define DMA_INT_INT1_SHIFT                       (1U)
#define DMA_INT_INT1_WIDTH                       (1U)
#define DMA_INT_INT1(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT1_SHIFT)) & DMA_INT_INT1_MASK)
#define DMA_INT_INT2_MASK                        (0x4U)
#define DMA_INT_INT2_SHIFT                       (2U)
#define DMA_INT_INT2_WIDTH                       (1U)
#define DMA_INT_INT2(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT2_SHIFT)) & DMA_INT_INT2_MASK)
#define DMA_INT_INT3_MASK                        (0x8U)
#define DMA_INT_INT3_SHIFT                       (3U)
#define DMA_INT_INT3_WIDTH                       (1U)
#define DMA_INT_INT3(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT3_SHIFT)) & DMA_INT_INT3_MASK)
#define DMA_INT_INT4_MASK                        (0x10U)
#define DMA_INT_INT4_SHIFT                       (4U)
#define DMA_INT_INT4_WIDTH                       (1U)
#define DMA_INT_INT4(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT4_SHIFT)) & DMA_INT_INT4_MASK)
#define DMA_INT_INT5_MASK                        (0x20U)
#define DMA_INT_INT5_SHIFT                       (5U)
#define DMA_INT_INT5_WIDTH                       (1U)
#define DMA_INT_INT5(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT5_SHIFT)) & DMA_INT_INT5_MASK)
#define DMA_INT_INT6_MASK                        (0x40U)
#define DMA_INT_INT6_SHIFT                       (6U)
#define DMA_INT_INT6_WIDTH                       (1U)
#define DMA_INT_INT6(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT6_SHIFT)) & DMA_INT_INT6_MASK)
#define DMA_INT_INT7_MASK                        (0x80U)
#define DMA_INT_INT7_SHIFT                       (7U)
#define DMA_INT_INT7_WIDTH                       (1U)
#define DMA_INT_INT7(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT7_SHIFT)) & DMA_INT_INT7_MASK)
#define DMA_INT_INT8_MASK                        (0x100U)
#define DMA_INT_INT8_SHIFT                       (8U)
#define DMA_INT_INT8_WIDTH                       (1U)
#define DMA_INT_INT8(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT8_SHIFT)) & DMA_INT_INT8_MASK)
#define DMA_INT_INT9_MASK                        (0x200U)
#define DMA_INT_INT9_SHIFT                       (9U)
#define DMA_INT_INT9_WIDTH                       (1U)
#define DMA_INT_INT9(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT9_SHIFT)) & DMA_INT_INT9_MASK)
#define DMA_INT_INT10_MASK                       (0x400U)
#define DMA_INT_INT10_SHIFT                      (10U)
#define DMA_INT_INT10_WIDTH                      (1U)
#define DMA_INT_INT10(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT10_SHIFT)) & DMA_INT_INT10_MASK)
#define DMA_INT_INT11_MASK                       (0x800U)
#define DMA_INT_INT11_SHIFT                      (11U)
#define DMA_INT_INT11_WIDTH                      (1U)
#define DMA_INT_INT11(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT11_SHIFT)) & DMA_INT_INT11_MASK)
#define DMA_INT_INT12_MASK                       (0x1000U)
#define DMA_INT_INT12_SHIFT                      (12U)
#define DMA_INT_INT12_WIDTH                      (1U)
#define DMA_INT_INT12(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT12_SHIFT)) & DMA_INT_INT12_MASK)
#define DMA_INT_INT13_MASK                       (0x2000U)
#define DMA_INT_INT13_SHIFT                      (13U)
#define DMA_INT_INT13_WIDTH                      (1U)
#define DMA_INT_INT13(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT13_SHIFT)) & DMA_INT_INT13_MASK)
#define DMA_INT_INT14_MASK                       (0x4000U)
#define DMA_INT_INT14_SHIFT                      (14U)
#define DMA_INT_INT14_WIDTH                      (1U)
#define DMA_INT_INT14(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT14_SHIFT)) & DMA_INT_INT14_MASK)
#define DMA_INT_INT15_MASK                       (0x8000U)
#define DMA_INT_INT15_SHIFT                      (15U)
#define DMA_INT_INT15_WIDTH                      (1U)
#define DMA_INT_INT15(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT15_SHIFT)) & DMA_INT_INT15_MASK)
/*! @} */

/*! @name ERR - Error Register */
/*! @{ */
#define DMA_ERR_ERR0_MASK                        (0x1U)
#define DMA_ERR_ERR0_SHIFT                       (0U)
#define DMA_ERR_ERR0_WIDTH                       (1U)
#define DMA_ERR_ERR0(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR0_SHIFT)) & DMA_ERR_ERR0_MASK)
#define DMA_ERR_ERR1_MASK                        (0x2U)
#define DMA_ERR_ERR1_SHIFT                       (1U)
#define DMA_ERR_ERR1_WIDTH                       (1U)
#define DMA_ERR_ERR1(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR1_SHIFT)) & DMA_ERR_ERR1_MASK)
#define DMA_ERR_ERR2_MASK                        (0x4U)
#define DMA_ERR_ERR2_SHIFT                       (2U)
#define DMA_ERR_ERR2_WIDTH                       (1U)
#define DMA_ERR_ERR2(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR2_SHIFT)) & DMA_ERR_ERR2_MASK)
#define DMA_ERR_ERR3_MASK                        (0x8U)
#define DMA_ERR_ERR3_SHIFT                       (3U)
#define DMA_ERR_ERR3_WIDTH                       (1U)
#define DMA_ERR_ERR3(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR3_SHIFT)) & DMA_ERR_ERR3_MASK)
#define DMA_ERR_ERR4_MASK                        (0x10U)
#define DMA_ERR_ERR4_SHIFT                       (4U)
#define DMA_ERR_ERR4_WIDTH                       (1U)
#define DMA_ERR_ERR4(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR4_SHIFT)) & DMA_ERR_ERR4_MASK)
#define DMA_ERR_ERR5_MASK                        (0x20U)
#define DMA_ERR_ERR5_SHIFT                       (5U)
#define DMA_ERR_ERR5_WIDTH                       (1U)
#define DMA_ERR_ERR5(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR5_SHIFT)) & DMA_ERR_ERR5_MASK)
#define DMA_ERR_ERR6_MASK                        (0x40U)
#define DMA_ERR_ERR6_SHIFT                       (6U)
#define DMA_ERR_ERR6_WIDTH                       (1U)
#define DMA_ERR_ERR6(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR6_SHIFT)) & DMA_ERR_ERR6_MASK)
#define DMA_ERR_ERR7_MASK                        (0x80U)
#define DMA_ERR_ERR7_SHIFT                       (7U)
#define DMA_ERR_ERR7_WIDTH                       (1U)
#define DMA_ERR_ERR7(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR7_SHIFT)) & DMA_ERR_ERR7_MASK)
#define DMA_ERR_ERR8_MASK                        (0x100U)
#define DMA_ERR_ERR8_SHIFT                       (8U)
#define DMA_ERR_ERR8_WIDTH                       (1U)
#define DMA_ERR_ERR8(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR8_SHIFT)) & DMA_ERR_ERR8_MASK)
#define DMA_ERR_ERR9_MASK                        (0x200U)
#define DMA_ERR_ERR9_SHIFT                       (9U)
#define DMA_ERR_ERR9_WIDTH                       (1U)
#define DMA_ERR_ERR9(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR9_SHIFT)) & DMA_ERR_ERR9_MASK)
#define DMA_ERR_ERR10_MASK                       (0x400U)
#define DMA_ERR_ERR10_SHIFT                      (10U)
#define DMA_ERR_ERR10_WIDTH                      (1U)
#define DMA_ERR_ERR10(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR10_SHIFT)) & DMA_ERR_ERR10_MASK)
#define DMA_ERR_ERR11_MASK                       (0x800U)
#define DMA_ERR_ERR11_SHIFT                      (11U)
#define DMA_ERR_ERR11_WIDTH                      (1U)
#define DMA_ERR_ERR11(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR11_SHIFT)) & DMA_ERR_ERR11_MASK)
#define DMA_ERR_ERR12_MASK                       (0x1000U)
#define DMA_ERR_ERR12_SHIFT                      (12U)
#define DMA_ERR_ERR12_WIDTH                      (1U)
#define DMA_ERR_ERR12(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR12_SHIFT)) & DMA_ERR_ERR12_MASK)
#define DMA_ERR_ERR13_MASK                       (0x2000U)
#define DMA_ERR_ERR13_SHIFT                      (13U)
#define DMA_ERR_ERR13_WIDTH                      (1U)
#define DMA_ERR_ERR13(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR13_SHIFT)) & DMA_ERR_ERR13_MASK)
#define DMA_ERR_ERR14_MASK                       (0x4000U)
#define DMA_ERR_ERR14_SHIFT                      (14U)
#define DMA_ERR_ERR14_WIDTH                      (1U)
#define DMA_ERR_ERR14(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR14_SHIFT)) & DMA_ERR_ERR14_MASK)
#define DMA_ERR_ERR15_MASK                       (0x8000U)
#define DMA_ERR_ERR15_SHIFT                      (15U)
#define DMA_ERR_ERR15_WIDTH                      (1U)
#define DMA_ERR_ERR15(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR15_SHIFT)) & DMA_ERR_ERR15_MASK)
/*! @} */

/*! @name HRS - Hardware Request Status Register */
/*! @{ */
#define DMA_HRS_HRS0_MASK                        (0x1U)
#define DMA_HRS_HRS0_SHIFT                       (0U)
#define DMA_HRS_HRS0_WIDTH                       (1U)
#define DMA_HRS_HRS0(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS0_SHIFT)) & DMA_HRS_HRS0_MASK)
#define DMA_HRS_HRS1_MASK                        (0x2U)
#define DMA_HRS_HRS1_SHIFT                       (1U)
#define DMA_HRS_HRS1_WIDTH                       (1U)
#define DMA_HRS_HRS1(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS1_SHIFT)) & DMA_HRS_HRS1_MASK)
#define DMA_HRS_HRS2_MASK                        (0x4U)
#define DMA_HRS_HRS2_SHIFT                       (2U)
#define DMA_HRS_HRS2_WIDTH                       (1U)
#define DMA_HRS_HRS2(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS2_SHIFT)) & DMA_HRS_HRS2_MASK)
#define DMA_HRS_HRS3_MASK                        (0x8U)
#define DMA_HRS_HRS3_SHIFT                       (3U)
#define DMA_HRS_HRS3_WIDTH                       (1U)
#define DMA_HRS_HRS3(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS3_SHIFT)) & DMA_HRS_HRS3_MASK)
#define DMA_HRS_HRS4_MASK                        (0x10U)
#define DMA_HRS_HRS4_SHIFT                       (4U)
#define DMA_HRS_HRS4_WIDTH                       (1U)
#define DMA_HRS_HRS4(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS4_SHIFT)) & DMA_HRS_HRS4_MASK)
#define DMA_HRS_HRS5_MASK                        (0x20U)
#define DMA_HRS_HRS5_SHIFT                       (5U)
#define DMA_HRS_HRS5_WIDTH                       (1U)
#define DMA_HRS_HRS5(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS5_SHIFT)) & DMA_HRS_HRS5_MASK)
#define DMA_HRS_HRS6_MASK                        (0x40U)
#define DMA_HRS_HRS6_SHIFT                       (6U)
#define DMA_HRS_HRS6_WIDTH                       (1U)
#define DMA_HRS_HRS6(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS6_SHIFT)) & DMA_HRS_HRS6_MASK)
#define DMA_HRS_HRS7_MASK                        (0x80U)
#define DMA_HRS_HRS7_SHIFT                       (7U)
#define DMA_HRS_HRS7_WIDTH                       (1U)
#define DMA_HRS_HRS7(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS7_SHIFT)) & DMA_HRS_HRS7_MASK)
#define DMA_HRS_HRS8_MASK                        (0x100U)
#define DMA_HRS_HRS8_SHIFT                       (8U)
#define DMA_HRS_HRS8_WIDTH                       (1U)
#define DMA_HRS_HRS8(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS8_SHIFT)) & DMA_HRS_HRS8_MASK)
#define DMA_HRS_HRS9_MASK                        (0x200U)
#define DMA_HRS_HRS9_SHIFT                       (9U)
#define DMA_HRS_HRS9_WIDTH                       (1U)
#define DMA_HRS_HRS9(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS9_SHIFT)) & DMA_HRS_HRS9_MASK)
#define DMA_HRS_HRS10_MASK                       (0x400U)
#define DMA_HRS_HRS10_SHIFT                      (10U)
#define DMA_HRS_HRS10_WIDTH                      (1U)
#define DMA_HRS_HRS10(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS10_SHIFT)) & DMA_HRS_HRS10_MASK)
#define DMA_HRS_HRS11_MASK                       (0x800U)
#define DMA_HRS_HRS11_SHIFT                      (11U)
#define DMA_HRS_HRS11_WIDTH                      (1U)
#define DMA_HRS_HRS11(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS11_SHIFT)) & DMA_HRS_HRS11_MASK)
#define DMA_HRS_HRS12_MASK                       (0x1000U)
#define DMA_HRS_HRS12_SHIFT                      (12U)
#define DMA_HRS_HRS12_WIDTH                      (1U)
#define DMA_HRS_HRS12(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS12_SHIFT)) & DMA_HRS_HRS12_MASK)
#define DMA_HRS_HRS13_MASK                       (0x2000U)
#define DMA_HRS_HRS13_SHIFT                      (13U)
#define DMA_HRS_HRS13_WIDTH                      (1U)
#define DMA_HRS_HRS13(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS13_SHIFT)) & DMA_HRS_HRS13_MASK)
#define DMA_HRS_HRS14_MASK                       (0x4000U)
#define DMA_HRS_HRS14_SHIFT                      (14U)
#define DMA_HRS_HRS14_WIDTH                      (1U)
#define DMA_HRS_HRS14(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS14_SHIFT)) & DMA_HRS_HRS14_MASK)
#define DMA_HRS_HRS15_MASK                       (0x8000U)
#define DMA_HRS_HRS15_SHIFT                      (15U)
#define DMA_HRS_HRS15_WIDTH                      (1U)
#define DMA_HRS_HRS15(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS15_SHIFT)) & DMA_HRS_HRS15_MASK)
/*! @} */

/*! @name EARS - Enable Asynchronous Request in Stop Register */
/*! @{ */
#define DMA_EARS_EDREQ_0_MASK                    (0x1U)
#define DMA_EARS_EDREQ_0_SHIFT                   (0U)
#define DMA_EARS_EDREQ_0_WIDTH                   (1U)
#define DMA_EARS_EDREQ_0(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_0_SHIFT)) & DMA_EARS_EDREQ_0_MASK)
#define DMA_EARS_EDREQ_1_MASK                    (0x2U)
#define DMA_EARS_EDREQ_1_SHIFT                   (1U)
#define DMA_EARS_EDREQ_1_WIDTH                   (1U)
#define DMA_EARS_EDREQ_1(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_1_SHIFT)) & DMA_EARS_EDREQ_1_MASK)
#define DMA_EARS_EDREQ_2_MASK                    (0x4U)
#define DMA_EARS_EDREQ_2_SHIFT                   (2U)
#define DMA_EARS_EDREQ_2_WIDTH                   (1U)
#define DMA_EARS_EDREQ_2(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_2_SHIFT)) & DMA_EARS_EDREQ_2_MASK)
#define DMA_EARS_EDREQ_3_MASK                    (0x8U)
#define DMA_EARS_EDREQ_3_SHIFT                   (3U)
#define DMA_EARS_EDREQ_3_WIDTH                   (1U)
#define DMA_EARS_EDREQ_3(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_3_SHIFT)) & DMA_EARS_EDREQ_3_MASK)
#define DMA_EARS_EDREQ_4_MASK                    (0x10U)
#define DMA_EARS_EDREQ_4_SHIFT                   (4U)
#define DMA_EARS_EDREQ_4_WIDTH                   (1U)
#define DMA_EARS_EDREQ_4(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_4_SHIFT)) & DMA_EARS_EDREQ_4_MASK)
#define DMA_EARS_EDREQ_5_MASK                    (0x20U)
#define DMA_EARS_EDREQ_5_SHIFT                   (5U)
#define DMA_EARS_EDREQ_5_WIDTH                   (1U)
#define DMA_EARS_EDREQ_5(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_5_SHIFT)) & DMA_EARS_EDREQ_5_MASK)
#define DMA_EARS_EDREQ_6_MASK                    (0x40U)
#define DMA_EARS_EDREQ_6_SHIFT                   (6U)
#define DMA_EARS_EDREQ_6_WIDTH                   (1U)
#define DMA_EARS_EDREQ_6(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_6_SHIFT)) & DMA_EARS_EDREQ_6_MASK)
#define DMA_EARS_EDREQ_7_MASK                    (0x80U)
#define DMA_EARS_EDREQ_7_SHIFT                   (7U)
#define DMA_EARS_EDREQ_7_WIDTH                   (1U)
#define DMA_EARS_EDREQ_7(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_7_SHIFT)) & DMA_EARS_EDREQ_7_MASK)
#define DMA_EARS_EDREQ_8_MASK                    (0x100U)
#define DMA_EARS_EDREQ_8_SHIFT                   (8U)
#define DMA_EARS_EDREQ_8_WIDTH                   (1U)
#define DMA_EARS_EDREQ_8(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_8_SHIFT)) & DMA_EARS_EDREQ_8_MASK)
#define DMA_EARS_EDREQ_9_MASK                    (0x200U)
#define DMA_EARS_EDREQ_9_SHIFT                   (9U)
#define DMA_EARS_EDREQ_9_WIDTH                   (1U)
#define DMA_EARS_EDREQ_9(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_9_SHIFT)) & DMA_EARS_EDREQ_9_MASK)
#define DMA_EARS_EDREQ_10_MASK                   (0x400U)
#define DMA_EARS_EDREQ_10_SHIFT                  (10U)
#define DMA_EARS_EDREQ_10_WIDTH                  (1U)
#define DMA_EARS_EDREQ_10(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_10_SHIFT)) & DMA_EARS_EDREQ_10_MASK)
#define DMA_EARS_EDREQ_11_MASK                   (0x800U)
#define DMA_EARS_EDREQ_11_SHIFT                  (11U)
#define DMA_EARS_EDREQ_11_WIDTH                  (1U)
#define DMA_EARS_EDREQ_11(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_11_SHIFT)) & DMA_EARS_EDREQ_11_MASK)
#define DMA_EARS_EDREQ_12_MASK                   (0x1000U)
#define DMA_EARS_EDREQ_12_SHIFT                  (12U)
#define DMA_EARS_EDREQ_12_WIDTH                  (1U)
#define DMA_EARS_EDREQ_12(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_12_SHIFT)) & DMA_EARS_EDREQ_12_MASK)
#define DMA_EARS_EDREQ_13_MASK                   (0x2000U)
#define DMA_EARS_EDREQ_13_SHIFT                  (13U)
#define DMA_EARS_EDREQ_13_WIDTH                  (1U)
#define DMA_EARS_EDREQ_13(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_13_SHIFT)) & DMA_EARS_EDREQ_13_MASK)
#define DMA_EARS_EDREQ_14_MASK                   (0x4000U)
#define DMA_EARS_EDREQ_14_SHIFT                  (14U)
#define DMA_EARS_EDREQ_14_WIDTH                  (1U)
#define DMA_EARS_EDREQ_14(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_14_SHIFT)) & DMA_EARS_EDREQ_14_MASK)
#define DMA_EARS_EDREQ_15_MASK                   (0x8000U)
#define DMA_EARS_EDREQ_15_SHIFT                  (15U)
#define DMA_EARS_EDREQ_15_WIDTH                  (1U)
#define DMA_EARS_EDREQ_15(x)                     (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_15_SHIFT)) & DMA_EARS_EDREQ_15_MASK)
/*! @} */

/*! @name DCHPRI - Channel Priority Register */
/*! @{ */
#define DMA_DCHPRI_CHPRI_MASK                    (0xFU)
#define DMA_DCHPRI_CHPRI_SHIFT                   (0U)
#define DMA_DCHPRI_CHPRI_WIDTH                   (4U)
#define DMA_DCHPRI_CHPRI(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI_CHPRI_SHIFT)) & DMA_DCHPRI_CHPRI_MASK)
#define DMA_DCHPRI_DPA_MASK                      (0x40U)
#define DMA_DCHPRI_DPA_SHIFT                     (6U)
#define DMA_DCHPRI_DPA_WIDTH                     (1U)
#define DMA_DCHPRI_DPA(x)                        (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI_DPA_SHIFT)) & DMA_DCHPRI_DPA_MASK)
#define DMA_DCHPRI_ECP_MASK                      (0x80U)
#define DMA_DCHPRI_ECP_SHIFT                     (7U)
#define DMA_DCHPRI_ECP_WIDTH                     (1U)
#define DMA_DCHPRI_ECP(x)                        (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI_ECP_SHIFT)) & DMA_DCHPRI_ECP_MASK)
/*! @} */

/*! @name TCD_SADDR - TCD Source Address */
/*! @{ */
#define DMA_TCD_SADDR_SADDR_MASK                 (0xFFFFFFFFU)
#define DMA_TCD_SADDR_SADDR_SHIFT                (0U)
#define DMA_TCD_SADDR_SADDR_WIDTH                (32U)
#define DMA_TCD_SADDR_SADDR(x)                   (((uint32_t)(((uint32_t)(x)) << DMA_TCD_SADDR_SADDR_SHIFT)) & DMA_TCD_SADDR_SADDR_MASK)
/*! @} */

/*! @name TCD_SOFF - TCD Signed Source Address Offset */
/*! @{ */
#define DMA_TCD_SOFF_SOFF_MASK                   (0xFFFFU)
#define DMA_TCD_SOFF_SOFF_SHIFT                  (0U)
#define DMA_TCD_SOFF_SOFF_WIDTH                  (16U)
#define DMA_TCD_SOFF_SOFF(x)                     (((uint16_t)(((uint16_t)(x)) << DMA_TCD_SOFF_SOFF_SHIFT)) & DMA_TCD_SOFF_SOFF_MASK)
/*! @} */

/*! @name TCD_ATTR - TCD Transfer Attributes */
/*! @{ */
#define DMA_TCD_ATTR_DSIZE_MASK                  (0x7U)
#define DMA_TCD_ATTR_DSIZE_SHIFT                 (0U)
#define DMA_TCD_ATTR_DSIZE_WIDTH                 (3U)
#define DMA_TCD_ATTR_DSIZE(x)                    (((uint16_t)(((uint16_t)(x)) << DMA_TCD_ATTR_DSIZE_SHIFT)) & DMA_TCD_ATTR_DSIZE_MASK)
#define DMA_TCD_ATTR_DMOD_MASK                   (0xF8U)
#define DMA_TCD_ATTR_DMOD_SHIFT                  (3U)
#define DMA_TCD_ATTR_DMOD_WIDTH                  (5U)
#define DMA_TCD_ATTR_DMOD(x)                     (((uint16_t)(((uint16_t)(x)) << DMA_TCD_ATTR_DMOD_SHIFT)) & DMA_TCD_ATTR_DMOD_MASK)
#define DMA_TCD_ATTR_SSIZE_MASK                  (0x700U)
#define DMA_TCD_ATTR_SSIZE_SHIFT                 (8U)
#define DMA_TCD_ATTR_SSIZE_WIDTH                 (3U)
#define DMA_TCD_ATTR_SSIZE(x)                    (((uint16_t)(((uint16_t)(x)) << DMA_TCD_ATTR_SSIZE_SHIFT)) & DMA_TCD_ATTR_SSIZE_MASK)
#define DMA_TCD_ATTR_SMOD_MASK                   (0xF800U)
#define DMA_TCD_ATTR_SMOD_SHIFT                  (11U)
#define DMA_TCD_ATTR_SMOD_WIDTH                  (5U)
#define DMA_TCD_ATTR_SMOD(x)                     (((uint16_t)(((uint16_t)(x)) << DMA_TCD_ATTR_SMOD_SHIFT)) & DMA_TCD_ATTR_SMOD_MASK)
/*! @} */

/*! @name TCD_NBYTES_MLNO - TCD Minor Byte Count (Minor Loop Mapping Disabled) */
/*! @{ */
#define DMA_TCD_NBYTES_MLNO_NBYTES_MASK          (0xFFFFFFFFU)
#define DMA_TCD_NBYTES_MLNO_NBYTES_SHIFT         (0U)
#define DMA_TCD_NBYTES_MLNO_NBYTES_WIDTH         (32U)
#define DMA_TCD_NBYTES_MLNO_NBYTES(x)            (((uint32_t)(((uint32_t)(x)) << DMA_TCD_NBYTES_MLNO_NBYTES_SHIFT)) & DMA_TCD_NBYTES_MLNO_NBYTES_MASK)
/*! @} */

/*! @name TCD_NBYTES_MLOFFNO - TCD Signed Minor Loop Offset (Minor Loop Mapping Enabled and Offset Disabled) */
/*! @{ */
#define DMA_TCD_NBYTES_MLOFFNO_NBYTES_MASK       (0x3FFFFFFFU)
#define DMA_TCD_NBYTES_MLOFFNO_NBYTES_SHIFT      (0U)
#define DMA_TCD_NBYTES_MLOFFNO_NBYTES_WIDTH      (30U)
#define DMA_TCD_NBYTES_MLOFFNO_NBYTES(x)         (((uint32_t)(((uint32_t)(x)) << DMA_TCD_NBYTES_MLOFFNO_NBYTES_SHIFT)) & DMA_TCD_NBYTES_MLOFFNO_NBYTES_MASK)
#define DMA_TCD_NBYTES_MLOFFNO_DMLOE_MASK        (0x40000000U)
#define DMA_TCD_NBYTES_MLOFFNO_DMLOE_SHIFT       (30U)
#define DMA_TCD_NBYTES_MLOFFNO_DMLOE_WIDTH       (1U)
#define DMA_TCD_NBYTES_MLOFFNO_DMLOE(x)          (((uint32_t)(((uint32_t)(x)) << DMA_TCD_NBYTES_MLOFFNO_DMLOE_SHIFT)) & DMA_TCD_NBYTES_MLOFFNO_DMLOE_MASK)
#define DMA_TCD_NBYTES_MLOFFNO_SMLOE_MASK        (0x80000000U)
#define DMA_TCD_NBYTES_MLOFFNO_SMLOE_SHIFT       (31U)
#define DMA_TCD_NBYTES_MLOFFNO_SMLOE_WIDTH       (1U)
#define DMA_TCD_NBYTES_MLOFFNO_SMLOE(x)          (((uint32_t)(((uint32_t)(x)) << DMA_TCD_NBYTES_MLOFFNO_SMLOE_SHIFT)) & DMA_TCD_NBYTES_MLOFFNO_SMLOE_MASK)
/*! @} */

/*! @name TCD_NBYTES_MLOFFYES - TCD Signed Minor Loop Offset (Minor Loop Mapping and Offset Enabled) */
/*! @{ */
#define DMA_TCD_NBYTES_MLOFFYES_NBYTES_MASK      (0x3FFU)
#define DMA_TCD_NBYTES_MLOFFYES_NBYTES_SHIFT     (0U)
#define DMA_TCD_NBYTES_MLOFFYES_NBYTES_WIDTH     (10U)
#define DMA_TCD_NBYTES_MLOFFYES_NBYTES(x)        (((uint32_t)(((uint32_t)(x)) << DMA_TCD_NBYTES_MLOFFYES_NBYTES_SHIFT)) & DMA_TCD_NBYTES_MLOFFYES_NBYTES_MASK)
#define DMA_TCD_NBYTES_MLOFFYES_MLOFF_MASK       (0x3FFFFC00U)
#define DMA_TCD_NBYTES_MLOFFYES_MLOFF_SHIFT      (10U)
#define DMA_TCD_NBYTES_MLOFFYES_MLOFF_WIDTH      (20U)
#define DMA_TCD_NBYTES_MLOFFYES_MLOFF(x)         (((uint32_t)(((uint32_t)(x)) << DMA_TCD_NBYTES_MLOFFYES_MLOFF_SHIFT)) & DMA_TCD_NBYTES_MLOFFYES_MLOFF_MASK)
#define DMA_TCD_NBYTES_MLOFFYES_DMLOE_MASK       (0x40000000U)
#define DMA_TCD_NBYTES_MLOFFYES_DMLOE_SHIFT      (30U)
#define DMA_TCD_NBYTES_MLOFFYES_DMLOE_WIDTH      (1U)
#define DMA_TCD_NBYTES_MLOFFYES_DMLOE(x)         (((uint32_t)(((uint32_t)(x)) << DMA_TCD_NBYTES_MLOFFYES_DMLOE_SHIFT)) & DMA_TCD_NBYTES_MLOFFYES_DMLOE_MASK)
#define DMA_TCD_NBYTES_MLOFFYES_SMLOE_MASK       (0x80000000U)
#define DMA_TCD_NBYTES_MLOFFYES_SMLOE_SHIFT      (31U)
#define DMA_TCD_NBYTES_MLOFFYES_SMLOE_WIDTH      (1U)
#define DMA_TCD_NBYTES_MLOFFYES_SMLOE(x)         (((uint32_t)(((uint32_t)(x)) << DMA_TCD_NBYTES_MLOFFYES_SMLOE_SHIFT)) & DMA_TCD_NBYTES_MLOFFYES_SMLOE_MASK)
/*! @} */

/*! @name TCD_SLAST - TCD Last Source Address Adjustment */
/*! @{ */
#define DMA_TCD_SLAST_SLAST_MASK                 (0xFFFFFFFFU)
#define DMA_TCD_SLAST_SLAST_SHIFT                (0U)
#define DMA_TCD_SLAST_SLAST_WIDTH                (32U)
#define DMA_TCD_SLAST_SLAST(x)                   (((uint32_t)(((uint32_t)(x)) << DMA_TCD_SLAST_SLAST_SHIFT)) & DMA_TCD_SLAST_SLAST_MASK)
/*! @} */

/*! @name TCD_DADDR - TCD Destination Address */
/*! @{ */
#define DMA_TCD_DADDR_DADDR_MASK                 (0xFFFFFFFFU)
#define DMA_TCD_DADDR_DADDR_SHIFT                (0U)
#define DMA_TCD_DADDR_DADDR_WIDTH                (32U)
#define DMA_TCD_DADDR_DADDR(x)                   (((uint32_t)(((uint32_t)(x)) << DMA_TCD_DADDR_DADDR_SHIFT)) & DMA_TCD_DADDR_DADDR_MASK)
/*! @} */

/*! @name TCD_DOFF - TCD Signed Destination Address Offset */
/*! @{ */
#define DMA_TCD_DOFF_DOFF_MASK                   (0xFFFFU)
#define DMA_TCD_DOFF_DOFF_SHIFT                  (0U)
#define DMA_TCD_DOFF_DOFF_WIDTH                  (16U)
#define DMA_TCD_DOFF_DOFF(x)                     (((uint16_t)(((uint16_t)(x)) << DMA_TCD_DOFF_DOFF_SHIFT)) & DMA_TCD_DOFF_DOFF_MASK)
/*! @} */

/*! @name TCD_CITER_ELINKNO - TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled) */
/*! @{ */
#define DMA_TCD_CITER_ELINKNO_CITER_MASK         (0x7FFFU)
#define DMA_TCD_CITER_ELINKNO_CITER_SHIFT        (0U)
#define DMA_TCD_CITER_ELINKNO_CITER_WIDTH        (15U)
#define DMA_TCD_CITER_ELINKNO_CITER(x)           (((uint16_t)(((uint16_t)(x)) << DMA_TCD_CITER_ELINKNO_CITER_SHIFT)) & DMA_TCD_CITER_ELINKNO_CITER_MASK)
#define DMA_TCD_CITER_ELINKNO_ELINK_MASK         (0x8000U)
#define DMA_TCD_CITER_ELINKNO_ELINK_SHIFT        (15U)
#define DMA_TCD_CITER_ELINKNO_ELINK_WIDTH        (1U)
#define DMA_TCD_CITER_ELINKNO_ELINK(x)           (((uint16_t)(((uint16_t)(x)) << DMA_TCD_CITER_ELINKNO_ELINK_SHIFT)) & DMA_TCD_CITER_ELINKNO_ELINK_MASK)
/*! @} */

/*! @name TCD_CITER_ELINKYES - TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled) */
/*! @{ */
#define DMA_TCD_CITER_ELINKYES_CITER_LE_MASK     (0x1FFU)
#define DMA_TCD_CITER_ELINKYES_CITER_LE_SHIFT    (0U)
#define DMA_TCD_CITER_ELINKYES_CITER_LE_WIDTH    (9U)
#define DMA_TCD_CITER_ELINKYES_CITER_LE(x)       (((uint16_t)(((uint16_t)(x)) << DMA_TCD_CITER_ELINKYES_CITER_LE_SHIFT)) & DMA_TCD_CITER_ELINKYES_CITER_LE_MASK)
#define DMA_TCD_CITER_ELINKYES_LINKCH_MASK       (0x1E00U)
#define DMA_TCD_CITER_ELINKYES_LINKCH_SHIFT      (9U)
#define DMA_TCD_CITER_ELINKYES_LINKCH_WIDTH      (4U)
#define DMA_TCD_CITER_ELINKYES_LINKCH(x)         (((uint16_t)(((uint16_t)(x)) << DMA_TCD_CITER_ELINKYES_LINKCH_SHIFT)) & DMA_TCD_CITER_ELINKYES_LINKCH_MASK)
#define DMA_TCD_CITER_ELINKYES_ELINK_MASK        (0x8000U)
#define DMA_TCD_CITER_ELINKYES_ELINK_SHIFT       (15U)
#define DMA_TCD_CITER_ELINKYES_ELINK_WIDTH       (1U)
#define DMA_TCD_CITER_ELINKYES_ELINK(x)          (((uint16_t)(((uint16_t)(x)) << DMA_TCD_CITER_ELINKYES_ELINK_SHIFT)) & DMA_TCD_CITER_ELINKYES_ELINK_MASK)
/*! @} */

/*! @name TCD_DLASTSGA - TCD Last Destination Address Adjustment/Scatter Gather Address */
/*! @{ */
#define DMA_TCD_DLASTSGA_DLASTSGA_MASK           (0xFFFFFFFFU)
#define DMA_TCD_DLASTSGA_DLASTSGA_SHIFT          (0U)
#define DMA_TCD_DLASTSGA_DLASTSGA_WIDTH          (32U)
#define DMA_TCD_DLASTSGA_DLASTSGA(x)             (((uint32_t)(((uint32_t)(x)) << DMA_TCD_DLASTSGA_DLASTSGA_SHIFT)) & DMA_TCD_DLASTSGA_DLASTSGA_MASK)
/*! @} */

/*! @name TCD_CSR - TCD Control and Status */
/*! @{ */
#define DMA_TCD_CSR_START_MASK                   (0x1U)
#define DMA_TCD_CSR_START_SHIFT                  (0U)
#define DMA_TCD_CSR_START_WIDTH                  (1U)
#define DMA_TCD_CSR_START(x)                     (((uint16_t)(((uint16_t)(x)) << DMA_TCD_CSR_START_SHIFT)) & DMA_TCD_CSR_START_MASK)
#define DMA_TCD_CSR_INTMAJOR_MASK                (0x2U)
#define DMA_TCD_CSR_INTMAJOR_SHIFT               (1U)
#define DMA_TCD_CSR_INTMAJOR_WIDTH               (1U)
#define DMA_TCD_CSR_INTMAJOR(x)                  (((uint16_t)(((uint16_t)(x)) << DMA_TCD_CSR_INTMAJOR_SHIFT)) & DMA_TCD_CSR_INTMAJOR_MASK)
#define DMA_TCD_CSR_INTHALF_MASK                 (0x4U)
#define DMA_TCD_CSR_INTHALF_SHIFT                (2U)
#define DMA_TCD_CSR_INTHALF_WIDTH                (1U)
#define DMA_TCD_CSR_INTHALF(x)                   (((uint16_t)(((uint16_t)(x)) << DMA_TCD_CSR_INTHALF_SHIFT)) & DMA_TCD_CSR_INTHALF_MASK)
#define DMA_TCD_CSR_DREQ_MASK                    (0x8U)
#define DMA_TCD_CSR_DREQ_SHIFT                   (3U)
#define DMA_TCD_CSR_DREQ_WIDTH                   (1U)
#define DMA_TCD_CSR_DREQ(x)                      (((uint16_t)(((uint16_t)(x)) << DMA_TCD_CSR_DREQ_SHIFT)) & DMA_TCD_CSR_DREQ_MASK)
#define DMA_TCD_CSR_ESG_MASK                     (0x10U)
#define DMA_TCD_CSR_ESG_SHIFT                    (4U)
#define DMA_TCD_CSR_ESG_WIDTH                    (1U)
#define DMA_TCD_CSR_ESG(x)                       (((uint16_t)(((uint16_t)(x)) << DMA_TCD_CSR_ESG_SHIFT)) & DMA_TCD_CSR_ESG_MASK)
#define DMA_TCD_CSR_MAJORELINK_MASK              (0x20U)
#define DMA_TCD_CSR_MAJORELINK_SHIFT             (5U)
#define DMA_TCD_CSR_MAJORELINK_WIDTH             (1U)
#define DMA_TCD_CSR_MAJORELINK(x)                (((uint16_t)(((uint16_t)(x)) << DMA_TCD_CSR_MAJORELINK_SHIFT)) & DMA_TCD_CSR_MAJORELINK_MASK)
#define DMA_TCD_CSR_ACTIVE_MASK                  (0x40U)
#define DMA_TCD_CSR_ACTIVE_SHIFT                 (6U)
#define DMA_TCD_CSR_ACTIVE_WIDTH                 (1U)
#define DMA_TCD_CSR_ACTIVE(x)                    (((uint16_t)(((uint16_t)(x)) << DMA_TCD_CSR_ACTIVE_SHIFT)) & DMA_TCD_CSR_ACTIVE_MASK)
#define DMA_TCD_CSR_DONE_MASK                    (0x80U)
#define DMA_TCD_CSR_DONE_SHIFT                   (7U)
#define DMA_TCD_CSR_DONE_WIDTH                   (1U)
#define DMA_TCD_CSR_DONE(x)                      (((uint16_t)(((uint16_t)(x)) << DMA_TCD_CSR_DONE_SHIFT)) & DMA_TCD_CSR_DONE_MASK)
#define DMA_TCD_CSR_MAJORLINKCH_MASK             (0xF00U)
#define DMA_TCD_CSR_MAJORLINKCH_SHIFT            (8U)
#define DMA_TCD_CSR_MAJORLINKCH_WIDTH            (4U)
#define DMA_TCD_CSR_MAJORLINKCH(x)               (((uint16_t)(((uint16_t)(x)) << DMA_TCD_CSR_MAJORLINKCH_SHIFT)) & DMA_TCD_CSR_MAJORLINKCH_MASK)
#define DMA_TCD_CSR_BWC_MASK                     (0xC000U)
#define DMA_TCD_CSR_BWC_SHIFT                    (14U)
#define DMA_TCD_CSR_BWC_WIDTH                    (2U)
#define DMA_TCD_CSR_BWC(x)                       (((uint16_t)(((uint16_t)(x)) << DMA_TCD_CSR_BWC_SHIFT)) & DMA_TCD_CSR_BWC_MASK)
/*! @} */

/*! @name TCD_BITER_ELINKNO - TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled) */
/*! @{ */
#define DMA_TCD_BITER_ELINKNO_BITER_MASK         (0x7FFFU)
#define DMA_TCD_BITER_ELINKNO_BITER_SHIFT        (0U)
#define DMA_TCD_BITER_ELINKNO_BITER_WIDTH        (15U)
#define DMA_TCD_BITER_ELINKNO_BITER(x)           (((uint16_t)(((uint16_t)(x)) << DMA_TCD_BITER_ELINKNO_BITER_SHIFT)) & DMA_TCD_BITER_ELINKNO_BITER_MASK)
#define DMA_TCD_BITER_ELINKNO_ELINK_MASK         (0x8000U)
#define DMA_TCD_BITER_ELINKNO_ELINK_SHIFT        (15U)
#define DMA_TCD_BITER_ELINKNO_ELINK_WIDTH        (1U)
#define DMA_TCD_BITER_ELINKNO_ELINK(x)           (((uint16_t)(((uint16_t)(x)) << DMA_TCD_BITER_ELINKNO_ELINK_SHIFT)) & DMA_TCD_BITER_ELINKNO_ELINK_MASK)
/*! @} */

/*! @name TCD_BITER_ELINKYES - TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled) */
/*! @{ */
#define DMA_TCD_BITER_ELINKYES_BITER_MASK        (0x1FFU)
#define DMA_TCD_BITER_ELINKYES_BITER_SHIFT       (0U)
#define DMA_TCD_BITER_ELINKYES_BITER_WIDTH       (9U)
#define DMA_TCD_BITER_ELINKYES_BITER(x)          (((uint16_t)(((uint16_t)(x)) << DMA_TCD_BITER_ELINKYES_BITER_SHIFT)) & DMA_TCD_BITER_ELINKYES_BITER_MASK)
#define DMA_TCD_BITER_ELINKYES_LINKCH_MASK       (0x1E00U)
#define DMA_TCD_BITER_ELINKYES_LINKCH_SHIFT      (9U)
#define DMA_TCD_BITER_ELINKYES_LINKCH_WIDTH      (4U)
#define DMA_TCD_BITER_ELINKYES_LINKCH(x)         (((uint16_t)(((uint16_t)(x)) << DMA_TCD_BITER_ELINKYES_LINKCH_SHIFT)) & DMA_TCD_BITER_ELINKYES_LINKCH_MASK)
#define DMA_TCD_BITER_ELINKYES_ELINK_MASK        (0x8000U)
#define DMA_TCD_BITER_ELINKYES_ELINK_SHIFT       (15U)
#define DMA_TCD_BITER_ELINKYES_ELINK_WIDTH       (1U)
#define DMA_TCD_BITER_ELINKYES_ELINK(x)          (((uint16_t)(((uint16_t)(x)) << DMA_TCD_BITER_ELINKYES_ELINK_SHIFT)) & DMA_TCD_BITER_ELINKYES_ELINK_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group DMA_Register_Masks */

/*!
 * @}
 */ /* end of group DMA_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- DMAMUX Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMAMUX_Peripheral_Access_Layer DMAMUX Peripheral Access Layer
 * @{
 */

/** DMAMUX - Size of Registers Arrays */
#define DMAMUX_CHCFG_COUNT                        16u

/** DMAMUX - Register Layout Typedef */
typedef struct {
  __IO uint8_t CHCFG[DMAMUX_CHCFG_COUNT];          /**< Channel Configuration register, array offset: 0x0, array step: 0x1 */
} DMAMUX_Type, *DMAMUX_MemMapPtr;

/** Number of instances of the DMAMUX module. */
#define DMAMUX_INSTANCE_COUNT                    (1u)

/* DMAMUX - Peripheral instance base addresses */
/** Peripheral DMAMUX base address */
#define DMAMUX_BASE                              (0x40021000u)
/** Peripheral DMAMUX base pointer */
#define DMAMUX                                   ((DMAMUX_Type *)DMAMUX_BASE)
/** Array initializer of DMAMUX peripheral base addresses */
#define DMAMUX_BASE_ADDRS                        { DMAMUX_BASE }
/** Array initializer of DMAMUX peripheral base pointers */
#define DMAMUX_BASE_PTRS                         { DMAMUX }

/* ----------------------------------------------------------------------------
   -- DMAMUX Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMAMUX_Register_Masks DMAMUX Register Masks
 * @{
 */

/*! @name CHCFG - Channel Configuration register */
/*! @{ */
#define DMAMUX_CHCFG_SOURCE_MASK                 (0x3FU)
#define DMAMUX_CHCFG_SOURCE_SHIFT                (0U)
#define DMAMUX_CHCFG_SOURCE_WIDTH                (6U)
#define DMAMUX_CHCFG_SOURCE(x)                   (((uint8_t)(((uint8_t)(x)) << DMAMUX_CHCFG_SOURCE_SHIFT)) & DMAMUX_CHCFG_SOURCE_MASK)
#define DMAMUX_CHCFG_TRIG_MASK                   (0x40U)
#define DMAMUX_CHCFG_TRIG_SHIFT                  (6U)
#define DMAMUX_CHCFG_TRIG_WIDTH                  (1U)
#define DMAMUX_CHCFG_TRIG(x)                     (((uint8_t)(((uint8_t)(x)) << DMAMUX_CHCFG_TRIG_SHIFT)) & DMAMUX_CHCFG_TRIG_MASK)
#define DMAMUX_CHCFG_ENBL_MASK                   (0x80U)
#define DMAMUX_CHCFG_ENBL_SHIFT                  (7U)
#define DMAMUX_CHCFG_ENBL_WIDTH                  (1U)
#define DMAMUX_CHCFG_ENBL(x)                     (((uint8_t)(((uint8_t)(x)) << DMAMUX_CHCFG_ENBL_SHIFT)) & DMAMUX_CHCFG_ENBL_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group DMAMUX_Register_Masks */

/*!
 * @}
 */ /* end of group DMAMUX_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- EIM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EIM_Peripheral_Access_Layer EIM Peripheral Access Layer
 * @{
 */

/** EIM - Size of Registers Arrays */
#define EIM_EICHDn_COUNT                          2u

/** EIM - Register Layout Typedef */
typedef struct {
  __IO uint32_t EIMCR;                             /**< Error Injection Module Configuration Register, offset: 0x0 */
  __IO uint32_t EICHEN;                            /**< Error Injection Channel Enable register, offset: 0x4 */
  uint8_t RESERVED_0[248];
  struct {                                         /* offset: 0x100, array step: 0x100 */
    __IO uint32_t WORD0;                             /**< Error Injection Channel Descriptor n, Word0, array offset: 0x100, array step: 0x100 */
    __IO uint32_t WORD1;                             /**< Error Injection Channel Descriptor n, Word1, array offset: 0x104, array step: 0x100 */
    uint8_t RESERVED_0[248];
  } EICHDn[EIM_EICHDn_COUNT];
} EIM_Type, *EIM_MemMapPtr;

/** Number of instances of the EIM module. */
#define EIM_INSTANCE_COUNT                       (1u)

/* EIM - Peripheral instance base addresses */
/** Peripheral EIM base address */
#define EIM_BASE                                 (0x40019000u)
/** Peripheral EIM base pointer */
#define EIM                                      ((EIM_Type *)EIM_BASE)
/** Array initializer of EIM peripheral base addresses */
#define EIM_BASE_ADDRS                           { EIM_BASE }
/** Array initializer of EIM peripheral base pointers */
#define EIM_BASE_PTRS                            { EIM }

/* ----------------------------------------------------------------------------
   -- EIM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EIM_Register_Masks EIM Register Masks
 * @{
 */

/*! @name EIMCR - Error Injection Module Configuration Register */
/*! @{ */
#define EIM_EIMCR_GEIEN_MASK                     (0x1U)
#define EIM_EIMCR_GEIEN_SHIFT                    (0U)
#define EIM_EIMCR_GEIEN_WIDTH                    (1U)
#define EIM_EIMCR_GEIEN(x)                       (((uint32_t)(((uint32_t)(x)) << EIM_EIMCR_GEIEN_SHIFT)) & EIM_EIMCR_GEIEN_MASK)
/*! @} */

/*! @name EICHEN - Error Injection Channel Enable register */
/*! @{ */
#define EIM_EICHEN_EICH1EN_MASK                  (0x40000000U)
#define EIM_EICHEN_EICH1EN_SHIFT                 (30U)
#define EIM_EICHEN_EICH1EN_WIDTH                 (1U)
#define EIM_EICHEN_EICH1EN(x)                    (((uint32_t)(((uint32_t)(x)) << EIM_EICHEN_EICH1EN_SHIFT)) & EIM_EICHEN_EICH1EN_MASK)
#define EIM_EICHEN_EICH0EN_MASK                  (0x80000000U)
#define EIM_EICHEN_EICH0EN_SHIFT                 (31U)
#define EIM_EICHEN_EICH0EN_WIDTH                 (1U)
#define EIM_EICHEN_EICH0EN(x)                    (((uint32_t)(((uint32_t)(x)) << EIM_EICHEN_EICH0EN_SHIFT)) & EIM_EICHEN_EICH0EN_MASK)
/*! @} */

/*! @name EICHDn_WORD0 - Error Injection Channel Descriptor n, Word0 */
/*! @{ */
#define EIM_EICHDn_WORD0_CHKBIT_MASK_MASK        (0xFE000000U)
#define EIM_EICHDn_WORD0_CHKBIT_MASK_SHIFT       (25U)
#define EIM_EICHDn_WORD0_CHKBIT_MASK_WIDTH       (7U)
#define EIM_EICHDn_WORD0_CHKBIT_MASK(x)          (((uint32_t)(((uint32_t)(x)) << EIM_EICHDn_WORD0_CHKBIT_MASK_SHIFT)) & EIM_EICHDn_WORD0_CHKBIT_MASK_MASK)
/*! @} */

/*! @name EICHDn_WORD1 - Error Injection Channel Descriptor n, Word1 */
/*! @{ */
#define EIM_EICHDn_WORD1_B0_3DATA_MASK_MASK      (0xFFFFFFFFU)
#define EIM_EICHDn_WORD1_B0_3DATA_MASK_SHIFT     (0U)
#define EIM_EICHDn_WORD1_B0_3DATA_MASK_WIDTH     (32U)
#define EIM_EICHDn_WORD1_B0_3DATA_MASK(x)        (((uint32_t)(((uint32_t)(x)) << EIM_EICHDn_WORD1_B0_3DATA_MASK_SHIFT)) & EIM_EICHDn_WORD1_B0_3DATA_MASK_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group EIM_Register_Masks */

/*!
 * @}
 */ /* end of group EIM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- ERM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ERM_Peripheral_Access_Layer ERM Peripheral Access Layer
 * @{
 */

/** ERM - Size of Registers Arrays */
#define ERM_EARn_COUNT                            2u

/** ERM - Register Layout Typedef */
typedef struct {
  __IO uint32_t CR0;                               /**< ERM Configuration Register 0, offset: 0x0 */
  uint8_t RESERVED_0[12];
  __IO uint32_t SR0;                               /**< ERM Status Register 0, offset: 0x10 */
  uint8_t RESERVED_1[236];
  struct {                                         /* offset: 0x100, array step: 0x10 */
    __I  uint32_t EAR;                               /**< ERM Memory n Error Address Register, array offset: 0x100, array step: 0x10 */
    uint8_t RESERVED_0[12];
  } EARn[ERM_EARn_COUNT];
} ERM_Type, *ERM_MemMapPtr;

/** Number of instances of the ERM module. */
#define ERM_INSTANCE_COUNT                       (1u)

/* ERM - Peripheral instance base addresses */
/** Peripheral ERM base address */
#define ERM_BASE                                 (0x40018000u)
/** Peripheral ERM base pointer */
#define ERM                                      ((ERM_Type *)ERM_BASE)
/** Array initializer of ERM peripheral base addresses */
#define ERM_BASE_ADDRS                           { ERM_BASE }
/** Array initializer of ERM peripheral base pointers */
#define ERM_BASE_PTRS                            { ERM }

/* ----------------------------------------------------------------------------
   -- ERM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ERM_Register_Masks ERM Register Masks
 * @{
 */

/*! @name CR0 - ERM Configuration Register 0 */
/*! @{ */
#define ERM_CR0_ENCIE1_MASK                      (0x4000000U)
#define ERM_CR0_ENCIE1_SHIFT                     (26U)
#define ERM_CR0_ENCIE1_WIDTH                     (1U)
#define ERM_CR0_ENCIE1(x)                        (((uint32_t)(((uint32_t)(x)) << ERM_CR0_ENCIE1_SHIFT)) & ERM_CR0_ENCIE1_MASK)
#define ERM_CR0_ESCIE1_MASK                      (0x8000000U)
#define ERM_CR0_ESCIE1_SHIFT                     (27U)
#define ERM_CR0_ESCIE1_WIDTH                     (1U)
#define ERM_CR0_ESCIE1(x)                        (((uint32_t)(((uint32_t)(x)) << ERM_CR0_ESCIE1_SHIFT)) & ERM_CR0_ESCIE1_MASK)
#define ERM_CR0_ENCIE0_MASK                      (0x40000000U)
#define ERM_CR0_ENCIE0_SHIFT                     (30U)
#define ERM_CR0_ENCIE0_WIDTH                     (1U)
#define ERM_CR0_ENCIE0(x)                        (((uint32_t)(((uint32_t)(x)) << ERM_CR0_ENCIE0_SHIFT)) & ERM_CR0_ENCIE0_MASK)
#define ERM_CR0_ESCIE0_MASK                      (0x80000000U)
#define ERM_CR0_ESCIE0_SHIFT                     (31U)
#define ERM_CR0_ESCIE0_WIDTH                     (1U)
#define ERM_CR0_ESCIE0(x)                        (((uint32_t)(((uint32_t)(x)) << ERM_CR0_ESCIE0_SHIFT)) & ERM_CR0_ESCIE0_MASK)
/*! @} */

/*! @name SR0 - ERM Status Register 0 */
/*! @{ */
#define ERM_SR0_NCE1_MASK                        (0x4000000U)
#define ERM_SR0_NCE1_SHIFT                       (26U)
#define ERM_SR0_NCE1_WIDTH                       (1U)
#define ERM_SR0_NCE1(x)                          (((uint32_t)(((uint32_t)(x)) << ERM_SR0_NCE1_SHIFT)) & ERM_SR0_NCE1_MASK)
#define ERM_SR0_SBC1_MASK                        (0x8000000U)
#define ERM_SR0_SBC1_SHIFT                       (27U)
#define ERM_SR0_SBC1_WIDTH                       (1U)
#define ERM_SR0_SBC1(x)                          (((uint32_t)(((uint32_t)(x)) << ERM_SR0_SBC1_SHIFT)) & ERM_SR0_SBC1_MASK)
#define ERM_SR0_NCE0_MASK                        (0x40000000U)
#define ERM_SR0_NCE0_SHIFT                       (30U)
#define ERM_SR0_NCE0_WIDTH                       (1U)
#define ERM_SR0_NCE0(x)                          (((uint32_t)(((uint32_t)(x)) << ERM_SR0_NCE0_SHIFT)) & ERM_SR0_NCE0_MASK)
#define ERM_SR0_SBC0_MASK                        (0x80000000U)
#define ERM_SR0_SBC0_SHIFT                       (31U)
#define ERM_SR0_SBC0_WIDTH                       (1U)
#define ERM_SR0_SBC0(x)                          (((uint32_t)(((uint32_t)(x)) << ERM_SR0_SBC0_SHIFT)) & ERM_SR0_SBC0_MASK)
/*! @} */

/*! @name EAR - ERM Memory n Error Address Register */
/*! @{ */
#define ERM_EAR_EAR_MASK                         (0xFFFFFFFFU)
#define ERM_EAR_EAR_SHIFT                        (0U)
#define ERM_EAR_EAR_WIDTH                        (32U)
#define ERM_EAR_EAR(x)                           (((uint32_t)(((uint32_t)(x)) << ERM_EAR_EAR_SHIFT)) & ERM_EAR_EAR_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group ERM_Register_Masks */

/*!
 * @}
 */ /* end of group ERM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- EWM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EWM_Peripheral_Access_Layer EWM Peripheral Access Layer
 * @{
 */

/** EWM - Register Layout Typedef */
typedef struct {
  __IO uint8_t CTRL;                               /**< Control Register, offset: 0x0 */
  __O  uint8_t SERV;                               /**< Service Register, offset: 0x1 */
  __IO uint8_t CMPL;                               /**< Compare Low Register, offset: 0x2 */
  __IO uint8_t CMPH;                               /**< Compare High Register, offset: 0x3 */
  uint8_t RESERVED_0[1];
  __IO uint8_t CLKPRESCALER;                       /**< Clock Prescaler Register, offset: 0x5 */
} EWM_Type, *EWM_MemMapPtr;

/** Number of instances of the EWM module. */
#define EWM_INSTANCE_COUNT                       (1u)

/* EWM - Peripheral instance base addresses */
/** Peripheral EWM base address */
#define EWM_BASE                                 (0x40061000u)
/** Peripheral EWM base pointer */
#define EWM                                      ((EWM_Type *)EWM_BASE)
/** Array initializer of EWM peripheral base addresses */
#define EWM_BASE_ADDRS                           { EWM_BASE }
/** Array initializer of EWM peripheral base pointers */
#define EWM_BASE_PTRS                            { EWM }

/* ----------------------------------------------------------------------------
   -- EWM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EWM_Register_Masks EWM Register Masks
 * @{
 */

/*! @name CTRL - Control Register */
/*! @{ */
#define EWM_CTRL_EWMEN_MASK                      (0x1U)
#define EWM_CTRL_EWMEN_SHIFT                     (0U)
#define EWM_CTRL_EWMEN_WIDTH                     (1U)
#define EWM_CTRL_EWMEN(x)                        (((uint8_t)(((uint8_t)(x)) << EWM_CTRL_EWMEN_SHIFT)) & EWM_CTRL_EWMEN_MASK)
#define EWM_CTRL_ASSIN_MASK                      (0x2U)
#define EWM_CTRL_ASSIN_SHIFT                     (1U)
#define EWM_CTRL_ASSIN_WIDTH                     (1U)
#define EWM_CTRL_ASSIN(x)                        (((uint8_t)(((uint8_t)(x)) << EWM_CTRL_ASSIN_SHIFT)) & EWM_CTRL_ASSIN_MASK)
#define EWM_CTRL_INEN_MASK                       (0x4U)
#define EWM_CTRL_INEN_SHIFT                      (2U)
#define EWM_CTRL_INEN_WIDTH                      (1U)
#define EWM_CTRL_INEN(x)                         (((uint8_t)(((uint8_t)(x)) << EWM_CTRL_INEN_SHIFT)) & EWM_CTRL_INEN_MASK)
#define EWM_CTRL_INTEN_MASK                      (0x8U)
#define EWM_CTRL_INTEN_SHIFT                     (3U)
#define EWM_CTRL_INTEN_WIDTH                     (1U)
#define EWM_CTRL_INTEN(x)                        (((uint8_t)(((uint8_t)(x)) << EWM_CTRL_INTEN_SHIFT)) & EWM_CTRL_INTEN_MASK)
/*! @} */

/*! @name SERV - Service Register */
/*! @{ */
#define EWM_SERV_SERVICE_MASK                    (0xFFU)
#define EWM_SERV_SERVICE_SHIFT                   (0U)
#define EWM_SERV_SERVICE_WIDTH                   (8U)
#define EWM_SERV_SERVICE(x)                      (((uint8_t)(((uint8_t)(x)) << EWM_SERV_SERVICE_SHIFT)) & EWM_SERV_SERVICE_MASK)
/*! @} */

/*! @name CMPL - Compare Low Register */
/*! @{ */
#define EWM_CMPL_COMPAREL_MASK                   (0xFFU)
#define EWM_CMPL_COMPAREL_SHIFT                  (0U)
#define EWM_CMPL_COMPAREL_WIDTH                  (8U)
#define EWM_CMPL_COMPAREL(x)                     (((uint8_t)(((uint8_t)(x)) << EWM_CMPL_COMPAREL_SHIFT)) & EWM_CMPL_COMPAREL_MASK)
/*! @} */

/*! @name CMPH - Compare High Register */
/*! @{ */
#define EWM_CMPH_COMPAREH_MASK                   (0xFFU)
#define EWM_CMPH_COMPAREH_SHIFT                  (0U)
#define EWM_CMPH_COMPAREH_WIDTH                  (8U)
#define EWM_CMPH_COMPAREH(x)                     (((uint8_t)(((uint8_t)(x)) << EWM_CMPH_COMPAREH_SHIFT)) & EWM_CMPH_COMPAREH_MASK)
/*! @} */

/*! @name CLKPRESCALER - Clock Prescaler Register */
/*! @{ */
#define EWM_CLKPRESCALER_CLK_DIV_MASK            (0xFFU)
#define EWM_CLKPRESCALER_CLK_DIV_SHIFT           (0U)
#define EWM_CLKPRESCALER_CLK_DIV_WIDTH           (8U)
#define EWM_CLKPRESCALER_CLK_DIV(x)              (((uint8_t)(((uint8_t)(x)) << EWM_CLKPRESCALER_CLK_DIV_SHIFT)) & EWM_CLKPRESCALER_CLK_DIV_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group EWM_Register_Masks */

/*!
 * @}
 */ /* end of group EWM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- FLEXIO Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FLEXIO_Peripheral_Access_Layer FLEXIO Peripheral Access Layer
 * @{
 */

/** FLEXIO - Size of Registers Arrays */
#define FLEXIO_SHIFTCTL_COUNT                     4u
#define FLEXIO_SHIFTCFG_COUNT                     4u
#define FLEXIO_SHIFTBUF_COUNT                     4u
#define FLEXIO_SHIFTBUFBIS_COUNT                  4u
#define FLEXIO_SHIFTBUFBYS_COUNT                  4u
#define FLEXIO_SHIFTBUFBBS_COUNT                  4u
#define FLEXIO_TIMCTL_COUNT                       4u
#define FLEXIO_TIMCFG_COUNT                       4u
#define FLEXIO_TIMCMP_COUNT                       4u

/** FLEXIO - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID Register, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< Parameter Register, offset: 0x4 */
  __IO uint32_t CTRL;                              /**< FlexIO Control Register, offset: 0x8 */
  __I  uint32_t PIN;                               /**< Pin State Register, offset: 0xC */
  __IO uint32_t SHIFTSTAT;                         /**< Shifter Status Register, offset: 0x10 */
  __IO uint32_t SHIFTERR;                          /**< Shifter Error Register, offset: 0x14 */
  __IO uint32_t TIMSTAT;                           /**< Timer Status Register, offset: 0x18 */
  uint8_t RESERVED_0[4];
  __IO uint32_t SHIFTSIEN;                         /**< Shifter Status Interrupt Enable, offset: 0x20 */
  __IO uint32_t SHIFTEIEN;                         /**< Shifter Error Interrupt Enable, offset: 0x24 */
  __IO uint32_t TIMIEN;                            /**< Timer Interrupt Enable Register, offset: 0x28 */
  uint8_t RESERVED_1[4];
  __IO uint32_t SHIFTSDEN;                         /**< Shifter Status DMA Enable, offset: 0x30 */
  uint8_t RESERVED_2[76];
  __IO uint32_t SHIFTCTL[FLEXIO_SHIFTCTL_COUNT];   /**< Shifter Control N Register, array offset: 0x80, array step: 0x4 */
  uint8_t RESERVED_3[112];
  __IO uint32_t SHIFTCFG[FLEXIO_SHIFTCFG_COUNT];   /**< Shifter Configuration N Register, array offset: 0x100, array step: 0x4 */
  uint8_t RESERVED_4[240];
  __IO uint32_t SHIFTBUF[FLEXIO_SHIFTBUF_COUNT];   /**< Shifter Buffer N Register, array offset: 0x200, array step: 0x4 */
  uint8_t RESERVED_5[112];
  __IO uint32_t SHIFTBUFBIS[FLEXIO_SHIFTBUFBIS_COUNT]; /**< Shifter Buffer N Bit Swapped Register, array offset: 0x280, array step: 0x4 */
  uint8_t RESERVED_6[112];
  __IO uint32_t SHIFTBUFBYS[FLEXIO_SHIFTBUFBYS_COUNT]; /**< Shifter Buffer N Byte Swapped Register, array offset: 0x300, array step: 0x4 */
  uint8_t RESERVED_7[112];
  __IO uint32_t SHIFTBUFBBS[FLEXIO_SHIFTBUFBBS_COUNT]; /**< Shifter Buffer N Bit Byte Swapped Register, array offset: 0x380, array step: 0x4 */
  uint8_t RESERVED_8[112];
  __IO uint32_t TIMCTL[FLEXIO_TIMCTL_COUNT];       /**< Timer Control N Register, array offset: 0x400, array step: 0x4 */
  uint8_t RESERVED_9[112];
  __IO uint32_t TIMCFG[FLEXIO_TIMCFG_COUNT];       /**< Timer Configuration N Register, array offset: 0x480, array step: 0x4 */
  uint8_t RESERVED_10[112];
  __IO uint32_t TIMCMP[FLEXIO_TIMCMP_COUNT];       /**< Timer Compare N Register, array offset: 0x500, array step: 0x4 */
} FLEXIO_Type, *FLEXIO_MemMapPtr;

/** Number of instances of the FLEXIO module. */
#define FLEXIO_INSTANCE_COUNT                    (1u)

/* FLEXIO - Peripheral instance base addresses */
/** Peripheral FLEXIO base address */
#define FLEXIO_BASE                              (0x4005A000u)
/** Peripheral FLEXIO base pointer */
#define FLEXIO                                   ((FLEXIO_Type *)FLEXIO_BASE)
/** Array initializer of FLEXIO peripheral base addresses */
#define FLEXIO_BASE_ADDRS                        { FLEXIO_BASE }
/** Array initializer of FLEXIO peripheral base pointers */
#define FLEXIO_BASE_PTRS                         { FLEXIO }

/* ----------------------------------------------------------------------------
   -- FLEXIO Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FLEXIO_Register_Masks FLEXIO Register Masks
 * @{
 */

/*! @name VERID - Version ID Register */
/*! @{ */
#define FLEXIO_VERID_FEATURE_MASK                (0xFFFFU)
#define FLEXIO_VERID_FEATURE_SHIFT               (0U)
#define FLEXIO_VERID_FEATURE_WIDTH               (16U)
#define FLEXIO_VERID_FEATURE(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_VERID_FEATURE_SHIFT)) & FLEXIO_VERID_FEATURE_MASK)
#define FLEXIO_VERID_MINOR_MASK                  (0xFF0000U)
#define FLEXIO_VERID_MINOR_SHIFT                 (16U)
#define FLEXIO_VERID_MINOR_WIDTH                 (8U)
#define FLEXIO_VERID_MINOR(x)                    (((uint32_t)(((uint32_t)(x)) << FLEXIO_VERID_MINOR_SHIFT)) & FLEXIO_VERID_MINOR_MASK)
#define FLEXIO_VERID_MAJOR_MASK                  (0xFF000000U)
#define FLEXIO_VERID_MAJOR_SHIFT                 (24U)
#define FLEXIO_VERID_MAJOR_WIDTH                 (8U)
#define FLEXIO_VERID_MAJOR(x)                    (((uint32_t)(((uint32_t)(x)) << FLEXIO_VERID_MAJOR_SHIFT)) & FLEXIO_VERID_MAJOR_MASK)
/*! @} */

/*! @name PARAM - Parameter Register */
/*! @{ */
#define FLEXIO_PARAM_SHIFTER_MASK                (0xFFU)
#define FLEXIO_PARAM_SHIFTER_SHIFT               (0U)
#define FLEXIO_PARAM_SHIFTER_WIDTH               (8U)
#define FLEXIO_PARAM_SHIFTER(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_PARAM_SHIFTER_SHIFT)) & FLEXIO_PARAM_SHIFTER_MASK)
#define FLEXIO_PARAM_TIMER_MASK                  (0xFF00U)
#define FLEXIO_PARAM_TIMER_SHIFT                 (8U)
#define FLEXIO_PARAM_TIMER_WIDTH                 (8U)
#define FLEXIO_PARAM_TIMER(x)                    (((uint32_t)(((uint32_t)(x)) << FLEXIO_PARAM_TIMER_SHIFT)) & FLEXIO_PARAM_TIMER_MASK)
#define FLEXIO_PARAM_PIN_MASK                    (0xFF0000U)
#define FLEXIO_PARAM_PIN_SHIFT                   (16U)
#define FLEXIO_PARAM_PIN_WIDTH                   (8U)
#define FLEXIO_PARAM_PIN(x)                      (((uint32_t)(((uint32_t)(x)) << FLEXIO_PARAM_PIN_SHIFT)) & FLEXIO_PARAM_PIN_MASK)
#define FLEXIO_PARAM_TRIGGER_MASK                (0xFF000000U)
#define FLEXIO_PARAM_TRIGGER_SHIFT               (24U)
#define FLEXIO_PARAM_TRIGGER_WIDTH               (8U)
#define FLEXIO_PARAM_TRIGGER(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_PARAM_TRIGGER_SHIFT)) & FLEXIO_PARAM_TRIGGER_MASK)
/*! @} */

/*! @name CTRL - FlexIO Control Register */
/*! @{ */
#define FLEXIO_CTRL_FLEXEN_MASK                  (0x1U)
#define FLEXIO_CTRL_FLEXEN_SHIFT                 (0U)
#define FLEXIO_CTRL_FLEXEN_WIDTH                 (1U)
#define FLEXIO_CTRL_FLEXEN(x)                    (((uint32_t)(((uint32_t)(x)) << FLEXIO_CTRL_FLEXEN_SHIFT)) & FLEXIO_CTRL_FLEXEN_MASK)
#define FLEXIO_CTRL_SWRST_MASK                   (0x2U)
#define FLEXIO_CTRL_SWRST_SHIFT                  (1U)
#define FLEXIO_CTRL_SWRST_WIDTH                  (1U)
#define FLEXIO_CTRL_SWRST(x)                     (((uint32_t)(((uint32_t)(x)) << FLEXIO_CTRL_SWRST_SHIFT)) & FLEXIO_CTRL_SWRST_MASK)
#define FLEXIO_CTRL_FASTACC_MASK                 (0x4U)
#define FLEXIO_CTRL_FASTACC_SHIFT                (2U)
#define FLEXIO_CTRL_FASTACC_WIDTH                (1U)
#define FLEXIO_CTRL_FASTACC(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXIO_CTRL_FASTACC_SHIFT)) & FLEXIO_CTRL_FASTACC_MASK)
#define FLEXIO_CTRL_DBGE_MASK                    (0x40000000U)
#define FLEXIO_CTRL_DBGE_SHIFT                   (30U)
#define FLEXIO_CTRL_DBGE_WIDTH                   (1U)
#define FLEXIO_CTRL_DBGE(x)                      (((uint32_t)(((uint32_t)(x)) << FLEXIO_CTRL_DBGE_SHIFT)) & FLEXIO_CTRL_DBGE_MASK)
#define FLEXIO_CTRL_DOZEN_MASK                   (0x80000000U)
#define FLEXIO_CTRL_DOZEN_SHIFT                  (31U)
#define FLEXIO_CTRL_DOZEN_WIDTH                  (1U)
#define FLEXIO_CTRL_DOZEN(x)                     (((uint32_t)(((uint32_t)(x)) << FLEXIO_CTRL_DOZEN_SHIFT)) & FLEXIO_CTRL_DOZEN_MASK)
/*! @} */

/*! @name PIN - Pin State Register */
/*! @{ */
#define FLEXIO_PIN_PDI_MASK                      (0xFFU)
#define FLEXIO_PIN_PDI_SHIFT                     (0U)
#define FLEXIO_PIN_PDI_WIDTH                     (8U)
#define FLEXIO_PIN_PDI(x)                        (((uint32_t)(((uint32_t)(x)) << FLEXIO_PIN_PDI_SHIFT)) & FLEXIO_PIN_PDI_MASK)
/*! @} */

/*! @name SHIFTSTAT - Shifter Status Register */
/*! @{ */
#define FLEXIO_SHIFTSTAT_SSF_MASK                (0xFU)
#define FLEXIO_SHIFTSTAT_SSF_SHIFT               (0U)
#define FLEXIO_SHIFTSTAT_SSF_WIDTH               (4U)
#define FLEXIO_SHIFTSTAT_SSF(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTSTAT_SSF_SHIFT)) & FLEXIO_SHIFTSTAT_SSF_MASK)
/*! @} */

/*! @name SHIFTERR - Shifter Error Register */
/*! @{ */
#define FLEXIO_SHIFTERR_SEF_MASK                 (0xFU)
#define FLEXIO_SHIFTERR_SEF_SHIFT                (0U)
#define FLEXIO_SHIFTERR_SEF_WIDTH                (4U)
#define FLEXIO_SHIFTERR_SEF(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTERR_SEF_SHIFT)) & FLEXIO_SHIFTERR_SEF_MASK)
/*! @} */

/*! @name TIMSTAT - Timer Status Register */
/*! @{ */
#define FLEXIO_TIMSTAT_TSF_MASK                  (0xFU)
#define FLEXIO_TIMSTAT_TSF_SHIFT                 (0U)
#define FLEXIO_TIMSTAT_TSF_WIDTH                 (4U)
#define FLEXIO_TIMSTAT_TSF(x)                    (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMSTAT_TSF_SHIFT)) & FLEXIO_TIMSTAT_TSF_MASK)
/*! @} */

/*! @name SHIFTSIEN - Shifter Status Interrupt Enable */
/*! @{ */
#define FLEXIO_SHIFTSIEN_SSIE_MASK               (0xFU)
#define FLEXIO_SHIFTSIEN_SSIE_SHIFT              (0U)
#define FLEXIO_SHIFTSIEN_SSIE_WIDTH              (4U)
#define FLEXIO_SHIFTSIEN_SSIE(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTSIEN_SSIE_SHIFT)) & FLEXIO_SHIFTSIEN_SSIE_MASK)
/*! @} */

/*! @name SHIFTEIEN - Shifter Error Interrupt Enable */
/*! @{ */
#define FLEXIO_SHIFTEIEN_SEIE_MASK               (0xFU)
#define FLEXIO_SHIFTEIEN_SEIE_SHIFT              (0U)
#define FLEXIO_SHIFTEIEN_SEIE_WIDTH              (4U)
#define FLEXIO_SHIFTEIEN_SEIE(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTEIEN_SEIE_SHIFT)) & FLEXIO_SHIFTEIEN_SEIE_MASK)
/*! @} */

/*! @name TIMIEN - Timer Interrupt Enable Register */
/*! @{ */
#define FLEXIO_TIMIEN_TEIE_MASK                  (0xFU)
#define FLEXIO_TIMIEN_TEIE_SHIFT                 (0U)
#define FLEXIO_TIMIEN_TEIE_WIDTH                 (4U)
#define FLEXIO_TIMIEN_TEIE(x)                    (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMIEN_TEIE_SHIFT)) & FLEXIO_TIMIEN_TEIE_MASK)
/*! @} */

/*! @name SHIFTSDEN - Shifter Status DMA Enable */
/*! @{ */
#define FLEXIO_SHIFTSDEN_SSDE_MASK               (0xFU)
#define FLEXIO_SHIFTSDEN_SSDE_SHIFT              (0U)
#define FLEXIO_SHIFTSDEN_SSDE_WIDTH              (4U)
#define FLEXIO_SHIFTSDEN_SSDE(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTSDEN_SSDE_SHIFT)) & FLEXIO_SHIFTSDEN_SSDE_MASK)
/*! @} */

/*! @name SHIFTCTL - Shifter Control N Register */
/*! @{ */
#define FLEXIO_SHIFTCTL_SMOD_MASK                (0x7U)
#define FLEXIO_SHIFTCTL_SMOD_SHIFT               (0U)
#define FLEXIO_SHIFTCTL_SMOD_WIDTH               (3U)
#define FLEXIO_SHIFTCTL_SMOD(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTCTL_SMOD_SHIFT)) & FLEXIO_SHIFTCTL_SMOD_MASK)
#define FLEXIO_SHIFTCTL_PINPOL_MASK              (0x80U)
#define FLEXIO_SHIFTCTL_PINPOL_SHIFT             (7U)
#define FLEXIO_SHIFTCTL_PINPOL_WIDTH             (1U)
#define FLEXIO_SHIFTCTL_PINPOL(x)                (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTCTL_PINPOL_SHIFT)) & FLEXIO_SHIFTCTL_PINPOL_MASK)
#define FLEXIO_SHIFTCTL_PINSEL_MASK              (0x700U)
#define FLEXIO_SHIFTCTL_PINSEL_SHIFT             (8U)
#define FLEXIO_SHIFTCTL_PINSEL_WIDTH             (3U)
#define FLEXIO_SHIFTCTL_PINSEL(x)                (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTCTL_PINSEL_SHIFT)) & FLEXIO_SHIFTCTL_PINSEL_MASK)
#define FLEXIO_SHIFTCTL_PINCFG_MASK              (0x30000U)
#define FLEXIO_SHIFTCTL_PINCFG_SHIFT             (16U)
#define FLEXIO_SHIFTCTL_PINCFG_WIDTH             (2U)
#define FLEXIO_SHIFTCTL_PINCFG(x)                (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTCTL_PINCFG_SHIFT)) & FLEXIO_SHIFTCTL_PINCFG_MASK)
#define FLEXIO_SHIFTCTL_TIMPOL_MASK              (0x800000U)
#define FLEXIO_SHIFTCTL_TIMPOL_SHIFT             (23U)
#define FLEXIO_SHIFTCTL_TIMPOL_WIDTH             (1U)
#define FLEXIO_SHIFTCTL_TIMPOL(x)                (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTCTL_TIMPOL_SHIFT)) & FLEXIO_SHIFTCTL_TIMPOL_MASK)
#define FLEXIO_SHIFTCTL_TIMSEL_MASK              (0x3000000U)
#define FLEXIO_SHIFTCTL_TIMSEL_SHIFT             (24U)
#define FLEXIO_SHIFTCTL_TIMSEL_WIDTH             (2U)
#define FLEXIO_SHIFTCTL_TIMSEL(x)                (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTCTL_TIMSEL_SHIFT)) & FLEXIO_SHIFTCTL_TIMSEL_MASK)
/*! @} */

/*! @name SHIFTCFG - Shifter Configuration N Register */
/*! @{ */
#define FLEXIO_SHIFTCFG_SSTART_MASK              (0x3U)
#define FLEXIO_SHIFTCFG_SSTART_SHIFT             (0U)
#define FLEXIO_SHIFTCFG_SSTART_WIDTH             (2U)
#define FLEXIO_SHIFTCFG_SSTART(x)                (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTCFG_SSTART_SHIFT)) & FLEXIO_SHIFTCFG_SSTART_MASK)
#define FLEXIO_SHIFTCFG_SSTOP_MASK               (0x30U)
#define FLEXIO_SHIFTCFG_SSTOP_SHIFT              (4U)
#define FLEXIO_SHIFTCFG_SSTOP_WIDTH              (2U)
#define FLEXIO_SHIFTCFG_SSTOP(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTCFG_SSTOP_SHIFT)) & FLEXIO_SHIFTCFG_SSTOP_MASK)
#define FLEXIO_SHIFTCFG_INSRC_MASK               (0x100U)
#define FLEXIO_SHIFTCFG_INSRC_SHIFT              (8U)
#define FLEXIO_SHIFTCFG_INSRC_WIDTH              (1U)
#define FLEXIO_SHIFTCFG_INSRC(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTCFG_INSRC_SHIFT)) & FLEXIO_SHIFTCFG_INSRC_MASK)
/*! @} */

/*! @name SHIFTBUF - Shifter Buffer N Register */
/*! @{ */
#define FLEXIO_SHIFTBUF_SHIFTBUF_MASK            (0xFFFFFFFFU)
#define FLEXIO_SHIFTBUF_SHIFTBUF_SHIFT           (0U)
#define FLEXIO_SHIFTBUF_SHIFTBUF_WIDTH           (32U)
#define FLEXIO_SHIFTBUF_SHIFTBUF(x)              (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTBUF_SHIFTBUF_SHIFT)) & FLEXIO_SHIFTBUF_SHIFTBUF_MASK)
/*! @} */

/*! @name SHIFTBUFBIS - Shifter Buffer N Bit Swapped Register */
/*! @{ */
#define FLEXIO_SHIFTBUFBIS_SHIFTBUFBIS_MASK      (0xFFFFFFFFU)
#define FLEXIO_SHIFTBUFBIS_SHIFTBUFBIS_SHIFT     (0U)
#define FLEXIO_SHIFTBUFBIS_SHIFTBUFBIS_WIDTH     (32U)
#define FLEXIO_SHIFTBUFBIS_SHIFTBUFBIS(x)        (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTBUFBIS_SHIFTBUFBIS_SHIFT)) & FLEXIO_SHIFTBUFBIS_SHIFTBUFBIS_MASK)
/*! @} */

/*! @name SHIFTBUFBYS - Shifter Buffer N Byte Swapped Register */
/*! @{ */
#define FLEXIO_SHIFTBUFBYS_SHIFTBUFBYS_MASK      (0xFFFFFFFFU)
#define FLEXIO_SHIFTBUFBYS_SHIFTBUFBYS_SHIFT     (0U)
#define FLEXIO_SHIFTBUFBYS_SHIFTBUFBYS_WIDTH     (32U)
#define FLEXIO_SHIFTBUFBYS_SHIFTBUFBYS(x)        (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTBUFBYS_SHIFTBUFBYS_SHIFT)) & FLEXIO_SHIFTBUFBYS_SHIFTBUFBYS_MASK)
/*! @} */

/*! @name SHIFTBUFBBS - Shifter Buffer N Bit Byte Swapped Register */
/*! @{ */
#define FLEXIO_SHIFTBUFBBS_SHIFTBUFBBS_MASK      (0xFFFFFFFFU)
#define FLEXIO_SHIFTBUFBBS_SHIFTBUFBBS_SHIFT     (0U)
#define FLEXIO_SHIFTBUFBBS_SHIFTBUFBBS_WIDTH     (32U)
#define FLEXIO_SHIFTBUFBBS_SHIFTBUFBBS(x)        (((uint32_t)(((uint32_t)(x)) << FLEXIO_SHIFTBUFBBS_SHIFTBUFBBS_SHIFT)) & FLEXIO_SHIFTBUFBBS_SHIFTBUFBBS_MASK)
/*! @} */

/*! @name TIMCTL - Timer Control N Register */
/*! @{ */
#define FLEXIO_TIMCTL_TIMOD_MASK                 (0x3U)
#define FLEXIO_TIMCTL_TIMOD_SHIFT                (0U)
#define FLEXIO_TIMCTL_TIMOD_WIDTH                (2U)
#define FLEXIO_TIMCTL_TIMOD(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCTL_TIMOD_SHIFT)) & FLEXIO_TIMCTL_TIMOD_MASK)
#define FLEXIO_TIMCTL_PINPOL_MASK                (0x80U)
#define FLEXIO_TIMCTL_PINPOL_SHIFT               (7U)
#define FLEXIO_TIMCTL_PINPOL_WIDTH               (1U)
#define FLEXIO_TIMCTL_PINPOL(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCTL_PINPOL_SHIFT)) & FLEXIO_TIMCTL_PINPOL_MASK)
#define FLEXIO_TIMCTL_PINSEL_MASK                (0x700U)
#define FLEXIO_TIMCTL_PINSEL_SHIFT               (8U)
#define FLEXIO_TIMCTL_PINSEL_WIDTH               (3U)
#define FLEXIO_TIMCTL_PINSEL(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCTL_PINSEL_SHIFT)) & FLEXIO_TIMCTL_PINSEL_MASK)
#define FLEXIO_TIMCTL_PINCFG_MASK                (0x30000U)
#define FLEXIO_TIMCTL_PINCFG_SHIFT               (16U)
#define FLEXIO_TIMCTL_PINCFG_WIDTH               (2U)
#define FLEXIO_TIMCTL_PINCFG(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCTL_PINCFG_SHIFT)) & FLEXIO_TIMCTL_PINCFG_MASK)
#define FLEXIO_TIMCTL_TRGSRC_MASK                (0x400000U)
#define FLEXIO_TIMCTL_TRGSRC_SHIFT               (22U)
#define FLEXIO_TIMCTL_TRGSRC_WIDTH               (1U)
#define FLEXIO_TIMCTL_TRGSRC(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCTL_TRGSRC_SHIFT)) & FLEXIO_TIMCTL_TRGSRC_MASK)
#define FLEXIO_TIMCTL_TRGPOL_MASK                (0x800000U)
#define FLEXIO_TIMCTL_TRGPOL_SHIFT               (23U)
#define FLEXIO_TIMCTL_TRGPOL_WIDTH               (1U)
#define FLEXIO_TIMCTL_TRGPOL(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCTL_TRGPOL_SHIFT)) & FLEXIO_TIMCTL_TRGPOL_MASK)
#define FLEXIO_TIMCTL_TRGSEL_MASK                (0xF000000U)
#define FLEXIO_TIMCTL_TRGSEL_SHIFT               (24U)
#define FLEXIO_TIMCTL_TRGSEL_WIDTH               (4U)
#define FLEXIO_TIMCTL_TRGSEL(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCTL_TRGSEL_SHIFT)) & FLEXIO_TIMCTL_TRGSEL_MASK)
/*! @} */

/*! @name TIMCFG - Timer Configuration N Register */
/*! @{ */
#define FLEXIO_TIMCFG_TSTART_MASK                (0x2U)
#define FLEXIO_TIMCFG_TSTART_SHIFT               (1U)
#define FLEXIO_TIMCFG_TSTART_WIDTH               (1U)
#define FLEXIO_TIMCFG_TSTART(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCFG_TSTART_SHIFT)) & FLEXIO_TIMCFG_TSTART_MASK)
#define FLEXIO_TIMCFG_TSTOP_MASK                 (0x30U)
#define FLEXIO_TIMCFG_TSTOP_SHIFT                (4U)
#define FLEXIO_TIMCFG_TSTOP_WIDTH                (2U)
#define FLEXIO_TIMCFG_TSTOP(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCFG_TSTOP_SHIFT)) & FLEXIO_TIMCFG_TSTOP_MASK)
#define FLEXIO_TIMCFG_TIMENA_MASK                (0x700U)
#define FLEXIO_TIMCFG_TIMENA_SHIFT               (8U)
#define FLEXIO_TIMCFG_TIMENA_WIDTH               (3U)
#define FLEXIO_TIMCFG_TIMENA(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCFG_TIMENA_SHIFT)) & FLEXIO_TIMCFG_TIMENA_MASK)
#define FLEXIO_TIMCFG_TIMDIS_MASK                (0x7000U)
#define FLEXIO_TIMCFG_TIMDIS_SHIFT               (12U)
#define FLEXIO_TIMCFG_TIMDIS_WIDTH               (3U)
#define FLEXIO_TIMCFG_TIMDIS(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCFG_TIMDIS_SHIFT)) & FLEXIO_TIMCFG_TIMDIS_MASK)
#define FLEXIO_TIMCFG_TIMRST_MASK                (0x70000U)
#define FLEXIO_TIMCFG_TIMRST_SHIFT               (16U)
#define FLEXIO_TIMCFG_TIMRST_WIDTH               (3U)
#define FLEXIO_TIMCFG_TIMRST(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCFG_TIMRST_SHIFT)) & FLEXIO_TIMCFG_TIMRST_MASK)
#define FLEXIO_TIMCFG_TIMDEC_MASK                (0x300000U)
#define FLEXIO_TIMCFG_TIMDEC_SHIFT               (20U)
#define FLEXIO_TIMCFG_TIMDEC_WIDTH               (2U)
#define FLEXIO_TIMCFG_TIMDEC(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCFG_TIMDEC_SHIFT)) & FLEXIO_TIMCFG_TIMDEC_MASK)
#define FLEXIO_TIMCFG_TIMOUT_MASK                (0x3000000U)
#define FLEXIO_TIMCFG_TIMOUT_SHIFT               (24U)
#define FLEXIO_TIMCFG_TIMOUT_WIDTH               (2U)
#define FLEXIO_TIMCFG_TIMOUT(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCFG_TIMOUT_SHIFT)) & FLEXIO_TIMCFG_TIMOUT_MASK)
/*! @} */

/*! @name TIMCMP - Timer Compare N Register */
/*! @{ */
#define FLEXIO_TIMCMP_CMP_MASK                   (0xFFFFU)
#define FLEXIO_TIMCMP_CMP_SHIFT                  (0U)
#define FLEXIO_TIMCMP_CMP_WIDTH                  (16U)
#define FLEXIO_TIMCMP_CMP(x)                     (((uint32_t)(((uint32_t)(x)) << FLEXIO_TIMCMP_CMP_SHIFT)) & FLEXIO_TIMCMP_CMP_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group FLEXIO_Register_Masks */

/*!
 * @}
 */ /* end of group FLEXIO_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- FTFM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTFM_Peripheral_Access_Layer FTFM Peripheral Access Layer
 * @{
 */

/** FTFM - Size of Registers Arrays */
#define FTFM_FCCOB_COUNT                          12u
#define FTFM_FPROT_COUNT                          4u

/** FTFM - Register Layout Typedef */
typedef struct {
  __IO uint8_t FSTAT;                              /**< Flash Status Register, offset: 0x0 */
  __IO uint8_t FCNFG;                              /**< Flash Configuration Register, offset: 0x1 */
  __I  uint8_t FSEC;                               /**< Flash Security Register, offset: 0x2 */
  __I  uint8_t FOPT;                               /**< Flash Option Register, offset: 0x3 */
  __IO uint8_t FCCOB[FTFM_FCCOB_COUNT];            /**< Flash Common Command Object Registers, array offset: 0x4, array step: 0x1 */
  __IO uint8_t FPROT[FTFM_FPROT_COUNT];            /**< Program Flash Protection Registers, array offset: 0x10, array step: 0x1 */
  uint8_t RESERVED_0[2];
  __IO uint8_t FEPROT;                             /**< EEPROM Protection Register, offset: 0x16 */
  __IO uint8_t FDPROT;                             /**< Data Flash Protection Register, offset: 0x17 */
  uint8_t RESERVED_1[20];
  __I  uint8_t FCSESTAT1;                          /**< Flash CSEc Status Register 1, offset: 0x2C */
  __I  uint8_t FCSESTAT0;                          /**< Flash CSEc Status Register 0, offset: 0x2D */
  __IO uint8_t FERSTAT;                            /**< Flash Error Status Register, offset: 0x2E */
  __IO uint8_t FERCNFG;                            /**< Flash Error Configuration Register, offset: 0x2F */
} FTFM_Type, *FTFM_MemMapPtr;

/** Number of instances of the FTFM module. */
#define FTFM_INSTANCE_COUNT                      (1u)

/* FTFM - Peripheral instance base addresses */
/** Peripheral FTFM base address */
#define FTFM_BASE                                (0x40020000u)
/** Peripheral FTFM base pointer */
#define FTFM                                     ((FTFM_Type *)FTFM_BASE)
/** Array initializer of FTFM peripheral base addresses */
#define FTFM_BASE_ADDRS                          { FTFM_BASE }
/** Array initializer of FTFM peripheral base pointers */
#define FTFM_BASE_PTRS                           { FTFM }

/* ----------------------------------------------------------------------------
   -- FTFM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTFM_Register_Masks FTFM Register Masks
 * @{
 */

/*! @name FSTAT - Flash Status Register */
/*! @{ */
#define FTFM_FSTAT_MGSTAT0_MASK                  (0x1U)
#define FTFM_FSTAT_MGSTAT0_SHIFT                 (0U)
#define FTFM_FSTAT_MGSTAT0_WIDTH                 (1U)
#define FTFM_FSTAT_MGSTAT0(x)                    (((uint8_t)(((uint8_t)(x)) << FTFM_FSTAT_MGSTAT0_SHIFT)) & FTFM_FSTAT_MGSTAT0_MASK)
#define FTFM_FSTAT_MGSTAT1_MASK                  (0x2U)
#define FTFM_FSTAT_MGSTAT1_SHIFT                 (1U)
#define FTFM_FSTAT_MGSTAT1_WIDTH                 (1U)
#define FTFM_FSTAT_MGSTAT1(x)                    (((uint8_t)(((uint8_t)(x)) << FTFM_FSTAT_MGSTAT1_SHIFT)) & FTFM_FSTAT_MGSTAT1_MASK)
#define FTFM_FSTAT_MGSTAT2_MASK                  (0x4U)
#define FTFM_FSTAT_MGSTAT2_SHIFT                 (2U)
#define FTFM_FSTAT_MGSTAT2_WIDTH                 (1U)
#define FTFM_FSTAT_MGSTAT2(x)                    (((uint8_t)(((uint8_t)(x)) << FTFM_FSTAT_MGSTAT2_SHIFT)) & FTFM_FSTAT_MGSTAT2_MASK)
#define FTFM_FSTAT_MGSTAT3_MASK                  (0x8U)
#define FTFM_FSTAT_MGSTAT3_SHIFT                 (3U)
#define FTFM_FSTAT_MGSTAT3_WIDTH                 (1U)
#define FTFM_FSTAT_MGSTAT3(x)                    (((uint8_t)(((uint8_t)(x)) << FTFM_FSTAT_MGSTAT3_SHIFT)) & FTFM_FSTAT_MGSTAT3_MASK)
#define FTFM_FSTAT_FPVIOL_MASK                   (0x10U)
#define FTFM_FSTAT_FPVIOL_SHIFT                  (4U)
#define FTFM_FSTAT_FPVIOL_WIDTH                  (1U)
#define FTFM_FSTAT_FPVIOL(x)                     (((uint8_t)(((uint8_t)(x)) << FTFM_FSTAT_FPVIOL_SHIFT)) & FTFM_FSTAT_FPVIOL_MASK)
#define FTFM_FSTAT_ACCERR_MASK                   (0x20U)
#define FTFM_FSTAT_ACCERR_SHIFT                  (5U)
#define FTFM_FSTAT_ACCERR_WIDTH                  (1U)
#define FTFM_FSTAT_ACCERR(x)                     (((uint8_t)(((uint8_t)(x)) << FTFM_FSTAT_ACCERR_SHIFT)) & FTFM_FSTAT_ACCERR_MASK)
#define FTFM_FSTAT_RDCOLERR_MASK                 (0x40U)
#define FTFM_FSTAT_RDCOLERR_SHIFT                (6U)
#define FTFM_FSTAT_RDCOLERR_WIDTH                (1U)
#define FTFM_FSTAT_RDCOLERR(x)                   (((uint8_t)(((uint8_t)(x)) << FTFM_FSTAT_RDCOLERR_SHIFT)) & FTFM_FSTAT_RDCOLERR_MASK)
#define FTFM_FSTAT_CCIF_MASK                     (0x80U)
#define FTFM_FSTAT_CCIF_SHIFT                    (7U)
#define FTFM_FSTAT_CCIF_WIDTH                    (1U)
#define FTFM_FSTAT_CCIF(x)                       (((uint8_t)(((uint8_t)(x)) << FTFM_FSTAT_CCIF_SHIFT)) & FTFM_FSTAT_CCIF_MASK)
/*! @} */

/*! @name FCNFG - Flash Configuration Register */
/*! @{ */
#define FTFM_FCNFG_EEERDY_MASK                   (0x1U)
#define FTFM_FCNFG_EEERDY_SHIFT                  (0U)
#define FTFM_FCNFG_EEERDY_WIDTH                  (1U)
#define FTFM_FCNFG_EEERDY(x)                     (((uint8_t)(((uint8_t)(x)) << FTFM_FCNFG_EEERDY_SHIFT)) & FTFM_FCNFG_EEERDY_MASK)
#define FTFM_FCNFG_RAMRDY_MASK                   (0x2U)
#define FTFM_FCNFG_RAMRDY_SHIFT                  (1U)
#define FTFM_FCNFG_RAMRDY_WIDTH                  (1U)
#define FTFM_FCNFG_RAMRDY(x)                     (((uint8_t)(((uint8_t)(x)) << FTFM_FCNFG_RAMRDY_SHIFT)) & FTFM_FCNFG_RAMRDY_MASK)
#define FTFM_FCNFG_ERSSUSP_MASK                  (0x10U)
#define FTFM_FCNFG_ERSSUSP_SHIFT                 (4U)
#define FTFM_FCNFG_ERSSUSP_WIDTH                 (1U)
#define FTFM_FCNFG_ERSSUSP(x)                    (((uint8_t)(((uint8_t)(x)) << FTFM_FCNFG_ERSSUSP_SHIFT)) & FTFM_FCNFG_ERSSUSP_MASK)
#define FTFM_FCNFG_ERSAREQ_MASK                  (0x20U)
#define FTFM_FCNFG_ERSAREQ_SHIFT                 (5U)
#define FTFM_FCNFG_ERSAREQ_WIDTH                 (1U)
#define FTFM_FCNFG_ERSAREQ(x)                    (((uint8_t)(((uint8_t)(x)) << FTFM_FCNFG_ERSAREQ_SHIFT)) & FTFM_FCNFG_ERSAREQ_MASK)
#define FTFM_FCNFG_RDCOLLIE_MASK                 (0x40U)
#define FTFM_FCNFG_RDCOLLIE_SHIFT                (6U)
#define FTFM_FCNFG_RDCOLLIE_WIDTH                (1U)
#define FTFM_FCNFG_RDCOLLIE(x)                   (((uint8_t)(((uint8_t)(x)) << FTFM_FCNFG_RDCOLLIE_SHIFT)) & FTFM_FCNFG_RDCOLLIE_MASK)
#define FTFM_FCNFG_CCIE_MASK                     (0x80U)
#define FTFM_FCNFG_CCIE_SHIFT                    (7U)
#define FTFM_FCNFG_CCIE_WIDTH                    (1U)
#define FTFM_FCNFG_CCIE(x)                       (((uint8_t)(((uint8_t)(x)) << FTFM_FCNFG_CCIE_SHIFT)) & FTFM_FCNFG_CCIE_MASK)
/*! @} */

/*! @name FSEC - Flash Security Register */
/*! @{ */
#define FTFM_FSEC_SEC_MASK                       (0x3U)
#define FTFM_FSEC_SEC_SHIFT                      (0U)
#define FTFM_FSEC_SEC_WIDTH                      (2U)
#define FTFM_FSEC_SEC(x)                         (((uint8_t)(((uint8_t)(x)) << FTFM_FSEC_SEC_SHIFT)) & FTFM_FSEC_SEC_MASK)
#define FTFM_FSEC_FSLACC_MASK                    (0xCU)
#define FTFM_FSEC_FSLACC_SHIFT                   (2U)
#define FTFM_FSEC_FSLACC_WIDTH                   (2U)
#define FTFM_FSEC_FSLACC(x)                      (((uint8_t)(((uint8_t)(x)) << FTFM_FSEC_FSLACC_SHIFT)) & FTFM_FSEC_FSLACC_MASK)
#define FTFM_FSEC_MEEN_MASK                      (0x30U)
#define FTFM_FSEC_MEEN_SHIFT                     (4U)
#define FTFM_FSEC_MEEN_WIDTH                     (2U)
#define FTFM_FSEC_MEEN(x)                        (((uint8_t)(((uint8_t)(x)) << FTFM_FSEC_MEEN_SHIFT)) & FTFM_FSEC_MEEN_MASK)
#define FTFM_FSEC_KEYEN_MASK                     (0xC0U)
#define FTFM_FSEC_KEYEN_SHIFT                    (6U)
#define FTFM_FSEC_KEYEN_WIDTH                    (2U)
#define FTFM_FSEC_KEYEN(x)                       (((uint8_t)(((uint8_t)(x)) << FTFM_FSEC_KEYEN_SHIFT)) & FTFM_FSEC_KEYEN_MASK)
/*! @} */

/*! @name FOPT - Flash Option Register */
/*! @{ */
#define FTFM_FOPT_OPT_MASK                       (0xFFU)
#define FTFM_FOPT_OPT_SHIFT                      (0U)
#define FTFM_FOPT_OPT_WIDTH                      (8U)
#define FTFM_FOPT_OPT(x)                         (((uint8_t)(((uint8_t)(x)) << FTFM_FOPT_OPT_SHIFT)) & FTFM_FOPT_OPT_MASK)
/*! @} */

/*! @name FCCOB - Flash Common Command Object Registers */
/*! @{ */
#define FTFM_FCCOB_CCOBn_MASK                    (0xFFU)
#define FTFM_FCCOB_CCOBn_SHIFT                   (0U)
#define FTFM_FCCOB_CCOBn_WIDTH                   (8U)
#define FTFM_FCCOB_CCOBn(x)                      (((uint8_t)(((uint8_t)(x)) << FTFM_FCCOB_CCOBn_SHIFT)) & FTFM_FCCOB_CCOBn_MASK)
/*! @} */

/*! @name FPROT - Program Flash Protection Registers */
/*! @{ */
#define FTFM_FPROT_PROT_MASK                     (0xFFU)
#define FTFM_FPROT_PROT_SHIFT                    (0U)
#define FTFM_FPROT_PROT_WIDTH                    (8U)
#define FTFM_FPROT_PROT(x)                       (((uint8_t)(((uint8_t)(x)) << FTFM_FPROT_PROT_SHIFT)) & FTFM_FPROT_PROT_MASK)
/*! @} */

/*! @name FEPROT - EEPROM Protection Register */
/*! @{ */
#define FTFM_FEPROT_EPROT_MASK                   (0xFFU)
#define FTFM_FEPROT_EPROT_SHIFT                  (0U)
#define FTFM_FEPROT_EPROT_WIDTH                  (8U)
#define FTFM_FEPROT_EPROT(x)                     (((uint8_t)(((uint8_t)(x)) << FTFM_FEPROT_EPROT_SHIFT)) & FTFM_FEPROT_EPROT_MASK)
/*! @} */

/*! @name FDPROT - Data Flash Protection Register */
/*! @{ */
#define FTFM_FDPROT_DPROT_MASK                   (0xFFU)
#define FTFM_FDPROT_DPROT_SHIFT                  (0U)
#define FTFM_FDPROT_DPROT_WIDTH                  (8U)
#define FTFM_FDPROT_DPROT(x)                     (((uint8_t)(((uint8_t)(x)) << FTFM_FDPROT_DPROT_SHIFT)) & FTFM_FDPROT_DPROT_MASK)
/*! @} */

/*! @name FCSESTAT1 - Flash CSEc Status Register 1 */
/*! @{ */
#define FTFM_FCSESTAT1_BSY_MASK                  (0x1U)
#define FTFM_FCSESTAT1_BSY_SHIFT                 (0U)
#define FTFM_FCSESTAT1_BSY_WIDTH                 (1U)
#define FTFM_FCSESTAT1_BSY(x)                    (((uint8_t)(((uint8_t)(x)) << FTFM_FCSESTAT1_BSY_SHIFT)) & FTFM_FCSESTAT1_BSY_MASK)
#define FTFM_FCSESTAT1_SB_MASK                   (0x2U)
#define FTFM_FCSESTAT1_SB_SHIFT                  (1U)
#define FTFM_FCSESTAT1_SB_WIDTH                  (1U)
#define FTFM_FCSESTAT1_SB(x)                     (((uint8_t)(((uint8_t)(x)) << FTFM_FCSESTAT1_SB_SHIFT)) & FTFM_FCSESTAT1_SB_MASK)
#define FTFM_FCSESTAT1_BIN_MASK                  (0x4U)
#define FTFM_FCSESTAT1_BIN_SHIFT                 (2U)
#define FTFM_FCSESTAT1_BIN_WIDTH                 (1U)
#define FTFM_FCSESTAT1_BIN(x)                    (((uint8_t)(((uint8_t)(x)) << FTFM_FCSESTAT1_BIN_SHIFT)) & FTFM_FCSESTAT1_BIN_MASK)
#define FTFM_FCSESTAT1_BFN_MASK                  (0x8U)
#define FTFM_FCSESTAT1_BFN_SHIFT                 (3U)
#define FTFM_FCSESTAT1_BFN_WIDTH                 (1U)
#define FTFM_FCSESTAT1_BFN(x)                    (((uint8_t)(((uint8_t)(x)) << FTFM_FCSESTAT1_BFN_SHIFT)) & FTFM_FCSESTAT1_BFN_MASK)
#define FTFM_FCSESTAT1_BOK_MASK                  (0x10U)
#define FTFM_FCSESTAT1_BOK_SHIFT                 (4U)
#define FTFM_FCSESTAT1_BOK_WIDTH                 (1U)
#define FTFM_FCSESTAT1_BOK(x)                    (((uint8_t)(((uint8_t)(x)) << FTFM_FCSESTAT1_BOK_SHIFT)) & FTFM_FCSESTAT1_BOK_MASK)
#define FTFM_FCSESTAT1_RIN_MASK                  (0x20U)
#define FTFM_FCSESTAT1_RIN_SHIFT                 (5U)
#define FTFM_FCSESTAT1_RIN_WIDTH                 (1U)
#define FTFM_FCSESTAT1_RIN(x)                    (((uint8_t)(((uint8_t)(x)) << FTFM_FCSESTAT1_RIN_SHIFT)) & FTFM_FCSESTAT1_RIN_MASK)
#define FTFM_FCSESTAT1_EDB_MASK                  (0x40U)
#define FTFM_FCSESTAT1_EDB_SHIFT                 (6U)
#define FTFM_FCSESTAT1_EDB_WIDTH                 (1U)
#define FTFM_FCSESTAT1_EDB(x)                    (((uint8_t)(((uint8_t)(x)) << FTFM_FCSESTAT1_EDB_SHIFT)) & FTFM_FCSESTAT1_EDB_MASK)
#define FTFM_FCSESTAT1_IDB_MASK                  (0x80U)
#define FTFM_FCSESTAT1_IDB_SHIFT                 (7U)
#define FTFM_FCSESTAT1_IDB_WIDTH                 (1U)
#define FTFM_FCSESTAT1_IDB(x)                    (((uint8_t)(((uint8_t)(x)) << FTFM_FCSESTAT1_IDB_SHIFT)) & FTFM_FCSESTAT1_IDB_MASK)
/*! @} */

/*! @name FCSESTAT0 - Flash CSEc Status Register 0 */
/*! @{ */
#define FTFM_FCSESTAT0_CMDTYPE_MASK              (0x2U)
#define FTFM_FCSESTAT0_CMDTYPE_SHIFT             (1U)
#define FTFM_FCSESTAT0_CMDTYPE_WIDTH             (1U)
#define FTFM_FCSESTAT0_CMDTYPE(x)                (((uint8_t)(((uint8_t)(x)) << FTFM_FCSESTAT0_CMDTYPE_SHIFT)) & FTFM_FCSESTAT0_CMDTYPE_MASK)
#define FTFM_FCSESTAT0_MEMERR_MASK               (0x4U)
#define FTFM_FCSESTAT0_MEMERR_SHIFT              (2U)
#define FTFM_FCSESTAT0_MEMERR_WIDTH              (1U)
#define FTFM_FCSESTAT0_MEMERR(x)                 (((uint8_t)(((uint8_t)(x)) << FTFM_FCSESTAT0_MEMERR_SHIFT)) & FTFM_FCSESTAT0_MEMERR_MASK)
/*! @} */

/*! @name FERSTAT - Flash Error Status Register */
/*! @{ */
#define FTFM_FERSTAT_PDFDIF_MASK                 (0x1U)
#define FTFM_FERSTAT_PDFDIF_SHIFT                (0U)
#define FTFM_FERSTAT_PDFDIF_WIDTH                (1U)
#define FTFM_FERSTAT_PDFDIF(x)                   (((uint8_t)(((uint8_t)(x)) << FTFM_FERSTAT_PDFDIF_SHIFT)) & FTFM_FERSTAT_PDFDIF_MASK)
#define FTFM_FERSTAT_DFDIF_MASK                  (0x2U)
#define FTFM_FERSTAT_DFDIF_SHIFT                 (1U)
#define FTFM_FERSTAT_DFDIF_WIDTH                 (1U)
#define FTFM_FERSTAT_DFDIF(x)                    (((uint8_t)(((uint8_t)(x)) << FTFM_FERSTAT_DFDIF_SHIFT)) & FTFM_FERSTAT_DFDIF_MASK)
#define FTFM_FERSTAT_EDFDIF_MASK                 (0x4U)
#define FTFM_FERSTAT_EDFDIF_SHIFT                (2U)
#define FTFM_FERSTAT_EDFDIF_WIDTH                (1U)
#define FTFM_FERSTAT_EDFDIF(x)                   (((uint8_t)(((uint8_t)(x)) << FTFM_FERSTAT_EDFDIF_SHIFT)) & FTFM_FERSTAT_EDFDIF_MASK)
#define FTFM_FERSTAT_CDFDIF_MASK                 (0x8U)
#define FTFM_FERSTAT_CDFDIF_SHIFT                (3U)
#define FTFM_FERSTAT_CDFDIF_WIDTH                (1U)
#define FTFM_FERSTAT_CDFDIF(x)                   (((uint8_t)(((uint8_t)(x)) << FTFM_FERSTAT_CDFDIF_SHIFT)) & FTFM_FERSTAT_CDFDIF_MASK)
/*! @} */

/*! @name FERCNFG - Flash Error Configuration Register */
/*! @{ */
#define FTFM_FERCNFG_PDFDIE_MASK                 (0x1U)
#define FTFM_FERCNFG_PDFDIE_SHIFT                (0U)
#define FTFM_FERCNFG_PDFDIE_WIDTH                (1U)
#define FTFM_FERCNFG_PDFDIE(x)                   (((uint8_t)(((uint8_t)(x)) << FTFM_FERCNFG_PDFDIE_SHIFT)) & FTFM_FERCNFG_PDFDIE_MASK)
#define FTFM_FERCNFG_DFDIE_MASK                  (0x2U)
#define FTFM_FERCNFG_DFDIE_SHIFT                 (1U)
#define FTFM_FERCNFG_DFDIE_WIDTH                 (1U)
#define FTFM_FERCNFG_DFDIE(x)                    (((uint8_t)(((uint8_t)(x)) << FTFM_FERCNFG_DFDIE_SHIFT)) & FTFM_FERCNFG_DFDIE_MASK)
#define FTFM_FERCNFG_EDFDIE_MASK                 (0x4U)
#define FTFM_FERCNFG_EDFDIE_SHIFT                (2U)
#define FTFM_FERCNFG_EDFDIE_WIDTH                (1U)
#define FTFM_FERCNFG_EDFDIE(x)                   (((uint8_t)(((uint8_t)(x)) << FTFM_FERCNFG_EDFDIE_SHIFT)) & FTFM_FERCNFG_EDFDIE_MASK)
#define FTFM_FERCNFG_CDFDIE_MASK                 (0x8U)
#define FTFM_FERCNFG_CDFDIE_SHIFT                (3U)
#define FTFM_FERCNFG_CDFDIE_WIDTH                (1U)
#define FTFM_FERCNFG_CDFDIE(x)                   (((uint8_t)(((uint8_t)(x)) << FTFM_FERCNFG_CDFDIE_SHIFT)) & FTFM_FERCNFG_CDFDIE_MASK)
#define FTFM_FERCNFG_PFDFD_MASK                  (0x10U)
#define FTFM_FERCNFG_PFDFD_SHIFT                 (4U)
#define FTFM_FERCNFG_PFDFD_WIDTH                 (1U)
#define FTFM_FERCNFG_PFDFD(x)                    (((uint8_t)(((uint8_t)(x)) << FTFM_FERCNFG_PFDFD_SHIFT)) & FTFM_FERCNFG_PFDFD_MASK)
#define FTFM_FERCNFG_FDFD_MASK                   (0x20U)
#define FTFM_FERCNFG_FDFD_SHIFT                  (5U)
#define FTFM_FERCNFG_FDFD_WIDTH                  (1U)
#define FTFM_FERCNFG_FDFD(x)                     (((uint8_t)(((uint8_t)(x)) << FTFM_FERCNFG_FDFD_SHIFT)) & FTFM_FERCNFG_FDFD_MASK)
#define FTFM_FERCNFG_EFDFD_MASK                  (0x40U)
#define FTFM_FERCNFG_EFDFD_SHIFT                 (6U)
#define FTFM_FERCNFG_EFDFD_WIDTH                 (1U)
#define FTFM_FERCNFG_EFDFD(x)                    (((uint8_t)(((uint8_t)(x)) << FTFM_FERCNFG_EFDFD_SHIFT)) & FTFM_FERCNFG_EFDFD_MASK)
#define FTFM_FERCNFG_CFDFD_MASK                  (0x80U)
#define FTFM_FERCNFG_CFDFD_SHIFT                 (7U)
#define FTFM_FERCNFG_CFDFD_WIDTH                 (1U)
#define FTFM_FERCNFG_CFDFD(x)                    (((uint8_t)(((uint8_t)(x)) << FTFM_FERCNFG_CFDFD_SHIFT)) & FTFM_FERCNFG_CFDFD_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group FTFM_Register_Masks */

/*!
 * @}
 */ /* end of group FTFM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- FTM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTM_Peripheral_Access_Layer FTM Peripheral Access Layer
 * @{
 */

/** FTM - Size of Registers Arrays */
#define FTM_CONTROLS_COUNT                        8u
#define FTM_CV_MIRROR_COUNT                       8u

/** FTM - Register Layout Typedef */
typedef struct {
  __IO uint32_t SC;                                /**< Status And Control, offset: 0x0 */
  __IO uint32_t CNT;                               /**< Counter, offset: 0x4 */
  __IO uint32_t MOD;                               /**< Modulo, offset: 0x8 */
  struct {                                         /* offset: 0xC, array step: 0x8 */
    __IO uint32_t CnSC;                              /**< Channel (n) Status And Control, array offset: 0xC, array step: 0x8 */
    __IO uint32_t CnV;                               /**< Channel (n) Value, array offset: 0x10, array step: 0x8 */
  } CONTROLS[FTM_CONTROLS_COUNT];
  __IO uint32_t CNTIN;                             /**< Counter Initial Value, offset: 0x4C */
  __IO uint32_t STATUS;                            /**< Capture And Compare Status, offset: 0x50 */
  __IO uint32_t MODE;                              /**< Features Mode Selection, offset: 0x54 */
  __IO uint32_t SYNC;                              /**< Synchronization, offset: 0x58 */
  __IO uint32_t OUTINIT;                           /**< Initial State For Channels Output, offset: 0x5C */
  __IO uint32_t OUTMASK;                           /**< Output Mask, offset: 0x60 */
  __IO uint32_t COMBINE;                           /**< Function For Linked Channels, offset: 0x64 */
  __IO uint32_t DEADTIME;                          /**< Deadtime Configuration, offset: 0x68 */
  __IO uint32_t EXTTRIG;                           /**< FTM External Trigger, offset: 0x6C */
  __IO uint32_t POL;                               /**< Channels Polarity, offset: 0x70 */
  __IO uint32_t FMS;                               /**< Fault Mode Status, offset: 0x74 */
  __IO uint32_t FILTER;                            /**< Input Capture Filter Control, offset: 0x78 */
  __IO uint32_t FLTCTRL;                           /**< Fault Control, offset: 0x7C */
  __IO uint32_t QDCTRL;                            /**< Quadrature Decoder Control And Status, offset: 0x80 */
  __IO uint32_t CONF;                              /**< Configuration, offset: 0x84 */
  __IO uint32_t FLTPOL;                            /**< FTM Fault Input Polarity, offset: 0x88 */
  __IO uint32_t SYNCONF;                           /**< Synchronization Configuration, offset: 0x8C */
  __IO uint32_t INVCTRL;                           /**< FTM Inverting Control, offset: 0x90 */
  __IO uint32_t SWOCTRL;                           /**< FTM Software Output Control, offset: 0x94 */
  __IO uint32_t PWMLOAD;                           /**< FTM PWM Load, offset: 0x98 */
  __IO uint32_t HCR;                               /**< Half Cycle Register, offset: 0x9C */
  __IO uint32_t PAIR0DEADTIME;                     /**< Pair 0 Deadtime Configuration, offset: 0xA0 */
  uint8_t RESERVED_0[4];
  __IO uint32_t PAIR1DEADTIME;                     /**< Pair 1 Deadtime Configuration, offset: 0xA8 */
  uint8_t RESERVED_1[4];
  __IO uint32_t PAIR2DEADTIME;                     /**< Pair 2 Deadtime Configuration, offset: 0xB0 */
  uint8_t RESERVED_2[4];
  __IO uint32_t PAIR3DEADTIME;                     /**< Pair 3 Deadtime Configuration, offset: 0xB8 */
  uint8_t RESERVED_3[324];
  __IO uint32_t MOD_MIRROR;                        /**< Mirror of Modulo Value, offset: 0x200 */
  __IO uint32_t CV_MIRROR[FTM_CV_MIRROR_COUNT];    /**< Mirror of Channel (n) Match Value, array offset: 0x204, array step: 0x4 */
} FTM_Type, *FTM_MemMapPtr;

/** Number of instances of the FTM module. */
#define FTM_INSTANCE_COUNT                       (4u)

/* FTM - Peripheral instance base addresses */
/** Peripheral FTM0 base address */
#define FTM0_BASE                                (0x40038000u)
/** Peripheral FTM0 base pointer */
#define FTM0                                     ((FTM_Type *)FTM0_BASE)
/** Peripheral FTM1 base address */
#define FTM1_BASE                                (0x40039000u)
/** Peripheral FTM1 base pointer */
#define FTM1                                     ((FTM_Type *)FTM1_BASE)
/** Peripheral FTM2 base address */
#define FTM2_BASE                                (0x4003A000u)
/** Peripheral FTM2 base pointer */
#define FTM2                                     ((FTM_Type *)FTM2_BASE)
/** Peripheral FTM3 base address */
#define FTM3_BASE                                (0x40026000u)
/** Peripheral FTM3 base pointer */
#define FTM3                                     ((FTM_Type *)FTM3_BASE)
/** Array initializer of FTM peripheral base addresses */
#define FTM_BASE_ADDRS                           { FTM0_BASE, FTM1_BASE, FTM2_BASE, FTM3_BASE }
/** Array initializer of FTM peripheral base pointers */
#define FTM_BASE_PTRS                            { FTM0, FTM1, FTM2, FTM3 }

/* ----------------------------------------------------------------------------
   -- FTM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTM_Register_Masks FTM Register Masks
 * @{
 */

/*! @name SC - Status And Control */
/*! @{ */
#define FTM_SC_PS_MASK                           (0x7U)
#define FTM_SC_PS_SHIFT                          (0U)
#define FTM_SC_PS_WIDTH                          (3U)
#define FTM_SC_PS(x)                             (((uint32_t)(((uint32_t)(x)) << FTM_SC_PS_SHIFT)) & FTM_SC_PS_MASK)
#define FTM_SC_CLKS_MASK                         (0x18U)
#define FTM_SC_CLKS_SHIFT                        (3U)
#define FTM_SC_CLKS_WIDTH                        (2U)
#define FTM_SC_CLKS(x)                           (((uint32_t)(((uint32_t)(x)) << FTM_SC_CLKS_SHIFT)) & FTM_SC_CLKS_MASK)
#define FTM_SC_CPWMS_MASK                        (0x20U)
#define FTM_SC_CPWMS_SHIFT                       (5U)
#define FTM_SC_CPWMS_WIDTH                       (1U)
#define FTM_SC_CPWMS(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_SC_CPWMS_SHIFT)) & FTM_SC_CPWMS_MASK)
#define FTM_SC_RIE_MASK                          (0x40U)
#define FTM_SC_RIE_SHIFT                         (6U)
#define FTM_SC_RIE_WIDTH                         (1U)
#define FTM_SC_RIE(x)                            (((uint32_t)(((uint32_t)(x)) << FTM_SC_RIE_SHIFT)) & FTM_SC_RIE_MASK)
#define FTM_SC_RF_MASK                           (0x80U)
#define FTM_SC_RF_SHIFT                          (7U)
#define FTM_SC_RF_WIDTH                          (1U)
#define FTM_SC_RF(x)                             (((uint32_t)(((uint32_t)(x)) << FTM_SC_RF_SHIFT)) & FTM_SC_RF_MASK)
#define FTM_SC_TOIE_MASK                         (0x100U)
#define FTM_SC_TOIE_SHIFT                        (8U)
#define FTM_SC_TOIE_WIDTH                        (1U)
#define FTM_SC_TOIE(x)                           (((uint32_t)(((uint32_t)(x)) << FTM_SC_TOIE_SHIFT)) & FTM_SC_TOIE_MASK)
#define FTM_SC_TOF_MASK                          (0x200U)
#define FTM_SC_TOF_SHIFT                         (9U)
#define FTM_SC_TOF_WIDTH                         (1U)
#define FTM_SC_TOF(x)                            (((uint32_t)(((uint32_t)(x)) << FTM_SC_TOF_SHIFT)) & FTM_SC_TOF_MASK)
#define FTM_SC_PWMEN0_MASK                       (0x10000U)
#define FTM_SC_PWMEN0_SHIFT                      (16U)
#define FTM_SC_PWMEN0_WIDTH                      (1U)
#define FTM_SC_PWMEN0(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_SC_PWMEN0_SHIFT)) & FTM_SC_PWMEN0_MASK)
#define FTM_SC_PWMEN1_MASK                       (0x20000U)
#define FTM_SC_PWMEN1_SHIFT                      (17U)
#define FTM_SC_PWMEN1_WIDTH                      (1U)
#define FTM_SC_PWMEN1(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_SC_PWMEN1_SHIFT)) & FTM_SC_PWMEN1_MASK)
#define FTM_SC_PWMEN2_MASK                       (0x40000U)
#define FTM_SC_PWMEN2_SHIFT                      (18U)
#define FTM_SC_PWMEN2_WIDTH                      (1U)
#define FTM_SC_PWMEN2(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_SC_PWMEN2_SHIFT)) & FTM_SC_PWMEN2_MASK)
#define FTM_SC_PWMEN3_MASK                       (0x80000U)
#define FTM_SC_PWMEN3_SHIFT                      (19U)
#define FTM_SC_PWMEN3_WIDTH                      (1U)
#define FTM_SC_PWMEN3(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_SC_PWMEN3_SHIFT)) & FTM_SC_PWMEN3_MASK)
#define FTM_SC_PWMEN4_MASK                       (0x100000U)
#define FTM_SC_PWMEN4_SHIFT                      (20U)
#define FTM_SC_PWMEN4_WIDTH                      (1U)
#define FTM_SC_PWMEN4(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_SC_PWMEN4_SHIFT)) & FTM_SC_PWMEN4_MASK)
#define FTM_SC_PWMEN5_MASK                       (0x200000U)
#define FTM_SC_PWMEN5_SHIFT                      (21U)
#define FTM_SC_PWMEN5_WIDTH                      (1U)
#define FTM_SC_PWMEN5(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_SC_PWMEN5_SHIFT)) & FTM_SC_PWMEN5_MASK)
#define FTM_SC_PWMEN6_MASK                       (0x400000U)
#define FTM_SC_PWMEN6_SHIFT                      (22U)
#define FTM_SC_PWMEN6_WIDTH                      (1U)
#define FTM_SC_PWMEN6(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_SC_PWMEN6_SHIFT)) & FTM_SC_PWMEN6_MASK)
#define FTM_SC_PWMEN7_MASK                       (0x800000U)
#define FTM_SC_PWMEN7_SHIFT                      (23U)
#define FTM_SC_PWMEN7_WIDTH                      (1U)
#define FTM_SC_PWMEN7(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_SC_PWMEN7_SHIFT)) & FTM_SC_PWMEN7_MASK)
#define FTM_SC_FLTPS_MASK                        (0xF000000U)
#define FTM_SC_FLTPS_SHIFT                       (24U)
#define FTM_SC_FLTPS_WIDTH                       (4U)
#define FTM_SC_FLTPS(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_SC_FLTPS_SHIFT)) & FTM_SC_FLTPS_MASK)
/*! @} */

/*! @name CNT - Counter */
/*! @{ */
#define FTM_CNT_COUNT_MASK                       (0xFFFFU)
#define FTM_CNT_COUNT_SHIFT                      (0U)
#define FTM_CNT_COUNT_WIDTH                      (16U)
#define FTM_CNT_COUNT(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_CNT_COUNT_SHIFT)) & FTM_CNT_COUNT_MASK)
/*! @} */

/*! @name MOD - Modulo */
/*! @{ */
#define FTM_MOD_MOD_MASK                         (0xFFFFU)
#define FTM_MOD_MOD_SHIFT                        (0U)
#define FTM_MOD_MOD_WIDTH                        (16U)
#define FTM_MOD_MOD(x)                           (((uint32_t)(((uint32_t)(x)) << FTM_MOD_MOD_SHIFT)) & FTM_MOD_MOD_MASK)
/*! @} */

/*! @name CnSC - Channel (n) Status And Control */
/*! @{ */
#define FTM_CnSC_DMA_MASK                        (0x1U)
#define FTM_CnSC_DMA_SHIFT                       (0U)
#define FTM_CnSC_DMA_WIDTH                       (1U)
#define FTM_CnSC_DMA(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_DMA_SHIFT)) & FTM_CnSC_DMA_MASK)
#define FTM_CnSC_ICRST_MASK                      (0x2U)
#define FTM_CnSC_ICRST_SHIFT                     (1U)
#define FTM_CnSC_ICRST_WIDTH                     (1U)
#define FTM_CnSC_ICRST(x)                        (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_ICRST_SHIFT)) & FTM_CnSC_ICRST_MASK)
#define FTM_CnSC_ELSA_MASK                       (0x4U)
#define FTM_CnSC_ELSA_SHIFT                      (2U)
#define FTM_CnSC_ELSA_WIDTH                      (1U)
#define FTM_CnSC_ELSA(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_ELSA_SHIFT)) & FTM_CnSC_ELSA_MASK)
#define FTM_CnSC_ELSB_MASK                       (0x8U)
#define FTM_CnSC_ELSB_SHIFT                      (3U)
#define FTM_CnSC_ELSB_WIDTH                      (1U)
#define FTM_CnSC_ELSB(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_ELSB_SHIFT)) & FTM_CnSC_ELSB_MASK)
#define FTM_CnSC_MSA_MASK                        (0x10U)
#define FTM_CnSC_MSA_SHIFT                       (4U)
#define FTM_CnSC_MSA_WIDTH                       (1U)
#define FTM_CnSC_MSA(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_MSA_SHIFT)) & FTM_CnSC_MSA_MASK)
#define FTM_CnSC_MSB_MASK                        (0x20U)
#define FTM_CnSC_MSB_SHIFT                       (5U)
#define FTM_CnSC_MSB_WIDTH                       (1U)
#define FTM_CnSC_MSB(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_MSB_SHIFT)) & FTM_CnSC_MSB_MASK)
#define FTM_CnSC_CHIE_MASK                       (0x40U)
#define FTM_CnSC_CHIE_SHIFT                      (6U)
#define FTM_CnSC_CHIE_WIDTH                      (1U)
#define FTM_CnSC_CHIE(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_CHIE_SHIFT)) & FTM_CnSC_CHIE_MASK)
#define FTM_CnSC_CHF_MASK                        (0x80U)
#define FTM_CnSC_CHF_SHIFT                       (7U)
#define FTM_CnSC_CHF_WIDTH                       (1U)
#define FTM_CnSC_CHF(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_CHF_SHIFT)) & FTM_CnSC_CHF_MASK)
#define FTM_CnSC_TRIGMODE_MASK                   (0x100U)
#define FTM_CnSC_TRIGMODE_SHIFT                  (8U)
#define FTM_CnSC_TRIGMODE_WIDTH                  (1U)
#define FTM_CnSC_TRIGMODE(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_TRIGMODE_SHIFT)) & FTM_CnSC_TRIGMODE_MASK)
#define FTM_CnSC_CHIS_MASK                       (0x200U)
#define FTM_CnSC_CHIS_SHIFT                      (9U)
#define FTM_CnSC_CHIS_WIDTH                      (1U)
#define FTM_CnSC_CHIS(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_CHIS_SHIFT)) & FTM_CnSC_CHIS_MASK)
#define FTM_CnSC_CHOV_MASK                       (0x400U)
#define FTM_CnSC_CHOV_SHIFT                      (10U)
#define FTM_CnSC_CHOV_WIDTH                      (1U)
#define FTM_CnSC_CHOV(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_CHOV_SHIFT)) & FTM_CnSC_CHOV_MASK)
/*! @} */

/*! @name CnV - Channel (n) Value */
/*! @{ */
#define FTM_CnV_VAL_MASK                         (0xFFFFU)
#define FTM_CnV_VAL_SHIFT                        (0U)
#define FTM_CnV_VAL_WIDTH                        (16U)
#define FTM_CnV_VAL(x)                           (((uint32_t)(((uint32_t)(x)) << FTM_CnV_VAL_SHIFT)) & FTM_CnV_VAL_MASK)
/*! @} */

/*! @name CNTIN - Counter Initial Value */
/*! @{ */
#define FTM_CNTIN_INIT_MASK                      (0xFFFFU)
#define FTM_CNTIN_INIT_SHIFT                     (0U)
#define FTM_CNTIN_INIT_WIDTH                     (16U)
#define FTM_CNTIN_INIT(x)                        (((uint32_t)(((uint32_t)(x)) << FTM_CNTIN_INIT_SHIFT)) & FTM_CNTIN_INIT_MASK)
/*! @} */

/*! @name STATUS - Capture And Compare Status */
/*! @{ */
#define FTM_STATUS_CH0F_MASK                     (0x1U)
#define FTM_STATUS_CH0F_SHIFT                    (0U)
#define FTM_STATUS_CH0F_WIDTH                    (1U)
#define FTM_STATUS_CH0F(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_STATUS_CH0F_SHIFT)) & FTM_STATUS_CH0F_MASK)
#define FTM_STATUS_CH1F_MASK                     (0x2U)
#define FTM_STATUS_CH1F_SHIFT                    (1U)
#define FTM_STATUS_CH1F_WIDTH                    (1U)
#define FTM_STATUS_CH1F(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_STATUS_CH1F_SHIFT)) & FTM_STATUS_CH1F_MASK)
#define FTM_STATUS_CH2F_MASK                     (0x4U)
#define FTM_STATUS_CH2F_SHIFT                    (2U)
#define FTM_STATUS_CH2F_WIDTH                    (1U)
#define FTM_STATUS_CH2F(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_STATUS_CH2F_SHIFT)) & FTM_STATUS_CH2F_MASK)
#define FTM_STATUS_CH3F_MASK                     (0x8U)
#define FTM_STATUS_CH3F_SHIFT                    (3U)
#define FTM_STATUS_CH3F_WIDTH                    (1U)
#define FTM_STATUS_CH3F(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_STATUS_CH3F_SHIFT)) & FTM_STATUS_CH3F_MASK)
#define FTM_STATUS_CH4F_MASK                     (0x10U)
#define FTM_STATUS_CH4F_SHIFT                    (4U)
#define FTM_STATUS_CH4F_WIDTH                    (1U)
#define FTM_STATUS_CH4F(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_STATUS_CH4F_SHIFT)) & FTM_STATUS_CH4F_MASK)
#define FTM_STATUS_CH5F_MASK                     (0x20U)
#define FTM_STATUS_CH5F_SHIFT                    (5U)
#define FTM_STATUS_CH5F_WIDTH                    (1U)
#define FTM_STATUS_CH5F(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_STATUS_CH5F_SHIFT)) & FTM_STATUS_CH5F_MASK)
#define FTM_STATUS_CH6F_MASK                     (0x40U)
#define FTM_STATUS_CH6F_SHIFT                    (6U)
#define FTM_STATUS_CH6F_WIDTH                    (1U)
#define FTM_STATUS_CH6F(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_STATUS_CH6F_SHIFT)) & FTM_STATUS_CH6F_MASK)
#define FTM_STATUS_CH7F_MASK                     (0x80U)
#define FTM_STATUS_CH7F_SHIFT                    (7U)
#define FTM_STATUS_CH7F_WIDTH                    (1U)
#define FTM_STATUS_CH7F(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_STATUS_CH7F_SHIFT)) & FTM_STATUS_CH7F_MASK)
/*! @} */

/*! @name MODE - Features Mode Selection */
/*! @{ */
#define FTM_MODE_FTMEN_MASK                      (0x1U)
#define FTM_MODE_FTMEN_SHIFT                     (0U)
#define FTM_MODE_FTMEN_WIDTH                     (1U)
#define FTM_MODE_FTMEN(x)                        (((uint32_t)(((uint32_t)(x)) << FTM_MODE_FTMEN_SHIFT)) & FTM_MODE_FTMEN_MASK)
#define FTM_MODE_INIT_MASK                       (0x2U)
#define FTM_MODE_INIT_SHIFT                      (1U)
#define FTM_MODE_INIT_WIDTH                      (1U)
#define FTM_MODE_INIT(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_MODE_INIT_SHIFT)) & FTM_MODE_INIT_MASK)
#define FTM_MODE_WPDIS_MASK                      (0x4U)
#define FTM_MODE_WPDIS_SHIFT                     (2U)
#define FTM_MODE_WPDIS_WIDTH                     (1U)
#define FTM_MODE_WPDIS(x)                        (((uint32_t)(((uint32_t)(x)) << FTM_MODE_WPDIS_SHIFT)) & FTM_MODE_WPDIS_MASK)
#define FTM_MODE_PWMSYNC_MASK                    (0x8U)
#define FTM_MODE_PWMSYNC_SHIFT                   (3U)
#define FTM_MODE_PWMSYNC_WIDTH                   (1U)
#define FTM_MODE_PWMSYNC(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_MODE_PWMSYNC_SHIFT)) & FTM_MODE_PWMSYNC_MASK)
#define FTM_MODE_CAPTEST_MASK                    (0x10U)
#define FTM_MODE_CAPTEST_SHIFT                   (4U)
#define FTM_MODE_CAPTEST_WIDTH                   (1U)
#define FTM_MODE_CAPTEST(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_MODE_CAPTEST_SHIFT)) & FTM_MODE_CAPTEST_MASK)
#define FTM_MODE_FAULTM_MASK                     (0x60U)
#define FTM_MODE_FAULTM_SHIFT                    (5U)
#define FTM_MODE_FAULTM_WIDTH                    (2U)
#define FTM_MODE_FAULTM(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_MODE_FAULTM_SHIFT)) & FTM_MODE_FAULTM_MASK)
#define FTM_MODE_FAULTIE_MASK                    (0x80U)
#define FTM_MODE_FAULTIE_SHIFT                   (7U)
#define FTM_MODE_FAULTIE_WIDTH                   (1U)
#define FTM_MODE_FAULTIE(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_MODE_FAULTIE_SHIFT)) & FTM_MODE_FAULTIE_MASK)
/*! @} */

/*! @name SYNC - Synchronization */
/*! @{ */
#define FTM_SYNC_CNTMIN_MASK                     (0x1U)
#define FTM_SYNC_CNTMIN_SHIFT                    (0U)
#define FTM_SYNC_CNTMIN_WIDTH                    (1U)
#define FTM_SYNC_CNTMIN(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_SYNC_CNTMIN_SHIFT)) & FTM_SYNC_CNTMIN_MASK)
#define FTM_SYNC_CNTMAX_MASK                     (0x2U)
#define FTM_SYNC_CNTMAX_SHIFT                    (1U)
#define FTM_SYNC_CNTMAX_WIDTH                    (1U)
#define FTM_SYNC_CNTMAX(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_SYNC_CNTMAX_SHIFT)) & FTM_SYNC_CNTMAX_MASK)
#define FTM_SYNC_REINIT_MASK                     (0x4U)
#define FTM_SYNC_REINIT_SHIFT                    (2U)
#define FTM_SYNC_REINIT_WIDTH                    (1U)
#define FTM_SYNC_REINIT(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_SYNC_REINIT_SHIFT)) & FTM_SYNC_REINIT_MASK)
#define FTM_SYNC_SYNCHOM_MASK                    (0x8U)
#define FTM_SYNC_SYNCHOM_SHIFT                   (3U)
#define FTM_SYNC_SYNCHOM_WIDTH                   (1U)
#define FTM_SYNC_SYNCHOM(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_SYNC_SYNCHOM_SHIFT)) & FTM_SYNC_SYNCHOM_MASK)
#define FTM_SYNC_TRIG0_MASK                      (0x10U)
#define FTM_SYNC_TRIG0_SHIFT                     (4U)
#define FTM_SYNC_TRIG0_WIDTH                     (1U)
#define FTM_SYNC_TRIG0(x)                        (((uint32_t)(((uint32_t)(x)) << FTM_SYNC_TRIG0_SHIFT)) & FTM_SYNC_TRIG0_MASK)
#define FTM_SYNC_TRIG1_MASK                      (0x20U)
#define FTM_SYNC_TRIG1_SHIFT                     (5U)
#define FTM_SYNC_TRIG1_WIDTH                     (1U)
#define FTM_SYNC_TRIG1(x)                        (((uint32_t)(((uint32_t)(x)) << FTM_SYNC_TRIG1_SHIFT)) & FTM_SYNC_TRIG1_MASK)
#define FTM_SYNC_TRIG2_MASK                      (0x40U)
#define FTM_SYNC_TRIG2_SHIFT                     (6U)
#define FTM_SYNC_TRIG2_WIDTH                     (1U)
#define FTM_SYNC_TRIG2(x)                        (((uint32_t)(((uint32_t)(x)) << FTM_SYNC_TRIG2_SHIFT)) & FTM_SYNC_TRIG2_MASK)
#define FTM_SYNC_SWSYNC_MASK                     (0x80U)
#define FTM_SYNC_SWSYNC_SHIFT                    (7U)
#define FTM_SYNC_SWSYNC_WIDTH                    (1U)
#define FTM_SYNC_SWSYNC(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_SYNC_SWSYNC_SHIFT)) & FTM_SYNC_SWSYNC_MASK)
/*! @} */

/*! @name OUTINIT - Initial State For Channels Output */
/*! @{ */
#define FTM_OUTINIT_CH0OI_MASK                   (0x1U)
#define FTM_OUTINIT_CH0OI_SHIFT                  (0U)
#define FTM_OUTINIT_CH0OI_WIDTH                  (1U)
#define FTM_OUTINIT_CH0OI(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTINIT_CH0OI_SHIFT)) & FTM_OUTINIT_CH0OI_MASK)
#define FTM_OUTINIT_CH1OI_MASK                   (0x2U)
#define FTM_OUTINIT_CH1OI_SHIFT                  (1U)
#define FTM_OUTINIT_CH1OI_WIDTH                  (1U)
#define FTM_OUTINIT_CH1OI(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTINIT_CH1OI_SHIFT)) & FTM_OUTINIT_CH1OI_MASK)
#define FTM_OUTINIT_CH2OI_MASK                   (0x4U)
#define FTM_OUTINIT_CH2OI_SHIFT                  (2U)
#define FTM_OUTINIT_CH2OI_WIDTH                  (1U)
#define FTM_OUTINIT_CH2OI(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTINIT_CH2OI_SHIFT)) & FTM_OUTINIT_CH2OI_MASK)
#define FTM_OUTINIT_CH3OI_MASK                   (0x8U)
#define FTM_OUTINIT_CH3OI_SHIFT                  (3U)
#define FTM_OUTINIT_CH3OI_WIDTH                  (1U)
#define FTM_OUTINIT_CH3OI(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTINIT_CH3OI_SHIFT)) & FTM_OUTINIT_CH3OI_MASK)
#define FTM_OUTINIT_CH4OI_MASK                   (0x10U)
#define FTM_OUTINIT_CH4OI_SHIFT                  (4U)
#define FTM_OUTINIT_CH4OI_WIDTH                  (1U)
#define FTM_OUTINIT_CH4OI(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTINIT_CH4OI_SHIFT)) & FTM_OUTINIT_CH4OI_MASK)
#define FTM_OUTINIT_CH5OI_MASK                   (0x20U)
#define FTM_OUTINIT_CH5OI_SHIFT                  (5U)
#define FTM_OUTINIT_CH5OI_WIDTH                  (1U)
#define FTM_OUTINIT_CH5OI(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTINIT_CH5OI_SHIFT)) & FTM_OUTINIT_CH5OI_MASK)
#define FTM_OUTINIT_CH6OI_MASK                   (0x40U)
#define FTM_OUTINIT_CH6OI_SHIFT                  (6U)
#define FTM_OUTINIT_CH6OI_WIDTH                  (1U)
#define FTM_OUTINIT_CH6OI(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTINIT_CH6OI_SHIFT)) & FTM_OUTINIT_CH6OI_MASK)
#define FTM_OUTINIT_CH7OI_MASK                   (0x80U)
#define FTM_OUTINIT_CH7OI_SHIFT                  (7U)
#define FTM_OUTINIT_CH7OI_WIDTH                  (1U)
#define FTM_OUTINIT_CH7OI(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTINIT_CH7OI_SHIFT)) & FTM_OUTINIT_CH7OI_MASK)
/*! @} */

/*! @name OUTMASK - Output Mask */
/*! @{ */
#define FTM_OUTMASK_CH0OM_MASK                   (0x1U)
#define FTM_OUTMASK_CH0OM_SHIFT                  (0U)
#define FTM_OUTMASK_CH0OM_WIDTH                  (1U)
#define FTM_OUTMASK_CH0OM(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTMASK_CH0OM_SHIFT)) & FTM_OUTMASK_CH0OM_MASK)
#define FTM_OUTMASK_CH1OM_MASK                   (0x2U)
#define FTM_OUTMASK_CH1OM_SHIFT                  (1U)
#define FTM_OUTMASK_CH1OM_WIDTH                  (1U)
#define FTM_OUTMASK_CH1OM(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTMASK_CH1OM_SHIFT)) & FTM_OUTMASK_CH1OM_MASK)
#define FTM_OUTMASK_CH2OM_MASK                   (0x4U)
#define FTM_OUTMASK_CH2OM_SHIFT                  (2U)
#define FTM_OUTMASK_CH2OM_WIDTH                  (1U)
#define FTM_OUTMASK_CH2OM(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTMASK_CH2OM_SHIFT)) & FTM_OUTMASK_CH2OM_MASK)
#define FTM_OUTMASK_CH3OM_MASK                   (0x8U)
#define FTM_OUTMASK_CH3OM_SHIFT                  (3U)
#define FTM_OUTMASK_CH3OM_WIDTH                  (1U)
#define FTM_OUTMASK_CH3OM(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTMASK_CH3OM_SHIFT)) & FTM_OUTMASK_CH3OM_MASK)
#define FTM_OUTMASK_CH4OM_MASK                   (0x10U)
#define FTM_OUTMASK_CH4OM_SHIFT                  (4U)
#define FTM_OUTMASK_CH4OM_WIDTH                  (1U)
#define FTM_OUTMASK_CH4OM(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTMASK_CH4OM_SHIFT)) & FTM_OUTMASK_CH4OM_MASK)
#define FTM_OUTMASK_CH5OM_MASK                   (0x20U)
#define FTM_OUTMASK_CH5OM_SHIFT                  (5U)
#define FTM_OUTMASK_CH5OM_WIDTH                  (1U)
#define FTM_OUTMASK_CH5OM(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTMASK_CH5OM_SHIFT)) & FTM_OUTMASK_CH5OM_MASK)
#define FTM_OUTMASK_CH6OM_MASK                   (0x40U)
#define FTM_OUTMASK_CH6OM_SHIFT                  (6U)
#define FTM_OUTMASK_CH6OM_WIDTH                  (1U)
#define FTM_OUTMASK_CH6OM(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTMASK_CH6OM_SHIFT)) & FTM_OUTMASK_CH6OM_MASK)
#define FTM_OUTMASK_CH7OM_MASK                   (0x80U)
#define FTM_OUTMASK_CH7OM_SHIFT                  (7U)
#define FTM_OUTMASK_CH7OM_WIDTH                  (1U)
#define FTM_OUTMASK_CH7OM(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTMASK_CH7OM_SHIFT)) & FTM_OUTMASK_CH7OM_MASK)
/*! @} */

/*! @name COMBINE - Function For Linked Channels */
/*! @{ */
#define FTM_COMBINE_COMBINE0_MASK                (0x1U)
#define FTM_COMBINE_COMBINE0_SHIFT               (0U)
#define FTM_COMBINE_COMBINE0_WIDTH               (1U)
#define FTM_COMBINE_COMBINE0(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_COMBINE0_SHIFT)) & FTM_COMBINE_COMBINE0_MASK)
#define FTM_COMBINE_COMP0_MASK                   (0x2U)
#define FTM_COMBINE_COMP0_SHIFT                  (1U)
#define FTM_COMBINE_COMP0_WIDTH                  (1U)
#define FTM_COMBINE_COMP0(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_COMP0_SHIFT)) & FTM_COMBINE_COMP0_MASK)
#define FTM_COMBINE_DECAPEN0_MASK                (0x4U)
#define FTM_COMBINE_DECAPEN0_SHIFT               (2U)
#define FTM_COMBINE_DECAPEN0_WIDTH               (1U)
#define FTM_COMBINE_DECAPEN0(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DECAPEN0_SHIFT)) & FTM_COMBINE_DECAPEN0_MASK)
#define FTM_COMBINE_DECAP0_MASK                  (0x8U)
#define FTM_COMBINE_DECAP0_SHIFT                 (3U)
#define FTM_COMBINE_DECAP0_WIDTH                 (1U)
#define FTM_COMBINE_DECAP0(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DECAP0_SHIFT)) & FTM_COMBINE_DECAP0_MASK)
#define FTM_COMBINE_DTEN0_MASK                   (0x10U)
#define FTM_COMBINE_DTEN0_SHIFT                  (4U)
#define FTM_COMBINE_DTEN0_WIDTH                  (1U)
#define FTM_COMBINE_DTEN0(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DTEN0_SHIFT)) & FTM_COMBINE_DTEN0_MASK)
#define FTM_COMBINE_SYNCEN0_MASK                 (0x20U)
#define FTM_COMBINE_SYNCEN0_SHIFT                (5U)
#define FTM_COMBINE_SYNCEN0_WIDTH                (1U)
#define FTM_COMBINE_SYNCEN0(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_SYNCEN0_SHIFT)) & FTM_COMBINE_SYNCEN0_MASK)
#define FTM_COMBINE_FAULTEN0_MASK                (0x40U)
#define FTM_COMBINE_FAULTEN0_SHIFT               (6U)
#define FTM_COMBINE_FAULTEN0_WIDTH               (1U)
#define FTM_COMBINE_FAULTEN0(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_FAULTEN0_SHIFT)) & FTM_COMBINE_FAULTEN0_MASK)
#define FTM_COMBINE_MCOMBINE0_MASK               (0x80U)
#define FTM_COMBINE_MCOMBINE0_SHIFT              (7U)
#define FTM_COMBINE_MCOMBINE0_WIDTH              (1U)
#define FTM_COMBINE_MCOMBINE0(x)                 (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_MCOMBINE0_SHIFT)) & FTM_COMBINE_MCOMBINE0_MASK)
#define FTM_COMBINE_COMBINE1_MASK                (0x100U)
#define FTM_COMBINE_COMBINE1_SHIFT               (8U)
#define FTM_COMBINE_COMBINE1_WIDTH               (1U)
#define FTM_COMBINE_COMBINE1(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_COMBINE1_SHIFT)) & FTM_COMBINE_COMBINE1_MASK)
#define FTM_COMBINE_COMP1_MASK                   (0x200U)
#define FTM_COMBINE_COMP1_SHIFT                  (9U)
#define FTM_COMBINE_COMP1_WIDTH                  (1U)
#define FTM_COMBINE_COMP1(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_COMP1_SHIFT)) & FTM_COMBINE_COMP1_MASK)
#define FTM_COMBINE_DECAPEN1_MASK                (0x400U)
#define FTM_COMBINE_DECAPEN1_SHIFT               (10U)
#define FTM_COMBINE_DECAPEN1_WIDTH               (1U)
#define FTM_COMBINE_DECAPEN1(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DECAPEN1_SHIFT)) & FTM_COMBINE_DECAPEN1_MASK)
#define FTM_COMBINE_DECAP1_MASK                  (0x800U)
#define FTM_COMBINE_DECAP1_SHIFT                 (11U)
#define FTM_COMBINE_DECAP1_WIDTH                 (1U)
#define FTM_COMBINE_DECAP1(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DECAP1_SHIFT)) & FTM_COMBINE_DECAP1_MASK)
#define FTM_COMBINE_DTEN1_MASK                   (0x1000U)
#define FTM_COMBINE_DTEN1_SHIFT                  (12U)
#define FTM_COMBINE_DTEN1_WIDTH                  (1U)
#define FTM_COMBINE_DTEN1(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DTEN1_SHIFT)) & FTM_COMBINE_DTEN1_MASK)
#define FTM_COMBINE_SYNCEN1_MASK                 (0x2000U)
#define FTM_COMBINE_SYNCEN1_SHIFT                (13U)
#define FTM_COMBINE_SYNCEN1_WIDTH                (1U)
#define FTM_COMBINE_SYNCEN1(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_SYNCEN1_SHIFT)) & FTM_COMBINE_SYNCEN1_MASK)
#define FTM_COMBINE_FAULTEN1_MASK                (0x4000U)
#define FTM_COMBINE_FAULTEN1_SHIFT               (14U)
#define FTM_COMBINE_FAULTEN1_WIDTH               (1U)
#define FTM_COMBINE_FAULTEN1(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_FAULTEN1_SHIFT)) & FTM_COMBINE_FAULTEN1_MASK)
#define FTM_COMBINE_MCOMBINE1_MASK               (0x8000U)
#define FTM_COMBINE_MCOMBINE1_SHIFT              (15U)
#define FTM_COMBINE_MCOMBINE1_WIDTH              (1U)
#define FTM_COMBINE_MCOMBINE1(x)                 (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_MCOMBINE1_SHIFT)) & FTM_COMBINE_MCOMBINE1_MASK)
#define FTM_COMBINE_COMBINE2_MASK                (0x10000U)
#define FTM_COMBINE_COMBINE2_SHIFT               (16U)
#define FTM_COMBINE_COMBINE2_WIDTH               (1U)
#define FTM_COMBINE_COMBINE2(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_COMBINE2_SHIFT)) & FTM_COMBINE_COMBINE2_MASK)
#define FTM_COMBINE_COMP2_MASK                   (0x20000U)
#define FTM_COMBINE_COMP2_SHIFT                  (17U)
#define FTM_COMBINE_COMP2_WIDTH                  (1U)
#define FTM_COMBINE_COMP2(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_COMP2_SHIFT)) & FTM_COMBINE_COMP2_MASK)
#define FTM_COMBINE_DECAPEN2_MASK                (0x40000U)
#define FTM_COMBINE_DECAPEN2_SHIFT               (18U)
#define FTM_COMBINE_DECAPEN2_WIDTH               (1U)
#define FTM_COMBINE_DECAPEN2(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DECAPEN2_SHIFT)) & FTM_COMBINE_DECAPEN2_MASK)
#define FTM_COMBINE_DECAP2_MASK                  (0x80000U)
#define FTM_COMBINE_DECAP2_SHIFT                 (19U)
#define FTM_COMBINE_DECAP2_WIDTH                 (1U)
#define FTM_COMBINE_DECAP2(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DECAP2_SHIFT)) & FTM_COMBINE_DECAP2_MASK)
#define FTM_COMBINE_DTEN2_MASK                   (0x100000U)
#define FTM_COMBINE_DTEN2_SHIFT                  (20U)
#define FTM_COMBINE_DTEN2_WIDTH                  (1U)
#define FTM_COMBINE_DTEN2(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DTEN2_SHIFT)) & FTM_COMBINE_DTEN2_MASK)
#define FTM_COMBINE_SYNCEN2_MASK                 (0x200000U)
#define FTM_COMBINE_SYNCEN2_SHIFT                (21U)
#define FTM_COMBINE_SYNCEN2_WIDTH                (1U)
#define FTM_COMBINE_SYNCEN2(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_SYNCEN2_SHIFT)) & FTM_COMBINE_SYNCEN2_MASK)
#define FTM_COMBINE_FAULTEN2_MASK                (0x400000U)
#define FTM_COMBINE_FAULTEN2_SHIFT               (22U)
#define FTM_COMBINE_FAULTEN2_WIDTH               (1U)
#define FTM_COMBINE_FAULTEN2(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_FAULTEN2_SHIFT)) & FTM_COMBINE_FAULTEN2_MASK)
#define FTM_COMBINE_MCOMBINE2_MASK               (0x800000U)
#define FTM_COMBINE_MCOMBINE2_SHIFT              (23U)
#define FTM_COMBINE_MCOMBINE2_WIDTH              (1U)
#define FTM_COMBINE_MCOMBINE2(x)                 (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_MCOMBINE2_SHIFT)) & FTM_COMBINE_MCOMBINE2_MASK)
#define FTM_COMBINE_COMBINE3_MASK                (0x1000000U)
#define FTM_COMBINE_COMBINE3_SHIFT               (24U)
#define FTM_COMBINE_COMBINE3_WIDTH               (1U)
#define FTM_COMBINE_COMBINE3(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_COMBINE3_SHIFT)) & FTM_COMBINE_COMBINE3_MASK)
#define FTM_COMBINE_COMP3_MASK                   (0x2000000U)
#define FTM_COMBINE_COMP3_SHIFT                  (25U)
#define FTM_COMBINE_COMP3_WIDTH                  (1U)
#define FTM_COMBINE_COMP3(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_COMP3_SHIFT)) & FTM_COMBINE_COMP3_MASK)
#define FTM_COMBINE_DECAPEN3_MASK                (0x4000000U)
#define FTM_COMBINE_DECAPEN3_SHIFT               (26U)
#define FTM_COMBINE_DECAPEN3_WIDTH               (1U)
#define FTM_COMBINE_DECAPEN3(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DECAPEN3_SHIFT)) & FTM_COMBINE_DECAPEN3_MASK)
#define FTM_COMBINE_DECAP3_MASK                  (0x8000000U)
#define FTM_COMBINE_DECAP3_SHIFT                 (27U)
#define FTM_COMBINE_DECAP3_WIDTH                 (1U)
#define FTM_COMBINE_DECAP3(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DECAP3_SHIFT)) & FTM_COMBINE_DECAP3_MASK)
#define FTM_COMBINE_DTEN3_MASK                   (0x10000000U)
#define FTM_COMBINE_DTEN3_SHIFT                  (28U)
#define FTM_COMBINE_DTEN3_WIDTH                  (1U)
#define FTM_COMBINE_DTEN3(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DTEN3_SHIFT)) & FTM_COMBINE_DTEN3_MASK)
#define FTM_COMBINE_SYNCEN3_MASK                 (0x20000000U)
#define FTM_COMBINE_SYNCEN3_SHIFT                (29U)
#define FTM_COMBINE_SYNCEN3_WIDTH                (1U)
#define FTM_COMBINE_SYNCEN3(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_SYNCEN3_SHIFT)) & FTM_COMBINE_SYNCEN3_MASK)
#define FTM_COMBINE_FAULTEN3_MASK                (0x40000000U)
#define FTM_COMBINE_FAULTEN3_SHIFT               (30U)
#define FTM_COMBINE_FAULTEN3_WIDTH               (1U)
#define FTM_COMBINE_FAULTEN3(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_FAULTEN3_SHIFT)) & FTM_COMBINE_FAULTEN3_MASK)
#define FTM_COMBINE_MCOMBINE3_MASK               (0x80000000U)
#define FTM_COMBINE_MCOMBINE3_SHIFT              (31U)
#define FTM_COMBINE_MCOMBINE3_WIDTH              (1U)
#define FTM_COMBINE_MCOMBINE3(x)                 (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_MCOMBINE3_SHIFT)) & FTM_COMBINE_MCOMBINE3_MASK)
/*! @} */

/*! @name DEADTIME - Deadtime Configuration */
/*! @{ */
#define FTM_DEADTIME_DTVAL_MASK                  (0x3FU)
#define FTM_DEADTIME_DTVAL_SHIFT                 (0U)
#define FTM_DEADTIME_DTVAL_WIDTH                 (6U)
#define FTM_DEADTIME_DTVAL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_DEADTIME_DTVAL_SHIFT)) & FTM_DEADTIME_DTVAL_MASK)
#define FTM_DEADTIME_DTPS_MASK                   (0xC0U)
#define FTM_DEADTIME_DTPS_SHIFT                  (6U)
#define FTM_DEADTIME_DTPS_WIDTH                  (2U)
#define FTM_DEADTIME_DTPS(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_DEADTIME_DTPS_SHIFT)) & FTM_DEADTIME_DTPS_MASK)
#define FTM_DEADTIME_DTVALEX_MASK                (0xF0000U)
#define FTM_DEADTIME_DTVALEX_SHIFT               (16U)
#define FTM_DEADTIME_DTVALEX_WIDTH               (4U)
#define FTM_DEADTIME_DTVALEX(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_DEADTIME_DTVALEX_SHIFT)) & FTM_DEADTIME_DTVALEX_MASK)
/*! @} */

/*! @name EXTTRIG - FTM External Trigger */
/*! @{ */
#define FTM_EXTTRIG_CH2TRIG_MASK                 (0x1U)
#define FTM_EXTTRIG_CH2TRIG_SHIFT                (0U)
#define FTM_EXTTRIG_CH2TRIG_WIDTH                (1U)
#define FTM_EXTTRIG_CH2TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_CH2TRIG_SHIFT)) & FTM_EXTTRIG_CH2TRIG_MASK)
#define FTM_EXTTRIG_CH3TRIG_MASK                 (0x2U)
#define FTM_EXTTRIG_CH3TRIG_SHIFT                (1U)
#define FTM_EXTTRIG_CH3TRIG_WIDTH                (1U)
#define FTM_EXTTRIG_CH3TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_CH3TRIG_SHIFT)) & FTM_EXTTRIG_CH3TRIG_MASK)
#define FTM_EXTTRIG_CH4TRIG_MASK                 (0x4U)
#define FTM_EXTTRIG_CH4TRIG_SHIFT                (2U)
#define FTM_EXTTRIG_CH4TRIG_WIDTH                (1U)
#define FTM_EXTTRIG_CH4TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_CH4TRIG_SHIFT)) & FTM_EXTTRIG_CH4TRIG_MASK)
#define FTM_EXTTRIG_CH5TRIG_MASK                 (0x8U)
#define FTM_EXTTRIG_CH5TRIG_SHIFT                (3U)
#define FTM_EXTTRIG_CH5TRIG_WIDTH                (1U)
#define FTM_EXTTRIG_CH5TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_CH5TRIG_SHIFT)) & FTM_EXTTRIG_CH5TRIG_MASK)
#define FTM_EXTTRIG_CH0TRIG_MASK                 (0x10U)
#define FTM_EXTTRIG_CH0TRIG_SHIFT                (4U)
#define FTM_EXTTRIG_CH0TRIG_WIDTH                (1U)
#define FTM_EXTTRIG_CH0TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_CH0TRIG_SHIFT)) & FTM_EXTTRIG_CH0TRIG_MASK)
#define FTM_EXTTRIG_CH1TRIG_MASK                 (0x20U)
#define FTM_EXTTRIG_CH1TRIG_SHIFT                (5U)
#define FTM_EXTTRIG_CH1TRIG_WIDTH                (1U)
#define FTM_EXTTRIG_CH1TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_CH1TRIG_SHIFT)) & FTM_EXTTRIG_CH1TRIG_MASK)
#define FTM_EXTTRIG_INITTRIGEN_MASK              (0x40U)
#define FTM_EXTTRIG_INITTRIGEN_SHIFT             (6U)
#define FTM_EXTTRIG_INITTRIGEN_WIDTH             (1U)
#define FTM_EXTTRIG_INITTRIGEN(x)                (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_INITTRIGEN_SHIFT)) & FTM_EXTTRIG_INITTRIGEN_MASK)
#define FTM_EXTTRIG_TRIGF_MASK                   (0x80U)
#define FTM_EXTTRIG_TRIGF_SHIFT                  (7U)
#define FTM_EXTTRIG_TRIGF_WIDTH                  (1U)
#define FTM_EXTTRIG_TRIGF(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_TRIGF_SHIFT)) & FTM_EXTTRIG_TRIGF_MASK)
#define FTM_EXTTRIG_CH6TRIG_MASK                 (0x100U)
#define FTM_EXTTRIG_CH6TRIG_SHIFT                (8U)
#define FTM_EXTTRIG_CH6TRIG_WIDTH                (1U)
#define FTM_EXTTRIG_CH6TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_CH6TRIG_SHIFT)) & FTM_EXTTRIG_CH6TRIG_MASK)
#define FTM_EXTTRIG_CH7TRIG_MASK                 (0x200U)
#define FTM_EXTTRIG_CH7TRIG_SHIFT                (9U)
#define FTM_EXTTRIG_CH7TRIG_WIDTH                (1U)
#define FTM_EXTTRIG_CH7TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_CH7TRIG_SHIFT)) & FTM_EXTTRIG_CH7TRIG_MASK)
/*! @} */

/*! @name POL - Channels Polarity */
/*! @{ */
#define FTM_POL_POL0_MASK                        (0x1U)
#define FTM_POL_POL0_SHIFT                       (0U)
#define FTM_POL_POL0_WIDTH                       (1U)
#define FTM_POL_POL0(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_POL_POL0_SHIFT)) & FTM_POL_POL0_MASK)
#define FTM_POL_POL1_MASK                        (0x2U)
#define FTM_POL_POL1_SHIFT                       (1U)
#define FTM_POL_POL1_WIDTH                       (1U)
#define FTM_POL_POL1(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_POL_POL1_SHIFT)) & FTM_POL_POL1_MASK)
#define FTM_POL_POL2_MASK                        (0x4U)
#define FTM_POL_POL2_SHIFT                       (2U)
#define FTM_POL_POL2_WIDTH                       (1U)
#define FTM_POL_POL2(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_POL_POL2_SHIFT)) & FTM_POL_POL2_MASK)
#define FTM_POL_POL3_MASK                        (0x8U)
#define FTM_POL_POL3_SHIFT                       (3U)
#define FTM_POL_POL3_WIDTH                       (1U)
#define FTM_POL_POL3(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_POL_POL3_SHIFT)) & FTM_POL_POL3_MASK)
#define FTM_POL_POL4_MASK                        (0x10U)
#define FTM_POL_POL4_SHIFT                       (4U)
#define FTM_POL_POL4_WIDTH                       (1U)
#define FTM_POL_POL4(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_POL_POL4_SHIFT)) & FTM_POL_POL4_MASK)
#define FTM_POL_POL5_MASK                        (0x20U)
#define FTM_POL_POL5_SHIFT                       (5U)
#define FTM_POL_POL5_WIDTH                       (1U)
#define FTM_POL_POL5(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_POL_POL5_SHIFT)) & FTM_POL_POL5_MASK)
#define FTM_POL_POL6_MASK                        (0x40U)
#define FTM_POL_POL6_SHIFT                       (6U)
#define FTM_POL_POL6_WIDTH                       (1U)
#define FTM_POL_POL6(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_POL_POL6_SHIFT)) & FTM_POL_POL6_MASK)
#define FTM_POL_POL7_MASK                        (0x80U)
#define FTM_POL_POL7_SHIFT                       (7U)
#define FTM_POL_POL7_WIDTH                       (1U)
#define FTM_POL_POL7(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_POL_POL7_SHIFT)) & FTM_POL_POL7_MASK)
/*! @} */

/*! @name FMS - Fault Mode Status */
/*! @{ */
#define FTM_FMS_FAULTF0_MASK                     (0x1U)
#define FTM_FMS_FAULTF0_SHIFT                    (0U)
#define FTM_FMS_FAULTF0_WIDTH                    (1U)
#define FTM_FMS_FAULTF0(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_FMS_FAULTF0_SHIFT)) & FTM_FMS_FAULTF0_MASK)
#define FTM_FMS_FAULTF1_MASK                     (0x2U)
#define FTM_FMS_FAULTF1_SHIFT                    (1U)
#define FTM_FMS_FAULTF1_WIDTH                    (1U)
#define FTM_FMS_FAULTF1(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_FMS_FAULTF1_SHIFT)) & FTM_FMS_FAULTF1_MASK)
#define FTM_FMS_FAULTF2_MASK                     (0x4U)
#define FTM_FMS_FAULTF2_SHIFT                    (2U)
#define FTM_FMS_FAULTF2_WIDTH                    (1U)
#define FTM_FMS_FAULTF2(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_FMS_FAULTF2_SHIFT)) & FTM_FMS_FAULTF2_MASK)
#define FTM_FMS_FAULTF3_MASK                     (0x8U)
#define FTM_FMS_FAULTF3_SHIFT                    (3U)
#define FTM_FMS_FAULTF3_WIDTH                    (1U)
#define FTM_FMS_FAULTF3(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_FMS_FAULTF3_SHIFT)) & FTM_FMS_FAULTF3_MASK)
#define FTM_FMS_FAULTIN_MASK                     (0x20U)
#define FTM_FMS_FAULTIN_SHIFT                    (5U)
#define FTM_FMS_FAULTIN_WIDTH                    (1U)
#define FTM_FMS_FAULTIN(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_FMS_FAULTIN_SHIFT)) & FTM_FMS_FAULTIN_MASK)
#define FTM_FMS_WPEN_MASK                        (0x40U)
#define FTM_FMS_WPEN_SHIFT                       (6U)
#define FTM_FMS_WPEN_WIDTH                       (1U)
#define FTM_FMS_WPEN(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_FMS_WPEN_SHIFT)) & FTM_FMS_WPEN_MASK)
#define FTM_FMS_FAULTF_MASK                      (0x80U)
#define FTM_FMS_FAULTF_SHIFT                     (7U)
#define FTM_FMS_FAULTF_WIDTH                     (1U)
#define FTM_FMS_FAULTF(x)                        (((uint32_t)(((uint32_t)(x)) << FTM_FMS_FAULTF_SHIFT)) & FTM_FMS_FAULTF_MASK)
/*! @} */

/*! @name FILTER - Input Capture Filter Control */
/*! @{ */
#define FTM_FILTER_CH0FVAL_MASK                  (0xFU)
#define FTM_FILTER_CH0FVAL_SHIFT                 (0U)
#define FTM_FILTER_CH0FVAL_WIDTH                 (4U)
#define FTM_FILTER_CH0FVAL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FILTER_CH0FVAL_SHIFT)) & FTM_FILTER_CH0FVAL_MASK)
#define FTM_FILTER_CH1FVAL_MASK                  (0xF0U)
#define FTM_FILTER_CH1FVAL_SHIFT                 (4U)
#define FTM_FILTER_CH1FVAL_WIDTH                 (4U)
#define FTM_FILTER_CH1FVAL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FILTER_CH1FVAL_SHIFT)) & FTM_FILTER_CH1FVAL_MASK)
#define FTM_FILTER_CH2FVAL_MASK                  (0xF00U)
#define FTM_FILTER_CH2FVAL_SHIFT                 (8U)
#define FTM_FILTER_CH2FVAL_WIDTH                 (4U)
#define FTM_FILTER_CH2FVAL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FILTER_CH2FVAL_SHIFT)) & FTM_FILTER_CH2FVAL_MASK)
#define FTM_FILTER_CH3FVAL_MASK                  (0xF000U)
#define FTM_FILTER_CH3FVAL_SHIFT                 (12U)
#define FTM_FILTER_CH3FVAL_WIDTH                 (4U)
#define FTM_FILTER_CH3FVAL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FILTER_CH3FVAL_SHIFT)) & FTM_FILTER_CH3FVAL_MASK)
/*! @} */

/*! @name FLTCTRL - Fault Control */
/*! @{ */
#define FTM_FLTCTRL_FAULT0EN_MASK                (0x1U)
#define FTM_FLTCTRL_FAULT0EN_SHIFT               (0U)
#define FTM_FLTCTRL_FAULT0EN_WIDTH               (1U)
#define FTM_FLTCTRL_FAULT0EN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FAULT0EN_SHIFT)) & FTM_FLTCTRL_FAULT0EN_MASK)
#define FTM_FLTCTRL_FAULT1EN_MASK                (0x2U)
#define FTM_FLTCTRL_FAULT1EN_SHIFT               (1U)
#define FTM_FLTCTRL_FAULT1EN_WIDTH               (1U)
#define FTM_FLTCTRL_FAULT1EN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FAULT1EN_SHIFT)) & FTM_FLTCTRL_FAULT1EN_MASK)
#define FTM_FLTCTRL_FAULT2EN_MASK                (0x4U)
#define FTM_FLTCTRL_FAULT2EN_SHIFT               (2U)
#define FTM_FLTCTRL_FAULT2EN_WIDTH               (1U)
#define FTM_FLTCTRL_FAULT2EN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FAULT2EN_SHIFT)) & FTM_FLTCTRL_FAULT2EN_MASK)
#define FTM_FLTCTRL_FAULT3EN_MASK                (0x8U)
#define FTM_FLTCTRL_FAULT3EN_SHIFT               (3U)
#define FTM_FLTCTRL_FAULT3EN_WIDTH               (1U)
#define FTM_FLTCTRL_FAULT3EN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FAULT3EN_SHIFT)) & FTM_FLTCTRL_FAULT3EN_MASK)
#define FTM_FLTCTRL_FFLTR0EN_MASK                (0x10U)
#define FTM_FLTCTRL_FFLTR0EN_SHIFT               (4U)
#define FTM_FLTCTRL_FFLTR0EN_WIDTH               (1U)
#define FTM_FLTCTRL_FFLTR0EN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FFLTR0EN_SHIFT)) & FTM_FLTCTRL_FFLTR0EN_MASK)
#define FTM_FLTCTRL_FFLTR1EN_MASK                (0x20U)
#define FTM_FLTCTRL_FFLTR1EN_SHIFT               (5U)
#define FTM_FLTCTRL_FFLTR1EN_WIDTH               (1U)
#define FTM_FLTCTRL_FFLTR1EN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FFLTR1EN_SHIFT)) & FTM_FLTCTRL_FFLTR1EN_MASK)
#define FTM_FLTCTRL_FFLTR2EN_MASK                (0x40U)
#define FTM_FLTCTRL_FFLTR2EN_SHIFT               (6U)
#define FTM_FLTCTRL_FFLTR2EN_WIDTH               (1U)
#define FTM_FLTCTRL_FFLTR2EN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FFLTR2EN_SHIFT)) & FTM_FLTCTRL_FFLTR2EN_MASK)
#define FTM_FLTCTRL_FFLTR3EN_MASK                (0x80U)
#define FTM_FLTCTRL_FFLTR3EN_SHIFT               (7U)
#define FTM_FLTCTRL_FFLTR3EN_WIDTH               (1U)
#define FTM_FLTCTRL_FFLTR3EN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FFLTR3EN_SHIFT)) & FTM_FLTCTRL_FFLTR3EN_MASK)
#define FTM_FLTCTRL_FFVAL_MASK                   (0xF00U)
#define FTM_FLTCTRL_FFVAL_SHIFT                  (8U)
#define FTM_FLTCTRL_FFVAL_WIDTH                  (4U)
#define FTM_FLTCTRL_FFVAL(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FFVAL_SHIFT)) & FTM_FLTCTRL_FFVAL_MASK)
#define FTM_FLTCTRL_FSTATE_MASK                  (0x8000U)
#define FTM_FLTCTRL_FSTATE_SHIFT                 (15U)
#define FTM_FLTCTRL_FSTATE_WIDTH                 (1U)
#define FTM_FLTCTRL_FSTATE(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FSTATE_SHIFT)) & FTM_FLTCTRL_FSTATE_MASK)
/*! @} */

/*! @name QDCTRL - Quadrature Decoder Control And Status */
/*! @{ */
#define FTM_QDCTRL_QUADEN_MASK                   (0x1U)
#define FTM_QDCTRL_QUADEN_SHIFT                  (0U)
#define FTM_QDCTRL_QUADEN_WIDTH                  (1U)
#define FTM_QDCTRL_QUADEN(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_QDCTRL_QUADEN_SHIFT)) & FTM_QDCTRL_QUADEN_MASK)
#define FTM_QDCTRL_TOFDIR_MASK                   (0x2U)
#define FTM_QDCTRL_TOFDIR_SHIFT                  (1U)
#define FTM_QDCTRL_TOFDIR_WIDTH                  (1U)
#define FTM_QDCTRL_TOFDIR(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_QDCTRL_TOFDIR_SHIFT)) & FTM_QDCTRL_TOFDIR_MASK)
#define FTM_QDCTRL_QUADIR_MASK                   (0x4U)
#define FTM_QDCTRL_QUADIR_SHIFT                  (2U)
#define FTM_QDCTRL_QUADIR_WIDTH                  (1U)
#define FTM_QDCTRL_QUADIR(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_QDCTRL_QUADIR_SHIFT)) & FTM_QDCTRL_QUADIR_MASK)
#define FTM_QDCTRL_QUADMODE_MASK                 (0x8U)
#define FTM_QDCTRL_QUADMODE_SHIFT                (3U)
#define FTM_QDCTRL_QUADMODE_WIDTH                (1U)
#define FTM_QDCTRL_QUADMODE(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_QDCTRL_QUADMODE_SHIFT)) & FTM_QDCTRL_QUADMODE_MASK)
#define FTM_QDCTRL_PHBPOL_MASK                   (0x10U)
#define FTM_QDCTRL_PHBPOL_SHIFT                  (4U)
#define FTM_QDCTRL_PHBPOL_WIDTH                  (1U)
#define FTM_QDCTRL_PHBPOL(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_QDCTRL_PHBPOL_SHIFT)) & FTM_QDCTRL_PHBPOL_MASK)
#define FTM_QDCTRL_PHAPOL_MASK                   (0x20U)
#define FTM_QDCTRL_PHAPOL_SHIFT                  (5U)
#define FTM_QDCTRL_PHAPOL_WIDTH                  (1U)
#define FTM_QDCTRL_PHAPOL(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_QDCTRL_PHAPOL_SHIFT)) & FTM_QDCTRL_PHAPOL_MASK)
#define FTM_QDCTRL_PHBFLTREN_MASK                (0x40U)
#define FTM_QDCTRL_PHBFLTREN_SHIFT               (6U)
#define FTM_QDCTRL_PHBFLTREN_WIDTH               (1U)
#define FTM_QDCTRL_PHBFLTREN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_QDCTRL_PHBFLTREN_SHIFT)) & FTM_QDCTRL_PHBFLTREN_MASK)
#define FTM_QDCTRL_PHAFLTREN_MASK                (0x80U)
#define FTM_QDCTRL_PHAFLTREN_SHIFT               (7U)
#define FTM_QDCTRL_PHAFLTREN_WIDTH               (1U)
#define FTM_QDCTRL_PHAFLTREN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_QDCTRL_PHAFLTREN_SHIFT)) & FTM_QDCTRL_PHAFLTREN_MASK)
/*! @} */

/*! @name CONF - Configuration */
/*! @{ */
#define FTM_CONF_LDFQ_MASK                       (0x1FU)
#define FTM_CONF_LDFQ_SHIFT                      (0U)
#define FTM_CONF_LDFQ_WIDTH                      (5U)
#define FTM_CONF_LDFQ(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_CONF_LDFQ_SHIFT)) & FTM_CONF_LDFQ_MASK)
#define FTM_CONF_BDMMODE_MASK                    (0xC0U)
#define FTM_CONF_BDMMODE_SHIFT                   (6U)
#define FTM_CONF_BDMMODE_WIDTH                   (2U)
#define FTM_CONF_BDMMODE(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_CONF_BDMMODE_SHIFT)) & FTM_CONF_BDMMODE_MASK)
#define FTM_CONF_GTBEEN_MASK                     (0x200U)
#define FTM_CONF_GTBEEN_SHIFT                    (9U)
#define FTM_CONF_GTBEEN_WIDTH                    (1U)
#define FTM_CONF_GTBEEN(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_CONF_GTBEEN_SHIFT)) & FTM_CONF_GTBEEN_MASK)
#define FTM_CONF_GTBEOUT_MASK                    (0x400U)
#define FTM_CONF_GTBEOUT_SHIFT                   (10U)
#define FTM_CONF_GTBEOUT_WIDTH                   (1U)
#define FTM_CONF_GTBEOUT(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_CONF_GTBEOUT_SHIFT)) & FTM_CONF_GTBEOUT_MASK)
#define FTM_CONF_ITRIGR_MASK                     (0x800U)
#define FTM_CONF_ITRIGR_SHIFT                    (11U)
#define FTM_CONF_ITRIGR_WIDTH                    (1U)
#define FTM_CONF_ITRIGR(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_CONF_ITRIGR_SHIFT)) & FTM_CONF_ITRIGR_MASK)
/*! @} */

/*! @name FLTPOL - FTM Fault Input Polarity */
/*! @{ */
#define FTM_FLTPOL_FLT0POL_MASK                  (0x1U)
#define FTM_FLTPOL_FLT0POL_SHIFT                 (0U)
#define FTM_FLTPOL_FLT0POL_WIDTH                 (1U)
#define FTM_FLTPOL_FLT0POL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FLTPOL_FLT0POL_SHIFT)) & FTM_FLTPOL_FLT0POL_MASK)
#define FTM_FLTPOL_FLT1POL_MASK                  (0x2U)
#define FTM_FLTPOL_FLT1POL_SHIFT                 (1U)
#define FTM_FLTPOL_FLT1POL_WIDTH                 (1U)
#define FTM_FLTPOL_FLT1POL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FLTPOL_FLT1POL_SHIFT)) & FTM_FLTPOL_FLT1POL_MASK)
#define FTM_FLTPOL_FLT2POL_MASK                  (0x4U)
#define FTM_FLTPOL_FLT2POL_SHIFT                 (2U)
#define FTM_FLTPOL_FLT2POL_WIDTH                 (1U)
#define FTM_FLTPOL_FLT2POL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FLTPOL_FLT2POL_SHIFT)) & FTM_FLTPOL_FLT2POL_MASK)
#define FTM_FLTPOL_FLT3POL_MASK                  (0x8U)
#define FTM_FLTPOL_FLT3POL_SHIFT                 (3U)
#define FTM_FLTPOL_FLT3POL_WIDTH                 (1U)
#define FTM_FLTPOL_FLT3POL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FLTPOL_FLT3POL_SHIFT)) & FTM_FLTPOL_FLT3POL_MASK)
/*! @} */

/*! @name SYNCONF - Synchronization Configuration */
/*! @{ */
#define FTM_SYNCONF_HWTRIGMODE_MASK              (0x1U)
#define FTM_SYNCONF_HWTRIGMODE_SHIFT             (0U)
#define FTM_SYNCONF_HWTRIGMODE_WIDTH             (1U)
#define FTM_SYNCONF_HWTRIGMODE(x)                (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_HWTRIGMODE_SHIFT)) & FTM_SYNCONF_HWTRIGMODE_MASK)
#define FTM_SYNCONF_CNTINC_MASK                  (0x4U)
#define FTM_SYNCONF_CNTINC_SHIFT                 (2U)
#define FTM_SYNCONF_CNTINC_WIDTH                 (1U)
#define FTM_SYNCONF_CNTINC(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_CNTINC_SHIFT)) & FTM_SYNCONF_CNTINC_MASK)
#define FTM_SYNCONF_INVC_MASK                    (0x10U)
#define FTM_SYNCONF_INVC_SHIFT                   (4U)
#define FTM_SYNCONF_INVC_WIDTH                   (1U)
#define FTM_SYNCONF_INVC(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_INVC_SHIFT)) & FTM_SYNCONF_INVC_MASK)
#define FTM_SYNCONF_SWOC_MASK                    (0x20U)
#define FTM_SYNCONF_SWOC_SHIFT                   (5U)
#define FTM_SYNCONF_SWOC_WIDTH                   (1U)
#define FTM_SYNCONF_SWOC(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_SWOC_SHIFT)) & FTM_SYNCONF_SWOC_MASK)
#define FTM_SYNCONF_SYNCMODE_MASK                (0x80U)
#define FTM_SYNCONF_SYNCMODE_SHIFT               (7U)
#define FTM_SYNCONF_SYNCMODE_WIDTH               (1U)
#define FTM_SYNCONF_SYNCMODE(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_SYNCMODE_SHIFT)) & FTM_SYNCONF_SYNCMODE_MASK)
#define FTM_SYNCONF_SWRSTCNT_MASK                (0x100U)
#define FTM_SYNCONF_SWRSTCNT_SHIFT               (8U)
#define FTM_SYNCONF_SWRSTCNT_WIDTH               (1U)
#define FTM_SYNCONF_SWRSTCNT(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_SWRSTCNT_SHIFT)) & FTM_SYNCONF_SWRSTCNT_MASK)
#define FTM_SYNCONF_SWWRBUF_MASK                 (0x200U)
#define FTM_SYNCONF_SWWRBUF_SHIFT                (9U)
#define FTM_SYNCONF_SWWRBUF_WIDTH                (1U)
#define FTM_SYNCONF_SWWRBUF(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_SWWRBUF_SHIFT)) & FTM_SYNCONF_SWWRBUF_MASK)
#define FTM_SYNCONF_SWOM_MASK                    (0x400U)
#define FTM_SYNCONF_SWOM_SHIFT                   (10U)
#define FTM_SYNCONF_SWOM_WIDTH                   (1U)
#define FTM_SYNCONF_SWOM(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_SWOM_SHIFT)) & FTM_SYNCONF_SWOM_MASK)
#define FTM_SYNCONF_SWINVC_MASK                  (0x800U)
#define FTM_SYNCONF_SWINVC_SHIFT                 (11U)
#define FTM_SYNCONF_SWINVC_WIDTH                 (1U)
#define FTM_SYNCONF_SWINVC(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_SWINVC_SHIFT)) & FTM_SYNCONF_SWINVC_MASK)
#define FTM_SYNCONF_SWSOC_MASK                   (0x1000U)
#define FTM_SYNCONF_SWSOC_SHIFT                  (12U)
#define FTM_SYNCONF_SWSOC_WIDTH                  (1U)
#define FTM_SYNCONF_SWSOC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_SWSOC_SHIFT)) & FTM_SYNCONF_SWSOC_MASK)
#define FTM_SYNCONF_HWRSTCNT_MASK                (0x10000U)
#define FTM_SYNCONF_HWRSTCNT_SHIFT               (16U)
#define FTM_SYNCONF_HWRSTCNT_WIDTH               (1U)
#define FTM_SYNCONF_HWRSTCNT(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_HWRSTCNT_SHIFT)) & FTM_SYNCONF_HWRSTCNT_MASK)
#define FTM_SYNCONF_HWWRBUF_MASK                 (0x20000U)
#define FTM_SYNCONF_HWWRBUF_SHIFT                (17U)
#define FTM_SYNCONF_HWWRBUF_WIDTH                (1U)
#define FTM_SYNCONF_HWWRBUF(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_HWWRBUF_SHIFT)) & FTM_SYNCONF_HWWRBUF_MASK)
#define FTM_SYNCONF_HWOM_MASK                    (0x40000U)
#define FTM_SYNCONF_HWOM_SHIFT                   (18U)
#define FTM_SYNCONF_HWOM_WIDTH                   (1U)
#define FTM_SYNCONF_HWOM(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_HWOM_SHIFT)) & FTM_SYNCONF_HWOM_MASK)
#define FTM_SYNCONF_HWINVC_MASK                  (0x80000U)
#define FTM_SYNCONF_HWINVC_SHIFT                 (19U)
#define FTM_SYNCONF_HWINVC_WIDTH                 (1U)
#define FTM_SYNCONF_HWINVC(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_HWINVC_SHIFT)) & FTM_SYNCONF_HWINVC_MASK)
#define FTM_SYNCONF_HWSOC_MASK                   (0x100000U)
#define FTM_SYNCONF_HWSOC_SHIFT                  (20U)
#define FTM_SYNCONF_HWSOC_WIDTH                  (1U)
#define FTM_SYNCONF_HWSOC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_HWSOC_SHIFT)) & FTM_SYNCONF_HWSOC_MASK)
/*! @} */

/*! @name INVCTRL - FTM Inverting Control */
/*! @{ */
#define FTM_INVCTRL_INV0EN_MASK                  (0x1U)
#define FTM_INVCTRL_INV0EN_SHIFT                 (0U)
#define FTM_INVCTRL_INV0EN_WIDTH                 (1U)
#define FTM_INVCTRL_INV0EN(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_INVCTRL_INV0EN_SHIFT)) & FTM_INVCTRL_INV0EN_MASK)
#define FTM_INVCTRL_INV1EN_MASK                  (0x2U)
#define FTM_INVCTRL_INV1EN_SHIFT                 (1U)
#define FTM_INVCTRL_INV1EN_WIDTH                 (1U)
#define FTM_INVCTRL_INV1EN(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_INVCTRL_INV1EN_SHIFT)) & FTM_INVCTRL_INV1EN_MASK)
#define FTM_INVCTRL_INV2EN_MASK                  (0x4U)
#define FTM_INVCTRL_INV2EN_SHIFT                 (2U)
#define FTM_INVCTRL_INV2EN_WIDTH                 (1U)
#define FTM_INVCTRL_INV2EN(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_INVCTRL_INV2EN_SHIFT)) & FTM_INVCTRL_INV2EN_MASK)
#define FTM_INVCTRL_INV3EN_MASK                  (0x8U)
#define FTM_INVCTRL_INV3EN_SHIFT                 (3U)
#define FTM_INVCTRL_INV3EN_WIDTH                 (1U)
#define FTM_INVCTRL_INV3EN(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_INVCTRL_INV3EN_SHIFT)) & FTM_INVCTRL_INV3EN_MASK)
/*! @} */

/*! @name SWOCTRL - FTM Software Output Control */
/*! @{ */
#define FTM_SWOCTRL_CH0OC_MASK                   (0x1U)
#define FTM_SWOCTRL_CH0OC_SHIFT                  (0U)
#define FTM_SWOCTRL_CH0OC_WIDTH                  (1U)
#define FTM_SWOCTRL_CH0OC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH0OC_SHIFT)) & FTM_SWOCTRL_CH0OC_MASK)
#define FTM_SWOCTRL_CH1OC_MASK                   (0x2U)
#define FTM_SWOCTRL_CH1OC_SHIFT                  (1U)
#define FTM_SWOCTRL_CH1OC_WIDTH                  (1U)
#define FTM_SWOCTRL_CH1OC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH1OC_SHIFT)) & FTM_SWOCTRL_CH1OC_MASK)
#define FTM_SWOCTRL_CH2OC_MASK                   (0x4U)
#define FTM_SWOCTRL_CH2OC_SHIFT                  (2U)
#define FTM_SWOCTRL_CH2OC_WIDTH                  (1U)
#define FTM_SWOCTRL_CH2OC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH2OC_SHIFT)) & FTM_SWOCTRL_CH2OC_MASK)
#define FTM_SWOCTRL_CH3OC_MASK                   (0x8U)
#define FTM_SWOCTRL_CH3OC_SHIFT                  (3U)
#define FTM_SWOCTRL_CH3OC_WIDTH                  (1U)
#define FTM_SWOCTRL_CH3OC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH3OC_SHIFT)) & FTM_SWOCTRL_CH3OC_MASK)
#define FTM_SWOCTRL_CH4OC_MASK                   (0x10U)
#define FTM_SWOCTRL_CH4OC_SHIFT                  (4U)
#define FTM_SWOCTRL_CH4OC_WIDTH                  (1U)
#define FTM_SWOCTRL_CH4OC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH4OC_SHIFT)) & FTM_SWOCTRL_CH4OC_MASK)
#define FTM_SWOCTRL_CH5OC_MASK                   (0x20U)
#define FTM_SWOCTRL_CH5OC_SHIFT                  (5U)
#define FTM_SWOCTRL_CH5OC_WIDTH                  (1U)
#define FTM_SWOCTRL_CH5OC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH5OC_SHIFT)) & FTM_SWOCTRL_CH5OC_MASK)
#define FTM_SWOCTRL_CH6OC_MASK                   (0x40U)
#define FTM_SWOCTRL_CH6OC_SHIFT                  (6U)
#define FTM_SWOCTRL_CH6OC_WIDTH                  (1U)
#define FTM_SWOCTRL_CH6OC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH6OC_SHIFT)) & FTM_SWOCTRL_CH6OC_MASK)
#define FTM_SWOCTRL_CH7OC_MASK                   (0x80U)
#define FTM_SWOCTRL_CH7OC_SHIFT                  (7U)
#define FTM_SWOCTRL_CH7OC_WIDTH                  (1U)
#define FTM_SWOCTRL_CH7OC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH7OC_SHIFT)) & FTM_SWOCTRL_CH7OC_MASK)
#define FTM_SWOCTRL_CH0OCV_MASK                  (0x100U)
#define FTM_SWOCTRL_CH0OCV_SHIFT                 (8U)
#define FTM_SWOCTRL_CH0OCV_WIDTH                 (1U)
#define FTM_SWOCTRL_CH0OCV(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH0OCV_SHIFT)) & FTM_SWOCTRL_CH0OCV_MASK)
#define FTM_SWOCTRL_CH1OCV_MASK                  (0x200U)
#define FTM_SWOCTRL_CH1OCV_SHIFT                 (9U)
#define FTM_SWOCTRL_CH1OCV_WIDTH                 (1U)
#define FTM_SWOCTRL_CH1OCV(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH1OCV_SHIFT)) & FTM_SWOCTRL_CH1OCV_MASK)
#define FTM_SWOCTRL_CH2OCV_MASK                  (0x400U)
#define FTM_SWOCTRL_CH2OCV_SHIFT                 (10U)
#define FTM_SWOCTRL_CH2OCV_WIDTH                 (1U)
#define FTM_SWOCTRL_CH2OCV(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH2OCV_SHIFT)) & FTM_SWOCTRL_CH2OCV_MASK)
#define FTM_SWOCTRL_CH3OCV_MASK                  (0x800U)
#define FTM_SWOCTRL_CH3OCV_SHIFT                 (11U)
#define FTM_SWOCTRL_CH3OCV_WIDTH                 (1U)
#define FTM_SWOCTRL_CH3OCV(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH3OCV_SHIFT)) & FTM_SWOCTRL_CH3OCV_MASK)
#define FTM_SWOCTRL_CH4OCV_MASK                  (0x1000U)
#define FTM_SWOCTRL_CH4OCV_SHIFT                 (12U)
#define FTM_SWOCTRL_CH4OCV_WIDTH                 (1U)
#define FTM_SWOCTRL_CH4OCV(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH4OCV_SHIFT)) & FTM_SWOCTRL_CH4OCV_MASK)
#define FTM_SWOCTRL_CH5OCV_MASK                  (0x2000U)
#define FTM_SWOCTRL_CH5OCV_SHIFT                 (13U)
#define FTM_SWOCTRL_CH5OCV_WIDTH                 (1U)
#define FTM_SWOCTRL_CH5OCV(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH5OCV_SHIFT)) & FTM_SWOCTRL_CH5OCV_MASK)
#define FTM_SWOCTRL_CH6OCV_MASK                  (0x4000U)
#define FTM_SWOCTRL_CH6OCV_SHIFT                 (14U)
#define FTM_SWOCTRL_CH6OCV_WIDTH                 (1U)
#define FTM_SWOCTRL_CH6OCV(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH6OCV_SHIFT)) & FTM_SWOCTRL_CH6OCV_MASK)
#define FTM_SWOCTRL_CH7OCV_MASK                  (0x8000U)
#define FTM_SWOCTRL_CH7OCV_SHIFT                 (15U)
#define FTM_SWOCTRL_CH7OCV_WIDTH                 (1U)
#define FTM_SWOCTRL_CH7OCV(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH7OCV_SHIFT)) & FTM_SWOCTRL_CH7OCV_MASK)
/*! @} */

/*! @name PWMLOAD - FTM PWM Load */
/*! @{ */
#define FTM_PWMLOAD_CH0SEL_MASK                  (0x1U)
#define FTM_PWMLOAD_CH0SEL_SHIFT                 (0U)
#define FTM_PWMLOAD_CH0SEL_WIDTH                 (1U)
#define FTM_PWMLOAD_CH0SEL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_CH0SEL_SHIFT)) & FTM_PWMLOAD_CH0SEL_MASK)
#define FTM_PWMLOAD_CH1SEL_MASK                  (0x2U)
#define FTM_PWMLOAD_CH1SEL_SHIFT                 (1U)
#define FTM_PWMLOAD_CH1SEL_WIDTH                 (1U)
#define FTM_PWMLOAD_CH1SEL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_CH1SEL_SHIFT)) & FTM_PWMLOAD_CH1SEL_MASK)
#define FTM_PWMLOAD_CH2SEL_MASK                  (0x4U)
#define FTM_PWMLOAD_CH2SEL_SHIFT                 (2U)
#define FTM_PWMLOAD_CH2SEL_WIDTH                 (1U)
#define FTM_PWMLOAD_CH2SEL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_CH2SEL_SHIFT)) & FTM_PWMLOAD_CH2SEL_MASK)
#define FTM_PWMLOAD_CH3SEL_MASK                  (0x8U)
#define FTM_PWMLOAD_CH3SEL_SHIFT                 (3U)
#define FTM_PWMLOAD_CH3SEL_WIDTH                 (1U)
#define FTM_PWMLOAD_CH3SEL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_CH3SEL_SHIFT)) & FTM_PWMLOAD_CH3SEL_MASK)
#define FTM_PWMLOAD_CH4SEL_MASK                  (0x10U)
#define FTM_PWMLOAD_CH4SEL_SHIFT                 (4U)
#define FTM_PWMLOAD_CH4SEL_WIDTH                 (1U)
#define FTM_PWMLOAD_CH4SEL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_CH4SEL_SHIFT)) & FTM_PWMLOAD_CH4SEL_MASK)
#define FTM_PWMLOAD_CH5SEL_MASK                  (0x20U)
#define FTM_PWMLOAD_CH5SEL_SHIFT                 (5U)
#define FTM_PWMLOAD_CH5SEL_WIDTH                 (1U)
#define FTM_PWMLOAD_CH5SEL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_CH5SEL_SHIFT)) & FTM_PWMLOAD_CH5SEL_MASK)
#define FTM_PWMLOAD_CH6SEL_MASK                  (0x40U)
#define FTM_PWMLOAD_CH6SEL_SHIFT                 (6U)
#define FTM_PWMLOAD_CH6SEL_WIDTH                 (1U)
#define FTM_PWMLOAD_CH6SEL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_CH6SEL_SHIFT)) & FTM_PWMLOAD_CH6SEL_MASK)
#define FTM_PWMLOAD_CH7SEL_MASK                  (0x80U)
#define FTM_PWMLOAD_CH7SEL_SHIFT                 (7U)
#define FTM_PWMLOAD_CH7SEL_WIDTH                 (1U)
#define FTM_PWMLOAD_CH7SEL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_CH7SEL_SHIFT)) & FTM_PWMLOAD_CH7SEL_MASK)
#define FTM_PWMLOAD_HCSEL_MASK                   (0x100U)
#define FTM_PWMLOAD_HCSEL_SHIFT                  (8U)
#define FTM_PWMLOAD_HCSEL_WIDTH                  (1U)
#define FTM_PWMLOAD_HCSEL(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_HCSEL_SHIFT)) & FTM_PWMLOAD_HCSEL_MASK)
#define FTM_PWMLOAD_LDOK_MASK                    (0x200U)
#define FTM_PWMLOAD_LDOK_SHIFT                   (9U)
#define FTM_PWMLOAD_LDOK_WIDTH                   (1U)
#define FTM_PWMLOAD_LDOK(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_LDOK_SHIFT)) & FTM_PWMLOAD_LDOK_MASK)
#define FTM_PWMLOAD_GLEN_MASK                    (0x400U)
#define FTM_PWMLOAD_GLEN_SHIFT                   (10U)
#define FTM_PWMLOAD_GLEN_WIDTH                   (1U)
#define FTM_PWMLOAD_GLEN(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_GLEN_SHIFT)) & FTM_PWMLOAD_GLEN_MASK)
#define FTM_PWMLOAD_GLDOK_MASK                   (0x800U)
#define FTM_PWMLOAD_GLDOK_SHIFT                  (11U)
#define FTM_PWMLOAD_GLDOK_WIDTH                  (1U)
#define FTM_PWMLOAD_GLDOK(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_GLDOK_SHIFT)) & FTM_PWMLOAD_GLDOK_MASK)
/*! @} */

/*! @name HCR - Half Cycle Register */
/*! @{ */
#define FTM_HCR_HCVAL_MASK                       (0xFFFFU)
#define FTM_HCR_HCVAL_SHIFT                      (0U)
#define FTM_HCR_HCVAL_WIDTH                      (16U)
#define FTM_HCR_HCVAL(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_HCR_HCVAL_SHIFT)) & FTM_HCR_HCVAL_MASK)
/*! @} */

/*! @name PAIR0DEADTIME - Pair 0 Deadtime Configuration */
/*! @{ */
#define FTM_PAIR0DEADTIME_DTVAL_MASK             (0x3FU)
#define FTM_PAIR0DEADTIME_DTVAL_SHIFT            (0U)
#define FTM_PAIR0DEADTIME_DTVAL_WIDTH            (6U)
#define FTM_PAIR0DEADTIME_DTVAL(x)               (((uint32_t)(((uint32_t)(x)) << FTM_PAIR0DEADTIME_DTVAL_SHIFT)) & FTM_PAIR0DEADTIME_DTVAL_MASK)
#define FTM_PAIR0DEADTIME_DTPS_MASK              (0xC0U)
#define FTM_PAIR0DEADTIME_DTPS_SHIFT             (6U)
#define FTM_PAIR0DEADTIME_DTPS_WIDTH             (2U)
#define FTM_PAIR0DEADTIME_DTPS(x)                (((uint32_t)(((uint32_t)(x)) << FTM_PAIR0DEADTIME_DTPS_SHIFT)) & FTM_PAIR0DEADTIME_DTPS_MASK)
#define FTM_PAIR0DEADTIME_DTVALEX_MASK           (0xF0000U)
#define FTM_PAIR0DEADTIME_DTVALEX_SHIFT          (16U)
#define FTM_PAIR0DEADTIME_DTVALEX_WIDTH          (4U)
#define FTM_PAIR0DEADTIME_DTVALEX(x)             (((uint32_t)(((uint32_t)(x)) << FTM_PAIR0DEADTIME_DTVALEX_SHIFT)) & FTM_PAIR0DEADTIME_DTVALEX_MASK)
/*! @} */

/*! @name PAIR1DEADTIME - Pair 1 Deadtime Configuration */
/*! @{ */
#define FTM_PAIR1DEADTIME_DTVAL_MASK             (0x3FU)
#define FTM_PAIR1DEADTIME_DTVAL_SHIFT            (0U)
#define FTM_PAIR1DEADTIME_DTVAL_WIDTH            (6U)
#define FTM_PAIR1DEADTIME_DTVAL(x)               (((uint32_t)(((uint32_t)(x)) << FTM_PAIR1DEADTIME_DTVAL_SHIFT)) & FTM_PAIR1DEADTIME_DTVAL_MASK)
#define FTM_PAIR1DEADTIME_DTPS_MASK              (0xC0U)
#define FTM_PAIR1DEADTIME_DTPS_SHIFT             (6U)
#define FTM_PAIR1DEADTIME_DTPS_WIDTH             (2U)
#define FTM_PAIR1DEADTIME_DTPS(x)                (((uint32_t)(((uint32_t)(x)) << FTM_PAIR1DEADTIME_DTPS_SHIFT)) & FTM_PAIR1DEADTIME_DTPS_MASK)
#define FTM_PAIR1DEADTIME_DTVALEX_MASK           (0xF0000U)
#define FTM_PAIR1DEADTIME_DTVALEX_SHIFT          (16U)
#define FTM_PAIR1DEADTIME_DTVALEX_WIDTH          (4U)
#define FTM_PAIR1DEADTIME_DTVALEX(x)             (((uint32_t)(((uint32_t)(x)) << FTM_PAIR1DEADTIME_DTVALEX_SHIFT)) & FTM_PAIR1DEADTIME_DTVALEX_MASK)
/*! @} */

/*! @name PAIR2DEADTIME - Pair 2 Deadtime Configuration */
/*! @{ */
#define FTM_PAIR2DEADTIME_DTVAL_MASK             (0x3FU)
#define FTM_PAIR2DEADTIME_DTVAL_SHIFT            (0U)
#define FTM_PAIR2DEADTIME_DTVAL_WIDTH            (6U)
#define FTM_PAIR2DEADTIME_DTVAL(x)               (((uint32_t)(((uint32_t)(x)) << FTM_PAIR2DEADTIME_DTVAL_SHIFT)) & FTM_PAIR2DEADTIME_DTVAL_MASK)
#define FTM_PAIR2DEADTIME_DTPS_MASK              (0xC0U)
#define FTM_PAIR2DEADTIME_DTPS_SHIFT             (6U)
#define FTM_PAIR2DEADTIME_DTPS_WIDTH             (2U)
#define FTM_PAIR2DEADTIME_DTPS(x)                (((uint32_t)(((uint32_t)(x)) << FTM_PAIR2DEADTIME_DTPS_SHIFT)) & FTM_PAIR2DEADTIME_DTPS_MASK)
#define FTM_PAIR2DEADTIME_DTVALEX_MASK           (0xF0000U)
#define FTM_PAIR2DEADTIME_DTVALEX_SHIFT          (16U)
#define FTM_PAIR2DEADTIME_DTVALEX_WIDTH          (4U)
#define FTM_PAIR2DEADTIME_DTVALEX(x)             (((uint32_t)(((uint32_t)(x)) << FTM_PAIR2DEADTIME_DTVALEX_SHIFT)) & FTM_PAIR2DEADTIME_DTVALEX_MASK)
/*! @} */

/*! @name PAIR3DEADTIME - Pair 3 Deadtime Configuration */
/*! @{ */
#define FTM_PAIR3DEADTIME_DTVAL_MASK             (0x3FU)
#define FTM_PAIR3DEADTIME_DTVAL_SHIFT            (0U)
#define FTM_PAIR3DEADTIME_DTVAL_WIDTH            (6U)
#define FTM_PAIR3DEADTIME_DTVAL(x)               (((uint32_t)(((uint32_t)(x)) << FTM_PAIR3DEADTIME_DTVAL_SHIFT)) & FTM_PAIR3DEADTIME_DTVAL_MASK)
#define FTM_PAIR3DEADTIME_DTPS_MASK              (0xC0U)
#define FTM_PAIR3DEADTIME_DTPS_SHIFT             (6U)
#define FTM_PAIR3DEADTIME_DTPS_WIDTH             (2U)
#define FTM_PAIR3DEADTIME_DTPS(x)                (((uint32_t)(((uint32_t)(x)) << FTM_PAIR3DEADTIME_DTPS_SHIFT)) & FTM_PAIR3DEADTIME_DTPS_MASK)
#define FTM_PAIR3DEADTIME_DTVALEX_MASK           (0xF0000U)
#define FTM_PAIR3DEADTIME_DTVALEX_SHIFT          (16U)
#define FTM_PAIR3DEADTIME_DTVALEX_WIDTH          (4U)
#define FTM_PAIR3DEADTIME_DTVALEX(x)             (((uint32_t)(((uint32_t)(x)) << FTM_PAIR3DEADTIME_DTVALEX_SHIFT)) & FTM_PAIR3DEADTIME_DTVALEX_MASK)
/*! @} */

/*! @name MOD_MIRROR - Mirror of Modulo Value */
/*! @{ */
#define FTM_MOD_MIRROR_FRACMOD_MASK              (0xF800U)
#define FTM_MOD_MIRROR_FRACMOD_SHIFT             (11U)
#define FTM_MOD_MIRROR_FRACMOD_WIDTH             (5U)
#define FTM_MOD_MIRROR_FRACMOD(x)                (((uint32_t)(((uint32_t)(x)) << FTM_MOD_MIRROR_FRACMOD_SHIFT)) & FTM_MOD_MIRROR_FRACMOD_MASK)
#define FTM_MOD_MIRROR_MOD_MASK                  (0xFFFF0000U)
#define FTM_MOD_MIRROR_MOD_SHIFT                 (16U)
#define FTM_MOD_MIRROR_MOD_WIDTH                 (16U)
#define FTM_MOD_MIRROR_MOD(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_MOD_MIRROR_MOD_SHIFT)) & FTM_MOD_MIRROR_MOD_MASK)
/*! @} */

/*! @name CV_MIRROR - Mirror of Channel (n) Match Value */
/*! @{ */
#define FTM_CV_MIRROR_FRACVAL_MASK               (0xF800U)
#define FTM_CV_MIRROR_FRACVAL_SHIFT              (11U)
#define FTM_CV_MIRROR_FRACVAL_WIDTH              (5U)
#define FTM_CV_MIRROR_FRACVAL(x)                 (((uint32_t)(((uint32_t)(x)) << FTM_CV_MIRROR_FRACVAL_SHIFT)) & FTM_CV_MIRROR_FRACVAL_MASK)
#define FTM_CV_MIRROR_VAL_MASK                   (0xFFFF0000U)
#define FTM_CV_MIRROR_VAL_SHIFT                  (16U)
#define FTM_CV_MIRROR_VAL_WIDTH                  (16U)
#define FTM_CV_MIRROR_VAL(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_CV_MIRROR_VAL_SHIFT)) & FTM_CV_MIRROR_VAL_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group FTM_Register_Masks */

/*!
 * @}
 */ /* end of group FTM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- GPIO Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Peripheral_Access_Layer GPIO Peripheral Access Layer
 * @{
 */

/** GPIO - Register Layout Typedef */
typedef struct {
  __IO uint32_t PDOR;                              /**< Port Data Output Register, offset: 0x0 */
  __O  uint32_t PSOR;                              /**< Port Set Output Register, offset: 0x4 */
  __O  uint32_t PCOR;                              /**< Port Clear Output Register, offset: 0x8 */
  __O  uint32_t PTOR;                              /**< Port Toggle Output Register, offset: 0xC */
  __I  uint32_t PDIR;                              /**< Port Data Input Register, offset: 0x10 */
  __IO uint32_t PDDR;                              /**< Port Data Direction Register, offset: 0x14 */
  __IO uint32_t PIDR;                              /**< Port Input Disable Register, offset: 0x18 */
} GPIO_Type, *GPIO_MemMapPtr;

/** Number of instances of the GPIO module. */
#define GPIO_INSTANCE_COUNT                      (5u)

/* GPIO - Peripheral instance base addresses */
/** Peripheral PTA base address */
#define PTA_BASE                                 (0x400FF000u)
/** Peripheral PTA base pointer */
#define PTA                                      ((GPIO_Type *)PTA_BASE)
/** Peripheral PTB base address */
#define PTB_BASE                                 (0x400FF040u)
/** Peripheral PTB base pointer */
#define PTB                                      ((GPIO_Type *)PTB_BASE)
/** Peripheral PTC base address */
#define PTC_BASE                                 (0x400FF080u)
/** Peripheral PTC base pointer */
#define PTC                                      ((GPIO_Type *)PTC_BASE)
/** Peripheral PTD base address */
#define PTD_BASE                                 (0x400FF0C0u)
/** Peripheral PTD base pointer */
#define PTD                                      ((GPIO_Type *)PTD_BASE)
/** Peripheral PTE base address */
#define PTE_BASE                                 (0x400FF100u)
/** Peripheral PTE base pointer */
#define PTE                                      ((GPIO_Type *)PTE_BASE)
/** Array initializer of GPIO peripheral base addresses */
#define GPIO_BASE_ADDRS                          { PTA_BASE, PTB_BASE, PTC_BASE, PTD_BASE, PTE_BASE }
/** Array initializer of GPIO peripheral base pointers */
#define GPIO_BASE_PTRS                           { PTA, PTB, PTC, PTD, PTE }

/* ----------------------------------------------------------------------------
   -- GPIO Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Register_Masks GPIO Register Masks
 * @{
 */

/*! @name PDOR - Port Data Output Register */
/*! @{ */
#define GPIO_PDOR_PDO_MASK                       (0xFFFFFFFFU)
#define GPIO_PDOR_PDO_SHIFT                      (0U)
#define GPIO_PDOR_PDO_WIDTH                      (32U)
#define GPIO_PDOR_PDO(x)                         (((uint32_t)(((uint32_t)(x)) << GPIO_PDOR_PDO_SHIFT)) & GPIO_PDOR_PDO_MASK)
/*! @} */

/*! @name PSOR - Port Set Output Register */
/*! @{ */
#define GPIO_PSOR_PTSO_MASK                      (0xFFFFFFFFU)
#define GPIO_PSOR_PTSO_SHIFT                     (0U)
#define GPIO_PSOR_PTSO_WIDTH                     (32U)
#define GPIO_PSOR_PTSO(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PSOR_PTSO_SHIFT)) & GPIO_PSOR_PTSO_MASK)
/*! @} */

/*! @name PCOR - Port Clear Output Register */
/*! @{ */
#define GPIO_PCOR_PTCO_MASK                      (0xFFFFFFFFU)
#define GPIO_PCOR_PTCO_SHIFT                     (0U)
#define GPIO_PCOR_PTCO_WIDTH                     (32U)
#define GPIO_PCOR_PTCO(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PCOR_PTCO_SHIFT)) & GPIO_PCOR_PTCO_MASK)
/*! @} */

/*! @name PTOR - Port Toggle Output Register */
/*! @{ */
#define GPIO_PTOR_PTTO_MASK                      (0xFFFFFFFFU)
#define GPIO_PTOR_PTTO_SHIFT                     (0U)
#define GPIO_PTOR_PTTO_WIDTH                     (32U)
#define GPIO_PTOR_PTTO(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_PTOR_PTTO_SHIFT)) & GPIO_PTOR_PTTO_MASK)
/*! @} */

/*! @name PDIR - Port Data Input Register */
/*! @{ */
#define GPIO_PDIR_PDI_MASK                       (0xFFFFFFFFU)
#define GPIO_PDIR_PDI_SHIFT                      (0U)
#define GPIO_PDIR_PDI_WIDTH                      (32U)
#define GPIO_PDIR_PDI(x)                         (((uint32_t)(((uint32_t)(x)) << GPIO_PDIR_PDI_SHIFT)) & GPIO_PDIR_PDI_MASK)
/*! @} */

/*! @name PDDR - Port Data Direction Register */
/*! @{ */
#define GPIO_PDDR_PDD_MASK                       (0xFFFFFFFFU)
#define GPIO_PDDR_PDD_SHIFT                      (0U)
#define GPIO_PDDR_PDD_WIDTH                      (32U)
#define GPIO_PDDR_PDD(x)                         (((uint32_t)(((uint32_t)(x)) << GPIO_PDDR_PDD_SHIFT)) & GPIO_PDDR_PDD_MASK)
/*! @} */

/*! @name PIDR - Port Input Disable Register */
/*! @{ */
#define GPIO_PIDR_PID_MASK                       (0xFFFFFFFFU)
#define GPIO_PIDR_PID_SHIFT                      (0U)
#define GPIO_PIDR_PID_WIDTH                      (32U)
#define GPIO_PIDR_PID(x)                         (((uint32_t)(((uint32_t)(x)) << GPIO_PIDR_PID_SHIFT)) & GPIO_PIDR_PID_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group GPIO_Register_Masks */

/*!
 * @}
 */ /* end of group GPIO_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- LMEM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LMEM_Peripheral_Access_Layer LMEM Peripheral Access Layer
 * @{
 */

/** LMEM - Register Layout Typedef */
typedef struct {
  __IO uint32_t PCCCR;                             /**< Cache control register, offset: 0x0 */
  __IO uint32_t PCCLCR;                            /**< Cache line control register, offset: 0x4 */
  __IO uint32_t PCCSAR;                            /**< Cache search address register, offset: 0x8 */
  __IO uint32_t PCCCVR;                            /**< Cache read/write value register, offset: 0xC */
  uint8_t RESERVED_0[16];
  __IO uint32_t PCCRMR;                            /**< Cache regions mode register, offset: 0x20 */
} LMEM_Type, *LMEM_MemMapPtr;

/** Number of instances of the LMEM module. */
#define LMEM_INSTANCE_COUNT                      (1u)

/* LMEM - Peripheral instance base addresses */
/** Peripheral LMEM base address */
#define LMEM_BASE                                (0xE0082000u)
/** Peripheral LMEM base pointer */
#define LMEM                                     ((LMEM_Type *)LMEM_BASE)
/** Array initializer of LMEM peripheral base addresses */
#define LMEM_BASE_ADDRS                          { LMEM_BASE }
/** Array initializer of LMEM peripheral base pointers */
#define LMEM_BASE_PTRS                           { LMEM }

/* ----------------------------------------------------------------------------
   -- LMEM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LMEM_Register_Masks LMEM Register Masks
 * @{
 */

/*! @name PCCCR - Cache control register */
/*! @{ */
#define LMEM_PCCCR_ENCACHE_MASK                  (0x1U)
#define LMEM_PCCCR_ENCACHE_SHIFT                 (0U)
#define LMEM_PCCCR_ENCACHE_WIDTH                 (1U)
#define LMEM_PCCCR_ENCACHE(x)                    (((uint32_t)(((uint32_t)(x)) << LMEM_PCCCR_ENCACHE_SHIFT)) & LMEM_PCCCR_ENCACHE_MASK)
#define LMEM_PCCCR_PCCR2_MASK                    (0x4U)
#define LMEM_PCCCR_PCCR2_SHIFT                   (2U)
#define LMEM_PCCCR_PCCR2_WIDTH                   (1U)
#define LMEM_PCCCR_PCCR2(x)                      (((uint32_t)(((uint32_t)(x)) << LMEM_PCCCR_PCCR2_SHIFT)) & LMEM_PCCCR_PCCR2_MASK)
#define LMEM_PCCCR_PCCR3_MASK                    (0x8U)
#define LMEM_PCCCR_PCCR3_SHIFT                   (3U)
#define LMEM_PCCCR_PCCR3_WIDTH                   (1U)
#define LMEM_PCCCR_PCCR3(x)                      (((uint32_t)(((uint32_t)(x)) << LMEM_PCCCR_PCCR3_SHIFT)) & LMEM_PCCCR_PCCR3_MASK)
#define LMEM_PCCCR_INVW0_MASK                    (0x1000000U)
#define LMEM_PCCCR_INVW0_SHIFT                   (24U)
#define LMEM_PCCCR_INVW0_WIDTH                   (1U)
#define LMEM_PCCCR_INVW0(x)                      (((uint32_t)(((uint32_t)(x)) << LMEM_PCCCR_INVW0_SHIFT)) & LMEM_PCCCR_INVW0_MASK)
#define LMEM_PCCCR_PUSHW0_MASK                   (0x2000000U)
#define LMEM_PCCCR_PUSHW0_SHIFT                  (25U)
#define LMEM_PCCCR_PUSHW0_WIDTH                  (1U)
#define LMEM_PCCCR_PUSHW0(x)                     (((uint32_t)(((uint32_t)(x)) << LMEM_PCCCR_PUSHW0_SHIFT)) & LMEM_PCCCR_PUSHW0_MASK)
#define LMEM_PCCCR_INVW1_MASK                    (0x4000000U)
#define LMEM_PCCCR_INVW1_SHIFT                   (26U)
#define LMEM_PCCCR_INVW1_WIDTH                   (1U)
#define LMEM_PCCCR_INVW1(x)                      (((uint32_t)(((uint32_t)(x)) << LMEM_PCCCR_INVW1_SHIFT)) & LMEM_PCCCR_INVW1_MASK)
#define LMEM_PCCCR_PUSHW1_MASK                   (0x8000000U)
#define LMEM_PCCCR_PUSHW1_SHIFT                  (27U)
#define LMEM_PCCCR_PUSHW1_WIDTH                  (1U)
#define LMEM_PCCCR_PUSHW1(x)                     (((uint32_t)(((uint32_t)(x)) << LMEM_PCCCR_PUSHW1_SHIFT)) & LMEM_PCCCR_PUSHW1_MASK)
#define LMEM_PCCCR_GO_MASK                       (0x80000000U)
#define LMEM_PCCCR_GO_SHIFT                      (31U)
#define LMEM_PCCCR_GO_WIDTH                      (1U)
#define LMEM_PCCCR_GO(x)                         (((uint32_t)(((uint32_t)(x)) << LMEM_PCCCR_GO_SHIFT)) & LMEM_PCCCR_GO_MASK)
/*! @} */

/*! @name PCCLCR - Cache line control register */
/*! @{ */
#define LMEM_PCCLCR_LGO_MASK                     (0x1U)
#define LMEM_PCCLCR_LGO_SHIFT                    (0U)
#define LMEM_PCCLCR_LGO_WIDTH                    (1U)
#define LMEM_PCCLCR_LGO(x)                       (((uint32_t)(((uint32_t)(x)) << LMEM_PCCLCR_LGO_SHIFT)) & LMEM_PCCLCR_LGO_MASK)
#define LMEM_PCCLCR_CACHEADDR_MASK               (0x3FFCU)
#define LMEM_PCCLCR_CACHEADDR_SHIFT              (2U)
#define LMEM_PCCLCR_CACHEADDR_WIDTH              (12U)
#define LMEM_PCCLCR_CACHEADDR(x)                 (((uint32_t)(((uint32_t)(x)) << LMEM_PCCLCR_CACHEADDR_SHIFT)) & LMEM_PCCLCR_CACHEADDR_MASK)
#define LMEM_PCCLCR_WSEL_MASK                    (0x4000U)
#define LMEM_PCCLCR_WSEL_SHIFT                   (14U)
#define LMEM_PCCLCR_WSEL_WIDTH                   (1U)
#define LMEM_PCCLCR_WSEL(x)                      (((uint32_t)(((uint32_t)(x)) << LMEM_PCCLCR_WSEL_SHIFT)) & LMEM_PCCLCR_WSEL_MASK)
#define LMEM_PCCLCR_TDSEL_MASK                   (0x10000U)
#define LMEM_PCCLCR_TDSEL_SHIFT                  (16U)
#define LMEM_PCCLCR_TDSEL_WIDTH                  (1U)
#define LMEM_PCCLCR_TDSEL(x)                     (((uint32_t)(((uint32_t)(x)) << LMEM_PCCLCR_TDSEL_SHIFT)) & LMEM_PCCLCR_TDSEL_MASK)
#define LMEM_PCCLCR_LCIVB_MASK                   (0x100000U)
#define LMEM_PCCLCR_LCIVB_SHIFT                  (20U)
#define LMEM_PCCLCR_LCIVB_WIDTH                  (1U)
#define LMEM_PCCLCR_LCIVB(x)                     (((uint32_t)(((uint32_t)(x)) << LMEM_PCCLCR_LCIVB_SHIFT)) & LMEM_PCCLCR_LCIVB_MASK)
#define LMEM_PCCLCR_LCIMB_MASK                   (0x200000U)
#define LMEM_PCCLCR_LCIMB_SHIFT                  (21U)
#define LMEM_PCCLCR_LCIMB_WIDTH                  (1U)
#define LMEM_PCCLCR_LCIMB(x)                     (((uint32_t)(((uint32_t)(x)) << LMEM_PCCLCR_LCIMB_SHIFT)) & LMEM_PCCLCR_LCIMB_MASK)
#define LMEM_PCCLCR_LCWAY_MASK                   (0x400000U)
#define LMEM_PCCLCR_LCWAY_SHIFT                  (22U)
#define LMEM_PCCLCR_LCWAY_WIDTH                  (1U)
#define LMEM_PCCLCR_LCWAY(x)                     (((uint32_t)(((uint32_t)(x)) << LMEM_PCCLCR_LCWAY_SHIFT)) & LMEM_PCCLCR_LCWAY_MASK)
#define LMEM_PCCLCR_LCMD_MASK                    (0x3000000U)
#define LMEM_PCCLCR_LCMD_SHIFT                   (24U)
#define LMEM_PCCLCR_LCMD_WIDTH                   (2U)
#define LMEM_PCCLCR_LCMD(x)                      (((uint32_t)(((uint32_t)(x)) << LMEM_PCCLCR_LCMD_SHIFT)) & LMEM_PCCLCR_LCMD_MASK)
#define LMEM_PCCLCR_LADSEL_MASK                  (0x4000000U)
#define LMEM_PCCLCR_LADSEL_SHIFT                 (26U)
#define LMEM_PCCLCR_LADSEL_WIDTH                 (1U)
#define LMEM_PCCLCR_LADSEL(x)                    (((uint32_t)(((uint32_t)(x)) << LMEM_PCCLCR_LADSEL_SHIFT)) & LMEM_PCCLCR_LADSEL_MASK)
#define LMEM_PCCLCR_LACC_MASK                    (0x8000000U)
#define LMEM_PCCLCR_LACC_SHIFT                   (27U)
#define LMEM_PCCLCR_LACC_WIDTH                   (1U)
#define LMEM_PCCLCR_LACC(x)                      (((uint32_t)(((uint32_t)(x)) << LMEM_PCCLCR_LACC_SHIFT)) & LMEM_PCCLCR_LACC_MASK)
/*! @} */

/*! @name PCCSAR - Cache search address register */
/*! @{ */
#define LMEM_PCCSAR_LGO_MASK                     (0x1U)
#define LMEM_PCCSAR_LGO_SHIFT                    (0U)
#define LMEM_PCCSAR_LGO_WIDTH                    (1U)
#define LMEM_PCCSAR_LGO(x)                       (((uint32_t)(((uint32_t)(x)) << LMEM_PCCSAR_LGO_SHIFT)) & LMEM_PCCSAR_LGO_MASK)
#define LMEM_PCCSAR_PHYADDR_MASK                 (0xFFFFFFFCU)
#define LMEM_PCCSAR_PHYADDR_SHIFT                (2U)
#define LMEM_PCCSAR_PHYADDR_WIDTH                (30U)
#define LMEM_PCCSAR_PHYADDR(x)                   (((uint32_t)(((uint32_t)(x)) << LMEM_PCCSAR_PHYADDR_SHIFT)) & LMEM_PCCSAR_PHYADDR_MASK)
/*! @} */

/*! @name PCCCVR - Cache read/write value register */
/*! @{ */
#define LMEM_PCCCVR_DATA_MASK                    (0xFFFFFFFFU)
#define LMEM_PCCCVR_DATA_SHIFT                   (0U)
#define LMEM_PCCCVR_DATA_WIDTH                   (32U)
#define LMEM_PCCCVR_DATA(x)                      (((uint32_t)(((uint32_t)(x)) << LMEM_PCCCVR_DATA_SHIFT)) & LMEM_PCCCVR_DATA_MASK)
/*! @} */

/*! @name PCCRMR - Cache regions mode register */
/*! @{ */
#define LMEM_PCCRMR_R15_MASK                     (0x3U)
#define LMEM_PCCRMR_R15_SHIFT                    (0U)
#define LMEM_PCCRMR_R15_WIDTH                    (2U)
#define LMEM_PCCRMR_R15(x)                       (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R15_SHIFT)) & LMEM_PCCRMR_R15_MASK)
#define LMEM_PCCRMR_R14_MASK                     (0xCU)
#define LMEM_PCCRMR_R14_SHIFT                    (2U)
#define LMEM_PCCRMR_R14_WIDTH                    (2U)
#define LMEM_PCCRMR_R14(x)                       (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R14_SHIFT)) & LMEM_PCCRMR_R14_MASK)
#define LMEM_PCCRMR_R13_MASK                     (0x30U)
#define LMEM_PCCRMR_R13_SHIFT                    (4U)
#define LMEM_PCCRMR_R13_WIDTH                    (2U)
#define LMEM_PCCRMR_R13(x)                       (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R13_SHIFT)) & LMEM_PCCRMR_R13_MASK)
#define LMEM_PCCRMR_R12_MASK                     (0xC0U)
#define LMEM_PCCRMR_R12_SHIFT                    (6U)
#define LMEM_PCCRMR_R12_WIDTH                    (2U)
#define LMEM_PCCRMR_R12(x)                       (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R12_SHIFT)) & LMEM_PCCRMR_R12_MASK)
#define LMEM_PCCRMR_R11_MASK                     (0x300U)
#define LMEM_PCCRMR_R11_SHIFT                    (8U)
#define LMEM_PCCRMR_R11_WIDTH                    (2U)
#define LMEM_PCCRMR_R11(x)                       (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R11_SHIFT)) & LMEM_PCCRMR_R11_MASK)
#define LMEM_PCCRMR_R10_MASK                     (0xC00U)
#define LMEM_PCCRMR_R10_SHIFT                    (10U)
#define LMEM_PCCRMR_R10_WIDTH                    (2U)
#define LMEM_PCCRMR_R10(x)                       (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R10_SHIFT)) & LMEM_PCCRMR_R10_MASK)
#define LMEM_PCCRMR_R9_MASK                      (0x3000U)
#define LMEM_PCCRMR_R9_SHIFT                     (12U)
#define LMEM_PCCRMR_R9_WIDTH                     (2U)
#define LMEM_PCCRMR_R9(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R9_SHIFT)) & LMEM_PCCRMR_R9_MASK)
#define LMEM_PCCRMR_R8_MASK                      (0xC000U)
#define LMEM_PCCRMR_R8_SHIFT                     (14U)
#define LMEM_PCCRMR_R8_WIDTH                     (2U)
#define LMEM_PCCRMR_R8(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R8_SHIFT)) & LMEM_PCCRMR_R8_MASK)
#define LMEM_PCCRMR_R7_MASK                      (0x30000U)
#define LMEM_PCCRMR_R7_SHIFT                     (16U)
#define LMEM_PCCRMR_R7_WIDTH                     (2U)
#define LMEM_PCCRMR_R7(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R7_SHIFT)) & LMEM_PCCRMR_R7_MASK)
#define LMEM_PCCRMR_R6_MASK                      (0xC0000U)
#define LMEM_PCCRMR_R6_SHIFT                     (18U)
#define LMEM_PCCRMR_R6_WIDTH                     (2U)
#define LMEM_PCCRMR_R6(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R6_SHIFT)) & LMEM_PCCRMR_R6_MASK)
#define LMEM_PCCRMR_R5_MASK                      (0x300000U)
#define LMEM_PCCRMR_R5_SHIFT                     (20U)
#define LMEM_PCCRMR_R5_WIDTH                     (2U)
#define LMEM_PCCRMR_R5(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R5_SHIFT)) & LMEM_PCCRMR_R5_MASK)
#define LMEM_PCCRMR_R4_MASK                      (0xC00000U)
#define LMEM_PCCRMR_R4_SHIFT                     (22U)
#define LMEM_PCCRMR_R4_WIDTH                     (2U)
#define LMEM_PCCRMR_R4(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R4_SHIFT)) & LMEM_PCCRMR_R4_MASK)
#define LMEM_PCCRMR_R3_MASK                      (0x3000000U)
#define LMEM_PCCRMR_R3_SHIFT                     (24U)
#define LMEM_PCCRMR_R3_WIDTH                     (2U)
#define LMEM_PCCRMR_R3(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R3_SHIFT)) & LMEM_PCCRMR_R3_MASK)
#define LMEM_PCCRMR_R2_MASK                      (0xC000000U)
#define LMEM_PCCRMR_R2_SHIFT                     (26U)
#define LMEM_PCCRMR_R2_WIDTH                     (2U)
#define LMEM_PCCRMR_R2(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R2_SHIFT)) & LMEM_PCCRMR_R2_MASK)
#define LMEM_PCCRMR_R1_MASK                      (0x30000000U)
#define LMEM_PCCRMR_R1_SHIFT                     (28U)
#define LMEM_PCCRMR_R1_WIDTH                     (2U)
#define LMEM_PCCRMR_R1(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R1_SHIFT)) & LMEM_PCCRMR_R1_MASK)
#define LMEM_PCCRMR_R0_MASK                      (0xC0000000U)
#define LMEM_PCCRMR_R0_SHIFT                     (30U)
#define LMEM_PCCRMR_R0_WIDTH                     (2U)
#define LMEM_PCCRMR_R0(x)                        (((uint32_t)(((uint32_t)(x)) << LMEM_PCCRMR_R0_SHIFT)) & LMEM_PCCRMR_R0_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group LMEM_Register_Masks */

/*!
 * @}
 */ /* end of group LMEM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- LPI2C Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPI2C_Peripheral_Access_Layer LPI2C Peripheral Access Layer
 * @{
 */

/** LPI2C - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID Register, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< Parameter Register, offset: 0x4 */
  uint8_t RESERVED_0[8];
  __IO uint32_t MCR;                               /**< Master Control Register, offset: 0x10 */
  __IO uint32_t MSR;                               /**< Master Status Register, offset: 0x14 */
  __IO uint32_t MIER;                              /**< Master Interrupt Enable Register, offset: 0x18 */
  __IO uint32_t MDER;                              /**< Master DMA Enable Register, offset: 0x1C */
  __IO uint32_t MCFGR0;                            /**< Master Configuration Register 0, offset: 0x20 */
  __IO uint32_t MCFGR1;                            /**< Master Configuration Register 1, offset: 0x24 */
  __IO uint32_t MCFGR2;                            /**< Master Configuration Register 2, offset: 0x28 */
  __IO uint32_t MCFGR3;                            /**< Master Configuration Register 3, offset: 0x2C */
  uint8_t RESERVED_1[16];
  __IO uint32_t MDMR;                              /**< Master Data Match Register, offset: 0x40 */
  uint8_t RESERVED_2[4];
  __IO uint32_t MCCR0;                             /**< Master Clock Configuration Register 0, offset: 0x48 */
  uint8_t RESERVED_3[4];
  __IO uint32_t MCCR1;                             /**< Master Clock Configuration Register 1, offset: 0x50 */
  uint8_t RESERVED_4[4];
  __IO uint32_t MFCR;                              /**< Master FIFO Control Register, offset: 0x58 */
  __I  uint32_t MFSR;                              /**< Master FIFO Status Register, offset: 0x5C */
  __O  uint32_t MTDR;                              /**< Master Transmit Data Register, offset: 0x60 */
  uint8_t RESERVED_5[12];
  __I  uint32_t MRDR;                              /**< Master Receive Data Register, offset: 0x70 */
  uint8_t RESERVED_6[156];
  __IO uint32_t SCR;                               /**< Slave Control Register, offset: 0x110 */
  __IO uint32_t SSR;                               /**< Slave Status Register, offset: 0x114 */
  __IO uint32_t SIER;                              /**< Slave Interrupt Enable Register, offset: 0x118 */
  __IO uint32_t SDER;                              /**< Slave DMA Enable Register, offset: 0x11C */
  uint8_t RESERVED_7[4];
  __IO uint32_t SCFGR1;                            /**< Slave Configuration Register 1, offset: 0x124 */
  __IO uint32_t SCFGR2;                            /**< Slave Configuration Register 2, offset: 0x128 */
  uint8_t RESERVED_8[20];
  __IO uint32_t SAMR;                              /**< Slave Address Match Register, offset: 0x140 */
  uint8_t RESERVED_9[12];
  __I  uint32_t SASR;                              /**< Slave Address Status Register, offset: 0x150 */
  __IO uint32_t STAR;                              /**< Slave Transmit ACK Register, offset: 0x154 */
  uint8_t RESERVED_10[8];
  __O  uint32_t STDR;                              /**< Slave Transmit Data Register, offset: 0x160 */
  uint8_t RESERVED_11[12];
  __I  uint32_t SRDR;                              /**< Slave Receive Data Register, offset: 0x170 */
} LPI2C_Type, *LPI2C_MemMapPtr;

/** Number of instances of the LPI2C module. */
#define LPI2C_INSTANCE_COUNT                     (1u)

/* LPI2C - Peripheral instance base addresses */
/** Peripheral LPI2C0 base address */
#define LPI2C0_BASE                              (0x40066000u)
/** Peripheral LPI2C0 base pointer */
#define LPI2C0                                   ((LPI2C_Type *)LPI2C0_BASE)
/** Array initializer of LPI2C peripheral base addresses */
#define LPI2C_BASE_ADDRS                         { LPI2C0_BASE }
/** Array initializer of LPI2C peripheral base pointers */
#define LPI2C_BASE_PTRS                          { LPI2C0 }

/* ----------------------------------------------------------------------------
   -- LPI2C Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPI2C_Register_Masks LPI2C Register Masks
 * @{
 */

/*! @name VERID - Version ID Register */
/*! @{ */
#define LPI2C_VERID_FEATURE_MASK                 (0xFFFFU)
#define LPI2C_VERID_FEATURE_SHIFT                (0U)
#define LPI2C_VERID_FEATURE_WIDTH                (16U)
#define LPI2C_VERID_FEATURE(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_VERID_FEATURE_SHIFT)) & LPI2C_VERID_FEATURE_MASK)
#define LPI2C_VERID_MINOR_MASK                   (0xFF0000U)
#define LPI2C_VERID_MINOR_SHIFT                  (16U)
#define LPI2C_VERID_MINOR_WIDTH                  (8U)
#define LPI2C_VERID_MINOR(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_VERID_MINOR_SHIFT)) & LPI2C_VERID_MINOR_MASK)
#define LPI2C_VERID_MAJOR_MASK                   (0xFF000000U)
#define LPI2C_VERID_MAJOR_SHIFT                  (24U)
#define LPI2C_VERID_MAJOR_WIDTH                  (8U)
#define LPI2C_VERID_MAJOR(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_VERID_MAJOR_SHIFT)) & LPI2C_VERID_MAJOR_MASK)
/*! @} */

/*! @name PARAM - Parameter Register */
/*! @{ */
#define LPI2C_PARAM_MTXFIFO_MASK                 (0xFU)
#define LPI2C_PARAM_MTXFIFO_SHIFT                (0U)
#define LPI2C_PARAM_MTXFIFO_WIDTH                (4U)
#define LPI2C_PARAM_MTXFIFO(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_PARAM_MTXFIFO_SHIFT)) & LPI2C_PARAM_MTXFIFO_MASK)
#define LPI2C_PARAM_MRXFIFO_MASK                 (0xF00U)
#define LPI2C_PARAM_MRXFIFO_SHIFT                (8U)
#define LPI2C_PARAM_MRXFIFO_WIDTH                (4U)
#define LPI2C_PARAM_MRXFIFO(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_PARAM_MRXFIFO_SHIFT)) & LPI2C_PARAM_MRXFIFO_MASK)
/*! @} */

/*! @name MCR - Master Control Register */
/*! @{ */
#define LPI2C_MCR_MEN_MASK                       (0x1U)
#define LPI2C_MCR_MEN_SHIFT                      (0U)
#define LPI2C_MCR_MEN_WIDTH                      (1U)
#define LPI2C_MCR_MEN(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MCR_MEN_SHIFT)) & LPI2C_MCR_MEN_MASK)
#define LPI2C_MCR_RST_MASK                       (0x2U)
#define LPI2C_MCR_RST_SHIFT                      (1U)
#define LPI2C_MCR_RST_WIDTH                      (1U)
#define LPI2C_MCR_RST(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MCR_RST_SHIFT)) & LPI2C_MCR_RST_MASK)
#define LPI2C_MCR_DOZEN_MASK                     (0x4U)
#define LPI2C_MCR_DOZEN_SHIFT                    (2U)
#define LPI2C_MCR_DOZEN_WIDTH                    (1U)
#define LPI2C_MCR_DOZEN(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MCR_DOZEN_SHIFT)) & LPI2C_MCR_DOZEN_MASK)
#define LPI2C_MCR_DBGEN_MASK                     (0x8U)
#define LPI2C_MCR_DBGEN_SHIFT                    (3U)
#define LPI2C_MCR_DBGEN_WIDTH                    (1U)
#define LPI2C_MCR_DBGEN(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MCR_DBGEN_SHIFT)) & LPI2C_MCR_DBGEN_MASK)
#define LPI2C_MCR_RTF_MASK                       (0x100U)
#define LPI2C_MCR_RTF_SHIFT                      (8U)
#define LPI2C_MCR_RTF_WIDTH                      (1U)
#define LPI2C_MCR_RTF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MCR_RTF_SHIFT)) & LPI2C_MCR_RTF_MASK)
#define LPI2C_MCR_RRF_MASK                       (0x200U)
#define LPI2C_MCR_RRF_SHIFT                      (9U)
#define LPI2C_MCR_RRF_WIDTH                      (1U)
#define LPI2C_MCR_RRF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MCR_RRF_SHIFT)) & LPI2C_MCR_RRF_MASK)
/*! @} */

/*! @name MSR - Master Status Register */
/*! @{ */
#define LPI2C_MSR_TDF_MASK                       (0x1U)
#define LPI2C_MSR_TDF_SHIFT                      (0U)
#define LPI2C_MSR_TDF_WIDTH                      (1U)
#define LPI2C_MSR_TDF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_TDF_SHIFT)) & LPI2C_MSR_TDF_MASK)
#define LPI2C_MSR_RDF_MASK                       (0x2U)
#define LPI2C_MSR_RDF_SHIFT                      (1U)
#define LPI2C_MSR_RDF_WIDTH                      (1U)
#define LPI2C_MSR_RDF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_RDF_SHIFT)) & LPI2C_MSR_RDF_MASK)
#define LPI2C_MSR_EPF_MASK                       (0x100U)
#define LPI2C_MSR_EPF_SHIFT                      (8U)
#define LPI2C_MSR_EPF_WIDTH                      (1U)
#define LPI2C_MSR_EPF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_EPF_SHIFT)) & LPI2C_MSR_EPF_MASK)
#define LPI2C_MSR_SDF_MASK                       (0x200U)
#define LPI2C_MSR_SDF_SHIFT                      (9U)
#define LPI2C_MSR_SDF_WIDTH                      (1U)
#define LPI2C_MSR_SDF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_SDF_SHIFT)) & LPI2C_MSR_SDF_MASK)
#define LPI2C_MSR_NDF_MASK                       (0x400U)
#define LPI2C_MSR_NDF_SHIFT                      (10U)
#define LPI2C_MSR_NDF_WIDTH                      (1U)
#define LPI2C_MSR_NDF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_NDF_SHIFT)) & LPI2C_MSR_NDF_MASK)
#define LPI2C_MSR_ALF_MASK                       (0x800U)
#define LPI2C_MSR_ALF_SHIFT                      (11U)
#define LPI2C_MSR_ALF_WIDTH                      (1U)
#define LPI2C_MSR_ALF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_ALF_SHIFT)) & LPI2C_MSR_ALF_MASK)
#define LPI2C_MSR_FEF_MASK                       (0x1000U)
#define LPI2C_MSR_FEF_SHIFT                      (12U)
#define LPI2C_MSR_FEF_WIDTH                      (1U)
#define LPI2C_MSR_FEF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_FEF_SHIFT)) & LPI2C_MSR_FEF_MASK)
#define LPI2C_MSR_PLTF_MASK                      (0x2000U)
#define LPI2C_MSR_PLTF_SHIFT                     (13U)
#define LPI2C_MSR_PLTF_WIDTH                     (1U)
#define LPI2C_MSR_PLTF(x)                        (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_PLTF_SHIFT)) & LPI2C_MSR_PLTF_MASK)
#define LPI2C_MSR_DMF_MASK                       (0x4000U)
#define LPI2C_MSR_DMF_SHIFT                      (14U)
#define LPI2C_MSR_DMF_WIDTH                      (1U)
#define LPI2C_MSR_DMF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_DMF_SHIFT)) & LPI2C_MSR_DMF_MASK)
#define LPI2C_MSR_MBF_MASK                       (0x1000000U)
#define LPI2C_MSR_MBF_SHIFT                      (24U)
#define LPI2C_MSR_MBF_WIDTH                      (1U)
#define LPI2C_MSR_MBF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_MBF_SHIFT)) & LPI2C_MSR_MBF_MASK)
#define LPI2C_MSR_BBF_MASK                       (0x2000000U)
#define LPI2C_MSR_BBF_SHIFT                      (25U)
#define LPI2C_MSR_BBF_WIDTH                      (1U)
#define LPI2C_MSR_BBF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_MSR_BBF_SHIFT)) & LPI2C_MSR_BBF_MASK)
/*! @} */

/*! @name MIER - Master Interrupt Enable Register */
/*! @{ */
#define LPI2C_MIER_TDIE_MASK                     (0x1U)
#define LPI2C_MIER_TDIE_SHIFT                    (0U)
#define LPI2C_MIER_TDIE_WIDTH                    (1U)
#define LPI2C_MIER_TDIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_TDIE_SHIFT)) & LPI2C_MIER_TDIE_MASK)
#define LPI2C_MIER_RDIE_MASK                     (0x2U)
#define LPI2C_MIER_RDIE_SHIFT                    (1U)
#define LPI2C_MIER_RDIE_WIDTH                    (1U)
#define LPI2C_MIER_RDIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_RDIE_SHIFT)) & LPI2C_MIER_RDIE_MASK)
#define LPI2C_MIER_EPIE_MASK                     (0x100U)
#define LPI2C_MIER_EPIE_SHIFT                    (8U)
#define LPI2C_MIER_EPIE_WIDTH                    (1U)
#define LPI2C_MIER_EPIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_EPIE_SHIFT)) & LPI2C_MIER_EPIE_MASK)
#define LPI2C_MIER_SDIE_MASK                     (0x200U)
#define LPI2C_MIER_SDIE_SHIFT                    (9U)
#define LPI2C_MIER_SDIE_WIDTH                    (1U)
#define LPI2C_MIER_SDIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_SDIE_SHIFT)) & LPI2C_MIER_SDIE_MASK)
#define LPI2C_MIER_NDIE_MASK                     (0x400U)
#define LPI2C_MIER_NDIE_SHIFT                    (10U)
#define LPI2C_MIER_NDIE_WIDTH                    (1U)
#define LPI2C_MIER_NDIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_NDIE_SHIFT)) & LPI2C_MIER_NDIE_MASK)
#define LPI2C_MIER_ALIE_MASK                     (0x800U)
#define LPI2C_MIER_ALIE_SHIFT                    (11U)
#define LPI2C_MIER_ALIE_WIDTH                    (1U)
#define LPI2C_MIER_ALIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_ALIE_SHIFT)) & LPI2C_MIER_ALIE_MASK)
#define LPI2C_MIER_FEIE_MASK                     (0x1000U)
#define LPI2C_MIER_FEIE_SHIFT                    (12U)
#define LPI2C_MIER_FEIE_WIDTH                    (1U)
#define LPI2C_MIER_FEIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_FEIE_SHIFT)) & LPI2C_MIER_FEIE_MASK)
#define LPI2C_MIER_PLTIE_MASK                    (0x2000U)
#define LPI2C_MIER_PLTIE_SHIFT                   (13U)
#define LPI2C_MIER_PLTIE_WIDTH                   (1U)
#define LPI2C_MIER_PLTIE(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_PLTIE_SHIFT)) & LPI2C_MIER_PLTIE_MASK)
#define LPI2C_MIER_DMIE_MASK                     (0x4000U)
#define LPI2C_MIER_DMIE_SHIFT                    (14U)
#define LPI2C_MIER_DMIE_WIDTH                    (1U)
#define LPI2C_MIER_DMIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MIER_DMIE_SHIFT)) & LPI2C_MIER_DMIE_MASK)
/*! @} */

/*! @name MDER - Master DMA Enable Register */
/*! @{ */
#define LPI2C_MDER_TDDE_MASK                     (0x1U)
#define LPI2C_MDER_TDDE_SHIFT                    (0U)
#define LPI2C_MDER_TDDE_WIDTH                    (1U)
#define LPI2C_MDER_TDDE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MDER_TDDE_SHIFT)) & LPI2C_MDER_TDDE_MASK)
#define LPI2C_MDER_RDDE_MASK                     (0x2U)
#define LPI2C_MDER_RDDE_SHIFT                    (1U)
#define LPI2C_MDER_RDDE_WIDTH                    (1U)
#define LPI2C_MDER_RDDE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MDER_RDDE_SHIFT)) & LPI2C_MDER_RDDE_MASK)
/*! @} */

/*! @name MCFGR0 - Master Configuration Register 0 */
/*! @{ */
#define LPI2C_MCFGR0_HREN_MASK                   (0x1U)
#define LPI2C_MCFGR0_HREN_SHIFT                  (0U)
#define LPI2C_MCFGR0_HREN_WIDTH                  (1U)
#define LPI2C_MCFGR0_HREN(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR0_HREN_SHIFT)) & LPI2C_MCFGR0_HREN_MASK)
#define LPI2C_MCFGR0_HRPOL_MASK                  (0x2U)
#define LPI2C_MCFGR0_HRPOL_SHIFT                 (1U)
#define LPI2C_MCFGR0_HRPOL_WIDTH                 (1U)
#define LPI2C_MCFGR0_HRPOL(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR0_HRPOL_SHIFT)) & LPI2C_MCFGR0_HRPOL_MASK)
#define LPI2C_MCFGR0_HRSEL_MASK                  (0x4U)
#define LPI2C_MCFGR0_HRSEL_SHIFT                 (2U)
#define LPI2C_MCFGR0_HRSEL_WIDTH                 (1U)
#define LPI2C_MCFGR0_HRSEL(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR0_HRSEL_SHIFT)) & LPI2C_MCFGR0_HRSEL_MASK)
#define LPI2C_MCFGR0_CIRFIFO_MASK                (0x100U)
#define LPI2C_MCFGR0_CIRFIFO_SHIFT               (8U)
#define LPI2C_MCFGR0_CIRFIFO_WIDTH               (1U)
#define LPI2C_MCFGR0_CIRFIFO(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR0_CIRFIFO_SHIFT)) & LPI2C_MCFGR0_CIRFIFO_MASK)
#define LPI2C_MCFGR0_RDMO_MASK                   (0x200U)
#define LPI2C_MCFGR0_RDMO_SHIFT                  (9U)
#define LPI2C_MCFGR0_RDMO_WIDTH                  (1U)
#define LPI2C_MCFGR0_RDMO(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR0_RDMO_SHIFT)) & LPI2C_MCFGR0_RDMO_MASK)
/*! @} */

/*! @name MCFGR1 - Master Configuration Register 1 */
/*! @{ */
#define LPI2C_MCFGR1_PRESCALE_MASK               (0x7U)
#define LPI2C_MCFGR1_PRESCALE_SHIFT              (0U)
#define LPI2C_MCFGR1_PRESCALE_WIDTH              (3U)
#define LPI2C_MCFGR1_PRESCALE(x)                 (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR1_PRESCALE_SHIFT)) & LPI2C_MCFGR1_PRESCALE_MASK)
#define LPI2C_MCFGR1_AUTOSTOP_MASK               (0x100U)
#define LPI2C_MCFGR1_AUTOSTOP_SHIFT              (8U)
#define LPI2C_MCFGR1_AUTOSTOP_WIDTH              (1U)
#define LPI2C_MCFGR1_AUTOSTOP(x)                 (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR1_AUTOSTOP_SHIFT)) & LPI2C_MCFGR1_AUTOSTOP_MASK)
#define LPI2C_MCFGR1_IGNACK_MASK                 (0x200U)
#define LPI2C_MCFGR1_IGNACK_SHIFT                (9U)
#define LPI2C_MCFGR1_IGNACK_WIDTH                (1U)
#define LPI2C_MCFGR1_IGNACK(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR1_IGNACK_SHIFT)) & LPI2C_MCFGR1_IGNACK_MASK)
#define LPI2C_MCFGR1_TIMECFG_MASK                (0x400U)
#define LPI2C_MCFGR1_TIMECFG_SHIFT               (10U)
#define LPI2C_MCFGR1_TIMECFG_WIDTH               (1U)
#define LPI2C_MCFGR1_TIMECFG(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR1_TIMECFG_SHIFT)) & LPI2C_MCFGR1_TIMECFG_MASK)
#define LPI2C_MCFGR1_MATCFG_MASK                 (0x70000U)
#define LPI2C_MCFGR1_MATCFG_SHIFT                (16U)
#define LPI2C_MCFGR1_MATCFG_WIDTH                (3U)
#define LPI2C_MCFGR1_MATCFG(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR1_MATCFG_SHIFT)) & LPI2C_MCFGR1_MATCFG_MASK)
#define LPI2C_MCFGR1_PINCFG_MASK                 (0x7000000U)
#define LPI2C_MCFGR1_PINCFG_SHIFT                (24U)
#define LPI2C_MCFGR1_PINCFG_WIDTH                (3U)
#define LPI2C_MCFGR1_PINCFG(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR1_PINCFG_SHIFT)) & LPI2C_MCFGR1_PINCFG_MASK)
/*! @} */

/*! @name MCFGR2 - Master Configuration Register 2 */
/*! @{ */
#define LPI2C_MCFGR2_BUSIDLE_MASK                (0xFFFU)
#define LPI2C_MCFGR2_BUSIDLE_SHIFT               (0U)
#define LPI2C_MCFGR2_BUSIDLE_WIDTH               (12U)
#define LPI2C_MCFGR2_BUSIDLE(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR2_BUSIDLE_SHIFT)) & LPI2C_MCFGR2_BUSIDLE_MASK)
#define LPI2C_MCFGR2_FILTSCL_MASK                (0xF0000U)
#define LPI2C_MCFGR2_FILTSCL_SHIFT               (16U)
#define LPI2C_MCFGR2_FILTSCL_WIDTH               (4U)
#define LPI2C_MCFGR2_FILTSCL(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR2_FILTSCL_SHIFT)) & LPI2C_MCFGR2_FILTSCL_MASK)
#define LPI2C_MCFGR2_FILTSDA_MASK                (0xF000000U)
#define LPI2C_MCFGR2_FILTSDA_SHIFT               (24U)
#define LPI2C_MCFGR2_FILTSDA_WIDTH               (4U)
#define LPI2C_MCFGR2_FILTSDA(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR2_FILTSDA_SHIFT)) & LPI2C_MCFGR2_FILTSDA_MASK)
/*! @} */

/*! @name MCFGR3 - Master Configuration Register 3 */
/*! @{ */
#define LPI2C_MCFGR3_PINLOW_MASK                 (0xFFF00U)
#define LPI2C_MCFGR3_PINLOW_SHIFT                (8U)
#define LPI2C_MCFGR3_PINLOW_WIDTH                (12U)
#define LPI2C_MCFGR3_PINLOW(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_MCFGR3_PINLOW_SHIFT)) & LPI2C_MCFGR3_PINLOW_MASK)
/*! @} */

/*! @name MDMR - Master Data Match Register */
/*! @{ */
#define LPI2C_MDMR_MATCH0_MASK                   (0xFFU)
#define LPI2C_MDMR_MATCH0_SHIFT                  (0U)
#define LPI2C_MDMR_MATCH0_WIDTH                  (8U)
#define LPI2C_MDMR_MATCH0(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_MDMR_MATCH0_SHIFT)) & LPI2C_MDMR_MATCH0_MASK)
#define LPI2C_MDMR_MATCH1_MASK                   (0xFF0000U)
#define LPI2C_MDMR_MATCH1_SHIFT                  (16U)
#define LPI2C_MDMR_MATCH1_WIDTH                  (8U)
#define LPI2C_MDMR_MATCH1(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_MDMR_MATCH1_SHIFT)) & LPI2C_MDMR_MATCH1_MASK)
/*! @} */

/*! @name MCCR0 - Master Clock Configuration Register 0 */
/*! @{ */
#define LPI2C_MCCR0_CLKLO_MASK                   (0x3FU)
#define LPI2C_MCCR0_CLKLO_SHIFT                  (0U)
#define LPI2C_MCCR0_CLKLO_WIDTH                  (6U)
#define LPI2C_MCCR0_CLKLO(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_MCCR0_CLKLO_SHIFT)) & LPI2C_MCCR0_CLKLO_MASK)
#define LPI2C_MCCR0_CLKHI_MASK                   (0x3F00U)
#define LPI2C_MCCR0_CLKHI_SHIFT                  (8U)
#define LPI2C_MCCR0_CLKHI_WIDTH                  (6U)
#define LPI2C_MCCR0_CLKHI(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_MCCR0_CLKHI_SHIFT)) & LPI2C_MCCR0_CLKHI_MASK)
#define LPI2C_MCCR0_SETHOLD_MASK                 (0x3F0000U)
#define LPI2C_MCCR0_SETHOLD_SHIFT                (16U)
#define LPI2C_MCCR0_SETHOLD_WIDTH                (6U)
#define LPI2C_MCCR0_SETHOLD(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_MCCR0_SETHOLD_SHIFT)) & LPI2C_MCCR0_SETHOLD_MASK)
#define LPI2C_MCCR0_DATAVD_MASK                  (0x3F000000U)
#define LPI2C_MCCR0_DATAVD_SHIFT                 (24U)
#define LPI2C_MCCR0_DATAVD_WIDTH                 (6U)
#define LPI2C_MCCR0_DATAVD(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MCCR0_DATAVD_SHIFT)) & LPI2C_MCCR0_DATAVD_MASK)
/*! @} */

/*! @name MCCR1 - Master Clock Configuration Register 1 */
/*! @{ */
#define LPI2C_MCCR1_CLKLO_MASK                   (0x3FU)
#define LPI2C_MCCR1_CLKLO_SHIFT                  (0U)
#define LPI2C_MCCR1_CLKLO_WIDTH                  (6U)
#define LPI2C_MCCR1_CLKLO(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_MCCR1_CLKLO_SHIFT)) & LPI2C_MCCR1_CLKLO_MASK)
#define LPI2C_MCCR1_CLKHI_MASK                   (0x3F00U)
#define LPI2C_MCCR1_CLKHI_SHIFT                  (8U)
#define LPI2C_MCCR1_CLKHI_WIDTH                  (6U)
#define LPI2C_MCCR1_CLKHI(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_MCCR1_CLKHI_SHIFT)) & LPI2C_MCCR1_CLKHI_MASK)
#define LPI2C_MCCR1_SETHOLD_MASK                 (0x3F0000U)
#define LPI2C_MCCR1_SETHOLD_SHIFT                (16U)
#define LPI2C_MCCR1_SETHOLD_WIDTH                (6U)
#define LPI2C_MCCR1_SETHOLD(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_MCCR1_SETHOLD_SHIFT)) & LPI2C_MCCR1_SETHOLD_MASK)
#define LPI2C_MCCR1_DATAVD_MASK                  (0x3F000000U)
#define LPI2C_MCCR1_DATAVD_SHIFT                 (24U)
#define LPI2C_MCCR1_DATAVD_WIDTH                 (6U)
#define LPI2C_MCCR1_DATAVD(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MCCR1_DATAVD_SHIFT)) & LPI2C_MCCR1_DATAVD_MASK)
/*! @} */

/*! @name MFCR - Master FIFO Control Register */
/*! @{ */
#define LPI2C_MFCR_TXWATER_MASK                  (0x3U)
#define LPI2C_MFCR_TXWATER_SHIFT                 (0U)
#define LPI2C_MFCR_TXWATER_WIDTH                 (2U)
#define LPI2C_MFCR_TXWATER(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MFCR_TXWATER_SHIFT)) & LPI2C_MFCR_TXWATER_MASK)
#define LPI2C_MFCR_RXWATER_MASK                  (0x30000U)
#define LPI2C_MFCR_RXWATER_SHIFT                 (16U)
#define LPI2C_MFCR_RXWATER_WIDTH                 (2U)
#define LPI2C_MFCR_RXWATER(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MFCR_RXWATER_SHIFT)) & LPI2C_MFCR_RXWATER_MASK)
/*! @} */

/*! @name MFSR - Master FIFO Status Register */
/*! @{ */
#define LPI2C_MFSR_TXCOUNT_MASK                  (0x7U)
#define LPI2C_MFSR_TXCOUNT_SHIFT                 (0U)
#define LPI2C_MFSR_TXCOUNT_WIDTH                 (3U)
#define LPI2C_MFSR_TXCOUNT(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MFSR_TXCOUNT_SHIFT)) & LPI2C_MFSR_TXCOUNT_MASK)
#define LPI2C_MFSR_RXCOUNT_MASK                  (0x70000U)
#define LPI2C_MFSR_RXCOUNT_SHIFT                 (16U)
#define LPI2C_MFSR_RXCOUNT_WIDTH                 (3U)
#define LPI2C_MFSR_RXCOUNT(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MFSR_RXCOUNT_SHIFT)) & LPI2C_MFSR_RXCOUNT_MASK)
/*! @} */

/*! @name MTDR - Master Transmit Data Register */
/*! @{ */
#define LPI2C_MTDR_DATA_MASK                     (0xFFU)
#define LPI2C_MTDR_DATA_SHIFT                    (0U)
#define LPI2C_MTDR_DATA_WIDTH                    (8U)
#define LPI2C_MTDR_DATA(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MTDR_DATA_SHIFT)) & LPI2C_MTDR_DATA_MASK)
#define LPI2C_MTDR_CMD_MASK                      (0x700U)
#define LPI2C_MTDR_CMD_SHIFT                     (8U)
#define LPI2C_MTDR_CMD_WIDTH                     (3U)
#define LPI2C_MTDR_CMD(x)                        (((uint32_t)(((uint32_t)(x)) << LPI2C_MTDR_CMD_SHIFT)) & LPI2C_MTDR_CMD_MASK)
/*! @} */

/*! @name MRDR - Master Receive Data Register */
/*! @{ */
#define LPI2C_MRDR_DATA_MASK                     (0xFFU)
#define LPI2C_MRDR_DATA_SHIFT                    (0U)
#define LPI2C_MRDR_DATA_WIDTH                    (8U)
#define LPI2C_MRDR_DATA(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_MRDR_DATA_SHIFT)) & LPI2C_MRDR_DATA_MASK)
#define LPI2C_MRDR_RXEMPTY_MASK                  (0x4000U)
#define LPI2C_MRDR_RXEMPTY_SHIFT                 (14U)
#define LPI2C_MRDR_RXEMPTY_WIDTH                 (1U)
#define LPI2C_MRDR_RXEMPTY(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_MRDR_RXEMPTY_SHIFT)) & LPI2C_MRDR_RXEMPTY_MASK)
/*! @} */

/*! @name SCR - Slave Control Register */
/*! @{ */
#define LPI2C_SCR_SEN_MASK                       (0x1U)
#define LPI2C_SCR_SEN_SHIFT                      (0U)
#define LPI2C_SCR_SEN_WIDTH                      (1U)
#define LPI2C_SCR_SEN(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SCR_SEN_SHIFT)) & LPI2C_SCR_SEN_MASK)
#define LPI2C_SCR_RST_MASK                       (0x2U)
#define LPI2C_SCR_RST_SHIFT                      (1U)
#define LPI2C_SCR_RST_WIDTH                      (1U)
#define LPI2C_SCR_RST(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SCR_RST_SHIFT)) & LPI2C_SCR_RST_MASK)
#define LPI2C_SCR_FILTEN_MASK                    (0x10U)
#define LPI2C_SCR_FILTEN_SHIFT                   (4U)
#define LPI2C_SCR_FILTEN_WIDTH                   (1U)
#define LPI2C_SCR_FILTEN(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_SCR_FILTEN_SHIFT)) & LPI2C_SCR_FILTEN_MASK)
#define LPI2C_SCR_FILTDZ_MASK                    (0x20U)
#define LPI2C_SCR_FILTDZ_SHIFT                   (5U)
#define LPI2C_SCR_FILTDZ_WIDTH                   (1U)
#define LPI2C_SCR_FILTDZ(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_SCR_FILTDZ_SHIFT)) & LPI2C_SCR_FILTDZ_MASK)
#define LPI2C_SCR_RTF_MASK                       (0x100U)
#define LPI2C_SCR_RTF_SHIFT                      (8U)
#define LPI2C_SCR_RTF_WIDTH                      (1U)
#define LPI2C_SCR_RTF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SCR_RTF_SHIFT)) & LPI2C_SCR_RTF_MASK)
#define LPI2C_SCR_RRF_MASK                       (0x200U)
#define LPI2C_SCR_RRF_SHIFT                      (9U)
#define LPI2C_SCR_RRF_WIDTH                      (1U)
#define LPI2C_SCR_RRF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SCR_RRF_SHIFT)) & LPI2C_SCR_RRF_MASK)
/*! @} */

/*! @name SSR - Slave Status Register */
/*! @{ */
#define LPI2C_SSR_TDF_MASK                       (0x1U)
#define LPI2C_SSR_TDF_SHIFT                      (0U)
#define LPI2C_SSR_TDF_WIDTH                      (1U)
#define LPI2C_SSR_TDF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_TDF_SHIFT)) & LPI2C_SSR_TDF_MASK)
#define LPI2C_SSR_RDF_MASK                       (0x2U)
#define LPI2C_SSR_RDF_SHIFT                      (1U)
#define LPI2C_SSR_RDF_WIDTH                      (1U)
#define LPI2C_SSR_RDF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_RDF_SHIFT)) & LPI2C_SSR_RDF_MASK)
#define LPI2C_SSR_AVF_MASK                       (0x4U)
#define LPI2C_SSR_AVF_SHIFT                      (2U)
#define LPI2C_SSR_AVF_WIDTH                      (1U)
#define LPI2C_SSR_AVF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_AVF_SHIFT)) & LPI2C_SSR_AVF_MASK)
#define LPI2C_SSR_TAF_MASK                       (0x8U)
#define LPI2C_SSR_TAF_SHIFT                      (3U)
#define LPI2C_SSR_TAF_WIDTH                      (1U)
#define LPI2C_SSR_TAF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_TAF_SHIFT)) & LPI2C_SSR_TAF_MASK)
#define LPI2C_SSR_RSF_MASK                       (0x100U)
#define LPI2C_SSR_RSF_SHIFT                      (8U)
#define LPI2C_SSR_RSF_WIDTH                      (1U)
#define LPI2C_SSR_RSF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_RSF_SHIFT)) & LPI2C_SSR_RSF_MASK)
#define LPI2C_SSR_SDF_MASK                       (0x200U)
#define LPI2C_SSR_SDF_SHIFT                      (9U)
#define LPI2C_SSR_SDF_WIDTH                      (1U)
#define LPI2C_SSR_SDF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_SDF_SHIFT)) & LPI2C_SSR_SDF_MASK)
#define LPI2C_SSR_BEF_MASK                       (0x400U)
#define LPI2C_SSR_BEF_SHIFT                      (10U)
#define LPI2C_SSR_BEF_WIDTH                      (1U)
#define LPI2C_SSR_BEF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_BEF_SHIFT)) & LPI2C_SSR_BEF_MASK)
#define LPI2C_SSR_FEF_MASK                       (0x800U)
#define LPI2C_SSR_FEF_SHIFT                      (11U)
#define LPI2C_SSR_FEF_WIDTH                      (1U)
#define LPI2C_SSR_FEF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_FEF_SHIFT)) & LPI2C_SSR_FEF_MASK)
#define LPI2C_SSR_AM0F_MASK                      (0x1000U)
#define LPI2C_SSR_AM0F_SHIFT                     (12U)
#define LPI2C_SSR_AM0F_WIDTH                     (1U)
#define LPI2C_SSR_AM0F(x)                        (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_AM0F_SHIFT)) & LPI2C_SSR_AM0F_MASK)
#define LPI2C_SSR_AM1F_MASK                      (0x2000U)
#define LPI2C_SSR_AM1F_SHIFT                     (13U)
#define LPI2C_SSR_AM1F_WIDTH                     (1U)
#define LPI2C_SSR_AM1F(x)                        (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_AM1F_SHIFT)) & LPI2C_SSR_AM1F_MASK)
#define LPI2C_SSR_GCF_MASK                       (0x4000U)
#define LPI2C_SSR_GCF_SHIFT                      (14U)
#define LPI2C_SSR_GCF_WIDTH                      (1U)
#define LPI2C_SSR_GCF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_GCF_SHIFT)) & LPI2C_SSR_GCF_MASK)
#define LPI2C_SSR_SARF_MASK                      (0x8000U)
#define LPI2C_SSR_SARF_SHIFT                     (15U)
#define LPI2C_SSR_SARF_WIDTH                     (1U)
#define LPI2C_SSR_SARF(x)                        (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_SARF_SHIFT)) & LPI2C_SSR_SARF_MASK)
#define LPI2C_SSR_SBF_MASK                       (0x1000000U)
#define LPI2C_SSR_SBF_SHIFT                      (24U)
#define LPI2C_SSR_SBF_WIDTH                      (1U)
#define LPI2C_SSR_SBF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_SBF_SHIFT)) & LPI2C_SSR_SBF_MASK)
#define LPI2C_SSR_BBF_MASK                       (0x2000000U)
#define LPI2C_SSR_BBF_SHIFT                      (25U)
#define LPI2C_SSR_BBF_WIDTH                      (1U)
#define LPI2C_SSR_BBF(x)                         (((uint32_t)(((uint32_t)(x)) << LPI2C_SSR_BBF_SHIFT)) & LPI2C_SSR_BBF_MASK)
/*! @} */

/*! @name SIER - Slave Interrupt Enable Register */
/*! @{ */
#define LPI2C_SIER_TDIE_MASK                     (0x1U)
#define LPI2C_SIER_TDIE_SHIFT                    (0U)
#define LPI2C_SIER_TDIE_WIDTH                    (1U)
#define LPI2C_SIER_TDIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_TDIE_SHIFT)) & LPI2C_SIER_TDIE_MASK)
#define LPI2C_SIER_RDIE_MASK                     (0x2U)
#define LPI2C_SIER_RDIE_SHIFT                    (1U)
#define LPI2C_SIER_RDIE_WIDTH                    (1U)
#define LPI2C_SIER_RDIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_RDIE_SHIFT)) & LPI2C_SIER_RDIE_MASK)
#define LPI2C_SIER_AVIE_MASK                     (0x4U)
#define LPI2C_SIER_AVIE_SHIFT                    (2U)
#define LPI2C_SIER_AVIE_WIDTH                    (1U)
#define LPI2C_SIER_AVIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_AVIE_SHIFT)) & LPI2C_SIER_AVIE_MASK)
#define LPI2C_SIER_TAIE_MASK                     (0x8U)
#define LPI2C_SIER_TAIE_SHIFT                    (3U)
#define LPI2C_SIER_TAIE_WIDTH                    (1U)
#define LPI2C_SIER_TAIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_TAIE_SHIFT)) & LPI2C_SIER_TAIE_MASK)
#define LPI2C_SIER_RSIE_MASK                     (0x100U)
#define LPI2C_SIER_RSIE_SHIFT                    (8U)
#define LPI2C_SIER_RSIE_WIDTH                    (1U)
#define LPI2C_SIER_RSIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_RSIE_SHIFT)) & LPI2C_SIER_RSIE_MASK)
#define LPI2C_SIER_SDIE_MASK                     (0x200U)
#define LPI2C_SIER_SDIE_SHIFT                    (9U)
#define LPI2C_SIER_SDIE_WIDTH                    (1U)
#define LPI2C_SIER_SDIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_SDIE_SHIFT)) & LPI2C_SIER_SDIE_MASK)
#define LPI2C_SIER_BEIE_MASK                     (0x400U)
#define LPI2C_SIER_BEIE_SHIFT                    (10U)
#define LPI2C_SIER_BEIE_WIDTH                    (1U)
#define LPI2C_SIER_BEIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_BEIE_SHIFT)) & LPI2C_SIER_BEIE_MASK)
#define LPI2C_SIER_FEIE_MASK                     (0x800U)
#define LPI2C_SIER_FEIE_SHIFT                    (11U)
#define LPI2C_SIER_FEIE_WIDTH                    (1U)
#define LPI2C_SIER_FEIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_FEIE_SHIFT)) & LPI2C_SIER_FEIE_MASK)
#define LPI2C_SIER_AM0IE_MASK                    (0x1000U)
#define LPI2C_SIER_AM0IE_SHIFT                   (12U)
#define LPI2C_SIER_AM0IE_WIDTH                   (1U)
#define LPI2C_SIER_AM0IE(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_AM0IE_SHIFT)) & LPI2C_SIER_AM0IE_MASK)
#define LPI2C_SIER_AM1F_MASK                     (0x2000U)
#define LPI2C_SIER_AM1F_SHIFT                    (13U)
#define LPI2C_SIER_AM1F_WIDTH                    (1U)
#define LPI2C_SIER_AM1F(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_AM1F_SHIFT)) & LPI2C_SIER_AM1F_MASK)
#define LPI2C_SIER_GCIE_MASK                     (0x4000U)
#define LPI2C_SIER_GCIE_SHIFT                    (14U)
#define LPI2C_SIER_GCIE_WIDTH                    (1U)
#define LPI2C_SIER_GCIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_GCIE_SHIFT)) & LPI2C_SIER_GCIE_MASK)
#define LPI2C_SIER_SARIE_MASK                    (0x8000U)
#define LPI2C_SIER_SARIE_SHIFT                   (15U)
#define LPI2C_SIER_SARIE_WIDTH                   (1U)
#define LPI2C_SIER_SARIE(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_SIER_SARIE_SHIFT)) & LPI2C_SIER_SARIE_MASK)
/*! @} */

/*! @name SDER - Slave DMA Enable Register */
/*! @{ */
#define LPI2C_SDER_TDDE_MASK                     (0x1U)
#define LPI2C_SDER_TDDE_SHIFT                    (0U)
#define LPI2C_SDER_TDDE_WIDTH                    (1U)
#define LPI2C_SDER_TDDE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SDER_TDDE_SHIFT)) & LPI2C_SDER_TDDE_MASK)
#define LPI2C_SDER_RDDE_MASK                     (0x2U)
#define LPI2C_SDER_RDDE_SHIFT                    (1U)
#define LPI2C_SDER_RDDE_WIDTH                    (1U)
#define LPI2C_SDER_RDDE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SDER_RDDE_SHIFT)) & LPI2C_SDER_RDDE_MASK)
#define LPI2C_SDER_AVDE_MASK                     (0x4U)
#define LPI2C_SDER_AVDE_SHIFT                    (2U)
#define LPI2C_SDER_AVDE_WIDTH                    (1U)
#define LPI2C_SDER_AVDE(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SDER_AVDE_SHIFT)) & LPI2C_SDER_AVDE_MASK)
/*! @} */

/*! @name SCFGR1 - Slave Configuration Register 1 */
/*! @{ */
#define LPI2C_SCFGR1_ADRSTALL_MASK               (0x1U)
#define LPI2C_SCFGR1_ADRSTALL_SHIFT              (0U)
#define LPI2C_SCFGR1_ADRSTALL_WIDTH              (1U)
#define LPI2C_SCFGR1_ADRSTALL(x)                 (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_ADRSTALL_SHIFT)) & LPI2C_SCFGR1_ADRSTALL_MASK)
#define LPI2C_SCFGR1_RXSTALL_MASK                (0x2U)
#define LPI2C_SCFGR1_RXSTALL_SHIFT               (1U)
#define LPI2C_SCFGR1_RXSTALL_WIDTH               (1U)
#define LPI2C_SCFGR1_RXSTALL(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_RXSTALL_SHIFT)) & LPI2C_SCFGR1_RXSTALL_MASK)
#define LPI2C_SCFGR1_TXDSTALL_MASK               (0x4U)
#define LPI2C_SCFGR1_TXDSTALL_SHIFT              (2U)
#define LPI2C_SCFGR1_TXDSTALL_WIDTH              (1U)
#define LPI2C_SCFGR1_TXDSTALL(x)                 (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_TXDSTALL_SHIFT)) & LPI2C_SCFGR1_TXDSTALL_MASK)
#define LPI2C_SCFGR1_ACKSTALL_MASK               (0x8U)
#define LPI2C_SCFGR1_ACKSTALL_SHIFT              (3U)
#define LPI2C_SCFGR1_ACKSTALL_WIDTH              (1U)
#define LPI2C_SCFGR1_ACKSTALL(x)                 (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_ACKSTALL_SHIFT)) & LPI2C_SCFGR1_ACKSTALL_MASK)
#define LPI2C_SCFGR1_GCEN_MASK                   (0x100U)
#define LPI2C_SCFGR1_GCEN_SHIFT                  (8U)
#define LPI2C_SCFGR1_GCEN_WIDTH                  (1U)
#define LPI2C_SCFGR1_GCEN(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_GCEN_SHIFT)) & LPI2C_SCFGR1_GCEN_MASK)
#define LPI2C_SCFGR1_SAEN_MASK                   (0x200U)
#define LPI2C_SCFGR1_SAEN_SHIFT                  (9U)
#define LPI2C_SCFGR1_SAEN_WIDTH                  (1U)
#define LPI2C_SCFGR1_SAEN(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_SAEN_SHIFT)) & LPI2C_SCFGR1_SAEN_MASK)
#define LPI2C_SCFGR1_TXCFG_MASK                  (0x400U)
#define LPI2C_SCFGR1_TXCFG_SHIFT                 (10U)
#define LPI2C_SCFGR1_TXCFG_WIDTH                 (1U)
#define LPI2C_SCFGR1_TXCFG(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_TXCFG_SHIFT)) & LPI2C_SCFGR1_TXCFG_MASK)
#define LPI2C_SCFGR1_RXCFG_MASK                  (0x800U)
#define LPI2C_SCFGR1_RXCFG_SHIFT                 (11U)
#define LPI2C_SCFGR1_RXCFG_WIDTH                 (1U)
#define LPI2C_SCFGR1_RXCFG(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_RXCFG_SHIFT)) & LPI2C_SCFGR1_RXCFG_MASK)
#define LPI2C_SCFGR1_IGNACK_MASK                 (0x1000U)
#define LPI2C_SCFGR1_IGNACK_SHIFT                (12U)
#define LPI2C_SCFGR1_IGNACK_WIDTH                (1U)
#define LPI2C_SCFGR1_IGNACK(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_IGNACK_SHIFT)) & LPI2C_SCFGR1_IGNACK_MASK)
#define LPI2C_SCFGR1_HSMEN_MASK                  (0x2000U)
#define LPI2C_SCFGR1_HSMEN_SHIFT                 (13U)
#define LPI2C_SCFGR1_HSMEN_WIDTH                 (1U)
#define LPI2C_SCFGR1_HSMEN(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_HSMEN_SHIFT)) & LPI2C_SCFGR1_HSMEN_MASK)
#define LPI2C_SCFGR1_ADDRCFG_MASK                (0x70000U)
#define LPI2C_SCFGR1_ADDRCFG_SHIFT               (16U)
#define LPI2C_SCFGR1_ADDRCFG_WIDTH               (3U)
#define LPI2C_SCFGR1_ADDRCFG(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR1_ADDRCFG_SHIFT)) & LPI2C_SCFGR1_ADDRCFG_MASK)
/*! @} */

/*! @name SCFGR2 - Slave Configuration Register 2 */
/*! @{ */
#define LPI2C_SCFGR2_CLKHOLD_MASK                (0xFU)
#define LPI2C_SCFGR2_CLKHOLD_SHIFT               (0U)
#define LPI2C_SCFGR2_CLKHOLD_WIDTH               (4U)
#define LPI2C_SCFGR2_CLKHOLD(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR2_CLKHOLD_SHIFT)) & LPI2C_SCFGR2_CLKHOLD_MASK)
#define LPI2C_SCFGR2_DATAVD_MASK                 (0x3F00U)
#define LPI2C_SCFGR2_DATAVD_SHIFT                (8U)
#define LPI2C_SCFGR2_DATAVD_WIDTH                (6U)
#define LPI2C_SCFGR2_DATAVD(x)                   (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR2_DATAVD_SHIFT)) & LPI2C_SCFGR2_DATAVD_MASK)
#define LPI2C_SCFGR2_FILTSCL_MASK                (0xF0000U)
#define LPI2C_SCFGR2_FILTSCL_SHIFT               (16U)
#define LPI2C_SCFGR2_FILTSCL_WIDTH               (4U)
#define LPI2C_SCFGR2_FILTSCL(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR2_FILTSCL_SHIFT)) & LPI2C_SCFGR2_FILTSCL_MASK)
#define LPI2C_SCFGR2_FILTSDA_MASK                (0xF000000U)
#define LPI2C_SCFGR2_FILTSDA_SHIFT               (24U)
#define LPI2C_SCFGR2_FILTSDA_WIDTH               (4U)
#define LPI2C_SCFGR2_FILTSDA(x)                  (((uint32_t)(((uint32_t)(x)) << LPI2C_SCFGR2_FILTSDA_SHIFT)) & LPI2C_SCFGR2_FILTSDA_MASK)
/*! @} */

/*! @name SAMR - Slave Address Match Register */
/*! @{ */
#define LPI2C_SAMR_ADDR0_MASK                    (0x7FEU)
#define LPI2C_SAMR_ADDR0_SHIFT                   (1U)
#define LPI2C_SAMR_ADDR0_WIDTH                   (10U)
#define LPI2C_SAMR_ADDR0(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_SAMR_ADDR0_SHIFT)) & LPI2C_SAMR_ADDR0_MASK)
#define LPI2C_SAMR_ADDR1_MASK                    (0x7FE0000U)
#define LPI2C_SAMR_ADDR1_SHIFT                   (17U)
#define LPI2C_SAMR_ADDR1_WIDTH                   (10U)
#define LPI2C_SAMR_ADDR1(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_SAMR_ADDR1_SHIFT)) & LPI2C_SAMR_ADDR1_MASK)
/*! @} */

/*! @name SASR - Slave Address Status Register */
/*! @{ */
#define LPI2C_SASR_RADDR_MASK                    (0x7FFU)
#define LPI2C_SASR_RADDR_SHIFT                   (0U)
#define LPI2C_SASR_RADDR_WIDTH                   (11U)
#define LPI2C_SASR_RADDR(x)                      (((uint32_t)(((uint32_t)(x)) << LPI2C_SASR_RADDR_SHIFT)) & LPI2C_SASR_RADDR_MASK)
#define LPI2C_SASR_ANV_MASK                      (0x4000U)
#define LPI2C_SASR_ANV_SHIFT                     (14U)
#define LPI2C_SASR_ANV_WIDTH                     (1U)
#define LPI2C_SASR_ANV(x)                        (((uint32_t)(((uint32_t)(x)) << LPI2C_SASR_ANV_SHIFT)) & LPI2C_SASR_ANV_MASK)
/*! @} */

/*! @name STAR - Slave Transmit ACK Register */
/*! @{ */
#define LPI2C_STAR_TXNACK_MASK                   (0x1U)
#define LPI2C_STAR_TXNACK_SHIFT                  (0U)
#define LPI2C_STAR_TXNACK_WIDTH                  (1U)
#define LPI2C_STAR_TXNACK(x)                     (((uint32_t)(((uint32_t)(x)) << LPI2C_STAR_TXNACK_SHIFT)) & LPI2C_STAR_TXNACK_MASK)
/*! @} */

/*! @name STDR - Slave Transmit Data Register */
/*! @{ */
#define LPI2C_STDR_DATA_MASK                     (0xFFU)
#define LPI2C_STDR_DATA_SHIFT                    (0U)
#define LPI2C_STDR_DATA_WIDTH                    (8U)
#define LPI2C_STDR_DATA(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_STDR_DATA_SHIFT)) & LPI2C_STDR_DATA_MASK)
/*! @} */

/*! @name SRDR - Slave Receive Data Register */
/*! @{ */
#define LPI2C_SRDR_DATA_MASK                     (0xFFU)
#define LPI2C_SRDR_DATA_SHIFT                    (0U)
#define LPI2C_SRDR_DATA_WIDTH                    (8U)
#define LPI2C_SRDR_DATA(x)                       (((uint32_t)(((uint32_t)(x)) << LPI2C_SRDR_DATA_SHIFT)) & LPI2C_SRDR_DATA_MASK)
#define LPI2C_SRDR_RXEMPTY_MASK                  (0x4000U)
#define LPI2C_SRDR_RXEMPTY_SHIFT                 (14U)
#define LPI2C_SRDR_RXEMPTY_WIDTH                 (1U)
#define LPI2C_SRDR_RXEMPTY(x)                    (((uint32_t)(((uint32_t)(x)) << LPI2C_SRDR_RXEMPTY_SHIFT)) & LPI2C_SRDR_RXEMPTY_MASK)
#define LPI2C_SRDR_SOF_MASK                      (0x8000U)
#define LPI2C_SRDR_SOF_SHIFT                     (15U)
#define LPI2C_SRDR_SOF_WIDTH                     (1U)
#define LPI2C_SRDR_SOF(x)                        (((uint32_t)(((uint32_t)(x)) << LPI2C_SRDR_SOF_SHIFT)) & LPI2C_SRDR_SOF_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group LPI2C_Register_Masks */

/*!
 * @}
 */ /* end of group LPI2C_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- LPIT Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPIT_Peripheral_Access_Layer LPIT Peripheral Access Layer
 * @{
 */

/** LPIT - Size of Registers Arrays */
#define LPIT_TMR_COUNT                            4u

/** LPIT - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID Register, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< Parameter Register, offset: 0x4 */
  __IO uint32_t MCR;                               /**< Module Control Register, offset: 0x8 */
  __IO uint32_t MSR;                               /**< Module Status Register, offset: 0xC */
  __IO uint32_t MIER;                              /**< Module Interrupt Enable Register, offset: 0x10 */
  __IO uint32_t SETTEN;                            /**< Set Timer Enable Register, offset: 0x14 */
  __O  uint32_t CLRTEN;                            /**< Clear Timer Enable Register, offset: 0x18 */
  uint8_t RESERVED_0[4];
  struct {                                         /* offset: 0x20, array step: 0x10 */
    __IO uint32_t TVAL;                              /**< Timer Value Register, array offset: 0x20, array step: 0x10 */
    __I  uint32_t CVAL;                              /**< Current Timer Value, array offset: 0x24, array step: 0x10 */
    __IO uint32_t TCTRL;                             /**< Timer Control Register, array offset: 0x28, array step: 0x10 */
    uint8_t RESERVED_0[4];
  } TMR[LPIT_TMR_COUNT];
} LPIT_Type, *LPIT_MemMapPtr;

/** Number of instances of the LPIT module. */
#define LPIT_INSTANCE_COUNT                      (1u)

/* LPIT - Peripheral instance base addresses */
/** Peripheral LPIT0 base address */
#define LPIT0_BASE                               (0x40037000u)
/** Peripheral LPIT0 base pointer */
#define LPIT0                                    ((LPIT_Type *)LPIT0_BASE)
/** Array initializer of LPIT peripheral base addresses */
#define LPIT_BASE_ADDRS                          { LPIT0_BASE }
/** Array initializer of LPIT peripheral base pointers */
#define LPIT_BASE_PTRS                           { LPIT0 }

/* ----------------------------------------------------------------------------
   -- LPIT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPIT_Register_Masks LPIT Register Masks
 * @{
 */

/*! @name VERID - Version ID Register */
/*! @{ */
#define LPIT_VERID_FEATURE_MASK                  (0xFFFFU)
#define LPIT_VERID_FEATURE_SHIFT                 (0U)
#define LPIT_VERID_FEATURE_WIDTH                 (16U)
#define LPIT_VERID_FEATURE(x)                    (((uint32_t)(((uint32_t)(x)) << LPIT_VERID_FEATURE_SHIFT)) & LPIT_VERID_FEATURE_MASK)
#define LPIT_VERID_MINOR_MASK                    (0xFF0000U)
#define LPIT_VERID_MINOR_SHIFT                   (16U)
#define LPIT_VERID_MINOR_WIDTH                   (8U)
#define LPIT_VERID_MINOR(x)                      (((uint32_t)(((uint32_t)(x)) << LPIT_VERID_MINOR_SHIFT)) & LPIT_VERID_MINOR_MASK)
#define LPIT_VERID_MAJOR_MASK                    (0xFF000000U)
#define LPIT_VERID_MAJOR_SHIFT                   (24U)
#define LPIT_VERID_MAJOR_WIDTH                   (8U)
#define LPIT_VERID_MAJOR(x)                      (((uint32_t)(((uint32_t)(x)) << LPIT_VERID_MAJOR_SHIFT)) & LPIT_VERID_MAJOR_MASK)
/*! @} */

/*! @name PARAM - Parameter Register */
/*! @{ */
#define LPIT_PARAM_CHANNEL_MASK                  (0xFFU)
#define LPIT_PARAM_CHANNEL_SHIFT                 (0U)
#define LPIT_PARAM_CHANNEL_WIDTH                 (8U)
#define LPIT_PARAM_CHANNEL(x)                    (((uint32_t)(((uint32_t)(x)) << LPIT_PARAM_CHANNEL_SHIFT)) & LPIT_PARAM_CHANNEL_MASK)
#define LPIT_PARAM_EXT_TRIG_MASK                 (0xFF00U)
#define LPIT_PARAM_EXT_TRIG_SHIFT                (8U)
#define LPIT_PARAM_EXT_TRIG_WIDTH                (8U)
#define LPIT_PARAM_EXT_TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << LPIT_PARAM_EXT_TRIG_SHIFT)) & LPIT_PARAM_EXT_TRIG_MASK)
/*! @} */

/*! @name MCR - Module Control Register */
/*! @{ */
#define LPIT_MCR_M_CEN_MASK                      (0x1U)
#define LPIT_MCR_M_CEN_SHIFT                     (0U)
#define LPIT_MCR_M_CEN_WIDTH                     (1U)
#define LPIT_MCR_M_CEN(x)                        (((uint32_t)(((uint32_t)(x)) << LPIT_MCR_M_CEN_SHIFT)) & LPIT_MCR_M_CEN_MASK)
#define LPIT_MCR_SW_RST_MASK                     (0x2U)
#define LPIT_MCR_SW_RST_SHIFT                    (1U)
#define LPIT_MCR_SW_RST_WIDTH                    (1U)
#define LPIT_MCR_SW_RST(x)                       (((uint32_t)(((uint32_t)(x)) << LPIT_MCR_SW_RST_SHIFT)) & LPIT_MCR_SW_RST_MASK)
#define LPIT_MCR_DOZE_EN_MASK                    (0x4U)
#define LPIT_MCR_DOZE_EN_SHIFT                   (2U)
#define LPIT_MCR_DOZE_EN_WIDTH                   (1U)
#define LPIT_MCR_DOZE_EN(x)                      (((uint32_t)(((uint32_t)(x)) << LPIT_MCR_DOZE_EN_SHIFT)) & LPIT_MCR_DOZE_EN_MASK)
#define LPIT_MCR_DBG_EN_MASK                     (0x8U)
#define LPIT_MCR_DBG_EN_SHIFT                    (3U)
#define LPIT_MCR_DBG_EN_WIDTH                    (1U)
#define LPIT_MCR_DBG_EN(x)                       (((uint32_t)(((uint32_t)(x)) << LPIT_MCR_DBG_EN_SHIFT)) & LPIT_MCR_DBG_EN_MASK)
/*! @} */

/*! @name MSR - Module Status Register */
/*! @{ */
#define LPIT_MSR_TIF0_MASK                       (0x1U)
#define LPIT_MSR_TIF0_SHIFT                      (0U)
#define LPIT_MSR_TIF0_WIDTH                      (1U)
#define LPIT_MSR_TIF0(x)                         (((uint32_t)(((uint32_t)(x)) << LPIT_MSR_TIF0_SHIFT)) & LPIT_MSR_TIF0_MASK)
#define LPIT_MSR_TIF1_MASK                       (0x2U)
#define LPIT_MSR_TIF1_SHIFT                      (1U)
#define LPIT_MSR_TIF1_WIDTH                      (1U)
#define LPIT_MSR_TIF1(x)                         (((uint32_t)(((uint32_t)(x)) << LPIT_MSR_TIF1_SHIFT)) & LPIT_MSR_TIF1_MASK)
#define LPIT_MSR_TIF2_MASK                       (0x4U)
#define LPIT_MSR_TIF2_SHIFT                      (2U)
#define LPIT_MSR_TIF2_WIDTH                      (1U)
#define LPIT_MSR_TIF2(x)                         (((uint32_t)(((uint32_t)(x)) << LPIT_MSR_TIF2_SHIFT)) & LPIT_MSR_TIF2_MASK)
#define LPIT_MSR_TIF3_MASK                       (0x8U)
#define LPIT_MSR_TIF3_SHIFT                      (3U)
#define LPIT_MSR_TIF3_WIDTH                      (1U)
#define LPIT_MSR_TIF3(x)                         (((uint32_t)(((uint32_t)(x)) << LPIT_MSR_TIF3_SHIFT)) & LPIT_MSR_TIF3_MASK)
/*! @} */

/*! @name MIER - Module Interrupt Enable Register */
/*! @{ */
#define LPIT_MIER_TIE0_MASK                      (0x1U)
#define LPIT_MIER_TIE0_SHIFT                     (0U)
#define LPIT_MIER_TIE0_WIDTH                     (1U)
#define LPIT_MIER_TIE0(x)                        (((uint32_t)(((uint32_t)(x)) << LPIT_MIER_TIE0_SHIFT)) & LPIT_MIER_TIE0_MASK)
#define LPIT_MIER_TIE1_MASK                      (0x2U)
#define LPIT_MIER_TIE1_SHIFT                     (1U)
#define LPIT_MIER_TIE1_WIDTH                     (1U)
#define LPIT_MIER_TIE1(x)                        (((uint32_t)(((uint32_t)(x)) << LPIT_MIER_TIE1_SHIFT)) & LPIT_MIER_TIE1_MASK)
#define LPIT_MIER_TIE2_MASK                      (0x4U)
#define LPIT_MIER_TIE2_SHIFT                     (2U)
#define LPIT_MIER_TIE2_WIDTH                     (1U)
#define LPIT_MIER_TIE2(x)                        (((uint32_t)(((uint32_t)(x)) << LPIT_MIER_TIE2_SHIFT)) & LPIT_MIER_TIE2_MASK)
#define LPIT_MIER_TIE3_MASK                      (0x8U)
#define LPIT_MIER_TIE3_SHIFT                     (3U)
#define LPIT_MIER_TIE3_WIDTH                     (1U)
#define LPIT_MIER_TIE3(x)                        (((uint32_t)(((uint32_t)(x)) << LPIT_MIER_TIE3_SHIFT)) & LPIT_MIER_TIE3_MASK)
/*! @} */

/*! @name SETTEN - Set Timer Enable Register */
/*! @{ */
#define LPIT_SETTEN_SET_T_EN_0_MASK              (0x1U)
#define LPIT_SETTEN_SET_T_EN_0_SHIFT             (0U)
#define LPIT_SETTEN_SET_T_EN_0_WIDTH             (1U)
#define LPIT_SETTEN_SET_T_EN_0(x)                (((uint32_t)(((uint32_t)(x)) << LPIT_SETTEN_SET_T_EN_0_SHIFT)) & LPIT_SETTEN_SET_T_EN_0_MASK)
#define LPIT_SETTEN_SET_T_EN_1_MASK              (0x2U)
#define LPIT_SETTEN_SET_T_EN_1_SHIFT             (1U)
#define LPIT_SETTEN_SET_T_EN_1_WIDTH             (1U)
#define LPIT_SETTEN_SET_T_EN_1(x)                (((uint32_t)(((uint32_t)(x)) << LPIT_SETTEN_SET_T_EN_1_SHIFT)) & LPIT_SETTEN_SET_T_EN_1_MASK)
#define LPIT_SETTEN_SET_T_EN_2_MASK              (0x4U)
#define LPIT_SETTEN_SET_T_EN_2_SHIFT             (2U)
#define LPIT_SETTEN_SET_T_EN_2_WIDTH             (1U)
#define LPIT_SETTEN_SET_T_EN_2(x)                (((uint32_t)(((uint32_t)(x)) << LPIT_SETTEN_SET_T_EN_2_SHIFT)) & LPIT_SETTEN_SET_T_EN_2_MASK)
#define LPIT_SETTEN_SET_T_EN_3_MASK              (0x8U)
#define LPIT_SETTEN_SET_T_EN_3_SHIFT             (3U)
#define LPIT_SETTEN_SET_T_EN_3_WIDTH             (1U)
#define LPIT_SETTEN_SET_T_EN_3(x)                (((uint32_t)(((uint32_t)(x)) << LPIT_SETTEN_SET_T_EN_3_SHIFT)) & LPIT_SETTEN_SET_T_EN_3_MASK)
/*! @} */

/*! @name CLRTEN - Clear Timer Enable Register */
/*! @{ */
#define LPIT_CLRTEN_CLR_T_EN_0_MASK              (0x1U)
#define LPIT_CLRTEN_CLR_T_EN_0_SHIFT             (0U)
#define LPIT_CLRTEN_CLR_T_EN_0_WIDTH             (1U)
#define LPIT_CLRTEN_CLR_T_EN_0(x)                (((uint32_t)(((uint32_t)(x)) << LPIT_CLRTEN_CLR_T_EN_0_SHIFT)) & LPIT_CLRTEN_CLR_T_EN_0_MASK)
#define LPIT_CLRTEN_CLR_T_EN_1_MASK              (0x2U)
#define LPIT_CLRTEN_CLR_T_EN_1_SHIFT             (1U)
#define LPIT_CLRTEN_CLR_T_EN_1_WIDTH             (1U)
#define LPIT_CLRTEN_CLR_T_EN_1(x)                (((uint32_t)(((uint32_t)(x)) << LPIT_CLRTEN_CLR_T_EN_1_SHIFT)) & LPIT_CLRTEN_CLR_T_EN_1_MASK)
#define LPIT_CLRTEN_CLR_T_EN_2_MASK              (0x4U)
#define LPIT_CLRTEN_CLR_T_EN_2_SHIFT             (2U)
#define LPIT_CLRTEN_CLR_T_EN_2_WIDTH             (1U)
#define LPIT_CLRTEN_CLR_T_EN_2(x)                (((uint32_t)(((uint32_t)(x)) << LPIT_CLRTEN_CLR_T_EN_2_SHIFT)) & LPIT_CLRTEN_CLR_T_EN_2_MASK)
#define LPIT_CLRTEN_CLR_T_EN_3_MASK              (0x8U)
#define LPIT_CLRTEN_CLR_T_EN_3_SHIFT             (3U)
#define LPIT_CLRTEN_CLR_T_EN_3_WIDTH             (1U)
#define LPIT_CLRTEN_CLR_T_EN_3(x)                (((uint32_t)(((uint32_t)(x)) << LPIT_CLRTEN_CLR_T_EN_3_SHIFT)) & LPIT_CLRTEN_CLR_T_EN_3_MASK)
/*! @} */

/*! @name TMR_TVAL - Timer Value Register */
/*! @{ */
#define LPIT_TMR_TVAL_TMR_VAL_MASK               (0xFFFFFFFFU)
#define LPIT_TMR_TVAL_TMR_VAL_SHIFT              (0U)
#define LPIT_TMR_TVAL_TMR_VAL_WIDTH              (32U)
#define LPIT_TMR_TVAL_TMR_VAL(x)                 (((uint32_t)(((uint32_t)(x)) << LPIT_TMR_TVAL_TMR_VAL_SHIFT)) & LPIT_TMR_TVAL_TMR_VAL_MASK)
/*! @} */

/*! @name TMR_CVAL - Current Timer Value */
/*! @{ */
#define LPIT_TMR_CVAL_TMR_CUR_VAL_MASK           (0xFFFFFFFFU)
#define LPIT_TMR_CVAL_TMR_CUR_VAL_SHIFT          (0U)
#define LPIT_TMR_CVAL_TMR_CUR_VAL_WIDTH          (32U)
#define LPIT_TMR_CVAL_TMR_CUR_VAL(x)             (((uint32_t)(((uint32_t)(x)) << LPIT_TMR_CVAL_TMR_CUR_VAL_SHIFT)) & LPIT_TMR_CVAL_TMR_CUR_VAL_MASK)
/*! @} */

/*! @name TMR_TCTRL - Timer Control Register */
/*! @{ */
#define LPIT_TMR_TCTRL_T_EN_MASK                 (0x1U)
#define LPIT_TMR_TCTRL_T_EN_SHIFT                (0U)
#define LPIT_TMR_TCTRL_T_EN_WIDTH                (1U)
#define LPIT_TMR_TCTRL_T_EN(x)                   (((uint32_t)(((uint32_t)(x)) << LPIT_TMR_TCTRL_T_EN_SHIFT)) & LPIT_TMR_TCTRL_T_EN_MASK)
#define LPIT_TMR_TCTRL_CHAIN_MASK                (0x2U)
#define LPIT_TMR_TCTRL_CHAIN_SHIFT               (1U)
#define LPIT_TMR_TCTRL_CHAIN_WIDTH               (1U)
#define LPIT_TMR_TCTRL_CHAIN(x)                  (((uint32_t)(((uint32_t)(x)) << LPIT_TMR_TCTRL_CHAIN_SHIFT)) & LPIT_TMR_TCTRL_CHAIN_MASK)
#define LPIT_TMR_TCTRL_MODE_MASK                 (0xCU)
#define LPIT_TMR_TCTRL_MODE_SHIFT                (2U)
#define LPIT_TMR_TCTRL_MODE_WIDTH                (2U)
#define LPIT_TMR_TCTRL_MODE(x)                   (((uint32_t)(((uint32_t)(x)) << LPIT_TMR_TCTRL_MODE_SHIFT)) & LPIT_TMR_TCTRL_MODE_MASK)
#define LPIT_TMR_TCTRL_TSOT_MASK                 (0x10000U)
#define LPIT_TMR_TCTRL_TSOT_SHIFT                (16U)
#define LPIT_TMR_TCTRL_TSOT_WIDTH                (1U)
#define LPIT_TMR_TCTRL_TSOT(x)                   (((uint32_t)(((uint32_t)(x)) << LPIT_TMR_TCTRL_TSOT_SHIFT)) & LPIT_TMR_TCTRL_TSOT_MASK)
#define LPIT_TMR_TCTRL_TSOI_MASK                 (0x20000U)
#define LPIT_TMR_TCTRL_TSOI_SHIFT                (17U)
#define LPIT_TMR_TCTRL_TSOI_WIDTH                (1U)
#define LPIT_TMR_TCTRL_TSOI(x)                   (((uint32_t)(((uint32_t)(x)) << LPIT_TMR_TCTRL_TSOI_SHIFT)) & LPIT_TMR_TCTRL_TSOI_MASK)
#define LPIT_TMR_TCTRL_TROT_MASK                 (0x40000U)
#define LPIT_TMR_TCTRL_TROT_SHIFT                (18U)
#define LPIT_TMR_TCTRL_TROT_WIDTH                (1U)
#define LPIT_TMR_TCTRL_TROT(x)                   (((uint32_t)(((uint32_t)(x)) << LPIT_TMR_TCTRL_TROT_SHIFT)) & LPIT_TMR_TCTRL_TROT_MASK)
#define LPIT_TMR_TCTRL_TRG_SRC_MASK              (0x800000U)
#define LPIT_TMR_TCTRL_TRG_SRC_SHIFT             (23U)
#define LPIT_TMR_TCTRL_TRG_SRC_WIDTH             (1U)
#define LPIT_TMR_TCTRL_TRG_SRC(x)                (((uint32_t)(((uint32_t)(x)) << LPIT_TMR_TCTRL_TRG_SRC_SHIFT)) & LPIT_TMR_TCTRL_TRG_SRC_MASK)
#define LPIT_TMR_TCTRL_TRG_SEL_MASK              (0xF000000U)
#define LPIT_TMR_TCTRL_TRG_SEL_SHIFT             (24U)
#define LPIT_TMR_TCTRL_TRG_SEL_WIDTH             (4U)
#define LPIT_TMR_TCTRL_TRG_SEL(x)                (((uint32_t)(((uint32_t)(x)) << LPIT_TMR_TCTRL_TRG_SEL_SHIFT)) & LPIT_TMR_TCTRL_TRG_SEL_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group LPIT_Register_Masks */

/*!
 * @}
 */ /* end of group LPIT_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- LPSPI Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPSPI_Peripheral_Access_Layer LPSPI Peripheral Access Layer
 * @{
 */

/** LPSPI - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID Register, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< Parameter Register, offset: 0x4 */
  uint8_t RESERVED_0[8];
  __IO uint32_t CR;                                /**< Control Register, offset: 0x10 */
  __IO uint32_t SR;                                /**< Status Register, offset: 0x14 */
  __IO uint32_t IER;                               /**< Interrupt Enable Register, offset: 0x18 */
  __IO uint32_t DER;                               /**< DMA Enable Register, offset: 0x1C */
  __IO uint32_t CFGR0;                             /**< Configuration Register 0, offset: 0x20 */
  __IO uint32_t CFGR1;                             /**< Configuration Register 1, offset: 0x24 */
  uint8_t RESERVED_1[8];
  __IO uint32_t DMR0;                              /**< Data Match Register 0, offset: 0x30 */
  __IO uint32_t DMR1;                              /**< Data Match Register 1, offset: 0x34 */
  uint8_t RESERVED_2[8];
  __IO uint32_t CCR;                               /**< Clock Configuration Register, offset: 0x40 */
  uint8_t RESERVED_3[20];
  __IO uint32_t FCR;                               /**< FIFO Control Register, offset: 0x58 */
  __I  uint32_t FSR;                               /**< FIFO Status Register, offset: 0x5C */
  __IO uint32_t TCR;                               /**< Transmit Command Register, offset: 0x60 */
  __O  uint32_t TDR;                               /**< Transmit Data Register, offset: 0x64 */
  uint8_t RESERVED_4[8];
  __I  uint32_t RSR;                               /**< Receive Status Register, offset: 0x70 */
  __I  uint32_t RDR;                               /**< Receive Data Register, offset: 0x74 */
} LPSPI_Type, *LPSPI_MemMapPtr;

/** Number of instances of the LPSPI module. */
#define LPSPI_INSTANCE_COUNT                     (3u)

/* LPSPI - Peripheral instance base addresses */
/** Peripheral LPSPI0 base address */
#define LPSPI0_BASE                              (0x4002C000u)
/** Peripheral LPSPI0 base pointer */
#define LPSPI0                                   ((LPSPI_Type *)LPSPI0_BASE)
/** Peripheral LPSPI1 base address */
#define LPSPI1_BASE                              (0x4002D000u)
/** Peripheral LPSPI1 base pointer */
#define LPSPI1                                   ((LPSPI_Type *)LPSPI1_BASE)
/** Peripheral LPSPI2 base address */
#define LPSPI2_BASE                              (0x4002E000u)
/** Peripheral LPSPI2 base pointer */
#define LPSPI2                                   ((LPSPI_Type *)LPSPI2_BASE)
/** Array initializer of LPSPI peripheral base addresses */
#define LPSPI_BASE_ADDRS                         { LPSPI0_BASE, LPSPI1_BASE, LPSPI2_BASE }
/** Array initializer of LPSPI peripheral base pointers */
#define LPSPI_BASE_PTRS                          { LPSPI0, LPSPI1, LPSPI2 }

/* ----------------------------------------------------------------------------
   -- LPSPI Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPSPI_Register_Masks LPSPI Register Masks
 * @{
 */

/*! @name VERID - Version ID Register */
/*! @{ */
#define LPSPI_VERID_FEATURE_MASK                 (0xFFFFU)
#define LPSPI_VERID_FEATURE_SHIFT                (0U)
#define LPSPI_VERID_FEATURE_WIDTH                (16U)
#define LPSPI_VERID_FEATURE(x)                   (((uint32_t)(((uint32_t)(x)) << LPSPI_VERID_FEATURE_SHIFT)) & LPSPI_VERID_FEATURE_MASK)
#define LPSPI_VERID_MINOR_MASK                   (0xFF0000U)
#define LPSPI_VERID_MINOR_SHIFT                  (16U)
#define LPSPI_VERID_MINOR_WIDTH                  (8U)
#define LPSPI_VERID_MINOR(x)                     (((uint32_t)(((uint32_t)(x)) << LPSPI_VERID_MINOR_SHIFT)) & LPSPI_VERID_MINOR_MASK)
#define LPSPI_VERID_MAJOR_MASK                   (0xFF000000U)
#define LPSPI_VERID_MAJOR_SHIFT                  (24U)
#define LPSPI_VERID_MAJOR_WIDTH                  (8U)
#define LPSPI_VERID_MAJOR(x)                     (((uint32_t)(((uint32_t)(x)) << LPSPI_VERID_MAJOR_SHIFT)) & LPSPI_VERID_MAJOR_MASK)
/*! @} */

/*! @name PARAM - Parameter Register */
/*! @{ */
#define LPSPI_PARAM_TXFIFO_MASK                  (0xFFU)
#define LPSPI_PARAM_TXFIFO_SHIFT                 (0U)
#define LPSPI_PARAM_TXFIFO_WIDTH                 (8U)
#define LPSPI_PARAM_TXFIFO(x)                    (((uint32_t)(((uint32_t)(x)) << LPSPI_PARAM_TXFIFO_SHIFT)) & LPSPI_PARAM_TXFIFO_MASK)
#define LPSPI_PARAM_RXFIFO_MASK                  (0xFF00U)
#define LPSPI_PARAM_RXFIFO_SHIFT                 (8U)
#define LPSPI_PARAM_RXFIFO_WIDTH                 (8U)
#define LPSPI_PARAM_RXFIFO(x)                    (((uint32_t)(((uint32_t)(x)) << LPSPI_PARAM_RXFIFO_SHIFT)) & LPSPI_PARAM_RXFIFO_MASK)
/*! @} */

/*! @name CR - Control Register */
/*! @{ */
#define LPSPI_CR_MEN_MASK                        (0x1U)
#define LPSPI_CR_MEN_SHIFT                       (0U)
#define LPSPI_CR_MEN_WIDTH                       (1U)
#define LPSPI_CR_MEN(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_CR_MEN_SHIFT)) & LPSPI_CR_MEN_MASK)
#define LPSPI_CR_RST_MASK                        (0x2U)
#define LPSPI_CR_RST_SHIFT                       (1U)
#define LPSPI_CR_RST_WIDTH                       (1U)
#define LPSPI_CR_RST(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_CR_RST_SHIFT)) & LPSPI_CR_RST_MASK)
#define LPSPI_CR_DOZEN_MASK                      (0x4U)
#define LPSPI_CR_DOZEN_SHIFT                     (2U)
#define LPSPI_CR_DOZEN_WIDTH                     (1U)
#define LPSPI_CR_DOZEN(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_CR_DOZEN_SHIFT)) & LPSPI_CR_DOZEN_MASK)
#define LPSPI_CR_DBGEN_MASK                      (0x8U)
#define LPSPI_CR_DBGEN_SHIFT                     (3U)
#define LPSPI_CR_DBGEN_WIDTH                     (1U)
#define LPSPI_CR_DBGEN(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_CR_DBGEN_SHIFT)) & LPSPI_CR_DBGEN_MASK)
#define LPSPI_CR_RTF_MASK                        (0x100U)
#define LPSPI_CR_RTF_SHIFT                       (8U)
#define LPSPI_CR_RTF_WIDTH                       (1U)
#define LPSPI_CR_RTF(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_CR_RTF_SHIFT)) & LPSPI_CR_RTF_MASK)
#define LPSPI_CR_RRF_MASK                        (0x200U)
#define LPSPI_CR_RRF_SHIFT                       (9U)
#define LPSPI_CR_RRF_WIDTH                       (1U)
#define LPSPI_CR_RRF(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_CR_RRF_SHIFT)) & LPSPI_CR_RRF_MASK)
/*! @} */

/*! @name SR - Status Register */
/*! @{ */
#define LPSPI_SR_TDF_MASK                        (0x1U)
#define LPSPI_SR_TDF_SHIFT                       (0U)
#define LPSPI_SR_TDF_WIDTH                       (1U)
#define LPSPI_SR_TDF(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_SR_TDF_SHIFT)) & LPSPI_SR_TDF_MASK)
#define LPSPI_SR_RDF_MASK                        (0x2U)
#define LPSPI_SR_RDF_SHIFT                       (1U)
#define LPSPI_SR_RDF_WIDTH                       (1U)
#define LPSPI_SR_RDF(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_SR_RDF_SHIFT)) & LPSPI_SR_RDF_MASK)
#define LPSPI_SR_WCF_MASK                        (0x100U)
#define LPSPI_SR_WCF_SHIFT                       (8U)
#define LPSPI_SR_WCF_WIDTH                       (1U)
#define LPSPI_SR_WCF(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_SR_WCF_SHIFT)) & LPSPI_SR_WCF_MASK)
#define LPSPI_SR_FCF_MASK                        (0x200U)
#define LPSPI_SR_FCF_SHIFT                       (9U)
#define LPSPI_SR_FCF_WIDTH                       (1U)
#define LPSPI_SR_FCF(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_SR_FCF_SHIFT)) & LPSPI_SR_FCF_MASK)
#define LPSPI_SR_TCF_MASK                        (0x400U)
#define LPSPI_SR_TCF_SHIFT                       (10U)
#define LPSPI_SR_TCF_WIDTH                       (1U)
#define LPSPI_SR_TCF(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_SR_TCF_SHIFT)) & LPSPI_SR_TCF_MASK)
#define LPSPI_SR_TEF_MASK                        (0x800U)
#define LPSPI_SR_TEF_SHIFT                       (11U)
#define LPSPI_SR_TEF_WIDTH                       (1U)
#define LPSPI_SR_TEF(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_SR_TEF_SHIFT)) & LPSPI_SR_TEF_MASK)
#define LPSPI_SR_REF_MASK                        (0x1000U)
#define LPSPI_SR_REF_SHIFT                       (12U)
#define LPSPI_SR_REF_WIDTH                       (1U)
#define LPSPI_SR_REF(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_SR_REF_SHIFT)) & LPSPI_SR_REF_MASK)
#define LPSPI_SR_DMF_MASK                        (0x2000U)
#define LPSPI_SR_DMF_SHIFT                       (13U)
#define LPSPI_SR_DMF_WIDTH                       (1U)
#define LPSPI_SR_DMF(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_SR_DMF_SHIFT)) & LPSPI_SR_DMF_MASK)
#define LPSPI_SR_MBF_MASK                        (0x1000000U)
#define LPSPI_SR_MBF_SHIFT                       (24U)
#define LPSPI_SR_MBF_WIDTH                       (1U)
#define LPSPI_SR_MBF(x)                          (((uint32_t)(((uint32_t)(x)) << LPSPI_SR_MBF_SHIFT)) & LPSPI_SR_MBF_MASK)
/*! @} */

/*! @name IER - Interrupt Enable Register */
/*! @{ */
#define LPSPI_IER_TDIE_MASK                      (0x1U)
#define LPSPI_IER_TDIE_SHIFT                     (0U)
#define LPSPI_IER_TDIE_WIDTH                     (1U)
#define LPSPI_IER_TDIE(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_IER_TDIE_SHIFT)) & LPSPI_IER_TDIE_MASK)
#define LPSPI_IER_RDIE_MASK                      (0x2U)
#define LPSPI_IER_RDIE_SHIFT                     (1U)
#define LPSPI_IER_RDIE_WIDTH                     (1U)
#define LPSPI_IER_RDIE(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_IER_RDIE_SHIFT)) & LPSPI_IER_RDIE_MASK)
#define LPSPI_IER_WCIE_MASK                      (0x100U)
#define LPSPI_IER_WCIE_SHIFT                     (8U)
#define LPSPI_IER_WCIE_WIDTH                     (1U)
#define LPSPI_IER_WCIE(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_IER_WCIE_SHIFT)) & LPSPI_IER_WCIE_MASK)
#define LPSPI_IER_FCIE_MASK                      (0x200U)
#define LPSPI_IER_FCIE_SHIFT                     (9U)
#define LPSPI_IER_FCIE_WIDTH                     (1U)
#define LPSPI_IER_FCIE(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_IER_FCIE_SHIFT)) & LPSPI_IER_FCIE_MASK)
#define LPSPI_IER_TCIE_MASK                      (0x400U)
#define LPSPI_IER_TCIE_SHIFT                     (10U)
#define LPSPI_IER_TCIE_WIDTH                     (1U)
#define LPSPI_IER_TCIE(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_IER_TCIE_SHIFT)) & LPSPI_IER_TCIE_MASK)
#define LPSPI_IER_TEIE_MASK                      (0x800U)
#define LPSPI_IER_TEIE_SHIFT                     (11U)
#define LPSPI_IER_TEIE_WIDTH                     (1U)
#define LPSPI_IER_TEIE(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_IER_TEIE_SHIFT)) & LPSPI_IER_TEIE_MASK)
#define LPSPI_IER_REIE_MASK                      (0x1000U)
#define LPSPI_IER_REIE_SHIFT                     (12U)
#define LPSPI_IER_REIE_WIDTH                     (1U)
#define LPSPI_IER_REIE(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_IER_REIE_SHIFT)) & LPSPI_IER_REIE_MASK)
#define LPSPI_IER_DMIE_MASK                      (0x2000U)
#define LPSPI_IER_DMIE_SHIFT                     (13U)
#define LPSPI_IER_DMIE_WIDTH                     (1U)
#define LPSPI_IER_DMIE(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_IER_DMIE_SHIFT)) & LPSPI_IER_DMIE_MASK)
/*! @} */

/*! @name DER - DMA Enable Register */
/*! @{ */
#define LPSPI_DER_TDDE_MASK                      (0x1U)
#define LPSPI_DER_TDDE_SHIFT                     (0U)
#define LPSPI_DER_TDDE_WIDTH                     (1U)
#define LPSPI_DER_TDDE(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_DER_TDDE_SHIFT)) & LPSPI_DER_TDDE_MASK)
#define LPSPI_DER_RDDE_MASK                      (0x2U)
#define LPSPI_DER_RDDE_SHIFT                     (1U)
#define LPSPI_DER_RDDE_WIDTH                     (1U)
#define LPSPI_DER_RDDE(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_DER_RDDE_SHIFT)) & LPSPI_DER_RDDE_MASK)
/*! @} */

/*! @name CFGR0 - Configuration Register 0 */
/*! @{ */
#define LPSPI_CFGR0_HREN_MASK                    (0x1U)
#define LPSPI_CFGR0_HREN_SHIFT                   (0U)
#define LPSPI_CFGR0_HREN_WIDTH                   (1U)
#define LPSPI_CFGR0_HREN(x)                      (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR0_HREN_SHIFT)) & LPSPI_CFGR0_HREN_MASK)
#define LPSPI_CFGR0_HRPOL_MASK                   (0x2U)
#define LPSPI_CFGR0_HRPOL_SHIFT                  (1U)
#define LPSPI_CFGR0_HRPOL_WIDTH                  (1U)
#define LPSPI_CFGR0_HRPOL(x)                     (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR0_HRPOL_SHIFT)) & LPSPI_CFGR0_HRPOL_MASK)
#define LPSPI_CFGR0_HRSEL_MASK                   (0x4U)
#define LPSPI_CFGR0_HRSEL_SHIFT                  (2U)
#define LPSPI_CFGR0_HRSEL_WIDTH                  (1U)
#define LPSPI_CFGR0_HRSEL(x)                     (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR0_HRSEL_SHIFT)) & LPSPI_CFGR0_HRSEL_MASK)
#define LPSPI_CFGR0_CIRFIFO_MASK                 (0x100U)
#define LPSPI_CFGR0_CIRFIFO_SHIFT                (8U)
#define LPSPI_CFGR0_CIRFIFO_WIDTH                (1U)
#define LPSPI_CFGR0_CIRFIFO(x)                   (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR0_CIRFIFO_SHIFT)) & LPSPI_CFGR0_CIRFIFO_MASK)
#define LPSPI_CFGR0_RDMO_MASK                    (0x200U)
#define LPSPI_CFGR0_RDMO_SHIFT                   (9U)
#define LPSPI_CFGR0_RDMO_WIDTH                   (1U)
#define LPSPI_CFGR0_RDMO(x)                      (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR0_RDMO_SHIFT)) & LPSPI_CFGR0_RDMO_MASK)
/*! @} */

/*! @name CFGR1 - Configuration Register 1 */
/*! @{ */
#define LPSPI_CFGR1_MASTER_MASK                  (0x1U)
#define LPSPI_CFGR1_MASTER_SHIFT                 (0U)
#define LPSPI_CFGR1_MASTER_WIDTH                 (1U)
#define LPSPI_CFGR1_MASTER(x)                    (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR1_MASTER_SHIFT)) & LPSPI_CFGR1_MASTER_MASK)
#define LPSPI_CFGR1_SAMPLE_MASK                  (0x2U)
#define LPSPI_CFGR1_SAMPLE_SHIFT                 (1U)
#define LPSPI_CFGR1_SAMPLE_WIDTH                 (1U)
#define LPSPI_CFGR1_SAMPLE(x)                    (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR1_SAMPLE_SHIFT)) & LPSPI_CFGR1_SAMPLE_MASK)
#define LPSPI_CFGR1_AUTOPCS_MASK                 (0x4U)
#define LPSPI_CFGR1_AUTOPCS_SHIFT                (2U)
#define LPSPI_CFGR1_AUTOPCS_WIDTH                (1U)
#define LPSPI_CFGR1_AUTOPCS(x)                   (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR1_AUTOPCS_SHIFT)) & LPSPI_CFGR1_AUTOPCS_MASK)
#define LPSPI_CFGR1_NOSTALL_MASK                 (0x8U)
#define LPSPI_CFGR1_NOSTALL_SHIFT                (3U)
#define LPSPI_CFGR1_NOSTALL_WIDTH                (1U)
#define LPSPI_CFGR1_NOSTALL(x)                   (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR1_NOSTALL_SHIFT)) & LPSPI_CFGR1_NOSTALL_MASK)
#define LPSPI_CFGR1_PCSPOL_MASK                  (0xF00U)
#define LPSPI_CFGR1_PCSPOL_SHIFT                 (8U)
#define LPSPI_CFGR1_PCSPOL_WIDTH                 (4U)
#define LPSPI_CFGR1_PCSPOL(x)                    (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR1_PCSPOL_SHIFT)) & LPSPI_CFGR1_PCSPOL_MASK)
#define LPSPI_CFGR1_MATCFG_MASK                  (0x70000U)
#define LPSPI_CFGR1_MATCFG_SHIFT                 (16U)
#define LPSPI_CFGR1_MATCFG_WIDTH                 (3U)
#define LPSPI_CFGR1_MATCFG(x)                    (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR1_MATCFG_SHIFT)) & LPSPI_CFGR1_MATCFG_MASK)
#define LPSPI_CFGR1_PINCFG_MASK                  (0x3000000U)
#define LPSPI_CFGR1_PINCFG_SHIFT                 (24U)
#define LPSPI_CFGR1_PINCFG_WIDTH                 (2U)
#define LPSPI_CFGR1_PINCFG(x)                    (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR1_PINCFG_SHIFT)) & LPSPI_CFGR1_PINCFG_MASK)
#define LPSPI_CFGR1_OUTCFG_MASK                  (0x4000000U)
#define LPSPI_CFGR1_OUTCFG_SHIFT                 (26U)
#define LPSPI_CFGR1_OUTCFG_WIDTH                 (1U)
#define LPSPI_CFGR1_OUTCFG(x)                    (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR1_OUTCFG_SHIFT)) & LPSPI_CFGR1_OUTCFG_MASK)
#define LPSPI_CFGR1_PCSCFG_MASK                  (0x8000000U)
#define LPSPI_CFGR1_PCSCFG_SHIFT                 (27U)
#define LPSPI_CFGR1_PCSCFG_WIDTH                 (1U)
#define LPSPI_CFGR1_PCSCFG(x)                    (((uint32_t)(((uint32_t)(x)) << LPSPI_CFGR1_PCSCFG_SHIFT)) & LPSPI_CFGR1_PCSCFG_MASK)
/*! @} */

/*! @name DMR0 - Data Match Register 0 */
/*! @{ */
#define LPSPI_DMR0_MATCH0_MASK                   (0xFFFFFFFFU)
#define LPSPI_DMR0_MATCH0_SHIFT                  (0U)
#define LPSPI_DMR0_MATCH0_WIDTH                  (32U)
#define LPSPI_DMR0_MATCH0(x)                     (((uint32_t)(((uint32_t)(x)) << LPSPI_DMR0_MATCH0_SHIFT)) & LPSPI_DMR0_MATCH0_MASK)
/*! @} */

/*! @name DMR1 - Data Match Register 1 */
/*! @{ */
#define LPSPI_DMR1_MATCH1_MASK                   (0xFFFFFFFFU)
#define LPSPI_DMR1_MATCH1_SHIFT                  (0U)
#define LPSPI_DMR1_MATCH1_WIDTH                  (32U)
#define LPSPI_DMR1_MATCH1(x)                     (((uint32_t)(((uint32_t)(x)) << LPSPI_DMR1_MATCH1_SHIFT)) & LPSPI_DMR1_MATCH1_MASK)
/*! @} */

/*! @name CCR - Clock Configuration Register */
/*! @{ */
#define LPSPI_CCR_SCKDIV_MASK                    (0xFFU)
#define LPSPI_CCR_SCKDIV_SHIFT                   (0U)
#define LPSPI_CCR_SCKDIV_WIDTH                   (8U)
#define LPSPI_CCR_SCKDIV(x)                      (((uint32_t)(((uint32_t)(x)) << LPSPI_CCR_SCKDIV_SHIFT)) & LPSPI_CCR_SCKDIV_MASK)
#define LPSPI_CCR_DBT_MASK                       (0xFF00U)
#define LPSPI_CCR_DBT_SHIFT                      (8U)
#define LPSPI_CCR_DBT_WIDTH                      (8U)
#define LPSPI_CCR_DBT(x)                         (((uint32_t)(((uint32_t)(x)) << LPSPI_CCR_DBT_SHIFT)) & LPSPI_CCR_DBT_MASK)
#define LPSPI_CCR_PCSSCK_MASK                    (0xFF0000U)
#define LPSPI_CCR_PCSSCK_SHIFT                   (16U)
#define LPSPI_CCR_PCSSCK_WIDTH                   (8U)
#define LPSPI_CCR_PCSSCK(x)                      (((uint32_t)(((uint32_t)(x)) << LPSPI_CCR_PCSSCK_SHIFT)) & LPSPI_CCR_PCSSCK_MASK)
#define LPSPI_CCR_SCKPCS_MASK                    (0xFF000000U)
#define LPSPI_CCR_SCKPCS_SHIFT                   (24U)
#define LPSPI_CCR_SCKPCS_WIDTH                   (8U)
#define LPSPI_CCR_SCKPCS(x)                      (((uint32_t)(((uint32_t)(x)) << LPSPI_CCR_SCKPCS_SHIFT)) & LPSPI_CCR_SCKPCS_MASK)
/*! @} */

/*! @name FCR - FIFO Control Register */
/*! @{ */
#define LPSPI_FCR_TXWATER_MASK                   (0x3U)
#define LPSPI_FCR_TXWATER_SHIFT                  (0U)
#define LPSPI_FCR_TXWATER_WIDTH                  (2U)
#define LPSPI_FCR_TXWATER(x)                     (((uint32_t)(((uint32_t)(x)) << LPSPI_FCR_TXWATER_SHIFT)) & LPSPI_FCR_TXWATER_MASK)
#define LPSPI_FCR_RXWATER_MASK                   (0x30000U)
#define LPSPI_FCR_RXWATER_SHIFT                  (16U)
#define LPSPI_FCR_RXWATER_WIDTH                  (2U)
#define LPSPI_FCR_RXWATER(x)                     (((uint32_t)(((uint32_t)(x)) << LPSPI_FCR_RXWATER_SHIFT)) & LPSPI_FCR_RXWATER_MASK)
/*! @} */

/*! @name FSR - FIFO Status Register */
/*! @{ */
#define LPSPI_FSR_TXCOUNT_MASK                   (0x7U)
#define LPSPI_FSR_TXCOUNT_SHIFT                  (0U)
#define LPSPI_FSR_TXCOUNT_WIDTH                  (3U)
#define LPSPI_FSR_TXCOUNT(x)                     (((uint32_t)(((uint32_t)(x)) << LPSPI_FSR_TXCOUNT_SHIFT)) & LPSPI_FSR_TXCOUNT_MASK)
#define LPSPI_FSR_RXCOUNT_MASK                   (0x70000U)
#define LPSPI_FSR_RXCOUNT_SHIFT                  (16U)
#define LPSPI_FSR_RXCOUNT_WIDTH                  (3U)
#define LPSPI_FSR_RXCOUNT(x)                     (((uint32_t)(((uint32_t)(x)) << LPSPI_FSR_RXCOUNT_SHIFT)) & LPSPI_FSR_RXCOUNT_MASK)
/*! @} */

/*! @name TCR - Transmit Command Register */
/*! @{ */
#define LPSPI_TCR_FRAMESZ_MASK                   (0xFFFU)
#define LPSPI_TCR_FRAMESZ_SHIFT                  (0U)
#define LPSPI_TCR_FRAMESZ_WIDTH                  (12U)
#define LPSPI_TCR_FRAMESZ(x)                     (((uint32_t)(((uint32_t)(x)) << LPSPI_TCR_FRAMESZ_SHIFT)) & LPSPI_TCR_FRAMESZ_MASK)
#define LPSPI_TCR_WIDTH_MASK                     (0x30000U)
#define LPSPI_TCR_WIDTH_SHIFT                    (16U)
#define LPSPI_TCR_WIDTH_WIDTH                    (2U)
#define LPSPI_TCR_WIDTH(x)                       (((uint32_t)(((uint32_t)(x)) << LPSPI_TCR_WIDTH_SHIFT)) & LPSPI_TCR_WIDTH_MASK)
#define LPSPI_TCR_TXMSK_MASK                     (0x40000U)
#define LPSPI_TCR_TXMSK_SHIFT                    (18U)
#define LPSPI_TCR_TXMSK_WIDTH                    (1U)
#define LPSPI_TCR_TXMSK(x)                       (((uint32_t)(((uint32_t)(x)) << LPSPI_TCR_TXMSK_SHIFT)) & LPSPI_TCR_TXMSK_MASK)
#define LPSPI_TCR_RXMSK_MASK                     (0x80000U)
#define LPSPI_TCR_RXMSK_SHIFT                    (19U)
#define LPSPI_TCR_RXMSK_WIDTH                    (1U)
#define LPSPI_TCR_RXMSK(x)                       (((uint32_t)(((uint32_t)(x)) << LPSPI_TCR_RXMSK_SHIFT)) & LPSPI_TCR_RXMSK_MASK)
#define LPSPI_TCR_CONTC_MASK                     (0x100000U)
#define LPSPI_TCR_CONTC_SHIFT                    (20U)
#define LPSPI_TCR_CONTC_WIDTH                    (1U)
#define LPSPI_TCR_CONTC(x)                       (((uint32_t)(((uint32_t)(x)) << LPSPI_TCR_CONTC_SHIFT)) & LPSPI_TCR_CONTC_MASK)
#define LPSPI_TCR_CONT_MASK                      (0x200000U)
#define LPSPI_TCR_CONT_SHIFT                     (21U)
#define LPSPI_TCR_CONT_WIDTH                     (1U)
#define LPSPI_TCR_CONT(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_TCR_CONT_SHIFT)) & LPSPI_TCR_CONT_MASK)
#define LPSPI_TCR_BYSW_MASK                      (0x400000U)
#define LPSPI_TCR_BYSW_SHIFT                     (22U)
#define LPSPI_TCR_BYSW_WIDTH                     (1U)
#define LPSPI_TCR_BYSW(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_TCR_BYSW_SHIFT)) & LPSPI_TCR_BYSW_MASK)
#define LPSPI_TCR_LSBF_MASK                      (0x800000U)
#define LPSPI_TCR_LSBF_SHIFT                     (23U)
#define LPSPI_TCR_LSBF_WIDTH                     (1U)
#define LPSPI_TCR_LSBF(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_TCR_LSBF_SHIFT)) & LPSPI_TCR_LSBF_MASK)
#define LPSPI_TCR_PCS_MASK                       (0x3000000U)
#define LPSPI_TCR_PCS_SHIFT                      (24U)
#define LPSPI_TCR_PCS_WIDTH                      (2U)
#define LPSPI_TCR_PCS(x)                         (((uint32_t)(((uint32_t)(x)) << LPSPI_TCR_PCS_SHIFT)) & LPSPI_TCR_PCS_MASK)
#define LPSPI_TCR_PRESCALE_MASK                  (0x38000000U)
#define LPSPI_TCR_PRESCALE_SHIFT                 (27U)
#define LPSPI_TCR_PRESCALE_WIDTH                 (3U)
#define LPSPI_TCR_PRESCALE(x)                    (((uint32_t)(((uint32_t)(x)) << LPSPI_TCR_PRESCALE_SHIFT)) & LPSPI_TCR_PRESCALE_MASK)
#define LPSPI_TCR_CPHA_MASK                      (0x40000000U)
#define LPSPI_TCR_CPHA_SHIFT                     (30U)
#define LPSPI_TCR_CPHA_WIDTH                     (1U)
#define LPSPI_TCR_CPHA(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_TCR_CPHA_SHIFT)) & LPSPI_TCR_CPHA_MASK)
#define LPSPI_TCR_CPOL_MASK                      (0x80000000U)
#define LPSPI_TCR_CPOL_SHIFT                     (31U)
#define LPSPI_TCR_CPOL_WIDTH                     (1U)
#define LPSPI_TCR_CPOL(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_TCR_CPOL_SHIFT)) & LPSPI_TCR_CPOL_MASK)
/*! @} */

/*! @name TDR - Transmit Data Register */
/*! @{ */
#define LPSPI_TDR_DATA_MASK                      (0xFFFFFFFFU)
#define LPSPI_TDR_DATA_SHIFT                     (0U)
#define LPSPI_TDR_DATA_WIDTH                     (32U)
#define LPSPI_TDR_DATA(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_TDR_DATA_SHIFT)) & LPSPI_TDR_DATA_MASK)
/*! @} */

/*! @name RSR - Receive Status Register */
/*! @{ */
#define LPSPI_RSR_SOF_MASK                       (0x1U)
#define LPSPI_RSR_SOF_SHIFT                      (0U)
#define LPSPI_RSR_SOF_WIDTH                      (1U)
#define LPSPI_RSR_SOF(x)                         (((uint32_t)(((uint32_t)(x)) << LPSPI_RSR_SOF_SHIFT)) & LPSPI_RSR_SOF_MASK)
#define LPSPI_RSR_RXEMPTY_MASK                   (0x2U)
#define LPSPI_RSR_RXEMPTY_SHIFT                  (1U)
#define LPSPI_RSR_RXEMPTY_WIDTH                  (1U)
#define LPSPI_RSR_RXEMPTY(x)                     (((uint32_t)(((uint32_t)(x)) << LPSPI_RSR_RXEMPTY_SHIFT)) & LPSPI_RSR_RXEMPTY_MASK)
/*! @} */

/*! @name RDR - Receive Data Register */
/*! @{ */
#define LPSPI_RDR_DATA_MASK                      (0xFFFFFFFFU)
#define LPSPI_RDR_DATA_SHIFT                     (0U)
#define LPSPI_RDR_DATA_WIDTH                     (32U)
#define LPSPI_RDR_DATA(x)                        (((uint32_t)(((uint32_t)(x)) << LPSPI_RDR_DATA_SHIFT)) & LPSPI_RDR_DATA_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group LPSPI_Register_Masks */

/*!
 * @}
 */ /* end of group LPSPI_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- LPTMR Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPTMR_Peripheral_Access_Layer LPTMR Peripheral Access Layer
 * @{
 */

/** LPTMR - Register Layout Typedef */
typedef struct {
  __IO uint32_t CSR;                               /**< Low Power Timer Control Status Register, offset: 0x0 */
  __IO uint32_t PSR;                               /**< Low Power Timer Prescale Register, offset: 0x4 */
  __IO uint32_t CMR;                               /**< Low Power Timer Compare Register, offset: 0x8 */
  __IO uint32_t CNR;                               /**< Low Power Timer Counter Register, offset: 0xC */
} LPTMR_Type, *LPTMR_MemMapPtr;

/** Number of instances of the LPTMR module. */
#define LPTMR_INSTANCE_COUNT                     (1u)

/* LPTMR - Peripheral instance base addresses */
/** Peripheral LPTMR0 base address */
#define LPTMR0_BASE                              (0x40040000u)
/** Peripheral LPTMR0 base pointer */
#define LPTMR0                                   ((LPTMR_Type *)LPTMR0_BASE)
/** Array initializer of LPTMR peripheral base addresses */
#define LPTMR_BASE_ADDRS                         { LPTMR0_BASE }
/** Array initializer of LPTMR peripheral base pointers */
#define LPTMR_BASE_PTRS                          { LPTMR0 }

/* ----------------------------------------------------------------------------
   -- LPTMR Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPTMR_Register_Masks LPTMR Register Masks
 * @{
 */

/*! @name CSR - Low Power Timer Control Status Register */
/*! @{ */
#define LPTMR_CSR_TEN_MASK                       (0x1U)
#define LPTMR_CSR_TEN_SHIFT                      (0U)
#define LPTMR_CSR_TEN_WIDTH                      (1U)
#define LPTMR_CSR_TEN(x)                         (((uint32_t)(((uint32_t)(x)) << LPTMR_CSR_TEN_SHIFT)) & LPTMR_CSR_TEN_MASK)
#define LPTMR_CSR_TMS_MASK                       (0x2U)
#define LPTMR_CSR_TMS_SHIFT                      (1U)
#define LPTMR_CSR_TMS_WIDTH                      (1U)
#define LPTMR_CSR_TMS(x)                         (((uint32_t)(((uint32_t)(x)) << LPTMR_CSR_TMS_SHIFT)) & LPTMR_CSR_TMS_MASK)
#define LPTMR_CSR_TFC_MASK                       (0x4U)
#define LPTMR_CSR_TFC_SHIFT                      (2U)
#define LPTMR_CSR_TFC_WIDTH                      (1U)
#define LPTMR_CSR_TFC(x)                         (((uint32_t)(((uint32_t)(x)) << LPTMR_CSR_TFC_SHIFT)) & LPTMR_CSR_TFC_MASK)
#define LPTMR_CSR_TPP_MASK                       (0x8U)
#define LPTMR_CSR_TPP_SHIFT                      (3U)
#define LPTMR_CSR_TPP_WIDTH                      (1U)
#define LPTMR_CSR_TPP(x)                         (((uint32_t)(((uint32_t)(x)) << LPTMR_CSR_TPP_SHIFT)) & LPTMR_CSR_TPP_MASK)
#define LPTMR_CSR_TPS_MASK                       (0x30U)
#define LPTMR_CSR_TPS_SHIFT                      (4U)
#define LPTMR_CSR_TPS_WIDTH                      (2U)
#define LPTMR_CSR_TPS(x)                         (((uint32_t)(((uint32_t)(x)) << LPTMR_CSR_TPS_SHIFT)) & LPTMR_CSR_TPS_MASK)
#define LPTMR_CSR_TIE_MASK                       (0x40U)
#define LPTMR_CSR_TIE_SHIFT                      (6U)
#define LPTMR_CSR_TIE_WIDTH                      (1U)
#define LPTMR_CSR_TIE(x)                         (((uint32_t)(((uint32_t)(x)) << LPTMR_CSR_TIE_SHIFT)) & LPTMR_CSR_TIE_MASK)
#define LPTMR_CSR_TCF_MASK                       (0x80U)
#define LPTMR_CSR_TCF_SHIFT                      (7U)
#define LPTMR_CSR_TCF_WIDTH                      (1U)
#define LPTMR_CSR_TCF(x)                         (((uint32_t)(((uint32_t)(x)) << LPTMR_CSR_TCF_SHIFT)) & LPTMR_CSR_TCF_MASK)
#define LPTMR_CSR_TDRE_MASK                      (0x100U)
#define LPTMR_CSR_TDRE_SHIFT                     (8U)
#define LPTMR_CSR_TDRE_WIDTH                     (1U)
#define LPTMR_CSR_TDRE(x)                        (((uint32_t)(((uint32_t)(x)) << LPTMR_CSR_TDRE_SHIFT)) & LPTMR_CSR_TDRE_MASK)
/*! @} */

/*! @name PSR - Low Power Timer Prescale Register */
/*! @{ */
#define LPTMR_PSR_PCS_MASK                       (0x3U)
#define LPTMR_PSR_PCS_SHIFT                      (0U)
#define LPTMR_PSR_PCS_WIDTH                      (2U)
#define LPTMR_PSR_PCS(x)                         (((uint32_t)(((uint32_t)(x)) << LPTMR_PSR_PCS_SHIFT)) & LPTMR_PSR_PCS_MASK)
#define LPTMR_PSR_PBYP_MASK                      (0x4U)
#define LPTMR_PSR_PBYP_SHIFT                     (2U)
#define LPTMR_PSR_PBYP_WIDTH                     (1U)
#define LPTMR_PSR_PBYP(x)                        (((uint32_t)(((uint32_t)(x)) << LPTMR_PSR_PBYP_SHIFT)) & LPTMR_PSR_PBYP_MASK)
#define LPTMR_PSR_PRESCALE_MASK                  (0x78U)
#define LPTMR_PSR_PRESCALE_SHIFT                 (3U)
#define LPTMR_PSR_PRESCALE_WIDTH                 (4U)
#define LPTMR_PSR_PRESCALE(x)                    (((uint32_t)(((uint32_t)(x)) << LPTMR_PSR_PRESCALE_SHIFT)) & LPTMR_PSR_PRESCALE_MASK)
/*! @} */

/*! @name CMR - Low Power Timer Compare Register */
/*! @{ */
#define LPTMR_CMR_COMPARE_MASK                   (0xFFFFU)
#define LPTMR_CMR_COMPARE_SHIFT                  (0U)
#define LPTMR_CMR_COMPARE_WIDTH                  (16U)
#define LPTMR_CMR_COMPARE(x)                     (((uint32_t)(((uint32_t)(x)) << LPTMR_CMR_COMPARE_SHIFT)) & LPTMR_CMR_COMPARE_MASK)
/*! @} */

/*! @name CNR - Low Power Timer Counter Register */
/*! @{ */
#define LPTMR_CNR_COUNTER_MASK                   (0xFFFFU)
#define LPTMR_CNR_COUNTER_SHIFT                  (0U)
#define LPTMR_CNR_COUNTER_WIDTH                  (16U)
#define LPTMR_CNR_COUNTER(x)                     (((uint32_t)(((uint32_t)(x)) << LPTMR_CNR_COUNTER_SHIFT)) & LPTMR_CNR_COUNTER_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group LPTMR_Register_Masks */

/*!
 * @}
 */ /* end of group LPTMR_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- LPUART Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPUART_Peripheral_Access_Layer LPUART Peripheral Access Layer
 * @{
 */

/** LPUART - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID Register, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< Parameter Register, offset: 0x4 */
  __IO uint32_t GLOBAL;                            /**< LPUART Global Register, offset: 0x8 */
  __IO uint32_t PINCFG;                            /**< LPUART Pin Configuration Register, offset: 0xC */
  __IO uint32_t BAUD;                              /**< LPUART Baud Rate Register, offset: 0x10 */
  __IO uint32_t STAT;                              /**< LPUART Status Register, offset: 0x14 */
  __IO uint32_t CTRL;                              /**< LPUART Control Register, offset: 0x18 */
  __IO uint32_t DATA;                              /**< LPUART Data Register, offset: 0x1C */
  __IO uint32_t MATCH;                             /**< LPUART Match Address Register, offset: 0x20 */
  __IO uint32_t MODIR;                             /**< LPUART Modem IrDA Register, offset: 0x24 */
  __IO uint32_t FIFO;                              /**< LPUART FIFO Register, offset: 0x28 */
  __IO uint32_t WATER;                             /**< LPUART Watermark Register, offset: 0x2C */
} LPUART_Type, *LPUART_MemMapPtr;

/** Number of instances of the LPUART module. */
#define LPUART_INSTANCE_COUNT                    (3u)

/* LPUART - Peripheral instance base addresses */
/** Peripheral LPUART0 base address */
#define LPUART0_BASE                             (0x4006A000u)
/** Peripheral LPUART0 base pointer */
#define LPUART0                                  ((LPUART_Type *)LPUART0_BASE)
/** Peripheral LPUART1 base address */
#define LPUART1_BASE                             (0x4006B000u)
/** Peripheral LPUART1 base pointer */
#define LPUART1                                  ((LPUART_Type *)LPUART1_BASE)
/** Peripheral LPUART2 base address */
#define LPUART2_BASE                             (0x4006C000u)
/** Peripheral LPUART2 base pointer */
#define LPUART2                                  ((LPUART_Type *)LPUART2_BASE)
/** Array initializer of LPUART peripheral base addresses */
#define LPUART_BASE_ADDRS                        { LPUART0_BASE, LPUART1_BASE, LPUART2_BASE }
/** Array initializer of LPUART peripheral base pointers */
#define LPUART_BASE_PTRS                         { LPUART0, LPUART1, LPUART2 }

/* ----------------------------------------------------------------------------
   -- LPUART Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPUART_Register_Masks LPUART Register Masks
 * @{
 */

/*! @name VERID - Version ID Register */
/*! @{ */
#define LPUART_VERID_FEATURE_MASK                (0xFFFFU)
#define LPUART_VERID_FEATURE_SHIFT               (0U)
#define LPUART_VERID_FEATURE_WIDTH               (16U)
#define LPUART_VERID_FEATURE(x)                  (((uint32_t)(((uint32_t)(x)) << LPUART_VERID_FEATURE_SHIFT)) & LPUART_VERID_FEATURE_MASK)
#define LPUART_VERID_MINOR_MASK                  (0xFF0000U)
#define LPUART_VERID_MINOR_SHIFT                 (16U)
#define LPUART_VERID_MINOR_WIDTH                 (8U)
#define LPUART_VERID_MINOR(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_VERID_MINOR_SHIFT)) & LPUART_VERID_MINOR_MASK)
#define LPUART_VERID_MAJOR_MASK                  (0xFF000000U)
#define LPUART_VERID_MAJOR_SHIFT                 (24U)
#define LPUART_VERID_MAJOR_WIDTH                 (8U)
#define LPUART_VERID_MAJOR(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_VERID_MAJOR_SHIFT)) & LPUART_VERID_MAJOR_MASK)
/*! @} */

/*! @name PARAM - Parameter Register */
/*! @{ */
#define LPUART_PARAM_TXFIFO_MASK                 (0xFFU)
#define LPUART_PARAM_TXFIFO_SHIFT                (0U)
#define LPUART_PARAM_TXFIFO_WIDTH                (8U)
#define LPUART_PARAM_TXFIFO(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_PARAM_TXFIFO_SHIFT)) & LPUART_PARAM_TXFIFO_MASK)
#define LPUART_PARAM_RXFIFO_MASK                 (0xFF00U)
#define LPUART_PARAM_RXFIFO_SHIFT                (8U)
#define LPUART_PARAM_RXFIFO_WIDTH                (8U)
#define LPUART_PARAM_RXFIFO(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_PARAM_RXFIFO_SHIFT)) & LPUART_PARAM_RXFIFO_MASK)
/*! @} */

/*! @name GLOBAL - LPUART Global Register */
/*! @{ */
#define LPUART_GLOBAL_RST_MASK                   (0x2U)
#define LPUART_GLOBAL_RST_SHIFT                  (1U)
#define LPUART_GLOBAL_RST_WIDTH                  (1U)
#define LPUART_GLOBAL_RST(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_GLOBAL_RST_SHIFT)) & LPUART_GLOBAL_RST_MASK)
/*! @} */

/*! @name PINCFG - LPUART Pin Configuration Register */
/*! @{ */
#define LPUART_PINCFG_TRGSEL_MASK                (0x3U)
#define LPUART_PINCFG_TRGSEL_SHIFT               (0U)
#define LPUART_PINCFG_TRGSEL_WIDTH               (2U)
#define LPUART_PINCFG_TRGSEL(x)                  (((uint32_t)(((uint32_t)(x)) << LPUART_PINCFG_TRGSEL_SHIFT)) & LPUART_PINCFG_TRGSEL_MASK)
/*! @} */

/*! @name BAUD - LPUART Baud Rate Register */
/*! @{ */
#define LPUART_BAUD_SBR_MASK                     (0x1FFFU)
#define LPUART_BAUD_SBR_SHIFT                    (0U)
#define LPUART_BAUD_SBR_WIDTH                    (13U)
#define LPUART_BAUD_SBR(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_SBR_SHIFT)) & LPUART_BAUD_SBR_MASK)
#define LPUART_BAUD_SBNS_MASK                    (0x2000U)
#define LPUART_BAUD_SBNS_SHIFT                   (13U)
#define LPUART_BAUD_SBNS_WIDTH                   (1U)
#define LPUART_BAUD_SBNS(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_SBNS_SHIFT)) & LPUART_BAUD_SBNS_MASK)
#define LPUART_BAUD_RXEDGIE_MASK                 (0x4000U)
#define LPUART_BAUD_RXEDGIE_SHIFT                (14U)
#define LPUART_BAUD_RXEDGIE_WIDTH                (1U)
#define LPUART_BAUD_RXEDGIE(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_RXEDGIE_SHIFT)) & LPUART_BAUD_RXEDGIE_MASK)
#define LPUART_BAUD_LBKDIE_MASK                  (0x8000U)
#define LPUART_BAUD_LBKDIE_SHIFT                 (15U)
#define LPUART_BAUD_LBKDIE_WIDTH                 (1U)
#define LPUART_BAUD_LBKDIE(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_LBKDIE_SHIFT)) & LPUART_BAUD_LBKDIE_MASK)
#define LPUART_BAUD_RESYNCDIS_MASK               (0x10000U)
#define LPUART_BAUD_RESYNCDIS_SHIFT              (16U)
#define LPUART_BAUD_RESYNCDIS_WIDTH              (1U)
#define LPUART_BAUD_RESYNCDIS(x)                 (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_RESYNCDIS_SHIFT)) & LPUART_BAUD_RESYNCDIS_MASK)
#define LPUART_BAUD_BOTHEDGE_MASK                (0x20000U)
#define LPUART_BAUD_BOTHEDGE_SHIFT               (17U)
#define LPUART_BAUD_BOTHEDGE_WIDTH               (1U)
#define LPUART_BAUD_BOTHEDGE(x)                  (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_BOTHEDGE_SHIFT)) & LPUART_BAUD_BOTHEDGE_MASK)
#define LPUART_BAUD_MATCFG_MASK                  (0xC0000U)
#define LPUART_BAUD_MATCFG_SHIFT                 (18U)
#define LPUART_BAUD_MATCFG_WIDTH                 (2U)
#define LPUART_BAUD_MATCFG(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_MATCFG_SHIFT)) & LPUART_BAUD_MATCFG_MASK)
#define LPUART_BAUD_RIDMAE_MASK                  (0x100000U)
#define LPUART_BAUD_RIDMAE_SHIFT                 (20U)
#define LPUART_BAUD_RIDMAE_WIDTH                 (1U)
#define LPUART_BAUD_RIDMAE(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_RIDMAE_SHIFT)) & LPUART_BAUD_RIDMAE_MASK)
#define LPUART_BAUD_RDMAE_MASK                   (0x200000U)
#define LPUART_BAUD_RDMAE_SHIFT                  (21U)
#define LPUART_BAUD_RDMAE_WIDTH                  (1U)
#define LPUART_BAUD_RDMAE(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_RDMAE_SHIFT)) & LPUART_BAUD_RDMAE_MASK)
#define LPUART_BAUD_TDMAE_MASK                   (0x800000U)
#define LPUART_BAUD_TDMAE_SHIFT                  (23U)
#define LPUART_BAUD_TDMAE_WIDTH                  (1U)
#define LPUART_BAUD_TDMAE(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_TDMAE_SHIFT)) & LPUART_BAUD_TDMAE_MASK)
#define LPUART_BAUD_OSR_MASK                     (0x1F000000U)
#define LPUART_BAUD_OSR_SHIFT                    (24U)
#define LPUART_BAUD_OSR_WIDTH                    (5U)
#define LPUART_BAUD_OSR(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_OSR_SHIFT)) & LPUART_BAUD_OSR_MASK)
#define LPUART_BAUD_M10_MASK                     (0x20000000U)
#define LPUART_BAUD_M10_SHIFT                    (29U)
#define LPUART_BAUD_M10_WIDTH                    (1U)
#define LPUART_BAUD_M10(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_M10_SHIFT)) & LPUART_BAUD_M10_MASK)
#define LPUART_BAUD_MAEN2_MASK                   (0x40000000U)
#define LPUART_BAUD_MAEN2_SHIFT                  (30U)
#define LPUART_BAUD_MAEN2_WIDTH                  (1U)
#define LPUART_BAUD_MAEN2(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_MAEN2_SHIFT)) & LPUART_BAUD_MAEN2_MASK)
#define LPUART_BAUD_MAEN1_MASK                   (0x80000000U)
#define LPUART_BAUD_MAEN1_SHIFT                  (31U)
#define LPUART_BAUD_MAEN1_WIDTH                  (1U)
#define LPUART_BAUD_MAEN1(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_MAEN1_SHIFT)) & LPUART_BAUD_MAEN1_MASK)
/*! @} */

/*! @name STAT - LPUART Status Register */
/*! @{ */
#define LPUART_STAT_MA2F_MASK                    (0x4000U)
#define LPUART_STAT_MA2F_SHIFT                   (14U)
#define LPUART_STAT_MA2F_WIDTH                   (1U)
#define LPUART_STAT_MA2F(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_MA2F_SHIFT)) & LPUART_STAT_MA2F_MASK)
#define LPUART_STAT_MA1F_MASK                    (0x8000U)
#define LPUART_STAT_MA1F_SHIFT                   (15U)
#define LPUART_STAT_MA1F_WIDTH                   (1U)
#define LPUART_STAT_MA1F(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_MA1F_SHIFT)) & LPUART_STAT_MA1F_MASK)
#define LPUART_STAT_PF_MASK                      (0x10000U)
#define LPUART_STAT_PF_SHIFT                     (16U)
#define LPUART_STAT_PF_WIDTH                     (1U)
#define LPUART_STAT_PF(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_PF_SHIFT)) & LPUART_STAT_PF_MASK)
#define LPUART_STAT_FE_MASK                      (0x20000U)
#define LPUART_STAT_FE_SHIFT                     (17U)
#define LPUART_STAT_FE_WIDTH                     (1U)
#define LPUART_STAT_FE(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_FE_SHIFT)) & LPUART_STAT_FE_MASK)
#define LPUART_STAT_NF_MASK                      (0x40000U)
#define LPUART_STAT_NF_SHIFT                     (18U)
#define LPUART_STAT_NF_WIDTH                     (1U)
#define LPUART_STAT_NF(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_NF_SHIFT)) & LPUART_STAT_NF_MASK)
#define LPUART_STAT_OR_MASK                      (0x80000U)
#define LPUART_STAT_OR_SHIFT                     (19U)
#define LPUART_STAT_OR_WIDTH                     (1U)
#define LPUART_STAT_OR(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_OR_SHIFT)) & LPUART_STAT_OR_MASK)
#define LPUART_STAT_IDLE_MASK                    (0x100000U)
#define LPUART_STAT_IDLE_SHIFT                   (20U)
#define LPUART_STAT_IDLE_WIDTH                   (1U)
#define LPUART_STAT_IDLE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_IDLE_SHIFT)) & LPUART_STAT_IDLE_MASK)
#define LPUART_STAT_RDRF_MASK                    (0x200000U)
#define LPUART_STAT_RDRF_SHIFT                   (21U)
#define LPUART_STAT_RDRF_WIDTH                   (1U)
#define LPUART_STAT_RDRF(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_RDRF_SHIFT)) & LPUART_STAT_RDRF_MASK)
#define LPUART_STAT_TC_MASK                      (0x400000U)
#define LPUART_STAT_TC_SHIFT                     (22U)
#define LPUART_STAT_TC_WIDTH                     (1U)
#define LPUART_STAT_TC(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_TC_SHIFT)) & LPUART_STAT_TC_MASK)
#define LPUART_STAT_TDRE_MASK                    (0x800000U)
#define LPUART_STAT_TDRE_SHIFT                   (23U)
#define LPUART_STAT_TDRE_WIDTH                   (1U)
#define LPUART_STAT_TDRE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_TDRE_SHIFT)) & LPUART_STAT_TDRE_MASK)
#define LPUART_STAT_RAF_MASK                     (0x1000000U)
#define LPUART_STAT_RAF_SHIFT                    (24U)
#define LPUART_STAT_RAF_WIDTH                    (1U)
#define LPUART_STAT_RAF(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_RAF_SHIFT)) & LPUART_STAT_RAF_MASK)
#define LPUART_STAT_LBKDE_MASK                   (0x2000000U)
#define LPUART_STAT_LBKDE_SHIFT                  (25U)
#define LPUART_STAT_LBKDE_WIDTH                  (1U)
#define LPUART_STAT_LBKDE(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_LBKDE_SHIFT)) & LPUART_STAT_LBKDE_MASK)
#define LPUART_STAT_BRK13_MASK                   (0x4000000U)
#define LPUART_STAT_BRK13_SHIFT                  (26U)
#define LPUART_STAT_BRK13_WIDTH                  (1U)
#define LPUART_STAT_BRK13(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_BRK13_SHIFT)) & LPUART_STAT_BRK13_MASK)
#define LPUART_STAT_RWUID_MASK                   (0x8000000U)
#define LPUART_STAT_RWUID_SHIFT                  (27U)
#define LPUART_STAT_RWUID_WIDTH                  (1U)
#define LPUART_STAT_RWUID(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_RWUID_SHIFT)) & LPUART_STAT_RWUID_MASK)
#define LPUART_STAT_RXINV_MASK                   (0x10000000U)
#define LPUART_STAT_RXINV_SHIFT                  (28U)
#define LPUART_STAT_RXINV_WIDTH                  (1U)
#define LPUART_STAT_RXINV(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_RXINV_SHIFT)) & LPUART_STAT_RXINV_MASK)
#define LPUART_STAT_MSBF_MASK                    (0x20000000U)
#define LPUART_STAT_MSBF_SHIFT                   (29U)
#define LPUART_STAT_MSBF_WIDTH                   (1U)
#define LPUART_STAT_MSBF(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_MSBF_SHIFT)) & LPUART_STAT_MSBF_MASK)
#define LPUART_STAT_RXEDGIF_MASK                 (0x40000000U)
#define LPUART_STAT_RXEDGIF_SHIFT                (30U)
#define LPUART_STAT_RXEDGIF_WIDTH                (1U)
#define LPUART_STAT_RXEDGIF(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_RXEDGIF_SHIFT)) & LPUART_STAT_RXEDGIF_MASK)
#define LPUART_STAT_LBKDIF_MASK                  (0x80000000U)
#define LPUART_STAT_LBKDIF_SHIFT                 (31U)
#define LPUART_STAT_LBKDIF_WIDTH                 (1U)
#define LPUART_STAT_LBKDIF(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_LBKDIF_SHIFT)) & LPUART_STAT_LBKDIF_MASK)
/*! @} */

/*! @name CTRL - LPUART Control Register */
/*! @{ */
#define LPUART_CTRL_PT_MASK                      (0x1U)
#define LPUART_CTRL_PT_SHIFT                     (0U)
#define LPUART_CTRL_PT_WIDTH                     (1U)
#define LPUART_CTRL_PT(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_PT_SHIFT)) & LPUART_CTRL_PT_MASK)
#define LPUART_CTRL_PE_MASK                      (0x2U)
#define LPUART_CTRL_PE_SHIFT                     (1U)
#define LPUART_CTRL_PE_WIDTH                     (1U)
#define LPUART_CTRL_PE(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_PE_SHIFT)) & LPUART_CTRL_PE_MASK)
#define LPUART_CTRL_ILT_MASK                     (0x4U)
#define LPUART_CTRL_ILT_SHIFT                    (2U)
#define LPUART_CTRL_ILT_WIDTH                    (1U)
#define LPUART_CTRL_ILT(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_ILT_SHIFT)) & LPUART_CTRL_ILT_MASK)
#define LPUART_CTRL_WAKE_MASK                    (0x8U)
#define LPUART_CTRL_WAKE_SHIFT                   (3U)
#define LPUART_CTRL_WAKE_WIDTH                   (1U)
#define LPUART_CTRL_WAKE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_WAKE_SHIFT)) & LPUART_CTRL_WAKE_MASK)
#define LPUART_CTRL_M_MASK                       (0x10U)
#define LPUART_CTRL_M_SHIFT                      (4U)
#define LPUART_CTRL_M_WIDTH                      (1U)
#define LPUART_CTRL_M(x)                         (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_M_SHIFT)) & LPUART_CTRL_M_MASK)
#define LPUART_CTRL_RSRC_MASK                    (0x20U)
#define LPUART_CTRL_RSRC_SHIFT                   (5U)
#define LPUART_CTRL_RSRC_WIDTH                   (1U)
#define LPUART_CTRL_RSRC(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_RSRC_SHIFT)) & LPUART_CTRL_RSRC_MASK)
#define LPUART_CTRL_DOZEEN_MASK                  (0x40U)
#define LPUART_CTRL_DOZEEN_SHIFT                 (6U)
#define LPUART_CTRL_DOZEEN_WIDTH                 (1U)
#define LPUART_CTRL_DOZEEN(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_DOZEEN_SHIFT)) & LPUART_CTRL_DOZEEN_MASK)
#define LPUART_CTRL_LOOPS_MASK                   (0x80U)
#define LPUART_CTRL_LOOPS_SHIFT                  (7U)
#define LPUART_CTRL_LOOPS_WIDTH                  (1U)
#define LPUART_CTRL_LOOPS(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_LOOPS_SHIFT)) & LPUART_CTRL_LOOPS_MASK)
#define LPUART_CTRL_IDLECFG_MASK                 (0x700U)
#define LPUART_CTRL_IDLECFG_SHIFT                (8U)
#define LPUART_CTRL_IDLECFG_WIDTH                (3U)
#define LPUART_CTRL_IDLECFG(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_IDLECFG_SHIFT)) & LPUART_CTRL_IDLECFG_MASK)
#define LPUART_CTRL_M7_MASK                      (0x800U)
#define LPUART_CTRL_M7_SHIFT                     (11U)
#define LPUART_CTRL_M7_WIDTH                     (1U)
#define LPUART_CTRL_M7(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_M7_SHIFT)) & LPUART_CTRL_M7_MASK)
#define LPUART_CTRL_MA2IE_MASK                   (0x4000U)
#define LPUART_CTRL_MA2IE_SHIFT                  (14U)
#define LPUART_CTRL_MA2IE_WIDTH                  (1U)
#define LPUART_CTRL_MA2IE(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_MA2IE_SHIFT)) & LPUART_CTRL_MA2IE_MASK)
#define LPUART_CTRL_MA1IE_MASK                   (0x8000U)
#define LPUART_CTRL_MA1IE_SHIFT                  (15U)
#define LPUART_CTRL_MA1IE_WIDTH                  (1U)
#define LPUART_CTRL_MA1IE(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_MA1IE_SHIFT)) & LPUART_CTRL_MA1IE_MASK)
#define LPUART_CTRL_SBK_MASK                     (0x10000U)
#define LPUART_CTRL_SBK_SHIFT                    (16U)
#define LPUART_CTRL_SBK_WIDTH                    (1U)
#define LPUART_CTRL_SBK(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_SBK_SHIFT)) & LPUART_CTRL_SBK_MASK)
#define LPUART_CTRL_RWU_MASK                     (0x20000U)
#define LPUART_CTRL_RWU_SHIFT                    (17U)
#define LPUART_CTRL_RWU_WIDTH                    (1U)
#define LPUART_CTRL_RWU(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_RWU_SHIFT)) & LPUART_CTRL_RWU_MASK)
#define LPUART_CTRL_RE_MASK                      (0x40000U)
#define LPUART_CTRL_RE_SHIFT                     (18U)
#define LPUART_CTRL_RE_WIDTH                     (1U)
#define LPUART_CTRL_RE(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_RE_SHIFT)) & LPUART_CTRL_RE_MASK)
#define LPUART_CTRL_TE_MASK                      (0x80000U)
#define LPUART_CTRL_TE_SHIFT                     (19U)
#define LPUART_CTRL_TE_WIDTH                     (1U)
#define LPUART_CTRL_TE(x)                        (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_TE_SHIFT)) & LPUART_CTRL_TE_MASK)
#define LPUART_CTRL_ILIE_MASK                    (0x100000U)
#define LPUART_CTRL_ILIE_SHIFT                   (20U)
#define LPUART_CTRL_ILIE_WIDTH                   (1U)
#define LPUART_CTRL_ILIE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_ILIE_SHIFT)) & LPUART_CTRL_ILIE_MASK)
#define LPUART_CTRL_RIE_MASK                     (0x200000U)
#define LPUART_CTRL_RIE_SHIFT                    (21U)
#define LPUART_CTRL_RIE_WIDTH                    (1U)
#define LPUART_CTRL_RIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_RIE_SHIFT)) & LPUART_CTRL_RIE_MASK)
#define LPUART_CTRL_TCIE_MASK                    (0x400000U)
#define LPUART_CTRL_TCIE_SHIFT                   (22U)
#define LPUART_CTRL_TCIE_WIDTH                   (1U)
#define LPUART_CTRL_TCIE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_TCIE_SHIFT)) & LPUART_CTRL_TCIE_MASK)
#define LPUART_CTRL_TIE_MASK                     (0x800000U)
#define LPUART_CTRL_TIE_SHIFT                    (23U)
#define LPUART_CTRL_TIE_WIDTH                    (1U)
#define LPUART_CTRL_TIE(x)                       (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_TIE_SHIFT)) & LPUART_CTRL_TIE_MASK)
#define LPUART_CTRL_PEIE_MASK                    (0x1000000U)
#define LPUART_CTRL_PEIE_SHIFT                   (24U)
#define LPUART_CTRL_PEIE_WIDTH                   (1U)
#define LPUART_CTRL_PEIE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_PEIE_SHIFT)) & LPUART_CTRL_PEIE_MASK)
#define LPUART_CTRL_FEIE_MASK                    (0x2000000U)
#define LPUART_CTRL_FEIE_SHIFT                   (25U)
#define LPUART_CTRL_FEIE_WIDTH                   (1U)
#define LPUART_CTRL_FEIE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_FEIE_SHIFT)) & LPUART_CTRL_FEIE_MASK)
#define LPUART_CTRL_NEIE_MASK                    (0x4000000U)
#define LPUART_CTRL_NEIE_SHIFT                   (26U)
#define LPUART_CTRL_NEIE_WIDTH                   (1U)
#define LPUART_CTRL_NEIE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_NEIE_SHIFT)) & LPUART_CTRL_NEIE_MASK)
#define LPUART_CTRL_ORIE_MASK                    (0x8000000U)
#define LPUART_CTRL_ORIE_SHIFT                   (27U)
#define LPUART_CTRL_ORIE_WIDTH                   (1U)
#define LPUART_CTRL_ORIE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_ORIE_SHIFT)) & LPUART_CTRL_ORIE_MASK)
#define LPUART_CTRL_TXINV_MASK                   (0x10000000U)
#define LPUART_CTRL_TXINV_SHIFT                  (28U)
#define LPUART_CTRL_TXINV_WIDTH                  (1U)
#define LPUART_CTRL_TXINV(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_TXINV_SHIFT)) & LPUART_CTRL_TXINV_MASK)
#define LPUART_CTRL_TXDIR_MASK                   (0x20000000U)
#define LPUART_CTRL_TXDIR_SHIFT                  (29U)
#define LPUART_CTRL_TXDIR_WIDTH                  (1U)
#define LPUART_CTRL_TXDIR(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_TXDIR_SHIFT)) & LPUART_CTRL_TXDIR_MASK)
#define LPUART_CTRL_R9T8_MASK                    (0x40000000U)
#define LPUART_CTRL_R9T8_SHIFT                   (30U)
#define LPUART_CTRL_R9T8_WIDTH                   (1U)
#define LPUART_CTRL_R9T8(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_R9T8_SHIFT)) & LPUART_CTRL_R9T8_MASK)
#define LPUART_CTRL_R8T9_MASK                    (0x80000000U)
#define LPUART_CTRL_R8T9_SHIFT                   (31U)
#define LPUART_CTRL_R8T9_WIDTH                   (1U)
#define LPUART_CTRL_R8T9(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_R8T9_SHIFT)) & LPUART_CTRL_R8T9_MASK)
/*! @} */

/*! @name DATA - LPUART Data Register */
/*! @{ */
#define LPUART_DATA_R0T0_MASK                    (0x1U)
#define LPUART_DATA_R0T0_SHIFT                   (0U)
#define LPUART_DATA_R0T0_WIDTH                   (1U)
#define LPUART_DATA_R0T0(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R0T0_SHIFT)) & LPUART_DATA_R0T0_MASK)
#define LPUART_DATA_R1T1_MASK                    (0x2U)
#define LPUART_DATA_R1T1_SHIFT                   (1U)
#define LPUART_DATA_R1T1_WIDTH                   (1U)
#define LPUART_DATA_R1T1(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R1T1_SHIFT)) & LPUART_DATA_R1T1_MASK)
#define LPUART_DATA_R2T2_MASK                    (0x4U)
#define LPUART_DATA_R2T2_SHIFT                   (2U)
#define LPUART_DATA_R2T2_WIDTH                   (1U)
#define LPUART_DATA_R2T2(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R2T2_SHIFT)) & LPUART_DATA_R2T2_MASK)
#define LPUART_DATA_R3T3_MASK                    (0x8U)
#define LPUART_DATA_R3T3_SHIFT                   (3U)
#define LPUART_DATA_R3T3_WIDTH                   (1U)
#define LPUART_DATA_R3T3(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R3T3_SHIFT)) & LPUART_DATA_R3T3_MASK)
#define LPUART_DATA_R4T4_MASK                    (0x10U)
#define LPUART_DATA_R4T4_SHIFT                   (4U)
#define LPUART_DATA_R4T4_WIDTH                   (1U)
#define LPUART_DATA_R4T4(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R4T4_SHIFT)) & LPUART_DATA_R4T4_MASK)
#define LPUART_DATA_R5T5_MASK                    (0x20U)
#define LPUART_DATA_R5T5_SHIFT                   (5U)
#define LPUART_DATA_R5T5_WIDTH                   (1U)
#define LPUART_DATA_R5T5(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R5T5_SHIFT)) & LPUART_DATA_R5T5_MASK)
#define LPUART_DATA_R6T6_MASK                    (0x40U)
#define LPUART_DATA_R6T6_SHIFT                   (6U)
#define LPUART_DATA_R6T6_WIDTH                   (1U)
#define LPUART_DATA_R6T6(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R6T6_SHIFT)) & LPUART_DATA_R6T6_MASK)
#define LPUART_DATA_R7T7_MASK                    (0x80U)
#define LPUART_DATA_R7T7_SHIFT                   (7U)
#define LPUART_DATA_R7T7_WIDTH                   (1U)
#define LPUART_DATA_R7T7(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R7T7_SHIFT)) & LPUART_DATA_R7T7_MASK)
#define LPUART_DATA_R8T8_MASK                    (0x100U)
#define LPUART_DATA_R8T8_SHIFT                   (8U)
#define LPUART_DATA_R8T8_WIDTH                   (1U)
#define LPUART_DATA_R8T8(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R8T8_SHIFT)) & LPUART_DATA_R8T8_MASK)
#define LPUART_DATA_R9T9_MASK                    (0x200U)
#define LPUART_DATA_R9T9_SHIFT                   (9U)
#define LPUART_DATA_R9T9_WIDTH                   (1U)
#define LPUART_DATA_R9T9(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R9T9_SHIFT)) & LPUART_DATA_R9T9_MASK)
#define LPUART_DATA_IDLINE_MASK                  (0x800U)
#define LPUART_DATA_IDLINE_SHIFT                 (11U)
#define LPUART_DATA_IDLINE_WIDTH                 (1U)
#define LPUART_DATA_IDLINE(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_IDLINE_SHIFT)) & LPUART_DATA_IDLINE_MASK)
#define LPUART_DATA_RXEMPT_MASK                  (0x1000U)
#define LPUART_DATA_RXEMPT_SHIFT                 (12U)
#define LPUART_DATA_RXEMPT_WIDTH                 (1U)
#define LPUART_DATA_RXEMPT(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_RXEMPT_SHIFT)) & LPUART_DATA_RXEMPT_MASK)
#define LPUART_DATA_FRETSC_MASK                  (0x2000U)
#define LPUART_DATA_FRETSC_SHIFT                 (13U)
#define LPUART_DATA_FRETSC_WIDTH                 (1U)
#define LPUART_DATA_FRETSC(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_FRETSC_SHIFT)) & LPUART_DATA_FRETSC_MASK)
#define LPUART_DATA_PARITYE_MASK                 (0x4000U)
#define LPUART_DATA_PARITYE_SHIFT                (14U)
#define LPUART_DATA_PARITYE_WIDTH                (1U)
#define LPUART_DATA_PARITYE(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_PARITYE_SHIFT)) & LPUART_DATA_PARITYE_MASK)
#define LPUART_DATA_NOISY_MASK                   (0x8000U)
#define LPUART_DATA_NOISY_SHIFT                  (15U)
#define LPUART_DATA_NOISY_WIDTH                  (1U)
#define LPUART_DATA_NOISY(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_NOISY_SHIFT)) & LPUART_DATA_NOISY_MASK)
/*! @} */

/*! @name MATCH - LPUART Match Address Register */
/*! @{ */
#define LPUART_MATCH_MA1_MASK                    (0x3FFU)
#define LPUART_MATCH_MA1_SHIFT                   (0U)
#define LPUART_MATCH_MA1_WIDTH                   (10U)
#define LPUART_MATCH_MA1(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_MATCH_MA1_SHIFT)) & LPUART_MATCH_MA1_MASK)
#define LPUART_MATCH_MA2_MASK                    (0x3FF0000U)
#define LPUART_MATCH_MA2_SHIFT                   (16U)
#define LPUART_MATCH_MA2_WIDTH                   (10U)
#define LPUART_MATCH_MA2(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_MATCH_MA2_SHIFT)) & LPUART_MATCH_MA2_MASK)
/*! @} */

/*! @name MODIR - LPUART Modem IrDA Register */
/*! @{ */
#define LPUART_MODIR_TXCTSE_MASK                 (0x1U)
#define LPUART_MODIR_TXCTSE_SHIFT                (0U)
#define LPUART_MODIR_TXCTSE_WIDTH                (1U)
#define LPUART_MODIR_TXCTSE(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_TXCTSE_SHIFT)) & LPUART_MODIR_TXCTSE_MASK)
#define LPUART_MODIR_TXRTSE_MASK                 (0x2U)
#define LPUART_MODIR_TXRTSE_SHIFT                (1U)
#define LPUART_MODIR_TXRTSE_WIDTH                (1U)
#define LPUART_MODIR_TXRTSE(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_TXRTSE_SHIFT)) & LPUART_MODIR_TXRTSE_MASK)
#define LPUART_MODIR_TXRTSPOL_MASK               (0x4U)
#define LPUART_MODIR_TXRTSPOL_SHIFT              (2U)
#define LPUART_MODIR_TXRTSPOL_WIDTH              (1U)
#define LPUART_MODIR_TXRTSPOL(x)                 (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_TXRTSPOL_SHIFT)) & LPUART_MODIR_TXRTSPOL_MASK)
#define LPUART_MODIR_RXRTSE_MASK                 (0x8U)
#define LPUART_MODIR_RXRTSE_SHIFT                (3U)
#define LPUART_MODIR_RXRTSE_WIDTH                (1U)
#define LPUART_MODIR_RXRTSE(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_RXRTSE_SHIFT)) & LPUART_MODIR_RXRTSE_MASK)
#define LPUART_MODIR_TXCTSC_MASK                 (0x10U)
#define LPUART_MODIR_TXCTSC_SHIFT                (4U)
#define LPUART_MODIR_TXCTSC_WIDTH                (1U)
#define LPUART_MODIR_TXCTSC(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_TXCTSC_SHIFT)) & LPUART_MODIR_TXCTSC_MASK)
#define LPUART_MODIR_TXCTSSRC_MASK               (0x20U)
#define LPUART_MODIR_TXCTSSRC_SHIFT              (5U)
#define LPUART_MODIR_TXCTSSRC_WIDTH              (1U)
#define LPUART_MODIR_TXCTSSRC(x)                 (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_TXCTSSRC_SHIFT)) & LPUART_MODIR_TXCTSSRC_MASK)
#define LPUART_MODIR_RTSWATER_MASK               (0x300U)
#define LPUART_MODIR_RTSWATER_SHIFT              (8U)
#define LPUART_MODIR_RTSWATER_WIDTH              (2U)
#define LPUART_MODIR_RTSWATER(x)                 (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_RTSWATER_SHIFT)) & LPUART_MODIR_RTSWATER_MASK)
#define LPUART_MODIR_TNP_MASK                    (0x30000U)
#define LPUART_MODIR_TNP_SHIFT                   (16U)
#define LPUART_MODIR_TNP_WIDTH                   (2U)
#define LPUART_MODIR_TNP(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_TNP_SHIFT)) & LPUART_MODIR_TNP_MASK)
#define LPUART_MODIR_IREN_MASK                   (0x40000U)
#define LPUART_MODIR_IREN_SHIFT                  (18U)
#define LPUART_MODIR_IREN_WIDTH                  (1U)
#define LPUART_MODIR_IREN(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_IREN_SHIFT)) & LPUART_MODIR_IREN_MASK)
/*! @} */

/*! @name FIFO - LPUART FIFO Register */
/*! @{ */
#define LPUART_FIFO_RXFIFOSIZE_MASK              (0x7U)
#define LPUART_FIFO_RXFIFOSIZE_SHIFT             (0U)
#define LPUART_FIFO_RXFIFOSIZE_WIDTH             (3U)
#define LPUART_FIFO_RXFIFOSIZE(x)                (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXFIFOSIZE_SHIFT)) & LPUART_FIFO_RXFIFOSIZE_MASK)
#define LPUART_FIFO_RXFE_MASK                    (0x8U)
#define LPUART_FIFO_RXFE_SHIFT                   (3U)
#define LPUART_FIFO_RXFE_WIDTH                   (1U)
#define LPUART_FIFO_RXFE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXFE_SHIFT)) & LPUART_FIFO_RXFE_MASK)
#define LPUART_FIFO_TXFIFOSIZE_MASK              (0x70U)
#define LPUART_FIFO_TXFIFOSIZE_SHIFT             (4U)
#define LPUART_FIFO_TXFIFOSIZE_WIDTH             (3U)
#define LPUART_FIFO_TXFIFOSIZE(x)                (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_TXFIFOSIZE_SHIFT)) & LPUART_FIFO_TXFIFOSIZE_MASK)
#define LPUART_FIFO_TXFE_MASK                    (0x80U)
#define LPUART_FIFO_TXFE_SHIFT                   (7U)
#define LPUART_FIFO_TXFE_WIDTH                   (1U)
#define LPUART_FIFO_TXFE(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_TXFE_SHIFT)) & LPUART_FIFO_TXFE_MASK)
#define LPUART_FIFO_RXUFE_MASK                   (0x100U)
#define LPUART_FIFO_RXUFE_SHIFT                  (8U)
#define LPUART_FIFO_RXUFE_WIDTH                  (1U)
#define LPUART_FIFO_RXUFE(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXUFE_SHIFT)) & LPUART_FIFO_RXUFE_MASK)
#define LPUART_FIFO_TXOFE_MASK                   (0x200U)
#define LPUART_FIFO_TXOFE_SHIFT                  (9U)
#define LPUART_FIFO_TXOFE_WIDTH                  (1U)
#define LPUART_FIFO_TXOFE(x)                     (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_TXOFE_SHIFT)) & LPUART_FIFO_TXOFE_MASK)
#define LPUART_FIFO_RXIDEN_MASK                  (0x1C00U)
#define LPUART_FIFO_RXIDEN_SHIFT                 (10U)
#define LPUART_FIFO_RXIDEN_WIDTH                 (3U)
#define LPUART_FIFO_RXIDEN(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXIDEN_SHIFT)) & LPUART_FIFO_RXIDEN_MASK)
#define LPUART_FIFO_RXFLUSH_MASK                 (0x4000U)
#define LPUART_FIFO_RXFLUSH_SHIFT                (14U)
#define LPUART_FIFO_RXFLUSH_WIDTH                (1U)
#define LPUART_FIFO_RXFLUSH(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXFLUSH_SHIFT)) & LPUART_FIFO_RXFLUSH_MASK)
#define LPUART_FIFO_TXFLUSH_MASK                 (0x8000U)
#define LPUART_FIFO_TXFLUSH_SHIFT                (15U)
#define LPUART_FIFO_TXFLUSH_WIDTH                (1U)
#define LPUART_FIFO_TXFLUSH(x)                   (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_TXFLUSH_SHIFT)) & LPUART_FIFO_TXFLUSH_MASK)
#define LPUART_FIFO_RXUF_MASK                    (0x10000U)
#define LPUART_FIFO_RXUF_SHIFT                   (16U)
#define LPUART_FIFO_RXUF_WIDTH                   (1U)
#define LPUART_FIFO_RXUF(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXUF_SHIFT)) & LPUART_FIFO_RXUF_MASK)
#define LPUART_FIFO_TXOF_MASK                    (0x20000U)
#define LPUART_FIFO_TXOF_SHIFT                   (17U)
#define LPUART_FIFO_TXOF_WIDTH                   (1U)
#define LPUART_FIFO_TXOF(x)                      (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_TXOF_SHIFT)) & LPUART_FIFO_TXOF_MASK)
#define LPUART_FIFO_RXEMPT_MASK                  (0x400000U)
#define LPUART_FIFO_RXEMPT_SHIFT                 (22U)
#define LPUART_FIFO_RXEMPT_WIDTH                 (1U)
#define LPUART_FIFO_RXEMPT(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXEMPT_SHIFT)) & LPUART_FIFO_RXEMPT_MASK)
#define LPUART_FIFO_TXEMPT_MASK                  (0x800000U)
#define LPUART_FIFO_TXEMPT_SHIFT                 (23U)
#define LPUART_FIFO_TXEMPT_WIDTH                 (1U)
#define LPUART_FIFO_TXEMPT(x)                    (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_TXEMPT_SHIFT)) & LPUART_FIFO_TXEMPT_MASK)
/*! @} */

/*! @name WATER - LPUART Watermark Register */
/*! @{ */
#define LPUART_WATER_TXWATER_MASK                (0x3U)
#define LPUART_WATER_TXWATER_SHIFT               (0U)
#define LPUART_WATER_TXWATER_WIDTH               (2U)
#define LPUART_WATER_TXWATER(x)                  (((uint32_t)(((uint32_t)(x)) << LPUART_WATER_TXWATER_SHIFT)) & LPUART_WATER_TXWATER_MASK)
#define LPUART_WATER_TXCOUNT_MASK                (0x700U)
#define LPUART_WATER_TXCOUNT_SHIFT               (8U)
#define LPUART_WATER_TXCOUNT_WIDTH               (3U)
#define LPUART_WATER_TXCOUNT(x)                  (((uint32_t)(((uint32_t)(x)) << LPUART_WATER_TXCOUNT_SHIFT)) & LPUART_WATER_TXCOUNT_MASK)
#define LPUART_WATER_RXWATER_MASK                (0x30000U)
#define LPUART_WATER_RXWATER_SHIFT               (16U)
#define LPUART_WATER_RXWATER_WIDTH               (2U)
#define LPUART_WATER_RXWATER(x)                  (((uint32_t)(((uint32_t)(x)) << LPUART_WATER_RXWATER_SHIFT)) & LPUART_WATER_RXWATER_MASK)
#define LPUART_WATER_RXCOUNT_MASK                (0x7000000U)
#define LPUART_WATER_RXCOUNT_SHIFT               (24U)
#define LPUART_WATER_RXCOUNT_WIDTH               (3U)
#define LPUART_WATER_RXCOUNT(x)                  (((uint32_t)(((uint32_t)(x)) << LPUART_WATER_RXCOUNT_SHIFT)) & LPUART_WATER_RXCOUNT_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group LPUART_Register_Masks */

/*!
 * @}
 */ /* end of group LPUART_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- MCM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCM_Peripheral_Access_Layer MCM Peripheral Access Layer
 * @{
 */

/** MCM - Size of Registers Arrays */
#define MCM_LMDR_COUNT                            2u

/** MCM - Register Layout Typedef */
typedef struct {
  uint8_t RESERVED_0[8];
  __I  uint16_t PLASC;                             /**< Crossbar Switch (AXBS) Slave Configuration, offset: 0x8 */
  __I  uint16_t PLAMC;                             /**< Crossbar Switch (AXBS) Master Configuration, offset: 0xA */
  __IO uint32_t CPCR;                              /**< Core Platform Control Register, offset: 0xC */
  __IO uint32_t ISCR;                              /**< Interrupt Status and Control Register, offset: 0x10 */
  uint8_t RESERVED_1[28];
  __IO uint32_t PID;                               /**< Process ID Register, offset: 0x30 */
  uint8_t RESERVED_2[12];
  __IO uint32_t CPO;                               /**< Compute Operation Control Register, offset: 0x40 */
  uint8_t RESERVED_3[956];
  __IO uint32_t LMDR[MCM_LMDR_COUNT];              /**< Local Memory Descriptor Register, array offset: 0x400, array step: 0x4 */
  __IO uint32_t LMDR2;                             /**< Local Memory Descriptor Register2, offset: 0x408 */
  uint8_t RESERVED_4[116];
  __IO uint32_t LMPECR;                            /**< LMEM Parity and ECC Control Register, offset: 0x480 */
  uint8_t RESERVED_5[4];
  __IO uint32_t LMPEIR;                            /**< LMEM Parity and ECC Interrupt Register, offset: 0x488 */
  uint8_t RESERVED_6[4];
  __I  uint32_t LMFAR;                             /**< LMEM Fault Address Register, offset: 0x490 */
  __I  uint32_t LMFATR;                            /**< LMEM Fault Attribute Register, offset: 0x494 */
  uint8_t RESERVED_7[8];
  __I  uint32_t LMFDHR;                            /**< LMEM Fault Data High Register, offset: 0x4A0 */
  __I  uint32_t LMFDLR;                            /**< LMEM Fault Data Low Register, offset: 0x4A4 */
} MCM_Type, *MCM_MemMapPtr;

/** Number of instances of the MCM module. */
#define MCM_INSTANCE_COUNT                       (1u)

/* MCM - Peripheral instance base addresses */
/** Peripheral MCM base address */
#define MCM_BASE                                 (0xE0080000u)
/** Peripheral MCM base pointer */
#define MCM                                      ((MCM_Type *)MCM_BASE)
/** Array initializer of MCM peripheral base addresses */
#define MCM_BASE_ADDRS                           { MCM_BASE }
/** Array initializer of MCM peripheral base pointers */
#define MCM_BASE_PTRS                            { MCM }

/* ----------------------------------------------------------------------------
   -- MCM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCM_Register_Masks MCM Register Masks
 * @{
 */

/*! @name PLASC - Crossbar Switch (AXBS) Slave Configuration */
/*! @{ */
#define MCM_PLASC_ASC_MASK                       (0xFFU)
#define MCM_PLASC_ASC_SHIFT                      (0U)
#define MCM_PLASC_ASC_WIDTH                      (8U)
#define MCM_PLASC_ASC(x)                         (((uint16_t)(((uint16_t)(x)) << MCM_PLASC_ASC_SHIFT)) & MCM_PLASC_ASC_MASK)
/*! @} */

/*! @name PLAMC - Crossbar Switch (AXBS) Master Configuration */
/*! @{ */
#define MCM_PLAMC_AMC_MASK                       (0xFFU)
#define MCM_PLAMC_AMC_SHIFT                      (0U)
#define MCM_PLAMC_AMC_WIDTH                      (8U)
#define MCM_PLAMC_AMC(x)                         (((uint16_t)(((uint16_t)(x)) << MCM_PLAMC_AMC_SHIFT)) & MCM_PLAMC_AMC_MASK)
/*! @} */

/*! @name CPCR - Core Platform Control Register */
/*! @{ */
#define MCM_CPCR_HLT_FSM_ST_MASK                 (0x3U)
#define MCM_CPCR_HLT_FSM_ST_SHIFT                (0U)
#define MCM_CPCR_HLT_FSM_ST_WIDTH                (2U)
#define MCM_CPCR_HLT_FSM_ST(x)                   (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_HLT_FSM_ST_SHIFT)) & MCM_CPCR_HLT_FSM_ST_MASK)
#define MCM_CPCR_AXBS_HLT_REQ_MASK               (0x4U)
#define MCM_CPCR_AXBS_HLT_REQ_SHIFT              (2U)
#define MCM_CPCR_AXBS_HLT_REQ_WIDTH              (1U)
#define MCM_CPCR_AXBS_HLT_REQ(x)                 (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_AXBS_HLT_REQ_SHIFT)) & MCM_CPCR_AXBS_HLT_REQ_MASK)
#define MCM_CPCR_AXBS_HLTD_MASK                  (0x8U)
#define MCM_CPCR_AXBS_HLTD_SHIFT                 (3U)
#define MCM_CPCR_AXBS_HLTD_WIDTH                 (1U)
#define MCM_CPCR_AXBS_HLTD(x)                    (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_AXBS_HLTD_SHIFT)) & MCM_CPCR_AXBS_HLTD_MASK)
#define MCM_CPCR_FMC_PF_IDLE_MASK                (0x10U)
#define MCM_CPCR_FMC_PF_IDLE_SHIFT               (4U)
#define MCM_CPCR_FMC_PF_IDLE_WIDTH               (1U)
#define MCM_CPCR_FMC_PF_IDLE(x)                  (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_FMC_PF_IDLE_SHIFT)) & MCM_CPCR_FMC_PF_IDLE_MASK)
#define MCM_CPCR_PBRIDGE_IDLE_MASK               (0x40U)
#define MCM_CPCR_PBRIDGE_IDLE_SHIFT              (6U)
#define MCM_CPCR_PBRIDGE_IDLE_WIDTH              (1U)
#define MCM_CPCR_PBRIDGE_IDLE(x)                 (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_PBRIDGE_IDLE_SHIFT)) & MCM_CPCR_PBRIDGE_IDLE_MASK)
#define MCM_CPCR_CBRR_MASK                       (0x200U)
#define MCM_CPCR_CBRR_SHIFT                      (9U)
#define MCM_CPCR_CBRR_WIDTH                      (1U)
#define MCM_CPCR_CBRR(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_CBRR_SHIFT)) & MCM_CPCR_CBRR_MASK)
#define MCM_CPCR_SRAMUAP_MASK                    (0x3000000U)
#define MCM_CPCR_SRAMUAP_SHIFT                   (24U)
#define MCM_CPCR_SRAMUAP_WIDTH                   (2U)
#define MCM_CPCR_SRAMUAP(x)                      (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_SRAMUAP_SHIFT)) & MCM_CPCR_SRAMUAP_MASK)
#define MCM_CPCR_SRAMUWP_MASK                    (0x4000000U)
#define MCM_CPCR_SRAMUWP_SHIFT                   (26U)
#define MCM_CPCR_SRAMUWP_WIDTH                   (1U)
#define MCM_CPCR_SRAMUWP(x)                      (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_SRAMUWP_SHIFT)) & MCM_CPCR_SRAMUWP_MASK)
#define MCM_CPCR_SRAMLAP_MASK                    (0x30000000U)
#define MCM_CPCR_SRAMLAP_SHIFT                   (28U)
#define MCM_CPCR_SRAMLAP_WIDTH                   (2U)
#define MCM_CPCR_SRAMLAP(x)                      (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_SRAMLAP_SHIFT)) & MCM_CPCR_SRAMLAP_MASK)
#define MCM_CPCR_SRAMLWP_MASK                    (0x40000000U)
#define MCM_CPCR_SRAMLWP_SHIFT                   (30U)
#define MCM_CPCR_SRAMLWP_WIDTH                   (1U)
#define MCM_CPCR_SRAMLWP(x)                      (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_SRAMLWP_SHIFT)) & MCM_CPCR_SRAMLWP_MASK)
/*! @} */

/*! @name ISCR - Interrupt Status and Control Register */
/*! @{ */
#define MCM_ISCR_FIOC_MASK                       (0x100U)
#define MCM_ISCR_FIOC_SHIFT                      (8U)
#define MCM_ISCR_FIOC_WIDTH                      (1U)
#define MCM_ISCR_FIOC(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_ISCR_FIOC_SHIFT)) & MCM_ISCR_FIOC_MASK)
#define MCM_ISCR_FDZC_MASK                       (0x200U)
#define MCM_ISCR_FDZC_SHIFT                      (9U)
#define MCM_ISCR_FDZC_WIDTH                      (1U)
#define MCM_ISCR_FDZC(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_ISCR_FDZC_SHIFT)) & MCM_ISCR_FDZC_MASK)
#define MCM_ISCR_FOFC_MASK                       (0x400U)
#define MCM_ISCR_FOFC_SHIFT                      (10U)
#define MCM_ISCR_FOFC_WIDTH                      (1U)
#define MCM_ISCR_FOFC(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_ISCR_FOFC_SHIFT)) & MCM_ISCR_FOFC_MASK)
#define MCM_ISCR_FUFC_MASK                       (0x800U)
#define MCM_ISCR_FUFC_SHIFT                      (11U)
#define MCM_ISCR_FUFC_WIDTH                      (1U)
#define MCM_ISCR_FUFC(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_ISCR_FUFC_SHIFT)) & MCM_ISCR_FUFC_MASK)
#define MCM_ISCR_FIXC_MASK                       (0x1000U)
#define MCM_ISCR_FIXC_SHIFT                      (12U)
#define MCM_ISCR_FIXC_WIDTH                      (1U)
#define MCM_ISCR_FIXC(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_ISCR_FIXC_SHIFT)) & MCM_ISCR_FIXC_MASK)
#define MCM_ISCR_FIDC_MASK                       (0x8000U)
#define MCM_ISCR_FIDC_SHIFT                      (15U)
#define MCM_ISCR_FIDC_WIDTH                      (1U)
#define MCM_ISCR_FIDC(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_ISCR_FIDC_SHIFT)) & MCM_ISCR_FIDC_MASK)
#define MCM_ISCR_FIOCE_MASK                      (0x1000000U)
#define MCM_ISCR_FIOCE_SHIFT                     (24U)
#define MCM_ISCR_FIOCE_WIDTH                     (1U)
#define MCM_ISCR_FIOCE(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_ISCR_FIOCE_SHIFT)) & MCM_ISCR_FIOCE_MASK)
#define MCM_ISCR_FDZCE_MASK                      (0x2000000U)
#define MCM_ISCR_FDZCE_SHIFT                     (25U)
#define MCM_ISCR_FDZCE_WIDTH                     (1U)
#define MCM_ISCR_FDZCE(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_ISCR_FDZCE_SHIFT)) & MCM_ISCR_FDZCE_MASK)
#define MCM_ISCR_FOFCE_MASK                      (0x4000000U)
#define MCM_ISCR_FOFCE_SHIFT                     (26U)
#define MCM_ISCR_FOFCE_WIDTH                     (1U)
#define MCM_ISCR_FOFCE(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_ISCR_FOFCE_SHIFT)) & MCM_ISCR_FOFCE_MASK)
#define MCM_ISCR_FUFCE_MASK                      (0x8000000U)
#define MCM_ISCR_FUFCE_SHIFT                     (27U)
#define MCM_ISCR_FUFCE_WIDTH                     (1U)
#define MCM_ISCR_FUFCE(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_ISCR_FUFCE_SHIFT)) & MCM_ISCR_FUFCE_MASK)
#define MCM_ISCR_FIXCE_MASK                      (0x10000000U)
#define MCM_ISCR_FIXCE_SHIFT                     (28U)
#define MCM_ISCR_FIXCE_WIDTH                     (1U)
#define MCM_ISCR_FIXCE(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_ISCR_FIXCE_SHIFT)) & MCM_ISCR_FIXCE_MASK)
#define MCM_ISCR_FIDCE_MASK                      (0x80000000U)
#define MCM_ISCR_FIDCE_SHIFT                     (31U)
#define MCM_ISCR_FIDCE_WIDTH                     (1U)
#define MCM_ISCR_FIDCE(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_ISCR_FIDCE_SHIFT)) & MCM_ISCR_FIDCE_MASK)
/*! @} */

/*! @name PID - Process ID Register */
/*! @{ */
#define MCM_PID_PID_MASK                         (0xFFU)
#define MCM_PID_PID_SHIFT                        (0U)
#define MCM_PID_PID_WIDTH                        (8U)
#define MCM_PID_PID(x)                           (((uint32_t)(((uint32_t)(x)) << MCM_PID_PID_SHIFT)) & MCM_PID_PID_MASK)
/*! @} */

/*! @name CPO - Compute Operation Control Register */
/*! @{ */
#define MCM_CPO_CPOREQ_MASK                      (0x1U)
#define MCM_CPO_CPOREQ_SHIFT                     (0U)
#define MCM_CPO_CPOREQ_WIDTH                     (1U)
#define MCM_CPO_CPOREQ(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_CPO_CPOREQ_SHIFT)) & MCM_CPO_CPOREQ_MASK)
#define MCM_CPO_CPOACK_MASK                      (0x2U)
#define MCM_CPO_CPOACK_SHIFT                     (1U)
#define MCM_CPO_CPOACK_WIDTH                     (1U)
#define MCM_CPO_CPOACK(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_CPO_CPOACK_SHIFT)) & MCM_CPO_CPOACK_MASK)
#define MCM_CPO_CPOWOI_MASK                      (0x4U)
#define MCM_CPO_CPOWOI_SHIFT                     (2U)
#define MCM_CPO_CPOWOI_WIDTH                     (1U)
#define MCM_CPO_CPOWOI(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_CPO_CPOWOI_SHIFT)) & MCM_CPO_CPOWOI_MASK)
/*! @} */

/*! @name LMDR - Local Memory Descriptor Register */
/*! @{ */
#define MCM_LMDR_CF0_MASK                        (0xFU)
#define MCM_LMDR_CF0_SHIFT                       (0U)
#define MCM_LMDR_CF0_WIDTH                       (4U)
#define MCM_LMDR_CF0(x)                          (((uint32_t)(((uint32_t)(x)) << MCM_LMDR_CF0_SHIFT)) & MCM_LMDR_CF0_MASK)
#define MCM_LMDR_MT_MASK                         (0xE000U)
#define MCM_LMDR_MT_SHIFT                        (13U)
#define MCM_LMDR_MT_WIDTH                        (3U)
#define MCM_LMDR_MT(x)                           (((uint32_t)(((uint32_t)(x)) << MCM_LMDR_MT_SHIFT)) & MCM_LMDR_MT_MASK)
#define MCM_LMDR_LOCK_MASK                       (0x10000U)
#define MCM_LMDR_LOCK_SHIFT                      (16U)
#define MCM_LMDR_LOCK_WIDTH                      (1U)
#define MCM_LMDR_LOCK(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_LMDR_LOCK_SHIFT)) & MCM_LMDR_LOCK_MASK)
#define MCM_LMDR_DPW_MASK                        (0xE0000U)
#define MCM_LMDR_DPW_SHIFT                       (17U)
#define MCM_LMDR_DPW_WIDTH                       (3U)
#define MCM_LMDR_DPW(x)                          (((uint32_t)(((uint32_t)(x)) << MCM_LMDR_DPW_SHIFT)) & MCM_LMDR_DPW_MASK)
#define MCM_LMDR_WY_MASK                         (0xF00000U)
#define MCM_LMDR_WY_SHIFT                        (20U)
#define MCM_LMDR_WY_WIDTH                        (4U)
#define MCM_LMDR_WY(x)                           (((uint32_t)(((uint32_t)(x)) << MCM_LMDR_WY_SHIFT)) & MCM_LMDR_WY_MASK)
#define MCM_LMDR_LMSZ_MASK                       (0xF000000U)
#define MCM_LMDR_LMSZ_SHIFT                      (24U)
#define MCM_LMDR_LMSZ_WIDTH                      (4U)
#define MCM_LMDR_LMSZ(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_LMDR_LMSZ_SHIFT)) & MCM_LMDR_LMSZ_MASK)
#define MCM_LMDR_LMSZH_MASK                      (0x10000000U)
#define MCM_LMDR_LMSZH_SHIFT                     (28U)
#define MCM_LMDR_LMSZH_WIDTH                     (1U)
#define MCM_LMDR_LMSZH(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_LMDR_LMSZH_SHIFT)) & MCM_LMDR_LMSZH_MASK)
#define MCM_LMDR_V_MASK                          (0x80000000U)
#define MCM_LMDR_V_SHIFT                         (31U)
#define MCM_LMDR_V_WIDTH                         (1U)
#define MCM_LMDR_V(x)                            (((uint32_t)(((uint32_t)(x)) << MCM_LMDR_V_SHIFT)) & MCM_LMDR_V_MASK)
/*! @} */

/*! @name LMDR2 - Local Memory Descriptor Register2 */
/*! @{ */
#define MCM_LMDR2_CF1_MASK                       (0xF0U)
#define MCM_LMDR2_CF1_SHIFT                      (4U)
#define MCM_LMDR2_CF1_WIDTH                      (4U)
#define MCM_LMDR2_CF1(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_LMDR2_CF1_SHIFT)) & MCM_LMDR2_CF1_MASK)
#define MCM_LMDR2_MT_MASK                        (0xE000U)
#define MCM_LMDR2_MT_SHIFT                       (13U)
#define MCM_LMDR2_MT_WIDTH                       (3U)
#define MCM_LMDR2_MT(x)                          (((uint32_t)(((uint32_t)(x)) << MCM_LMDR2_MT_SHIFT)) & MCM_LMDR2_MT_MASK)
#define MCM_LMDR2_LOCK_MASK                      (0x10000U)
#define MCM_LMDR2_LOCK_SHIFT                     (16U)
#define MCM_LMDR2_LOCK_WIDTH                     (1U)
#define MCM_LMDR2_LOCK(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_LMDR2_LOCK_SHIFT)) & MCM_LMDR2_LOCK_MASK)
#define MCM_LMDR2_DPW_MASK                       (0xE0000U)
#define MCM_LMDR2_DPW_SHIFT                      (17U)
#define MCM_LMDR2_DPW_WIDTH                      (3U)
#define MCM_LMDR2_DPW(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_LMDR2_DPW_SHIFT)) & MCM_LMDR2_DPW_MASK)
#define MCM_LMDR2_WY_MASK                        (0xF00000U)
#define MCM_LMDR2_WY_SHIFT                       (20U)
#define MCM_LMDR2_WY_WIDTH                       (4U)
#define MCM_LMDR2_WY(x)                          (((uint32_t)(((uint32_t)(x)) << MCM_LMDR2_WY_SHIFT)) & MCM_LMDR2_WY_MASK)
#define MCM_LMDR2_LMSZ_MASK                      (0xF000000U)
#define MCM_LMDR2_LMSZ_SHIFT                     (24U)
#define MCM_LMDR2_LMSZ_WIDTH                     (4U)
#define MCM_LMDR2_LMSZ(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_LMDR2_LMSZ_SHIFT)) & MCM_LMDR2_LMSZ_MASK)
#define MCM_LMDR2_LMSZH_MASK                     (0x10000000U)
#define MCM_LMDR2_LMSZH_SHIFT                    (28U)
#define MCM_LMDR2_LMSZH_WIDTH                    (1U)
#define MCM_LMDR2_LMSZH(x)                       (((uint32_t)(((uint32_t)(x)) << MCM_LMDR2_LMSZH_SHIFT)) & MCM_LMDR2_LMSZH_MASK)
#define MCM_LMDR2_V_MASK                         (0x80000000U)
#define MCM_LMDR2_V_SHIFT                        (31U)
#define MCM_LMDR2_V_WIDTH                        (1U)
#define MCM_LMDR2_V(x)                           (((uint32_t)(((uint32_t)(x)) << MCM_LMDR2_V_SHIFT)) & MCM_LMDR2_V_MASK)
/*! @} */

/*! @name LMPECR - LMEM Parity and ECC Control Register */
/*! @{ */
#define MCM_LMPECR_ERNCR_MASK                    (0x1U)
#define MCM_LMPECR_ERNCR_SHIFT                   (0U)
#define MCM_LMPECR_ERNCR_WIDTH                   (1U)
#define MCM_LMPECR_ERNCR(x)                      (((uint32_t)(((uint32_t)(x)) << MCM_LMPECR_ERNCR_SHIFT)) & MCM_LMPECR_ERNCR_MASK)
#define MCM_LMPECR_ER1BR_MASK                    (0x100U)
#define MCM_LMPECR_ER1BR_SHIFT                   (8U)
#define MCM_LMPECR_ER1BR_WIDTH                   (1U)
#define MCM_LMPECR_ER1BR(x)                      (((uint32_t)(((uint32_t)(x)) << MCM_LMPECR_ER1BR_SHIFT)) & MCM_LMPECR_ER1BR_MASK)
#define MCM_LMPECR_ECPR_MASK                     (0x100000U)
#define MCM_LMPECR_ECPR_SHIFT                    (20U)
#define MCM_LMPECR_ECPR_WIDTH                    (1U)
#define MCM_LMPECR_ECPR(x)                       (((uint32_t)(((uint32_t)(x)) << MCM_LMPECR_ECPR_SHIFT)) & MCM_LMPECR_ECPR_MASK)
/*! @} */

/*! @name LMPEIR - LMEM Parity and ECC Interrupt Register */
/*! @{ */
#define MCM_LMPEIR_ENC_MASK                      (0xFFU)
#define MCM_LMPEIR_ENC_SHIFT                     (0U)
#define MCM_LMPEIR_ENC_WIDTH                     (8U)
#define MCM_LMPEIR_ENC(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_LMPEIR_ENC_SHIFT)) & MCM_LMPEIR_ENC_MASK)
#define MCM_LMPEIR_E1B_MASK                      (0xFF00U)
#define MCM_LMPEIR_E1B_SHIFT                     (8U)
#define MCM_LMPEIR_E1B_WIDTH                     (8U)
#define MCM_LMPEIR_E1B(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_LMPEIR_E1B_SHIFT)) & MCM_LMPEIR_E1B_MASK)
#define MCM_LMPEIR_PE_MASK                       (0xFF0000U)
#define MCM_LMPEIR_PE_SHIFT                      (16U)
#define MCM_LMPEIR_PE_WIDTH                      (8U)
#define MCM_LMPEIR_PE(x)                         (((uint32_t)(((uint32_t)(x)) << MCM_LMPEIR_PE_SHIFT)) & MCM_LMPEIR_PE_MASK)
#define MCM_LMPEIR_PEELOC_MASK                   (0x1F000000U)
#define MCM_LMPEIR_PEELOC_SHIFT                  (24U)
#define MCM_LMPEIR_PEELOC_WIDTH                  (5U)
#define MCM_LMPEIR_PEELOC(x)                     (((uint32_t)(((uint32_t)(x)) << MCM_LMPEIR_PEELOC_SHIFT)) & MCM_LMPEIR_PEELOC_MASK)
#define MCM_LMPEIR_V_MASK                        (0x80000000U)
#define MCM_LMPEIR_V_SHIFT                       (31U)
#define MCM_LMPEIR_V_WIDTH                       (1U)
#define MCM_LMPEIR_V(x)                          (((uint32_t)(((uint32_t)(x)) << MCM_LMPEIR_V_SHIFT)) & MCM_LMPEIR_V_MASK)
/*! @} */

/*! @name LMFAR - LMEM Fault Address Register */
/*! @{ */
#define MCM_LMFAR_EFADD_MASK                     (0xFFFFFFFFU)
#define MCM_LMFAR_EFADD_SHIFT                    (0U)
#define MCM_LMFAR_EFADD_WIDTH                    (32U)
#define MCM_LMFAR_EFADD(x)                       (((uint32_t)(((uint32_t)(x)) << MCM_LMFAR_EFADD_SHIFT)) & MCM_LMFAR_EFADD_MASK)
/*! @} */

/*! @name LMFATR - LMEM Fault Attribute Register */
/*! @{ */
#define MCM_LMFATR_PEFPRT_MASK                   (0xFU)
#define MCM_LMFATR_PEFPRT_SHIFT                  (0U)
#define MCM_LMFATR_PEFPRT_WIDTH                  (4U)
#define MCM_LMFATR_PEFPRT(x)                     (((uint32_t)(((uint32_t)(x)) << MCM_LMFATR_PEFPRT_SHIFT)) & MCM_LMFATR_PEFPRT_MASK)
#define MCM_LMFATR_PEFSIZE_MASK                  (0x70U)
#define MCM_LMFATR_PEFSIZE_SHIFT                 (4U)
#define MCM_LMFATR_PEFSIZE_WIDTH                 (3U)
#define MCM_LMFATR_PEFSIZE(x)                    (((uint32_t)(((uint32_t)(x)) << MCM_LMFATR_PEFSIZE_SHIFT)) & MCM_LMFATR_PEFSIZE_MASK)
#define MCM_LMFATR_PEFW_MASK                     (0x80U)
#define MCM_LMFATR_PEFW_SHIFT                    (7U)
#define MCM_LMFATR_PEFW_WIDTH                    (1U)
#define MCM_LMFATR_PEFW(x)                       (((uint32_t)(((uint32_t)(x)) << MCM_LMFATR_PEFW_SHIFT)) & MCM_LMFATR_PEFW_MASK)
#define MCM_LMFATR_PEFMST_MASK                   (0xFF00U)
#define MCM_LMFATR_PEFMST_SHIFT                  (8U)
#define MCM_LMFATR_PEFMST_WIDTH                  (8U)
#define MCM_LMFATR_PEFMST(x)                     (((uint32_t)(((uint32_t)(x)) << MCM_LMFATR_PEFMST_SHIFT)) & MCM_LMFATR_PEFMST_MASK)
#define MCM_LMFATR_OVR_MASK                      (0x80000000U)
#define MCM_LMFATR_OVR_SHIFT                     (31U)
#define MCM_LMFATR_OVR_WIDTH                     (1U)
#define MCM_LMFATR_OVR(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_LMFATR_OVR_SHIFT)) & MCM_LMFATR_OVR_MASK)
/*! @} */

/*! @name LMFDHR - LMEM Fault Data High Register */
/*! @{ */
#define MCM_LMFDHR_PEFDH_MASK                    (0xFFFFFFFFU)
#define MCM_LMFDHR_PEFDH_SHIFT                   (0U)
#define MCM_LMFDHR_PEFDH_WIDTH                   (32U)
#define MCM_LMFDHR_PEFDH(x)                      (((uint32_t)(((uint32_t)(x)) << MCM_LMFDHR_PEFDH_SHIFT)) & MCM_LMFDHR_PEFDH_MASK)
/*! @} */

/*! @name LMFDLR - LMEM Fault Data Low Register */
/*! @{ */
#define MCM_LMFDLR_PEFDL_MASK                    (0xFFFFFFFFU)
#define MCM_LMFDLR_PEFDL_SHIFT                   (0U)
#define MCM_LMFDLR_PEFDL_WIDTH                   (32U)
#define MCM_LMFDLR_PEFDL(x)                      (((uint32_t)(((uint32_t)(x)) << MCM_LMFDLR_PEFDL_SHIFT)) & MCM_LMFDLR_PEFDL_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group MCM_Register_Masks */

/*!
 * @}
 */ /* end of group MCM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- MPU Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MPU_Peripheral_Access_Layer MPU Peripheral Access Layer
 * @{
 */

/** MPU - Size of Registers Arrays */
#define MPU_EAR_EDR_COUNT                         4u
#define MPU_RGD_COUNT                             8u
#define MPU_RGDAAC_COUNT                          8u

/** MPU - Register Layout Typedef */
typedef struct {
  __IO uint32_t CESR;                              /**< Control/Error Status Register, offset: 0x0 */
  uint8_t RESERVED_0[12];
  struct {                                         /* offset: 0x10, array step: 0x8 */
    __I  uint32_t EAR;                               /**< Error Address Register, slave port 0..Error Address Register, slave port 3, array offset: 0x10, array step: 0x8 */
    __I  uint32_t EDR;                               /**< Error Detail Register, slave port 0..Error Detail Register, slave port 3, array offset: 0x14, array step: 0x8 */
  } EAR_EDR[MPU_EAR_EDR_COUNT];
  uint8_t RESERVED_1[976];
  struct {                                         /* offset: 0x400, array step: 0x10 */
    __IO uint32_t WORD0;                         /**< Region Descriptor 0, Word 0..Region Descriptor 7, Word 0, array offset: 0x400, array step: 0x10 */
    __IO uint32_t WORD1;                         /**< Region Descriptor 0, Word 1..Region Descriptor 7, Word 1, array offset: 0x404, array step: 0x10 */
    __IO uint32_t WORD2;                         /**< Region Descriptor 0, Word 2..Region Descriptor 7, Word 2, array offset: 0x408, array step: 0x10 */
    __IO uint32_t WORD3;                         /**< Region Descriptor 0, Word 3..Region Descriptor 7, Word 3, array offset: 0x40C, array step: 0x10 */
  } RGD[MPU_RGD_COUNT];
  uint8_t RESERVED_2[896];
  __IO uint32_t RGDAAC[MPU_RGDAAC_COUNT];          /**< Region Descriptor Alternate Access Control 0..Region Descriptor Alternate Access Control 7, array offset: 0x800, array step: 0x4 */
} MPU_Type, *MPU_MemMapPtr;

/** Number of instances of the MPU module. */
#define MPU_INSTANCE_COUNT                       (1u)

/* MPU - Peripheral instance base addresses */
/** Peripheral MPU base address */
#define MPU_BASE                                 (0x4000D000u)
/** Peripheral MPU base pointer */
#define MPU                                      ((MPU_Type *)MPU_BASE)
/** Array initializer of MPU peripheral base addresses */
#define MPU_BASE_ADDRS                           { MPU_BASE }
/** Array initializer of MPU peripheral base pointers */
#define MPU_BASE_PTRS                            { MPU }

/* ----------------------------------------------------------------------------
   -- MPU Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MPU_Register_Masks MPU Register Masks
 * @{
 */

/*! @name CESR - Control/Error Status Register */
/*! @{ */
#define MPU_CESR_VLD_MASK                        (0x1U)
#define MPU_CESR_VLD_SHIFT                       (0U)
#define MPU_CESR_VLD_WIDTH                       (1U)
#define MPU_CESR_VLD(x)                          (((uint32_t)(((uint32_t)(x)) << MPU_CESR_VLD_SHIFT)) & MPU_CESR_VLD_MASK)
#define MPU_CESR_NRGD_MASK                       (0xF00U)
#define MPU_CESR_NRGD_SHIFT                      (8U)
#define MPU_CESR_NRGD_WIDTH                      (4U)
#define MPU_CESR_NRGD(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_CESR_NRGD_SHIFT)) & MPU_CESR_NRGD_MASK)
#define MPU_CESR_NSP_MASK                        (0xF000U)
#define MPU_CESR_NSP_SHIFT                       (12U)
#define MPU_CESR_NSP_WIDTH                       (4U)
#define MPU_CESR_NSP(x)                          (((uint32_t)(((uint32_t)(x)) << MPU_CESR_NSP_SHIFT)) & MPU_CESR_NSP_MASK)
#define MPU_CESR_HRL_MASK                        (0xF0000U)
#define MPU_CESR_HRL_SHIFT                       (16U)
#define MPU_CESR_HRL_WIDTH                       (4U)
#define MPU_CESR_HRL(x)                          (((uint32_t)(((uint32_t)(x)) << MPU_CESR_HRL_SHIFT)) & MPU_CESR_HRL_MASK)
#define MPU_CESR_SPERR3_MASK                     (0x10000000U)
#define MPU_CESR_SPERR3_SHIFT                    (28U)
#define MPU_CESR_SPERR3_WIDTH                    (1U)
#define MPU_CESR_SPERR3(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_CESR_SPERR3_SHIFT)) & MPU_CESR_SPERR3_MASK)
#define MPU_CESR_SPERR2_MASK                     (0x20000000U)
#define MPU_CESR_SPERR2_SHIFT                    (29U)
#define MPU_CESR_SPERR2_WIDTH                    (1U)
#define MPU_CESR_SPERR2(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_CESR_SPERR2_SHIFT)) & MPU_CESR_SPERR2_MASK)
#define MPU_CESR_SPERR1_MASK                     (0x40000000U)
#define MPU_CESR_SPERR1_SHIFT                    (30U)
#define MPU_CESR_SPERR1_WIDTH                    (1U)
#define MPU_CESR_SPERR1(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_CESR_SPERR1_SHIFT)) & MPU_CESR_SPERR1_MASK)
#define MPU_CESR_SPERR0_MASK                     (0x80000000U)
#define MPU_CESR_SPERR0_SHIFT                    (31U)
#define MPU_CESR_SPERR0_WIDTH                    (1U)
#define MPU_CESR_SPERR0(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_CESR_SPERR0_SHIFT)) & MPU_CESR_SPERR0_MASK)
/*! @} */

/*! @name EAR - Error Address Register, slave port 0..Error Address Register, slave port 3 */
/*! @{ */
#define MPU_EAR_EADDR_MASK                       (0xFFFFFFFFU)
#define MPU_EAR_EADDR_SHIFT                      (0U)
#define MPU_EAR_EADDR_WIDTH                      (32U)
#define MPU_EAR_EADDR(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_EAR_EADDR_SHIFT)) & MPU_EAR_EADDR_MASK)
/*! @} */

/*! @name EDR - Error Detail Register, slave port 0..Error Detail Register, slave port 3 */
/*! @{ */
#define MPU_EDR_ERW_MASK                         (0x1U)
#define MPU_EDR_ERW_SHIFT                        (0U)
#define MPU_EDR_ERW_WIDTH                        (1U)
#define MPU_EDR_ERW(x)                           (((uint32_t)(((uint32_t)(x)) << MPU_EDR_ERW_SHIFT)) & MPU_EDR_ERW_MASK)
#define MPU_EDR_EATTR_MASK                       (0xEU)
#define MPU_EDR_EATTR_SHIFT                      (1U)
#define MPU_EDR_EATTR_WIDTH                      (3U)
#define MPU_EDR_EATTR(x)                         (((uint32_t)(((uint32_t)(x)) << MPU_EDR_EATTR_SHIFT)) & MPU_EDR_EATTR_MASK)
#define MPU_EDR_EMN_MASK                         (0xF0U)
#define MPU_EDR_EMN_SHIFT                        (4U)
#define MPU_EDR_EMN_WIDTH                        (4U)
#define MPU_EDR_EMN(x)                           (((uint32_t)(((uint32_t)(x)) << MPU_EDR_EMN_SHIFT)) & MPU_EDR_EMN_MASK)
#define MPU_EDR_EPID_MASK                        (0xFF00U)
#define MPU_EDR_EPID_SHIFT                       (8U)
#define MPU_EDR_EPID_WIDTH                       (8U)
#define MPU_EDR_EPID(x)                          (((uint32_t)(((uint32_t)(x)) << MPU_EDR_EPID_SHIFT)) & MPU_EDR_EPID_MASK)
#define MPU_EDR_EACD_MASK                        (0xFFFF0000U)
#define MPU_EDR_EACD_SHIFT                       (16U)
#define MPU_EDR_EACD_WIDTH                       (16U)
#define MPU_EDR_EACD(x)                          (((uint32_t)(((uint32_t)(x)) << MPU_EDR_EACD_SHIFT)) & MPU_EDR_EACD_MASK)
/*! @} */

/*! @name RGD_WORD0 - Region Descriptor 0, Word 0..Region Descriptor 7, Word 0 */
/*! @{ */
#define MPU_RGD_WORD0_SRTADDR_MASK               (0xFFFFFFE0U)
#define MPU_RGD_WORD0_SRTADDR_SHIFT              (5U)
#define MPU_RGD_WORD0_SRTADDR_WIDTH              (27U)
#define MPU_RGD_WORD0_SRTADDR(x)                 (((uint32_t)(((uint32_t)(x)) << MPU_RGD_WORD0_SRTADDR_SHIFT)) & MPU_RGD_WORD0_SRTADDR_MASK)
/*! @} */

/*! @name RGD_WORD1 - Region Descriptor 0, Word 1..Region Descriptor 7, Word 1 */
/*! @{ */
#define MPU_RGD_WORD1_ENDADDR_MASK               (0xFFFFFFE0U)
#define MPU_RGD_WORD1_ENDADDR_SHIFT              (5U)
#define MPU_RGD_WORD1_ENDADDR_WIDTH              (27U)
#define MPU_RGD_WORD1_ENDADDR(x)                 (((uint32_t)(((uint32_t)(x)) << MPU_RGD_WORD1_ENDADDR_SHIFT)) & MPU_RGD_WORD1_ENDADDR_MASK)
/*! @} */

/*! @name RGD_WORD2 - Region Descriptor 0, Word 2..Region Descriptor 7, Word 2 */
/*! @{ */
#define MPU_RGD_WORD2_M0UM_MASK                  (0x7U)
#define MPU_RGD_WORD2_M0UM_SHIFT                 (0U)
#define MPU_RGD_WORD2_M0UM_WIDTH                 (3U)
#define MPU_RGD_WORD2_M0UM(x)                    (((uint32_t)(((uint32_t)(x)) << MPU_RGD_WORD2_M0UM_SHIFT)) & MPU_RGD_WORD2_M0UM_MASK)
#define MPU_RGD_WORD2_M0SM_MASK                  (0x18U)
#define MPU_RGD_WORD2_M0SM_SHIFT                 (3U)
#define MPU_RGD_WORD2_M0SM_WIDTH                 (2U)
#define MPU_RGD_WORD2_M0SM(x)                    (((uint32_t)(((uint32_t)(x)) << MPU_RGD_WORD2_M0SM_SHIFT)) & MPU_RGD_WORD2_M0SM_MASK)
#define MPU_RGD_WORD2_M0PE_MASK                  (0x20U)
#define MPU_RGD_WORD2_M0PE_SHIFT                 (5U)
#define MPU_RGD_WORD2_M0PE_WIDTH                 (1U)
#define MPU_RGD_WORD2_M0PE(x)                    (((uint32_t)(((uint32_t)(x)) << MPU_RGD_WORD2_M0PE_SHIFT)) & MPU_RGD_WORD2_M0PE_MASK)
#define MPU_RGD_WORD2_M1UM_MASK                  (0x1C0U)
#define MPU_RGD_WORD2_M1UM_SHIFT                 (6U)
#define MPU_RGD_WORD2_M1UM_WIDTH                 (3U)
#define MPU_RGD_WORD2_M1UM(x)                    (((uint32_t)(((uint32_t)(x)) << MPU_RGD_WORD2_M1UM_SHIFT)) & MPU_RGD_WORD2_M1UM_MASK)
#define MPU_RGD_WORD2_M1SM_MASK                  (0x600U)
#define MPU_RGD_WORD2_M1SM_SHIFT                 (9U)
#define MPU_RGD_WORD2_M1SM_WIDTH                 (2U)
#define MPU_RGD_WORD2_M1SM(x)                    (((uint32_t)(((uint32_t)(x)) << MPU_RGD_WORD2_M1SM_SHIFT)) & MPU_RGD_WORD2_M1SM_MASK)
#define MPU_RGD_WORD2_M1PE_MASK                  (0x800U)
#define MPU_RGD_WORD2_M1PE_SHIFT                 (11U)
#define MPU_RGD_WORD2_M1PE_WIDTH                 (1U)
#define MPU_RGD_WORD2_M1PE(x)                    (((uint32_t)(((uint32_t)(x)) << MPU_RGD_WORD2_M1PE_SHIFT)) & MPU_RGD_WORD2_M1PE_MASK)
#define MPU_RGD_WORD2_M2UM_MASK                  (0x7000U)
#define MPU_RGD_WORD2_M2UM_SHIFT                 (12U)
#define MPU_RGD_WORD2_M2UM_WIDTH                 (3U)
#define MPU_RGD_WORD2_M2UM(x)                    (((uint32_t)(((uint32_t)(x)) << MPU_RGD_WORD2_M2UM_SHIFT)) & MPU_RGD_WORD2_M2UM_MASK)
#define MPU_RGD_WORD2_M2SM_MASK                  (0x18000U)
#define MPU_RGD_WORD2_M2SM_SHIFT                 (15U)
#define MPU_RGD_WORD2_M2SM_WIDTH                 (2U)
#define MPU_RGD_WORD2_M2SM(x)                    (((uint32_t)(((uint32_t)(x)) << MPU_RGD_WORD2_M2SM_SHIFT)) & MPU_RGD_WORD2_M2SM_MASK)
#define MPU_RGD_WORD2_M3UM_MASK                  (0x1C0000U)
#define MPU_RGD_WORD2_M3UM_SHIFT                 (18U)
#define MPU_RGD_WORD2_M3UM_WIDTH                 (3U)
#define MPU_RGD_WORD2_M3UM(x)                    (((uint32_t)(((uint32_t)(x)) << MPU_RGD_WORD2_M3UM_SHIFT)) & MPU_RGD_WORD2_M3UM_MASK)
#define MPU_RGD_WORD2_M3SM_MASK                  (0x600000U)
#define MPU_RGD_WORD2_M3SM_SHIFT                 (21U)
#define MPU_RGD_WORD2_M3SM_WIDTH                 (2U)
#define MPU_RGD_WORD2_M3SM(x)                    (((uint32_t)(((uint32_t)(x)) << MPU_RGD_WORD2_M3SM_SHIFT)) & MPU_RGD_WORD2_M3SM_MASK)
#define MPU_RGD_WORD2_M4WE_MASK                  (0x1000000U)
#define MPU_RGD_WORD2_M4WE_SHIFT                 (24U)
#define MPU_RGD_WORD2_M4WE_WIDTH                 (1U)
#define MPU_RGD_WORD2_M4WE(x)                    (((uint32_t)(((uint32_t)(x)) << MPU_RGD_WORD2_M4WE_SHIFT)) & MPU_RGD_WORD2_M4WE_MASK)
#define MPU_RGD_WORD2_M4RE_MASK                  (0x2000000U)
#define MPU_RGD_WORD2_M4RE_SHIFT                 (25U)
#define MPU_RGD_WORD2_M4RE_WIDTH                 (1U)
#define MPU_RGD_WORD2_M4RE(x)                    (((uint32_t)(((uint32_t)(x)) << MPU_RGD_WORD2_M4RE_SHIFT)) & MPU_RGD_WORD2_M4RE_MASK)
#define MPU_RGD_WORD2_M5WE_MASK                  (0x4000000U)
#define MPU_RGD_WORD2_M5WE_SHIFT                 (26U)
#define MPU_RGD_WORD2_M5WE_WIDTH                 (1U)
#define MPU_RGD_WORD2_M5WE(x)                    (((uint32_t)(((uint32_t)(x)) << MPU_RGD_WORD2_M5WE_SHIFT)) & MPU_RGD_WORD2_M5WE_MASK)
#define MPU_RGD_WORD2_M5RE_MASK                  (0x8000000U)
#define MPU_RGD_WORD2_M5RE_SHIFT                 (27U)
#define MPU_RGD_WORD2_M5RE_WIDTH                 (1U)
#define MPU_RGD_WORD2_M5RE(x)                    (((uint32_t)(((uint32_t)(x)) << MPU_RGD_WORD2_M5RE_SHIFT)) & MPU_RGD_WORD2_M5RE_MASK)
#define MPU_RGD_WORD2_M6WE_MASK                  (0x10000000U)
#define MPU_RGD_WORD2_M6WE_SHIFT                 (28U)
#define MPU_RGD_WORD2_M6WE_WIDTH                 (1U)
#define MPU_RGD_WORD2_M6WE(x)                    (((uint32_t)(((uint32_t)(x)) << MPU_RGD_WORD2_M6WE_SHIFT)) & MPU_RGD_WORD2_M6WE_MASK)
#define MPU_RGD_WORD2_M6RE_MASK                  (0x20000000U)
#define MPU_RGD_WORD2_M6RE_SHIFT                 (29U)
#define MPU_RGD_WORD2_M6RE_WIDTH                 (1U)
#define MPU_RGD_WORD2_M6RE(x)                    (((uint32_t)(((uint32_t)(x)) << MPU_RGD_WORD2_M6RE_SHIFT)) & MPU_RGD_WORD2_M6RE_MASK)
#define MPU_RGD_WORD2_M7WE_MASK                  (0x40000000U)
#define MPU_RGD_WORD2_M7WE_SHIFT                 (30U)
#define MPU_RGD_WORD2_M7WE_WIDTH                 (1U)
#define MPU_RGD_WORD2_M7WE(x)                    (((uint32_t)(((uint32_t)(x)) << MPU_RGD_WORD2_M7WE_SHIFT)) & MPU_RGD_WORD2_M7WE_MASK)
#define MPU_RGD_WORD2_M7RE_MASK                  (0x80000000U)
#define MPU_RGD_WORD2_M7RE_SHIFT                 (31U)
#define MPU_RGD_WORD2_M7RE_WIDTH                 (1U)
#define MPU_RGD_WORD2_M7RE(x)                    (((uint32_t)(((uint32_t)(x)) << MPU_RGD_WORD2_M7RE_SHIFT)) & MPU_RGD_WORD2_M7RE_MASK)
/*! @} */

/*! @name RGD_WORD3 - Region Descriptor 0, Word 3..Region Descriptor 7, Word 3 */
/*! @{ */
#define MPU_RGD_WORD3_VLD_MASK                   (0x1U)
#define MPU_RGD_WORD3_VLD_SHIFT                  (0U)
#define MPU_RGD_WORD3_VLD_WIDTH                  (1U)
#define MPU_RGD_WORD3_VLD(x)                     (((uint32_t)(((uint32_t)(x)) << MPU_RGD_WORD3_VLD_SHIFT)) & MPU_RGD_WORD3_VLD_MASK)
#define MPU_RGD_WORD3_PIDMASK_MASK               (0xFF0000U)
#define MPU_RGD_WORD3_PIDMASK_SHIFT              (16U)
#define MPU_RGD_WORD3_PIDMASK_WIDTH              (8U)
#define MPU_RGD_WORD3_PIDMASK(x)                 (((uint32_t)(((uint32_t)(x)) << MPU_RGD_WORD3_PIDMASK_SHIFT)) & MPU_RGD_WORD3_PIDMASK_MASK)
#define MPU_RGD_WORD3_PID_MASK                   (0xFF000000U)
#define MPU_RGD_WORD3_PID_SHIFT                  (24U)
#define MPU_RGD_WORD3_PID_WIDTH                  (8U)
#define MPU_RGD_WORD3_PID(x)                     (((uint32_t)(((uint32_t)(x)) << MPU_RGD_WORD3_PID_SHIFT)) & MPU_RGD_WORD3_PID_MASK)
/*! @} */

/*! @name RGDAAC - Region Descriptor Alternate Access Control 0..Region Descriptor Alternate Access Control 7 */
/*! @{ */
#define MPU_RGDAAC_M0UM_MASK                     (0x7U)
#define MPU_RGDAAC_M0UM_SHIFT                    (0U)
#define MPU_RGDAAC_M0UM_WIDTH                    (3U)
#define MPU_RGDAAC_M0UM(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M0UM_SHIFT)) & MPU_RGDAAC_M0UM_MASK)
#define MPU_RGDAAC_M0SM_MASK                     (0x18U)
#define MPU_RGDAAC_M0SM_SHIFT                    (3U)
#define MPU_RGDAAC_M0SM_WIDTH                    (2U)
#define MPU_RGDAAC_M0SM(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M0SM_SHIFT)) & MPU_RGDAAC_M0SM_MASK)
#define MPU_RGDAAC_M0PE_MASK                     (0x20U)
#define MPU_RGDAAC_M0PE_SHIFT                    (5U)
#define MPU_RGDAAC_M0PE_WIDTH                    (1U)
#define MPU_RGDAAC_M0PE(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M0PE_SHIFT)) & MPU_RGDAAC_M0PE_MASK)
#define MPU_RGDAAC_M1UM_MASK                     (0x1C0U)
#define MPU_RGDAAC_M1UM_SHIFT                    (6U)
#define MPU_RGDAAC_M1UM_WIDTH                    (3U)
#define MPU_RGDAAC_M1UM(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M1UM_SHIFT)) & MPU_RGDAAC_M1UM_MASK)
#define MPU_RGDAAC_M1SM_MASK                     (0x600U)
#define MPU_RGDAAC_M1SM_SHIFT                    (9U)
#define MPU_RGDAAC_M1SM_WIDTH                    (2U)
#define MPU_RGDAAC_M1SM(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M1SM_SHIFT)) & MPU_RGDAAC_M1SM_MASK)
#define MPU_RGDAAC_M1PE_MASK                     (0x800U)
#define MPU_RGDAAC_M1PE_SHIFT                    (11U)
#define MPU_RGDAAC_M1PE_WIDTH                    (1U)
#define MPU_RGDAAC_M1PE(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M1PE_SHIFT)) & MPU_RGDAAC_M1PE_MASK)
#define MPU_RGDAAC_M2UM_MASK                     (0x7000U)
#define MPU_RGDAAC_M2UM_SHIFT                    (12U)
#define MPU_RGDAAC_M2UM_WIDTH                    (3U)
#define MPU_RGDAAC_M2UM(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M2UM_SHIFT)) & MPU_RGDAAC_M2UM_MASK)
#define MPU_RGDAAC_M2SM_MASK                     (0x18000U)
#define MPU_RGDAAC_M2SM_SHIFT                    (15U)
#define MPU_RGDAAC_M2SM_WIDTH                    (2U)
#define MPU_RGDAAC_M2SM(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M2SM_SHIFT)) & MPU_RGDAAC_M2SM_MASK)
#define MPU_RGDAAC_M3UM_MASK                     (0x1C0000U)
#define MPU_RGDAAC_M3UM_SHIFT                    (18U)
#define MPU_RGDAAC_M3UM_WIDTH                    (3U)
#define MPU_RGDAAC_M3UM(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M3UM_SHIFT)) & MPU_RGDAAC_M3UM_MASK)
#define MPU_RGDAAC_M3SM_MASK                     (0x600000U)
#define MPU_RGDAAC_M3SM_SHIFT                    (21U)
#define MPU_RGDAAC_M3SM_WIDTH                    (2U)
#define MPU_RGDAAC_M3SM(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M3SM_SHIFT)) & MPU_RGDAAC_M3SM_MASK)
#define MPU_RGDAAC_M4WE_MASK                     (0x1000000U)
#define MPU_RGDAAC_M4WE_SHIFT                    (24U)
#define MPU_RGDAAC_M4WE_WIDTH                    (1U)
#define MPU_RGDAAC_M4WE(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M4WE_SHIFT)) & MPU_RGDAAC_M4WE_MASK)
#define MPU_RGDAAC_M4RE_MASK                     (0x2000000U)
#define MPU_RGDAAC_M4RE_SHIFT                    (25U)
#define MPU_RGDAAC_M4RE_WIDTH                    (1U)
#define MPU_RGDAAC_M4RE(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M4RE_SHIFT)) & MPU_RGDAAC_M4RE_MASK)
#define MPU_RGDAAC_M5WE_MASK                     (0x4000000U)
#define MPU_RGDAAC_M5WE_SHIFT                    (26U)
#define MPU_RGDAAC_M5WE_WIDTH                    (1U)
#define MPU_RGDAAC_M5WE(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M5WE_SHIFT)) & MPU_RGDAAC_M5WE_MASK)
#define MPU_RGDAAC_M5RE_MASK                     (0x8000000U)
#define MPU_RGDAAC_M5RE_SHIFT                    (27U)
#define MPU_RGDAAC_M5RE_WIDTH                    (1U)
#define MPU_RGDAAC_M5RE(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M5RE_SHIFT)) & MPU_RGDAAC_M5RE_MASK)
#define MPU_RGDAAC_M6WE_MASK                     (0x10000000U)
#define MPU_RGDAAC_M6WE_SHIFT                    (28U)
#define MPU_RGDAAC_M6WE_WIDTH                    (1U)
#define MPU_RGDAAC_M6WE(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M6WE_SHIFT)) & MPU_RGDAAC_M6WE_MASK)
#define MPU_RGDAAC_M6RE_MASK                     (0x20000000U)
#define MPU_RGDAAC_M6RE_SHIFT                    (29U)
#define MPU_RGDAAC_M6RE_WIDTH                    (1U)
#define MPU_RGDAAC_M6RE(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M6RE_SHIFT)) & MPU_RGDAAC_M6RE_MASK)
#define MPU_RGDAAC_M7WE_MASK                     (0x40000000U)
#define MPU_RGDAAC_M7WE_SHIFT                    (30U)
#define MPU_RGDAAC_M7WE_WIDTH                    (1U)
#define MPU_RGDAAC_M7WE(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M7WE_SHIFT)) & MPU_RGDAAC_M7WE_MASK)
#define MPU_RGDAAC_M7RE_MASK                     (0x80000000U)
#define MPU_RGDAAC_M7RE_SHIFT                    (31U)
#define MPU_RGDAAC_M7RE_WIDTH                    (1U)
#define MPU_RGDAAC_M7RE(x)                       (((uint32_t)(((uint32_t)(x)) << MPU_RGDAAC_M7RE_SHIFT)) & MPU_RGDAAC_M7RE_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group MPU_Register_Masks */

/*!
 * @}
 */ /* end of group MPU_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- MSCM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MSCM_Peripheral_Access_Layer MSCM Peripheral Access Layer
 * @{
 */

/** MSCM - Size of Registers Arrays */
#define MSCM_OCMDR_COUNT                          3u

/** MSCM - Register Layout Typedef */
typedef struct {
  __I  uint32_t CPxTYPE;                           /**< Processor X Type Register, offset: 0x0 */
  __I  uint32_t CPxNUM;                            /**< Processor X Number Register, offset: 0x4 */
  __I  uint32_t CPxMASTER;                         /**< Processor X Master Register, offset: 0x8 */
  __I  uint32_t CPxCOUNT;                          /**< Processor X Count Register, offset: 0xC */
  __I  uint32_t CPxCFG0;                           /**< Processor X Configuration Register 0, offset: 0x10 */
  __I  uint32_t CPxCFG1;                           /**< Processor X Configuration Register 1, offset: 0x14 */
  __I  uint32_t CPxCFG2;                           /**< Processor X Configuration Register 2, offset: 0x18 */
  __I  uint32_t CPxCFG3;                           /**< Processor X Configuration Register 3, offset: 0x1C */
  __I  uint32_t CP0TYPE;                           /**< Processor 0 Type Register, offset: 0x20 */
  __I  uint32_t CP0NUM;                            /**< Processor 0 Number Register, offset: 0x24 */
  __I  uint32_t CP0MASTER;                         /**< Processor 0 Master Register, offset: 0x28 */
  __I  uint32_t CP0COUNT;                          /**< Processor 0 Count Register, offset: 0x2C */
  __I  uint32_t CP0CFG0;                           /**< Processor 0 Configuration Register 0, offset: 0x30 */
  __I  uint32_t CP0CFG1;                           /**< Processor 0 Configuration Register 1, offset: 0x34 */
  __I  uint32_t CP0CFG2;                           /**< Processor 0 Configuration Register 2, offset: 0x38 */
  __I  uint32_t CP0CFG3;                           /**< Processor 0 Configuration Register 3, offset: 0x3C */
  uint8_t RESERVED_0[960];
  __IO uint32_t OCMDR[MSCM_OCMDR_COUNT];           /**< On-Chip Memory Descriptor Register, array offset: 0x400, array step: 0x4 */
} MSCM_Type, *MSCM_MemMapPtr;

/** Number of instances of the MSCM module. */
#define MSCM_INSTANCE_COUNT                      (1u)

/* MSCM - Peripheral instance base addresses */
/** Peripheral MSCM base address */
#define MSCM_BASE                                (0x40001000u)
/** Peripheral MSCM base pointer */
#define MSCM                                     ((MSCM_Type *)MSCM_BASE)
/** Array initializer of MSCM peripheral base addresses */
#define MSCM_BASE_ADDRS                          { MSCM_BASE }
/** Array initializer of MSCM peripheral base pointers */
#define MSCM_BASE_PTRS                           { MSCM }

/* ----------------------------------------------------------------------------
   -- MSCM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MSCM_Register_Masks MSCM Register Masks
 * @{
 */

/*! @name CPxTYPE - Processor X Type Register */
/*! @{ */
#define MSCM_CPxTYPE_RYPZ_MASK                   (0xFFU)
#define MSCM_CPxTYPE_RYPZ_SHIFT                  (0U)
#define MSCM_CPxTYPE_RYPZ_WIDTH                  (8U)
#define MSCM_CPxTYPE_RYPZ(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CPxTYPE_RYPZ_SHIFT)) & MSCM_CPxTYPE_RYPZ_MASK)
#define MSCM_CPxTYPE_PERSONALITY_MASK            (0xFFFFFF00U)
#define MSCM_CPxTYPE_PERSONALITY_SHIFT           (8U)
#define MSCM_CPxTYPE_PERSONALITY_WIDTH           (24U)
#define MSCM_CPxTYPE_PERSONALITY(x)              (((uint32_t)(((uint32_t)(x)) << MSCM_CPxTYPE_PERSONALITY_SHIFT)) & MSCM_CPxTYPE_PERSONALITY_MASK)
/*! @} */

/*! @name CPxNUM - Processor X Number Register */
/*! @{ */
#define MSCM_CPxNUM_CPN_MASK                     (0x1U)
#define MSCM_CPxNUM_CPN_SHIFT                    (0U)
#define MSCM_CPxNUM_CPN_WIDTH                    (1U)
#define MSCM_CPxNUM_CPN(x)                       (((uint32_t)(((uint32_t)(x)) << MSCM_CPxNUM_CPN_SHIFT)) & MSCM_CPxNUM_CPN_MASK)
/*! @} */

/*! @name CPxMASTER - Processor X Master Register */
/*! @{ */
#define MSCM_CPxMASTER_PPMN_MASK                 (0x3FU)
#define MSCM_CPxMASTER_PPMN_SHIFT                (0U)
#define MSCM_CPxMASTER_PPMN_WIDTH                (6U)
#define MSCM_CPxMASTER_PPMN(x)                   (((uint32_t)(((uint32_t)(x)) << MSCM_CPxMASTER_PPMN_SHIFT)) & MSCM_CPxMASTER_PPMN_MASK)
/*! @} */

/*! @name CPxCOUNT - Processor X Count Register */
/*! @{ */
#define MSCM_CPxCOUNT_PCNT_MASK                  (0x3U)
#define MSCM_CPxCOUNT_PCNT_SHIFT                 (0U)
#define MSCM_CPxCOUNT_PCNT_WIDTH                 (2U)
#define MSCM_CPxCOUNT_PCNT(x)                    (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCOUNT_PCNT_SHIFT)) & MSCM_CPxCOUNT_PCNT_MASK)
/*! @} */

/*! @name CPxCFG0 - Processor X Configuration Register 0 */
/*! @{ */
#define MSCM_CPxCFG0_DCWY_MASK                   (0xFFU)
#define MSCM_CPxCFG0_DCWY_SHIFT                  (0U)
#define MSCM_CPxCFG0_DCWY_WIDTH                  (8U)
#define MSCM_CPxCFG0_DCWY(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG0_DCWY_SHIFT)) & MSCM_CPxCFG0_DCWY_MASK)
#define MSCM_CPxCFG0_DCSZ_MASK                   (0xFF00U)
#define MSCM_CPxCFG0_DCSZ_SHIFT                  (8U)
#define MSCM_CPxCFG0_DCSZ_WIDTH                  (8U)
#define MSCM_CPxCFG0_DCSZ(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG0_DCSZ_SHIFT)) & MSCM_CPxCFG0_DCSZ_MASK)
#define MSCM_CPxCFG0_ICWY_MASK                   (0xFF0000U)
#define MSCM_CPxCFG0_ICWY_SHIFT                  (16U)
#define MSCM_CPxCFG0_ICWY_WIDTH                  (8U)
#define MSCM_CPxCFG0_ICWY(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG0_ICWY_SHIFT)) & MSCM_CPxCFG0_ICWY_MASK)
#define MSCM_CPxCFG0_ICSZ_MASK                   (0xFF000000U)
#define MSCM_CPxCFG0_ICSZ_SHIFT                  (24U)
#define MSCM_CPxCFG0_ICSZ_WIDTH                  (8U)
#define MSCM_CPxCFG0_ICSZ(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG0_ICSZ_SHIFT)) & MSCM_CPxCFG0_ICSZ_MASK)
/*! @} */

/*! @name CPxCFG1 - Processor X Configuration Register 1 */
/*! @{ */
#define MSCM_CPxCFG1_L2WY_MASK                   (0xFF0000U)
#define MSCM_CPxCFG1_L2WY_SHIFT                  (16U)
#define MSCM_CPxCFG1_L2WY_WIDTH                  (8U)
#define MSCM_CPxCFG1_L2WY(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG1_L2WY_SHIFT)) & MSCM_CPxCFG1_L2WY_MASK)
#define MSCM_CPxCFG1_L2SZ_MASK                   (0xFF000000U)
#define MSCM_CPxCFG1_L2SZ_SHIFT                  (24U)
#define MSCM_CPxCFG1_L2SZ_WIDTH                  (8U)
#define MSCM_CPxCFG1_L2SZ(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG1_L2SZ_SHIFT)) & MSCM_CPxCFG1_L2SZ_MASK)
/*! @} */

/*! @name CPxCFG2 - Processor X Configuration Register 2 */
/*! @{ */
#define MSCM_CPxCFG2_TMUSZ_MASK                  (0xFF00U)
#define MSCM_CPxCFG2_TMUSZ_SHIFT                 (8U)
#define MSCM_CPxCFG2_TMUSZ_WIDTH                 (8U)
#define MSCM_CPxCFG2_TMUSZ(x)                    (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG2_TMUSZ_SHIFT)) & MSCM_CPxCFG2_TMUSZ_MASK)
#define MSCM_CPxCFG2_TMLSZ_MASK                  (0xFF000000U)
#define MSCM_CPxCFG2_TMLSZ_SHIFT                 (24U)
#define MSCM_CPxCFG2_TMLSZ_WIDTH                 (8U)
#define MSCM_CPxCFG2_TMLSZ(x)                    (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG2_TMLSZ_SHIFT)) & MSCM_CPxCFG2_TMLSZ_MASK)
/*! @} */

/*! @name CPxCFG3 - Processor X Configuration Register 3 */
/*! @{ */
#define MSCM_CPxCFG3_FPU_MASK                    (0x1U)
#define MSCM_CPxCFG3_FPU_SHIFT                   (0U)
#define MSCM_CPxCFG3_FPU_WIDTH                   (1U)
#define MSCM_CPxCFG3_FPU(x)                      (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG3_FPU_SHIFT)) & MSCM_CPxCFG3_FPU_MASK)
#define MSCM_CPxCFG3_SIMD_MASK                   (0x2U)
#define MSCM_CPxCFG3_SIMD_SHIFT                  (1U)
#define MSCM_CPxCFG3_SIMD_WIDTH                  (1U)
#define MSCM_CPxCFG3_SIMD(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG3_SIMD_SHIFT)) & MSCM_CPxCFG3_SIMD_MASK)
#define MSCM_CPxCFG3_JAZ_MASK                    (0x4U)
#define MSCM_CPxCFG3_JAZ_SHIFT                   (2U)
#define MSCM_CPxCFG3_JAZ_WIDTH                   (1U)
#define MSCM_CPxCFG3_JAZ(x)                      (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG3_JAZ_SHIFT)) & MSCM_CPxCFG3_JAZ_MASK)
#define MSCM_CPxCFG3_MMU_MASK                    (0x8U)
#define MSCM_CPxCFG3_MMU_SHIFT                   (3U)
#define MSCM_CPxCFG3_MMU_WIDTH                   (1U)
#define MSCM_CPxCFG3_MMU(x)                      (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG3_MMU_SHIFT)) & MSCM_CPxCFG3_MMU_MASK)
#define MSCM_CPxCFG3_TZ_MASK                     (0x10U)
#define MSCM_CPxCFG3_TZ_SHIFT                    (4U)
#define MSCM_CPxCFG3_TZ_WIDTH                    (1U)
#define MSCM_CPxCFG3_TZ(x)                       (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG3_TZ_SHIFT)) & MSCM_CPxCFG3_TZ_MASK)
#define MSCM_CPxCFG3_CMP_MASK                    (0x20U)
#define MSCM_CPxCFG3_CMP_SHIFT                   (5U)
#define MSCM_CPxCFG3_CMP_WIDTH                   (1U)
#define MSCM_CPxCFG3_CMP(x)                      (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG3_CMP_SHIFT)) & MSCM_CPxCFG3_CMP_MASK)
#define MSCM_CPxCFG3_BB_MASK                     (0x40U)
#define MSCM_CPxCFG3_BB_SHIFT                    (6U)
#define MSCM_CPxCFG3_BB_WIDTH                    (1U)
#define MSCM_CPxCFG3_BB(x)                       (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG3_BB_SHIFT)) & MSCM_CPxCFG3_BB_MASK)
#define MSCM_CPxCFG3_SBP_MASK                    (0x300U)
#define MSCM_CPxCFG3_SBP_SHIFT                   (8U)
#define MSCM_CPxCFG3_SBP_WIDTH                   (2U)
#define MSCM_CPxCFG3_SBP(x)                      (((uint32_t)(((uint32_t)(x)) << MSCM_CPxCFG3_SBP_SHIFT)) & MSCM_CPxCFG3_SBP_MASK)
/*! @} */

/*! @name CP0TYPE - Processor 0 Type Register */
/*! @{ */
#define MSCM_CP0TYPE_RYPZ_MASK                   (0xFFU)
#define MSCM_CP0TYPE_RYPZ_SHIFT                  (0U)
#define MSCM_CP0TYPE_RYPZ_WIDTH                  (8U)
#define MSCM_CP0TYPE_RYPZ(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CP0TYPE_RYPZ_SHIFT)) & MSCM_CP0TYPE_RYPZ_MASK)
#define MSCM_CP0TYPE_PERSONALITY_MASK            (0xFFFFFF00U)
#define MSCM_CP0TYPE_PERSONALITY_SHIFT           (8U)
#define MSCM_CP0TYPE_PERSONALITY_WIDTH           (24U)
#define MSCM_CP0TYPE_PERSONALITY(x)              (((uint32_t)(((uint32_t)(x)) << MSCM_CP0TYPE_PERSONALITY_SHIFT)) & MSCM_CP0TYPE_PERSONALITY_MASK)
/*! @} */

/*! @name CP0NUM - Processor 0 Number Register */
/*! @{ */
#define MSCM_CP0NUM_CPN_MASK                     (0x1U)
#define MSCM_CP0NUM_CPN_SHIFT                    (0U)
#define MSCM_CP0NUM_CPN_WIDTH                    (1U)
#define MSCM_CP0NUM_CPN(x)                       (((uint32_t)(((uint32_t)(x)) << MSCM_CP0NUM_CPN_SHIFT)) & MSCM_CP0NUM_CPN_MASK)
/*! @} */

/*! @name CP0MASTER - Processor 0 Master Register */
/*! @{ */
#define MSCM_CP0MASTER_PPMN_MASK                 (0x3FU)
#define MSCM_CP0MASTER_PPMN_SHIFT                (0U)
#define MSCM_CP0MASTER_PPMN_WIDTH                (6U)
#define MSCM_CP0MASTER_PPMN(x)                   (((uint32_t)(((uint32_t)(x)) << MSCM_CP0MASTER_PPMN_SHIFT)) & MSCM_CP0MASTER_PPMN_MASK)
/*! @} */

/*! @name CP0COUNT - Processor 0 Count Register */
/*! @{ */
#define MSCM_CP0COUNT_PCNT_MASK                  (0x3U)
#define MSCM_CP0COUNT_PCNT_SHIFT                 (0U)
#define MSCM_CP0COUNT_PCNT_WIDTH                 (2U)
#define MSCM_CP0COUNT_PCNT(x)                    (((uint32_t)(((uint32_t)(x)) << MSCM_CP0COUNT_PCNT_SHIFT)) & MSCM_CP0COUNT_PCNT_MASK)
/*! @} */

/*! @name CP0CFG0 - Processor 0 Configuration Register 0 */
/*! @{ */
#define MSCM_CP0CFG0_DCWY_MASK                   (0xFFU)
#define MSCM_CP0CFG0_DCWY_SHIFT                  (0U)
#define MSCM_CP0CFG0_DCWY_WIDTH                  (8U)
#define MSCM_CP0CFG0_DCWY(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CP0CFG0_DCWY_SHIFT)) & MSCM_CP0CFG0_DCWY_MASK)
#define MSCM_CP0CFG0_DCSZ_MASK                   (0xFF00U)
#define MSCM_CP0CFG0_DCSZ_SHIFT                  (8U)
#define MSCM_CP0CFG0_DCSZ_WIDTH                  (8U)
#define MSCM_CP0CFG0_DCSZ(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CP0CFG0_DCSZ_SHIFT)) & MSCM_CP0CFG0_DCSZ_MASK)
#define MSCM_CP0CFG0_ICWY_MASK                   (0xFF0000U)
#define MSCM_CP0CFG0_ICWY_SHIFT                  (16U)
#define MSCM_CP0CFG0_ICWY_WIDTH                  (8U)
#define MSCM_CP0CFG0_ICWY(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CP0CFG0_ICWY_SHIFT)) & MSCM_CP0CFG0_ICWY_MASK)
#define MSCM_CP0CFG0_ICSZ_MASK                   (0xFF000000U)
#define MSCM_CP0CFG0_ICSZ_SHIFT                  (24U)
#define MSCM_CP0CFG0_ICSZ_WIDTH                  (8U)
#define MSCM_CP0CFG0_ICSZ(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CP0CFG0_ICSZ_SHIFT)) & MSCM_CP0CFG0_ICSZ_MASK)
/*! @} */

/*! @name CP0CFG1 - Processor 0 Configuration Register 1 */
/*! @{ */
#define MSCM_CP0CFG1_L2WY_MASK                   (0xFF0000U)
#define MSCM_CP0CFG1_L2WY_SHIFT                  (16U)
#define MSCM_CP0CFG1_L2WY_WIDTH                  (8U)
#define MSCM_CP0CFG1_L2WY(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CP0CFG1_L2WY_SHIFT)) & MSCM_CP0CFG1_L2WY_MASK)
#define MSCM_CP0CFG1_L2SZ_MASK                   (0xFF000000U)
#define MSCM_CP0CFG1_L2SZ_SHIFT                  (24U)
#define MSCM_CP0CFG1_L2SZ_WIDTH                  (8U)
#define MSCM_CP0CFG1_L2SZ(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CP0CFG1_L2SZ_SHIFT)) & MSCM_CP0CFG1_L2SZ_MASK)
/*! @} */

/*! @name CP0CFG2 - Processor 0 Configuration Register 2 */
/*! @{ */
#define MSCM_CP0CFG2_TMUSZ_MASK                  (0xFF00U)
#define MSCM_CP0CFG2_TMUSZ_SHIFT                 (8U)
#define MSCM_CP0CFG2_TMUSZ_WIDTH                 (8U)
#define MSCM_CP0CFG2_TMUSZ(x)                    (((uint32_t)(((uint32_t)(x)) << MSCM_CP0CFG2_TMUSZ_SHIFT)) & MSCM_CP0CFG2_TMUSZ_MASK)
#define MSCM_CP0CFG2_TMLSZ_MASK                  (0xFF000000U)
#define MSCM_CP0CFG2_TMLSZ_SHIFT                 (24U)
#define MSCM_CP0CFG2_TMLSZ_WIDTH                 (8U)
#define MSCM_CP0CFG2_TMLSZ(x)                    (((uint32_t)(((uint32_t)(x)) << MSCM_CP0CFG2_TMLSZ_SHIFT)) & MSCM_CP0CFG2_TMLSZ_MASK)
/*! @} */

/*! @name CP0CFG3 - Processor 0 Configuration Register 3 */
/*! @{ */
#define MSCM_CP0CFG3_FPU_MASK                    (0x1U)
#define MSCM_CP0CFG3_FPU_SHIFT                   (0U)
#define MSCM_CP0CFG3_FPU_WIDTH                   (1U)
#define MSCM_CP0CFG3_FPU(x)                      (((uint32_t)(((uint32_t)(x)) << MSCM_CP0CFG3_FPU_SHIFT)) & MSCM_CP0CFG3_FPU_MASK)
#define MSCM_CP0CFG3_SIMD_MASK                   (0x2U)
#define MSCM_CP0CFG3_SIMD_SHIFT                  (1U)
#define MSCM_CP0CFG3_SIMD_WIDTH                  (1U)
#define MSCM_CP0CFG3_SIMD(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_CP0CFG3_SIMD_SHIFT)) & MSCM_CP0CFG3_SIMD_MASK)
#define MSCM_CP0CFG3_JAZ_MASK                    (0x4U)
#define MSCM_CP0CFG3_JAZ_SHIFT                   (2U)
#define MSCM_CP0CFG3_JAZ_WIDTH                   (1U)
#define MSCM_CP0CFG3_JAZ(x)                      (((uint32_t)(((uint32_t)(x)) << MSCM_CP0CFG3_JAZ_SHIFT)) & MSCM_CP0CFG3_JAZ_MASK)
#define MSCM_CP0CFG3_MMU_MASK                    (0x8U)
#define MSCM_CP0CFG3_MMU_SHIFT                   (3U)
#define MSCM_CP0CFG3_MMU_WIDTH                   (1U)
#define MSCM_CP0CFG3_MMU(x)                      (((uint32_t)(((uint32_t)(x)) << MSCM_CP0CFG3_MMU_SHIFT)) & MSCM_CP0CFG3_MMU_MASK)
#define MSCM_CP0CFG3_TZ_MASK                     (0x10U)
#define MSCM_CP0CFG3_TZ_SHIFT                    (4U)
#define MSCM_CP0CFG3_TZ_WIDTH                    (1U)
#define MSCM_CP0CFG3_TZ(x)                       (((uint32_t)(((uint32_t)(x)) << MSCM_CP0CFG3_TZ_SHIFT)) & MSCM_CP0CFG3_TZ_MASK)
#define MSCM_CP0CFG3_CMP_MASK                    (0x20U)
#define MSCM_CP0CFG3_CMP_SHIFT                   (5U)
#define MSCM_CP0CFG3_CMP_WIDTH                   (1U)
#define MSCM_CP0CFG3_CMP(x)                      (((uint32_t)(((uint32_t)(x)) << MSCM_CP0CFG3_CMP_SHIFT)) & MSCM_CP0CFG3_CMP_MASK)
#define MSCM_CP0CFG3_BB_MASK                     (0x40U)
#define MSCM_CP0CFG3_BB_SHIFT                    (6U)
#define MSCM_CP0CFG3_BB_WIDTH                    (1U)
#define MSCM_CP0CFG3_BB(x)                       (((uint32_t)(((uint32_t)(x)) << MSCM_CP0CFG3_BB_SHIFT)) & MSCM_CP0CFG3_BB_MASK)
#define MSCM_CP0CFG3_SBP_MASK                    (0x300U)
#define MSCM_CP0CFG3_SBP_SHIFT                   (8U)
#define MSCM_CP0CFG3_SBP_WIDTH                   (2U)
#define MSCM_CP0CFG3_SBP(x)                      (((uint32_t)(((uint32_t)(x)) << MSCM_CP0CFG3_SBP_SHIFT)) & MSCM_CP0CFG3_SBP_MASK)
/*! @} */

/*! @name OCMDR - On-Chip Memory Descriptor Register */
/*! @{ */
#define MSCM_OCMDR_OCM1_MASK                     (0x30U)
#define MSCM_OCMDR_OCM1_SHIFT                    (4U)
#define MSCM_OCMDR_OCM1_WIDTH                    (2U)
#define MSCM_OCMDR_OCM1(x)                       (((uint32_t)(((uint32_t)(x)) << MSCM_OCMDR_OCM1_SHIFT)) & MSCM_OCMDR_OCM1_MASK)
#define MSCM_OCMDR_OCMPU_MASK                    (0x1000U)
#define MSCM_OCMDR_OCMPU_SHIFT                   (12U)
#define MSCM_OCMDR_OCMPU_WIDTH                   (1U)
#define MSCM_OCMDR_OCMPU(x)                      (((uint32_t)(((uint32_t)(x)) << MSCM_OCMDR_OCMPU_SHIFT)) & MSCM_OCMDR_OCMPU_MASK)
#define MSCM_OCMDR_OCMT_MASK                     (0xE000U)
#define MSCM_OCMDR_OCMT_SHIFT                    (13U)
#define MSCM_OCMDR_OCMT_WIDTH                    (3U)
#define MSCM_OCMDR_OCMT(x)                       (((uint32_t)(((uint32_t)(x)) << MSCM_OCMDR_OCMT_SHIFT)) & MSCM_OCMDR_OCMT_MASK)
#define MSCM_OCMDR_RO_MASK                       (0x10000U)
#define MSCM_OCMDR_RO_SHIFT                      (16U)
#define MSCM_OCMDR_RO_WIDTH                      (1U)
#define MSCM_OCMDR_RO(x)                         (((uint32_t)(((uint32_t)(x)) << MSCM_OCMDR_RO_SHIFT)) & MSCM_OCMDR_RO_MASK)
#define MSCM_OCMDR_OCMW_MASK                     (0xE0000U)
#define MSCM_OCMDR_OCMW_SHIFT                    (17U)
#define MSCM_OCMDR_OCMW_WIDTH                    (3U)
#define MSCM_OCMDR_OCMW(x)                       (((uint32_t)(((uint32_t)(x)) << MSCM_OCMDR_OCMW_SHIFT)) & MSCM_OCMDR_OCMW_MASK)
#define MSCM_OCMDR_OCMSZ_MASK                    (0xF000000U)
#define MSCM_OCMDR_OCMSZ_SHIFT                   (24U)
#define MSCM_OCMDR_OCMSZ_WIDTH                   (4U)
#define MSCM_OCMDR_OCMSZ(x)                      (((uint32_t)(((uint32_t)(x)) << MSCM_OCMDR_OCMSZ_SHIFT)) & MSCM_OCMDR_OCMSZ_MASK)
#define MSCM_OCMDR_OCMSZH_MASK                   (0x10000000U)
#define MSCM_OCMDR_OCMSZH_SHIFT                  (28U)
#define MSCM_OCMDR_OCMSZH_WIDTH                  (1U)
#define MSCM_OCMDR_OCMSZH(x)                     (((uint32_t)(((uint32_t)(x)) << MSCM_OCMDR_OCMSZH_SHIFT)) & MSCM_OCMDR_OCMSZH_MASK)
#define MSCM_OCMDR_V_MASK                        (0x80000000U)
#define MSCM_OCMDR_V_SHIFT                       (31U)
#define MSCM_OCMDR_V_WIDTH                       (1U)
#define MSCM_OCMDR_V(x)                          (((uint32_t)(((uint32_t)(x)) << MSCM_OCMDR_V_SHIFT)) & MSCM_OCMDR_V_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group MSCM_Register_Masks */

/*!
 * @}
 */ /* end of group MSCM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- PCC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PCC_Peripheral_Access_Layer PCC Peripheral Access Layer
 * @{
 */

/** PCC - Size of Registers Arrays */
#define PCC_PCCn_COUNT                            116u

/** PCC - Register Layout Typedef */
typedef struct {
  __IO uint32_t PCCn[PCC_PCCn_COUNT];              /**< PCC FTFM Register..PCC CMP0 Register, array offset: 0x0, array step: 0x4 */
} PCC_Type, *PCC_MemMapPtr;

/** Number of instances of the PCC module. */
#define PCC_INSTANCE_COUNT                       (1u)

/* PCC - Peripheral instance base addresses */
/** Peripheral PCC base address */
#define PCC_BASE                                 (0x40065000u)
/** Peripheral PCC base pointer */
#define PCC                                      ((PCC_Type *)PCC_BASE)
/** Array initializer of PCC peripheral base addresses */
#define PCC_BASE_ADDRS                           { PCC_BASE }
/** Array initializer of PCC peripheral base pointers */
#define PCC_BASE_PTRS                            { PCC }

/* ----------------------------------------------------------------------------
   -- PCC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PCC_Register_Masks PCC Register Masks
 * @{
 */

/*! @name PCCn - PCC FTFM Register..PCC CMP0 Register */
/*! @{ */
#define PCC_PCCn_PCD_MASK                        (0x7U)
#define PCC_PCCn_PCD_SHIFT                       (0U)
#define PCC_PCCn_PCD_WIDTH                       (3U)
#define PCC_PCCn_PCD(x)                          (((uint32_t)(((uint32_t)(x)) << PCC_PCCn_PCD_SHIFT)) & PCC_PCCn_PCD_MASK)
#define PCC_PCCn_FRAC_MASK                       (0x8U)
#define PCC_PCCn_FRAC_SHIFT                      (3U)
#define PCC_PCCn_FRAC_WIDTH                      (1U)
#define PCC_PCCn_FRAC(x)                         (((uint32_t)(((uint32_t)(x)) << PCC_PCCn_FRAC_SHIFT)) & PCC_PCCn_FRAC_MASK)
#define PCC_PCCn_PCS_MASK                        (0x7000000U)
#define PCC_PCCn_PCS_SHIFT                       (24U)
#define PCC_PCCn_PCS_WIDTH                       (3U)
#define PCC_PCCn_PCS(x)                          (((uint32_t)(((uint32_t)(x)) << PCC_PCCn_PCS_SHIFT)) & PCC_PCCn_PCS_MASK)
#define PCC_PCCn_CGC_MASK                        (0x40000000U)
#define PCC_PCCn_CGC_SHIFT                       (30U)
#define PCC_PCCn_CGC_WIDTH                       (1U)
#define PCC_PCCn_CGC(x)                          (((uint32_t)(((uint32_t)(x)) << PCC_PCCn_CGC_SHIFT)) & PCC_PCCn_CGC_MASK)
#define PCC_PCCn_PR_MASK                         (0x80000000U)
#define PCC_PCCn_PR_SHIFT                        (31U)
#define PCC_PCCn_PR_WIDTH                        (1U)
#define PCC_PCCn_PR(x)                           (((uint32_t)(((uint32_t)(x)) << PCC_PCCn_PR_SHIFT)) & PCC_PCCn_PR_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group PCC_Register_Masks */
#define PCC_FTFM_INDEX 32
#define PCC_DMAMUX_INDEX 33
#define PCC_FlexCAN0_INDEX 36
#define PCC_FlexCAN1_INDEX 37
#define PCC_FTM3_INDEX 38
#define PCC_ADC1_INDEX 39
#define PCC_LPSPI0_INDEX 44
#define PCC_LPSPI1_INDEX 45
#define PCC_LPSPI2_INDEX 46
#define PCC_PDB1_INDEX 49
#define PCC_CRC_INDEX 50
#define PCC_PDB0_INDEX 54
#define PCC_LPIT_INDEX 55
#define PCC_FTM0_INDEX 56
#define PCC_FTM1_INDEX 57
#define PCC_FTM2_INDEX 58
#define PCC_ADC0_INDEX 59
#define PCC_RTC_INDEX 61
#define PCC_LPTMR0_INDEX 64
#define PCC_PORTA_INDEX 73
#define PCC_PORTB_INDEX 74
#define PCC_PORTC_INDEX 75
#define PCC_PORTD_INDEX 76
#define PCC_PORTE_INDEX 77
#define PCC_FlexIO_INDEX 90
#define PCC_EWM_INDEX 97
#define PCC_LPI2C0_INDEX 102
#define PCC_LPUART0_INDEX 106
#define PCC_LPUART1_INDEX 107
#define PCC_LPUART2_INDEX 108
#define PCC_CMP0_INDEX 115


/*!
 * @}
 */ /* end of group PCC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- PDB Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PDB_Peripheral_Access_Layer PDB Peripheral Access Layer
 * @{
 */

/** PDB - Size of Registers Arrays */
#define PDB_DLY_COUNT                          8u
#define PDB_CH_COUNT                              2u
#define PDB_POnDLY_COUNT                       1u

/** PDB - Register Layout Typedef */
typedef struct {
  __IO uint32_t SC;                                /**< Status and Control register, offset: 0x0 */
  __IO uint32_t MOD;                               /**< Modulus register, offset: 0x4 */
  __I  uint32_t CNT;                               /**< Counter register, offset: 0x8 */
  __IO uint32_t IDLY;                              /**< Interrupt Delay register, offset: 0xC */
  struct {                                         /* offset: 0x10, array step: 0x28 */
    __IO uint32_t C1;                                /**< Channel n Control register 1, array offset: 0x10, array step: 0x28 */
    __IO uint32_t S;                                 /**< Channel n Status register, array offset: 0x14, array step: 0x28 */
    __IO uint32_t DLY[PDB_DLY_COUNT];             /**< Channel n Delay 0 register..Channel n Delay 7 register, array offset: 0x18, array step: index*0x28, index2*0x4 */
  } CH[PDB_CH_COUNT];
  uint8_t RESERVED_0[304];
  __IO uint32_t POEN;                              /**< Pulse-Out n Enable register, offset: 0x190 */
  union {                                          /* offset: 0x194 */
    struct {                                         /* offset: 0x194 */
      __IO uint16_t DLY2;                           /**< PDB_DLY2 register, offset: 0x194 */
      __IO uint16_t DLY1;                           /**< PDB_DLY1 register, offset: 0x196 */
    } ACCESS16BIT;
    __IO uint32_t PODLY;                            /**< Pulse-Out n Delay register, offset: 0x194 */
  } POnDLY[PDB_POnDLY_COUNT];
} PDB_Type, *PDB_MemMapPtr;

/** Number of instances of the PDB module. */
#define PDB_INSTANCE_COUNT                       (2u)

/* PDB - Peripheral instance base addresses */
/** Peripheral PDB0 base address */
#define PDB0_BASE                                (0x40036000u)
/** Peripheral PDB0 base pointer */
#define PDB0                                     ((PDB_Type *)PDB0_BASE)
/** Peripheral PDB1 base address */
#define PDB1_BASE                                (0x40031000u)
/** Peripheral PDB1 base pointer */
#define PDB1                                     ((PDB_Type *)PDB1_BASE)
/** Array initializer of PDB peripheral base addresses */
#define PDB_BASE_ADDRS                           { PDB0_BASE, PDB1_BASE }
/** Array initializer of PDB peripheral base pointers */
#define PDB_BASE_PTRS                            { PDB0, PDB1 }

/* ----------------------------------------------------------------------------
   -- PDB Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PDB_Register_Masks PDB Register Masks
 * @{
 */

/*! @name SC - Status and Control register */
/*! @{ */
#define PDB_SC_LDOK_MASK                         (0x1U)
#define PDB_SC_LDOK_SHIFT                        (0U)
#define PDB_SC_LDOK_WIDTH                        (1U)
#define PDB_SC_LDOK(x)                           (((uint32_t)(((uint32_t)(x)) << PDB_SC_LDOK_SHIFT)) & PDB_SC_LDOK_MASK)
#define PDB_SC_CONT_MASK                         (0x2U)
#define PDB_SC_CONT_SHIFT                        (1U)
#define PDB_SC_CONT_WIDTH                        (1U)
#define PDB_SC_CONT(x)                           (((uint32_t)(((uint32_t)(x)) << PDB_SC_CONT_SHIFT)) & PDB_SC_CONT_MASK)
#define PDB_SC_MULT_MASK                         (0xCU)
#define PDB_SC_MULT_SHIFT                        (2U)
#define PDB_SC_MULT_WIDTH                        (2U)
#define PDB_SC_MULT(x)                           (((uint32_t)(((uint32_t)(x)) << PDB_SC_MULT_SHIFT)) & PDB_SC_MULT_MASK)
#define PDB_SC_PDBIE_MASK                        (0x20U)
#define PDB_SC_PDBIE_SHIFT                       (5U)
#define PDB_SC_PDBIE_WIDTH                       (1U)
#define PDB_SC_PDBIE(x)                          (((uint32_t)(((uint32_t)(x)) << PDB_SC_PDBIE_SHIFT)) & PDB_SC_PDBIE_MASK)
#define PDB_SC_PDBIF_MASK                        (0x40U)
#define PDB_SC_PDBIF_SHIFT                       (6U)
#define PDB_SC_PDBIF_WIDTH                       (1U)
#define PDB_SC_PDBIF(x)                          (((uint32_t)(((uint32_t)(x)) << PDB_SC_PDBIF_SHIFT)) & PDB_SC_PDBIF_MASK)
#define PDB_SC_PDBEN_MASK                        (0x80U)
#define PDB_SC_PDBEN_SHIFT                       (7U)
#define PDB_SC_PDBEN_WIDTH                       (1U)
#define PDB_SC_PDBEN(x)                          (((uint32_t)(((uint32_t)(x)) << PDB_SC_PDBEN_SHIFT)) & PDB_SC_PDBEN_MASK)
#define PDB_SC_TRGSEL_MASK                       (0xF00U)
#define PDB_SC_TRGSEL_SHIFT                      (8U)
#define PDB_SC_TRGSEL_WIDTH                      (4U)
#define PDB_SC_TRGSEL(x)                         (((uint32_t)(((uint32_t)(x)) << PDB_SC_TRGSEL_SHIFT)) & PDB_SC_TRGSEL_MASK)
#define PDB_SC_PRESCALER_MASK                    (0x7000U)
#define PDB_SC_PRESCALER_SHIFT                   (12U)
#define PDB_SC_PRESCALER_WIDTH                   (3U)
#define PDB_SC_PRESCALER(x)                      (((uint32_t)(((uint32_t)(x)) << PDB_SC_PRESCALER_SHIFT)) & PDB_SC_PRESCALER_MASK)
#define PDB_SC_DMAEN_MASK                        (0x8000U)
#define PDB_SC_DMAEN_SHIFT                       (15U)
#define PDB_SC_DMAEN_WIDTH                       (1U)
#define PDB_SC_DMAEN(x)                          (((uint32_t)(((uint32_t)(x)) << PDB_SC_DMAEN_SHIFT)) & PDB_SC_DMAEN_MASK)
#define PDB_SC_SWTRIG_MASK                       (0x10000U)
#define PDB_SC_SWTRIG_SHIFT                      (16U)
#define PDB_SC_SWTRIG_WIDTH                      (1U)
#define PDB_SC_SWTRIG(x)                         (((uint32_t)(((uint32_t)(x)) << PDB_SC_SWTRIG_SHIFT)) & PDB_SC_SWTRIG_MASK)
#define PDB_SC_PDBEIE_MASK                       (0x20000U)
#define PDB_SC_PDBEIE_SHIFT                      (17U)
#define PDB_SC_PDBEIE_WIDTH                      (1U)
#define PDB_SC_PDBEIE(x)                         (((uint32_t)(((uint32_t)(x)) << PDB_SC_PDBEIE_SHIFT)) & PDB_SC_PDBEIE_MASK)
#define PDB_SC_LDMOD_MASK                        (0xC0000U)
#define PDB_SC_LDMOD_SHIFT                       (18U)
#define PDB_SC_LDMOD_WIDTH                       (2U)
#define PDB_SC_LDMOD(x)                          (((uint32_t)(((uint32_t)(x)) << PDB_SC_LDMOD_SHIFT)) & PDB_SC_LDMOD_MASK)
/*! @} */

/*! @name MOD - Modulus register */
/*! @{ */
#define PDB_MOD_MOD_MASK                         (0xFFFFU)
#define PDB_MOD_MOD_SHIFT                        (0U)
#define PDB_MOD_MOD_WIDTH                        (16U)
#define PDB_MOD_MOD(x)                           (((uint32_t)(((uint32_t)(x)) << PDB_MOD_MOD_SHIFT)) & PDB_MOD_MOD_MASK)
/*! @} */

/*! @name CNT - Counter register */
/*! @{ */
#define PDB_CNT_CNT_MASK                         (0xFFFFU)
#define PDB_CNT_CNT_SHIFT                        (0U)
#define PDB_CNT_CNT_WIDTH                        (16U)
#define PDB_CNT_CNT(x)                           (((uint32_t)(((uint32_t)(x)) << PDB_CNT_CNT_SHIFT)) & PDB_CNT_CNT_MASK)
/*! @} */

/*! @name IDLY - Interrupt Delay register */
/*! @{ */
#define PDB_IDLY_IDLY_MASK                       (0xFFFFU)
#define PDB_IDLY_IDLY_SHIFT                      (0U)
#define PDB_IDLY_IDLY_WIDTH                      (16U)
#define PDB_IDLY_IDLY(x)                         (((uint32_t)(((uint32_t)(x)) << PDB_IDLY_IDLY_SHIFT)) & PDB_IDLY_IDLY_MASK)
/*! @} */

/*! @name C1 - Channel n Control register 1 */
/*! @{ */
#define PDB_C1_EN_MASK                           (0xFFU)
#define PDB_C1_EN_SHIFT                          (0U)
#define PDB_C1_EN_WIDTH                          (8U)
#define PDB_C1_EN(x)                             (((uint32_t)(((uint32_t)(x)) << PDB_C1_EN_SHIFT)) & PDB_C1_EN_MASK)
#define PDB_C1_TOS_MASK                          (0xFF00U)
#define PDB_C1_TOS_SHIFT                         (8U)
#define PDB_C1_TOS_WIDTH                         (8U)
#define PDB_C1_TOS(x)                            (((uint32_t)(((uint32_t)(x)) << PDB_C1_TOS_SHIFT)) & PDB_C1_TOS_MASK)
#define PDB_C1_BB_MASK                           (0xFF0000U)
#define PDB_C1_BB_SHIFT                          (16U)
#define PDB_C1_BB_WIDTH                          (8U)
#define PDB_C1_BB(x)                             (((uint32_t)(((uint32_t)(x)) << PDB_C1_BB_SHIFT)) & PDB_C1_BB_MASK)
/*! @} */

/*! @name S - Channel n Status register */
/*! @{ */
#define PDB_S_ERR_MASK                           (0xFFU)
#define PDB_S_ERR_SHIFT                          (0U)
#define PDB_S_ERR_WIDTH                          (8U)
#define PDB_S_ERR(x)                             (((uint32_t)(((uint32_t)(x)) << PDB_S_ERR_SHIFT)) & PDB_S_ERR_MASK)
#define PDB_S_CF_MASK                            (0xFF0000U)
#define PDB_S_CF_SHIFT                           (16U)
#define PDB_S_CF_WIDTH                           (8U)
#define PDB_S_CF(x)                              (((uint32_t)(((uint32_t)(x)) << PDB_S_CF_SHIFT)) & PDB_S_CF_MASK)
/*! @} */

/*! @name DLY - Channel n Delay 0 register..Channel n Delay 7 register */
/*! @{ */
#define PDB_DLY_DLY_MASK                         (0xFFFFU)
#define PDB_DLY_DLY_SHIFT                        (0U)
#define PDB_DLY_DLY_WIDTH                        (16U)
#define PDB_DLY_DLY(x)                           (((uint32_t)(((uint32_t)(x)) << PDB_DLY_DLY_SHIFT)) & PDB_DLY_DLY_MASK)
/*! @} */

/*! @name POEN - Pulse-Out n Enable register */
/*! @{ */
#define PDB_POEN_POEN_MASK                       (0xFFU)
#define PDB_POEN_POEN_SHIFT                      (0U)
#define PDB_POEN_POEN_WIDTH                      (8U)
#define PDB_POEN_POEN(x)                         (((uint32_t)(((uint32_t)(x)) << PDB_POEN_POEN_SHIFT)) & PDB_POEN_POEN_MASK)
/*! @} */

/*! @name POnDLY_PODLY - Pulse-Out n Delay register */
/*! @{ */
#define PDB_POnDLY_PODLY_DLY2_MASK               (0xFFFFU)
#define PDB_POnDLY_PODLY_DLY2_SHIFT              (0U)
#define PDB_POnDLY_PODLY_DLY2_WIDTH              (16U)
#define PDB_POnDLY_PODLY_DLY2(x)                 (((uint32_t)(((uint32_t)(x))<<PDB_POnDLY_PODLY_DLY2_SHIFT))&PDB_POnDLY_PODLY_DLY2_MASK)
#define PDB_POnDLY_PODLY_DLY1_MASK               (0xFFFF0000U)
#define PDB_POnDLY_PODLY_DLY1_SHIFT              (16U)
#define PDB_POnDLY_PODLY_DLY1_WIDTH              (16U)
#define PDB_POnDLY_PODLY_DLY1(x)                 (((uint32_t)(((uint32_t)(x))<<PDB_POnDLY_PODLY_DLY1_SHIFT))&PDB_POnDLY_PODLY_DLY1_MASK)
/*! @} */

/*! @name POnDLY_ACCESS16BIT_DLY2 - PDB_DLY2 register */
/*! @{ */
#define PDB_POnDLY_ACCESS16BIT_DLY2_DLY2_MASK (0xFFFFU)
#define PDB_POnDLY_ACCESS16BIT_DLY2_DLY2_SHIFT (0U)
#define PDB_POnDLY_ACCESS16BIT_DLY2_DLY2_WIDTH (16U)
#define PDB_POnDLY_ACCESS16BIT_DLY2_DLY2(x)      (((uint16_t)(((uint16_t)(x))<<PDB_POnDLY_ACCESS16BIT_DLY2_DLY2_SHIFT))&PDB_POnDLY_ACCESS16BIT_DLY2_DLY2_MASK)
/*! @} */

/*! @name POnDLY_ACCESS16BIT_DLY1 - PDB_DLY1 register */
/*! @{ */
#define PDB_POnDLY_ACCESS16BIT_DLY1_DLY1_MASK (0xFFFFU)
#define PDB_POnDLY_ACCESS16BIT_DLY1_DLY1_SHIFT (0U)
#define PDB_POnDLY_ACCESS16BIT_DLY1_DLY1_WIDTH (16U)
#define PDB_POnDLY_ACCESS16BIT_DLY1_DLY1(x)      (((uint16_t)(((uint16_t)(x))<<PDB_POnDLY_ACCESS16BIT_DLY1_DLY1_SHIFT))&PDB_POnDLY_ACCESS16BIT_DLY1_DLY1_MASK)
/*! @} */



/*!
 * @}
 */ /* end of group PDB_Register_Masks */

/*!
 * @}
 */ /* end of group PDB_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- PMC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PMC_Peripheral_Access_Layer PMC Peripheral Access Layer
 * @{
 */

/** PMC - Register Layout Typedef */
typedef struct {
  uint8_t RESERVED_0[1];
  __IO uint8_t LVDSC2;                             /**< Low Voltage Detect Status and Control 2 Register, offset: 0x1 */
  __IO uint8_t REGSC;                              /**< Regulator Status and Control Register, offset: 0x2 */
  __IO uint8_t LVRFLG;                             /**< Low Voltage Reset Flags Register, offset: 0x3 */
  __IO uint8_t LPOTRIM;                            /**< Low Power Oscillator Trim Register, offset: 0x4 */
} PMC_Type, *PMC_MemMapPtr;

/** Number of instances of the PMC module. */
#define PMC_INSTANCE_COUNT                       (1u)

/* PMC - Peripheral instance base addresses */
/** Peripheral PMC base address */
#define PMC_BASE                                 (0x4007D000u)
/** Peripheral PMC base pointer */
#define PMC                                      ((PMC_Type *)PMC_BASE)
/** Array initializer of PMC peripheral base addresses */
#define PMC_BASE_ADDRS                           { PMC_BASE }
/** Array initializer of PMC peripheral base pointers */
#define PMC_BASE_PTRS                            { PMC }

/* ----------------------------------------------------------------------------
   -- PMC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PMC_Register_Masks PMC Register Masks
 * @{
 */

/*! @name LVDSC2 - Low Voltage Detect Status and Control 2 Register */
/*! @{ */
#define PMC_LVDSC2_LVWIE_MASK                    (0x20U)
#define PMC_LVDSC2_LVWIE_SHIFT                   (5U)
#define PMC_LVDSC2_LVWIE_WIDTH                   (1U)
#define PMC_LVDSC2_LVWIE(x)                      (((uint8_t)(((uint8_t)(x)) << PMC_LVDSC2_LVWIE_SHIFT)) & PMC_LVDSC2_LVWIE_MASK)
#define PMC_LVDSC2_LVWACK_MASK                   (0x40U)
#define PMC_LVDSC2_LVWACK_SHIFT                  (6U)
#define PMC_LVDSC2_LVWACK_WIDTH                  (1U)
#define PMC_LVDSC2_LVWACK(x)                     (((uint8_t)(((uint8_t)(x)) << PMC_LVDSC2_LVWACK_SHIFT)) & PMC_LVDSC2_LVWACK_MASK)
#define PMC_LVDSC2_LVWF_MASK                     (0x80U)
#define PMC_LVDSC2_LVWF_SHIFT                    (7U)
#define PMC_LVDSC2_LVWF_WIDTH                    (1U)
#define PMC_LVDSC2_LVWF(x)                       (((uint8_t)(((uint8_t)(x)) << PMC_LVDSC2_LVWF_SHIFT)) & PMC_LVDSC2_LVWF_MASK)
/*! @} */

/*! @name REGSC - Regulator Status and Control Register */
/*! @{ */
#define PMC_REGSC_BIASEN_MASK                    (0x1U)
#define PMC_REGSC_BIASEN_SHIFT                   (0U)
#define PMC_REGSC_BIASEN_WIDTH                   (1U)
#define PMC_REGSC_BIASEN(x)                      (((uint8_t)(((uint8_t)(x)) << PMC_REGSC_BIASEN_SHIFT)) & PMC_REGSC_BIASEN_MASK)
#define PMC_REGSC_CLKBIASDIS_MASK                (0x2U)
#define PMC_REGSC_CLKBIASDIS_SHIFT               (1U)
#define PMC_REGSC_CLKBIASDIS_WIDTH               (1U)
#define PMC_REGSC_CLKBIASDIS(x)                  (((uint8_t)(((uint8_t)(x)) << PMC_REGSC_CLKBIASDIS_SHIFT)) & PMC_REGSC_CLKBIASDIS_MASK)
#define PMC_REGSC_REGFPM_MASK                    (0x4U)
#define PMC_REGSC_REGFPM_SHIFT                   (2U)
#define PMC_REGSC_REGFPM_WIDTH                   (1U)
#define PMC_REGSC_REGFPM(x)                      (((uint8_t)(((uint8_t)(x)) << PMC_REGSC_REGFPM_SHIFT)) & PMC_REGSC_REGFPM_MASK)
#define PMC_REGSC_LPOSTAT_MASK                   (0x40U)
#define PMC_REGSC_LPOSTAT_SHIFT                  (6U)
#define PMC_REGSC_LPOSTAT_WIDTH                  (1U)
#define PMC_REGSC_LPOSTAT(x)                     (((uint8_t)(((uint8_t)(x)) << PMC_REGSC_LPOSTAT_SHIFT)) & PMC_REGSC_LPOSTAT_MASK)
#define PMC_REGSC_LPODIS_MASK                    (0x80U)
#define PMC_REGSC_LPODIS_SHIFT                   (7U)
#define PMC_REGSC_LPODIS_WIDTH                   (1U)
#define PMC_REGSC_LPODIS(x)                      (((uint8_t)(((uint8_t)(x)) << PMC_REGSC_LPODIS_SHIFT)) & PMC_REGSC_LPODIS_MASK)
/*! @} */

/*! @name LVRFLG - Low Voltage Reset Flags Register */
/*! @{ */
#define PMC_LVRFLG_LVRF_MASK                     (0x1U)
#define PMC_LVRFLG_LVRF_SHIFT                    (0U)
#define PMC_LVRFLG_LVRF_WIDTH                    (1U)
#define PMC_LVRFLG_LVRF(x)                       (((uint8_t)(((uint8_t)(x)) << PMC_LVRFLG_LVRF_SHIFT)) & PMC_LVRFLG_LVRF_MASK)
#define PMC_LVRFLG_LVRLPF_MASK                   (0x2U)
#define PMC_LVRFLG_LVRLPF_SHIFT                  (1U)
#define PMC_LVRFLG_LVRLPF_WIDTH                  (1U)
#define PMC_LVRFLG_LVRLPF(x)                     (((uint8_t)(((uint8_t)(x)) << PMC_LVRFLG_LVRLPF_SHIFT)) & PMC_LVRFLG_LVRLPF_MASK)
#define PMC_LVRFLG_LVRXF_MASK                    (0x4U)
#define PMC_LVRFLG_LVRXF_SHIFT                   (2U)
#define PMC_LVRFLG_LVRXF_WIDTH                   (1U)
#define PMC_LVRFLG_LVRXF(x)                      (((uint8_t)(((uint8_t)(x)) << PMC_LVRFLG_LVRXF_SHIFT)) & PMC_LVRFLG_LVRXF_MASK)
#define PMC_LVRFLG_LVRXLPF_MASK                  (0x8U)
#define PMC_LVRFLG_LVRXLPF_SHIFT                 (3U)
#define PMC_LVRFLG_LVRXLPF_WIDTH                 (1U)
#define PMC_LVRFLG_LVRXLPF(x)                    (((uint8_t)(((uint8_t)(x)) << PMC_LVRFLG_LVRXLPF_SHIFT)) & PMC_LVRFLG_LVRXLPF_MASK)
#define PMC_LVRFLG_LVR3F_MASK                    (0x10U)
#define PMC_LVRFLG_LVR3F_SHIFT                   (4U)
#define PMC_LVRFLG_LVR3F_WIDTH                   (1U)
#define PMC_LVRFLG_LVR3F(x)                      (((uint8_t)(((uint8_t)(x)) << PMC_LVRFLG_LVR3F_SHIFT)) & PMC_LVRFLG_LVR3F_MASK)
#define PMC_LVRFLG_LVR3FLSF_MASK                 (0x20U)
#define PMC_LVRFLG_LVR3FLSF_SHIFT                (5U)
#define PMC_LVRFLG_LVR3FLSF_WIDTH                (1U)
#define PMC_LVRFLG_LVR3FLSF(x)                   (((uint8_t)(((uint8_t)(x)) << PMC_LVRFLG_LVR3FLSF_SHIFT)) & PMC_LVRFLG_LVR3FLSF_MASK)
#define PMC_LVRFLG_PORF_MASK                     (0x80U)
#define PMC_LVRFLG_PORF_SHIFT                    (7U)
#define PMC_LVRFLG_PORF_WIDTH                    (1U)
#define PMC_LVRFLG_PORF(x)                       (((uint8_t)(((uint8_t)(x)) << PMC_LVRFLG_PORF_SHIFT)) & PMC_LVRFLG_PORF_MASK)
/*! @} */

/*! @name LPOTRIM - Low Power Oscillator Trim Register */
/*! @{ */
#define PMC_LPOTRIM_LPOTRIM_MASK                 (0x1FU)
#define PMC_LPOTRIM_LPOTRIM_SHIFT                (0U)
#define PMC_LPOTRIM_LPOTRIM_WIDTH                (5U)
#define PMC_LPOTRIM_LPOTRIM(x)                   (((uint8_t)(((uint8_t)(x)) << PMC_LPOTRIM_LPOTRIM_SHIFT)) & PMC_LPOTRIM_LPOTRIM_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group PMC_Register_Masks */

/*!
 * @}
 */ /* end of group PMC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- PORT Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PORT_Peripheral_Access_Layer PORT Peripheral Access Layer
 * @{
 */

/** PORT - Size of Registers Arrays */
#define PORT_PCR_COUNT                            32u

/** PORT - Register Layout Typedef */
typedef struct {
  __IO uint32_t PCR[PORT_PCR_COUNT];               /**< Pin Control Register n, array offset: 0x0, array step: 0x4 */
  __O  uint32_t GPCLR;                             /**< Global Pin Control Low Register, offset: 0x80 */
  __O  uint32_t GPCHR;                             /**< Global Pin Control High Register, offset: 0x84 */
  __O  uint32_t GICLR;                             /**< Global Interrupt Control Low Register, offset: 0x88 */
  __O  uint32_t GICHR;                             /**< Global Interrupt Control High Register, offset: 0x8C */
  uint8_t RESERVED_0[16];
  __IO uint32_t ISFR;                              /**< Interrupt Status Flag Register, offset: 0xA0 */
  uint8_t RESERVED_1[28];
  __IO uint32_t DFER;                              /**< Digital Filter Enable Register, offset: 0xC0 */
  __IO uint32_t DFCR;                              /**< Digital Filter Clock Register, offset: 0xC4 */
  __IO uint32_t DFWR;                              /**< Digital Filter Width Register, offset: 0xC8 */
} PORT_Type, *PORT_MemMapPtr;

/** Number of instances of the PORT module. */
#define PORT_INSTANCE_COUNT                      (5u)

/* PORT - Peripheral instance base addresses */
/** Peripheral PORTA base address */
#define PORTA_BASE                               (0x40049000u)
/** Peripheral PORTA base pointer */
#define PORTA                                    ((PORT_Type *)PORTA_BASE)
/** Peripheral PORTB base address */
#define PORTB_BASE                               (0x4004A000u)
/** Peripheral PORTB base pointer */
#define PORTB                                    ((PORT_Type *)PORTB_BASE)
/** Peripheral PORTC base address */
#define PORTC_BASE                               (0x4004B000u)
/** Peripheral PORTC base pointer */
#define PORTC                                    ((PORT_Type *)PORTC_BASE)
/** Peripheral PORTD base address */
#define PORTD_BASE                               (0x4004C000u)
/** Peripheral PORTD base pointer */
#define PORTD                                    ((PORT_Type *)PORTD_BASE)
/** Peripheral PORTE base address */
#define PORTE_BASE                               (0x4004D000u)
/** Peripheral PORTE base pointer */
#define PORTE                                    ((PORT_Type *)PORTE_BASE)
/** Array initializer of PORT peripheral base addresses */
#define PORT_BASE_ADDRS                          { PORTA_BASE, PORTB_BASE, PORTC_BASE, PORTD_BASE, PORTE_BASE }
/** Array initializer of PORT peripheral base pointers */
#define PORT_BASE_PTRS                           { PORTA, PORTB, PORTC, PORTD, PORTE }

/* ----------------------------------------------------------------------------
   -- PORT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PORT_Register_Masks PORT Register Masks
 * @{
 */

/*! @name PCR - Pin Control Register n */
/*! @{ */
#define PORT_PCR_PS_MASK                         (0x1U)
#define PORT_PCR_PS_SHIFT                        (0U)
#define PORT_PCR_PS_WIDTH                        (1U)
#define PORT_PCR_PS(x)                           (((uint32_t)(((uint32_t)(x)) << PORT_PCR_PS_SHIFT)) & PORT_PCR_PS_MASK)
#define PORT_PCR_PE_MASK                         (0x2U)
#define PORT_PCR_PE_SHIFT                        (1U)
#define PORT_PCR_PE_WIDTH                        (1U)
#define PORT_PCR_PE(x)                           (((uint32_t)(((uint32_t)(x)) << PORT_PCR_PE_SHIFT)) & PORT_PCR_PE_MASK)
#define PORT_PCR_PFE_MASK                        (0x10U)
#define PORT_PCR_PFE_SHIFT                       (4U)
#define PORT_PCR_PFE_WIDTH                       (1U)
#define PORT_PCR_PFE(x)                          (((uint32_t)(((uint32_t)(x)) << PORT_PCR_PFE_SHIFT)) & PORT_PCR_PFE_MASK)
#define PORT_PCR_DSE_MASK                        (0x40U)
#define PORT_PCR_DSE_SHIFT                       (6U)
#define PORT_PCR_DSE_WIDTH                       (1U)
#define PORT_PCR_DSE(x)                          (((uint32_t)(((uint32_t)(x)) << PORT_PCR_DSE_SHIFT)) & PORT_PCR_DSE_MASK)
#define PORT_PCR_MUX_MASK                        (0x700U)
#define PORT_PCR_MUX_SHIFT                       (8U)
#define PORT_PCR_MUX_WIDTH                       (3U)
#define PORT_PCR_MUX(x)                          (((uint32_t)(((uint32_t)(x)) << PORT_PCR_MUX_SHIFT)) & PORT_PCR_MUX_MASK)
#define PORT_PCR_LK_MASK                         (0x8000U)
#define PORT_PCR_LK_SHIFT                        (15U)
#define PORT_PCR_LK_WIDTH                        (1U)
#define PORT_PCR_LK(x)                           (((uint32_t)(((uint32_t)(x)) << PORT_PCR_LK_SHIFT)) & PORT_PCR_LK_MASK)
#define PORT_PCR_IRQC_MASK                       (0xF0000U)
#define PORT_PCR_IRQC_SHIFT                      (16U)
#define PORT_PCR_IRQC_WIDTH                      (4U)
#define PORT_PCR_IRQC(x)                         (((uint32_t)(((uint32_t)(x)) << PORT_PCR_IRQC_SHIFT)) & PORT_PCR_IRQC_MASK)
#define PORT_PCR_ISF_MASK                        (0x1000000U)
#define PORT_PCR_ISF_SHIFT                       (24U)
#define PORT_PCR_ISF_WIDTH                       (1U)
#define PORT_PCR_ISF(x)                          (((uint32_t)(((uint32_t)(x)) << PORT_PCR_ISF_SHIFT)) & PORT_PCR_ISF_MASK)
/*! @} */

/*! @name GPCLR - Global Pin Control Low Register */
/*! @{ */
#define PORT_GPCLR_GPWD_MASK                     (0xFFFFU)
#define PORT_GPCLR_GPWD_SHIFT                    (0U)
#define PORT_GPCLR_GPWD_WIDTH                    (16U)
#define PORT_GPCLR_GPWD(x)                       (((uint32_t)(((uint32_t)(x)) << PORT_GPCLR_GPWD_SHIFT)) & PORT_GPCLR_GPWD_MASK)
#define PORT_GPCLR_GPWE_MASK                     (0xFFFF0000U)
#define PORT_GPCLR_GPWE_SHIFT                    (16U)
#define PORT_GPCLR_GPWE_WIDTH                    (16U)
#define PORT_GPCLR_GPWE(x)                       (((uint32_t)(((uint32_t)(x)) << PORT_GPCLR_GPWE_SHIFT)) & PORT_GPCLR_GPWE_MASK)
/*! @} */

/*! @name GPCHR - Global Pin Control High Register */
/*! @{ */
#define PORT_GPCHR_GPWD_MASK                     (0xFFFFU)
#define PORT_GPCHR_GPWD_SHIFT                    (0U)
#define PORT_GPCHR_GPWD_WIDTH                    (16U)
#define PORT_GPCHR_GPWD(x)                       (((uint32_t)(((uint32_t)(x)) << PORT_GPCHR_GPWD_SHIFT)) & PORT_GPCHR_GPWD_MASK)
#define PORT_GPCHR_GPWE_MASK                     (0xFFFF0000U)
#define PORT_GPCHR_GPWE_SHIFT                    (16U)
#define PORT_GPCHR_GPWE_WIDTH                    (16U)
#define PORT_GPCHR_GPWE(x)                       (((uint32_t)(((uint32_t)(x)) << PORT_GPCHR_GPWE_SHIFT)) & PORT_GPCHR_GPWE_MASK)
/*! @} */

/*! @name GICLR - Global Interrupt Control Low Register */
/*! @{ */
#define PORT_GICLR_GIWE_MASK                     (0xFFFFU)
#define PORT_GICLR_GIWE_SHIFT                    (0U)
#define PORT_GICLR_GIWE_WIDTH                    (16U)
#define PORT_GICLR_GIWE(x)                       (((uint32_t)(((uint32_t)(x)) << PORT_GICLR_GIWE_SHIFT)) & PORT_GICLR_GIWE_MASK)
#define PORT_GICLR_GIWD_MASK                     (0xFFFF0000U)
#define PORT_GICLR_GIWD_SHIFT                    (16U)
#define PORT_GICLR_GIWD_WIDTH                    (16U)
#define PORT_GICLR_GIWD(x)                       (((uint32_t)(((uint32_t)(x)) << PORT_GICLR_GIWD_SHIFT)) & PORT_GICLR_GIWD_MASK)
/*! @} */

/*! @name GICHR - Global Interrupt Control High Register */
/*! @{ */
#define PORT_GICHR_GIWE_MASK                     (0xFFFFU)
#define PORT_GICHR_GIWE_SHIFT                    (0U)
#define PORT_GICHR_GIWE_WIDTH                    (16U)
#define PORT_GICHR_GIWE(x)                       (((uint32_t)(((uint32_t)(x)) << PORT_GICHR_GIWE_SHIFT)) & PORT_GICHR_GIWE_MASK)
#define PORT_GICHR_GIWD_MASK                     (0xFFFF0000U)
#define PORT_GICHR_GIWD_SHIFT                    (16U)
#define PORT_GICHR_GIWD_WIDTH                    (16U)
#define PORT_GICHR_GIWD(x)                       (((uint32_t)(((uint32_t)(x)) << PORT_GICHR_GIWD_SHIFT)) & PORT_GICHR_GIWD_MASK)
/*! @} */

/*! @name ISFR - Interrupt Status Flag Register */
/*! @{ */
#define PORT_ISFR_ISF_MASK                       (0xFFFFFFFFU)
#define PORT_ISFR_ISF_SHIFT                      (0U)
#define PORT_ISFR_ISF_WIDTH                      (32U)
#define PORT_ISFR_ISF(x)                         (((uint32_t)(((uint32_t)(x)) << PORT_ISFR_ISF_SHIFT)) & PORT_ISFR_ISF_MASK)
/*! @} */

/*! @name DFER - Digital Filter Enable Register */
/*! @{ */
#define PORT_DFER_DFE_MASK                       (0xFFFFFFFFU)
#define PORT_DFER_DFE_SHIFT                      (0U)
#define PORT_DFER_DFE_WIDTH                      (32U)
#define PORT_DFER_DFE(x)                         (((uint32_t)(((uint32_t)(x)) << PORT_DFER_DFE_SHIFT)) & PORT_DFER_DFE_MASK)
/*! @} */

/*! @name DFCR - Digital Filter Clock Register */
/*! @{ */
#define PORT_DFCR_CS_MASK                        (0x1U)
#define PORT_DFCR_CS_SHIFT                       (0U)
#define PORT_DFCR_CS_WIDTH                       (1U)
#define PORT_DFCR_CS(x)                          (((uint32_t)(((uint32_t)(x)) << PORT_DFCR_CS_SHIFT)) & PORT_DFCR_CS_MASK)
/*! @} */

/*! @name DFWR - Digital Filter Width Register */
/*! @{ */
#define PORT_DFWR_FILT_MASK                      (0x1FU)
#define PORT_DFWR_FILT_SHIFT                     (0U)
#define PORT_DFWR_FILT_WIDTH                     (5U)
#define PORT_DFWR_FILT(x)                        (((uint32_t)(((uint32_t)(x)) << PORT_DFWR_FILT_SHIFT)) & PORT_DFWR_FILT_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group PORT_Register_Masks */

/*!
 * @}
 */ /* end of group PORT_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- RCM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RCM_Peripheral_Access_Layer RCM Peripheral Access Layer
 * @{
 */

/** RCM - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID Register, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< Parameter Register, offset: 0x4 */
  __I  uint32_t SRS;                               /**< System Reset Status Register, offset: 0x8 */
  __IO uint32_t RPC;                               /**< Reset Pin Control register, offset: 0xC */
  uint8_t RESERVED_0[8];
  __IO uint32_t SSRS;                              /**< Sticky System Reset Status Register, offset: 0x18 */
  __IO uint32_t SRIE;                              /**< System Reset Interrupt Enable Register, offset: 0x1C */
} RCM_Type, *RCM_MemMapPtr;

/** Number of instances of the RCM module. */
#define RCM_INSTANCE_COUNT                       (1u)

/* RCM - Peripheral instance base addresses */
/** Peripheral RCM base address */
#define RCM_BASE                                 (0x4007F000u)
/** Peripheral RCM base pointer */
#define RCM                                      ((RCM_Type *)RCM_BASE)
/** Array initializer of RCM peripheral base addresses */
#define RCM_BASE_ADDRS                           { RCM_BASE }
/** Array initializer of RCM peripheral base pointers */
#define RCM_BASE_PTRS                            { RCM }

/* ----------------------------------------------------------------------------
   -- RCM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RCM_Register_Masks RCM Register Masks
 * @{
 */

/*! @name VERID - Version ID Register */
/*! @{ */
#define RCM_VERID_FEATURE_MASK                   (0xFFFFU)
#define RCM_VERID_FEATURE_SHIFT                  (0U)
#define RCM_VERID_FEATURE_WIDTH                  (16U)
#define RCM_VERID_FEATURE(x)                     (((uint32_t)(((uint32_t)(x)) << RCM_VERID_FEATURE_SHIFT)) & RCM_VERID_FEATURE_MASK)
#define RCM_VERID_MINOR_MASK                     (0xFF0000U)
#define RCM_VERID_MINOR_SHIFT                    (16U)
#define RCM_VERID_MINOR_WIDTH                    (8U)
#define RCM_VERID_MINOR(x)                       (((uint32_t)(((uint32_t)(x)) << RCM_VERID_MINOR_SHIFT)) & RCM_VERID_MINOR_MASK)
#define RCM_VERID_MAJOR_MASK                     (0xFF000000U)
#define RCM_VERID_MAJOR_SHIFT                    (24U)
#define RCM_VERID_MAJOR_WIDTH                    (8U)
#define RCM_VERID_MAJOR(x)                       (((uint32_t)(((uint32_t)(x)) << RCM_VERID_MAJOR_SHIFT)) & RCM_VERID_MAJOR_MASK)
/*! @} */

/*! @name PARAM - Parameter Register */
/*! @{ */
#define RCM_PARAM_EWAKEUP_MASK                   (0x1U)
#define RCM_PARAM_EWAKEUP_SHIFT                  (0U)
#define RCM_PARAM_EWAKEUP_WIDTH                  (1U)
#define RCM_PARAM_EWAKEUP(x)                     (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_EWAKEUP_SHIFT)) & RCM_PARAM_EWAKEUP_MASK)
#define RCM_PARAM_ELVD_MASK                      (0x2U)
#define RCM_PARAM_ELVD_SHIFT                     (1U)
#define RCM_PARAM_ELVD_WIDTH                     (1U)
#define RCM_PARAM_ELVD(x)                        (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_ELVD_SHIFT)) & RCM_PARAM_ELVD_MASK)
#define RCM_PARAM_ELOC_MASK                      (0x4U)
#define RCM_PARAM_ELOC_SHIFT                     (2U)
#define RCM_PARAM_ELOC_WIDTH                     (1U)
#define RCM_PARAM_ELOC(x)                        (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_ELOC_SHIFT)) & RCM_PARAM_ELOC_MASK)
#define RCM_PARAM_ELOL_MASK                      (0x8U)
#define RCM_PARAM_ELOL_SHIFT                     (3U)
#define RCM_PARAM_ELOL_WIDTH                     (1U)
#define RCM_PARAM_ELOL(x)                        (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_ELOL_SHIFT)) & RCM_PARAM_ELOL_MASK)
#define RCM_PARAM_ECMU_LOC_MASK                  (0x10U)
#define RCM_PARAM_ECMU_LOC_SHIFT                 (4U)
#define RCM_PARAM_ECMU_LOC_WIDTH                 (1U)
#define RCM_PARAM_ECMU_LOC(x)                    (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_ECMU_LOC_SHIFT)) & RCM_PARAM_ECMU_LOC_MASK)
#define RCM_PARAM_EWDOG_MASK                     (0x20U)
#define RCM_PARAM_EWDOG_SHIFT                    (5U)
#define RCM_PARAM_EWDOG_WIDTH                    (1U)
#define RCM_PARAM_EWDOG(x)                       (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_EWDOG_SHIFT)) & RCM_PARAM_EWDOG_MASK)
#define RCM_PARAM_EPIN_MASK                      (0x40U)
#define RCM_PARAM_EPIN_SHIFT                     (6U)
#define RCM_PARAM_EPIN_WIDTH                     (1U)
#define RCM_PARAM_EPIN(x)                        (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_EPIN_SHIFT)) & RCM_PARAM_EPIN_MASK)
#define RCM_PARAM_EPOR_MASK                      (0x80U)
#define RCM_PARAM_EPOR_SHIFT                     (7U)
#define RCM_PARAM_EPOR_WIDTH                     (1U)
#define RCM_PARAM_EPOR(x)                        (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_EPOR_SHIFT)) & RCM_PARAM_EPOR_MASK)
#define RCM_PARAM_EJTAG_MASK                     (0x100U)
#define RCM_PARAM_EJTAG_SHIFT                    (8U)
#define RCM_PARAM_EJTAG_WIDTH                    (1U)
#define RCM_PARAM_EJTAG(x)                       (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_EJTAG_SHIFT)) & RCM_PARAM_EJTAG_MASK)
#define RCM_PARAM_ELOCKUP_MASK                   (0x200U)
#define RCM_PARAM_ELOCKUP_SHIFT                  (9U)
#define RCM_PARAM_ELOCKUP_WIDTH                  (1U)
#define RCM_PARAM_ELOCKUP(x)                     (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_ELOCKUP_SHIFT)) & RCM_PARAM_ELOCKUP_MASK)
#define RCM_PARAM_ESW_MASK                       (0x400U)
#define RCM_PARAM_ESW_SHIFT                      (10U)
#define RCM_PARAM_ESW_WIDTH                      (1U)
#define RCM_PARAM_ESW(x)                         (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_ESW_SHIFT)) & RCM_PARAM_ESW_MASK)
#define RCM_PARAM_EMDM_AP_MASK                   (0x800U)
#define RCM_PARAM_EMDM_AP_SHIFT                  (11U)
#define RCM_PARAM_EMDM_AP_WIDTH                  (1U)
#define RCM_PARAM_EMDM_AP(x)                     (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_EMDM_AP_SHIFT)) & RCM_PARAM_EMDM_AP_MASK)
#define RCM_PARAM_ESACKERR_MASK                  (0x2000U)
#define RCM_PARAM_ESACKERR_SHIFT                 (13U)
#define RCM_PARAM_ESACKERR_WIDTH                 (1U)
#define RCM_PARAM_ESACKERR(x)                    (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_ESACKERR_SHIFT)) & RCM_PARAM_ESACKERR_MASK)
#define RCM_PARAM_ETAMPER_MASK                   (0x8000U)
#define RCM_PARAM_ETAMPER_SHIFT                  (15U)
#define RCM_PARAM_ETAMPER_WIDTH                  (1U)
#define RCM_PARAM_ETAMPER(x)                     (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_ETAMPER_SHIFT)) & RCM_PARAM_ETAMPER_MASK)
#define RCM_PARAM_ECORE1_MASK                    (0x10000U)
#define RCM_PARAM_ECORE1_SHIFT                   (16U)
#define RCM_PARAM_ECORE1_WIDTH                   (1U)
#define RCM_PARAM_ECORE1(x)                      (((uint32_t)(((uint32_t)(x)) << RCM_PARAM_ECORE1_SHIFT)) & RCM_PARAM_ECORE1_MASK)
/*! @} */

/*! @name SRS - System Reset Status Register */
/*! @{ */
#define RCM_SRS_LVD_MASK                         (0x2U)
#define RCM_SRS_LVD_SHIFT                        (1U)
#define RCM_SRS_LVD_WIDTH                        (1U)
#define RCM_SRS_LVD(x)                           (((uint32_t)(((uint32_t)(x)) << RCM_SRS_LVD_SHIFT)) & RCM_SRS_LVD_MASK)
#define RCM_SRS_LOC_MASK                         (0x4U)
#define RCM_SRS_LOC_SHIFT                        (2U)
#define RCM_SRS_LOC_WIDTH                        (1U)
#define RCM_SRS_LOC(x)                           (((uint32_t)(((uint32_t)(x)) << RCM_SRS_LOC_SHIFT)) & RCM_SRS_LOC_MASK)
#define RCM_SRS_LOL_MASK                         (0x8U)
#define RCM_SRS_LOL_SHIFT                        (3U)
#define RCM_SRS_LOL_WIDTH                        (1U)
#define RCM_SRS_LOL(x)                           (((uint32_t)(((uint32_t)(x)) << RCM_SRS_LOL_SHIFT)) & RCM_SRS_LOL_MASK)
#define RCM_SRS_WDOG_MASK                        (0x20U)
#define RCM_SRS_WDOG_SHIFT                       (5U)
#define RCM_SRS_WDOG_WIDTH                       (1U)
#define RCM_SRS_WDOG(x)                          (((uint32_t)(((uint32_t)(x)) << RCM_SRS_WDOG_SHIFT)) & RCM_SRS_WDOG_MASK)
#define RCM_SRS_PIN_MASK                         (0x40U)
#define RCM_SRS_PIN_SHIFT                        (6U)
#define RCM_SRS_PIN_WIDTH                        (1U)
#define RCM_SRS_PIN(x)                           (((uint32_t)(((uint32_t)(x)) << RCM_SRS_PIN_SHIFT)) & RCM_SRS_PIN_MASK)
#define RCM_SRS_POR_MASK                         (0x80U)
#define RCM_SRS_POR_SHIFT                        (7U)
#define RCM_SRS_POR_WIDTH                        (1U)
#define RCM_SRS_POR(x)                           (((uint32_t)(((uint32_t)(x)) << RCM_SRS_POR_SHIFT)) & RCM_SRS_POR_MASK)
#define RCM_SRS_JTAG_MASK                        (0x100U)
#define RCM_SRS_JTAG_SHIFT                       (8U)
#define RCM_SRS_JTAG_WIDTH                       (1U)
#define RCM_SRS_JTAG(x)                          (((uint32_t)(((uint32_t)(x)) << RCM_SRS_JTAG_SHIFT)) & RCM_SRS_JTAG_MASK)
#define RCM_SRS_LOCKUP_MASK                      (0x200U)
#define RCM_SRS_LOCKUP_SHIFT                     (9U)
#define RCM_SRS_LOCKUP_WIDTH                     (1U)
#define RCM_SRS_LOCKUP(x)                        (((uint32_t)(((uint32_t)(x)) << RCM_SRS_LOCKUP_SHIFT)) & RCM_SRS_LOCKUP_MASK)
#define RCM_SRS_SW_MASK                          (0x400U)
#define RCM_SRS_SW_SHIFT                         (10U)
#define RCM_SRS_SW_WIDTH                         (1U)
#define RCM_SRS_SW(x)                            (((uint32_t)(((uint32_t)(x)) << RCM_SRS_SW_SHIFT)) & RCM_SRS_SW_MASK)
#define RCM_SRS_MDM_AP_MASK                      (0x800U)
#define RCM_SRS_MDM_AP_SHIFT                     (11U)
#define RCM_SRS_MDM_AP_WIDTH                     (1U)
#define RCM_SRS_MDM_AP(x)                        (((uint32_t)(((uint32_t)(x)) << RCM_SRS_MDM_AP_SHIFT)) & RCM_SRS_MDM_AP_MASK)
#define RCM_SRS_SACKERR_MASK                     (0x2000U)
#define RCM_SRS_SACKERR_SHIFT                    (13U)
#define RCM_SRS_SACKERR_WIDTH                    (1U)
#define RCM_SRS_SACKERR(x)                       (((uint32_t)(((uint32_t)(x)) << RCM_SRS_SACKERR_SHIFT)) & RCM_SRS_SACKERR_MASK)
/*! @} */

/*! @name RPC - Reset Pin Control register */
/*! @{ */
#define RCM_RPC_RSTFLTSRW_MASK                   (0x3U)
#define RCM_RPC_RSTFLTSRW_SHIFT                  (0U)
#define RCM_RPC_RSTFLTSRW_WIDTH                  (2U)
#define RCM_RPC_RSTFLTSRW(x)                     (((uint32_t)(((uint32_t)(x)) << RCM_RPC_RSTFLTSRW_SHIFT)) & RCM_RPC_RSTFLTSRW_MASK)
#define RCM_RPC_RSTFLTSS_MASK                    (0x4U)
#define RCM_RPC_RSTFLTSS_SHIFT                   (2U)
#define RCM_RPC_RSTFLTSS_WIDTH                   (1U)
#define RCM_RPC_RSTFLTSS(x)                      (((uint32_t)(((uint32_t)(x)) << RCM_RPC_RSTFLTSS_SHIFT)) & RCM_RPC_RSTFLTSS_MASK)
#define RCM_RPC_RSTFLTSEL_MASK                   (0x1F00U)
#define RCM_RPC_RSTFLTSEL_SHIFT                  (8U)
#define RCM_RPC_RSTFLTSEL_WIDTH                  (5U)
#define RCM_RPC_RSTFLTSEL(x)                     (((uint32_t)(((uint32_t)(x)) << RCM_RPC_RSTFLTSEL_SHIFT)) & RCM_RPC_RSTFLTSEL_MASK)
/*! @} */

/*! @name SSRS - Sticky System Reset Status Register */
/*! @{ */
#define RCM_SSRS_SLVD_MASK                       (0x2U)
#define RCM_SSRS_SLVD_SHIFT                      (1U)
#define RCM_SSRS_SLVD_WIDTH                      (1U)
#define RCM_SSRS_SLVD(x)                         (((uint32_t)(((uint32_t)(x)) << RCM_SSRS_SLVD_SHIFT)) & RCM_SSRS_SLVD_MASK)
#define RCM_SSRS_SLOC_MASK                       (0x4U)
#define RCM_SSRS_SLOC_SHIFT                      (2U)
#define RCM_SSRS_SLOC_WIDTH                      (1U)
#define RCM_SSRS_SLOC(x)                         (((uint32_t)(((uint32_t)(x)) << RCM_SSRS_SLOC_SHIFT)) & RCM_SSRS_SLOC_MASK)
#define RCM_SSRS_SLOL_MASK                       (0x8U)
#define RCM_SSRS_SLOL_SHIFT                      (3U)
#define RCM_SSRS_SLOL_WIDTH                      (1U)
#define RCM_SSRS_SLOL(x)                         (((uint32_t)(((uint32_t)(x)) << RCM_SSRS_SLOL_SHIFT)) & RCM_SSRS_SLOL_MASK)
#define RCM_SSRS_SWDOG_MASK                      (0x20U)
#define RCM_SSRS_SWDOG_SHIFT                     (5U)
#define RCM_SSRS_SWDOG_WIDTH                     (1U)
#define RCM_SSRS_SWDOG(x)                        (((uint32_t)(((uint32_t)(x)) << RCM_SSRS_SWDOG_SHIFT)) & RCM_SSRS_SWDOG_MASK)
#define RCM_SSRS_SPIN_MASK                       (0x40U)
#define RCM_SSRS_SPIN_SHIFT                      (6U)
#define RCM_SSRS_SPIN_WIDTH                      (1U)
#define RCM_SSRS_SPIN(x)                         (((uint32_t)(((uint32_t)(x)) << RCM_SSRS_SPIN_SHIFT)) & RCM_SSRS_SPIN_MASK)
#define RCM_SSRS_SPOR_MASK                       (0x80U)
#define RCM_SSRS_SPOR_SHIFT                      (7U)
#define RCM_SSRS_SPOR_WIDTH                      (1U)
#define RCM_SSRS_SPOR(x)                         (((uint32_t)(((uint32_t)(x)) << RCM_SSRS_SPOR_SHIFT)) & RCM_SSRS_SPOR_MASK)
#define RCM_SSRS_SJTAG_MASK                      (0x100U)
#define RCM_SSRS_SJTAG_SHIFT                     (8U)
#define RCM_SSRS_SJTAG_WIDTH                     (1U)
#define RCM_SSRS_SJTAG(x)                        (((uint32_t)(((uint32_t)(x)) << RCM_SSRS_SJTAG_SHIFT)) & RCM_SSRS_SJTAG_MASK)
#define RCM_SSRS_SLOCKUP_MASK                    (0x200U)
#define RCM_SSRS_SLOCKUP_SHIFT                   (9U)
#define RCM_SSRS_SLOCKUP_WIDTH                   (1U)
#define RCM_SSRS_SLOCKUP(x)                      (((uint32_t)(((uint32_t)(x)) << RCM_SSRS_SLOCKUP_SHIFT)) & RCM_SSRS_SLOCKUP_MASK)
#define RCM_SSRS_SSW_MASK                        (0x400U)
#define RCM_SSRS_SSW_SHIFT                       (10U)
#define RCM_SSRS_SSW_WIDTH                       (1U)
#define RCM_SSRS_SSW(x)                          (((uint32_t)(((uint32_t)(x)) << RCM_SSRS_SSW_SHIFT)) & RCM_SSRS_SSW_MASK)
#define RCM_SSRS_SMDM_AP_MASK                    (0x800U)
#define RCM_SSRS_SMDM_AP_SHIFT                   (11U)
#define RCM_SSRS_SMDM_AP_WIDTH                   (1U)
#define RCM_SSRS_SMDM_AP(x)                      (((uint32_t)(((uint32_t)(x)) << RCM_SSRS_SMDM_AP_SHIFT)) & RCM_SSRS_SMDM_AP_MASK)
#define RCM_SSRS_SSACKERR_MASK                   (0x2000U)
#define RCM_SSRS_SSACKERR_SHIFT                  (13U)
#define RCM_SSRS_SSACKERR_WIDTH                  (1U)
#define RCM_SSRS_SSACKERR(x)                     (((uint32_t)(((uint32_t)(x)) << RCM_SSRS_SSACKERR_SHIFT)) & RCM_SSRS_SSACKERR_MASK)
/*! @} */

/*! @name SRIE - System Reset Interrupt Enable Register */
/*! @{ */
#define RCM_SRIE_DELAY_MASK                      (0x3U)
#define RCM_SRIE_DELAY_SHIFT                     (0U)
#define RCM_SRIE_DELAY_WIDTH                     (2U)
#define RCM_SRIE_DELAY(x)                        (((uint32_t)(((uint32_t)(x)) << RCM_SRIE_DELAY_SHIFT)) & RCM_SRIE_DELAY_MASK)
#define RCM_SRIE_LOC_MASK                        (0x4U)
#define RCM_SRIE_LOC_SHIFT                       (2U)
#define RCM_SRIE_LOC_WIDTH                       (1U)
#define RCM_SRIE_LOC(x)                          (((uint32_t)(((uint32_t)(x)) << RCM_SRIE_LOC_SHIFT)) & RCM_SRIE_LOC_MASK)
#define RCM_SRIE_LOL_MASK                        (0x8U)
#define RCM_SRIE_LOL_SHIFT                       (3U)
#define RCM_SRIE_LOL_WIDTH                       (1U)
#define RCM_SRIE_LOL(x)                          (((uint32_t)(((uint32_t)(x)) << RCM_SRIE_LOL_SHIFT)) & RCM_SRIE_LOL_MASK)
#define RCM_SRIE_WDOG_MASK                       (0x20U)
#define RCM_SRIE_WDOG_SHIFT                      (5U)
#define RCM_SRIE_WDOG_WIDTH                      (1U)
#define RCM_SRIE_WDOG(x)                         (((uint32_t)(((uint32_t)(x)) << RCM_SRIE_WDOG_SHIFT)) & RCM_SRIE_WDOG_MASK)
#define RCM_SRIE_PIN_MASK                        (0x40U)
#define RCM_SRIE_PIN_SHIFT                       (6U)
#define RCM_SRIE_PIN_WIDTH                       (1U)
#define RCM_SRIE_PIN(x)                          (((uint32_t)(((uint32_t)(x)) << RCM_SRIE_PIN_SHIFT)) & RCM_SRIE_PIN_MASK)
#define RCM_SRIE_GIE_MASK                        (0x80U)
#define RCM_SRIE_GIE_SHIFT                       (7U)
#define RCM_SRIE_GIE_WIDTH                       (1U)
#define RCM_SRIE_GIE(x)                          (((uint32_t)(((uint32_t)(x)) << RCM_SRIE_GIE_SHIFT)) & RCM_SRIE_GIE_MASK)
#define RCM_SRIE_JTAG_MASK                       (0x100U)
#define RCM_SRIE_JTAG_SHIFT                      (8U)
#define RCM_SRIE_JTAG_WIDTH                      (1U)
#define RCM_SRIE_JTAG(x)                         (((uint32_t)(((uint32_t)(x)) << RCM_SRIE_JTAG_SHIFT)) & RCM_SRIE_JTAG_MASK)
#define RCM_SRIE_LOCKUP_MASK                     (0x200U)
#define RCM_SRIE_LOCKUP_SHIFT                    (9U)
#define RCM_SRIE_LOCKUP_WIDTH                    (1U)
#define RCM_SRIE_LOCKUP(x)                       (((uint32_t)(((uint32_t)(x)) << RCM_SRIE_LOCKUP_SHIFT)) & RCM_SRIE_LOCKUP_MASK)
#define RCM_SRIE_SW_MASK                         (0x400U)
#define RCM_SRIE_SW_SHIFT                        (10U)
#define RCM_SRIE_SW_WIDTH                        (1U)
#define RCM_SRIE_SW(x)                           (((uint32_t)(((uint32_t)(x)) << RCM_SRIE_SW_SHIFT)) & RCM_SRIE_SW_MASK)
#define RCM_SRIE_MDM_AP_MASK                     (0x800U)
#define RCM_SRIE_MDM_AP_SHIFT                    (11U)
#define RCM_SRIE_MDM_AP_WIDTH                    (1U)
#define RCM_SRIE_MDM_AP(x)                       (((uint32_t)(((uint32_t)(x)) << RCM_SRIE_MDM_AP_SHIFT)) & RCM_SRIE_MDM_AP_MASK)
#define RCM_SRIE_SACKERR_MASK                    (0x2000U)
#define RCM_SRIE_SACKERR_SHIFT                   (13U)
#define RCM_SRIE_SACKERR_WIDTH                   (1U)
#define RCM_SRIE_SACKERR(x)                      (((uint32_t)(((uint32_t)(x)) << RCM_SRIE_SACKERR_SHIFT)) & RCM_SRIE_SACKERR_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group RCM_Register_Masks */

/*!
 * @}
 */ /* end of group RCM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- RTC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RTC_Peripheral_Access_Layer RTC Peripheral Access Layer
 * @{
 */

/** RTC - Register Layout Typedef */
typedef struct {
  __IO uint32_t TSR;                               /**< RTC Time Seconds Register, offset: 0x0 */
  __IO uint32_t TPR;                               /**< RTC Time Prescaler Register, offset: 0x4 */
  __IO uint32_t TAR;                               /**< RTC Time Alarm Register, offset: 0x8 */
  __IO uint32_t TCR;                               /**< RTC Time Compensation Register, offset: 0xC */
  __IO uint32_t CR;                                /**< RTC Control Register, offset: 0x10 */
  __IO uint32_t SR;                                /**< RTC Status Register, offset: 0x14 */
  __IO uint32_t LR;                                /**< RTC Lock Register, offset: 0x18 */
  __IO uint32_t IER;                               /**< RTC Interrupt Enable Register, offset: 0x1C */
} RTC_Type, *RTC_MemMapPtr;

/** Number of instances of the RTC module. */
#define RTC_INSTANCE_COUNT                       (1u)

/* RTC - Peripheral instance base addresses */
/** Peripheral RTC base address */
#define RTC_BASE                                 (0x4003D000u)
/** Peripheral RTC base pointer */
#define RTC                                      ((RTC_Type *)RTC_BASE)
/** Array initializer of RTC peripheral base addresses */
#define RTC_BASE_ADDRS                           { RTC_BASE }
/** Array initializer of RTC peripheral base pointers */
#define RTC_BASE_PTRS                            { RTC }

/* ----------------------------------------------------------------------------
   -- RTC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RTC_Register_Masks RTC Register Masks
 * @{
 */

/*! @name TSR - RTC Time Seconds Register */
/*! @{ */
#define RTC_TSR_TSR_MASK                         (0xFFFFFFFFU)
#define RTC_TSR_TSR_SHIFT                        (0U)
#define RTC_TSR_TSR_WIDTH                        (32U)
#define RTC_TSR_TSR(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_TSR_TSR_SHIFT)) & RTC_TSR_TSR_MASK)
/*! @} */

/*! @name TPR - RTC Time Prescaler Register */
/*! @{ */
#define RTC_TPR_TPR_MASK                         (0xFFFFU)
#define RTC_TPR_TPR_SHIFT                        (0U)
#define RTC_TPR_TPR_WIDTH                        (16U)
#define RTC_TPR_TPR(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_TPR_TPR_SHIFT)) & RTC_TPR_TPR_MASK)
/*! @} */

/*! @name TAR - RTC Time Alarm Register */
/*! @{ */
#define RTC_TAR_TAR_MASK                         (0xFFFFFFFFU)
#define RTC_TAR_TAR_SHIFT                        (0U)
#define RTC_TAR_TAR_WIDTH                        (32U)
#define RTC_TAR_TAR(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_TAR_TAR_SHIFT)) & RTC_TAR_TAR_MASK)
/*! @} */

/*! @name TCR - RTC Time Compensation Register */
/*! @{ */
#define RTC_TCR_TCR_MASK                         (0xFFU)
#define RTC_TCR_TCR_SHIFT                        (0U)
#define RTC_TCR_TCR_WIDTH                        (8U)
#define RTC_TCR_TCR(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_TCR_TCR_SHIFT)) & RTC_TCR_TCR_MASK)
#define RTC_TCR_CIR_MASK                         (0xFF00U)
#define RTC_TCR_CIR_SHIFT                        (8U)
#define RTC_TCR_CIR_WIDTH                        (8U)
#define RTC_TCR_CIR(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_TCR_CIR_SHIFT)) & RTC_TCR_CIR_MASK)
#define RTC_TCR_TCV_MASK                         (0xFF0000U)
#define RTC_TCR_TCV_SHIFT                        (16U)
#define RTC_TCR_TCV_WIDTH                        (8U)
#define RTC_TCR_TCV(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_TCR_TCV_SHIFT)) & RTC_TCR_TCV_MASK)
#define RTC_TCR_CIC_MASK                         (0xFF000000U)
#define RTC_TCR_CIC_SHIFT                        (24U)
#define RTC_TCR_CIC_WIDTH                        (8U)
#define RTC_TCR_CIC(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_TCR_CIC_SHIFT)) & RTC_TCR_CIC_MASK)
/*! @} */

/*! @name CR - RTC Control Register */
/*! @{ */
#define RTC_CR_SWR_MASK                          (0x1U)
#define RTC_CR_SWR_SHIFT                         (0U)
#define RTC_CR_SWR_WIDTH                         (1U)
#define RTC_CR_SWR(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_CR_SWR_SHIFT)) & RTC_CR_SWR_MASK)
#define RTC_CR_SUP_MASK                          (0x4U)
#define RTC_CR_SUP_SHIFT                         (2U)
#define RTC_CR_SUP_WIDTH                         (1U)
#define RTC_CR_SUP(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_CR_SUP_SHIFT)) & RTC_CR_SUP_MASK)
#define RTC_CR_UM_MASK                           (0x8U)
#define RTC_CR_UM_SHIFT                          (3U)
#define RTC_CR_UM_WIDTH                          (1U)
#define RTC_CR_UM(x)                             (((uint32_t)(((uint32_t)(x)) << RTC_CR_UM_SHIFT)) & RTC_CR_UM_MASK)
#define RTC_CR_CPS_MASK                          (0x20U)
#define RTC_CR_CPS_SHIFT                         (5U)
#define RTC_CR_CPS_WIDTH                         (1U)
#define RTC_CR_CPS(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_CR_CPS_SHIFT)) & RTC_CR_CPS_MASK)
#define RTC_CR_LPOS_MASK                         (0x80U)
#define RTC_CR_LPOS_SHIFT                        (7U)
#define RTC_CR_LPOS_WIDTH                        (1U)
#define RTC_CR_LPOS(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_CR_LPOS_SHIFT)) & RTC_CR_LPOS_MASK)
#define RTC_CR_CLKO_MASK                         (0x200U)
#define RTC_CR_CLKO_SHIFT                        (9U)
#define RTC_CR_CLKO_WIDTH                        (1U)
#define RTC_CR_CLKO(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_CR_CLKO_SHIFT)) & RTC_CR_CLKO_MASK)
#define RTC_CR_CPE_MASK                          (0x1000000U)
#define RTC_CR_CPE_SHIFT                         (24U)
#define RTC_CR_CPE_WIDTH                         (1U)
#define RTC_CR_CPE(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_CR_CPE_SHIFT)) & RTC_CR_CPE_MASK)
/*! @} */

/*! @name SR - RTC Status Register */
/*! @{ */
#define RTC_SR_TIF_MASK                          (0x1U)
#define RTC_SR_TIF_SHIFT                         (0U)
#define RTC_SR_TIF_WIDTH                         (1U)
#define RTC_SR_TIF(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_SR_TIF_SHIFT)) & RTC_SR_TIF_MASK)
#define RTC_SR_TOF_MASK                          (0x2U)
#define RTC_SR_TOF_SHIFT                         (1U)
#define RTC_SR_TOF_WIDTH                         (1U)
#define RTC_SR_TOF(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_SR_TOF_SHIFT)) & RTC_SR_TOF_MASK)
#define RTC_SR_TAF_MASK                          (0x4U)
#define RTC_SR_TAF_SHIFT                         (2U)
#define RTC_SR_TAF_WIDTH                         (1U)
#define RTC_SR_TAF(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_SR_TAF_SHIFT)) & RTC_SR_TAF_MASK)
#define RTC_SR_TCE_MASK                          (0x10U)
#define RTC_SR_TCE_SHIFT                         (4U)
#define RTC_SR_TCE_WIDTH                         (1U)
#define RTC_SR_TCE(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_SR_TCE_SHIFT)) & RTC_SR_TCE_MASK)
/*! @} */

/*! @name LR - RTC Lock Register */
/*! @{ */
#define RTC_LR_TCL_MASK                          (0x8U)
#define RTC_LR_TCL_SHIFT                         (3U)
#define RTC_LR_TCL_WIDTH                         (1U)
#define RTC_LR_TCL(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_LR_TCL_SHIFT)) & RTC_LR_TCL_MASK)
#define RTC_LR_CRL_MASK                          (0x10U)
#define RTC_LR_CRL_SHIFT                         (4U)
#define RTC_LR_CRL_WIDTH                         (1U)
#define RTC_LR_CRL(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_LR_CRL_SHIFT)) & RTC_LR_CRL_MASK)
#define RTC_LR_SRL_MASK                          (0x20U)
#define RTC_LR_SRL_SHIFT                         (5U)
#define RTC_LR_SRL_WIDTH                         (1U)
#define RTC_LR_SRL(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_LR_SRL_SHIFT)) & RTC_LR_SRL_MASK)
#define RTC_LR_LRL_MASK                          (0x40U)
#define RTC_LR_LRL_SHIFT                         (6U)
#define RTC_LR_LRL_WIDTH                         (1U)
#define RTC_LR_LRL(x)                            (((uint32_t)(((uint32_t)(x)) << RTC_LR_LRL_SHIFT)) & RTC_LR_LRL_MASK)
/*! @} */

/*! @name IER - RTC Interrupt Enable Register */
/*! @{ */
#define RTC_IER_TIIE_MASK                        (0x1U)
#define RTC_IER_TIIE_SHIFT                       (0U)
#define RTC_IER_TIIE_WIDTH                       (1U)
#define RTC_IER_TIIE(x)                          (((uint32_t)(((uint32_t)(x)) << RTC_IER_TIIE_SHIFT)) & RTC_IER_TIIE_MASK)
#define RTC_IER_TOIE_MASK                        (0x2U)
#define RTC_IER_TOIE_SHIFT                       (1U)
#define RTC_IER_TOIE_WIDTH                       (1U)
#define RTC_IER_TOIE(x)                          (((uint32_t)(((uint32_t)(x)) << RTC_IER_TOIE_SHIFT)) & RTC_IER_TOIE_MASK)
#define RTC_IER_TAIE_MASK                        (0x4U)
#define RTC_IER_TAIE_SHIFT                       (2U)
#define RTC_IER_TAIE_WIDTH                       (1U)
#define RTC_IER_TAIE(x)                          (((uint32_t)(((uint32_t)(x)) << RTC_IER_TAIE_SHIFT)) & RTC_IER_TAIE_MASK)
#define RTC_IER_TSIE_MASK                        (0x10U)
#define RTC_IER_TSIE_SHIFT                       (4U)
#define RTC_IER_TSIE_WIDTH                       (1U)
#define RTC_IER_TSIE(x)                          (((uint32_t)(((uint32_t)(x)) << RTC_IER_TSIE_SHIFT)) & RTC_IER_TSIE_MASK)
#define RTC_IER_TSIC_MASK                        (0x70000U)
#define RTC_IER_TSIC_SHIFT                       (16U)
#define RTC_IER_TSIC_WIDTH                       (3U)
#define RTC_IER_TSIC(x)                          (((uint32_t)(((uint32_t)(x)) << RTC_IER_TSIC_SHIFT)) & RTC_IER_TSIC_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group RTC_Register_Masks */

/*!
 * @}
 */ /* end of group RTC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- SCG Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SCG_Peripheral_Access_Layer SCG Peripheral Access Layer
 * @{
 */

/** SCG - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< Version ID Register, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< Parameter Register, offset: 0x4 */
  uint8_t RESERVED_0[8];
  __I  uint32_t CSR;                               /**< Clock Status Register, offset: 0x10 */
  __IO uint32_t RCCR;                              /**< Run Clock Control Register, offset: 0x14 */
  __IO uint32_t VCCR;                              /**< VLPR Clock Control Register, offset: 0x18 */
  uint8_t RESERVED_1[4];
  __IO uint32_t CLKOUTCNFG;                        /**< SCG CLKOUT Configuration Register, offset: 0x20 */
  uint8_t RESERVED_2[220];
  __IO uint32_t SOSCCSR;                           /**< System OSC Control Status Register, offset: 0x100 */
  __IO uint32_t SOSCDIV;                           /**< System OSC Divide Register, offset: 0x104 */
  __IO uint32_t SOSCCFG;                           /**< System Oscillator Configuration Register, offset: 0x108 */
  uint8_t RESERVED_3[244];
  __IO uint32_t SIRCCSR;                           /**< Slow IRC Control Status Register, offset: 0x200 */
  __IO uint32_t SIRCDIV;                           /**< Slow IRC Divide Register, offset: 0x204 */
  __IO uint32_t SIRCCFG;                           /**< Slow IRC Configuration Register, offset: 0x208 */
  uint8_t RESERVED_4[244];
  __IO uint32_t FIRCCSR;                           /**< Fast IRC Control Status Register, offset: 0x300 */
  __IO uint32_t FIRCDIV;                           /**< Fast IRC Divide Register, offset: 0x304 */
  __IO uint32_t FIRCCFG;                           /**< Fast IRC Configuration Register, offset: 0x308 */
  uint8_t RESERVED_5[756];
  __IO uint32_t SPLLCSR;                           /**< System PLL Control Status Register, offset: 0x600 */
  __IO uint32_t SPLLDIV;                           /**< System PLL Divide Register, offset: 0x604 */
  __IO uint32_t SPLLCFG;                           /**< System PLL Configuration Register, offset: 0x608 */
} SCG_Type, *SCG_MemMapPtr;

/** Number of instances of the SCG module. */
#define SCG_INSTANCE_COUNT                       (1u)

/* SCG - Peripheral instance base addresses */
/** Peripheral SCG base address */
#define SCG_BASE                                 (0x40064000u)
/** Peripheral SCG base pointer */
#define SCG                                      ((SCG_Type *)SCG_BASE)
/** Array initializer of SCG peripheral base addresses */
#define SCG_BASE_ADDRS                           { SCG_BASE }
/** Array initializer of SCG peripheral base pointers */
#define SCG_BASE_PTRS                            { SCG }

/* ----------------------------------------------------------------------------
   -- SCG Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SCG_Register_Masks SCG Register Masks
 * @{
 */

/*! @name VERID - Version ID Register */
/*! @{ */
#define SCG_VERID_VERSION_MASK                   (0xFFFFFFFFU)
#define SCG_VERID_VERSION_SHIFT                  (0U)
#define SCG_VERID_VERSION_WIDTH                  (32U)
#define SCG_VERID_VERSION(x)                     (((uint32_t)(((uint32_t)(x)) << SCG_VERID_VERSION_SHIFT)) & SCG_VERID_VERSION_MASK)
/*! @} */

/*! @name PARAM - Parameter Register */
/*! @{ */
#define SCG_PARAM_CLKPRES_MASK                   (0xFFU)
#define SCG_PARAM_CLKPRES_SHIFT                  (0U)
#define SCG_PARAM_CLKPRES_WIDTH                  (8U)
#define SCG_PARAM_CLKPRES(x)                     (((uint32_t)(((uint32_t)(x)) << SCG_PARAM_CLKPRES_SHIFT)) & SCG_PARAM_CLKPRES_MASK)
#define SCG_PARAM_DIVPRES_MASK                   (0xF8000000U)
#define SCG_PARAM_DIVPRES_SHIFT                  (27U)
#define SCG_PARAM_DIVPRES_WIDTH                  (5U)
#define SCG_PARAM_DIVPRES(x)                     (((uint32_t)(((uint32_t)(x)) << SCG_PARAM_DIVPRES_SHIFT)) & SCG_PARAM_DIVPRES_MASK)
/*! @} */

/*! @name CSR - Clock Status Register */
/*! @{ */
#define SCG_CSR_DIVSLOW_MASK                     (0xFU)
#define SCG_CSR_DIVSLOW_SHIFT                    (0U)
#define SCG_CSR_DIVSLOW_WIDTH                    (4U)
#define SCG_CSR_DIVSLOW(x)                       (((uint32_t)(((uint32_t)(x)) << SCG_CSR_DIVSLOW_SHIFT)) & SCG_CSR_DIVSLOW_MASK)
#define SCG_CSR_DIVBUS_MASK                      (0xF0U)
#define SCG_CSR_DIVBUS_SHIFT                     (4U)
#define SCG_CSR_DIVBUS_WIDTH                     (4U)
#define SCG_CSR_DIVBUS(x)                        (((uint32_t)(((uint32_t)(x)) << SCG_CSR_DIVBUS_SHIFT)) & SCG_CSR_DIVBUS_MASK)
#define SCG_CSR_DIVCORE_MASK                     (0xF0000U)
#define SCG_CSR_DIVCORE_SHIFT                    (16U)
#define SCG_CSR_DIVCORE_WIDTH                    (4U)
#define SCG_CSR_DIVCORE(x)                       (((uint32_t)(((uint32_t)(x)) << SCG_CSR_DIVCORE_SHIFT)) & SCG_CSR_DIVCORE_MASK)
#define SCG_CSR_SCS_MASK                         (0xF000000U)
#define SCG_CSR_SCS_SHIFT                        (24U)
#define SCG_CSR_SCS_WIDTH                        (4U)
#define SCG_CSR_SCS(x)                           (((uint32_t)(((uint32_t)(x)) << SCG_CSR_SCS_SHIFT)) & SCG_CSR_SCS_MASK)
/*! @} */

/*! @name RCCR - Run Clock Control Register */
/*! @{ */
#define SCG_RCCR_DIVSLOW_MASK                    (0xFU)
#define SCG_RCCR_DIVSLOW_SHIFT                   (0U)
#define SCG_RCCR_DIVSLOW_WIDTH                   (4U)
#define SCG_RCCR_DIVSLOW(x)                      (((uint32_t)(((uint32_t)(x)) << SCG_RCCR_DIVSLOW_SHIFT)) & SCG_RCCR_DIVSLOW_MASK)
#define SCG_RCCR_DIVBUS_MASK                     (0xF0U)
#define SCG_RCCR_DIVBUS_SHIFT                    (4U)
#define SCG_RCCR_DIVBUS_WIDTH                    (4U)
#define SCG_RCCR_DIVBUS(x)                       (((uint32_t)(((uint32_t)(x)) << SCG_RCCR_DIVBUS_SHIFT)) & SCG_RCCR_DIVBUS_MASK)
#define SCG_RCCR_DIVCORE_MASK                    (0xF0000U)
#define SCG_RCCR_DIVCORE_SHIFT                   (16U)
#define SCG_RCCR_DIVCORE_WIDTH                   (4U)
#define SCG_RCCR_DIVCORE(x)                      (((uint32_t)(((uint32_t)(x)) << SCG_RCCR_DIVCORE_SHIFT)) & SCG_RCCR_DIVCORE_MASK)
#define SCG_RCCR_SCS_MASK                        (0xF000000U)
#define SCG_RCCR_SCS_SHIFT                       (24U)
#define SCG_RCCR_SCS_WIDTH                       (4U)
#define SCG_RCCR_SCS(x)                          (((uint32_t)(((uint32_t)(x)) << SCG_RCCR_SCS_SHIFT)) & SCG_RCCR_SCS_MASK)
/*! @} */

/*! @name VCCR - VLPR Clock Control Register */
/*! @{ */
#define SCG_VCCR_DIVSLOW_MASK                    (0xFU)
#define SCG_VCCR_DIVSLOW_SHIFT                   (0U)
#define SCG_VCCR_DIVSLOW_WIDTH                   (4U)
#define SCG_VCCR_DIVSLOW(x)                      (((uint32_t)(((uint32_t)(x)) << SCG_VCCR_DIVSLOW_SHIFT)) & SCG_VCCR_DIVSLOW_MASK)
#define SCG_VCCR_DIVBUS_MASK                     (0xF0U)
#define SCG_VCCR_DIVBUS_SHIFT                    (4U)
#define SCG_VCCR_DIVBUS_WIDTH                    (4U)
#define SCG_VCCR_DIVBUS(x)                       (((uint32_t)(((uint32_t)(x)) << SCG_VCCR_DIVBUS_SHIFT)) & SCG_VCCR_DIVBUS_MASK)
#define SCG_VCCR_DIVCORE_MASK                    (0xF0000U)
#define SCG_VCCR_DIVCORE_SHIFT                   (16U)
#define SCG_VCCR_DIVCORE_WIDTH                   (4U)
#define SCG_VCCR_DIVCORE(x)                      (((uint32_t)(((uint32_t)(x)) << SCG_VCCR_DIVCORE_SHIFT)) & SCG_VCCR_DIVCORE_MASK)
#define SCG_VCCR_SCS_MASK                        (0xF000000U)
#define SCG_VCCR_SCS_SHIFT                       (24U)
#define SCG_VCCR_SCS_WIDTH                       (4U)
#define SCG_VCCR_SCS(x)                          (((uint32_t)(((uint32_t)(x)) << SCG_VCCR_SCS_SHIFT)) & SCG_VCCR_SCS_MASK)
/*! @} */

/*! @name CLKOUTCNFG - SCG CLKOUT Configuration Register */
/*! @{ */
#define SCG_CLKOUTCNFG_CLKOUTSEL_MASK            (0xF000000U)
#define SCG_CLKOUTCNFG_CLKOUTSEL_SHIFT           (24U)
#define SCG_CLKOUTCNFG_CLKOUTSEL_WIDTH           (4U)
#define SCG_CLKOUTCNFG_CLKOUTSEL(x)              (((uint32_t)(((uint32_t)(x)) << SCG_CLKOUTCNFG_CLKOUTSEL_SHIFT)) & SCG_CLKOUTCNFG_CLKOUTSEL_MASK)
/*! @} */

/*! @name SOSCCSR - System OSC Control Status Register */
/*! @{ */
#define SCG_SOSCCSR_SOSCEN_MASK                  (0x1U)
#define SCG_SOSCCSR_SOSCEN_SHIFT                 (0U)
#define SCG_SOSCCSR_SOSCEN_WIDTH                 (1U)
#define SCG_SOSCCSR_SOSCEN(x)                    (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_SOSCEN_SHIFT)) & SCG_SOSCCSR_SOSCEN_MASK)
#define SCG_SOSCCSR_SOSCCM_MASK                  (0x10000U)
#define SCG_SOSCCSR_SOSCCM_SHIFT                 (16U)
#define SCG_SOSCCSR_SOSCCM_WIDTH                 (1U)
#define SCG_SOSCCSR_SOSCCM(x)                    (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_SOSCCM_SHIFT)) & SCG_SOSCCSR_SOSCCM_MASK)
#define SCG_SOSCCSR_SOSCCMRE_MASK                (0x20000U)
#define SCG_SOSCCSR_SOSCCMRE_SHIFT               (17U)
#define SCG_SOSCCSR_SOSCCMRE_WIDTH               (1U)
#define SCG_SOSCCSR_SOSCCMRE(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_SOSCCMRE_SHIFT)) & SCG_SOSCCSR_SOSCCMRE_MASK)
#define SCG_SOSCCSR_LK_MASK                      (0x800000U)
#define SCG_SOSCCSR_LK_SHIFT                     (23U)
#define SCG_SOSCCSR_LK_WIDTH                     (1U)
#define SCG_SOSCCSR_LK(x)                        (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_LK_SHIFT)) & SCG_SOSCCSR_LK_MASK)
#define SCG_SOSCCSR_SOSCVLD_MASK                 (0x1000000U)
#define SCG_SOSCCSR_SOSCVLD_SHIFT                (24U)
#define SCG_SOSCCSR_SOSCVLD_WIDTH                (1U)
#define SCG_SOSCCSR_SOSCVLD(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_SOSCVLD_SHIFT)) & SCG_SOSCCSR_SOSCVLD_MASK)
#define SCG_SOSCCSR_SOSCSEL_MASK                 (0x2000000U)
#define SCG_SOSCCSR_SOSCSEL_SHIFT                (25U)
#define SCG_SOSCCSR_SOSCSEL_WIDTH                (1U)
#define SCG_SOSCCSR_SOSCSEL(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_SOSCSEL_SHIFT)) & SCG_SOSCCSR_SOSCSEL_MASK)
#define SCG_SOSCCSR_SOSCERR_MASK                 (0x4000000U)
#define SCG_SOSCCSR_SOSCERR_SHIFT                (26U)
#define SCG_SOSCCSR_SOSCERR_WIDTH                (1U)
#define SCG_SOSCCSR_SOSCERR(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCSR_SOSCERR_SHIFT)) & SCG_SOSCCSR_SOSCERR_MASK)
/*! @} */

/*! @name SOSCDIV - System OSC Divide Register */
/*! @{ */
#define SCG_SOSCDIV_SOSCDIV1_MASK                (0x7U)
#define SCG_SOSCDIV_SOSCDIV1_SHIFT               (0U)
#define SCG_SOSCDIV_SOSCDIV1_WIDTH               (3U)
#define SCG_SOSCDIV_SOSCDIV1(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SOSCDIV_SOSCDIV1_SHIFT)) & SCG_SOSCDIV_SOSCDIV1_MASK)
#define SCG_SOSCDIV_SOSCDIV2_MASK                (0x700U)
#define SCG_SOSCDIV_SOSCDIV2_SHIFT               (8U)
#define SCG_SOSCDIV_SOSCDIV2_WIDTH               (3U)
#define SCG_SOSCDIV_SOSCDIV2(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SOSCDIV_SOSCDIV2_SHIFT)) & SCG_SOSCDIV_SOSCDIV2_MASK)
/*! @} */

/*! @name SOSCCFG - System Oscillator Configuration Register */
/*! @{ */
#define SCG_SOSCCFG_EREFS_MASK                   (0x4U)
#define SCG_SOSCCFG_EREFS_SHIFT                  (2U)
#define SCG_SOSCCFG_EREFS_WIDTH                  (1U)
#define SCG_SOSCCFG_EREFS(x)                     (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCFG_EREFS_SHIFT)) & SCG_SOSCCFG_EREFS_MASK)
#define SCG_SOSCCFG_HGO_MASK                     (0x8U)
#define SCG_SOSCCFG_HGO_SHIFT                    (3U)
#define SCG_SOSCCFG_HGO_WIDTH                    (1U)
#define SCG_SOSCCFG_HGO(x)                       (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCFG_HGO_SHIFT)) & SCG_SOSCCFG_HGO_MASK)
#define SCG_SOSCCFG_RANGE_MASK                   (0x30U)
#define SCG_SOSCCFG_RANGE_SHIFT                  (4U)
#define SCG_SOSCCFG_RANGE_WIDTH                  (2U)
#define SCG_SOSCCFG_RANGE(x)                     (((uint32_t)(((uint32_t)(x)) << SCG_SOSCCFG_RANGE_SHIFT)) & SCG_SOSCCFG_RANGE_MASK)
/*! @} */

/*! @name SIRCCSR - Slow IRC Control Status Register */
/*! @{ */
#define SCG_SIRCCSR_SIRCEN_MASK                  (0x1U)
#define SCG_SIRCCSR_SIRCEN_SHIFT                 (0U)
#define SCG_SIRCCSR_SIRCEN_WIDTH                 (1U)
#define SCG_SIRCCSR_SIRCEN(x)                    (((uint32_t)(((uint32_t)(x)) << SCG_SIRCCSR_SIRCEN_SHIFT)) & SCG_SIRCCSR_SIRCEN_MASK)
#define SCG_SIRCCSR_SIRCSTEN_MASK                (0x2U)
#define SCG_SIRCCSR_SIRCSTEN_SHIFT               (1U)
#define SCG_SIRCCSR_SIRCSTEN_WIDTH               (1U)
#define SCG_SIRCCSR_SIRCSTEN(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SIRCCSR_SIRCSTEN_SHIFT)) & SCG_SIRCCSR_SIRCSTEN_MASK)
#define SCG_SIRCCSR_SIRCLPEN_MASK                (0x4U)
#define SCG_SIRCCSR_SIRCLPEN_SHIFT               (2U)
#define SCG_SIRCCSR_SIRCLPEN_WIDTH               (1U)
#define SCG_SIRCCSR_SIRCLPEN(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SIRCCSR_SIRCLPEN_SHIFT)) & SCG_SIRCCSR_SIRCLPEN_MASK)
#define SCG_SIRCCSR_LK_MASK                      (0x800000U)
#define SCG_SIRCCSR_LK_SHIFT                     (23U)
#define SCG_SIRCCSR_LK_WIDTH                     (1U)
#define SCG_SIRCCSR_LK(x)                        (((uint32_t)(((uint32_t)(x)) << SCG_SIRCCSR_LK_SHIFT)) & SCG_SIRCCSR_LK_MASK)
#define SCG_SIRCCSR_SIRCVLD_MASK                 (0x1000000U)
#define SCG_SIRCCSR_SIRCVLD_SHIFT                (24U)
#define SCG_SIRCCSR_SIRCVLD_WIDTH                (1U)
#define SCG_SIRCCSR_SIRCVLD(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_SIRCCSR_SIRCVLD_SHIFT)) & SCG_SIRCCSR_SIRCVLD_MASK)
#define SCG_SIRCCSR_SIRCSEL_MASK                 (0x2000000U)
#define SCG_SIRCCSR_SIRCSEL_SHIFT                (25U)
#define SCG_SIRCCSR_SIRCSEL_WIDTH                (1U)
#define SCG_SIRCCSR_SIRCSEL(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_SIRCCSR_SIRCSEL_SHIFT)) & SCG_SIRCCSR_SIRCSEL_MASK)
/*! @} */

/*! @name SIRCDIV - Slow IRC Divide Register */
/*! @{ */
#define SCG_SIRCDIV_SIRCDIV1_MASK                (0x7U)
#define SCG_SIRCDIV_SIRCDIV1_SHIFT               (0U)
#define SCG_SIRCDIV_SIRCDIV1_WIDTH               (3U)
#define SCG_SIRCDIV_SIRCDIV1(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SIRCDIV_SIRCDIV1_SHIFT)) & SCG_SIRCDIV_SIRCDIV1_MASK)
#define SCG_SIRCDIV_SIRCDIV2_MASK                (0x700U)
#define SCG_SIRCDIV_SIRCDIV2_SHIFT               (8U)
#define SCG_SIRCDIV_SIRCDIV2_WIDTH               (3U)
#define SCG_SIRCDIV_SIRCDIV2(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SIRCDIV_SIRCDIV2_SHIFT)) & SCG_SIRCDIV_SIRCDIV2_MASK)
/*! @} */

/*! @name SIRCCFG - Slow IRC Configuration Register */
/*! @{ */
#define SCG_SIRCCFG_RANGE_MASK                   (0x1U)
#define SCG_SIRCCFG_RANGE_SHIFT                  (0U)
#define SCG_SIRCCFG_RANGE_WIDTH                  (1U)
#define SCG_SIRCCFG_RANGE(x)                     (((uint32_t)(((uint32_t)(x)) << SCG_SIRCCFG_RANGE_SHIFT)) & SCG_SIRCCFG_RANGE_MASK)
/*! @} */

/*! @name FIRCCSR - Fast IRC Control Status Register */
/*! @{ */
#define SCG_FIRCCSR_FIRCEN_MASK                  (0x1U)
#define SCG_FIRCCSR_FIRCEN_SHIFT                 (0U)
#define SCG_FIRCCSR_FIRCEN_WIDTH                 (1U)
#define SCG_FIRCCSR_FIRCEN(x)                    (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRCEN_SHIFT)) & SCG_FIRCCSR_FIRCEN_MASK)
#define SCG_FIRCCSR_FIRCREGOFF_MASK              (0x8U)
#define SCG_FIRCCSR_FIRCREGOFF_SHIFT             (3U)
#define SCG_FIRCCSR_FIRCREGOFF_WIDTH             (1U)
#define SCG_FIRCCSR_FIRCREGOFF(x)                (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRCREGOFF_SHIFT)) & SCG_FIRCCSR_FIRCREGOFF_MASK)
#define SCG_FIRCCSR_LK_MASK                      (0x800000U)
#define SCG_FIRCCSR_LK_SHIFT                     (23U)
#define SCG_FIRCCSR_LK_WIDTH                     (1U)
#define SCG_FIRCCSR_LK(x)                        (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_LK_SHIFT)) & SCG_FIRCCSR_LK_MASK)
#define SCG_FIRCCSR_FIRCVLD_MASK                 (0x1000000U)
#define SCG_FIRCCSR_FIRCVLD_SHIFT                (24U)
#define SCG_FIRCCSR_FIRCVLD_WIDTH                (1U)
#define SCG_FIRCCSR_FIRCVLD(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRCVLD_SHIFT)) & SCG_FIRCCSR_FIRCVLD_MASK)
#define SCG_FIRCCSR_FIRCSEL_MASK                 (0x2000000U)
#define SCG_FIRCCSR_FIRCSEL_SHIFT                (25U)
#define SCG_FIRCCSR_FIRCSEL_WIDTH                (1U)
#define SCG_FIRCCSR_FIRCSEL(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRCSEL_SHIFT)) & SCG_FIRCCSR_FIRCSEL_MASK)
#define SCG_FIRCCSR_FIRCERR_MASK                 (0x4000000U)
#define SCG_FIRCCSR_FIRCERR_SHIFT                (26U)
#define SCG_FIRCCSR_FIRCERR_WIDTH                (1U)
#define SCG_FIRCCSR_FIRCERR(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCSR_FIRCERR_SHIFT)) & SCG_FIRCCSR_FIRCERR_MASK)
/*! @} */

/*! @name FIRCDIV - Fast IRC Divide Register */
/*! @{ */
#define SCG_FIRCDIV_FIRCDIV1_MASK                (0x7U)
#define SCG_FIRCDIV_FIRCDIV1_SHIFT               (0U)
#define SCG_FIRCDIV_FIRCDIV1_WIDTH               (3U)
#define SCG_FIRCDIV_FIRCDIV1(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_FIRCDIV_FIRCDIV1_SHIFT)) & SCG_FIRCDIV_FIRCDIV1_MASK)
#define SCG_FIRCDIV_FIRCDIV2_MASK                (0x700U)
#define SCG_FIRCDIV_FIRCDIV2_SHIFT               (8U)
#define SCG_FIRCDIV_FIRCDIV2_WIDTH               (3U)
#define SCG_FIRCDIV_FIRCDIV2(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_FIRCDIV_FIRCDIV2_SHIFT)) & SCG_FIRCDIV_FIRCDIV2_MASK)
/*! @} */

/*! @name FIRCCFG - Fast IRC Configuration Register */
/*! @{ */
#define SCG_FIRCCFG_RANGE_MASK                   (0x3U)
#define SCG_FIRCCFG_RANGE_SHIFT                  (0U)
#define SCG_FIRCCFG_RANGE_WIDTH                  (2U)
#define SCG_FIRCCFG_RANGE(x)                     (((uint32_t)(((uint32_t)(x)) << SCG_FIRCCFG_RANGE_SHIFT)) & SCG_FIRCCFG_RANGE_MASK)
/*! @} */

/*! @name SPLLCSR - System PLL Control Status Register */
/*! @{ */
#define SCG_SPLLCSR_SPLLEN_MASK                  (0x1U)
#define SCG_SPLLCSR_SPLLEN_SHIFT                 (0U)
#define SCG_SPLLCSR_SPLLEN_WIDTH                 (1U)
#define SCG_SPLLCSR_SPLLEN(x)                    (((uint32_t)(((uint32_t)(x)) << SCG_SPLLCSR_SPLLEN_SHIFT)) & SCG_SPLLCSR_SPLLEN_MASK)
#define SCG_SPLLCSR_SPLLCM_MASK                  (0x10000U)
#define SCG_SPLLCSR_SPLLCM_SHIFT                 (16U)
#define SCG_SPLLCSR_SPLLCM_WIDTH                 (1U)
#define SCG_SPLLCSR_SPLLCM(x)                    (((uint32_t)(((uint32_t)(x)) << SCG_SPLLCSR_SPLLCM_SHIFT)) & SCG_SPLLCSR_SPLLCM_MASK)
#define SCG_SPLLCSR_SPLLCMRE_MASK                (0x20000U)
#define SCG_SPLLCSR_SPLLCMRE_SHIFT               (17U)
#define SCG_SPLLCSR_SPLLCMRE_WIDTH               (1U)
#define SCG_SPLLCSR_SPLLCMRE(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SPLLCSR_SPLLCMRE_SHIFT)) & SCG_SPLLCSR_SPLLCMRE_MASK)
#define SCG_SPLLCSR_LK_MASK                      (0x800000U)
#define SCG_SPLLCSR_LK_SHIFT                     (23U)
#define SCG_SPLLCSR_LK_WIDTH                     (1U)
#define SCG_SPLLCSR_LK(x)                        (((uint32_t)(((uint32_t)(x)) << SCG_SPLLCSR_LK_SHIFT)) & SCG_SPLLCSR_LK_MASK)
#define SCG_SPLLCSR_SPLLVLD_MASK                 (0x1000000U)
#define SCG_SPLLCSR_SPLLVLD_SHIFT                (24U)
#define SCG_SPLLCSR_SPLLVLD_WIDTH                (1U)
#define SCG_SPLLCSR_SPLLVLD(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_SPLLCSR_SPLLVLD_SHIFT)) & SCG_SPLLCSR_SPLLVLD_MASK)
#define SCG_SPLLCSR_SPLLSEL_MASK                 (0x2000000U)
#define SCG_SPLLCSR_SPLLSEL_SHIFT                (25U)
#define SCG_SPLLCSR_SPLLSEL_WIDTH                (1U)
#define SCG_SPLLCSR_SPLLSEL(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_SPLLCSR_SPLLSEL_SHIFT)) & SCG_SPLLCSR_SPLLSEL_MASK)
#define SCG_SPLLCSR_SPLLERR_MASK                 (0x4000000U)
#define SCG_SPLLCSR_SPLLERR_SHIFT                (26U)
#define SCG_SPLLCSR_SPLLERR_WIDTH                (1U)
#define SCG_SPLLCSR_SPLLERR(x)                   (((uint32_t)(((uint32_t)(x)) << SCG_SPLLCSR_SPLLERR_SHIFT)) & SCG_SPLLCSR_SPLLERR_MASK)
/*! @} */

/*! @name SPLLDIV - System PLL Divide Register */
/*! @{ */
#define SCG_SPLLDIV_SPLLDIV1_MASK                (0x7U)
#define SCG_SPLLDIV_SPLLDIV1_SHIFT               (0U)
#define SCG_SPLLDIV_SPLLDIV1_WIDTH               (3U)
#define SCG_SPLLDIV_SPLLDIV1(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SPLLDIV_SPLLDIV1_SHIFT)) & SCG_SPLLDIV_SPLLDIV1_MASK)
#define SCG_SPLLDIV_SPLLDIV2_MASK                (0x700U)
#define SCG_SPLLDIV_SPLLDIV2_SHIFT               (8U)
#define SCG_SPLLDIV_SPLLDIV2_WIDTH               (3U)
#define SCG_SPLLDIV_SPLLDIV2(x)                  (((uint32_t)(((uint32_t)(x)) << SCG_SPLLDIV_SPLLDIV2_SHIFT)) & SCG_SPLLDIV_SPLLDIV2_MASK)
/*! @} */

/*! @name SPLLCFG - System PLL Configuration Register */
/*! @{ */
#define SCG_SPLLCFG_SOURCE_MASK                  (0x1U)
#define SCG_SPLLCFG_SOURCE_SHIFT                 (0U)
#define SCG_SPLLCFG_SOURCE_WIDTH                 (1U)
#define SCG_SPLLCFG_SOURCE(x)                    (((uint32_t)(((uint32_t)(x)) << SCG_SPLLCFG_SOURCE_SHIFT)) & SCG_SPLLCFG_SOURCE_MASK)
#define SCG_SPLLCFG_PREDIV_MASK                  (0x700U)
#define SCG_SPLLCFG_PREDIV_SHIFT                 (8U)
#define SCG_SPLLCFG_PREDIV_WIDTH                 (3U)
#define SCG_SPLLCFG_PREDIV(x)                    (((uint32_t)(((uint32_t)(x)) << SCG_SPLLCFG_PREDIV_SHIFT)) & SCG_SPLLCFG_PREDIV_MASK)
#define SCG_SPLLCFG_MULT_MASK                    (0x1F0000U)
#define SCG_SPLLCFG_MULT_SHIFT                   (16U)
#define SCG_SPLLCFG_MULT_WIDTH                   (5U)
#define SCG_SPLLCFG_MULT(x)                      (((uint32_t)(((uint32_t)(x)) << SCG_SPLLCFG_MULT_SHIFT)) & SCG_SPLLCFG_MULT_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group SCG_Register_Masks */

/*!
 * @}
 */ /* end of group SCG_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- SIM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SIM_Peripheral_Access_Layer SIM Peripheral Access Layer
 * @{
 */

/** SIM - Register Layout Typedef */
typedef struct {
  uint8_t RESERVED_0[4];
  __IO uint32_t CHIPCTL;                           /**< Chip Control register, offset: 0x4 */
  uint8_t RESERVED_1[4];
  __IO uint32_t FTMOPT0;                           /**< FTM Option Register 0, offset: 0xC */
  __IO uint32_t LPOCLKS;                           /**< LPO Clock Select Register, offset: 0x10 */
  uint8_t RESERVED_2[4];
  __IO uint32_t ADCOPT;                            /**< ADC Options Register, offset: 0x18 */
  __IO uint32_t FTMOPT1;                           /**< FTM Option Register 1, offset: 0x1C */
  __IO uint32_t MISCTRL0;                          /**< Miscellaneous control register 0, offset: 0x20 */
  __I  uint32_t SDID;                              /**< System Device Identification Register, offset: 0x24 */
  uint8_t RESERVED_3[24];
  __IO uint32_t PLATCGC;                           /**< Platform Clock Gating Control Register, offset: 0x40 */
  uint8_t RESERVED_4[8];
  __I  uint32_t FCFG1;                             /**< Flash Configuration Register 1, offset: 0x4C */
  uint8_t RESERVED_5[4];
  __I  uint32_t UIDH;                              /**< Unique Identification Register High, offset: 0x54 */
  __I  uint32_t UIDMH;                             /**< Unique Identification Register Mid-High, offset: 0x58 */
  __I  uint32_t UIDML;                             /**< Unique Identification Register Mid Low, offset: 0x5C */
  __I  uint32_t UIDL;                              /**< Unique Identification Register Low, offset: 0x60 */
  uint8_t RESERVED_6[4];
  __IO uint32_t CLKDIV4;                           /**< System Clock Divider Register 4, offset: 0x68 */
  __IO uint32_t MISCTRL1;                          /**< Miscellaneous Control register 1, offset: 0x6C */
} SIM_Type, *SIM_MemMapPtr;

/** Number of instances of the SIM module. */
#define SIM_INSTANCE_COUNT                       (1u)

/* SIM - Peripheral instance base addresses */
/** Peripheral SIM base address */
#define SIM_BASE                                 (0x40048000u)
/** Peripheral SIM base pointer */
#define SIM                                      ((SIM_Type *)SIM_BASE)
/** Array initializer of SIM peripheral base addresses */
#define SIM_BASE_ADDRS                           { SIM_BASE }
/** Array initializer of SIM peripheral base pointers */
#define SIM_BASE_PTRS                            { SIM }

/* ----------------------------------------------------------------------------
   -- SIM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SIM_Register_Masks SIM Register Masks
 * @{
 */

/*! @name CHIPCTL - Chip Control register */
/*! @{ */
#define SIM_CHIPCTL_ADC_INTERLEAVE_EN_MASK       (0xFU)
#define SIM_CHIPCTL_ADC_INTERLEAVE_EN_SHIFT      (0U)
#define SIM_CHIPCTL_ADC_INTERLEAVE_EN_WIDTH      (4U)
#define SIM_CHIPCTL_ADC_INTERLEAVE_EN(x)         (((uint32_t)(((uint32_t)(x)) << SIM_CHIPCTL_ADC_INTERLEAVE_EN_SHIFT)) & SIM_CHIPCTL_ADC_INTERLEAVE_EN_MASK)
#define SIM_CHIPCTL_CLKOUTSEL_MASK               (0xF0U)
#define SIM_CHIPCTL_CLKOUTSEL_SHIFT              (4U)
#define SIM_CHIPCTL_CLKOUTSEL_WIDTH              (4U)
#define SIM_CHIPCTL_CLKOUTSEL(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_CHIPCTL_CLKOUTSEL_SHIFT)) & SIM_CHIPCTL_CLKOUTSEL_MASK)
#define SIM_CHIPCTL_CLKOUTDIV_MASK               (0x700U)
#define SIM_CHIPCTL_CLKOUTDIV_SHIFT              (8U)
#define SIM_CHIPCTL_CLKOUTDIV_WIDTH              (3U)
#define SIM_CHIPCTL_CLKOUTDIV(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_CHIPCTL_CLKOUTDIV_SHIFT)) & SIM_CHIPCTL_CLKOUTDIV_MASK)
#define SIM_CHIPCTL_CLKOUTEN_MASK                (0x800U)
#define SIM_CHIPCTL_CLKOUTEN_SHIFT               (11U)
#define SIM_CHIPCTL_CLKOUTEN_WIDTH               (1U)
#define SIM_CHIPCTL_CLKOUTEN(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_CHIPCTL_CLKOUTEN_SHIFT)) & SIM_CHIPCTL_CLKOUTEN_MASK)
#define SIM_CHIPCTL_TRACECLK_SEL_MASK            (0x1000U)
#define SIM_CHIPCTL_TRACECLK_SEL_SHIFT           (12U)
#define SIM_CHIPCTL_TRACECLK_SEL_WIDTH           (1U)
#define SIM_CHIPCTL_TRACECLK_SEL(x)              (((uint32_t)(((uint32_t)(x)) << SIM_CHIPCTL_TRACECLK_SEL_SHIFT)) & SIM_CHIPCTL_TRACECLK_SEL_MASK)
#define SIM_CHIPCTL_PDB_BB_SEL_MASK              (0x2000U)
#define SIM_CHIPCTL_PDB_BB_SEL_SHIFT             (13U)
#define SIM_CHIPCTL_PDB_BB_SEL_WIDTH             (1U)
#define SIM_CHIPCTL_PDB_BB_SEL(x)                (((uint32_t)(((uint32_t)(x)) << SIM_CHIPCTL_PDB_BB_SEL_SHIFT)) & SIM_CHIPCTL_PDB_BB_SEL_MASK)
#define SIM_CHIPCTL_ADC_SUPPLY_MASK              (0x70000U)
#define SIM_CHIPCTL_ADC_SUPPLY_SHIFT             (16U)
#define SIM_CHIPCTL_ADC_SUPPLY_WIDTH             (3U)
#define SIM_CHIPCTL_ADC_SUPPLY(x)                (((uint32_t)(((uint32_t)(x)) << SIM_CHIPCTL_ADC_SUPPLY_SHIFT)) & SIM_CHIPCTL_ADC_SUPPLY_MASK)
#define SIM_CHIPCTL_ADC_SUPPLYEN_MASK            (0x80000U)
#define SIM_CHIPCTL_ADC_SUPPLYEN_SHIFT           (19U)
#define SIM_CHIPCTL_ADC_SUPPLYEN_WIDTH           (1U)
#define SIM_CHIPCTL_ADC_SUPPLYEN(x)              (((uint32_t)(((uint32_t)(x)) << SIM_CHIPCTL_ADC_SUPPLYEN_SHIFT)) & SIM_CHIPCTL_ADC_SUPPLYEN_MASK)
#define SIM_CHIPCTL_SRAMU_RETEN_MASK             (0x100000U)
#define SIM_CHIPCTL_SRAMU_RETEN_SHIFT            (20U)
#define SIM_CHIPCTL_SRAMU_RETEN_WIDTH            (1U)
#define SIM_CHIPCTL_SRAMU_RETEN(x)               (((uint32_t)(((uint32_t)(x)) << SIM_CHIPCTL_SRAMU_RETEN_SHIFT)) & SIM_CHIPCTL_SRAMU_RETEN_MASK)
#define SIM_CHIPCTL_SRAML_RETEN_MASK             (0x200000U)
#define SIM_CHIPCTL_SRAML_RETEN_SHIFT            (21U)
#define SIM_CHIPCTL_SRAML_RETEN_WIDTH            (1U)
#define SIM_CHIPCTL_SRAML_RETEN(x)               (((uint32_t)(((uint32_t)(x)) << SIM_CHIPCTL_SRAML_RETEN_SHIFT)) & SIM_CHIPCTL_SRAML_RETEN_MASK)
#define SIM_CHIPCTL_PDB_BB_SEL_1_MASK            (0x400000U)
#define SIM_CHIPCTL_PDB_BB_SEL_1_SHIFT           (22U)
#define SIM_CHIPCTL_PDB_BB_SEL_1_WIDTH           (1U)
#define SIM_CHIPCTL_PDB_BB_SEL_1(x)              (((uint32_t)(((uint32_t)(x)) << SIM_CHIPCTL_PDB_BB_SEL_1_SHIFT)) & SIM_CHIPCTL_PDB_BB_SEL_1_MASK)
#define SIM_CHIPCTL_PDB_BB_SEL_2_MASK            (0x800000U)
#define SIM_CHIPCTL_PDB_BB_SEL_2_SHIFT           (23U)
#define SIM_CHIPCTL_PDB_BB_SEL_2_WIDTH           (1U)
#define SIM_CHIPCTL_PDB_BB_SEL_2(x)              (((uint32_t)(((uint32_t)(x)) << SIM_CHIPCTL_PDB_BB_SEL_2_SHIFT)) & SIM_CHIPCTL_PDB_BB_SEL_2_MASK)
/*! @} */

/*! @name FTMOPT0 - FTM Option Register 0 */
/*! @{ */
#define SIM_FTMOPT0_FTM0FLTxSEL_MASK             (0x7U)
#define SIM_FTMOPT0_FTM0FLTxSEL_SHIFT            (0U)
#define SIM_FTMOPT0_FTM0FLTxSEL_WIDTH            (3U)
#define SIM_FTMOPT0_FTM0FLTxSEL(x)               (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT0_FTM0FLTxSEL_SHIFT)) & SIM_FTMOPT0_FTM0FLTxSEL_MASK)
#define SIM_FTMOPT0_FTM1FLTxSEL_MASK             (0x70U)
#define SIM_FTMOPT0_FTM1FLTxSEL_SHIFT            (4U)
#define SIM_FTMOPT0_FTM1FLTxSEL_WIDTH            (3U)
#define SIM_FTMOPT0_FTM1FLTxSEL(x)               (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT0_FTM1FLTxSEL_SHIFT)) & SIM_FTMOPT0_FTM1FLTxSEL_MASK)
#define SIM_FTMOPT0_FTM2FLTxSEL_MASK             (0x700U)
#define SIM_FTMOPT0_FTM2FLTxSEL_SHIFT            (8U)
#define SIM_FTMOPT0_FTM2FLTxSEL_WIDTH            (3U)
#define SIM_FTMOPT0_FTM2FLTxSEL(x)               (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT0_FTM2FLTxSEL_SHIFT)) & SIM_FTMOPT0_FTM2FLTxSEL_MASK)
#define SIM_FTMOPT0_FTM3FLTxSEL_MASK             (0x7000U)
#define SIM_FTMOPT0_FTM3FLTxSEL_SHIFT            (12U)
#define SIM_FTMOPT0_FTM3FLTxSEL_WIDTH            (3U)
#define SIM_FTMOPT0_FTM3FLTxSEL(x)               (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT0_FTM3FLTxSEL_SHIFT)) & SIM_FTMOPT0_FTM3FLTxSEL_MASK)
#define SIM_FTMOPT0_FTM0CLKSEL_MASK              (0x3000000U)
#define SIM_FTMOPT0_FTM0CLKSEL_SHIFT             (24U)
#define SIM_FTMOPT0_FTM0CLKSEL_WIDTH             (2U)
#define SIM_FTMOPT0_FTM0CLKSEL(x)                (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT0_FTM0CLKSEL_SHIFT)) & SIM_FTMOPT0_FTM0CLKSEL_MASK)
#define SIM_FTMOPT0_FTM1CLKSEL_MASK              (0xC000000U)
#define SIM_FTMOPT0_FTM1CLKSEL_SHIFT             (26U)
#define SIM_FTMOPT0_FTM1CLKSEL_WIDTH             (2U)
#define SIM_FTMOPT0_FTM1CLKSEL(x)                (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT0_FTM1CLKSEL_SHIFT)) & SIM_FTMOPT0_FTM1CLKSEL_MASK)
#define SIM_FTMOPT0_FTM2CLKSEL_MASK              (0x30000000U)
#define SIM_FTMOPT0_FTM2CLKSEL_SHIFT             (28U)
#define SIM_FTMOPT0_FTM2CLKSEL_WIDTH             (2U)
#define SIM_FTMOPT0_FTM2CLKSEL(x)                (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT0_FTM2CLKSEL_SHIFT)) & SIM_FTMOPT0_FTM2CLKSEL_MASK)
#define SIM_FTMOPT0_FTM3CLKSEL_MASK              (0xC0000000U)
#define SIM_FTMOPT0_FTM3CLKSEL_SHIFT             (30U)
#define SIM_FTMOPT0_FTM3CLKSEL_WIDTH             (2U)
#define SIM_FTMOPT0_FTM3CLKSEL(x)                (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT0_FTM3CLKSEL_SHIFT)) & SIM_FTMOPT0_FTM3CLKSEL_MASK)
/*! @} */

/*! @name LPOCLKS - LPO Clock Select Register */
/*! @{ */
#define SIM_LPOCLKS_LPO1KCLKEN_MASK              (0x1U)
#define SIM_LPOCLKS_LPO1KCLKEN_SHIFT             (0U)
#define SIM_LPOCLKS_LPO1KCLKEN_WIDTH             (1U)
#define SIM_LPOCLKS_LPO1KCLKEN(x)                (((uint32_t)(((uint32_t)(x)) << SIM_LPOCLKS_LPO1KCLKEN_SHIFT)) & SIM_LPOCLKS_LPO1KCLKEN_MASK)
#define SIM_LPOCLKS_LPO32KCLKEN_MASK             (0x2U)
#define SIM_LPOCLKS_LPO32KCLKEN_SHIFT            (1U)
#define SIM_LPOCLKS_LPO32KCLKEN_WIDTH            (1U)
#define SIM_LPOCLKS_LPO32KCLKEN(x)               (((uint32_t)(((uint32_t)(x)) << SIM_LPOCLKS_LPO32KCLKEN_SHIFT)) & SIM_LPOCLKS_LPO32KCLKEN_MASK)
#define SIM_LPOCLKS_LPOCLKSEL_MASK               (0xCU)
#define SIM_LPOCLKS_LPOCLKSEL_SHIFT              (2U)
#define SIM_LPOCLKS_LPOCLKSEL_WIDTH              (2U)
#define SIM_LPOCLKS_LPOCLKSEL(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_LPOCLKS_LPOCLKSEL_SHIFT)) & SIM_LPOCLKS_LPOCLKSEL_MASK)
#define SIM_LPOCLKS_RTCCLKSEL_MASK               (0x30U)
#define SIM_LPOCLKS_RTCCLKSEL_SHIFT              (4U)
#define SIM_LPOCLKS_RTCCLKSEL_WIDTH              (2U)
#define SIM_LPOCLKS_RTCCLKSEL(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_LPOCLKS_RTCCLKSEL_SHIFT)) & SIM_LPOCLKS_RTCCLKSEL_MASK)
/*! @} */

/*! @name ADCOPT - ADC Options Register */
/*! @{ */
#define SIM_ADCOPT_ADC0TRGSEL_MASK               (0x1U)
#define SIM_ADCOPT_ADC0TRGSEL_SHIFT              (0U)
#define SIM_ADCOPT_ADC0TRGSEL_WIDTH              (1U)
#define SIM_ADCOPT_ADC0TRGSEL(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_ADCOPT_ADC0TRGSEL_SHIFT)) & SIM_ADCOPT_ADC0TRGSEL_MASK)
#define SIM_ADCOPT_ADC0SWPRETRG_MASK             (0xEU)
#define SIM_ADCOPT_ADC0SWPRETRG_SHIFT            (1U)
#define SIM_ADCOPT_ADC0SWPRETRG_WIDTH            (3U)
#define SIM_ADCOPT_ADC0SWPRETRG(x)               (((uint32_t)(((uint32_t)(x)) << SIM_ADCOPT_ADC0SWPRETRG_SHIFT)) & SIM_ADCOPT_ADC0SWPRETRG_MASK)
#define SIM_ADCOPT_ADC0PRETRGSEL_MASK            (0x30U)
#define SIM_ADCOPT_ADC0PRETRGSEL_SHIFT           (4U)
#define SIM_ADCOPT_ADC0PRETRGSEL_WIDTH           (2U)
#define SIM_ADCOPT_ADC0PRETRGSEL(x)              (((uint32_t)(((uint32_t)(x)) << SIM_ADCOPT_ADC0PRETRGSEL_SHIFT)) & SIM_ADCOPT_ADC0PRETRGSEL_MASK)
#define SIM_ADCOPT_ADC1TRGSEL_MASK               (0x100U)
#define SIM_ADCOPT_ADC1TRGSEL_SHIFT              (8U)
#define SIM_ADCOPT_ADC1TRGSEL_WIDTH              (1U)
#define SIM_ADCOPT_ADC1TRGSEL(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_ADCOPT_ADC1TRGSEL_SHIFT)) & SIM_ADCOPT_ADC1TRGSEL_MASK)
#define SIM_ADCOPT_ADC1SWPRETRG_MASK             (0xE00U)
#define SIM_ADCOPT_ADC1SWPRETRG_SHIFT            (9U)
#define SIM_ADCOPT_ADC1SWPRETRG_WIDTH            (3U)
#define SIM_ADCOPT_ADC1SWPRETRG(x)               (((uint32_t)(((uint32_t)(x)) << SIM_ADCOPT_ADC1SWPRETRG_SHIFT)) & SIM_ADCOPT_ADC1SWPRETRG_MASK)
#define SIM_ADCOPT_ADC1PRETRGSEL_MASK            (0x3000U)
#define SIM_ADCOPT_ADC1PRETRGSEL_SHIFT           (12U)
#define SIM_ADCOPT_ADC1PRETRGSEL_WIDTH           (2U)
#define SIM_ADCOPT_ADC1PRETRGSEL(x)              (((uint32_t)(((uint32_t)(x)) << SIM_ADCOPT_ADC1PRETRGSEL_SHIFT)) & SIM_ADCOPT_ADC1PRETRGSEL_MASK)
/*! @} */

/*! @name FTMOPT1 - FTM Option Register 1 */
/*! @{ */
#define SIM_FTMOPT1_FTM0SYNCBIT_MASK             (0x1U)
#define SIM_FTMOPT1_FTM0SYNCBIT_SHIFT            (0U)
#define SIM_FTMOPT1_FTM0SYNCBIT_WIDTH            (1U)
#define SIM_FTMOPT1_FTM0SYNCBIT(x)               (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT1_FTM0SYNCBIT_SHIFT)) & SIM_FTMOPT1_FTM0SYNCBIT_MASK)
#define SIM_FTMOPT1_FTM1SYNCBIT_MASK             (0x2U)
#define SIM_FTMOPT1_FTM1SYNCBIT_SHIFT            (1U)
#define SIM_FTMOPT1_FTM1SYNCBIT_WIDTH            (1U)
#define SIM_FTMOPT1_FTM1SYNCBIT(x)               (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT1_FTM1SYNCBIT_SHIFT)) & SIM_FTMOPT1_FTM1SYNCBIT_MASK)
#define SIM_FTMOPT1_FTM2SYNCBIT_MASK             (0x4U)
#define SIM_FTMOPT1_FTM2SYNCBIT_SHIFT            (2U)
#define SIM_FTMOPT1_FTM2SYNCBIT_WIDTH            (1U)
#define SIM_FTMOPT1_FTM2SYNCBIT(x)               (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT1_FTM2SYNCBIT_SHIFT)) & SIM_FTMOPT1_FTM2SYNCBIT_MASK)
#define SIM_FTMOPT1_FTM3SYNCBIT_MASK             (0x8U)
#define SIM_FTMOPT1_FTM3SYNCBIT_SHIFT            (3U)
#define SIM_FTMOPT1_FTM3SYNCBIT_WIDTH            (1U)
#define SIM_FTMOPT1_FTM3SYNCBIT(x)               (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT1_FTM3SYNCBIT_SHIFT)) & SIM_FTMOPT1_FTM3SYNCBIT_MASK)
#define SIM_FTMOPT1_FTM1CH0SEL_MASK              (0x30U)
#define SIM_FTMOPT1_FTM1CH0SEL_SHIFT             (4U)
#define SIM_FTMOPT1_FTM1CH0SEL_WIDTH             (2U)
#define SIM_FTMOPT1_FTM1CH0SEL(x)                (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT1_FTM1CH0SEL_SHIFT)) & SIM_FTMOPT1_FTM1CH0SEL_MASK)
#define SIM_FTMOPT1_FTM2CH0SEL_MASK              (0xC0U)
#define SIM_FTMOPT1_FTM2CH0SEL_SHIFT             (6U)
#define SIM_FTMOPT1_FTM2CH0SEL_WIDTH             (2U)
#define SIM_FTMOPT1_FTM2CH0SEL(x)                (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT1_FTM2CH0SEL_SHIFT)) & SIM_FTMOPT1_FTM2CH0SEL_MASK)
#define SIM_FTMOPT1_FTM2CH1SEL_MASK              (0x100U)
#define SIM_FTMOPT1_FTM2CH1SEL_SHIFT             (8U)
#define SIM_FTMOPT1_FTM2CH1SEL_WIDTH             (1U)
#define SIM_FTMOPT1_FTM2CH1SEL(x)                (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT1_FTM2CH1SEL_SHIFT)) & SIM_FTMOPT1_FTM2CH1SEL_MASK)
#define SIM_FTMOPT1_FTMGLDOK_MASK                (0x8000U)
#define SIM_FTMOPT1_FTMGLDOK_SHIFT               (15U)
#define SIM_FTMOPT1_FTMGLDOK_WIDTH               (1U)
#define SIM_FTMOPT1_FTMGLDOK(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT1_FTMGLDOK_SHIFT)) & SIM_FTMOPT1_FTMGLDOK_MASK)
#define SIM_FTMOPT1_FTM0_OUTSEL_MASK             (0xFF0000U)
#define SIM_FTMOPT1_FTM0_OUTSEL_SHIFT            (16U)
#define SIM_FTMOPT1_FTM0_OUTSEL_WIDTH            (8U)
#define SIM_FTMOPT1_FTM0_OUTSEL(x)               (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT1_FTM0_OUTSEL_SHIFT)) & SIM_FTMOPT1_FTM0_OUTSEL_MASK)
#define SIM_FTMOPT1_FTM3_OUTSEL_MASK             (0xFF000000U)
#define SIM_FTMOPT1_FTM3_OUTSEL_SHIFT            (24U)
#define SIM_FTMOPT1_FTM3_OUTSEL_WIDTH            (8U)
#define SIM_FTMOPT1_FTM3_OUTSEL(x)               (((uint32_t)(((uint32_t)(x)) << SIM_FTMOPT1_FTM3_OUTSEL_SHIFT)) & SIM_FTMOPT1_FTM3_OUTSEL_MASK)
/*! @} */

/*! @name MISCTRL0 - Miscellaneous control register 0 */
/*! @{ */
#define SIM_MISCTRL0_STOP1_MONITOR_MASK          (0x200U)
#define SIM_MISCTRL0_STOP1_MONITOR_SHIFT         (9U)
#define SIM_MISCTRL0_STOP1_MONITOR_WIDTH         (1U)
#define SIM_MISCTRL0_STOP1_MONITOR(x)            (((uint32_t)(((uint32_t)(x)) << SIM_MISCTRL0_STOP1_MONITOR_SHIFT)) & SIM_MISCTRL0_STOP1_MONITOR_MASK)
#define SIM_MISCTRL0_STOP2_MONITOR_MASK          (0x400U)
#define SIM_MISCTRL0_STOP2_MONITOR_SHIFT         (10U)
#define SIM_MISCTRL0_STOP2_MONITOR_WIDTH         (1U)
#define SIM_MISCTRL0_STOP2_MONITOR(x)            (((uint32_t)(((uint32_t)(x)) << SIM_MISCTRL0_STOP2_MONITOR_SHIFT)) & SIM_MISCTRL0_STOP2_MONITOR_MASK)
#define SIM_MISCTRL0_ECC_EEERAM_STAT_MASK        (0x800U)
#define SIM_MISCTRL0_ECC_EEERAM_STAT_SHIFT       (11U)
#define SIM_MISCTRL0_ECC_EEERAM_STAT_WIDTH       (1U)
#define SIM_MISCTRL0_ECC_EEERAM_STAT(x)          (((uint32_t)(((uint32_t)(x)) << SIM_MISCTRL0_ECC_EEERAM_STAT_SHIFT)) & SIM_MISCTRL0_ECC_EEERAM_STAT_MASK)
#define SIM_MISCTRL0_ECC_MGRAM_STAT_MASK         (0x1000U)
#define SIM_MISCTRL0_ECC_MGRAM_STAT_SHIFT        (12U)
#define SIM_MISCTRL0_ECC_MGRAM_STAT_WIDTH        (1U)
#define SIM_MISCTRL0_ECC_MGRAM_STAT(x)           (((uint32_t)(((uint32_t)(x)) << SIM_MISCTRL0_ECC_MGRAM_STAT_SHIFT)) & SIM_MISCTRL0_ECC_MGRAM_STAT_MASK)
#define SIM_MISCTRL0_FTM0_OBE_CTRL_MASK          (0x10000U)
#define SIM_MISCTRL0_FTM0_OBE_CTRL_SHIFT         (16U)
#define SIM_MISCTRL0_FTM0_OBE_CTRL_WIDTH         (1U)
#define SIM_MISCTRL0_FTM0_OBE_CTRL(x)            (((uint32_t)(((uint32_t)(x)) << SIM_MISCTRL0_FTM0_OBE_CTRL_SHIFT)) & SIM_MISCTRL0_FTM0_OBE_CTRL_MASK)
#define SIM_MISCTRL0_FTM1_OBE_CTRL_MASK          (0x20000U)
#define SIM_MISCTRL0_FTM1_OBE_CTRL_SHIFT         (17U)
#define SIM_MISCTRL0_FTM1_OBE_CTRL_WIDTH         (1U)
#define SIM_MISCTRL0_FTM1_OBE_CTRL(x)            (((uint32_t)(((uint32_t)(x)) << SIM_MISCTRL0_FTM1_OBE_CTRL_SHIFT)) & SIM_MISCTRL0_FTM1_OBE_CTRL_MASK)
#define SIM_MISCTRL0_FTM2_OBE_CTRL_MASK          (0x40000U)
#define SIM_MISCTRL0_FTM2_OBE_CTRL_SHIFT         (18U)
#define SIM_MISCTRL0_FTM2_OBE_CTRL_WIDTH         (1U)
#define SIM_MISCTRL0_FTM2_OBE_CTRL(x)            (((uint32_t)(((uint32_t)(x)) << SIM_MISCTRL0_FTM2_OBE_CTRL_SHIFT)) & SIM_MISCTRL0_FTM2_OBE_CTRL_MASK)
#define SIM_MISCTRL0_FTM3_OBE_CTRL_MASK          (0x80000U)
#define SIM_MISCTRL0_FTM3_OBE_CTRL_SHIFT         (19U)
#define SIM_MISCTRL0_FTM3_OBE_CTRL_WIDTH         (1U)
#define SIM_MISCTRL0_FTM3_OBE_CTRL(x)            (((uint32_t)(((uint32_t)(x)) << SIM_MISCTRL0_FTM3_OBE_CTRL_SHIFT)) & SIM_MISCTRL0_FTM3_OBE_CTRL_MASK)
/*! @} */

/*! @name SDID - System Device Identification Register */
/*! @{ */
#define SIM_SDID_FEATURES_MASK                   (0xFFU)
#define SIM_SDID_FEATURES_SHIFT                  (0U)
#define SIM_SDID_FEATURES_WIDTH                  (8U)
#define SIM_SDID_FEATURES(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_SDID_FEATURES_SHIFT)) & SIM_SDID_FEATURES_MASK)
#define SIM_SDID_PACKAGE_MASK                    (0xF00U)
#define SIM_SDID_PACKAGE_SHIFT                   (8U)
#define SIM_SDID_PACKAGE_WIDTH                   (4U)
#define SIM_SDID_PACKAGE(x)                      (((uint32_t)(((uint32_t)(x)) << SIM_SDID_PACKAGE_SHIFT)) & SIM_SDID_PACKAGE_MASK)
#define SIM_SDID_REVID_MASK                      (0xF000U)
#define SIM_SDID_REVID_SHIFT                     (12U)
#define SIM_SDID_REVID_WIDTH                     (4U)
#define SIM_SDID_REVID(x)                        (((uint32_t)(((uint32_t)(x)) << SIM_SDID_REVID_SHIFT)) & SIM_SDID_REVID_MASK)
#define SIM_SDID_RAMSIZE_MASK                    (0xF0000U)
#define SIM_SDID_RAMSIZE_SHIFT                   (16U)
#define SIM_SDID_RAMSIZE_WIDTH                   (4U)
#define SIM_SDID_RAMSIZE(x)                      (((uint32_t)(((uint32_t)(x)) << SIM_SDID_RAMSIZE_SHIFT)) & SIM_SDID_RAMSIZE_MASK)
#define SIM_SDID_DERIVATE_MASK                   (0xF00000U)
#define SIM_SDID_DERIVATE_SHIFT                  (20U)
#define SIM_SDID_DERIVATE_WIDTH                  (4U)
#define SIM_SDID_DERIVATE(x)                     (((uint32_t)(((uint32_t)(x)) << SIM_SDID_DERIVATE_SHIFT)) & SIM_SDID_DERIVATE_MASK)
#define SIM_SDID_SUBSERIES_MASK                  (0xF000000U)
#define SIM_SDID_SUBSERIES_SHIFT                 (24U)
#define SIM_SDID_SUBSERIES_WIDTH                 (4U)
#define SIM_SDID_SUBSERIES(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_SDID_SUBSERIES_SHIFT)) & SIM_SDID_SUBSERIES_MASK)
#define SIM_SDID_GENERATION_MASK                 (0xF0000000U)
#define SIM_SDID_GENERATION_SHIFT                (28U)
#define SIM_SDID_GENERATION_WIDTH                (4U)
#define SIM_SDID_GENERATION(x)                   (((uint32_t)(((uint32_t)(x)) << SIM_SDID_GENERATION_SHIFT)) & SIM_SDID_GENERATION_MASK)
/*! @} */

/*! @name PLATCGC - Platform Clock Gating Control Register */
/*! @{ */
#define SIM_PLATCGC_CGCMSCM_MASK                 (0x1U)
#define SIM_PLATCGC_CGCMSCM_SHIFT                (0U)
#define SIM_PLATCGC_CGCMSCM_WIDTH                (1U)
#define SIM_PLATCGC_CGCMSCM(x)                   (((uint32_t)(((uint32_t)(x)) << SIM_PLATCGC_CGCMSCM_SHIFT)) & SIM_PLATCGC_CGCMSCM_MASK)
#define SIM_PLATCGC_CGCMPU_MASK                  (0x2U)
#define SIM_PLATCGC_CGCMPU_SHIFT                 (1U)
#define SIM_PLATCGC_CGCMPU_WIDTH                 (1U)
#define SIM_PLATCGC_CGCMPU(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_PLATCGC_CGCMPU_SHIFT)) & SIM_PLATCGC_CGCMPU_MASK)
#define SIM_PLATCGC_CGCDMA_MASK                  (0x4U)
#define SIM_PLATCGC_CGCDMA_SHIFT                 (2U)
#define SIM_PLATCGC_CGCDMA_WIDTH                 (1U)
#define SIM_PLATCGC_CGCDMA(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_PLATCGC_CGCDMA_SHIFT)) & SIM_PLATCGC_CGCDMA_MASK)
#define SIM_PLATCGC_CGCERM_MASK                  (0x8U)
#define SIM_PLATCGC_CGCERM_SHIFT                 (3U)
#define SIM_PLATCGC_CGCERM_WIDTH                 (1U)
#define SIM_PLATCGC_CGCERM(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_PLATCGC_CGCERM_SHIFT)) & SIM_PLATCGC_CGCERM_MASK)
#define SIM_PLATCGC_CGCEIM_MASK                  (0x10U)
#define SIM_PLATCGC_CGCEIM_SHIFT                 (4U)
#define SIM_PLATCGC_CGCEIM_WIDTH                 (1U)
#define SIM_PLATCGC_CGCEIM(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_PLATCGC_CGCEIM_SHIFT)) & SIM_PLATCGC_CGCEIM_MASK)
/*! @} */

/*! @name FCFG1 - Flash Configuration Register 1 */
/*! @{ */
#define SIM_FCFG1_DEPART_MASK                    (0xF000U)
#define SIM_FCFG1_DEPART_SHIFT                   (12U)
#define SIM_FCFG1_DEPART_WIDTH                   (4U)
#define SIM_FCFG1_DEPART(x)                      (((uint32_t)(((uint32_t)(x)) << SIM_FCFG1_DEPART_SHIFT)) & SIM_FCFG1_DEPART_MASK)
#define SIM_FCFG1_EEERAMSIZE_MASK                (0xF0000U)
#define SIM_FCFG1_EEERAMSIZE_SHIFT               (16U)
#define SIM_FCFG1_EEERAMSIZE_WIDTH               (4U)
#define SIM_FCFG1_EEERAMSIZE(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_FCFG1_EEERAMSIZE_SHIFT)) & SIM_FCFG1_EEERAMSIZE_MASK)
/*! @} */

/*! @name UIDH - Unique Identification Register High */
/*! @{ */
#define SIM_UIDH_UID127_96_MASK                  (0xFFFFFFFFU)
#define SIM_UIDH_UID127_96_SHIFT                 (0U)
#define SIM_UIDH_UID127_96_WIDTH                 (32U)
#define SIM_UIDH_UID127_96(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_UIDH_UID127_96_SHIFT)) & SIM_UIDH_UID127_96_MASK)
/*! @} */

/*! @name UIDMH - Unique Identification Register Mid-High */
/*! @{ */
#define SIM_UIDMH_UID95_64_MASK                  (0xFFFFFFFFU)
#define SIM_UIDMH_UID95_64_SHIFT                 (0U)
#define SIM_UIDMH_UID95_64_WIDTH                 (32U)
#define SIM_UIDMH_UID95_64(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_UIDMH_UID95_64_SHIFT)) & SIM_UIDMH_UID95_64_MASK)
/*! @} */

/*! @name UIDML - Unique Identification Register Mid Low */
/*! @{ */
#define SIM_UIDML_UID63_32_MASK                  (0xFFFFFFFFU)
#define SIM_UIDML_UID63_32_SHIFT                 (0U)
#define SIM_UIDML_UID63_32_WIDTH                 (32U)
#define SIM_UIDML_UID63_32(x)                    (((uint32_t)(((uint32_t)(x)) << SIM_UIDML_UID63_32_SHIFT)) & SIM_UIDML_UID63_32_MASK)
/*! @} */

/*! @name UIDL - Unique Identification Register Low */
/*! @{ */
#define SIM_UIDL_UID31_0_MASK                    (0xFFFFFFFFU)
#define SIM_UIDL_UID31_0_SHIFT                   (0U)
#define SIM_UIDL_UID31_0_WIDTH                   (32U)
#define SIM_UIDL_UID31_0(x)                      (((uint32_t)(((uint32_t)(x)) << SIM_UIDL_UID31_0_SHIFT)) & SIM_UIDL_UID31_0_MASK)
/*! @} */

/*! @name CLKDIV4 - System Clock Divider Register 4 */
/*! @{ */
#define SIM_CLKDIV4_TRACEFRAC_MASK               (0x1U)
#define SIM_CLKDIV4_TRACEFRAC_SHIFT              (0U)
#define SIM_CLKDIV4_TRACEFRAC_WIDTH              (1U)
#define SIM_CLKDIV4_TRACEFRAC(x)                 (((uint32_t)(((uint32_t)(x)) << SIM_CLKDIV4_TRACEFRAC_SHIFT)) & SIM_CLKDIV4_TRACEFRAC_MASK)
#define SIM_CLKDIV4_TRACEDIV_MASK                (0xEU)
#define SIM_CLKDIV4_TRACEDIV_SHIFT               (1U)
#define SIM_CLKDIV4_TRACEDIV_WIDTH               (3U)
#define SIM_CLKDIV4_TRACEDIV(x)                  (((uint32_t)(((uint32_t)(x)) << SIM_CLKDIV4_TRACEDIV_SHIFT)) & SIM_CLKDIV4_TRACEDIV_MASK)
#define SIM_CLKDIV4_TRACEDIVEN_MASK              (0x10000000U)
#define SIM_CLKDIV4_TRACEDIVEN_SHIFT             (28U)
#define SIM_CLKDIV4_TRACEDIVEN_WIDTH             (1U)
#define SIM_CLKDIV4_TRACEDIVEN(x)                (((uint32_t)(((uint32_t)(x)) << SIM_CLKDIV4_TRACEDIVEN_SHIFT)) & SIM_CLKDIV4_TRACEDIVEN_MASK)
/*! @} */

/*! @name MISCTRL1 - Miscellaneous Control register 1 */
/*! @{ */
#define SIM_MISCTRL1_SW_TRG_MASK                 (0x1U)
#define SIM_MISCTRL1_SW_TRG_SHIFT                (0U)
#define SIM_MISCTRL1_SW_TRG_WIDTH                (1U)
#define SIM_MISCTRL1_SW_TRG(x)                   (((uint32_t)(((uint32_t)(x)) << SIM_MISCTRL1_SW_TRG_SHIFT)) & SIM_MISCTRL1_SW_TRG_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group SIM_Register_Masks */

/*!
 * @}
 */ /* end of group SIM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- SMC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SMC_Peripheral_Access_Layer SMC Peripheral Access Layer
 * @{
 */

/** SMC - Register Layout Typedef */
typedef struct {
  __I  uint32_t VERID;                             /**< SMC Version ID Register, offset: 0x0 */
  __I  uint32_t PARAM;                             /**< SMC Parameter Register, offset: 0x4 */
  __IO uint32_t PMPROT;                            /**< Power Mode Protection register, offset: 0x8 */
  __IO uint32_t PMCTRL;                            /**< Power Mode Control register, offset: 0xC */
  __IO uint32_t STOPCTRL;                          /**< Stop Control Register, offset: 0x10 */
  __I  uint32_t PMSTAT;                            /**< Power Mode Status register, offset: 0x14 */
} SMC_Type, *SMC_MemMapPtr;

/** Number of instances of the SMC module. */
#define SMC_INSTANCE_COUNT                       (1u)

/* SMC - Peripheral instance base addresses */
/** Peripheral SMC base address */
#define SMC_BASE                                 (0x4007E000u)
/** Peripheral SMC base pointer */
#define SMC                                      ((SMC_Type *)SMC_BASE)
/** Array initializer of SMC peripheral base addresses */
#define SMC_BASE_ADDRS                           { SMC_BASE }
/** Array initializer of SMC peripheral base pointers */
#define SMC_BASE_PTRS                            { SMC }

/* ----------------------------------------------------------------------------
   -- SMC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SMC_Register_Masks SMC Register Masks
 * @{
 */

/*! @name VERID - SMC Version ID Register */
/*! @{ */
#define SMC_VERID_FEATURE_MASK                   (0xFFFFU)
#define SMC_VERID_FEATURE_SHIFT                  (0U)
#define SMC_VERID_FEATURE_WIDTH                  (16U)
#define SMC_VERID_FEATURE(x)                     (((uint32_t)(((uint32_t)(x)) << SMC_VERID_FEATURE_SHIFT)) & SMC_VERID_FEATURE_MASK)
#define SMC_VERID_MINOR_MASK                     (0xFF0000U)
#define SMC_VERID_MINOR_SHIFT                    (16U)
#define SMC_VERID_MINOR_WIDTH                    (8U)
#define SMC_VERID_MINOR(x)                       (((uint32_t)(((uint32_t)(x)) << SMC_VERID_MINOR_SHIFT)) & SMC_VERID_MINOR_MASK)
#define SMC_VERID_MAJOR_MASK                     (0xFF000000U)
#define SMC_VERID_MAJOR_SHIFT                    (24U)
#define SMC_VERID_MAJOR_WIDTH                    (8U)
#define SMC_VERID_MAJOR(x)                       (((uint32_t)(((uint32_t)(x)) << SMC_VERID_MAJOR_SHIFT)) & SMC_VERID_MAJOR_MASK)
/*! @} */

/*! @name PARAM - SMC Parameter Register */
/*! @{ */
#define SMC_PARAM_EHSRUN_MASK                    (0x1U)
#define SMC_PARAM_EHSRUN_SHIFT                   (0U)
#define SMC_PARAM_EHSRUN_WIDTH                   (1U)
#define SMC_PARAM_EHSRUN(x)                      (((uint32_t)(((uint32_t)(x)) << SMC_PARAM_EHSRUN_SHIFT)) & SMC_PARAM_EHSRUN_MASK)
#define SMC_PARAM_ELLS_MASK                      (0x8U)
#define SMC_PARAM_ELLS_SHIFT                     (3U)
#define SMC_PARAM_ELLS_WIDTH                     (1U)
#define SMC_PARAM_ELLS(x)                        (((uint32_t)(((uint32_t)(x)) << SMC_PARAM_ELLS_SHIFT)) & SMC_PARAM_ELLS_MASK)
#define SMC_PARAM_ELLS2_MASK                     (0x20U)
#define SMC_PARAM_ELLS2_SHIFT                    (5U)
#define SMC_PARAM_ELLS2_WIDTH                    (1U)
#define SMC_PARAM_ELLS2(x)                       (((uint32_t)(((uint32_t)(x)) << SMC_PARAM_ELLS2_SHIFT)) & SMC_PARAM_ELLS2_MASK)
#define SMC_PARAM_EVLLS0_MASK                    (0x40U)
#define SMC_PARAM_EVLLS0_SHIFT                   (6U)
#define SMC_PARAM_EVLLS0_WIDTH                   (1U)
#define SMC_PARAM_EVLLS0(x)                      (((uint32_t)(((uint32_t)(x)) << SMC_PARAM_EVLLS0_SHIFT)) & SMC_PARAM_EVLLS0_MASK)
/*! @} */

/*! @name PMPROT - Power Mode Protection register */
/*! @{ */
#define SMC_PMPROT_AVLP_MASK                     (0x20U)
#define SMC_PMPROT_AVLP_SHIFT                    (5U)
#define SMC_PMPROT_AVLP_WIDTH                    (1U)
#define SMC_PMPROT_AVLP(x)                       (((uint32_t)(((uint32_t)(x)) << SMC_PMPROT_AVLP_SHIFT)) & SMC_PMPROT_AVLP_MASK)
/*! @} */

/*! @name PMCTRL - Power Mode Control register */
/*! @{ */
#define SMC_PMCTRL_STOPM_MASK                    (0x7U)
#define SMC_PMCTRL_STOPM_SHIFT                   (0U)
#define SMC_PMCTRL_STOPM_WIDTH                   (3U)
#define SMC_PMCTRL_STOPM(x)                      (((uint32_t)(((uint32_t)(x)) << SMC_PMCTRL_STOPM_SHIFT)) & SMC_PMCTRL_STOPM_MASK)
#define SMC_PMCTRL_VLPSA_MASK                    (0x8U)
#define SMC_PMCTRL_VLPSA_SHIFT                   (3U)
#define SMC_PMCTRL_VLPSA_WIDTH                   (1U)
#define SMC_PMCTRL_VLPSA(x)                      (((uint32_t)(((uint32_t)(x)) << SMC_PMCTRL_VLPSA_SHIFT)) & SMC_PMCTRL_VLPSA_MASK)
#define SMC_PMCTRL_RUNM_MASK                     (0x60U)
#define SMC_PMCTRL_RUNM_SHIFT                    (5U)
#define SMC_PMCTRL_RUNM_WIDTH                    (2U)
#define SMC_PMCTRL_RUNM(x)                       (((uint32_t)(((uint32_t)(x)) << SMC_PMCTRL_RUNM_SHIFT)) & SMC_PMCTRL_RUNM_MASK)
/*! @} */

/*! @name STOPCTRL - Stop Control Register */
/*! @{ */
#define SMC_STOPCTRL_STOPO_MASK                  (0xC0U)
#define SMC_STOPCTRL_STOPO_SHIFT                 (6U)
#define SMC_STOPCTRL_STOPO_WIDTH                 (2U)
#define SMC_STOPCTRL_STOPO(x)                    (((uint32_t)(((uint32_t)(x)) << SMC_STOPCTRL_STOPO_SHIFT)) & SMC_STOPCTRL_STOPO_MASK)
/*! @} */

/*! @name PMSTAT - Power Mode Status register */
/*! @{ */
#define SMC_PMSTAT_PMSTAT_MASK                   (0xFFU)
#define SMC_PMSTAT_PMSTAT_SHIFT                  (0U)
#define SMC_PMSTAT_PMSTAT_WIDTH                  (8U)
#define SMC_PMSTAT_PMSTAT(x)                     (((uint32_t)(((uint32_t)(x)) << SMC_PMSTAT_PMSTAT_SHIFT)) & SMC_PMSTAT_PMSTAT_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group SMC_Register_Masks */

/*!
 * @}
 */ /* end of group SMC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- TRGMUX Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TRGMUX_Peripheral_Access_Layer TRGMUX Peripheral Access Layer
 * @{
 */

/** TRGMUX - Size of Registers Arrays */
#define TRGMUX_TRGMUXn_COUNT                      26u

/** TRGMUX - Register Layout Typedef */
typedef struct {
  __IO uint32_t TRGMUXn[TRGMUX_TRGMUXn_COUNT];     /**< TRGMUX DMAMUX0 Register..TRGMUX LPTMR0 Register, array offset: 0x0, array step: 0x4 */
} TRGMUX_Type, *TRGMUX_MemMapPtr;

/** Number of instances of the TRGMUX module. */
#define TRGMUX_INSTANCE_COUNT                    (1u)

/* TRGMUX - Peripheral instance base addresses */
/** Peripheral TRGMUX base address */
#define TRGMUX_BASE                              (0x40063000u)
/** Peripheral TRGMUX base pointer */
#define TRGMUX                                   ((TRGMUX_Type *)TRGMUX_BASE)
/** Array initializer of TRGMUX peripheral base addresses */
#define TRGMUX_BASE_ADDRS                        { TRGMUX_BASE }
/** Array initializer of TRGMUX peripheral base pointers */
#define TRGMUX_BASE_PTRS                         { TRGMUX }

/* ----------------------------------------------------------------------------
   -- TRGMUX Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TRGMUX_Register_Masks TRGMUX Register Masks
 * @{
 */

/*! @name TRGMUXn - TRGMUX DMAMUX0 Register..TRGMUX LPTMR0 Register */
/*! @{ */
#define TRGMUX_TRGMUXn_SEL0_MASK                 (0x3FU)
#define TRGMUX_TRGMUXn_SEL0_SHIFT                (0U)
#define TRGMUX_TRGMUXn_SEL0_WIDTH                (6U)
#define TRGMUX_TRGMUXn_SEL0(x)                   (((uint32_t)(((uint32_t)(x)) << TRGMUX_TRGMUXn_SEL0_SHIFT)) & TRGMUX_TRGMUXn_SEL0_MASK)
#define TRGMUX_TRGMUXn_SEL1_MASK                 (0x3F00U)
#define TRGMUX_TRGMUXn_SEL1_SHIFT                (8U)
#define TRGMUX_TRGMUXn_SEL1_WIDTH                (6U)
#define TRGMUX_TRGMUXn_SEL1(x)                   (((uint32_t)(((uint32_t)(x)) << TRGMUX_TRGMUXn_SEL1_SHIFT)) & TRGMUX_TRGMUXn_SEL1_MASK)
#define TRGMUX_TRGMUXn_SEL2_MASK                 (0x3F0000U)
#define TRGMUX_TRGMUXn_SEL2_SHIFT                (16U)
#define TRGMUX_TRGMUXn_SEL2_WIDTH                (6U)
#define TRGMUX_TRGMUXn_SEL2(x)                   (((uint32_t)(((uint32_t)(x)) << TRGMUX_TRGMUXn_SEL2_SHIFT)) & TRGMUX_TRGMUXn_SEL2_MASK)
#define TRGMUX_TRGMUXn_SEL3_MASK                 (0x3F000000U)
#define TRGMUX_TRGMUXn_SEL3_SHIFT                (24U)
#define TRGMUX_TRGMUXn_SEL3_WIDTH                (6U)
#define TRGMUX_TRGMUXn_SEL3(x)                   (((uint32_t)(((uint32_t)(x)) << TRGMUX_TRGMUXn_SEL3_SHIFT)) & TRGMUX_TRGMUXn_SEL3_MASK)
#define TRGMUX_TRGMUXn_LK_MASK                   (0x80000000U)
#define TRGMUX_TRGMUXn_LK_SHIFT                  (31U)
#define TRGMUX_TRGMUXn_LK_WIDTH                  (1U)
#define TRGMUX_TRGMUXn_LK(x)                     (((uint32_t)(((uint32_t)(x)) << TRGMUX_TRGMUXn_LK_SHIFT)) & TRGMUX_TRGMUXn_LK_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group TRGMUX_Register_Masks */
#define TRGMUX_DMAMUX0_INDEX 0
#define TRGMUX_EXTOUT0_INDEX 1
#define TRGMUX_EXTOUT1_INDEX 2
#define TRGMUX_ADC0_INDEX 3
#define TRGMUX_ADC1_INDEX 4
#define TRGMUX_CMP0_INDEX 7
#define TRGMUX_FTM0_INDEX 10
#define TRGMUX_FTM1_INDEX 11
#define TRGMUX_FTM2_INDEX 12
#define TRGMUX_FTM3_INDEX 13
#define TRGMUX_PDB0_INDEX 14
#define TRGMUX_PDB1_INDEX 15
#define TRGMUX_FLEXIO_INDEX 17
#define TRGMUX_LPIT0_INDEX 18
#define TRGMUX_LPUART0_INDEX 19
#define TRGMUX_LPUART1_INDEX 20
#define TRGMUX_LPI2C0_INDEX 21
#define TRGMUX_LPSPI0_INDEX 23
#define TRGMUX_LPSPI1_INDEX 24
#define TRGMUX_LPTMR0_INDEX 25


/*!
 * @}
 */ /* end of group TRGMUX_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- WDOG Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup WDOG_Peripheral_Access_Layer WDOG Peripheral Access Layer
 * @{
 */

/** WDOG - Register Layout Typedef */
typedef struct {
  __IO uint32_t CS;                                /**< Watchdog Control and Status Register, offset: 0x0 */
  __IO uint32_t CNT;                               /**< Watchdog Counter Register, offset: 0x4 */
  __IO uint32_t TOVAL;                             /**< Watchdog Timeout Value Register, offset: 0x8 */
  __IO uint32_t WIN;                               /**< Watchdog Window Register, offset: 0xC */
} WDOG_Type, *WDOG_MemMapPtr;

/** Number of instances of the WDOG module. */
#define WDOG_INSTANCE_COUNT                      (1u)

/* WDOG - Peripheral instance base addresses */
/** Peripheral WDOG base address */
#define WDOG_BASE                                (0x40052000u)
/** Peripheral WDOG base pointer */
#define WDOG                                     ((WDOG_Type *)WDOG_BASE)
/** Array initializer of WDOG peripheral base addresses */
#define WDOG_BASE_ADDRS                          { WDOG_BASE }
/** Array initializer of WDOG peripheral base pointers */
#define WDOG_BASE_PTRS                           { WDOG }

/* ----------------------------------------------------------------------------
   -- WDOG Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup WDOG_Register_Masks WDOG Register Masks
 * @{
 */

/*! @name CS - Watchdog Control and Status Register */
/*! @{ */
#define WDOG_CS_STOP_MASK                        (0x1U)
#define WDOG_CS_STOP_SHIFT                       (0U)
#define WDOG_CS_STOP_WIDTH                       (1U)
#define WDOG_CS_STOP(x)                          (((uint32_t)(((uint32_t)(x)) << WDOG_CS_STOP_SHIFT)) & WDOG_CS_STOP_MASK)
#define WDOG_CS_WAIT_MASK                        (0x2U)
#define WDOG_CS_WAIT_SHIFT                       (1U)
#define WDOG_CS_WAIT_WIDTH                       (1U)
#define WDOG_CS_WAIT(x)                          (((uint32_t)(((uint32_t)(x)) << WDOG_CS_WAIT_SHIFT)) & WDOG_CS_WAIT_MASK)
#define WDOG_CS_DBG_MASK                         (0x4U)
#define WDOG_CS_DBG_SHIFT                        (2U)
#define WDOG_CS_DBG_WIDTH                        (1U)
#define WDOG_CS_DBG(x)                           (((uint32_t)(((uint32_t)(x)) << WDOG_CS_DBG_SHIFT)) & WDOG_CS_DBG_MASK)
#define WDOG_CS_TST_MASK                         (0x18U)
#define WDOG_CS_TST_SHIFT                        (3U)
#define WDOG_CS_TST_WIDTH                        (2U)
#define WDOG_CS_TST(x)                           (((uint32_t)(((uint32_t)(x)) << WDOG_CS_TST_SHIFT)) & WDOG_CS_TST_MASK)
#define WDOG_CS_UPDATE_MASK                      (0x20U)
#define WDOG_CS_UPDATE_SHIFT                     (5U)
#define WDOG_CS_UPDATE_WIDTH                     (1U)
#define WDOG_CS_UPDATE(x)                        (((uint32_t)(((uint32_t)(x)) << WDOG_CS_UPDATE_SHIFT)) & WDOG_CS_UPDATE_MASK)
#define WDOG_CS_INT_MASK                         (0x40U)
#define WDOG_CS_INT_SHIFT                        (6U)
#define WDOG_CS_INT_WIDTH                        (1U)
#define WDOG_CS_INT(x)                           (((uint32_t)(((uint32_t)(x)) << WDOG_CS_INT_SHIFT)) & WDOG_CS_INT_MASK)
#define WDOG_CS_EN_MASK                          (0x80U)
#define WDOG_CS_EN_SHIFT                         (7U)
#define WDOG_CS_EN_WIDTH                         (1U)
#define WDOG_CS_EN(x)                            (((uint32_t)(((uint32_t)(x)) << WDOG_CS_EN_SHIFT)) & WDOG_CS_EN_MASK)
#define WDOG_CS_CLK_MASK                         (0x300U)
#define WDOG_CS_CLK_SHIFT                        (8U)
#define WDOG_CS_CLK_WIDTH                        (2U)
#define WDOG_CS_CLK(x)                           (((uint32_t)(((uint32_t)(x)) << WDOG_CS_CLK_SHIFT)) & WDOG_CS_CLK_MASK)
#define WDOG_CS_RCS_MASK                         (0x400U)
#define WDOG_CS_RCS_SHIFT                        (10U)
#define WDOG_CS_RCS_WIDTH                        (1U)
#define WDOG_CS_RCS(x)                           (((uint32_t)(((uint32_t)(x)) << WDOG_CS_RCS_SHIFT)) & WDOG_CS_RCS_MASK)
#define WDOG_CS_ULK_MASK                         (0x800U)
#define WDOG_CS_ULK_SHIFT                        (11U)
#define WDOG_CS_ULK_WIDTH                        (1U)
#define WDOG_CS_ULK(x)                           (((uint32_t)(((uint32_t)(x)) << WDOG_CS_ULK_SHIFT)) & WDOG_CS_ULK_MASK)
#define WDOG_CS_PRES_MASK                        (0x1000U)
#define WDOG_CS_PRES_SHIFT                       (12U)
#define WDOG_CS_PRES_WIDTH                       (1U)
#define WDOG_CS_PRES(x)                          (((uint32_t)(((uint32_t)(x)) << WDOG_CS_PRES_SHIFT)) & WDOG_CS_PRES_MASK)
#define WDOG_CS_CMD32EN_MASK                     (0x2000U)
#define WDOG_CS_CMD32EN_SHIFT                    (13U)
#define WDOG_CS_CMD32EN_WIDTH                    (1U)
#define WDOG_CS_CMD32EN(x)                       (((uint32_t)(((uint32_t)(x)) << WDOG_CS_CMD32EN_SHIFT)) & WDOG_CS_CMD32EN_MASK)
#define WDOG_CS_FLG_MASK                         (0x4000U)
#define WDOG_CS_FLG_SHIFT                        (14U)
#define WDOG_CS_FLG_WIDTH                        (1U)
#define WDOG_CS_FLG(x)                           (((uint32_t)(((uint32_t)(x)) << WDOG_CS_FLG_SHIFT)) & WDOG_CS_FLG_MASK)
#define WDOG_CS_WIN_MASK                         (0x8000U)
#define WDOG_CS_WIN_SHIFT                        (15U)
#define WDOG_CS_WIN_WIDTH                        (1U)
#define WDOG_CS_WIN(x)                           (((uint32_t)(((uint32_t)(x)) << WDOG_CS_WIN_SHIFT)) & WDOG_CS_WIN_MASK)
/*! @} */

/*! @name CNT - Watchdog Counter Register */
/*! @{ */
#define WDOG_CNT_CNTLOW_MASK                     (0xFFU)
#define WDOG_CNT_CNTLOW_SHIFT                    (0U)
#define WDOG_CNT_CNTLOW_WIDTH                    (8U)
#define WDOG_CNT_CNTLOW(x)                       (((uint32_t)(((uint32_t)(x)) << WDOG_CNT_CNTLOW_SHIFT)) & WDOG_CNT_CNTLOW_MASK)
#define WDOG_CNT_CNTHIGH_MASK                    (0xFF00U)
#define WDOG_CNT_CNTHIGH_SHIFT                   (8U)
#define WDOG_CNT_CNTHIGH_WIDTH                   (8U)
#define WDOG_CNT_CNTHIGH(x)                      (((uint32_t)(((uint32_t)(x)) << WDOG_CNT_CNTHIGH_SHIFT)) & WDOG_CNT_CNTHIGH_MASK)
/*! @} */

/*! @name TOVAL - Watchdog Timeout Value Register */
/*! @{ */
#define WDOG_TOVAL_TOVALLOW_MASK                 (0xFFU)
#define WDOG_TOVAL_TOVALLOW_SHIFT                (0U)
#define WDOG_TOVAL_TOVALLOW_WIDTH                (8U)
#define WDOG_TOVAL_TOVALLOW(x)                   (((uint32_t)(((uint32_t)(x)) << WDOG_TOVAL_TOVALLOW_SHIFT)) & WDOG_TOVAL_TOVALLOW_MASK)
#define WDOG_TOVAL_TOVALHIGH_MASK                (0xFF00U)
#define WDOG_TOVAL_TOVALHIGH_SHIFT               (8U)
#define WDOG_TOVAL_TOVALHIGH_WIDTH               (8U)
#define WDOG_TOVAL_TOVALHIGH(x)                  (((uint32_t)(((uint32_t)(x)) << WDOG_TOVAL_TOVALHIGH_SHIFT)) & WDOG_TOVAL_TOVALHIGH_MASK)
/*! @} */

/*! @name WIN - Watchdog Window Register */
/*! @{ */
#define WDOG_WIN_WINLOW_MASK                     (0xFFU)
#define WDOG_WIN_WINLOW_SHIFT                    (0U)
#define WDOG_WIN_WINLOW_WIDTH                    (8U)
#define WDOG_WIN_WINLOW(x)                       (((uint32_t)(((uint32_t)(x)) << WDOG_WIN_WINLOW_SHIFT)) & WDOG_WIN_WINLOW_MASK)
#define WDOG_WIN_WINHIGH_MASK                    (0xFF00U)
#define WDOG_WIN_WINHIGH_SHIFT                   (8U)
#define WDOG_WIN_WINHIGH_WIDTH                   (8U)
#define WDOG_WIN_WINHIGH(x)                      (((uint32_t)(((uint32_t)(x)) << WDOG_WIN_WINHIGH_SHIFT)) & WDOG_WIN_WINHIGH_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group WDOG_Register_Masks */

/*!
 * @}
 */ /* end of group WDOG_Peripheral_Access_Layer */


/*!
 * @}
 */ /* end of group Peripheral_access_layer */


/* ----------------------------------------------------------------------------
   -- SDK Compatibility
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SDK_Compatibility_Symbols SDK Compatibility
 * @{
 */

/* No SDK compatibility issues. */

/*!
 * @}
 */ /* end of group SDK_Compatibility_Symbols */


#endif  /* #if !defined(S32K144W_H_) */

