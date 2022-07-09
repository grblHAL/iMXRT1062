/*
 * WMXZ Teensy uSDFS library
 * Copyright (c) 2016 Walter Zimmer.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// following code is modified by Walter Zimmer from
// from version provided by
// Petr Gargulak (NXP Employee)
//https://community.nxp.com/servlet/JiveServlet/download/339474-1-263510/SDHC_K60_Baremetal.ZIP
//see also
//https://community.nxp.com/thread/99202

#if defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1052__) || defined(__IMXRT1062__)

#include <stdlib.h>
#include <string.h>
#include "core_pins.h"  // include calls to kinetis.h or imxrt.h

// for debugging in C
/*
#include "usb_serial.h" // for Serial
#include <stdio.h>
void logg(char c) {usb_serial_putchar(c); usb_serial_flush_output();}
void logVar(char *str, uint32_t var) 
{	char txt[80]; sprintf(txt,"%s: 0x%x\n",str,var); usb_serial_write(txt, strlen(txt)+1); usb_serial_flush_output();}
*/
#include "../ff.h"
#include "../diskio.h"

#include "sd_sdhc.h"
#include "sdio_priv.h"


/******************************************************************************
  Types
******************************************************************************/
#ifdef OLD
/* Status of Disk Functions */
typedef BYTE	DSTATUS;

/* Results of Disk Functions */

typedef enum {
	RES_OK = 0,		/* 0: Successful */
	RES_ERROR,		/* 1: R/W Error */
	RES_WRPRT,		/* 2: Write Protected */
	RES_NOTRDY,		/* 3: Not Ready */
	RES_PARERR,		/* 4: Invalid Parameter */
    RES_NONRSPNS,   /* 5: No Response */ // from old diskio.h
	RES_READERROR,  /* 6: Read Error */
	RES_WRITEERROR  /* 7: Write Error */
} DRESULT;

#endif

enum {
  SDHC_RESULT_OK = 0,             /* 0: Successful */
  SDHC_RESULT_ERROR,              /* 1: R/W Error */
  SDHC_RESULT_WRPRT,              /* 2: Write Protected */
  SDHC_RESULT_NOT_READY,          /* 3: Not Ready */
  SDHC_RESULT_PARERR,             /* 4: Invalid Parameter */
  SDHC_RESULT_NO_RESPONSE         /* 5: No Response */ // from old diskio.h
};

#define SDHC_STATUS_NOINIT              0x01    /* Drive not initialized */
#define SDHC_STATUS_NODISK              0x02    /* No medium in the drive */
#define SDHC_STATUS_PROTECT             0x04    /* Write protected */


typedef struct {
  uint8_t  status;
  uint8_t  highCapacity;
  uint8_t  version2;
  uint8_t  tranSpeed;
  uint32_t address;
  uint32_t numBlocks;
  uint32_t lastCardStatus;
} SD_CARD_DESCRIPTOR;


/******************************************************************************
  Private variables
******************************************************************************/

static SD_CARD_DESCRIPTOR sdCardDesc;

/******************************************************************************
  Private functions
******************************************************************************/

uint8_t sd_CardInit(void);
int sd_CardReadBlocks(void * buff, uint32_t sector, uint32_t count);
int sd_CardWriteBlocks(const void * buff, uint32_t sector, uint32_t count);

static int sd_CMD0_GoToIdle(void);
static int sd_CMD2_Identify(void);
static int sd_CMD3_GetAddress(void);
static int sd_ACMD6_SetBusWidth(uint32_t address, uint32_t width);
static int sd_CMD7_SelectCard(uint32_t address);
static int sd_CMD8_SetInterface(uint32_t cond);
static int sd_CMD9_GetParameters(uint32_t address);
static int sd_CMD16_SetBlockSize(uint32_t block_size);
//static int sd_CMD17_ReadBlock(uint32_t sector);
//static int sd_CMD24_WriteBlock(uint32_t sector);
static int sd_ACMD41_SendOperationCond(uint32_t cond);

static int sd_CMD12_StopTransferWaitForBusy(void);
static int sd_CMD13_WaitForReady(uint32_t address);

/******************************************************************************

    Global functions

******************************************************************************/

DSTATUS SDHC_disk_status()
{	return (DSTATUS) sdCardDesc.status;
}

DSTATUS SDHC_disk_initialize()
{	return (DSTATUS) sd_CardInit();
}

DRESULT SDHC_disk_read(BYTE *buff, DWORD sector, UINT count)
{   return (DRESULT) sd_CardReadBlocks((void *) buff, (uint32_t) sector, (uint32_t) count);
}

DRESULT SDHC_disk_write(const BYTE *buff, DWORD sector, UINT count)
{	return (DRESULT) sd_CardWriteBlocks((void *) buff, (uint32_t) sector, (uint32_t) count);
}

DRESULT SDHC_disk_ioctl(BYTE cmd, BYTE *buff)
{   return RES_OK;
}




#define SDHC_IRQSIGEN_DMA_MASK (SDHC_IRQSIGEN_TCIEN | SDHC_IRQSIGEN_DINTIEN | SDHC_IRQSIGEN_DMAEIEN)

/******************************************************************************
 local functions
******************************************************************************/

#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
  // initialize the SDHC Controller signals
  static void sd_InitGPIO(void)
  {
    PORTE_PCR0 = PORT_PCR_MUX(4) | PORT_PCR_PS | PORT_PCR_PE | PORT_PCR_DSE; /* SDHC.D1  */
    PORTE_PCR1 = PORT_PCR_MUX(4) | PORT_PCR_PS | PORT_PCR_PE | PORT_PCR_DSE; /* SDHC.D0  */
    PORTE_PCR2 = PORT_PCR_MUX(4) | PORT_PCR_DSE;                             /* SDHC.CLK */
    PORTE_PCR3 = PORT_PCR_MUX(4) | PORT_PCR_PS | PORT_PCR_PE | PORT_PCR_DSE; /* SDHC.CMD */
    PORTE_PCR4 = PORT_PCR_MUX(4) | PORT_PCR_PS | PORT_PCR_PE | PORT_PCR_DSE; /* SDHC.D3  */
    PORTE_PCR5 = PORT_PCR_MUX(4) | PORT_PCR_PS | PORT_PCR_PE | PORT_PCR_DSE; /* SDHC.D2  */
  }
  
  // release the SDHC Controller signals
  static void sd_ReleaseGPIO(void)
  {
    PORTE_PCR0 = PORT_PCR_MUX(1) | PORT_PCR_PE | PORT_PCR_PS;   /* PULLUP SDHC.D1  */
    PORTE_PCR1 = PORT_PCR_MUX(1) | PORT_PCR_PE | PORT_PCR_PS;   /* PULLUP SDHC.D0  */
    PORTE_PCR2 = 0;           /* SDHC.CLK */
    PORTE_PCR3 = PORT_PCR_MUX(1) | PORT_PCR_PE | PORT_PCR_PS;   /* PULLUP SDHC.CMD */
    PORTE_PCR4 = PORT_PCR_MUX(1) | PORT_PCR_PE | PORT_PCR_PS;   /* PULLUP SDHC.D3  */
    PORTE_PCR5 = PORT_PCR_MUX(1) | PORT_PCR_PE | PORT_PCR_PS;   /* PULLUP SDHC.D2  */
  }
  
  static void initClock()
  {
    #ifdef HAS_KINETIS_MPU
      // Allow SDHC Bus Master access.
      MPU_RGDAAC0 |= 0x0C000000;
    #endif
    // Enable SDHC clock.
    SIM_SCGC3 |= SIM_SCGC3_SDHC;
  }
  
  static uint32_t sdClock()
  { return F_CPU;
  }

#else

  static void sd_InitGPIO(void)
  {
    { //T4                              // Inverted pins(T4)
      IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_04 = 0; //DAT2  
      IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_05 = 0; //DAT3  
      IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_00 = 0; //CMD   
      //3.3V                                           
      IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_01 = 0; //CLK   
      //GND                                           
      IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_02 = 0; //DAT0 
      IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_03 = 0; //DAT1 
  
      const uint32_t CLOCK_MASK = IOMUXC_SW_PAD_CTL_PAD_PKE |
                                  IOMUXC_SW_PAD_CTL_PAD_DSE(1) |
                                  IOMUXC_SW_PAD_CTL_PAD_SPEED(2);
  
      const uint32_t DATA_MASK = CLOCK_MASK |
                                 (IOMUXC_SW_PAD_CTL_PAD_PUE | IOMUXC_SW_PAD_CTL_PAD_PUS(1));
  
      IOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B0_04 = DATA_MASK;
      IOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B0_05 = DATA_MASK;
      IOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B0_00 = DATA_MASK;
      IOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B0_01 = CLOCK_MASK;
      IOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B0_02 = DATA_MASK;
      IOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B0_03 = DATA_MASK;
    }
  }
  
  static void sd_ReleaseGPIO(void)
  {
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_04 = 5; //GPIO3_IO16
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_05 = 5; //GPIO3_IO17
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_00 = 5; //GPIO3_IO12
    //3.3V
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_01 = 5; //GPIO3_IO13
    //GND
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_02 = 5; //GPIO3_IO14
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_03 = 5; //GPIO3_IO15
  }
  
  static void initClock()
  {
    /* set PDF_528 PLL2PFD0 */
    CCM_ANALOG_PFD_528 |= (1 << 7);
    CCM_ANALOG_PFD_528 &= ~(0x3F << 0);
    CCM_ANALOG_PFD_528 |= ((24) & 0x3F << 0); // 12 - 35
    CCM_ANALOG_PFD_528 &= ~(1 << 7);
  
    /* Enable USDHC clock. */
    CCM_CCGR6 |= CCM_CCGR6_USDHC1(CCM_CCGR_ON);
    CCM_CSCDR1 &= ~(CCM_CSCDR1_USDHC1_CLK_PODF_MASK);
    //
    //  CCM_CSCMR1 &= ~(CCM_CSCMR1_USDHC1_CLK_SEL);     // PLL2PFD2
    CCM_CSCMR1 |= CCM_CSCMR1_USDHC1_CLK_SEL;          // PLL2PFD0
    CCM_CSCDR1 |= CCM_CSCDR1_USDHC1_CLK_PODF((7)); // &0x7
  
    // for testing
    CCM_CCOSR = CCM_CCOSR_CLKO1_EN | CCM_CCOSR_CLKO1_DIV(7) | CCM_CCOSR_CLKO1_SEL(1); //(1: SYS_PLL/2)
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_04 = 6; //CCM_CLKO1 (0 is USDHC1_DAT2)
    // for testing
    CCM_CCOSR |= (CCM_CCOSR_CLKO2_EN | CCM_CCOSR_CLKO2_DIV(7) | CCM_CCOSR_CLKO2_SEL(3)); //(3: usdhc1_clk_root))
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_05 = 6; //CCM_CLKO2 (0 is USDHC1_DAT3)
  }
  
  static uint32_t sdClock()
  {
    uint32_t divider = ((CCM_CSCDR1 >> 11) & 0x7) + 1;
    uint32_t PLL2PFD0 = (528000000U * 3) / ((CCM_ANALOG_PFD_528 & 0x3F) / 6) / divider;
    return PLL2PFD0;
  }

#endif

static void setSdclk(uint32_t kHzMax) {
  const uint32_t DVS_LIMIT = 0X10;
  const uint32_t SDCLKFS_LIMIT = 0X100;
  uint32_t dvs = 1;
  uint32_t sdclkfs = 1;
  uint32_t maxSdclk = 1000 * kHzMax;

  //  uint32_t f_pll = F_CPU;
  uint32_t f_pll = sdClock();

  while ((f_pll / (sdclkfs * DVS_LIMIT) > maxSdclk) && (sdclkfs < SDCLKFS_LIMIT)) {
    sdclkfs <<= 1;
  }
  while ((f_pll / (sdclkfs * dvs) > maxSdclk) && (dvs < DVS_LIMIT)) {
    dvs++;
  }
  // unused // uint32_t m_sdClkKhz = f_pll / (1000 * sdclkfs * dvs);

  sdclkfs >>= 1;
  dvs--;

  #if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    // Disable SDHC clock.
    SDHC_SYSCTL &= ~SDHC_SYSCTL_SDCLKEN;
  #endif

  // Change dividers.
  uint32_t sysctl = SDHC_SYSCTL & ~(SDHC_SYSCTL_DTOCV_MASK
                                    | SDHC_SYSCTL_DVS_MASK | SDHC_SYSCTL_SDCLKFS_MASK);

  SDHC_SYSCTL = sysctl | SDHC_SYSCTL_DTOCV(0x0E) | SDHC_SYSCTL_DVS(dvs)
                | SDHC_SYSCTL_SDCLKFS(sdclkfs);

  // Wait until the SDHC clock is stable.
  while (!(SDHC_PRSSTAT & SDHC_PRSSTAT_SDSTB)) { }

  #if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    // Enable the SDHC clock.
    SDHC_SYSCTL |= SDHC_SYSCTL_SDCLKEN;
  #endif
}


/******************************************************************************

    SDHC functions

******************************************************************************/

static volatile uint32_t dmaDone=0;
//

void sd_isr(void)
{ SDHC_IRQSIGEN &= ~SDHC_IRQSIGEN_DMA_MASK;
  //
  while(!(SDHC_IRQSTAT & SDHC_IRQSTAT_TC));//  SDHC_IRQSTAT &= ~SDHC_IRQSTAT_TC;

  #if defined(__IMXRT1052__) || defined(__IMXRT1062__)
    SDHC_MIX_CTRL &= ~(SDHC_MIX_CTRL_AC23EN | SDHC_MIX_CTRL_DMAEN) ;  
  #endif
  
  if(SDHC_SYSCTL & SDHC_SYSCTL_HCKEN) SDHC_SYSCTL &=  ~SDHC_SYSCTL_HCKEN;
  SDHC_PROCTL &= ~SDHC_PROCTL_D3CD; SDHC_PROCTL |=  SDHC_PROCTL_D3CD;

  dmaDone=1;
}

// initialize the SDHC Controller
// returns status of initialization(OK, nonInit, noCard, CardProtected)
static uint8_t sd_Init(void)
{
  initClock();
  
  // De-init GPIO - to prevent unwanted clocks on bus
  sd_ReleaseGPIO();
  #if defined (__IMXRT1052__) || defined (__IMXRT1062__)
    SDHC_SYSCTL   |= 0xF;
    SDHC_MIX_CTRL |= 0x80000000;
  #endif  

  /* Reset SDHC */
  SDHC_SYSCTL |= SDHC_SYSCTL_RSTA | SDHC_SYSCTL_SDCLKFS(0x80);
  while (SDHC_SYSCTL & SDHC_SYSCTL_RSTA) ; // wait

  /* Set the SDHC initial baud rate divider and start */
  setSdclk(400);

  /* Poll inhibit bits */
  while (SDHC_PRSSTAT & (SDHC_PRSSTAT_CIHB | SDHC_PRSSTAT_CDIHB)) ;

  /* Init GPIO again */
  sd_InitGPIO();

  /* Initial values */ // to do - Check values
//  SDHC_BLKATTR = SDHC_BLKATTR_BLKCNT(1) | SDHC_BLKATTR_BLKSIZE(512);
  
  SDHC_PROCTL &= ~SDHC_PROCTL_DMAS(3); // clear ADMA

  SDHC_PROCTL |=  SDHC_PROCTL_D3CD;
  // SDHC_PROCTL = SDHC_PROCTL_EMODE(SDHC_PROCTL_EMODE_INVARIANT) | SDHC_PROCTL_D3CD;
  //  SDHC_WML |= SDHC_WML_RDWML(SDHC_FIFO_BUFFER_SIZE) | SDHC_WML_WRWML(SDHC_FIFO_BUFFER_SIZE);

  #if defined(__IMXRT1052__) || defined (__IMXRT1062__)
    SDHC_VENDOR = 0x2000F801; // (1<<29 | 0x1F<<11 | 1);
    SDHC_VENDOR2 &= ~(1<<12); //switch off ACMD23 sharing SDMA
  #endif
  
  // clear interrupt status
  SDHC_IRQSTAT = SDHC_IRQSTAT;

  /* Enable requests */
  SDHC_IRQSTATEN =  SDHC_IRQSTAT_CRM | SDHC_IRQSTATEN_CIESEN | 
                    SDHC_IRQSTATEN_TCSEN | SDHC_IRQSTATEN_CCSEN;

  attachInterruptVector(IRQ_SDHC, sd_isr);
  NVIC_SET_PRIORITY(IRQ_SDHC, 6 * 16);
  NVIC_ENABLE_IRQ(IRQ_SDHC);

  // initial clocks... SD spec says only 74 clocks are needed, but if Teensy rebooted
  // while the card was in middle of an operation, thousands of clock cycles can be
  // needed to get the card to complete a prior command and return to a usable state.
  for (int ii = 0; ii < 500; ii++) {
    SDHC_SYSCTL |= SDHC_SYSCTL_INITA;
    while (SDHC_SYSCTL & SDHC_SYSCTL_INITA) ;
  }

  if(!(SDHC_PRSSTAT & SDHC_PRSSTAT_CINS)) return SDHC_STATUS_NODISK;
  return 0;
}

uint8_t SDHC_GetCardType(void)
{
  if (sdCardDesc.status) return 0;
  if (sdCardDesc.version2 == 0) return 1; // SD_CARD_TYPE_SD1
  if (sdCardDesc.highCapacity == 0) return 2; // SD_CARD_TYPE_SD2
  return 3; // SD_CARD_TYPE_SDHC
}

//-----------------------------------------------------------------------------
// initialize the SDHC Controller and SD Card
// returns status of initialization(OK, nonInit, noCard, CardProtected)
uint8_t sd_CardInit(void)
{
  uint8_t resS;
  int resR;

  resS = sd_Init();

  sdCardDesc.status = resS;
  sdCardDesc.address = 0;
  sdCardDesc.highCapacity = 0;
  sdCardDesc.version2 = 0;
  sdCardDesc.numBlocks = 0;
  
  if (resS)  return resS;

  SDHC_IRQSIGEN = 0;
  #if defined (__IMXRT1052__) || defined(__IMXRT1062__)
    uint32_t mixCtrl = 0x80000000; //SDHC_MIX_CTRL;
    // mixCtrl |= SDHC_MIX_CTRL_BCEN; // does not hurt
    // mixCtrl |= SDHC_MIX_CTRL_DTDSEL; // write/read (will be set later
    // mixCtrl |= SDHC_MIX_CTRL_MSBSEL; //for multi block transfer
    // mixCtrl |= SDHC_MIX_CTRL_AC12EN; //for multi block transfer
    // mixCtrl |= SDHC_MIX_CTRL_AC23EN;
    SDHC_MIX_CTRL = mixCtrl;
  #endif

  resR = sd_CMD0_GoToIdle();
  if (resR) { return sdCardDesc.status = SDHC_STATUS_NOINIT;}
  resR = sd_CMD8_SetInterface(0x000001AA); // 3.3V and AA check pattern
  if (resR == SDHC_RESULT_OK) 
  { if (!((SDHC_CMDRSP0 & 0x000001AA)== 0x000001AA)) return sdCardDesc.status = SDHC_STATUS_NOINIT;
    sdCardDesc.highCapacity = 1;
  } 
  else if (resR == SDHC_RESULT_NO_RESPONSE) 
  { // version 1 cards do not respond to CMD8
  } 
  else return sdCardDesc.status = SDHC_STATUS_NOINIT;

  if (sd_ACMD41_SendOperationCond(0))  return sdCardDesc.status = SDHC_STATUS_NOINIT;

  if (SDHC_CMDRSP0 & 0x300000) {
    uint32_t condition = 0x00300000;
    if (sdCardDesc.highCapacity) condition |= 0x40000000;
    //
    uint32_t ii = 0;
    do {
      ii++;
      if (sd_ACMD41_SendOperationCond(condition)) {
        resS = SDHC_STATUS_NOINIT;
        break;
      }
    } while ((!(SDHC_CMDRSP0 & 0x80000000)) && (ii < SDHC_INITIALIZATION_MAX_CNT));

    if (resS) return resS;

    if ((ii >= SDHC_INITIALIZATION_MAX_CNT) || (!(SDHC_CMDRSP0 & 0x40000000)))
      sdCardDesc.highCapacity = 0;
  }

  // Card identify
  SDHC_CMDRSP0=SDHC_CMDRSP1=SDHC_CMDRSP2=SDHC_CMDRSP3=0;
  if (sd_CMD2_Identify())  return sdCardDesc.status = SDHC_STATUS_NOINIT;

  // Get card address
  if (sd_CMD3_GetAddress())  return sdCardDesc.status = SDHC_STATUS_NOINIT;

  sdCardDesc.address = SDHC_CMDRSP0 & 0xFFFF0000;


  // Get card parameters
  if (sd_CMD9_GetParameters(sdCardDesc.address))  return sdCardDesc.status = SDHC_STATUS_NOINIT;

  if (!(SDHC_CMDRSP3 & 0x00C00000)) {
    uint32_t read_bl_len, c_size, c_size_mult;

    read_bl_len = (SDHC_CMDRSP2 >> 8) & 0x0F;
    c_size = SDHC_CMDRSP2 & 0x03;
    c_size = (c_size << 10) | (SDHC_CMDRSP1 >> 22);
    c_size_mult = (SDHC_CMDRSP1 >> 7) & 0x07;
    sdCardDesc.numBlocks = (c_size + 1) * (1 << (c_size_mult + 2)) * (1 << (read_bl_len - 9));
  } else {
    uint32_t c_size;
    sdCardDesc.version2 = 1;
    c_size = (SDHC_CMDRSP1 >> 8) & 0x003FFFFF;
    sdCardDesc.numBlocks = (c_size + 1) << 10;
  }

  // Select card
  if (sd_CMD7_SelectCard(sdCardDesc.address)) return sdCardDesc.status = SDHC_STATUS_NOINIT;

  // Set 512 Block size in SD card
  if (sd_CMD16_SetBlockSize(SDHC_BLOCK_SIZE))  return sdCardDesc.status = SDHC_STATUS_NOINIT;

  // Set 4 bit data bus width
  if (sd_ACMD6_SetBusWidth(sdCardDesc.address, 2))  return sdCardDesc.status = SDHC_STATUS_NOINIT;

  // Set Data bus width also in SDHC controller
  SDHC_PROCTL &= ~SDHC_PROCTL_DTW_MASK;
  SDHC_PROCTL |= SDHC_PROCTL_DTW(SDHC_PROCTL_DTW_4BIT);
//  SDHC_PROCTL |= SDHC_PROTCT_BURST_LENEN(7);
  
  // De-Init GPIO
  sd_ReleaseGPIO();

  // Set the SDHC default baud rate
  setSdclk(60000);
  // SDHC_SetClock(SDHC_SYSCTL_25MHZ);
  // TODO: use CMD6 and CMD9 to detect if card supports 50 MHz
  // then use CMD4 to configure card to high speed mode,
  // and SDHC_SetClock() for 50 MHz config

  // Init GPIO
  sd_InitGPIO();

  return sdCardDesc.status;
}


//-----------------------------------------------------------------------------
// FUNCTION:    sd_CardReadBlock (disk_read)
// SCOPE:       SDHC public related function
// DESCRIPTION: Function read block to disk
//
// PARAMETERS:  buff - pointer on buffer where read data should be stored
//              sector - index of sector
//				count - number of secorts
//
// RETURNS:     result of operation
//-----------------------------------------------------------------------------
#define SDHC_CMD17_XFERTYP (SDHC_XFERTYP_CMDINX(SDHC_CMD17) | SDHC_XFERTYP_RSPTYP(SDHC_XFERTYP_RSPTYP_48) \
                            | SDHC_XFERTYP_DPSEL | SDHC_XFERTYP_DMAEN | SDHC_XFERTYP_DTDSEL) 
#define SDHC_CMD18_XFERTYP (SDHC_XFERTYP_CMDINX(SDHC_CMD18) | SDHC_XFERTYP_RSPTYP(SDHC_XFERTYP_RSPTYP_48) \
                            | SDHC_XFERTYP_DPSEL | SDHC_XFERTYP_DMAEN | SDHC_XFERTYP_DTDSEL \
                            | SDHC_XFERTYP_AC12EN| SDHC_XFERTYP_BCEN | SDHC_XFERTYP_MSBSEL )
//
int sd_CardReadBlocks(void * buff, uint32_t sector, uint32_t count)
{
  int result=0;
  // unused // uint32_t* pData = (uint32_t*)buff;

  
  // Convert LBA to BYTE address if needed
  if (!sdCardDesc.highCapacity)  sector *= 512;

  // Check if this is ready
  if (sdCardDesc.status != 0) return SDHC_RESULT_NOT_READY;

	while(SDHC_PRSSTAT & (SDHC_PRSSTAT_CIHB | SDHC_PRSSTAT_CDIHB | SDHC_PRSSTAT_DLA)) ;

  // clear status
  SDHC_IRQSTAT = SDHC_IRQSTAT;
  
  // use dma: disabling polling
  uint32_t irqstat = SDHC_IRQSTATEN;
  irqstat &= ~(SDHC_IRQSTATEN_BRRSEN | SDHC_IRQSTATEN_BWRSEN | SDHC_IRQSTATEN_CCSEN) ;
  irqstat &= ~(SDHC_IRQSTATEN_DCESEN | SDHC_IRQSTATEN_CCESEN) ;
  // enable status
  irqstat |= /*SDHC_IRQSTATEN_DCESEN | SDHC_IRQSTATEN_CCESEN |*/ SDHC_IRQSTATEN_DMAESEN ; 
  irqstat |= SDHC_IRQSTATEN_DINTSEN | SDHC_IRQSTATEN_TCSEN ;//| SDHC_IRQSTATEN_CCSEN ; 
  SDHC_IRQSTATEN = irqstat;
  
  uint32_t sigen = SDHC_IRQSIGEN;
  sigen |= SDHC_IRQSIGEN_DMA_MASK ;
  SDHC_IRQSIGEN = sigen;
  
  SDHC_SYSCTL |= SDHC_SYSCTL_HCKEN;
  #if defined(__IMXRT1052__) || defined(__IMXRT1062__)
    
    SDHC_MIX_CTRL |= SDHC_MIX_CTRL_DTDSEL ; // read
    SDHC_MIX_CTRL |= SDHC_MIX_CTRL_DMAEN ; // DMA
    SDHC_MIX_CTRL &= ~SDHC_MIX_CTRL_BCEN; // Block Count
    SDHC_MIX_CTRL &= ~SDHC_MIX_CTRL_MSBSEL; //for multi block transfer
    SDHC_MIX_CTRL &= ~SDHC_MIX_CTRL_AC12EN; //for multi block transfer
    if(count>1)
    {
      SDHC_MIX_CTRL |= SDHC_MIX_CTRL_BCEN; // Block Count
      SDHC_MIX_CTRL |= SDHC_MIX_CTRL_MSBSEL; //for multi block transfer
      SDHC_MIX_CTRL |= SDHC_MIX_CTRL_AC12EN; //for multi block transfer
    }
  #endif
  SDHC_BLKATTR = SDHC_BLKATTR_BLKCNT(count) | SDHC_BLKATTR_BLKSIZE(512);

  // enable DMA
  dmaDone=0;
  SDHC_DSADDR  = (uint32_t)buff;

#if defined(__IMXRT1062__)
  if((uint32_t)buff >= 0x20200000U)
    arm_dcache_flush_delete((void *)buff, 512 * count);
#endif

  // send command
    SDHC_CMDARG = sector;
    SDHC_XFERTYP = count==1 ? SDHC_CMD17_XFERTYP: SDHC_CMD18_XFERTYP; 

  // wait for DMA
  while(!dmaDone);
  SDHC_IRQSTAT &= (SDHC_IRQSTAT_CC | SDHC_IRQSTAT_TC);

#if defined(__IMXRT1062__)
  if((uint32_t)buff >= 0x20200000U)
    arm_dcache_flush_delete((void *)buff, 512 * count);
#endif

	// Auto CMD12 is enabled for DMA so call it if DMA error
	if((SDHC_DSADDR < (uint32_t)(buff+(count*512))) && (count>1))
		result=sd_CMD12_StopTransferWaitForBusy();

  return result;
}

//-----------------------------------------------------------------------------
// FUNCTION:    sd_CardWriteBlock (disk_write)
// SCOPE:       SDHC public related function
// DESCRIPTION: Function write block to disk
//
// PARAMETERS:  buff - pointer on buffer where is stored data
//              sector - index of sector
//
// RETURNS:     result of operation
//-----------------------------------------------------------------------------
#define SDHC_CMD24_XFERTYP (SDHC_XFERTYP_CMDINX(SDHC_CMD24) |SDHC_XFERTYP_RSPTYP(SDHC_XFERTYP_RSPTYP_48) \
                            | SDHC_XFERTYP_DPSEL | SDHC_XFERTYP_DMAEN)
                     //| SDHC_XFERTYP_AC12EN | SDHC_XFERTYP_BCEN | SDHC_XFERTYP_MSBSEL 

#define SDHC_CMD25_XFERTYP (SDHC_XFERTYP_CMDINX(SDHC_CMD25) |SDHC_XFERTYP_RSPTYP(SDHC_XFERTYP_RSPTYP_48) \
                            | SDHC_XFERTYP_DPSEL | SDHC_XFERTYP_DMAEN \
                            | SDHC_XFERTYP_AC12EN| SDHC_XFERTYP_BCEN | SDHC_XFERTYP_MSBSEL )
//
int sd_CardWriteBlocks(const void * buff, uint32_t sector, uint32_t count)
{
  int result=0;
  void *buff2 = NULL;
  // unused // const uint32_t *pData = (const uint32_t *)buff;

  // Convert LBA to uint8_t address if needed
  if (!sdCardDesc.highCapacity) sector *= 512;

  // Check if this is ready
  if (sdCardDesc.status != 0) return SDHC_RESULT_NOT_READY;

	while(SDHC_PRSSTAT & (SDHC_PRSSTAT_CIHB | SDHC_PRSSTAT_CDIHB | SDHC_PRSSTAT_DLA)) ;

  // clear status
  SDHC_IRQSTAT = SDHC_IRQSTAT;

  uint32_t irqstat = SDHC_IRQSTATEN;
  // use dma: disabling polling
  irqstat &= ~(SDHC_IRQSTATEN_BRRSEN | SDHC_IRQSTATEN_BWRSEN | SDHC_IRQSTATEN_CCSEN) ;
  irqstat &= ~(SDHC_IRQSTATEN_DCESEN | SDHC_IRQSTATEN_CCESEN) ;
  // enable status
  irqstat |= SDHC_IRQSTATEN_DCESEN | SDHC_IRQSTATEN_CCESEN | SDHC_IRQSTATEN_DMAESEN ; 
  irqstat |= SDHC_IRQSTATEN_DINTSEN | SDHC_IRQSTATEN_TCSEN ;//| SDHC_IRQSTATEN_CCSEN ; 
  SDHC_IRQSTATEN = irqstat;
  
  uint32_t sigen = SDHC_IRQSIGEN;
  sigen |= SDHC_IRQSIGEN_DMA_MASK ;
  SDHC_IRQSIGEN = sigen;
  
  SDHC_SYSCTL |= SDHC_SYSCTL_HCKEN;
  #if defined(__IMXRT1052__) || defined(__IMXRT1062__)
    SDHC_MIX_CTRL &= ~ SDHC_MIX_CTRL_DTDSEL;  // write
    SDHC_MIX_CTRL |=  SDHC_MIX_CTRL_DMAEN ;   //DMA
    SDHC_MIX_CTRL &= ~SDHC_MIX_CTRL_BCEN; // Block Count
    SDHC_MIX_CTRL &= ~SDHC_MIX_CTRL_MSBSEL; //for multi block transfer
    SDHC_MIX_CTRL &= ~SDHC_MIX_CTRL_AC12EN; //for multi block transfer
    if(count>1)
    {
      SDHC_MIX_CTRL |= SDHC_MIX_CTRL_BCEN; // Block Count
      SDHC_MIX_CTRL |= SDHC_MIX_CTRL_MSBSEL; //for multi block transfer
      SDHC_MIX_CTRL |= SDHC_MIX_CTRL_AC12EN; //for multi block transfer
    }
  #endif
  SDHC_BLKATTR = SDHC_BLKATTR_BLKCNT(count) | SDHC_BLKATTR_BLKSIZE(512);

  if((uint32_t)buff % 4) {
    if((buff2 = malloc(512 * count)))
        buff = memcpy(buff2, buff, 512 * count);
  }

#if defined(__IMXRT1062__)
  if((uint32_t)buff >= 0x20200000U)
    arm_dcache_flush_delete((void *)buff, 512 * count);
#endif

  // enable DMA
  dmaDone=0;
  SDHC_DSADDR  = (uint32_t)buff;
  //
  // send write command
  SDHC_CMDARG = sector;
  SDHC_XFERTYP = count==1 ? SDHC_CMD24_XFERTYP: SDHC_CMD25_XFERTYP; 
  //
  // wait for  DMA to finish
  while(!dmaDone);

  SDHC_IRQSTAT &= (SDHC_IRQSTAT_CC | SDHC_IRQSTAT_TC);
  while(SDHC_PRSSTAT & SDHC_PRSSTAT_DLA);

  //check for SD status (if data are written?)
  result = sd_CMD13_WaitForReady(sdCardDesc.address);

  if(buff2)
    free(buff2);
  
	// Auto CMD12 is enabled for DMA so call it when transfer error
	if((result != SDHC_RESULT_OK) && (count>1))
		result=sd_CMD12_StopTransferWaitForBusy();

  return result;
}

/******************************************************************************

    Private functions

******************************************************************************/
// waits for status bits sets
static uint32_t sd_WaitStatus(uint32_t mask)
{
  uint32_t             result;
  uint32_t             timeout = 1 << 24;
  do
  { result = SDHC_IRQSTAT & mask;
    timeout--;
  } while (!result && (timeout));
  if (timeout) return result;
  return 0;
}

/***************************** LOW Level SDHC interface ********************************/
// sends the command to SDcard
static int sd_CMD(uint32_t xfertyp, uint32_t arg)
{
  // Card removal check preparation
  SDHC_IRQSTAT |= SDHC_IRQSTAT_CRM;

  // Wait for cmd line idle // to do timeout PRSSTAT[CDIHB] and the PRSSTAT[CIHB]
  while ((SDHC_PRSSTAT & SDHC_PRSSTAT_CIHB) || (SDHC_PRSSTAT & SDHC_PRSSTAT_CDIHB));

  // send command
  SDHC_CMDARG = arg;
  SDHC_XFERTYP = xfertyp;

  /* Wait for response */
  const uint32_t mask = SDHC_IRQSTAT_CIE | SDHC_IRQSTAT_CEBE | SDHC_IRQSTAT_CCE | SDHC_IRQSTAT_CC;
  if (sd_WaitStatus(mask) != SDHC_IRQSTAT_CC)
  { SDHC_IRQSTAT |= mask;
    return SDHC_RESULT_ERROR;
  }
  return SDHC_RESULT_OK;
  
  /* Check card removal */
  if (SDHC_IRQSTAT & SDHC_IRQSTAT_CRM) 
  {   SDHC_IRQSTAT |= SDHC_IRQSTAT_CTOE | SDHC_IRQSTAT_CC;
      return SDHC_RESULT_NOT_READY;
  }
  
  /* Get response, if available */
  if (SDHC_IRQSTAT & SDHC_IRQSTAT_CTOE)
  { SDHC_IRQSTAT |= SDHC_IRQSTAT_CTOE | SDHC_IRQSTAT_CC;
    return SDHC_RESULT_NO_RESPONSE;
  }
  SDHC_IRQSTAT |= SDHC_IRQSTAT_CC;

  return SDHC_RESULT_OK;
}

// send CMD 55 Application specific command
#define SDHC_CMD55_XFERTYP (SDHC_XFERTYP_CMDINX(SDHC_CMD55) | SDHC_XFERTYP_CICEN | \
             SDHC_XFERTYP_CCCEN | SDHC_XFERTYP_RSPTYP(SDHC_XFERTYP_RSPTYP_48))

static int sd_ACMD(uint32_t xfertyp, uint32_t arg1, uint32_t arg2)
{
  int result =sd_CMD(SDHC_CMD55_XFERTYP,arg1);
  if(!(result == SDHC_RESULT_OK)) return result;
  return sd_CMD(xfertyp,arg2);
}

/*
 * Convenience interfaces
 */
// ---------- sends CMD0 to put SDCARD to idle
#define SDHC_CMD0_XFERTYP (SDHC_XFERTYP_CMDINX(SDHC_CMD0) | SDHC_XFERTYP_RSPTYP(SDHC_XFERTYP_RSPTYP_NO))
//
static int sd_CMD0_GoToIdle(void){ return sd_CMD(SDHC_CMD0_XFERTYP,0); }

// ---------- sends CMD2 to identify card
#define SDHC_CMD2_XFERTYP (SDHC_XFERTYP_CMDINX(SDHC_CMD2) | SDHC_XFERTYP_CCCEN \
                          | SDHC_XFERTYP_RSPTYP(SDHC_XFERTYP_RSPTYP_136))
//
static int sd_CMD2_Identify(void){  return sd_CMD(SDHC_CMD2_XFERTYP,0); }

// ---------- sends CMD 3 to get address
#define SDHC_CMD3_XFERTYP (SDHC_XFERTYP_CMDINX(SDHC_CMD3) | SDHC_XFERTYP_CICEN | \
                         SDHC_XFERTYP_CCCEN | SDHC_XFERTYP_RSPTYP(SDHC_XFERTYP_RSPTYP_48))
//
static int sd_CMD3_GetAddress(void){  return sd_CMD(SDHC_CMD3_XFERTYP,0); }


// ---------- sends ACMD6 to set bus width
#define SDHC_ACMD6_XFERTYP (SDHC_XFERTYP_CMDINX(SDHC_ACMD6) | SDHC_XFERTYP_CICEN | \
             SDHC_XFERTYP_CCCEN | SDHC_XFERTYP_RSPTYP(SDHC_XFERTYP_RSPTYP_48))
//
static int sd_ACMD6_SetBusWidth(uint32_t address, uint32_t width) 
{ return sd_ACMD(SDHC_ACMD6_XFERTYP,address, width); }

             
// ---------- sends CMD 7 to select card
#define SDHC_CMD7_XFERTYP (SDHC_XFERTYP_CMDINX(SDHC_CMD7) | SDHC_XFERTYP_CICEN | \
             SDHC_XFERTYP_CCCEN | SDHC_XFERTYP_RSPTYP(SDHC_XFERTYP_RSPTYP_48BUSY))
//
static int sd_CMD7_SelectCard(uint32_t address){  return sd_CMD(SDHC_CMD7_XFERTYP, address);}
             
// ---------- CMD8 to send interface condition
#define SDHC_CMD8_XFERTYP (SDHC_XFERTYP_CMDINX(SDHC_CMD8) | SDHC_XFERTYP_CICEN | \
             SDHC_XFERTYP_CCCEN | SDHC_XFERTYP_RSPTYP(SDHC_XFERTYP_RSPTYP_48))
//
static int sd_CMD8_SetInterface(uint32_t cond){  return sd_CMD(SDHC_CMD8_XFERTYP, cond); }

// ---------- sends CMD 9 to get interface condition
#define SDHC_CMD9_XFERTYP (SDHC_XFERTYP_CMDINX(SDHC_CMD9) | SDHC_XFERTYP_CCCEN | \
             SDHC_XFERTYP_RSPTYP(SDHC_XFERTYP_RSPTYP_136))
//
static int sd_CMD9_GetParameters(uint32_t address)
{
  int result = sd_CMD(SDHC_CMD9_XFERTYP, address);
  if (result == SDHC_RESULT_OK) { sdCardDesc.tranSpeed = SDHC_CMDRSP2 >> 24;}
  return result;
}

// ---------- sends CMD12 to stop transfer
#define SDHC_CMD12_XFERTYP (SDHC_XFERTYP_CMDINX(SDHC_CMD12) | SDHC_XFERTYP_CMDTYP(SDHC_XFERTYP_CMDTYP_ABORT) | \
             SDHC_XFERTYP_CICEN | SDHC_XFERTYP_CCCEN | SDHC_XFERTYP_RSPTYP(SDHC_XFERTYP_RSPTYP_48BUSY))
//
static int sd_CMD12_StopTransfer(void){  return sd_CMD(SDHC_CMD12_XFERTYP, 0);}

// ---------- sends CMD12 to stop transfer and first waits to ready SDCard
static int sd_CMD12_StopTransferWaitForBusy(void)
{
  uint32_t timeOut = 1000;
  int result;
  do 
  { result = sd_CMD12_StopTransfer();
    timeOut--;
  } while (timeOut && (SDHC_PRSSTAT & SDHC_PRSSTAT_DLA) && result == SDHC_RESULT_OK);
  
  if (result != SDHC_RESULT_OK)  return result;
  if (!timeOut)  return SDHC_RESULT_NO_RESPONSE;

  return SDHC_RESULT_OK;
}

// ---------- sends CMD13 to check uSD status
#define SDHC_CMD13_XFERTYP (SDHC_XFERTYP_CMDINX(SDHC_CMD13) | SDHC_XFERTYP_RSPTYP(SDHC_XFERTYP_RSPTYP_48))
//
static int sd_CMD13_Check_Status(uint32_t address){  return sd_CMD(SDHC_CMD13_XFERTYP, address);}

#define CARD_STATUS_READY_FOR_DATA	(1UL << 8)
// ---------- sends CMD13 to check uSD status and wait for ready
static int sd_CMD13_WaitForReady(uint32_t address)
{ int result;
  do
  { while ((SDHC_PRSSTAT & SDHC_PRSSTAT_CIHB) || (SDHC_PRSSTAT & SDHC_PRSSTAT_CDIHB)) ;
    SDHC_IRQSTATEN |= SDHC_IRQSTATEN_CCSEN;
    SDHC_IRQSTAT=SDHC_IRQSTAT;
    // CMD13 to check uSD status
    result = sd_CMD13_Check_Status(sdCardDesc.address);
    if (result != SDHC_RESULT_OK)  return result;
  } while(!((SDHC_CMDRSP0 & CARD_STATUS_READY_FOR_DATA)==CARD_STATUS_READY_FOR_DATA)); // while data?
  return SDHC_RESULT_OK;
}

// ---------- sends CMD16 to set block size
#define SDHC_CMD16_XFERTYP (SDHC_XFERTYP_CMDINX(SDHC_CMD16) | SDHC_XFERTYP_CICEN | \
             SDHC_XFERTYP_CCCEN | SDHC_XFERTYP_RSPTYP(SDHC_XFERTYP_RSPTYP_48))
//
static int sd_CMD16_SetBlockSize(uint32_t block_size){  return sd_CMD(SDHC_CMD16_XFERTYP, block_size);}

// ---------- ACMD 41 to send operation condition
#define SDHC_ACMD41_XFERTYP (SDHC_XFERTYP_CMDINX(SDHC_ACMD41) | SDHC_XFERTYP_RSPTYP(SDHC_XFERTYP_RSPTYP_48))
//
static int sd_ACMD41_SendOperationCond(uint32_t cond){  return sd_ACMD(SDHC_ACMD41_XFERTYP,0, cond);}


#endif // __MK64FX512__ or __MK66FX1M0__ or __IMXRT1052__ or __IMXRT1062__














#ifdef XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#if defined(__MK66FX1M0__) || defined (__MK64FX512__) || defined(__IMXRT1052__) || defined(__IMXRT1062__)
#include "sd_defs.h"
#include "sd_sdhc.h"

#include "sdio.h"
#include "sdio_priv.h"

#define SDHC_IRQSIGEN_DMA_MASK (SDHC_IRQSIGEN_TCIEN | SDHC_IRQSIGEN_DINTIEN | SDHC_IRQSIGEN_DMAEIEN)
SD_CARD_DESCRIPTOR sdCardDesc;

int SDHC_disk_status()
{
    return sdCardDesc.status;
}

int SDHC_disk_initialize()
{
	DSTATUS resS;

	uint32_t kbaudrate;

	resS = sdhc_Init();

	sdCardDesc.status = resS;
	sdCardDesc.address = 0;
	sdCardDesc.highCapacity = 0;
	sdCardDesc.version2 = 0;
	sdCardDesc.numBlocks = 0;
	
	if(resS) return resS;
	if(!sdhc_CMD(SDHC_CMD0_XFERTYP, 0)) SDHC_ERROR(STA_NOINIT, SD_CARD_ERROR_CMD0);

	if(sdhc_CMD(SDHC_CMD8_XFERTYP, 0X1AA)) // 3.3V and AA check pattern
	{
		if (SDHC_CMDRSP0 != 0X1AA) SDHC_ERROR(STA_NOINIT, SD_CARD_ERROR_CMD8);
		sdCardDesc.version2 = 1;
	}

	uint32_t arg = sdCardDesc.version2 ? 0X40300000 : 0x00300000;
	int ii = SDHC_INITIALIZATION_MAX_CNT;
	do {
		if(!(sdhc_CMD(SDHC_CMD55_XFERTYP,0) && sdhc_CMD(SDHC_ACMD41_XFERTYP,arg)) || !ii ) 
			SDHC_ERROR(STA_NOINIT, SD_CARD_ERROR_ACMD41);
	} while ((SDHC_CMDRSP0 & 0x80000000) == 0 && ii--);

	m_sdhc_ocr = SDHC_CMDRSP0;
	if (SDHC_CMDRSP0 & 0x40000000) 
	{	// is high capacity
		sdCardDesc.highCapacity = 1;
	}

	// Card identify
	if(!sdhc_CMD(SDHC_CMD2_XFERTYP,0)) SDHC_ERROR(STA_NOINIT, SD_CARD_ERROR_CMD2);

	// Get card address
	if(!sdhc_CMD(SDHC_CMD3_XFERTYP,0)) SDHC_ERROR(STA_NOINIT, SD_CARD_ERROR_CMD3);

	sdCardDesc.address = SDHC_CMDRSP0 & 0xFFFF0000;

	// Get card parameters 
	if(!sdhc_CMD(SDHC_CMD9_XFERTYP,sdCardDesc.address)) SDHC_ERROR(STA_NOINIT, SD_CARD_ERROR_CMD9);
	if (0 == (SDHC_CMDRSP3 & 0x00C00000))
	{
		LWord read_bl_len, c_size, c_size_mult;

		read_bl_len = (SDHC_CMDRSP2 >> 8) & 0x0F;
		c_size = SDHC_CMDRSP2 & 0x03;
		c_size = (c_size << 10) | (SDHC_CMDRSP1 >> 22);
		c_size_mult = (SDHC_CMDRSP1 >> 7) & 0x07;
		sdCardDesc.numBlocks = (c_size + 1) * (1 << (c_size_mult + 2)) * (1 << (read_bl_len - 9));
	}
	else
	{
		LWord c_size;

		sdCardDesc.version2 = 1;
		c_size = (SDHC_CMDRSP1 >> 8) & 0x003FFFFF;
		sdCardDesc.numBlocks = (c_size + 1) << 10;
	}

	if(!sdhc_CMD(SDHC_CMD10_XFERTYP,sdCardDesc.address)) {SDHC_ERROR(STA_NOINIT, SD_CARD_ERROR_CMD10);}
	else
	{	uint8_t d[16];
		  d[0]  = SDHC_CMDRSP3 >> 16;
		  d[1]  = SDHC_CMDRSP3 >> 8;
		  d[2]  = SDHC_CMDRSP3;
		  d[3]  = SDHC_CMDRSP2 >> 24;
		  d[4]  = SDHC_CMDRSP2 >> 16;
		  d[5]  = SDHC_CMDRSP2 >> 8;
		  d[6]  = SDHC_CMDRSP2;
		  d[7]  = SDHC_CMDRSP1 >> 24;
		  d[8]  = SDHC_CMDRSP1 >> 16;
		  d[9]  = SDHC_CMDRSP1 >> 8;
		  d[10] = SDHC_CMDRSP1;
		  d[11] = SDHC_CMDRSP0 >> 24;
		  d[12] = SDHC_CMDRSP0 >> 16;
		  d[13] = SDHC_CMDRSP0 >> 8;
		  d[14] = SDHC_CMDRSP0;
		  d[15] = 0;
	} // function not used yet
	
logg('h');
	// Select card
	if(!sdhc_CMD(SDHC_CMD7_XFERTYP,sdCardDesc.address)) SDHC_ERROR(STA_NOINIT, SD_CARD_ERROR_CMD7);

logg('j');
	// Set Block Size to 512
	// Block Size in SDHC Controller is already set to 512 by SDHC_Init();
	// Set 512 Block size in SD card
	if(!sdhc_CMD(SDHC_CMD16_XFERTYP,SDHC_BLOCK_SIZE)) SDHC_ERROR(STA_NOINIT, SD_CARD_ERROR_CMD16);

logg('k');
	if(SDHC_DO4BITS)
	{
		// Set 4 bit data bus width
		if(!(sdhc_CMD(SDHC_CMD55_XFERTYP,sdCardDesc.address) && sdhc_CMD(SDHC_ACMD6_XFERTYP,2))) SDHC_ERROR(STA_NOINIT, SD_CARD_ERROR_ACMD6);

		// Set Data bus width also in SDHC controller
		SDHC_PROCTL &= (~ SDHC_PROCTL_DTW_MASK);
		SDHC_PROCTL |= SDHC_PROCTL_DTW(SDHC_PROCTL_DTW_4BIT);
	}  
logg('l');

#if defined(__MK66FX1M0__) || defined (__MK64FX512__)
	#if SDHC_USE_ISR == 1
		// adapted from Bill Greiman
		// but he has last condition wrong (following section 60.7.4.2 in K66P144M180SF5RMV2.pdf)
		if(sdhc_CMD6_Switch(0X00FFFFF1,m_sdhc_CMD6_Status) && (m_sdhc_CMD6_Status[13] & 2) &&
		   sdhc_CMD6_Switch(0X80FFFFF1,m_sdhc_CMD6_Status) && !((m_sdhc_CMD6_Status[16] & 0xF) == 0xf))
			kbaudrate = 60000;
		else 
			kbaudrate = 25000;
	#else
			kbaudrate = 50000;	
	#endif
#else
		kbaudrate = 50000;	
#endif
/*
	// De-Init GPIO
	sdhc_InitGPIO(3);

	// Set the SDHC default baud rate
	sdhc_SetBaudrate(kbaudrate);

	// Init GPIO
	sdhc_InitGPIO(0xFFFF);
*/
logg('m');
	return sdCardDesc.status;
}

int SDHC_disk_read(BYTE *buff, DWORD sector, UINT count)
{
	DRESULT result = RES_OK;
	LWord* pData = (LWord*)buff;
logg('A');
	// Check if this is ready
	if(sdCardDesc.status != 0)  return RES_NOTRDY;
logg('B');

	// Check the valid Count of block
	if(!count) return RES_PARERR; 

	// Convert LBA to UCHAR address if needed
	if(!sdCardDesc.highCapacity) sector *= 512;
    
//	delayMicroseconds(100); // this is workaround to avoid sdhc blocking on BREN
//	m_sdhc_waitCmd13 = 1;
//	uint32_t cnt = 1<<16; while ((--cnt) && sdhc_isBusy()) /* yield() */;  if(!cnt) return RES_READERROR;
//	m_sdhc_waitCmd13 = 0;
logg('C');

	while(SDHC_PRSSTAT & (SDHC_PRSSTAT_CIHB | SDHC_PRSSTAT_CDIHB | SDHC_PRSSTAT_DLA)) /* yield() */;
logg('D');
	
	SDHC_IRQSTAT = SDHC_IRQSTAT; // clear interrupt status register

	//
	#if SDHC_TRANSFERTYPE == SDHC_TRANSFERTYPE_DMA
		SDHC_DSADDR  = (LWord)pData;  
		SDHC_SYSCTL |=  SDHC_SYSCTL_HCKEN;
	#endif   

	// use dma: disabling polling
	uint32_t irqstat = SDHC_IRQSTATEN;
	irqstat &= ~(SDHC_IRQSTATEN_BRRSEN | SDHC_IRQSTATEN_BWRSEN | SDHC_IRQSTATEN_CCSEN) ;
	irqstat &= ~(SDHC_IRQSTATEN_DCESEN | SDHC_IRQSTATEN_CCESEN) ;
	// enable status
	irqstat |= SDHC_IRQSTATEN_DMAESEN | SDHC_IRQSTATEN_DINTSEN | SDHC_IRQSTATEN_TCSEN ;
	SDHC_IRQSTATEN = irqstat;
	
	uint32_t sigen = SDHC_IRQSIGEN;
	sigen |= SDHC_IRQSIGEN_DMA_MASK ;
	SDHC_IRQSIGEN = sigen;
	
	SDHC_BLKATTR = SDHC_BLKATTR_BLKCNT(count) | SDHC_BLKATTR_BLKSIZE(SDHC_BLOCK_SIZE);
	#if defined(__IMXRT1052__) || defined(__IMXRT1062__)
		SDHC_MIX_CTRL |= SDHC_MIX_CTRL_DTDSEL ; // read 
		SDHC_MIX_CTRL |=  SDHC_MIX_CTRL_DMAEN ; // DMA
	#endif
	sdhc_enableDma();
logg('E');
	
	SDHC_CMDARG = sector;
	SDHC_XFERTYP = count==1 ? SDHC_CMD17_XFERTYP: SDHC_CMD18_XFERTYP; 

	#if SDHC_TRANSFERTYPE == SDHC_TRANSFERTYPE_SWPOLL
		if(sdhc_waitCommandReady())
			result = sdhc_ReadBlock(pData,count,SDHC_BLOCK_SIZE);
		else
			result=RES_READERROR;
	#elif SDHC_TRANSFERTYPE == SDHC_TRANSFERTYPE_DMA
		result=RES_OK;
	#endif	
logg('F');
	// Auto CMD12 is enabled
	if((result != RES_OK) && (count>1))
  		result=sdhc_CMD12_StopTransferWaitForBusy();
	// wait for end of DMA
logg('G');
	sdhc_DMAWait();
	SDHC_IRQSTAT &= (SDHC_IRQSTAT_CC | SDHC_IRQSTAT_TC);
logg('H');

	return result; 
}

int SDHC_disk_write(const BYTE *buff, DWORD sector, UINT count)
{
	DRESULT result = RES_OK;
	LWord* pData = (LWord*)buff;

	// Check if this is ready
	if(sdCardDesc.status != 0)  return RES_NOTRDY;

	// Check the valid Count of block
	if(!count) return RES_PARERR; 

	// Convert LBA to UCHAR address if needed
	if(!sdCardDesc.highCapacity)  sector *= 512;

//	delayMicroseconds(100); // this is workaround to avoid sdhc blocking on BWEN
	m_sdhc_waitCmd13 = 1;
	uint32_t cnt = 1<<16; while ((--cnt) && sdhc_isBusy()) /* yield() */;  if(!cnt) return RES_WRITEERROR;
	m_sdhc_waitCmd13 = 0;

	while(SDHC_PRSSTAT & (SDHC_PRSSTAT_CIHB | SDHC_PRSSTAT_CDIHB | SDHC_PRSSTAT_DLA)) /* yield() */;

	SDHC_IRQSTAT = 0xffff; // clear interrupt status register
#if SDHC_TRANSFERTYPE == SDHC_TRANSFERTYPE_DMA
	SDHC_DSADDR  = (LWord)pData;  
#endif   
	SDHC_BLKATTR = SDHC_BLKATTR_BLKCNT(count) | SDHC_BLKATTR_BLKSIZE(SDHC_BLOCK_SIZE);
	#if defined(__IMXRT1052__)
		SDHC_MIX_CTRL &= ~ SDHC_MIX_CTRL_DTDSEL;  // write
		SDHC_MIX_CTRL |=  SDHC_MIX_CTRL_DMAEN ;   //DMA
	#endif
	sdhc_enableDma();
	// if multi-block write 
	// pre-erase blocks
	if(count>1)
	{
		sdhc_CMD(SDHC_CMD55_XFERTYP, 0);
		sdhc_CMD(SDHC_ACMD23_XFERTYP, count);

		SDHC_CMDARG = sector;
		SDHC_XFERTYP = SDHC_CMD25_XFERTYP;
	}
	else
	{
		SDHC_CMDARG = sector;
		SDHC_XFERTYP = SDHC_CMD24_XFERTYP;
	}

//	SDHC_CMDARG = sector;
//	SDHC_XFERTYP = (count==1) ? SDHC_CMD24_XFERTYP: SDHC_CMD25_XFERTYP;

#if SDHC_TRANSFERTYPE == SDHC_TRANSFERTYPE_SWPOLL
	if(sdhc_waitCommandReady())
		result = sdhc_WriteBlock(pData,count,SDHC_BLOCK_SIZE);
	else
		result=RES_WRITEERROR;
#elif SDHC_TRANSFERTYPE == SDHC_TRANSFERTYPE_DMA
		result=RES_OK;
#endif	
	// Auto CMD12 is enabled for DMA
	if((result != RES_OK) && (count>1))
		result=sdhc_CMD12_StopTransferWaitForBusy();

// wait for end of DMA
  sdhc_DMAWait();

	return result;
}

int SDHC_ioctl(BYTE cmd, BYTE *buff)
{
    return RES_OK;
}
#endif

#if defined(__MK20DX256__) // has no sdhc
  int SDHC_disk_status(){    return RES_OK;}
  int SDHC_disk_initialize(){    return RES_OK;}
  int SDHC_disk_read(BYTE *buff, DWORD sector, UINT count){    return RES_OK;}
  int SDHC_disk_write(const BYTE *buff, DWORD sector, UINT count){    return RES_OK;}
  int SDHC_ioctl(BYTE cmd, BYTE *buff){    return RES_OK;}
#endif
#endif
