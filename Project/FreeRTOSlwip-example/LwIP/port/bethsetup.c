/*
 * bethsetup.c
 *
 *  Created on: 26.08.2012
 *      Author: diego
 */

#include <stm32f2xx_hal_eth.h>
#include <stm32f2xx_hal_rcc.h>
#include <stm32f2xx_hal_gpio.h>
#include <stm32f2xx_hal_gpio_ex.h>
#include <misc.h>

#include <FreeRTOSConfig.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "beth.h"

#include <lwip/err.h>

/// public interface
#define  ETH_DMARxDesc_FrameLengthShift           16
/* The number of descriptors to chain together for use by the Rx DMA. */
#define NUM_RX_DESCRIPTORS		2   ///  one descriptor for each buffer.
#define NUM_TX_DESCRIPTORS		1   ///  one descriptor for each buffer.
// Number of buffers /// descriptor are chained so a frame can expand two buffers
#define NUM_RX_BUFFERS			2   /// set two rx buffers (frame max 3000bytes)
#define NUM_TX_BUFFERS			1	/// set two tx buffers
/// ETH frame lenght
#define MAX_PACKET_SIZE			1520

/* Allocate the Rx and descriptors used by the DMA. */
static ETH_DMADescTypeDef xRxDescriptors[NUM_RX_DESCRIPTORS] __attribute__((aligned(4)));
static ETH_DMADescTypeDef xTxDescriptors[NUM_TX_DESCRIPTORS] __attribute__((aligned(4)));
/* Allocate separated buffers for receive and transmit data*/
static unsigned char rxMACBuffers[NUM_RX_BUFFERS][MAX_PACKET_SIZE] __attribute__((aligned(4)));
static unsigned char txMACBuffers[NUM_TX_BUFFERS][MAX_PACKET_SIZE] __attribute__((aligned(4)));

extern ETH_DMADescTypeDef *DMATxDescToSet;
extern ETH_DMADescTypeDef *DMARxDescToGet;

/// internal interface
static void beth_setup_gpio(void);
static portBASE_TYPE beth_init_eth(unsigned char* ucMACAddress);

/* If no buffers are available, then wait this long before looking again.... */
#define netifBUFFER_WAIT_DELAY					( 100 / portTICK_RATE_MS )

/* The semaphore used by the ISR to wake the lwIP task. */
xSemaphoreHandle ETH_RX_Sem = NULL;

#include <stm322xg_eval.h>
//#include <../../AppTemplate/src/hwsetup.h>

#define netifINIT_WAIT	( 100 / portTICK_RATE_MS )

err_t beth_initialize(struct netif *netif)
{
	STM_EVAL_LEDInit(LED3);
	/// Create the reception semaphore!.
	if (ETH_RX_Sem == NULL) {
		vSemaphoreCreateBinary( ETH_RX_Sem );
		xSemaphoreTake(ETH_RX_Sem, 1);
	}
	/// load macadress;
	unsigned char ucMACAddress[6];
	ucMACAddress[0] = netif->hwaddr[0];
	ucMACAddress[1] = netif->hwaddr[1];
	ucMACAddress[2] = netif->hwaddr[2];
	ucMACAddress[3] = netif->hwaddr[3];
	ucMACAddress[4] = netif->hwaddr[4];
	ucMACAddress[5] = netif->hwaddr[5];
	/* Initialize the MAC. */
	while (beth_init_eth(ucMACAddress) != pdPASS) {
		vTaskDelay(netifINIT_WAIT);
	}
	return ERR_OK;
}

err_t beth_wait_packet()
{
	/// wait for next packet. The ISR gives this
	/// semaphore when a new packet arrive.
	xSemaphoreTake( ETH_RX_Sem, portMAX_DELAY );
	return ERR_OK;
}

void ETH_IRQHandler()
{
	long xHigherPriorityTaskWoken = pdFALSE;
	/// give away taked semaphore. Used to inform the ethernet_input
	/// thread that new data is available
	/// give away only if frame received interrupt was triggered
	if(ETH_GetDMAFlagStatus(ETH_DMA_FLAG_R) == SET)
	{
		xSemaphoreGiveFromISR( ETH_RX_Sem, &xHigherPriorityTaskWoken );
		ETH_DMAClearITPendingBit(ETH_DMA_IT_R);
	}
	if(ETH_GetDMAFlagStatus(ETH_DMA_FLAG_T)==SET)
	{
		xHigherPriorityTaskWoken = pdFALSE;
	}
	if(ETH_GetDMAFlagStatus(ETH_DMA_FLAG_TBU)==SET)
	{
		xHigherPriorityTaskWoken = pdFALSE;
	}
	ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

/**
 * Configure the IO for Ethernet use.
 */
static void beth_setup_gpio(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ETHERNET pins configuration */
	/* AF Output Push Pull:
	- ETH_MII_MDIO / ETH_RMII_MDIO: PA2
	- ETH_MII_MDC / ETH_RMII_MDC: PC1
	- ETH_MII_TXD2: PC2
	- ETH_MII_TX_EN / ETH_RMII_TX_EN: PB11
	- ETH_MII_TXD0 / ETH_RMII_TXD0: PB12
	- ETH_MII_TXD1 / ETH_RMII_TXD1: PB13
	- ETH_MII_PPS_OUT / ETH_RMII_PPS_OUT: PB5
	- ETH_MII_TXD3: PB8 */

	/* Configure PA2 as alternate function push-pull */
	GPIO_InitStructure.Pin = GPIO_PIN_2;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure PC1, PC2 and PC3 as alternate function push-pull */
	GPIO_InitStructure.Pin = GPIO_PIN_1;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure PB5, PB8, PB11, PB12 and PB13 as alternate function push-pull */
	GPIO_InitStructure.Pin =  GPIO_PIN_11 |
	GPIO_PIN_12 | GPIO_PIN_13;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/**************************************************************/
	/*               For Remapped Ethernet pins                   */
	/*************************************************************/
	/* Input (Reset Value):
	- ETH_MII_CRS CRS: PA0
	- ETH_MII_RX_CLK / ETH_RMII_REF_CLK: PA1
	- ETH_MII_COL: PA3
	- ETH_MII_RX_DV / ETH_RMII_CRS_DV: PD8
	- ETH_MII_TX_CLK: PC3
	- ETH_MII_RXD0 / ETH_RMII_RXD0: PD9
	- ETH_MII_RXD1 / ETH_RMII_RXD1: PD10
	- ETH_MII_RXD2: PD11
	- ETH_MII_RXD3: PD12
	- ETH_MII_RX_ER: PB10 */

	/* ETHERNET pins remapp in STM3210C-EVAL board: RX_DV and RxD[3:0] */
	GPIO_PinRemapConfig(GPIO_AF11_ETH, DISABLE);

	/* Configure PA0, PA1 and PA3 as input */
	GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_7 ;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure PB10 as input */
	GPIO_InitStructure.Pin = GPIO_PIN_10;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure PC3 as input */
	GPIO_InitStructure.Pin = GPIO_PIN_3;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure PD8, PD9, PD10, PD11 and PD12 as input */
	GPIO_InitStructure.Pin = GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_Init(GPIOC, &GPIO_InitStructure); /**/

	/* MCO pin configuration------------------------------------------------- */
	/* Configure MCO (PA8) as alternate function push-pull */
	GPIO_InitStructure.Pin = GPIO_PIN_8;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
 * Initialize all IO and peripherals required for Ethernet communication.
 *
 * @return pdSUCCESS if success, pdFAIL to signal some error.
 */
portBASE_TYPE beth_init_eth(unsigned char* ucMACAddress) {
	static ETH_InitTypeDef xEthInit; /* Static so as not to take up too much stack space. */
	NVIC_InitTypeDef xNVICInit;
	portBASE_TYPE xReturn;
	unsigned long ul;

	/* Enable ETHERNET clock  */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ETH_MAC | RCC_AHBPeriph_ETH_MAC_Tx |
					RCC_AHBPeriph_ETH_MAC_Rx, ENABLE);
	/* Use MII mode. */
	GPIO_ETH_MediaInterfaceConfig(GPIO_ETH_MediaInterface_RMII);
	/* Enable GPIOs clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC |
					RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE| RCC_APB2Periph_AFIO, ENABLE);
	/* Get HSE clock = 25MHz on PA8 pin(MCO) */
	beth_setup_gpio();
	/* set PLL3 clock output to 50MHz (25MHz /5 *10 =50MHz) */
	RCC_PLL3Config(RCC_PLL3Mul_10);
	/* Enable PLL3 */
	RCC_PLL3Cmd(ENABLE);
	/* Wait till PLL3 is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_PLL3RDY) == RESET);
	/* Get clock PLL3 clock on PA8 pin */
	RCC_MCOConfig(RCC_MCO_PLL3CLK);
	/* Start with things in a safe known state. */
	ETH_DeInit();
	/* Reset the peripheral. */
	ETH_SoftwareReset();
	while (ETH_GetSoftwareResetStatus() == SET);
	/* Set the MAC address. */
	ETH_MACAddressConfig(ETH_MAC_Address0, (unsigned char *) ucMACAddress);
	/* Initialise using the whopping big structure.  Code space could be saved
	 by making this a const struct, however that would mean changes to the
	 structure within the library header files could break the code, so for now
	 just set everything manually at run time. */
	ETH_StructInit(&xEthInit);
	xEthInit.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
	//xEthInit.Watchdog = ETH_WATCHDOG_DISABLE;
	//xEthInit.Jabber = ETH_JABBER_DISABLE;
	//xEthInit.ReceiveOwn = ETH_RECEIVEOWN_DISABLE;
	//xEthInit.RetryTransmission = ETH_RETRYTRANSMISSION_DISABLE;
	//xEthInit.ReceiveAll = ETH_RECEIVEALL_ENABLE;
	//xEthInit.PassControlFrames
	//		= ETH_PASSCONTROLFRAMES_FORWARDPASSEDADDRFILTER;

	//xEthInit.TxDMABurstLength = ETH_TXDMABURSTLENGTH_32BEAT;
	//xEthInit.RxDMABurstLength = ETH_RXDMABURSTLENGTH_32BEAT;
	//xEthInit.DMAArbitration = ETH_DMAARBITRATION_ROUNDROBIN_RXTX_2_1;
	//xEthInit.DropTCPIPChecksumErrorFrame = ETH_DROPTCPIPCHECKSUMERRORFRAME_ENABLE;
	//xEthInit.ReceiveStoreForward = ETH_RECEIVESTOREFORWARD_ENABLE;
	//xEthInit.TransmitStoreForward = ETH_TRANSMITSTOREFORWARD_ENABLE;
	xReturn = ETH_Init(&xEthInit, PHY_ADDRESS);
	/* Check a link was established. */
	if (xReturn != pdFAIL) {
		/* Rx and Tx interrupts are used. */
		ETH_DMAITConfig(ETH_DMA_IT_NIS | ETH_DMA_IT_R | ETH_DMA_IT_T, ENABLE);
		/* Only a single Tx descriptor is used.  For now it is set to use an Rx
		 buffer, but will get updated to point to where ever s_lwip_buf is
		 pointing prior to its use. */
		ETH_DMATxDescChainInit(xTxDescriptors, (void *) txMACBuffers, NUM_TX_DESCRIPTORS);
		ETH_DMARxDescChainInit(xRxDescriptors, (void *) rxMACBuffers, NUM_RX_DESCRIPTORS);
		for (ul = 0; ul < NUM_RX_DESCRIPTORS; ul++) {
			/* Ensure received data generates an interrupt. */
			ETH_DMARxDescReceiveITConfig(&(xRxDescriptors[ul]), ENABLE);
		}
		/* SendCount must be initialized to 2 to ensure the Tx descriptor looks
		 as if its available (as if it has already been sent twice. */
		//xTxDescriptor.SendCount = 2;
		// TODO: Enable hardware checksums
		/* Switch on the interrupts in the NVIC. */
		xNVICInit.NVIC_IRQChannel = ETH_IRQn;
//		xNVICInit.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
//		xNVICInit.NVIC_IRQChannelSubPriority = 0;
		xNVICInit.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&xNVICInit);
		/* Let the DMA know there are Rx descriptors available. */
		//ETH->DMARPDR = 0;
		ETH_Start(); /// Start eth hardware.
	}
	return xReturn;
}

u32_t ETH_GetCurrentTxBuffer(void)
{
	/// return buffer address
	return DMATxDescToSet->Buffer1Addr;
}

err_t ETH_TxPkt_ChainMode(u16_t FrameLength)
{
	  /* Check if the descriptor is owned by the ETHERNET DMA (when set) or CPU (when reset) */
	  //while((DMATxDescToSet->Status & ETH_DMATxDesc_OWN) != (u32)RESET);
	 if((DMATxDescToSet->Status & ETH_DMATXDESC_OWN) != (u32_t)RESET)
		 return ERR_USE;

	  /* Setting the Frame Length: bits[12:0] */
	  DMATxDescToSet->ControlBufferSize = (FrameLength & ETH_DMATXDESC_TBS1);

	  /* Setting the last segment and first segment bits (in this case a frame is transmitted in one descriptor) */
	  DMATxDescToSet->Status |= ETH_DMATXDESC_LS | ETH_DMATXDESC_FS;

	  /* Set Own bit of the Tx descriptor Status: gives the buffer back to ETHERNET DMA */
	  DMATxDescToSet->Status |= ETH_DMATXDESC_OWN;

	  /* When Tx Buffer unavailable flag is set: clear it and resume transmission */
	  if ((ETH->DMASR & ETH_DMASR_TBUS) != (u32_t)RESET)
	  {
	    /* Clear TBUS ETHERNET DMA flag */
	    ETH->DMASR = ETH_DMASR_TBUS;
	    /* Resume DMA transmission*/
	    ETH->DMATPDR = 0;
	  }
	  /* Update the ETHERNET DMA global Tx descriptor with next Tx decriptor */
	  /* Chained Mode */
	  /* Selects the next DMA Tx descriptor list for next buffer to send */
	  DMATxDescToSet = (ETH_DMADescTypeDef*) (DMATxDescToSet->Buffer2NextDescAddr);

	  /* Return SUCCESS */
	  return ERR_OK;
}


FrameTypeDef ETH_RxPkt_ChainMode(void)
{
  uint32_t framelength = 0;
  FrameTypeDef frame = {0,0};

  /* Check if the descriptor is owned by the ETHERNET DMA (when set) or CPU (when reset) */
  if((DMARxDescToGet->Status & ETH_DMARXDESC_OWN) != (uint32_t)RESET)
  {
	frame.length = ERR_MEM;

    if ((ETH->DMASR & ETH_DMASR_RBUS) != (uint32_t)RESET)
    {
      /* Clear RBUS ETHERNET DMA flag */
      ETH->DMASR = ETH_DMASR_RBUS;
      /* Resume DMA reception */
      ETH->DMARPDR = 0;
    }

	/* Return error: OWN bit set */
    return frame;
  }

  if(((DMARxDescToGet->Status & ETH_DMARXDESC_ES) == (uint32_t)RESET) &&
     ((DMARxDescToGet->Status & ETH_DMARXDESC_LS) != (uint32_t)RESET) &&
     ((DMARxDescToGet->Status & ETH_DMARXDESC_FS) != (uint32_t)RESET))
  {
    /* Get the Frame Length of the received packet: substruct 4 bytes of the CRC */
    framelength = ((DMARxDescToGet->Status & ETH_DMARXDESC_FL) >> ETH_DMARxDesc_FrameLengthShift) - 4;

	/* Get the addrees of the actual buffer */
	frame.buffer = DMARxDescToGet->Buffer1Addr;
  }
  else
  {
    /* Return ERROR */
    framelength = ERR_MEM;
  }
  frame.length = framelength;
  frame.descriptor = DMARxDescToGet;

  /* Update the ETHERNET DMA global Rx descriptor with next Rx decriptor */
  /* Chained Mode */
  /* Selects the next DMA Rx descriptor list for next buffer to read */
  DMARxDescToGet = (ETH_DMADescTypeDef*) (DMARxDescToGet->Buffer2NextDescAddr);
  /* Return Frame */
  return (frame);
}
