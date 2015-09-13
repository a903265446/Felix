
#include "fsl_gpio_hal.h"
#include "fsl_port_hal.h"

#include "fsl_debug_console.h"
#include "fsl_device_registers.h"
#include "board.h"
#include "fsl_interrupt_manager.h"
#include "fsl_os_abstraction.h"

#include "test_sdspi_function.h"






GPIO_Type * const g_gpioBase[GPIO_INSTANCE_COUNT] = GPIO_BASE_PTRS;
PORT_Type * const g_portBase[PORT_INSTANCE_COUNT] = PORT_BASE_PTRS;
const IRQn_Type g_portIrqId[PORT_INSTANCE_COUNT] = PORT_IRQS;


void PORT_ISR(PORT_Type *base)
{
    /* Get interrupt flag */
    if (PORT_HAL_GetPortIntFlag(base) == (1 << SDCARD_CD_PIN))
    {
        card_detect_irq_handler();
    }
    
    /* Clear interrupt flag.*/
    PORT_HAL_ClearPortIntFlag(base);
}

void PORTA_IRQHandler(void)
{
    PORT_ISR(PORTA);
}

#if (PORT_INSTANCE_COUNT > 1U)
void PORTB_IRQHandler(void)
{
    PORT_ISR(PORTB);
}
#endif

#if (PORT_INSTANCE_COUNT > 2U)
void PORTC_IRQHandler(void)
{
    PORT_ISR(PORTC);
}
#endif

#if (PORT_INSTANCE_COUNT > 3U)
void PORTD_IRQHandler(void)
{
    PORT_ISR(PORTD);
}
#endif

#if (PORT_INSTANCE_COUNT > 4U)
void PORTE_IRQHandler(void)
{
    PORT_ISR(PORTE);
}
#endif

void init_sdspi_hardware(void)
{
    uint8_t i;
    /* Init hardware */
    hardware_init();
    
/* Set card detect pin mux */
#if defined FRDM_K22F
    GPIO_DRV_Init(sdcardCdPin, 0);
#endif
#if defined TWR_K22F120M
    GPIO_DRV_Init(sdcardCardDectionPin, 0);
#endif

    INT_SYS_EnableIRQ(g_portIrqId[SDCARD_CD_PORT]);
    
    /* Set SPI pin mux */
#if defined FRDM_K22F
    /* SPI0_CS0 */
    PORT_HAL_SetMuxMode(g_portBase[2],  4, kPortMuxAlt2);
    PORT_HAL_SetPullMode(g_portBase[2], 4, kPortPullUp);
    PORT_HAL_SetPullCmd(g_portBase[2],  4, true);
    /* SPI0_SCK */
    PORT_HAL_SetMuxMode(g_portBase[3], 1, kPortMuxAlt2);
    /* SPI0_SOUT */
    PORT_HAL_SetMuxMode(g_portBase[3], 2, kPortMuxAlt2);
    PORT_HAL_SetPullMode(g_portBase[3], 2, kPortPullUp);
    PORT_HAL_SetPullCmd(g_portBase[3],  2, true);
    /* SPI0_SIN */
    PORT_HAL_SetMuxMode(g_portBase[3], 3, kPortMuxAlt2);
    PORT_HAL_SetPullMode(g_portBase[3], 3, kPortPullUp);
    PORT_HAL_SetPullCmd(g_portBase[3],  3, true);
#endif
#if defined TWR_K22F120M
    configure_spi_pins(1);
#endif
    
    OSA_Init();

}