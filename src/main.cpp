#include "stm32l1xx.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>

#define LED_PIN 5

void vApplicationIdleHook(void ) { } // See configUSE_IDLE_HOOK in FreeRTOSConfig.h
void vApplicationTickHook(void ) { } // See configUSE_TICK_HOOK	
void vApplicationMallocFailedHook(void ) { while(true); }
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName ) { while(true); }


void init_clock();
void uart_setup();

void uart_write(USART_TypeDef* uart, char c)
{
    uart->DR = c;
    while (!(uart->SR & USART_SR_TC));
}

static void task1(void* args)
{
    while (true)
    {
        // Toggle LED pin
        GPIOA->ODR ^= (1 << LED_PIN);

        uart_write(USART2, 'v');
        uart_write(USART2, 'a');
        uart_write(USART2, 'q');
        uart_write(USART2, 'o');
        uart_write(USART2, ' ');
        
        uart_write(USART2, '\n');

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

int main()
{
    init_clock();
    
    SystemCoreClockUpdate();


    // Use system clock for following ports
    RCC->AHBENR |= (1 << RCC_AHBENR_GPIOAEN_Pos);
    // Dummy reads as per errata
    volatile uint32_t dummy= RCC->AHBENR; dummy = RCC->AHBENR;

    // SysTick_Config(100000);
    // __enable_irq();
    
    uart_setup();

    GPIOA->MODER |= GPIO_MODER_MODER5_0; // enable LED

    xTaskCreate(task1, "LED", 100, nullptr, configMAX_PRIORITIES - 1, nullptr);
    vTaskStartScheduler();


    while(true);

    return 0;
}

void init_clock()
{
    {   /* Enable external clock located on ST-LINK */
        RCC->CR |= RCC_CR_HSEBYP_Msk; // Use external clock fromOSC_IN instead of crystal
        RCC->CR |= RCC_CR_HSEON_Msk;  // Enable high-speed external oscilator (HSE)
        while (!(RCC->CR & RCC_CR_HSERDY_Msk)); // Wait till HSE is stable
    }

    {   /* Set higher voltage to drive CPU at max frequency of 32 MHz */
        RCC->APB1ENR |= RCC_APB1ENR_PWREN_Msk; // Enable PWR register
        volatile uint32_t dummy = RCC->APB1ENR; dummy = RCC->APB1ENR; // dummy reads, per errata

        MODIFY_REG(PWR->CR, PWR_CR_VOS_Msk, PWR_CR_VOS_0); // Voltage scaling range 1: 1.8 V

        while ((PWR->CSR & PWR_CSR_VOSF_Msk));
    }

    {   /* Set number of wait states (WS) according to CPU clock (HCLK) frequency [Table 13. Reference] */
        FLASH->ACR |= FLASH_ACR_ACC64_Msk;   // Enable 64 bit access
        FLASH->ACR |= FLASH_ACR_PRFTEN_Msk;  // Enable prefetch
        FLASH->ACR |= FLASH_ACR_LATENCY_Msk; // 1 wait state for memory access
    }

    {   /* Configure phase-locked loop (PLL) to get desired frequency for SYSCLK */
        RCC->CFGR |= RCC_CFGR_PLLSRC_HSE; // set HSE as a PLL source
        // set PLL multiplication factor
        RCC->CFGR |= RCC_CFGR_PLLMUL8;
        // set PLL division factor
        RCC->CFGR |= RCC_CFGR_PLLDIV2;
    }   /* Output frequency for SYSCLK = (f_HSE * PLLMUL / PLLDIV) */

    {
        RCC->CR |= RCC_CR_PLLON_Msk; // enable PLL
        while (!(RCC->CR & RCC_CR_PLLRDY_Msk)); // wait till ready

        MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_Msk, RCC_CFGR_SW_PLL);

        while (!(RCC->CFGR & RCC_CFGR_SWS_PLL)); // wait till system clock is set
    }
}

void uart_setup()
{
    volatile uint32_t dummy;

    /* Enable USART2 clock */
    // RCC->APB1ENR |= (1 << RCC_APB1ENR_USART2EN_Pos);
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    // do two dummy reads after enabling the peripheral clock, as per the errata
    dummy = RCC->APB1ENR; dummy = RCC->APB1ENR;

    /* Enable GPIOA clock*/
    RCC->AHBENR |= (1 << RCC_AHBENR_GPIOAEN);
    // do two dummy reads after enabling the peripheral clock, as per the errata
    dummy = RCC->AHBENR; dummy = RCC->AHBENR;

      /* Set PA2 and PA3 to alternate function */
    GPIOA->MODER &= ~(GPIO_MODER_MODER2_Msk | GPIO_MODER_MODER3_Msk);
    GPIOA->MODER |= (0b10 << GPIO_MODER_MODER2_Pos) | (0b10 << GPIO_MODER_MODER3_Pos);

    // USART2 is AF7 (found in datasheet)
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2_Msk | GPIO_AFRL_AFSEL3_Msk);
    GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL2_Pos) | (7 << GPIO_AFRL_AFSEL3_Pos);

    /* Configure and enable USART2 */
    USART2->BRR = 278; // 115200 baud @ 32 MHz APB1 clock and 16x oversampling
    USART2->CR1 |= (USART_CR1_UE | USART_CR1_TE); // USART enable and transmitter enable

    // Dummy write, because the first byte seems to always be dropped
    USART2->DR = 0;
    while (!(USART2->SR & USART_SR_TC));
}