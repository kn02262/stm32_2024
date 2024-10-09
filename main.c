#include <stdint.h>
#include <stm32f10x.h>


int __attribute((noreturn)) main(void) {
	//RCC->APB2ENR |= 0b10000;
	// Включает тактирование PORTC, PORTB
	RCC->APB2ENR |= (RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPBEN);
	// Настройка PORTC, PC13 на выход, General purpose output push-pull
	GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13) | GPIO_CRH_MODE13_0; 
	// Настройка PORTB, PB9 на вход с подтягивающим резистором
	GPIOB->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9) | GPIO_CRH_CNF9_1;
	// Включение pull-up на PB9
	GPIOB->ODR |= GPIO_ODR_ODR9;
	// Запись логического 0 на выход PC13 (LED ON)
	GPIOC->ODR &= ~GPIO_ODR_ODR13;

    while (1) {
		if(GPIOB->IDR & GPIO_IDR_IDR9){ 
			GPIOC->ODR &= ~GPIO_ODR_ODR13; // LED ON
		} else { // Button is pressed
			GPIOC->ODR |= GPIO_ODR_ODR13; // LED OFF
		}
    }
	
}
