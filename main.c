#include <stdint.h>
#include <stm32f10x.h>

void SPI1_Init(){
	// Включить тактирование PORTA и SPI1
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_SPI1EN;

	// PA5 Alternate function Output
	GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5);
	GPIOA->CRL |= GPIO_CRL_MODE5_0; // Mode='01' Output 10Mhz
	GPIOA->CRL |= GPIO_CRL_CNF5_1; // Alt function Push-pull

	// PA7 Alternate function Output
	GPIOA->CRL &= ~(GPIO_CRL_CNF7 | GPIO_CRL_MODE7);
	GPIOA->CRL |= GPIO_CRL_MODE7_0; // Mode='01' Output 10Mhz
	GPIOA->CRL |= GPIO_CRL_CNF7_1; // Alt function Push-pull

	// PA6 Alternate function Input
	GPIOA->CRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_MODE6);
	GPIOA->CRL |= GPIO_CRL_CNF6_1; // Input with pull-up

	// SPI1_BR = '011' Freq=F/16
	SPI1->CR1 &= ~ SPI_CR1_BR;
	SPI1->CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_1;

	SPI1->CR1 |= SPI_CR1_MSTR; // Master mode
	SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI; // Software CS, Chip select = ON
	SPI1->CR1 |= SPI_CR1_CPOL; // CLK=1 when idle
	SPI1->CR1 |= SPI_CR1_CPHA; // Falling edge
	SPI1->CR1 |= SPI_CR1_LSBFIRST; // Low significant bit first
	SPI1->CR1 &= ~SPI_CR1_DFF; // 8-bit data Frame

	// Включить SPI
	SPI1->CR1 |= SPI_CR1_SPE;
}

void SPI1_Send(uint8_t data){
	while( SPI1->SR & SPI_SR_BSY); 
	while( (SPI1->SR & SPI_SR_TXE) == 0);
	SPI1->DR = data;
	return;
}

uint8_t SPI1_Read(){
	while( (SPI1->SR & SPI_SR_RXNE) == 0 );
	return SPI1->DR;
}



int __attribute((noreturn)) main(void) {

	//RCC->APB2ENR |= 0b10000;
	// Включает тактирование PORTC, PORTB
	RCC->APB2ENR |= (RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPBEN);
	// Настройка PORTC, PC13 на выход, General purpose output push-pull
	GPIOC->CRH = ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13) | GPIO_CRH_MODE13_0; 
	// Настройка PORTB, PB9 на вход с подтягивающим резистором
	GPIOB->CRH = ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9) | GPIO_CRH_CNF9_1;
	// Включение pull-up на PB9
	GPIOB->ODR |= GPIO_ODR_ODR9;
	// Запись логического 0 на выход PC13 (LED ON)
	GPIOC->ODR &= ~GPIO_ODR_ODR13;
	// Инициализация SPI1
	SPI1_Init();
	uint8_t var = 32;
	SPI1_Send(var);
	uint8_t var2 = SPI1_Read();

    while (var == var2) {
		if(GPIOB->IDR & GPIO_IDR_IDR9){ 
			GPIOC->ODR &= ~GPIO_ODR_ODR13; // LED ON
		} else { // Button is pressed
			GPIOC->ODR |= GPIO_ODR_ODR13; // LED OFF
		}
    }
	
}
