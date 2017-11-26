//#include "I2C.h"
#include <stdio.h>
#include <stdlib.h>


GPIO_TypeDef * gpiob = GPIOB;
I2C_TypeDef * i2c = I2C1;
RCC_TypeDef * rcc = RCC;
USART_TypeDef * uart = USART2;


void main(void){
	uint8_t HTS221_Found = 0;
	float humi=0;
	float temp=0;
	uart_Init();
	I2C_Init();
	HTS221_Found = HTS221_Init(); //Initializes the device if found
	if(HTS221_Found){
			printf("#####  HTS221 Found  #####\r\n");
		}
	else printf("#####  HTS221 Not Connected  #####\r\n");
	HTS221_Configuration();
	
	humi=HTS221_Humidity_Read();
	temp=HTS221_Temp_Read();
	UART_Transmit(USART2,humi);
	UART_Transmit(USART2,temp);
	printf("#####  Temperature is  %f #####\r\n",temp);
	printf("#####  Humidity is  %f #####\r\n",humi);
	
}




void uart_Init(void)
{

	/*-------------- clock enable --------------- */
	/* Enable gpiob clock */
	rcc->AHB1ENR |= RCC_AHB1ENR_gpiobEN;
	/* Enable USARTx clock */
	rcc->APB1ENR |= RCC_APB1ENR_USART2EN;
	/* enable DMA1 clock */
	rcc->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

	/* reset USART2 */
	rcc->APB1RSTR |= RCC_APB1RSTR_USART2RST;
	rcc->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;

	/*-------------- Pin muxing configuration --------------- */
	/* TX on PA2 alternate function 7 */
	gpiob->AFR[0] &= ~(0xF << (2 *4) );	/* clear the 4 bits */
	gpiob->AFR[0] |= (7 << (2*4) ); 	/* set alternate function */
	/* RX on PA3 alternate function 7 */
	gpiob->AFR[0] &= ~(0xF << (3*4) );	/* clear the 4 bits */
	gpiob->AFR[0] |= (7 << (3*4) );		/* set alternate function */

	/* Configure alternate function for PA2 and PA3*/
	gpiob->MODER &= ~GPIO_MODER_MODER3;
	gpiob->MODER &= ~GPIO_MODER_MODER2;
	gpiob->MODER |= GPIO_MODER_MODER3_1;
	gpiob->MODER |= GPIO_MODER_MODER2_1;

	/*-------------- UART parameters configuration --------------- */
	/* USART CR1 Configuration */
	uart->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_OVER8;	/* tx and rx enable; oversampling = 8 */
	/* USART CR2 Configuration */
	uart->CR2 = 0 ; /* 1 stop bit */
	/* USART CR3 Configuration */
	uart->CR3 = 0; /* no flow control */
	/* USART BRR Configuration */
	uart->BRR = 0x2D5; /*45,625 with fpclk= 42E6*/
	/* enable USART */
	uart->CR1 |= USART_CR1_UE;

	/*-------------- variable intialisation --------------- */
	uart.state &= ~(TX_LOCK | RX_LOCK);
}




uint32_t UART_Transmit(USART_TypeDef * uart, float data)
{
	/* test the lock */
	if ((uart.state & TX_LOCK) == TX_LOCK)
		return -1;
	/* get the lock*/
	uart.state &= TX_LOCK;

	
	//while (len-->0){
		/* wait for TX data register to be empty */ 
		while (!(uart->SR & USART_SR_TXE)){
		}
		uart->DR = data;
	//}	
	/* wait last char to be finished (optionnal) */
	//while (!(uart->SR & USART_SR_TC));

	/*release the lock */
	uart.state &= TX_LOCK;

	return len;
}

uint32_t UART_Receive(USART_TypeDef * uart, uint8_t * data, uint32_t len, uint32_t timeout)
{
	uint32_t dec;

	/* test the lock */
	if ((uart.state & RX_LOCK) == RX_LOCK)
		return -1;
	/* get the lock*/
	uart.state &= RX_LOCK;

	while (len > 0){
		dec = timeout;
		/* wait for a new char */
		while (!(uart->SR & USART_SR_RXNE)) {
				if (timeout-- == 0)
					return len;
		}
		/* get the data */
		*data++ = (uart->DR & 0xFF);
		len--;
	}

	/*release the lock */
	uart.state &= RX_LOCK;

	return len;
}

void I2C_Init(void){
	
/*-------------- clock enable --------------- */
	/* Enable i2c clock */
	rcc->APB1ENR |= RCC_APB1ENR_i2cEN;
	/* Enable GPIOB clock */
	rcc->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	/* Configure alternate function for I2C */
	gpiob->MODER &= ~GPIO_MODER_MODER9;
	gpiob->MODER &= ~GPIO_MODER_MODER8;
	gpiob->MODER |= GPIO_MODER_MODER9_1;
	gpiob->MODER |= GPIO_MODER_MODER8_1;
/* sda on PB8 and scl on PB9 alternate function 4 */
	gpiob->AFR[1] &= 0xFF ; /* clear */
	gpiob->AFR[1] |= (0x4<<9); /* set alternate function */
	/* open drain with no pull up no pull down */
	gpiob->PUPDR |= GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9;
	gpiob->OTYPER |= GPIO_OTYPER_ODR_8;
	gpiob->OTYPER |= GPIO_OTYPER_ODR_9;

	/*	disable i2c peripheral*/
	i2c->CR1 &=~I2C_CR1_PE;
	
	/*configure peripheral input clock at 42mhz*/
	i2c->CR2=I2C_CR2_FREQ_1|I2C_CR2_FREQ_3|I2C_CR2_FREQ_5;

	/*configure the rise time register*/
	i2c->TRISE_TRISE|=I2C_TRISE_TRISE;
	
	
	/*setting bus in i2c mode
	i2c->CR1_SMBUS&=I2C_CR1_SMBUS;*/

	
	/*	Enable i2c peripheral*/
	i2c->CR1 |= I2C_CR1_PE;
	
}



/*Generate Start Bit*/
void i2c_start()
{
i2c->CR1 |= I2C_CR1_START; //I2C Start
while (!(i2c->SR1 & I2C_SR1_SB)); //wait for start to be sent
}
/*Generate Stop Bit*/
void i2c_stop()
{
I2C1->CR1 |= I2C_CR1_STOP;
while (I2C1->SR2 & I2C_SR2_BUSY); //wait for BUSY set see status register
}


uint16_t I2C_Read_Reg(uint16_t address){
	
	//Reset CR2 Register
	//i2c->CR2 = 0x00000000;
	i2c_start();
	
	//Check to see if the bus is busy
	while((i2c->SR2 & I2C_SR2_BUSY) == I2C_SR2_BUSY);
	
	
	//Start communication
	i2c_start();
	
	//Check Tx empty before writing to it
	if((i2c->SR1 & I2C_SR1_TXE) == (I2C_SR1_TXE)){
		i2c->DR = address;
	}
	
	//Wait for transfer to complete
	while((i2c->SR2 & I2C_SR2_TRA) == 0);
	//recepton of acknowledge pulse
	while(i2c->SR1 & I2C_SR1_TXE) == (I2C_SR1_TXE));

	/*Clear CR2 for new configuration
	i2c->CR2 = 0x00000000;
	
	//Set CR2 for 1-byte transfer, in read mode for Device
	i2c->CR2 |= (1UL<<16) | I2C_CR2_RD_WRN | (Device<<1);*/
	
	//Start communication
	i2c_start();
	
	
	//Wait for transfer to complete
	
	while(((i2c->SR1 & I2C_SR1_BTF) == 0) &&(!(i2c->SR1 & I2C_SR1_RXNE)) )

	i2c_stop();
	
	return(i2c->DR);
	
}

/**
  \fn					uint32_t I2C_Read_Reg(uint32_t Register)
  \brief			Reads a register, the entire sequence to read (at least for HTS221)
	\param			uint32_t Register: The register that you would like to write to
	\param			uint32_t Data: The data that you would like to write to the register
*/

void I2C_Write_Reg(uint32_t Device,uint32_t 4, uint32_t Data)
{
	
	
	//Check to see if the bus is busy
	while((i2c->SR2 & I2C_SR2_BUSY) == I2C_SR2_BUSY);
	
		i2c_start();

	//Check Tx empty before writing to it
	if((i2c->SR1 & I2C_SR1_TXE) == (I2C_SR1_TXE)){
		i2c->DR = address;
	}
	
	//Wait for TX Register to clear
	while((i2c->SR1 & I2C_SR1_TXE) ==0);
	
	
	//Check Tx empty before writing to it
	if((i2c->SR1 & I2C_SR1_TXE) == (I2C_SR1_TXE)){
		i2c->DR = Data;
	}
	
	//Wait for transfer to complete( data bytes transmitted)
	while((i2c->SR2 & I2C_SR2_TRA));

	//Send Stop Condition
	i2c_stop();
}



uint8_t HTS221_Init(void){
	
	//Local variables
	uint8_t Device_Found = 0;
	uint32_t AV_CONF_Init = 0x1B;		/*16 Temp (AVGT) and 32 Hum (AVGT)*/

	//Read data from register and check signature	
	I2C_Read_Reg(HTS221_WHO_AM_I);
	
	//Check if device signature is correct
	if (i2c->DR == HTS221_DEVICE_ID){
		Device_Found = 1;
	}
	else Device_Found = 0;
	
	/* Setup HTS221_AV_CONF Register */
	if(Device_Found){
		//Set to Default Configuration
		I2C_Write_Reg(HTS221_AV_CONF,AV_CONF_Init);
		
		//Activate and Block Data Update, this will ensure that both the higher and lower bits are read
		I2C_Write_Reg(HTS221_CTRL_REG1,(HTS221_CTRL_REG1_PD | HTS221_CTRL_REG1_BDU));
	}
	
	return(Device_Found);
}


void HTS221_Configuration(void){
	
	printf("----------------Configuration Settings-------------------\r\n");	
	//HTS221_AV_CONF Settings
	I2C_Read_Reg(HTS221_AV_CONF);
	printf("HTS221_AV_CONF: %x\r\n",i2c->DR);
	
	//HTS221_CTRL_REG1 Settings
	I2C_Read_Reg(HTS221_CTRL_REG1);
	printf("HTS221_CTRL_REG1: %x\r\n",i2c->DR);
	
	printf("---------------------------------------------------------\r\n");
}

float HTS221_Temp_Read(void){
	
	/* Local Variables */
	uint8_t STATUS_REG = 0;
	
	//T0_degC and T1_degC
	uint16_t T0_degC_x8 = 0;
	uint16_t T1_degC_x8 = 0;
	uint16_t Msb_TO_T1_degC = 0;
	float T0_DegC = 0;
	float T1_DegC = 0;
	
	//T_OUT
	uint16_t T_OUT_L = 0;
	uint16_t T_OUT_H = 0;
	float T_OUT = 0;
	
	//T0_OUT and T1_OUT
	int16_t T0_OUT_L = 0;
	int16_t T0_OUT_H = 0;
	int16_t T1_OUT_L = 0;
	int16_t T1_OUT_H = 0;
	float T0_OUT = 0;
	float T1_OUT = 0;
	
	//Temperature Variables
	float Temperature_In_C = 0;
	float Temperature_In_F = 0;
	
	//Start a temperature conversion
	I2C_Write_Reg(HTS221_CTRL_REG2,HTS221_CTRL_REG2_ONE_SHOT);
	
	//Wait for Temperature data to be ready
	do{
		I2C_Read_Reg(HTS221_STATUS_REG);
		STATUS_REG = i2c->DR;
	}while((STATUS_REG & HTS221_STATUS_REG_TDA) == 0);
	
	//Read Temperature Data and Calibration
	I2C_Read_Reg(HTS221_TEMP_OUT_L);
	T_OUT_L = i2c->DR;
	
	I2C_Read_Reg(HTS221_TEMP_OUT_H);
	T_OUT_H = i2c->DR;
	
	I2C_Read_Reg(HTS221_TO_OUT_L);
	T0_OUT_L = i2c->DR;
	
	I2C_Read_Reg(HTS221_T0_OUT_H);
	T0_OUT_H = i2c->DR;
	
	I2C_Read_Reg(HTS221_T1_OUT_L);
	T1_OUT_L = i2c->DR;
	
	I2C_Read_Reg(HTS221_T1_OUT_H);
	T1_OUT_H = i2c->DR;
	
	I2C_Read_Reg(HTS221_T0_degC_x8);
	T0_degC_x8 = i2c->DR;
	
	I2C_Read_Reg(HTS221_T1_degC_x8);
	T1_degC_x8 = i2c->DR;
	
	I2C_Read_Reg(HTS221_T1_T0_Msb);
	Msb_TO_T1_degC = i2c->DR;
	
	//Process Calibration Registers
	T0_DegC = ((float)(((Msb_TO_T1_degC & 0x3) << 8) | (T0_degC_x8))/8.0);
	T1_DegC = ((float)(((Msb_TO_T1_degC & 0xC) << 6) | (T1_degC_x8))/8.0); //Value in 3rd and 4th bit so only shift 6
	T0_OUT = (float)((T0_OUT_H << 8) | T0_OUT_L);
	T1_OUT = (float)((T1_OUT_H << 8) | T1_OUT_L);
	T_OUT = (float)((T_OUT_H << 8) | T_OUT_L);
	
	//Calculate Temperatuer using linear interpolation and convert to Fahrenheit
	Temperature_In_C = (float)(T0_DegC + ((T_OUT - T0_OUT)*(T1_DegC - T0_DegC))/(T1_OUT - T0_OUT));
	Temperature_In_F = (Temperature_In_C*(9.0/5.0)) +32.0;
	
	return(Temperature_In_F);
}


float HTS221_Humidity_Read(void){
	
	/* Local Variables */
	uint8_t STATUS_REG = 0;
	
	//H0_rH and H1_rH
	uint8_t H0_rH_x2 = 0;
	float H0_rH = 0;
	uint8_t H1_rH_x2 = 0;
	float H1_rH = 0;

	//H_OUT
	float H_OUT = 0;
	uint16_t H_OUT_L = 0;
	uint16_t H_OUT_H = 0;

	//H0_TO_OUT and H1_TO_OUT
	float H0_T0_OUT = 0;
	float H1_T0_OUT = 0;
	uint16_t H0_T0_OUT_L = 0;
	uint16_t H0_T0_OUT_H = 0;
	uint16_t H1_T0_OUT_L = 0;
	uint16_t H1_T0_OUT_H = 0;

	//Humidity Variables
	float Humidity_rH = 0;
	
	//Start a humidity conversion
	I2C_Write_Reg(HTS221_CTRL_REG2,HTS221_CTRL_REG2_ONE_SHOT);
	
	//Wait for Humidity data to be ready
	do{
		I2C_Read_Reg(HTS221_STATUS_REG);
		STATUS_REG = i2c->DR;
	}while((STATUS_REG & HTS221_STATUS_REG_HDA) == 0);
	
	//Read Humidity data and Calibration
	I2C_Read_Reg(HTS221_H0_rH_x2);
	H0_rH_x2 = i2c->DR;
	
	I2C_Read_Reg(HTS221_H1_rH_x2);
	H1_rH_x2 = i2c->DR;
	
	I2C_Read_Reg(HTS221_HUMIDITY_OUT_L);
	H_OUT_L = i2c->DR;
	
	I2C_Read_Reg(HTS221_HUMIDITY_OUT_H);
	H_OUT_H = i2c->DR;
	
	I2C_Read_Reg(HTS221_H0_T0_OUT_L);
	H0_T0_OUT_L = i2c->DR;
	
	I2C_Read_Reg(HTS221_H0_T0_OUT_H);
	H0_T0_OUT_H = i2c->DR;
	
	I2C_Read_Reg(HTS221_H1_T0_OUT_L);
	H1_T0_OUT_L = i2c->DR;
	
	I2C_Read_Reg(HTS221_H1_T0_OUT_H);
	H1_T0_OUT_H = i2c->DR;
	
	//Process Calibration Registers
	H0_rH = (float)H0_rH_x2/2.0;
	H1_rH = (float)H1_rH_x2/2.0;
	H_OUT = (float)((H_OUT_H << 8) | H_OUT_L);
	H0_T0_OUT = (float)((H0_T0_OUT_H << 8) | H0_T0_OUT_L);
	H1_T0_OUT = (float)((H1_T0_OUT_H << 8) | H1_T0_OUT_L);
	
	//Calculate the relative Humidity using linear interpolation
	Humidity_rH = ( float )(((( H_OUT - H0_T0_OUT ) * ( H1_rH - H0_rH )) / ( H1_T0_OUT - H0_T0_OUT )) + H0_rH );
	
	return(Humidity_rH);
}	




void main(void){
	uint8_t HTS221_Found = 0;
	float humi=0;
	float temp=0;
	uart_Init();
	I2C_Init();
	HTS221_Found = HTS221_Init(); //Initializes the device if found
	if(HTS221_Found){
			printf("#####  HTS221 Found  #####\r\n");
		}
	else printf("#####  HTS221 Not Connected  #####\r\n");
	HTS221_Configuration();
	
	humi=HTS221_Humidity_Read();
	temp=HTS221_Temp_Read();
	UART_Transmit(USART2,humi);
	UART_Transmit(USART2,temp);
	printf("#####  Temperature is  %f #####\r\n",temp);
	printf("#####  Humidity is  %f #####\r\n",humi);
	
}


