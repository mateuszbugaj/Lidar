#include "FreeRTOS.h"
#include "task.h"
#include "i2c.h"

static const char *i2c_msg[] = {
	"OK",
	"Address timeout",
	"Address NAK",
	"Write timeout",
	"Read timeout"
};

void i2c_setup(uint32_t dev) {
	i2c_peripheral_disable(dev); // Disable the I2C before changing any configuration
	
	i2c_set_standard_mode(dev);	// 100 kHz mode
	i2c_set_clock_frequency(dev,I2C_CR2_FREQ_36MHZ); // APB Freq
	i2c_set_trise(dev,36);		// 1000 ns
	i2c_set_dutycycle(dev,I2C_CCR_DUTY_DIV2);
	i2c_set_ccr(dev,180);		// 100 kHz <= 180 * 1 /36M

	i2c_peripheral_enable(dev);
}

void i2c_start_addr(uint32_t dev,uint8_t addr,enum I2C_RW rw) {
	TickType_t t0 = xTaskGetTickCount();

	/* Block until not busy */
	while ( I2C_SR2(dev) & I2C_SR2_BUSY ) {
		taskYIELD();
  }

	/* Clear Acknowledge failure */
	I2C_SR1(dev) &= ~I2C_SR1_AF;

	if ( rw == Read ) {
		i2c_enable_ack(dev);
	}

	/* Generate a Start/Restart */
	i2c_send_start(dev);

	/* Waiting for START is send and switched to master mode */
  while ( !((I2C_SR1(dev) & I2C_SR1_SB) && (I2C_SR2(dev) & (I2C_SR2_MSL|I2C_SR2_BUSY))) ) {
		taskYIELD();
	}

	/* Send Address & R/W flag */
	i2c_send_7bit_address(dev, addr, rw == Read ? I2C_READ : I2C_WRITE);

	/* Waiting for address is transferred, NAK or timeout */
	t0 = xTaskGetTickCount();
	while ( !(I2C_SR1(dev) & I2C_SR1_ADDR) ) {

		/* NACK */
		if ( I2C_SR1(dev) & I2C_SR1_AF ) {
			i2c_send_stop(dev);
			(void)I2C_SR1(dev);
			(void)I2C_SR2(dev); 	// Clear flags
		}

		taskYIELD();
	}

	(void)I2C_SR2(dev);		// Clear flags
}

void i2c_write(uint32_t dev, uint8_t byte) {
	TickType_t t0 = xTaskGetTickCount();

	i2c_send_data(dev, byte);
	while ( !(I2C_SR1(dev) & (I2C_SR1_BTF)) ) {
		taskYIELD();
	}
}

uint8_t i2c_read(uint32_t dev, bool lastf) {
	TickType_t t0 = xTaskGetTickCount();

	if ( lastf ) {
		i2c_disable_ack(dev);	// Reading last/only byte
	}

	while ( !(I2C_SR1(dev) & I2C_SR1_RxNE) ) {
		taskYIELD();
	}
	
	return i2c_get_data(dev);
}

void i2c_write_read(uint32_t dev, uint8_t addr, uint8_t byte) {
	TickType_t t0 = xTaskGetTickCount();

	taskENTER_CRITICAL();
	i2c_send_data(dev,byte);
	// Must set start before byte has written out
	i2c_send_start(dev);
	taskEXIT_CRITICAL();

	/* Wait for start to complete */
	while ( !(I2C_SR1(dev) & (I2C_SR1_BTF)) ) {
		taskYIELD();
	}

	/* Waiting for START is send and switched to master mode */
	t0 = xTaskGetTickCount();
  while ( !((I2C_SR1(dev) & I2C_SR1_SB) && (I2C_SR2(dev) & (I2C_SR2_MSL|I2C_SR2_BUSY))) ) {
		taskYIELD();
	}

	/* Send Address & Read command bit */
	i2c_send_7bit_address(dev,addr,I2C_READ);

	/* Waiting for address is transferred, NAK or timeout */
	t0 = xTaskGetTickCount();
	while ( !(I2C_SR1(dev) & I2C_SR1_ADDR) ) {
		if ( I2C_SR1(dev) & I2C_SR1_AF ) {
			i2c_send_stop(dev);
			(void)I2C_SR1(dev);
			(void)I2C_SR2(dev); 	// Clear flags
		}

		taskYIELD();
	}

	(void)I2C_SR2(dev);		// Clear flags
}

void i2c_stop(uint32_t dev) { 
	i2c_send_stop(dev); 
}

uint8_t i2c_readN(uint32_t dev, uint8_t slave_address, uint8_t* data, uint8_t N){
	i2c_start_addr(dev, slave_address, Read);
	for (uint8_t i = 0; i < N; i++){
		data[i] = i2c_read(dev, i == N-1);
	}
	i2c_stop(dev);
	return 0;
}

uint8_t i2c_writeN(uint32_t dev, uint8_t slave_address, uint8_t* data, uint8_t N){
	i2c_start_addr(dev, slave_address, Write);
	for (uint8_t i = 0; i < N; i++){
		i2c_write(dev, data[i]);
	}
	i2c_stop(dev);
	return 0;
}