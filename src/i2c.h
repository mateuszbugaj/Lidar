#pragma once

#include <stdbool.h>
#include <setjmp.h>

#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

enum I2C_RW {
	Read = 1,
	Write = 0
};

void i2c_setup(uint32_t dev);
void i2c_start_addr(uint32_t dev,uint8_t addr,enum I2C_RW rw);
void i2c_stop(uint32_t dev);

void i2c_write(uint32_t dev,uint8_t byte);
uint8_t i2c_read(uint32_t dev,bool lastf);
void i2c_write_read(uint32_t dev,uint8_t byte,uint8_t addr);

uint8_t i2c_readN(uint32_t dev, uint8_t slave_address, uint8_t* data, uint8_t N);
uint8_t i2c_writeN(uint32_t dev, uint8_t slave_address, uint8_t* data, uint8_t N);