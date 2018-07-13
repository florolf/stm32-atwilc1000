#include <stdlib.h>
#include <stdint.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/spi.h>

#include "systick.h"

static void spi_setup(void) {

	/* Configure GPIOs: SS=PA4, SCK=PA5, MISO=PA6 and MOSI=PA7 */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
	              GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
	              GPIO5 | GPIO7 );
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
	              GPIO4);

	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
	              GPIO6);

	spi_reset(SPI1);

	//spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_256, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
	spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_256, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
	                SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

	spi_enable_software_slave_management(SPI1);
	spi_set_nss_high(SPI1);

	spi_enable(SPI1);
}

void reset(void)
{
	gpio_clear(GPIOA, GPIO4);
	gpio_clear(GPIOA, GPIO1);
	delay_ms(10);

	gpio_set(GPIOA, GPIO4);
	delay_ms(10);

	gpio_set(GPIOA, GPIO1);
	delay_ms(10);
}

static const uint8_t crc7_syndrome_table[256] = {
	0x00, 0x09, 0x12, 0x1b, 0x24, 0x2d, 0x36, 0x3f,
	0x48, 0x41, 0x5a, 0x53, 0x6c, 0x65, 0x7e, 0x77,
	0x19, 0x10, 0x0b, 0x02, 0x3d, 0x34, 0x2f, 0x26,
	0x51, 0x58, 0x43, 0x4a, 0x75, 0x7c, 0x67, 0x6e,
	0x32, 0x3b, 0x20, 0x29, 0x16, 0x1f, 0x04, 0x0d,
	0x7a, 0x73, 0x68, 0x61, 0x5e, 0x57, 0x4c, 0x45,
	0x2b, 0x22, 0x39, 0x30, 0x0f, 0x06, 0x1d, 0x14,
	0x63, 0x6a, 0x71, 0x78, 0x47, 0x4e, 0x55, 0x5c,
	0x64, 0x6d, 0x76, 0x7f, 0x40, 0x49, 0x52, 0x5b,
	0x2c, 0x25, 0x3e, 0x37, 0x08, 0x01, 0x1a, 0x13,
	0x7d, 0x74, 0x6f, 0x66, 0x59, 0x50, 0x4b, 0x42,
	0x35, 0x3c, 0x27, 0x2e, 0x11, 0x18, 0x03, 0x0a,
	0x56, 0x5f, 0x44, 0x4d, 0x72, 0x7b, 0x60, 0x69,
	0x1e, 0x17, 0x0c, 0x05, 0x3a, 0x33, 0x28, 0x21,
	0x4f, 0x46, 0x5d, 0x54, 0x6b, 0x62, 0x79, 0x70,
	0x07, 0x0e, 0x15, 0x1c, 0x23, 0x2a, 0x31, 0x38,
	0x41, 0x48, 0x53, 0x5a, 0x65, 0x6c, 0x77, 0x7e,
	0x09, 0x00, 0x1b, 0x12, 0x2d, 0x24, 0x3f, 0x36,
	0x58, 0x51, 0x4a, 0x43, 0x7c, 0x75, 0x6e, 0x67,
	0x10, 0x19, 0x02, 0x0b, 0x34, 0x3d, 0x26, 0x2f,
	0x73, 0x7a, 0x61, 0x68, 0x57, 0x5e, 0x45, 0x4c,
	0x3b, 0x32, 0x29, 0x20, 0x1f, 0x16, 0x0d, 0x04,
	0x6a, 0x63, 0x78, 0x71, 0x4e, 0x47, 0x5c, 0x55,
	0x22, 0x2b, 0x30, 0x39, 0x06, 0x0f, 0x14, 0x1d,
	0x25, 0x2c, 0x37, 0x3e, 0x01, 0x08, 0x13, 0x1a,
	0x6d, 0x64, 0x7f, 0x76, 0x49, 0x40, 0x5b, 0x52,
	0x3c, 0x35, 0x2e, 0x27, 0x18, 0x11, 0x0a, 0x03,
	0x74, 0x7d, 0x66, 0x6f, 0x50, 0x59, 0x42, 0x4b,
	0x17, 0x1e, 0x05, 0x0c, 0x33, 0x3a, 0x21, 0x28,
	0x5f, 0x56, 0x4d, 0x44, 0x7b, 0x72, 0x69, 0x60,
	0x0e, 0x07, 0x1c, 0x15, 0x2a, 0x23, 0x38, 0x31,
	0x46, 0x4f, 0x54, 0x5d, 0x62, 0x6b, 0x70, 0x79
};

static uint8_t crc7_byte(uint8_t crc, uint8_t data)
{
	return crc7_syndrome_table[(crc << 1) ^ data];
}

static uint8_t crc7(const uint8_t *buffer, uint32_t len)
{
	uint8_t crc = 0x7f;

	while (len--)
		crc = crc7_byte(crc, *buffer++);

	return crc << 1;
}

static void cs(bool en)
{
	if (en)
		gpio_clear(GPIOA, GPIO4);
	else {
		gpio_set(GPIOA, GPIO4);
		delay_ms(1);
	}
}

#define CMD_INTERNAL_WRITE		0xc3
#define CMD_INTERNAL_READ		0xc4
#define CMD_SINGLE_WRITE		0xc9
#define CMD_SINGLE_READ			0xca

static void put16be(uint8_t *out, uint16_t data)
{
	out[0] = (data >>  8);
	out[1] =  data;
}

static void put24be(uint8_t *out, uint32_t data)
{
	out[0] = (data >> 16);
	out[1] = (data >>  8);
	out[2] =  data;
}

static void put32be(uint8_t *out, uint32_t data)
{
	out[0] = (data >> 24);
	out[1] = (data >> 16);
	out[2] = (data >>  8);
	out[3] =  data;
}

static uint32_t get32le(uint8_t *in)
{
	return in[3] << 24 |
	       in[2] << 16 |
	       in[1] <<  8 |
	       in[0];
}

static void xfer(uint8_t *out, size_t out_len, uint8_t *in, size_t in_len)
{
	for (size_t i = 0; i < out_len; i++)
		spi_xfer(SPI1, out[i]);

	for (size_t i = 0; i < in_len; i++) {
		uint8_t tmp;

		tmp = spi_xfer(SPI1, 0x00);

		if (in)
			in[i] = tmp;
	}
}

static int get_status(uint8_t cmd)
{
	int retry;
	uint8_t tmp;

	retry = 10;
	while (1) {
		tmp = spi_xfer(SPI1, 0x00);
		if (tmp == cmd)
			break;

		if (retry == 0)
			return -1;

		retry--;
	}

	retry = 10;
	while (1) {
		tmp = spi_xfer(SPI1, 0x00);
		if (tmp == 0x00)
			break;

		if (retry == 0)
			return -1;

		retry--;
	}

	return 0;
}

// TODO: 8k chunking
static int read_payload(uint8_t *out, size_t len, bool crc)
{
	int retry = 10;
	uint8_t tmp;

	// read header
	while (1) {
		tmp = spi_xfer(SPI1, 0x00);

		if ((tmp & 0xF0) == 0xF0)
			break;

		if (retry == 0)
			return -1;

		retry--;
	}

	// read payload
	xfer(NULL, 0, out, len);

	if (crc) {
		// drop CRC (TODO: check this, contrary to vendor code?)
		spi_xfer(SPI1, 0x00);
		spi_xfer(SPI1, 0x00);
	}

	return 0;
}

static int read_reg(uint32_t reg, bool crc, uint32_t *out)
{
	bool clockless, internal;
	int ret = -1;

	if (reg <= 0x10) {
		internal = true;
		clockless = true;
	} else if (reg <= 0xff) {
		internal = true;
		clockless = false;
	} else {
		internal = false;
		clockless = false;
	}

	uint8_t cmd_buf[4];
	cmd_buf[0] = internal ? CMD_INTERNAL_READ : CMD_SINGLE_READ;
	if (internal) {
		uint16_t tmp = reg;

		if (clockless)
			tmp |= 0x8000;

		put16be(&cmd_buf[1], tmp);
		cmd_buf[3] = 0;
	} else
		put24be(&cmd_buf[1], reg);

	cs(true);

	xfer(cmd_buf, sizeof(cmd_buf), NULL, 0);
	if (crc)
		spi_xfer(SPI1, crc7(cmd_buf, sizeof(cmd_buf)));

	if (get_status(cmd_buf[0]) < 0)
		goto out;

	uint8_t out_buf[4];
	if (read_payload(out_buf, sizeof(out_buf), clockless) < 0)
		goto out;

	ret = 0;

out:
	cs(false);

	if (out)
		*out = get32le(out_buf);

	return ret;
}

static int write_reg(uint32_t reg, bool crc, uint32_t data)
{
	bool clockless, internal;
	int ret = -1;

	if (reg <= 0x10) {
		internal = true;
		clockless = true;
	} else if (reg <= 0xff) {
		internal = true;
		clockless = false;
	} else {
		internal = false;
		clockless = false;
	}

	uint8_t cmd_buf[8];
	size_t len;

	cmd_buf[0] = internal ? CMD_INTERNAL_WRITE : CMD_SINGLE_WRITE;
	if (internal) {
		uint16_t tmp = reg;

		if (clockless)
			tmp |= 0x8000;

		put16be(&cmd_buf[1], tmp);
		put32be(&cmd_buf[3], data);

		len = 7;
	} else {
		put24be(&cmd_buf[1], reg);
		put32be(&cmd_buf[4], data);

		len = 8;
	}

	cs(true);

	xfer(cmd_buf, len, NULL, 0);
	if (crc)
		spi_xfer(SPI1, crc7(cmd_buf, len));

	if (get_status(cmd_buf[0]) < 0)
		goto out;

	ret = 0;

out:
	cs(false);

	return ret;
}

int main(void)
{
	rcc_clock_setup_in_hse_8mhz_out_24mhz();
	init_systick();

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_SPI1);

	spi_setup();
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
			GPIO1);

	//uint8_t buf[] = {0xc4, 0x80, 0x01, 0x00, 0x00};

	delay_ms(100);

	cs(false);

	while (1) {
		uint32_t tmp = 0;
		int ret;

		reset();
		ret = read_reg(0x24, true, &tmp);

		ret = read_reg(0x0b, true, &tmp);
		if (ret == 0) {
			tmp |= 1;
			write_reg(0x0b, true, tmp);
		}

		ret = read_reg(0x01, true, &tmp);
		if (ret == 0) {
			tmp |= 2;
			write_reg(0x01, true, tmp);
		}

		int retries = 10;
		while (retries--) {
			ret = read_reg(0x0f, true, &tmp);
			if (ret < 0)
				continue;

			if (tmp & (1<<2))
				break;
		}

		read_reg(0x1000, true, &tmp);
		read_reg(0x13f4, true, &tmp);

		delay_ms(100);
	}


	return 0;
}
