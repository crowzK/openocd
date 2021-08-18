// Copyright (C) 2021 by Yudoc Kim, craven@crowz.kr
//
// build:
//  ./bootstrap
//  ./configure --enable-sysfsgpio --enable-bcm2835spi
//  make
//
// it uses AUS_SPI0 port

#include <jtag/interface.h>
#include <transport/transport.h>
#include <jtag/swd.h>
#include <sys/mman.h>

typedef struct
{
	volatile uint32_t gpfsel[6];	// 0x0
	volatile uint32_t reserve1[1];
	volatile uint32_t gpset[3];		// 0x1C
	volatile uint32_t gpclr[3];		// 10
	volatile uint32_t gplev[3];		// 13
	volatile uint32_t gpeds[3];		// 16
	volatile uint32_t gpren[3];		// 19
	volatile uint32_t gpfen[3];		// 22
	volatile uint32_t gphen[3];		// 25
	volatile uint32_t gplen[3];		// 28
	volatile uint32_t gparen[3];	// 31
	volatile uint32_t gpafen[3];	// 34
	volatile uint32_t reserve2[20];	// 34
	volatile uint32_t gppupdctrl[4];
}GPIO;

typedef struct
{
	volatile uint32_t cntl0;
	volatile uint32_t cntl1;
	volatile uint32_t stat;
	volatile uint32_t peek;
	volatile uint32_t ioRegA;
	volatile uint32_t ioRegB;
	volatile uint32_t ioRegC;
	volatile uint32_t ioRegD;
	volatile uint32_t txHoldA;
	volatile uint32_t txHoldB;
	volatile uint32_t txHoldC;
	volatile uint32_t txHoldD;
}AUX_SPI;

#define AUXSPI_CNTL0_SPEED(x)		((x) << 20)
#define AUXSPI_CNTL0_CS(x)			((x) << 17)
#define AUXSPI_CNTL0_POSTINP		(1 << 16)
#define AUXSPI_CNTL0_VAR_CS			(1 << 15)
#define AUXSPI_CNTL0_VAR_WIDTH		(1 << 14)
#define AUXSPI_CNTL0_DOUT_HOLD(x)	((x) << 12)
#define AUXSPI_CNTL0_ENABLE			(1 << 11)
#define AUXSPI_CNTL0_IN_RISING(x)	((x) << 10)
#define AUXSPI_CNTL0_CLR_FIFOS		(1 << 9)
#define AUXSPI_CNTL0_OUT_RISING(x)	((x) << 8)
#define AUXSPI_CNTL0_INVERT_CLK(x)	((x) << 7)
#define AUXSPI_CNTL0_MSB_FIRST(x)	((x) << 6)
#define AUXSPI_CNTL0_SHIFT_LEN(x)	((x) << 0)

#define AUXSPI_CNTL1_CS_HIGH(x)		((x) << 8)
#define AUXSPI_CNTL1_TX_IRQ			(1 << 7)
#define AUXSPI_CNTL1_DONE_IRQ		(1 << 6)
#define AUXSPI_CNTL1_MSB_FIRST(x)	((x) << 1)
#define AUXSPI_CNTL1_KEEP_INPUT		(1 << 0)

#define AUXSPI_STAT_TX_FIFO(x)		((x) << 28)
#define AUXSPI_STAT_RX_FIFO(x)		((x) << 20)
#define AUXSPI_STAT_TX_FULL			(1 << 10)
#define AUXSPI_STAT_TX_EMPTY		(1 << 9)
#define AUXSPI_STAT_RX_EMPTY		(1 << 7)
#define AUXSPI_STAT_BUSY			(1 << 6)
#define AUXSPI_STAT_BITS(x)			((x) << 0)


#define GPIO_FUNCTION_IN		0x0
#define GPIO_FUNCTION_OUTPUT	0x1
#define GPIO_FUNCTION_ALT4		0x3

static uint32_t bcm2835_peri_base = 0xfe000000;
static uint32_t systemClk = 250000000;
static uint32_t spiSpeed = 100000;

#define GPIO_OFFSET		(0x200000)
#define AUX_OFFSET		(0x215000)
#define AUX_SPI1_OFFSET	(0x80 / sizeof(uint32_t))
#define AUX_SPI2_OFFSET	(0xC0 / sizeof(uint32_t))

static volatile GPIO* Gpio;
static volatile uint32_t* Aux;
static volatile AUX_SPI* AuxSpi;

static int bcm2835spi_init(void);
static int bcm2835spi_quit(void);

static void bcm2835spi_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk);

static int srst_gpio = -1;

static int gpio_mm_fd = -1;
static int aux_mm_fd = -1;

static void initGpio(int idx, int function)
{
	LOG_DEBUG("%s idx %d function %d", __func__, idx, function);
	const uint32_t wordIdx = idx / 10;
	const uint32_t bitIdx = (idx % 10) * 3;
	uint32_t mask = Gpio->gpfsel[wordIdx] & ~(0x7 << bitIdx);
	Gpio->gpfsel[wordIdx] = mask | ((function & 0x7) << bitIdx);
}

static void setGpio(int idx, bool set)
{
	LOG_DEBUG("%s idx %d set %d", __func__, idx, set);
	const uint32_t wordIdx = idx / 32;
	const uint32_t bitIdx = idx % 32;
	if(set)
	{
		Gpio->gpset[wordIdx] = (0x1 << bitIdx);
	}
	else
	{
		Gpio->gpclr[wordIdx] = (0x1 << bitIdx);
	}
}

static int bcm2835spi_reset(int trst, int srst)
{
	LOG_DEBUG(__func__);
	if(srst_gpio >= 0)
	{
		setGpio(srst_gpio, !srst);
	}
	return ERROR_OK;
}

static int bcm2835spi_khz(int khz, int *jtag_speed)
{
	LOG_DEBUG(__func__);
	*jtag_speed = khz;
	return ERROR_OK;
}

static int bcm2835spi_speed_div(int speed, int *khz)
{
	LOG_DEBUG(__func__);
	*khz = speed;
	return ERROR_OK;
}

static int bcm2835spi_speed(int speed)
{
	LOG_DEBUG("%s speed %d hz", __func__, speed);
	spiSpeed = speed * 1000;
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835spi_handle_jtag_gpionum_srst)
{
	LOG_DEBUG(__func__);
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], srst_gpio);

	command_print(CMD, "BCM2835 GPIO config: srst = %d", srst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835spi_handle_peripheral_base)
{
	LOG_DEBUG(__func__);
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], bcm2835_peri_base);

	command_print(CMD, "BCM2835 GPIO: peripheral_base = 0x%08x",
				  bcm2835_peri_base);
	return ERROR_OK;
}

static const struct command_registration bcm2835spi_command_handlers[] = {
	{
		.name = "bcm2835spi_srst_num",
		.handler = &bcm2835spi_handle_jtag_gpionum_srst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for srst.",
		.usage = "[srst]",
	},
	{
		.name = "bcm2835spi_peripheral_base",
		.handler = &bcm2835spi_handle_peripheral_base,
		.mode = COMMAND_CONFIG,
		.help = "peripheral base to access GPIOs (RPi1 0x20000000, RPi2 0x3F000000).",
		.usage = "[base]",
	},

	COMMAND_REGISTRATION_DONE
};

static uint32_t swd_send_rcv(uint32_t data, uint32_t bit_len)
{
	AuxSpi->cntl0 = AUXSPI_CNTL0_ENABLE | AUXSPI_CNTL0_CLR_FIFOS;
	AuxSpi->cntl0 = AUXSPI_CNTL0_SPEED((systemClk / spiSpeed) - 1)
			| AUXSPI_CNTL0_SHIFT_LEN(bit_len)
//			| AUXSPI_CNTL0_MSB_FIRST(1)
			| AUXSPI_CNTL0_ENABLE
//			| AUXSPI_CNTL0_DOUT_HOLD(1)
			| AUXSPI_CNTL0_IN_RISING(1)
//			| AUXSPI_CNTL0_INVERT_CLK(1)
			| AUXSPI_CNTL0_OUT_RISING(0);

	AuxSpi->txHoldA = (uint32_t)(data);
	while(AuxSpi->stat & AUXSPI_STAT_BUSY);
	return AuxSpi->peek >> (32 - bit_len);
}

static void swd_send(uint8_t data[], uint32_t length)
{
	LOG_DEBUG("%s len %d", __func__, length);
	for(uint32_t idx = 0; idx < (length / 8); idx++)
	{
		swd_send_rcv(data != NULL ? data[idx] : 0, 8);
	}
}

static int execute_queue(void)
{
	LOG_DEBUG(__func__);
	return ERROR_FAIL;
}

static int bcm2835spi_swd_init(void)
{
	LOG_DEBUG(__func__);
	
	return ERROR_OK;
}

static void bcm2835spi_swd_clear_sticky_errors(void)
{
	LOG_DEBUG(__func__);
	bcm2835spi_swd_write_reg(swd_cmd(false,  false, DP_ABORT),
		STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR, 0);
}

static int bcm2835spi_swd_switch_seq(enum swd_special_seq seq)
{
	LOG_DEBUG(__func__);

	switch (seq) {
	case LINE_RESET:
		LOG_DEBUG("SWD line reset");
		swd_send((uint8_t *)swd_seq_line_reset, swd_seq_line_reset_len);
		break;
	case JTAG_TO_SWD:
		LOG_DEBUG("JTAG-to-SWD");
		swd_send((uint8_t *)swd_seq_jtag_to_swd, swd_seq_jtag_to_swd_len);
		break;
	case SWD_TO_JTAG:
		LOG_DEBUG("SWD-to-JTAG");
		swd_send((uint8_t *)swd_seq_swd_to_jtag, swd_seq_swd_to_jtag_len);
		break;
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int cmd_queued_retval;

static void bcm2835spi_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
	assert(cmd & SWD_CMD_RnW);
	if (cmd_queued_retval != ERROR_OK) 
	{
		LOG_DEBUG("Skip %s because cmd_queued_retval=%d", __func__, cmd_queued_retval);
		return;
	}

	cmd |= SWD_CMD_START | (1 << 7);
	LOG_DEBUG("%s cmd %X", __func__, cmd);

	while(1)
	{
		// send cmd + trn + ack
		const uint32_t ack = (swd_send_rcv(cmd, 8 + 1 + 3) >> 9) & 0x7; // LSB first

		// uint32_t header = cmd | (SWD_ACK_OK << 1) << 8;
		// const uint32_t ack = (swd_send_rcv(header, 8 + 1 + 3) >> 9) & 0x7; // LSB first

		switch(ack) 
		{
		case SWD_ACK_OK:
		{
			// rcv data
			uint32_t _value;
			_value= swd_send_rcv(0, 32); 
			int parity = swd_send_rcv(0, 1);
			if(parity != (parity_u32(_value)))
			{
				LOG_DEBUG("Wrong parity detected");
				cmd_queued_retval = ERROR_FAIL;
			}
			if(value != NULL)
			{
				*value = _value;
			}
			if (cmd & SWD_CMD_APnDP)
				swd_send(NULL, ap_delay_clk);
			return;
		}
		case SWD_ACK_WAIT:
			LOG_DEBUG("SWD_ACK_WAIT");
			bcm2835spi_swd_clear_sticky_errors();
			// rcv it again
			break;
		case SWD_ACK_FAULT:
			LOG_DEBUG("SWD_ACK_FAULT");
			cmd_queued_retval = ack;
			return;
		default:
			LOG_DEBUG("No valid acknowledge: ack=%d", ack);
			cmd_queued_retval = ack;
			return;
		}
	}
}

static void bcm2835spi_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
	assert(!(cmd & SWD_CMD_RnW));
	if (cmd_queued_retval != ERROR_OK) 
	{
		LOG_DEBUG("Skip %s because cmd_queued_retval=%d", __func__, cmd_queued_retval);
		return;
	}

	cmd |= SWD_CMD_START | (1 << 7);
	LOG_DEBUG("%s cmd %X data %X", __func__, cmd, value);

	while(1)
	{
		// send cmd + trn + ack + trn
		const uint32_t ack = (swd_send_rcv(cmd, 8 + 1 + 3 + 1) >> 9) & 0x7; // LSB first

		// uint32_t header = cmd | (SWD_ACK_OK << 1) << 8;
		// const uint32_t ack = (swd_send_rcv(header, 8 + 1 + 3 + 1) >> 9) & 0x7; // LSB first
		switch (ack) 
		{
		case SWD_ACK_OK:
		{
			// send data
			swd_send_rcv(value, 32);
			// send parity
			uint32_t parity = parity_u32(value);
			swd_send_rcv(parity, 1);
			if (cmd & SWD_CMD_APnDP)
				swd_send(NULL, ap_delay_clk);
			return;
		}
		case SWD_ACK_WAIT:
			LOG_DEBUG("SWD_ACK_WAIT");
			bcm2835spi_swd_clear_sticky_errors();
			// send it again
			break;
		case SWD_ACK_FAULT:
			LOG_DEBUG("SWD_ACK_FAULT");
			cmd_queued_retval = ack;
			return;
		default:
			LOG_DEBUG("No valid acknowledge: ack=%d", ack);
			cmd_queued_retval = ack;
			return;
		}
	}
}

static int bcm2835spi_swd_run_queue(void)
{
	/* A transaction must be followed by another transaction or at least 8 idle cycles to
	 * ensure that data is clocked through the AP. */
	swd_send(NULL, 8);

	int retval = cmd_queued_retval;
	cmd_queued_retval = ERROR_OK;
	LOG_DEBUG("%s SWD queue return value: %02x", __func__, retval);
	return retval;
}

static const char * const bcm2835_transports[] = { "swd", NULL };

static struct jtag_interface bcm2835spi_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = execute_queue,
};

const struct swd_driver bcm2835spi_swd = {
	.init = bcm2835spi_swd_init,
	.switch_seq = bcm2835spi_swd_switch_seq,
	.read_reg = bcm2835spi_swd_read_reg,
	.write_reg = bcm2835spi_swd_write_reg,
	.run = bcm2835spi_swd_run_queue,
};

struct adapter_driver bcm2835spi_adapter_driver = {
	.name = "bcm2835spi",
	.transports = bcm2835_transports,
	.commands = bcm2835spi_command_handlers,

	.init = bcm2835spi_init,
	.quit = bcm2835spi_quit,
	.reset = bcm2835spi_reset,
	.speed = bcm2835spi_speed,
	.khz = bcm2835spi_khz,
	.speed_div = bcm2835spi_speed_div,

	.jtag_ops = &bcm2835spi_interface,
	.swd_ops = &bcm2835spi_swd,
};

static int bcm2835spi_init(void)
{
	LOG_DEBUG(__func__);
	if(transport_is_jtag()) 
	{
		LOG_DEBUG("JTAG DOES NOT SUPPORT");
		return ERROR_FAIL;
	}

	gpio_mm_fd = open("/dev/mem", O_RDWR|O_SYNC);
	if (gpio_mm_fd < 0) 
	{
		LOG_DEBUG("Failed to open /dev/mem, try checking permissions.");
		return ERROR_FAIL;
	}

	Gpio = (GPIO*) mmap(
		NULL,
		sysconf(_SC_PAGE_SIZE),
		PROT_READ | PROT_WRITE,
		MAP_SHARED,
		gpio_mm_fd,
		bcm2835_peri_base + GPIO_OFFSET
	);

	if(Gpio == MAP_FAILED)
	{
		LOG_DEBUG("GPIO MAP FAILED %lX", (uint64_t)Gpio);
		return ERROR_FAIL;
	}
	
	// rx pin init P19, ALT4
	initGpio(19, GPIO_FUNCTION_ALT4);

	// tx pin init P20, ALT4
	initGpio(20, GPIO_FUNCTION_ALT4);

	// clk pin init P21, ALT4
	initGpio(21, GPIO_FUNCTION_ALT4);

	aux_mm_fd = open("/dev/mem", O_RDWR|O_SYNC);
	if (aux_mm_fd < 0) 
	{
		LOG_DEBUG("Failed to open /dev/mem, try checking permissions.");
		return ERROR_FAIL;
	}

	Aux = (uint32_t*) mmap(
		NULL,
		sysconf(_SC_PAGE_SIZE),
		PROT_READ | PROT_WRITE,
		MAP_SHARED | MAP_LOCKED,
		aux_mm_fd,
		bcm2835_peri_base + AUX_OFFSET
	);

	if(Aux == MAP_FAILED)
	{
		LOG_DEBUG("AUX MAP FAILED %lX", (uint64_t)Aux);
		return ERROR_FAIL;
	}

	// enable AUX
	Aux[1] = Aux[1] | (0x1 << 1);
	AuxSpi = (AUX_SPI*)&Aux[AUX_SPI1_OFFSET];

	if (srst_gpio != -1) 
	{
		initGpio(srst_gpio, GPIO_FUNCTION_OUTPUT);
		setGpio(srst_gpio, true);
	}

	return ERROR_OK;
}

static int bcm2835spi_quit(void)
{
	LOG_DEBUG(__func__);
	if(gpio_mm_fd != -1)
	{
		munmap((void*)Gpio, sysconf(_SC_PAGE_SIZE));
		close(gpio_mm_fd);
	}

	if(aux_mm_fd != -1)
	{
		Aux[1] = Aux[1] & (~0x1);
		munmap((void*)Aux, sysconf(_SC_PAGE_SIZE));
		close(aux_mm_fd);
	}
	return ERROR_OK;
}
