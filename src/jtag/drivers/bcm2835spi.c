#include <jtag/interface.h>
#include <transport/transport.h>
#include <jtag/swd.h>
#include <sys/mman.h>

static uint32_t bcm2835_peri_base = 0xFE000000;
#define BCM2835_GPIO_BASE	(bcm2835_peri_base + 0x200000) /* GPIO controller */

static int bcm2835spi_init(void);
static int bcm2835spi_quit(void);

static int trst_gpio = -1;
static int srst_gpio = -1;

static int bcm2835spi_reset(int trst, int srst)
{
	return ERROR_FAIL;
}


static int bcm2835spi_khz(int khz, int *jtag_speed)
{
	return ERROR_FAIL;
}

static int bcm2835spi_speed_div(int speed, int *khz)
{
	return ERROR_FAIL;
}

static int bcm2835spi_speed(int speed)
{
	return ERROR_FAIL;
}


COMMAND_HANDLER(bcm2835spi_handle_jtag_gpionum_srst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], srst_gpio);

	command_print(CMD, "BCM2835 GPIO config: srst = %d", srst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835spi_handle_jtag_gpionum_trst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], trst_gpio);

	command_print(CMD, "BCM2835 GPIO config: trst = %d", trst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835spi_handle_peripheral_base)
{
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
		.name = "bcm2835spi_trst_num",
		.handler = &bcm2835spi_handle_jtag_gpionum_trst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for trst.",
		.usage = "[trst]",
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

static int execute_queue(void)
{
	return ERROR_FAIL;
}

static int bcm2835spi_swd_init(void)
{
	LOG_DEBUG("bcm2835spi_swd_init");
	return ERROR_OK;
}

static int bcm2835spi_swd_switch_seq(enum swd_special_seq seq)
{
	LOG_DEBUG("bcm2835spi_swd_switch_seq");

	switch (seq) {
	case LINE_RESET:
		LOG_DEBUG("SWD line reset");
		break;
	case JTAG_TO_SWD:
		LOG_DEBUG("JTAG-to-SWD");
		break;
	case SWD_TO_JTAG:
		LOG_DEBUG("SWD-to-JTAG");
		break;
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static void bcm2835spi_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{

}

static void bcm2835spi_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{

}
static int bcm2835spi_swd_run_queue(void)
{
	return ERROR_OK;
}

static const char * const bcm2835_transports[] = { "jtag", "swd", NULL };

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
	return ERROR_FAIL;
}

static int bcm2835spi_quit(void)
{
	return ERROR_FAIL;
}
