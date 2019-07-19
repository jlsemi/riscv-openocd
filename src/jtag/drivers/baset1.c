#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include <jtag/interface.h>
#include "bitbang.h"
#include "mpsse.h"

#define DEBUG_BASET1	0
#define DEBUG_BASET1_MDIO_READ	0
#define DEBUG_BASET1_MDIO_WRITE	0

/* CONFIG */
/* speedup JTAG */
#define _FAST_JTAG_	1
#define COPY_TO_BITS false

/* Constants & Macros */
#define MDIO_MODE	(MSB_FIRST | POS_EDGE_OUT | NEG_EDGE_IN)

#define MDIO_READ	0
#define MDIO_WRITE	1

#define MAX_USB_IDS 8

#define MDIO_PREAMBLE_SIZE	(4)
#define MDIO_COMMAND_SIZE	(4)
#define MDIO_ADDR_SIZE		(MDIO_PREAMBLE_SIZE + MDIO_COMMAND_SIZE)
#define MDIO_VALUE_SIZE		(MDIO_PREAMBLE_SIZE + MDIO_COMMAND_SIZE)
#define MDIO_BUFF_SIZE		(MDIO_ADDR_SIZE  + MDIO_VALUE_SIZE)
#define MDIO_BUFF_SIZE_FAST	((1 + MDIO_COMMAND_SIZE) * 2)

#define MDIO_JTAG_DEV	0x1f
#define MDIO_JTAG_REG	0x10

#define TARGET_PMU	8
#define TARGET_MCU	0

#define JTAG_RST(target)	(1 << (target + 4))
#define JTAG_TCK(target)	(1 << (target + 3))
#define JTAG_TMS(target)	(1 << (target + 2))
#define JTAG_TDI(target)	(1 << (target + 1))
#define JTAG_TDO(target)	(1 << (target + 0))

static struct mdio_ctx {
	uint16_t init_output;
	uint16_t init_direction;
	uint8_t target_cpu; /* PMU or MCU */
	int freq;
	uint8_t phy_id;

	/* vid = pid = 0 marks the end of the list */
	uint16_t ftdi_vid[MAX_USB_IDS + 1];
	uint16_t ftdi_pid[MAX_USB_IDS + 1];

	uint8_t input[MDIO_BUFF_SIZE];
	uint8_t output[MDIO_BUFF_SIZE];
	uint16_t jtag_reg;

	bool is_running;
	struct mpsse_ctx *mpsse_ctx;
} _mdio_ctx = {
	.phy_id = 0x1a,
	.target_cpu = TARGET_PMU,
	.jtag_reg = 0,
	.ftdi_vid = { 0 },
	.ftdi_pid = { 0 },
	.init_output = 0x0003,
	.init_direction = 0xfffd,
	.mpsse_ctx = NULL,
	.is_running = false,
};
struct mdio_ctx *mdio_ctx = &_mdio_ctx;
bool fast_mode_en = false;

typedef uint16_t offset_t;

#if COPY_TO_BITS
#define COPY_32BITS(out, offset, val) \
	_mdio_bytes_to_bits(out, offset, val, 32)
/* MSB */
offset_t _mdio_bytes_to_bits(
	uint8_t *out, offset_t offset, uint32_t val, uint8_t nbits)
{
	uint8_t size = nbits;
	val &= (1 << size) - 1;
	uint32_t mask = 0;
	mask |= 1 << (size - 1);
	int i;
	for (i = 0; i < size; i++) {
		out[i + offset] = !!(val & mask);
		val <<= 1;
	}
	return offset + size;
}

#define COPY_8BITS(out, offset, val) \
	_mdio_bytes_to_bits(out, offset, val, 8)

#else /* COPY_TO_BITS */
#define COPY_32BITS(out, offset, val) \
	_mdio_bytes_to_bytes(out, offset, val)
static inline offset_t _mdio_bytes_to_bytes(
	uint8_t *out, offset_t offset, uint32_t val)
{
	out[offset++] = (val >> (8 * 3)) & 0xff;
	out[offset++] = (val >> (8 * 2)) & 0xff;
	out[offset++] = (val >> (8 * 1)) & 0xff;
	out[offset++] = (val >> (8 * 0)) & 0xff;
	return offset;
}

#define COPY_8BITS(out, offset, val) \
	_mdio_byte_to_byte(out, offset, val)
static inline offset_t _mdio_byte_to_byte(
	uint8_t *out, offset_t offset, uint8_t val)
{
	out[offset++] = (val >> (8 * 0)) & 0xff;
	return offset;
}
#endif

static int mdio_init(void)
{
	if (mdio_ctx->is_running)
		return ERROR_OK;

	struct mpsse_ctx *mpsse_ctx = mdio_ctx->mpsse_ctx;
	for (int i = 0; mdio_ctx->ftdi_vid[i] || mdio_ctx->ftdi_pid[i]; i++) {
		mpsse_ctx = mpsse_open(
				&mdio_ctx->ftdi_vid[i],
				&mdio_ctx->ftdi_pid[i],
				NULL, NULL, NULL, 0);
		if (mpsse_ctx)
			break;
	}
	if (!mpsse_ctx)
		return ERROR_JTAG_INIT_FAILED;

	mdio_ctx->mpsse_ctx = mpsse_ctx;

	mpsse_set_3phase_en(mpsse_ctx, true);
	mpsse_set_adaptive_clk_en(mpsse_ctx, false);
	mpsse_divide_by_5_config(mpsse_ctx, false);
	mpsse_loopback_config(mpsse_ctx, false);

	mpsse_set_divisor(mpsse_ctx, 0x0002);
	mpsse_flush(mpsse_ctx);
	mpsse_purge(mpsse_ctx);

	mpsse_set_data_bits_low_byte(mpsse_ctx, 0x03, 0x3);
	mpsse_flush(mpsse_ctx);
	mdio_ctx->is_running = true;
	LOG_INFO("INIT: baset1\n");
	return ERROR_OK;
}

static void mdio_quit(void)
{
	mpsse_close(mdio_ctx->mpsse_ctx);
}

static inline offset_t _mdio_preamble(uint8_t *out, offset_t offset)
{
	 offset = COPY_32BITS(out, offset, 0xffffffff);
	 return offset;
}

static inline offset_t _mdio_preamble_fast(uint8_t *out, offset_t offset)
{
	 offset = COPY_8BITS(out, offset, 0xff);
	 return offset;
}

static inline offset_t _mdio_command_address(
	uint8_t *out, offset_t offset,
	uint8_t phy, uint8_t dev, uint16_t reg)
{
	/* st:0b00, op:0b00, phy_addr[4:0]:phy, dev_type[4:0], TA[1:0]:0x02 */
	uint16_t cmd = ((phy & 0x1f) << 7) | ((dev & 0x1f) << 2) | 0x2;
	uint32_t opcode = (cmd << 16) | reg;
	return COPY_32BITS(out, offset, opcode);
}

static inline offset_t _mdio_command_value(
	uint8_t *out, offset_t offset,
	uint8_t phy, uint8_t dev, uint8_t mode, uint16_t val)
{
	/* st:0b00, op:0bxx, phy_addr[4:0]:phy, dev_type[4:0], TA[1:0]:0x02 */
	uint16_t cmd = 0;
	if (mode == MDIO_WRITE) {
		cmd = 0x1000;
	} else { /* read for else */
		cmd = 0x3000;
	}
	cmd |= ((phy & 0x1f) << 7) | ((dev & 0x1f) << 2) | 0x2;

	uint32_t opcode = (cmd << 16) | val;
	return COPY_32BITS(out, offset, opcode);
}

#if DEBUG_BASET1
#define DUMP_BUF(buf, offset, len)	do { \
	printf("---- sep --- %s:+%d %s\n", __FILE__, __LINE__, __func__); \
	_mdio_dump_buffer(buf, offset, len); \
} while(0)
static void _mdio_dump_buffer(
	const uint8_t *buff, offset_t offset, uint16_t length)
{
	int i = 0;
	for (i = 0; i < length; i++) {
		if (i % 8 == 0) printf("\n");
		printf("%02x\t", buff[i + offset]);
	}
	printf("\n");
}
#else
#define DUMP_BUF(buf, offset, len)
#endif

static uint16_t _mdio_communicate(
	uint8_t mode, uint8_t phy, uint8_t dev, uint16_t reg, uint16_t val)
{
	struct mpsse_ctx *mpsse_ctx = mdio_ctx->mpsse_ctx;
	uint8_t *output = mdio_ctx->output;
	uint8_t *input = mdio_ctx->input;

	offset_t offset = 0;
	if (fast_mode_en)
		offset = _mdio_preamble_fast(output, offset);
	else
		offset = _mdio_preamble(output, offset);

	offset = _mdio_command_address(output, offset, phy, dev, reg);

	if (fast_mode_en)
		offset = _mdio_preamble_fast(output, offset);
	else
		offset = _mdio_preamble(output, offset);

	offset = _mdio_command_value(output, offset, phy, dev, mode, val);

	if (fast_mode_en)
		assert(offset == MDIO_BUFF_SIZE_FAST);
	else
		assert(offset == MDIO_BUFF_SIZE);

	DUMP_BUF(output, 0, offset);

	mpsse_clock_data(mpsse_ctx,
		output, 0, input, 0,
		offset * 8, MDIO_MODE);
	mpsse_flush(mdio_ctx->mpsse_ctx);
	DUMP_BUF(input, 0, offset);
	uint16_t res = (input[offset - 2] << 8) | input[offset - 1];
	return res;
}

static inline uint16_t
mdio_read(uint8_t phy, uint8_t dev, uint16_t reg)
{
	uint16_t val = _mdio_communicate(MDIO_READ, phy, dev, reg, 0xffff);
#if DEBUG_BASET1_MDIO_READ
	printf("BASET1-MDIO: <read>  => "
		"phy:0x%x, dev:0x%x, reg:0x%04x | val:0x%04x\n",
		phy, dev, reg, val);
#endif
	return val;
}

static inline void
mdio_write(uint8_t phy, uint8_t dev, uint16_t reg, uint16_t val)
{
#if DEBUG_BASET1_MDIO_WRITE
	printf("BASET1-MDIO: <write> => "
		"phy:0x%x, dev:0x%x, reg:0x%04x | val:0x%04x\n",
		phy, dev, reg, val);
#endif
	_mdio_communicate(MDIO_WRITE, phy, dev, reg, val);
}

#if _FAST_JTAG_
/*
 * 1. Reduce number of preamble
 * 2. Turn on fast mode
 */
static inline void mdio_init_fast_mode(uint16_t phy)
{
	uint16_t mdio_cfg = mdio_read(phy, 0x1d, 0x0020);
	mdio_cfg &= ~0x3f;
	mdio_cfg |= 0x4;
	mdio_write(phy, 0x1d, 0x0020, mdio_cfg);
	fast_mode_en = true;
}
#endif /* _FAST_JTAG_ */

static struct bitbang_interface baset1_bitbang;
static int baset1_init(void)
{
	bitbang_interface = &baset1_bitbang;
	mdio_init();
#if _FAST_JTAG_
	mdio_init_fast_mode(mdio_ctx->phy_id);
#endif
	return ERROR_OK;
}

static int baset1_quit(void)
{
	mdio_quit();
	return ERROR_OK;
}

static bb_value_t baset1_read(void)
{
	const uint8_t phy_id = mdio_ctx->phy_id;
	const uint8_t target = mdio_ctx->target_cpu;
	uint16_t reg = mdio_read(phy_id, MDIO_JTAG_DEV, MDIO_JTAG_REG);
	return (reg & JTAG_TDO(target)) ?  BB_HIGH : BB_LOW;
}

static int baset1_write(int tck, int tms, int tdi)
{
	const uint8_t phy_id = mdio_ctx->phy_id;
	const uint8_t target = mdio_ctx->target_cpu;
	uint16_t jtag_reg = mdio_ctx->jtag_reg;

	if (tdi)
		jtag_reg |= JTAG_TDI(target);
	else
		jtag_reg &= ~JTAG_TDI(target);

	if (tck)
		jtag_reg |= JTAG_TCK(target);
	else
		jtag_reg &= ~JTAG_TCK(target);

	if (tms)
		jtag_reg |= JTAG_TMS(target);
	else
		jtag_reg &= ~JTAG_TMS(target);

	mdio_write(phy_id, MDIO_JTAG_DEV, MDIO_JTAG_REG, jtag_reg);
	mdio_ctx->jtag_reg = jtag_reg;
	return ERROR_OK;
}

static int baset1_reset(int trst, int srst)
{
	const uint8_t phy_id = mdio_ctx->phy_id;
	const uint8_t target = mdio_ctx->target_cpu;
	uint16_t jtag_reg = mdio_ctx->jtag_reg;

	if (trst)
		jtag_reg |= JTAG_RST(target);
	else
		jtag_reg &= ~JTAG_RST(target);

	if (srst)
		LOG_WARNING("Warn: baset1 dose nothing with jtag srst\n");

	mdio_write(phy_id, MDIO_JTAG_DEV, MDIO_JTAG_REG, jtag_reg);
	mdio_ctx->jtag_reg = jtag_reg;
	return ERROR_OK;
}

COMMAND_HANDLER(baset1_cmd_preinit)
{
	mdio_init();
	return ERROR_OK;
}

COMMAND_HANDLER(baset1_mdio_write)
{
	uint16_t phy, dev, reg, val;
	if (CMD_ARGC < 4) return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], phy);
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], dev);
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[2], reg);
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[3], val);

	mdio_write(phy, dev, reg, val);
#if !DEBUG_BASET1_MDIO_WRITE
	printf("BASET1-MDIO: <write> => "
		"phy:0x%x, dev:0x%x, reg:0x%04x | val:0x%04x\n",
		phy, dev, reg, val);
#endif
	return ERROR_OK;
}

COMMAND_HANDLER(baset1_mdio_read)
{

	uint16_t phy, dev, reg;
	if (CMD_ARGC < 3) return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], phy);
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], dev);
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[2], reg);

#if !DEBUG_BASET1_MDIO_WRITE
	uint16_t val = mdio_read(phy, dev, reg);
	printf("BASET1-MDIO: <read>  => "
		"phy:0x%x, dev:0x%x, reg:0x%04x | val:0x%04x\n",
		phy, dev, reg, val);
#else
	mdio_read(phy, dev, reg);
#endif
	return ERROR_OK;
}

COMMAND_HANDLER(baset1_target_phy_id)
{

	uint8_t phy;
	if (CMD_ARGC < 1) return ERROR_COMMAND_SYNTAX_ERROR;
	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], phy);
	mdio_ctx->phy_id = phy;
	return ERROR_OK;
}

COMMAND_HANDLER(baset1_handle_vid_pid_command)
{
	if (CMD_ARGC > MAX_USB_IDS * 2) {
		LOG_WARNING("ignoring extra IDs in ftdi_vid_pid "
			"(maximum is %d pairs)", MAX_USB_IDS);
		CMD_ARGC = MAX_USB_IDS * 2;
	}
	if (CMD_ARGC < 2 || (CMD_ARGC & 1)) {
		LOG_WARNING("incomplete ftdi_vid_pid configuration directive");
		if (CMD_ARGC < 2)
			return ERROR_COMMAND_SYNTAX_ERROR;
		/* remove the incomplete trailing id */
		CMD_ARGC -= 1;
	}

	unsigned i;
	for (i = 0; i < CMD_ARGC; i += 2) {
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i],
				mdio_ctx->ftdi_vid[i >> 1]);
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1],
				mdio_ctx->ftdi_pid[i >> 1]);
	}

	/*
	 * Explicitly terminate, in case there are multiples instances of
	 * ftdi_vid_pid.
	 */
	mdio_ctx->ftdi_vid[i >> 1] = mdio_ctx->ftdi_pid[i >> 1] = 0;
	return ERROR_OK;
}

COMMAND_HANDLER(baset1_target_pmu)
{
	mdio_ctx->target_cpu = TARGET_PMU;
	return ERROR_OK;
}

COMMAND_HANDLER(baset1_target_mcu)
{
	mdio_ctx->target_cpu = TARGET_MCU;
	return ERROR_OK;
}

static const struct command_registration baset1_command_handlers[] = {
	{
		.name = "preinit",
		.handler = &baset1_cmd_preinit,
		.mode = COMMAND_CONFIG,
		.help = "preinit mdio",
		.usage = "preinit",
	},
	{
		.name = "mdio_read",
		.handler = &baset1_mdio_read,
		.mode = COMMAND_CONFIG,
		.help = "mdio read",
		.usage = "(phy dev reg)* ",
	},
	{
		.name = "mdio_write",
		.handler = &baset1_mdio_write,
		.mode = COMMAND_CONFIG,
		.help = "mdio write",
		.usage = "(phy dev reg val)* ",
	},
	{
		.name = "baset1_target_phy_id",
		.handler = &baset1_target_phy_id,
		.mode = COMMAND_CONFIG,
		.help = "set target phy id",
		.usage = "(phy_id)",
	},
	{
		.name = "baset1_target_pmu",
		.handler = &baset1_target_pmu,
		.mode = COMMAND_CONFIG,
		.help = "set PMU as target cpu",
		.usage = "baset1_target_pmu",
	},
	{
		.name = "baset1_target_mcu",
		.handler = &baset1_target_mcu,
		.mode = COMMAND_CONFIG,
		.help = "set MCU as target cpu",
		.usage = "baset1_target_mcu",
	},
	{
		.name = "baset1_vid_pid",
		.handler = &baset1_handle_vid_pid_command,
		.mode = COMMAND_CONFIG,
		.help = "the vendor ID and product ID of the FTDI device",
		.usage = "(vid pid)* ",
	},
	COMMAND_REGISTRATION_DONE
};

static int baset1_speed(int speed)
{
	return ERROR_OK;
}

static int baset1_speed_div(int speed, int *khz)
{
	*khz = speed / 1000;
	return ERROR_OK;
}

static int baset1_khz(int khz, int *jtag_speed)
{
	if (khz == 0 && !mpsse_is_high_speed(mdio_ctx->mpsse_ctx)) {
		LOG_DEBUG("RCLK not supported");
		return ERROR_FAIL;
	}

	*jtag_speed = khz * 1000;
	return ERROR_OK;
}

static struct bitbang_interface baset1_bitbang = {
	.read = baset1_read,
	.write = baset1_write,
	.reset = baset1_reset,
	.blink = 0,
};

static const char * const baset1_transports[] = { "jtag", "swd", NULL };

struct jtag_interface baset1_interface = {
	.name = "baset1",
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = bitbang_execute_queue,
	.transports = baset1_transports,
	.swd = &bitbang_swd,
	.commands = baset1_command_handlers,
	.init = baset1_init,
	.quit = baset1_quit,
	.speed_div = baset1_speed_div,
	.speed = baset1_speed,
	.khz = baset1_khz,
};
