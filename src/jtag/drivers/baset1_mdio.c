#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "mpsse.h"
#include "jtag/jtag.h"
#include "helper/log.h"

#include "baset1_mdio.h"

#define DEBUG_BASET1 0

#define OP_SIZE			4
#define PREAMBLE_SIZE		4
#define FAST_PREAMBLE_SIZE	1
/* with 32 bits preamble and 32 bits command */
#define MDIO_COMMAND_SIZE	((PREAMBLE_SIZE + OP_SIZE) * 2)
/* with 8 bits preamble and 32 bits command */
#define MDIO_FAST_COMMAND_SIZE	((FAST_PREAMBLE_SIZE + OP_SIZE) * 2)


#define MDIO_MODE	(MSB_FIRST | POS_EDGE_OUT | NEG_EDGE_IN)
#define FTID_OUTPUT	0x03
#define FTID_DIRECTION	0x03

#define MAX_BUFF_SIZE	(MDIO_FAST_COMMAND_SIZE * FAST_COMMAND_NUM)

#if DEBUG_BASET1
#define DUMP_BUF_SLOW(buf, offset, len) \
	DUMP_BUF(buf, offset, len, MDIO_COMMAND_SIZE / 2)
#define DUMP_BUF_FAST(buf, offset, len) \
	DUMP_BUF(buf, offset, len, MDIO_FAST_COMMAND_SIZE / 2)
#else
#define DUMP_BUF_SLOW(buf, offset, len)
#define DUMP_BUF_FAST(buf, offset, len)
#endif

void _mdio_dump_buffer(
	const uint8_t *buff, offset_t offset, uint16_t length, uint16_t sep)
{
	int i = 0;
	for (i = 0; i < length; i++) {
		if (i % sep == 0) printf("\n");
		printf("%02x\t", buff[i + offset]);
	}
	printf("\n");
}

struct mdio_ctx {
	uint8_t phy_id;
	uint8_t target_cpu;

	/* vid = pid = 0 marks the end of the list */
	uint16_t ftdi_vid;
	uint16_t ftdi_pid;

	/* fast input/output */
	uint8_t finput[MAX_BUFF_SIZE];
	uint8_t foutput[MAX_BUFF_SIZE];
	uint16_t findex;
	uint16_t ftotal;
	uint16_t fread_cnt;
	uint16_t fread_queue[FAST_COMMAND_NUM];

	/* slow input/output */
	uint8_t sinput[MDIO_COMMAND_SIZE];
	uint8_t soutput[MDIO_COMMAND_SIZE];

	bool is_running;
	struct mpsse_ctx *mpsse_ctx;
};

static inline offset_t _mdio_preamble(uint8_t *out, offset_t offset)
{
	out[offset++] = 0xff;
	out[offset++] = 0xff;
	out[offset++] = 0xff;
	out[offset++] = 0xff;
	return offset;
}

static inline offset_t _mdio_preamble_fast(uint8_t *out, offset_t offset)
{
	out[offset++] = 0xff;
	return offset;
}

static inline uint16_t _mdio_bytes_to_u16(uint8_t *input, offset_t offset)
{
	return (input[offset] << 8) | input[offset + 1];
}

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
	uint8_t phy, uint8_t dev, enum mdio_mode mode, uint16_t val)
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

/* slow functions */
static inline offset_t
_prepare_mdio_slow(uint8_t *output, offset_t offset, struct mdio_ops *ops)
{
	offset_t end = offset;
	end = _mdio_preamble(output, end);
	end = _mdio_command_address(output, end, ops->phy, ops->dev, ops->reg);
	end = _mdio_preamble(output, end);
	end = _mdio_command_value(
		output, end, ops->phy, ops->dev, ops->mode, ops->val);
	return end;
}

static inline void
_setup_mdio_slow(uint8_t *output, offset_t offset, struct mdio_ops *ops)
{
	offset = offset + PREAMBLE_SIZE;
	offset = _mdio_command_address(
		output, offset, ops->phy, ops->dev, ops->reg);
	offset = offset + PREAMBLE_SIZE;
	offset = _mdio_command_value(
		output, offset, ops->phy, ops->dev, ops->mode, ops->val);
}

int mdio_slow_setup(struct mdio_ctx *ctx, struct mdio_ops *ops)
{
	_setup_mdio_slow(ctx->soutput, 0, ops);
	return ERROR_OK;
}

int mdio_slow_flush(struct mdio_ctx *ctx)
{
	mpsse_clock_data(
		ctx->mpsse_ctx,
		ctx->soutput, 0,
		ctx->sinput, 0,
		MDIO_COMMAND_SIZE * 8,
		MDIO_MODE
	);
	return mpsse_flush(ctx->mpsse_ctx);
}

int mdio_slow_readback(struct mdio_ctx *ctx, uint16_t *val)
{
	*val = _mdio_bytes_to_u16(ctx->sinput, MDIO_COMMAND_SIZE - 2);
	return ERROR_OK;
}

/* fast functions */
static inline offset_t
_prepare_mdio_fast(uint8_t *output, offset_t offset, struct mdio_ops *ops)
{
	offset_t end = offset;
	end = _mdio_preamble_fast(output, end);
	end = _mdio_command_address(output, end, ops->phy, ops->dev, ops->reg);
	end = _mdio_preamble_fast(output, end);
	end = _mdio_command_value(
		output, end, ops->phy, ops->dev, ops->mode, ops->val);
	return end;
}

static inline void
_setup_mdio_fast(uint8_t *output, offset_t offset, struct mdio_ops *ops)
{
	uint16_t val = 0xffff;
	/* only modify the last 32bits */
	offset = offset + FAST_PREAMBLE_SIZE + OP_SIZE + FAST_PREAMBLE_SIZE;
	output[offset+0] &= ~0xf0;
	if (ops->mode == MDIO_WRITE) {
		output[offset+0] |= 0x10;
		val = ops->val;
	} else {
		output[offset+0] |= 0x30;
	}
	output[offset+2] = (val >> 8) & 0xff;
	output[offset+3] = val & 0xff;
}

int mdio_fast_add(struct mdio_ctx *ctx, enum mdio_mode mode, uint16_t val)
{
	if (ctx->findex >= FAST_COMMAND_NUM)
		return ERROR_BUF_TOO_SMALL;

	struct mdio_ops ops = {
		.mode = mode,
		.val = val,
	};
	uint8_t *output = ctx->foutput;
	uint16_t index = ctx->findex;
	offset_t offset = index * MDIO_FAST_COMMAND_SIZE;
	_setup_mdio_fast(output, offset, &ops);
	DUMP_BUF_FAST(output, offset, MDIO_FAST_COMMAND_SIZE);

	if (mode != MDIO_WRITE)
		ctx->fread_queue[ctx->fread_cnt++] = index;

	ctx->findex = index + 1;
	return index;
}

int mdio_fast_flush(struct mdio_ctx *ctx)
{
	mpsse_clock_data(
		ctx->mpsse_ctx,
		ctx->foutput, 0,
		ctx->finput, 0,
		ctx->findex * MDIO_FAST_COMMAND_SIZE * 8,
		MDIO_MODE
	);
	int res = mpsse_flush(ctx->mpsse_ctx);
	if (res != ERROR_OK)
		return res;

	ctx->ftotal = ctx->findex;
	ctx->findex = 0;
	return ERROR_OK;
}

int mdio_fast_clean(struct mdio_ctx *ctx)
{
	ctx->ftotal = 0;
	ctx->fread_cnt = 0;
	return ERROR_OK;
}

int mdio_fast_fetch(struct mdio_ctx *ctx, uint16_t index, uint16_t *val)
{
	if (index >= ctx->ftotal)
		return ERROR_BUF_TOO_SMALL;
	uint16_t read_idx = index + 1;
	offset_t offset = read_idx * MDIO_FAST_COMMAND_SIZE;
	*val = _mdio_bytes_to_u16(ctx->finput, offset - 2);
	return 1;
}

uint16_t mdio_fast_total(struct mdio_ctx *ctx)
{
	return ctx->ftotal;
}

uint16_t mdio_fast_readback_num(struct mdio_ctx *ctx)
{
	return ctx->fread_cnt;
}

int mdio_fast_find_index(
	struct mdio_ctx *ctx, uint16_t read_index, uint16_t *index)
{
	if (read_index >= ctx->fread_cnt)
		return ERROR_BUF_TOO_SMALL;
	*index = ctx->fread_queue[read_index];
	return ERROR_OK;
}

static inline int init_mpsse(struct mdio_ctx *mdio_ctx)
{
	struct mpsse_ctx *mpsse_ctx = mpsse_open(
		&mdio_ctx->ftdi_vid,
		&mdio_ctx->ftdi_pid,
		NULL, NULL, NULL, 0);
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

	mpsse_set_data_bits_low_byte(mpsse_ctx, FTID_OUTPUT, FTID_DIRECTION);
	mpsse_flush(mpsse_ctx);
	return ERROR_OK;
}

static inline void init_mdio_queue(struct mdio_ctx *ctx)
{
	/* setup fast queue on default */
	ctx->findex = 0;
	struct mdio_ops ops = {
		.phy = ctx->phy_id,
		.dev = MDIO_JTAG_DEV,
		.reg = MDIO_JTAG_REG,
		.mode = MDIO_READ,
		.val = 0,
	};

	_prepare_mdio_slow(ctx->soutput, 0, &ops);

	uint8_t *foutput = ctx->foutput;
	offset_t foffset = 0;
	for (uint32_t i = 0; i < FAST_COMMAND_NUM; i++) {
		foffset = _prepare_mdio_fast(foutput, foffset, &ops);
	}
}

/* construct and destroy */
struct mdio_ctx*
mdio_init(uint16_t vid, uint16_t pid, uint8_t phy_id, uint8_t target)
{
	struct mdio_ctx *ctx = calloc(1, sizeof(*ctx));
	if (!ctx)
		return NULL;

	ctx->ftdi_vid = vid;
	ctx->ftdi_pid = pid;
	ctx->phy_id = phy_id;
	ctx->target_cpu = target;

	ctx->mpsse_ctx = NULL;
	ctx->is_running = false;
	ctx->findex = 0;
	ctx->fread_cnt = 0;

	return ctx;
}

int mdio_open(struct mdio_ctx *mdio_ctx)
{
	if (mdio_ctx->is_running)
		return ERROR_OK;

	init_mpsse(mdio_ctx);
	init_mdio_queue(mdio_ctx);
	mdio_ctx->is_running = true;
	return ERROR_OK;
}

void mdio_close(struct mdio_ctx *mdio_ctx)
{
	mpsse_close(mdio_ctx->mpsse_ctx);
	mdio_ctx->is_running = false;
}

void mdio_deinit(struct mdio_ctx *mdio_ctx)
{
	if(!mdio_ctx)
		return;
	if (mdio_ctx->is_running)
		mdio_close(mdio_ctx);
	free(mdio_ctx);
}
