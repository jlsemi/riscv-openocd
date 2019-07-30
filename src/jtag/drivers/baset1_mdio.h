#ifndef __BASET1_MDIO_H__
#define __BASET1_MDIO_H__

#include <stdbool.h>
#include <stdint.h>

#define FAST_COMMAND_NUM	4096

#define TARGET_PMU	8
#define TARGET_MCU	0

#define JTAG_RST(target)	(1 << (target + 4))
#define JTAG_TCK(target)	(1 << (target + 3))
#define JTAG_TMS(target)	(1 << (target + 2))
#define JTAG_TDI(target)	(1 << (target + 1))
#define JTAG_TDO(target)	(1 << (target + 0))

#define MDIO_JTAG_DEV	0x1f
#define MDIO_JTAG_REG	0x10

enum mdio_mode {
	MDIO_READ,
	MDIO_WRITE,
};

typedef uint16_t offset_t;

struct mdio_ctx;

struct mdio_ops {
	uint8_t phy;
	uint8_t dev;
	uint16_t reg;
	uint16_t val;
	enum mdio_mode mode;
};

struct mdio_ctx*
mdio_init(uint16_t vid, uint16_t pid, uint8_t phy_id, uint8_t target);
void mdio_deinit(struct mdio_ctx *mdio_ctx);
int mdio_open(struct mdio_ctx *mdio_ctx);
void mdio_close(struct mdio_ctx *mdio_ctx);

/**
 * Slow functions are designed to communicate with all register.
 * The `setup` function is no need to setup preamble everytime.
 */
int mdio_slow_setup(struct mdio_ctx *ctx, struct mdio_ops *ops);
int mdio_slow_flush(struct mdio_ctx *ctx);
int mdio_slow_readback(struct mdio_ctx *ctx, uint16_t *val);
static inline uint16_t mdio_read(
	struct mdio_ctx *ctx, uint8_t phy, uint8_t dev, uint16_t reg)
{
	struct mdio_ops ops = {
		.phy = phy,
		.dev = dev,
		.reg = reg,
		.val = 0xffff,
		.mode = MDIO_READ,
	};
	uint16_t val;
	mdio_slow_setup(ctx, &ops);
	mdio_slow_flush(ctx);
	mdio_slow_readback(ctx, &val);
	return val;
}

static inline void mdio_write(
	struct mdio_ctx *ctx, uint8_t phy, uint8_t dev,
	uint16_t reg, uint16_t val)
{
	struct mdio_ops ops = {
		.phy = phy,
		.dev = dev,
		.reg = reg,
		.val = val,
		.mode = MDIO_WRITE,
	};
	mdio_slow_setup(ctx, &ops);
	mdio_slow_flush(ctx);
}

/**
 * Fast functions are designed to communicate to JTAG.
 * The `setup` function only need to override ops.mode and ops.value.
 */

/**
 * Setup modification of JTAG register.
 * if success, return index (>=0).
 * else, return negative number.
 */
int mdio_fast_add(struct mdio_ctx *ctx, enum mdio_mode mode, uint16_t val);
/* Flush commands to driver. You should call readback() after flush() */
int mdio_fast_flush(struct mdio_ctx *ctx);
/* Clean the readback queue. You should call clean() before add() */
int mdio_fast_clean(struct mdio_ctx *ctx);

int mdio_fast_fetch(struct mdio_ctx *ctx, uint16_t index, uint16_t *val);
uint16_t mdio_fast_total(struct mdio_ctx *ctx);
uint16_t mdio_fast_readback_num(struct mdio_ctx *ctx);
int mdio_fast_find_index(
	struct mdio_ctx *ctx, uint16_t read_index, uint16_t *index);

static inline int
mdio_fast_readback(struct mdio_ctx *ctx, uint16_t read_index, uint16_t *val)
{
	uint16_t index;
	if (mdio_fast_find_index(ctx, read_index, &index) != ERROR_OK)
		return ERROR_FAIL;
	return mdio_fast_fetch(ctx, index, val);
}

static inline int
mdio_fast_readback_list(struct mdio_ctx *ctx, uint16_t num, uint16_t *val)
{
	if (num > mdio_fast_readback_num(ctx))
		return ERROR_BUF_TOO_SMALL;
	for (uint16_t i = 0; i < num; i++) {
		uint16_t index;
		if (mdio_fast_find_index(ctx, i, &index) != ERROR_OK)
			return ERROR_FAIL;
		mdio_fast_fetch(ctx, index, &val[i]);
	}
	return num;
}

static inline int
mdio_fast_readback_all(struct mdio_ctx *ctx, uint16_t *val)
{
	uint16_t num = mdio_fast_readback_num(ctx);
	return mdio_fast_readback_list(ctx, num, val);
}

#define DUMP_BUF(buf, offset, len, sep)	do { \
	printf("---- sep --- %s:+%d %s\n", __FILE__, __LINE__, __func__); \
	_mdio_dump_buffer(buf, offset, len, sep); \
	printf("---- end ---\n"); \
} while(0)
void _mdio_dump_buffer(
	const uint8_t *buff, offset_t offset, uint16_t length, uint16_t sep);
#endif /* __BASET1_MDIO_H__ */
