#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include <jtag/interface.h>

#include "baset1_mdio.h"

#define CLOCK_IDLE() 0

#define DEBUG_BASET1_FAST	0

#if DEBUG_BASET1_FAST
#define ASSERT(expr)	assert(expr)
#define DEBUG(expr...)	printf("MCD: " expr)
#else
#define ASSERT(expr)
#define DEBUG(expr...)
#undef DUMP_BUF
#define DUMP_BUF(expr...)
#endif

typedef enum {
	BB_LOW,
	BB_HIGH,
	BB_ERROR
} bb_value_t;

struct baset1_ops {
	uint16_t vid;
	uint16_t pid;
	uint8_t target;
	uint8_t phy_id;
	struct mdio_ctx *mdio_ctx;
	uint16_t jtag_reg;
} baset1_ops = {
	.vid = 0,
	.pid = 0,
	.target = TARGET_PMU,
	.phy_id = 0x1a,
	.mdio_ctx = NULL,
	.jtag_reg = 0,
};

char* scan_type_name(enum scan_type type)
{
	switch (type) {
	case SCAN_IN: return "I";
	case SCAN_OUT: return "O";
	case SCAN_IO: return "IO";
	default: return "UNKNOW";
	}
}

char *bb_type_name(bb_value_t type)
{
	switch (type) {
	case BB_LOW: return "LOW";
	case BB_HIGH: return "HIGH";
	case BB_ERROR: return "ERROR";
	default: return "UNKNOW";
	}
}

static inline uint16_t get_jtag_reg(void)
{
	return baset1_ops.jtag_reg;
}

static inline void set_jtag_reg(uint16_t reg)
{
	baset1_ops.jtag_reg = reg;
}

static inline uint8_t get_target_cpu(void)
{
	return baset1_ops.target;
}

static inline struct mdio_ctx *get_mdio(void)
{
	return baset1_ops.mdio_ctx;
}

static inline bb_value_t get_mdio_tdi(uint16_t mdio_idx)
{
	uint8_t target = get_target_cpu();
	uint16_t val;
	int res = mdio_fast_fetch(get_mdio(), mdio_idx, &val);

	if (res < 0) return BB_ERROR;
	return (val & JTAG_TDO(target)) ?  BB_HIGH : BB_LOW;
}

static inline void mdio_reduce_preamble(struct baset1_ops *ops)
{
	struct mdio_ctx *mdio_ctx = ops->mdio_ctx;
	uint8_t phy = ops->phy_id;
	uint16_t mdio_cfg = mdio_read(mdio_ctx, phy, 0x1d, 0x0020);
	mdio_cfg &= ~0x3f;
	mdio_cfg |= 0x4;
	mdio_write(mdio_ctx, phy, 0x1d, 0x0020, mdio_cfg);
}

static int init_mdio(struct baset1_ops *ops)
{
	if (ops->mdio_ctx)
		return ERROR_OK;
	ops->mdio_ctx = mdio_init(ops->vid, ops->pid, ops->phy_id, ops->target);
	if (!ops->mdio_ctx) {
		LOG_ERROR("init mdio for baset1 failed");
		return ERROR_FAIL;
	}
	mdio_open(ops->mdio_ctx);
	mdio_reduce_preamble(ops);
	return ERROR_OK;
}

static int baset1_init(void)
{
	init_mdio(&baset1_ops);
	return ERROR_OK;
}

static int baset1_quit(void)
{
	mdio_deinit(baset1_ops.mdio_ctx);
	return ERROR_OK;
}

/* Functions with 'execute_queue' */
static int baset1_reset(int trst, int srst)
{
	uint8_t target = get_target_cpu();
	uint16_t jtag_reg = get_jtag_reg();
	if (trst)
		jtag_reg |= JTAG_RST(target);
	else
		jtag_reg &= ~JTAG_RST(target);

	if (srst)
		LOG_WARNING("Warn: baset1 dose nothing with jtag srst\n");

	int ret = mdio_fast_add(get_mdio(), MDIO_WRITE, jtag_reg);
	set_jtag_reg(jtag_reg);
	if (ret >= 0)
		return ERROR_OK;
	return ERROR_FAIL;
}

static int baset1_write(int tck, int tms, int tdi)
{
	/** XXX
	 * In this case, we can improve this 'write' function by
	 * setting or resetting bits of 'jtag_reg' once.
	 * BUT we don't think it will be a bottleneck. The improved
	 * version may sacrifice a bit of readability.
	 * So we leave that lumber thing along.
	 */
	uint8_t target = get_target_cpu();
	uint16_t jtag_reg = get_jtag_reg();
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

	int ret = mdio_fast_add(get_mdio(), MDIO_WRITE, jtag_reg);
	set_jtag_reg(jtag_reg);
	if (ret >= 0)
		return ERROR_OK;
	return ERROR_FAIL;
}

static inline int baset1_read(void)
{
	/* send all ones when reading */
	return mdio_fast_add(get_mdio(), MDIO_READ, 0xffff);
}

static inline void baset1_fast_clean(void)
{
	mdio_fast_clean(get_mdio());
}

static inline int baset1_fast_flush(void)
{
	return mdio_fast_flush(get_mdio());
}

static void baset1_end_state(tap_state_t state)
{
	assert(tap_is_state_stable(state));
	tap_set_end_state(state);
}

static int baset1_state_move(int skip)
{
	int i = 0, tms = 0;
	tap_state_t start = tap_get_state();
	tap_state_t end = tap_get_end_state();
	uint8_t tms_scan = tap_get_tms_path(start, end);
	int tms_count = tap_get_tms_path_len(start, end);

	for (i = skip; i < tms_count; i++) {
		tms = (tms_scan >> i) & 1;
		if (baset1_write(0, tms, 0) != ERROR_OK)
			return ERROR_FAIL;
		if (baset1_write(1, tms, 0) != ERROR_OK)
			return ERROR_FAIL;
	}
	if (baset1_write(CLOCK_IDLE(), tms, 0) != ERROR_OK)
		return ERROR_FAIL;

	tap_set_state(end);
	return ERROR_OK;
}

static inline int baset1_move_to(tap_state_t state)
{
	tap_state_t saved_end = tap_get_end_state();
	baset1_end_state(state);
	int res = baset1_state_move(0);
	baset1_end_state(saved_end);
	return res;
}

static int baset1_execute_reset(struct jtag_command *cmd)
{
	DEBUG("reset trst: %i, srst: %i\n",
		cmd->cmd.reset->trst, cmd->cmd.reset->srst);

	if (cmd->cmd.reset->trst == 1
	    || (cmd->cmd.reset->srst
		&& (jtag_get_reset_config() & RESET_SRST_PULLS_TRST)))
		tap_set_state(TAP_RESET);

	baset1_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
	return baset1_fast_flush();
}

static int baset1_execute_runtest(struct jtag_command *cmd)
{
	DEBUG("runtest %i cycles, end in %s\n",
		cmd->cmd.runtest->num_cycles,
		tap_state_name(cmd->cmd.runtest->end_state));

	baset1_end_state(cmd->cmd.runtest->end_state);
	int i;
	int num_cycles = cmd->cmd.runtest->num_cycles;
	tap_state_t saved_end_state = tap_get_end_state();

	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE) {
		baset1_end_state(TAP_IDLE);
		if (baset1_state_move(0) != ERROR_OK)
			return ERROR_FAIL;
	}

	/* execute num_cycles */
	for (i = 0; i < num_cycles; i++) {
		if (baset1_write(0, 0, 0) != ERROR_OK)
			return ERROR_FAIL;
		if (baset1_write(1, 0, 0) != ERROR_OK)
			return ERROR_FAIL;
	}
	if (baset1_write(CLOCK_IDLE(), 0, 0) != ERROR_OK)
		return ERROR_FAIL;

	/* finish in end_state */
	baset1_end_state(saved_end_state);
	if (tap_get_state() != tap_get_end_state())
		if (baset1_state_move(0) != ERROR_OK)
			return ERROR_FAIL;
	return baset1_fast_flush();
}

static int baset1_execute_statemove(struct jtag_command *cmd)
{
	DEBUG("statemove end in %s\n",
		tap_state_name(cmd->cmd.statemove->end_state));
	baset1_end_state(cmd->cmd.statemove->end_state);
	if (baset1_state_move(0) != ERROR_OK)
		return ERROR_FAIL;
	return baset1_fast_flush();
}

static int baset1_execute_pathmove(struct jtag_command *cmd)
{
	DEBUG("pathmove: %i states, end in %s\n",
		cmd->cmd.pathmove->num_states,
		tap_state_name(
			cmd->cmd.pathmove->path[
				cmd->cmd.pathmove->num_states - 1]
		)
	);
	struct pathmove_command *pcmd = cmd->cmd.pathmove;
	int num_states = pcmd->num_states;
	int state_count;
	int tms = 0;

	state_count = 0;
	while (num_states) {
		if (tap_state_transition(tap_get_state(), false) == pcmd->path[state_count])
			tms = 0;
		else if (tap_state_transition(tap_get_state(), true) == pcmd->path[state_count])
			tms = 1;
		else {
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition",
				tap_state_name(tap_get_state()),
				tap_state_name(pcmd->path[state_count]));
			exit(-1);
		}

		if (baset1_write(0, tms, 0) != ERROR_OK)
			return ERROR_FAIL;
		if (baset1_write(1, tms, 0) != ERROR_OK)
			return ERROR_FAIL;

		tap_set_state(pcmd->path[state_count]);
		state_count++;
		num_states--;
	}

	if (baset1_write(CLOCK_IDLE(), tms, 0) != ERROR_OK)
		return ERROR_FAIL;

	tap_set_end_state(tap_get_state());
	return baset1_fast_flush();
}

static int baset1_execute_scan(struct jtag_command *cmd)
{
	/**
	 * For saving time, we use some tricks here.
	 * - Apply all command once.
	 * - Save the info of read site(index of field, byte, bit) to speed up
	 *   modifying 'field.input'. It means we do not change every bit.
	 * - Use static structure to avoid allocate memory everytime.
	 * - Porvide some inline funtion to imporve readability.
	 */
	static struct {
		int read_count;
		struct sb_queue {
			int field_idx;
			int byte_cnt;
			int bit_cnt;
			uint16_t mdio_idx;
		} queue[FAST_COMMAND_NUM];
	} scan_buffer = {
		.read_count = 0,
		.queue = {},
	};

	/**
	 * Record site info of reading bit.
	 */
	inline void queue_add(int fidx, int bytec, int bitc, uint16_t mdio_idx)
	{
		int cnt = scan_buffer.read_count;
		struct sb_queue *queue = scan_buffer.queue;
		queue[cnt].field_idx = fidx;
		queue[cnt].byte_cnt = bytec;
		queue[cnt].bit_cnt = bitc;
		queue[cnt].mdio_idx = mdio_idx;
		scan_buffer.read_count++;
	}

	/**
	 * Get bits should be read back, and assemble those bits to up-level.
	 * Note:
	 * - This function should be called after calling `mdio_flush()`.
	 */
	inline int queue_apply(struct scan_field *fields)
	{
		struct sb_queue *queue = scan_buffer.queue;
		for (int i = 0; i < scan_buffer.read_count; i++, queue++) {
			struct scan_field *field = &fields[queue->field_idx];
			ASSERT(field->in_value);
			int bytec = queue->byte_cnt;
			int bitc = queue->bit_cnt;
			DEBUG("queue[%d]: field_%d[%d], bitc %x = "
				"mdio_idx %d, tdi %s\n",
				i, queue->field_idx, bytec, bitc, queue->mdio_idx,
				bb_type_name(get_mdio_tdi(queue->mdio_idx)));

			switch (get_mdio_tdi(queue->mdio_idx)) {
			case BB_LOW:
				field->in_value[bytec] &= ~bitc;
				break;
			case BB_HIGH:
				field->in_value[bytec] |= bitc;
				break;
			default:
				return ERROR_FAIL;
			}
		}
		return ERROR_OK;
	}

	/**
	 * Clean the readback queue.
	 * Note:
	 * - The user should call 'clean' before 'add'.
	 */
	inline void queue_clean(void)
	{
		scan_buffer.read_count = 0;
	}

	inline enum scan_type field_type(struct scan_field *field)
	{
		int type = 0;
		if (field->in_value) type |= SCAN_IN;
		if (field->out_value) type |= SCAN_OUT;
		return type;
	}

	DEBUG("scan %s\n", cmd->cmd.scan->ir_scan ? "IR" : "DR");
	/* Move to state */
	if (cmd->cmd.scan->ir_scan) {
		if (tap_get_state() != TAP_IRSHIFT)
			baset1_move_to(TAP_IRSHIFT);
	} else {
		if (tap_get_state() != TAP_DRSHIFT)
			baset1_move_to(TAP_DRSHIFT);
	}
	baset1_end_state(cmd->cmd.scan->end_state);
	bool tap_need_move = tap_get_state() != tap_get_end_state();

	/* Preapare command */
	queue_clean();
	int num_fields = cmd->cmd.scan->num_fields;
	struct scan_field *field = cmd->cmd.scan->fields;
	for (int fidx = 0; fidx < num_fields; fidx++, field++) {
		enum scan_type type = field_type(field);
		ASSERT(type != 0);
		DEBUG("type: %s, field: %d, bits: %d\n",
			scan_type_name(type), fidx, field->num_bits);
		DUMP_BUF(field->out_value, 0, field->num_bits / 8, 8);
		if (type == 0) {
			LOG_WARNING("UNKNOWN type\n");
			continue;
		}
		for (int bcnt = 0; bcnt < field->num_bits; bcnt++) {
			/* if necessary, we send 1 bit tms with last bit */
			int tms = tap_need_move &
				(fidx == num_fields - 1) &
				(bcnt == field->num_bits - 1);
			int bytec = bcnt / 8;
			int bitc = 1 << (bcnt & 0x7);
			/* if type is equal to SCAN_IN, we just output 'low' */
			int tdi = (type != SCAN_IN) &
				((field->out_value[bytec] & bitc) ? 1 : 0);

			baset1_write(0, tms, tdi);
			/**
			 * Capture SCAN_IN or SCAN_IO. There is no need to
			 * read back in case of SCAN_OUT.
			 */
			if (type != SCAN_OUT) {
				uint16_t mdio_idx = baset1_read();
				queue_add(fidx, bytec, bitc, mdio_idx);
			}
			baset1_write(1, tms, tdi);
		}
	}

	/* Move to state */
	if (tap_need_move) {
		/* we send tms in last bit, so we should skip it */
		if (baset1_state_move(1) != ERROR_OK)
			return ERROR_FAIL;
	}

	/* Flush a batch of commands */
	if (baset1_fast_flush() != ERROR_OK)
		return ERROR_FAIL;

	/* Assemble readback info */
	int res = queue_apply(cmd->cmd.scan->fields);
#if DEBUG_BASET1_FAST
	/* Dump all readback values for debugging */
	field = cmd->cmd.scan->fields;
	for (int i = 0; i < num_fields; i++, field++) {
		printf("field[%d]\n", i);
		DUMP_BUF(field->in_value, 0, field->num_bits / 8, 8);
	}
#endif
	return res;
}

static int baset1_execute_sleep(struct jtag_command *cmd)
{
	DEBUG("sleep %" PRIi32 "\n", cmd->cmd.sleep->us);
	jtag_sleep(cmd->cmd.sleep->us);
	return ERROR_OK;
}

static int baset1_execute_stableclocks(struct jtag_command *cmd)
{
	int num_cycles = cmd->cmd.stableclocks->num_cycles;
	int tms = (tap_get_state() == TAP_RESET ? 1 : 0);
	int i;

	/* send num_cycles clocks onto the cable */
	for (i = 0; i < num_cycles; i++) {
		if (baset1_write(1, tms, 0) != ERROR_OK)
			return ERROR_FAIL;
		if (baset1_write(0, tms, 0) != ERROR_OK)
			return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int baset1_execute_tms(struct jtag_command *cmd)
{
	unsigned num_bits = cmd->cmd.tms->num_bits;
	const uint8_t *bits = cmd->cmd.tms->bits;

	int tms = 0;
	for (unsigned i = 0; i < num_bits; i++) {
		tms = ((bits[i/8] >> (i % 8)) & 1);
		if (baset1_write(0, tms, 0) != ERROR_OK)
			return ERROR_FAIL;
		if (baset1_write(1, tms, 0) != ERROR_OK)
			return ERROR_FAIL;
	}
	if (baset1_write(CLOCK_IDLE(), tms, 0) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static inline int baset1_execute_command(struct jtag_command *cmd)
{
	int ret = ERROR_FAIL;
	baset1_fast_clean();
	switch (cmd->type) {
	case JTAG_RESET:
		ret = baset1_execute_reset(cmd);
		break;
	case JTAG_RUNTEST:
		ret = baset1_execute_runtest(cmd);
		break;
	case JTAG_TLR_RESET:
		ret = baset1_execute_statemove(cmd);
		break;
	case JTAG_PATHMOVE:
		ret = baset1_execute_pathmove(cmd);
		break;
	case JTAG_SCAN:
		ret = baset1_execute_scan(cmd);
		break;
	case JTAG_SLEEP:
		ret = baset1_execute_sleep(cmd);
		break;
	case JTAG_STABLECLOCKS:
		ret = baset1_execute_stableclocks(cmd);
		break;
	case JTAG_TMS:
		ret = baset1_execute_tms(cmd);
		break;
	default:
		LOG_ERROR("BUG: unknown JTAG command type encountered: %d",
				cmd->type);
		break;
	}
	return ret;
}

static int baset1_execute_queue(void)
{
	int retval = ERROR_OK;
	struct jtag_command *cmd = NULL;
	for (cmd = jtag_command_queue; cmd; cmd = cmd->next) {
		retval = baset1_execute_command(cmd);
	}
	if (retval != ERROR_OK)
		LOG_ERROR("error while flushing MPSSE queue: %d", retval);
	return retval;
}

COMMAND_HANDLER(baset1_target_pmu)
{
	baset1_ops.target = TARGET_PMU;
	return ERROR_OK;
}

COMMAND_HANDLER(baset1_target_mcu)
{
	baset1_ops.target = TARGET_MCU;
	return ERROR_OK;
}

COMMAND_HANDLER(baset1_handle_vid_pid_command)
{
	if (CMD_ARGC < 2 ) {
		LOG_WARNING("incomplete baset1_vid_pid configuration directive");
		if (CMD_ARGC < 2)
			return ERROR_COMMAND_SYNTAX_ERROR;
		/* remove the incomplete trailing id */
		CMD_ARGC -= 1;
	}

	uint16_t pid, vid;
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], vid);
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], pid);
	baset1_ops.vid = vid;
	baset1_ops.pid = pid;

	return ERROR_OK;
}

COMMAND_HANDLER(baset1_target_phy_id)
{
	if (CMD_ARGC < 1) return ERROR_COMMAND_SYNTAX_ERROR;
	uint8_t phy;
	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], phy);
	baset1_ops.phy_id = phy;
	return ERROR_OK;
}

COMMAND_HANDLER(baset1_cmd_preinit)
{
	init_mdio(&baset1_ops);
	return ERROR_OK;
}

COMMAND_HANDLER(baset1_mdio_read)
{
	uint16_t phy, dev, reg;
	if (CMD_ARGC < 3) return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], phy);
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], dev);
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[2], reg);

	struct mdio_ctx *mdio_ctx = get_mdio();
	uint16_t val = mdio_read(mdio_ctx, phy, dev, reg);
	printf("BASET1-MDIO: <read>  => "
		"phy:0x%x, dev:0x%x, reg:0x%04x | val:0x%04x\n",
		phy, dev, reg, val);
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

	struct mdio_ctx *mdio_ctx = get_mdio();
	mdio_write(mdio_ctx, phy, dev, reg, val);
	printf("BASET1-MDIO: <write> => "
		"phy:0x%x, dev:0x%x, reg:0x%04x | val:0x%04x\n",
		phy, dev, reg, val);
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
	COMMAND_REGISTRATION_DONE
};

static const char * const baset1_transports[] = { "jtag",  NULL };

struct jtag_interface baset1_fast_interface = {
	.name = "baset1_fast",
	.supported = DEBUG_CAP_TMS_SEQ,
	.transports = baset1_transports,
	.execute_queue = baset1_execute_queue,

	.commands = baset1_command_handlers,
	.init = baset1_init,
	.quit = baset1_quit,
};
