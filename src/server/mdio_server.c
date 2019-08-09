#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>

#include "tcl_server.h"
#include <target/target.h>
#include <helper/binarybuffer.h>
#include <jtag/drivers/baset1_mdio.h>
#include <jtag/drivers/baset1_fast.h>

#define DEBUG_MDIO_SERVER	0

#if DEBUG_MDIO_SERVER
#define ASSERT(expr)	assert(expr)
#define DEBUG(expr...)	printf("MdioServer: " expr)
#else
#define ASSERT(expr)
#define DEBUG(expr...)
#endif

static char *mdio_port;

static int mdio_new_connection(struct connection *connection);
static int mdio_input(struct connection *connection);
static int mdio_closed(struct connection *connection);
int mdio_output(struct connection *connection, const void *buf, ssize_t len);

static int mdio_new_connection(struct connection *connection)
{
	DEBUG("mdio_init\n");
	driver_mdio_init();
	return ERROR_OK;
}

static int mdio_closed(struct connection *connection)
{
	DEBUG("mdio_closed\n");
	return ERROR_OK;
}

int mdio_output(struct connection *connection, const void *data, ssize_t len)
{
	ssize_t wlen;
	wlen = connection_write(connection, data, len);

	if (wlen == len)
		return ERROR_OK;

	LOG_ERROR("error during write: %d != %d", (int)wlen, (int)len);
	return ERROR_SERVER_REMOTE_CLOSED;
}

static int find_next_char(char aim, char *in, ssize_t size, ssize_t offset)
{
	while (offset < size) {
		char c = in[offset];
		if (c == aim)
			return offset + 1;
		else if (c == '\0')
			return size;
		offset += 1;
	}
	return size;
}

static int parse_args(
	char *msg, ssize_t size, enum mdio_mode *mode,
	uint8_t *phy, uint8_t *dev, uint16_t *reg, uint16_t *val)
{
	int expect = 0;
	switch(msg[0]) {
	case 'r':
	case 'R':
		*mode = MDIO_READ;
		expect = 3;
		break;
	case 'w':
	case 'W':
		*mode = MDIO_WRITE;
		expect = 4;
		break;
	default:
		return ERROR_FAIL;
	}

	int idx = 0;
	uint16_t arr[4] = {0};
	ssize_t offset = find_next_char(':', msg, size, 0);
	do {
		uint16_t v = strtoul(&msg[offset], NULL, 16);
		arr[idx++] = v;
		if (idx >= expect)
			break;

		offset = find_next_char(',', msg, size, offset);
		if (offset < 0 || offset >= size)
			break;
	} while (true);
	DEBUG("find %d args\n", idx);
	for (int i = 0; i < idx; i++) {
		DEBUG("val: 0x%04x\n", arr[i]);
	}
	if (idx < expect)
		return ERROR_FAIL;

	*phy = arr[0];
	*dev = arr[1];
	*reg = arr[2];
	*val = arr[3];
	return ERROR_OK;
}

static int mdio_input(struct connection *connection)
{
	int res = 0;
	ssize_t rlen;
	ssize_t tlen;
	char in[256];
	char out[256];
	enum mdio_mode mode;
	uint8_t phy, dev;
	uint16_t reg, val;

	DEBUG("mdio_input\n");
	rlen = connection_read(connection, &in, sizeof(in));
	if (rlen <= 0) {
		if (rlen < 0)
			LOG_ERROR("error during read: %s", strerror(errno));
		return ERROR_SERVER_REMOTE_CLOSED;
	}
	DEBUG("mdio_got ==> %ld bytes: %s\n", rlen, in);
	res = parse_args(in, rlen, &mode, &phy, &dev, &reg, &val);
	if (res != ERROR_OK)
		return res;

	switch (mode) {
	case MDIO_WRITE:
		driver_mdio_write(phy, dev, reg, val);
		tlen = sprintf(out, "W Done");
		mdio_output(connection, out, tlen);
		break;
	case MDIO_READ:
		val = driver_mdio_read(phy, dev, reg);
		tlen = sprintf(out, "R Done: 0x%04x", val);
		mdio_output(connection, out, tlen);
		break;
	}
	return ERROR_OK;
}

int mdio_service_init(void)
{
	if (strcmp(mdio_port, "disabled") == 0) {
		LOG_INFO("mdio server disabled");
		return ERROR_OK;
	}

	return add_service("mdio", mdio_port, CONNECTION_LIMIT_UNLIMITED,
		&mdio_new_connection, &mdio_input,
		&mdio_closed, NULL);
}

static const struct command_registration mdio_command_handlers[] = {
	COMMAND_REGISTRATION_DONE
};

int mdio_register_commands(struct command_context *cmd_ctx)
{
	mdio_port = strdup("7777");
	return register_commands(cmd_ctx, NULL, mdio_command_handlers);
}

void mdio_service_free(void)
{
	free(mdio_port);
}
