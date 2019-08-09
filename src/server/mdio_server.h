#ifndef OPENOCD_SERVER_MDIO_SERVER_H
#define OPENOCD_SERVER_MDIO_SERVER_H

#include <server/server.h>

#if BUILD_JLBASET1 == 1
int mdio_service_init(void);
int mdio_register_commands(struct command_context *cmd_ctx);
void mdio_service_free(void);
#else
#include <stdbool.h>
#define mdio_service_init()
#define mdio_register_commands(expr) (true)
#define mdio_service_free()
#endif

#endif /* OPENOCD_SERVER_MDIO_SERVER_H */
