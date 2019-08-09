#ifndef __BASET1_FAST_H__
#define __BASET1_FAST_H__

int driver_mdio_init(void);
uint16_t driver_mdio_read(uint8_t phy, uint8_t dev, uint16_t reg);
void driver_mdio_write(uint8_t phy, uint8_t dev, uint16_t reg, uint16_t val);
#endif /* __BASET1_FAST_H__ */
