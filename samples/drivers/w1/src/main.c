/* Zephyr includes */
#include <device.h>
#include <devicetree.h>
#include <drivers/w1.h>
#include <drivers/gpio.h>
#include <drivers/gpio/gpio_ds2413.h>
#include <logging/log.h>
#include <zephyr.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

const struct device *dev_w1 = DEVICE_DT_GET(DT_NODELABEL(w1));
const struct device *dev_w1_gpio = DEVICE_DT_GET(DT_NODELABEL(w1_gpio));

void search_callback(struct w1_rom rom, void *cb_arg)
{
	int ret;

	ret = gpio_drv_cmd(dev_w1_gpio, DS2413_CMD_SET_ROM, &rom, sizeof(rom));
	LOG_INF("ret: %d", ret);
}

void main(void)
{
	LOG_INF("Build time: " __DATE__ " " __TIME__);

	w1_lock_bus(dev_w1);
	size_t num_devices = w1_search_rom(dev_w1, search_callback, NULL);
	LOG_INF("Number of devices found on bus: %u", num_devices);
	w1_unlock_bus(dev_w1);
}
