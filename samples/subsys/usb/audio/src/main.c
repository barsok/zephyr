/*
 * TODO
 */

#include <zephyr.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(main);

extern void usb_log_status(void);

void main(void)
{
	LOG_INF("entered main.");


	while(1)
	{
		usb_log_status();
		k_sleep(200);
	}
}

