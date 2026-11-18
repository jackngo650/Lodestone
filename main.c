/*
 * Lodestone dummy node: Heltec Mesh Node T114 (nRF52840 + SX1262), Zephyr / nRF Connect SDK.
 *
 * Every ~10 s it broadcasts one beacon that the Heltec receiver shows on its
 * "nodes" page:
 *
 * BCN <name> <seq> <lat_e6> <lon_e6> <fix>
 *
 * name : up to 7 characters, no spaces (NODE_NAME)
 * seq : counts up from 0
 * lat/lon : millionths of a degree (42393521 = 42.393521)
 * fix : 1 = position is valid
 *
 * The position is FIXED: NODE_NORTH_M / NODE_EAST_M metres from the Heltec
 * receiver's HOME spot. Nothing else needs to be connected.
 *
 * Radio settings match the rest of the system:
 * 915 MHz, 125 kHz BW, SF9, CR 4/5, sync word 0x12 (private), preamble 8
 *
 * The green LED blinks once per beacon, so you can see it working even if no
 * USB serial port shows up. Attach the antenna before powering the board.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* ---------------- who this node is ---------------- */

#define NODE_NAME "T114" /* max 7 characters, no spaces */
#define NODE_NORTH_M (-30) /* metres north of HOME (negative = south) */
#define NODE_EAST_M 50 /* metres east of HOME (negative = west) */

/* HOME = where the Heltec receiver sits. Keep identical to its HOME_LAT_E6 / HOME_LON_E6. */
#define HOME_LAT_E6 42393521
#define HOME_LON_E6 (-72528341)

#define TX_POWER_DBM 10
#define BEACON_PERIOD_MS 10000
#define BEACON_JITTER_MS 2000 /* +/- so several nodes don't collide */

/* ---------------- hardware ---------------- */

/* The board's devicetree normally has the alias lora0; fall back to "any SX1262 node" */
#if DT_NODE_EXISTS(DT_ALIAS(lora0))
#define LORA_DT_NODE DT_ALIAS(lora0)
#else
#define LORA_DT_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(semtech_sx1262)
#endif

static const struct device *const lora = DEVICE_DT_GET(LORA_DT_NODE);

#if DT_NODE_EXISTS(DT_ALIAS(led0))
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#define HAVE_LED 1
#else
#define HAVE_LED 0
#endif

static struct lora_modem_config cfg = {
	.frequency = 915000000, /* US 915 MHz band */
	.bandwidth = BW_125_KHZ,
	.datarate = SF_9,
	.preamble_len = 8,
	.coding_rate = CR_4_5,
	.iq_inverted = false,
	.public_network = false, /* private sync word 0x12, same as RadioLib's default private */
	.tx_power = TX_POWER_DBM,
	.tx = true,
};

int main(void)
{
	char msg[64];
	unsigned int seq = 0;

#if HAVE_LED
	if (gpio_is_ready_dt(&led)) {
		gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	}
#endif

	if (!device_is_ready(lora)) {
		printk("LoRa radio not ready (is this the T114 v2 build target?)\n");
		return 0;
	}

	int ret = lora_config(lora, &cfg);

	if (ret < 0) {
		printk("lora_config failed: %d\n", ret);
		return 0;
	}

	/* Fixed position: metres -> millionths of a degree */
	const float lat_deg = HOME_LAT_E6 / 1e6f;
	const int lat_e6 = HOME_LAT_E6 + (int)lroundf(NODE_NORTH_M / 111320.0f * 1e6f);
	const int lon_e6 = HOME_LON_E6 +
		(int)lroundf(NODE_EAST_M / (111320.0f * cosf(lat_deg * 0.017453293f)) * 1e6f);

	printk("\nLodestone dummy node \"%s\" ready: beacon every %d s at %d, %d\n",
	       NODE_NAME, BEACON_PERIOD_MS / 1000, lat_e6, lon_e6);

	while (1) {
		int len = snprintk(msg, sizeof(msg), "BCN %s %u %d %d %d",
				   NODE_NAME, seq, lat_e6, lon_e6, 1);

		ret = lora_send(lora, (uint8_t *)msg, len);
		if (ret < 0) {
			printk("TX FAILED (%d)\n", ret);
		} else {
			printk("TX \"%s\"\n", msg);
#if HAVE_LED
			gpio_pin_set_dt(&led, 1);
			k_msleep(60);
			gpio_pin_set_dt(&led, 0);
#endif
		}
		seq++;

		/* period +/- a pseudo-random jitter */
		int jitter = (int)(k_cycle_get_32() % (2 * BEACON_JITTER_MS + 1)) - BEACON_JITTER_MS;

		k_msleep(BEACON_PERIOD_MS + jitter);
	}

	return 0;
}
