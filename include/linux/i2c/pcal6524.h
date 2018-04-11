/* platform data for the PCAL6524 16-bit I/O expander driver */

#ifndef _PCAL6524_H_
#define _PCAL6524_H_

#define DRV_NAME	"pcal6524-gpio"

#define PCAL6524_INPUT			0x00  /* Input port [RO]                            */
#define PCAL6524_DAT_OUT		0x04  /* GPIO DATA OUT [R/W]                        */
#define PCAL6524_POLARITY		0x08  /* Polarity Inversion port [R/W]              */
#define PCAL6524_CONFIG			0x0C  /* Configuration port [R/W]                   */
#define PCAL6524_DRIVE0			0x40  /* Output drive strength register Port0 [R/W] */
#define PCAL6524_DRIVE1			0x42  /* Output drive strength register Port1 [R/W] */
#define PCAL6524_DRIVE2			0x44  /* Output drive strength register Port1 [R/W] */
#define PCAL6524_INPUT_LATCH		0x48  /* Port0 Input latch register  [R/W]          */
#define PCAL6524_EN_PULLUPDOWN		0x4C  /* Port0 Pull-up/Pull-down Enable [R/W]       */
#define PCAL6524_SEL_PULLUPDOWN		0x50  /* Port0 Pull-up/Pull-down selection [R/W]    */
#define PCAL6524_INT_MASK		0x54  /* Interrupt mask register [R/W]              */
#define PCAL6524_INT_STATUS		0x58  /* Interrupt status register [RO]             */
#define PCAL6524_OUTPUT_CONFIG		0x5C  /* Output port configuration register [R/W]   */

#define NO_PULL				0x00
#define PULL_DOWN			0x01
#define PULL_UP				0x02

#define POWER_ON			1
#define POWER_OFF			0

#define PCAL6524_PORT_CNT		3
#define PCAL6524_MAX_GPIO		23

#define NUM_PCAL6524_CHIP		2

/* EXPANDER GPIO Drive Strength */
enum {
	GPIO_CFG_6_25MA,
	GPIO_CFG_12_5MA,
	GPIO_CFG_18_75MA,
	GPIO_CFG_25MA,
};

struct pcal6524_platform_data {
	/* number of the first GPIO */
	int gpio_start;
	int ngpio;
	int irq_base;
	int irq_gpio;
	int reset_gpio;
	int addr;
	uint32_t support_init;
	uint32_t init_config;
	uint32_t init_data_out;
	uint32_t init_en_pull;
	uint32_t init_sel_pull;
	struct regulator *vdd;
};

struct pcal6524_chip {
	struct i2c_client *client;
	struct device *expander;
	struct gpio_chip gpio_chip;
	struct dentry	*debugfs_reg;
	struct mutex lock;
	struct pcal6524_platform_data *pdata;

	uint8_t reg_output[3];
	uint8_t reg_polarity[3];
	uint8_t reg_config[3];
	uint8_t reg_drive0[2];
	uint8_t reg_drive1[2];
	uint8_t reg_drive2[2];
	uint8_t reg_inputlatch[3];
	uint8_t reg_enpullupdown[3];
	uint8_t reg_selpullupdown[3];
/*	uint8_t reg_intmask[3];*/
	uint8_t reg_outputconfig;
};

#endif
