
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>
#include <zephyr/sys/util.h>

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
#define SPI_1_NODE	DT_NODELABEL(spi1)
#define LED0_NODE DT_NODELABEL(led0)
#define SAMP_BITS 12
#define NUM_BITS 16
#define NUM_SAMPLES 32//pow(2,SAMP_BITS)
#define FULL_SCALE 5
#define MAX_NUM pow(2,NUM_BITS)

const struct gpio_dt_spec reset = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, reset_gpios);
const struct gpio_dt_spec start = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, start_gpios);
const struct gpio_dt_spec drdy = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, drdy_gpios);
const struct gpio_dt_spec en = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, en_gpios);
const struct gpio_dt_spec done = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, done_gpios);
const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
const struct device *const dev = DEVICE_DT_GET(SPI_1_NODE);

struct spi_cs_control cs_ctrl = (struct spi_cs_control){
	.gpio = GPIO_DT_SPEC_GET(SPI_1_NODE, cs_gpios),
	.delay = 0u,
};

uint8_t spiData[4] = {0x0000};

uint16_t adcRX, cSample;
uint16_t sampleCounter;

uint32_t sumSAMP;
int32_t vMIN, vMAX, vDC, pk_pk,	vRMS;
int32_t dataOutput[5] ;//= {0,0,0,0,0};

bool  dataReady;

static struct gpio_callback drdy_cb_data;

void setSTART(const bool state){

	uint8_t value = (uint8_t)(state ? 1 : 0);
	gpio_pin_set_dt(&start, value);
	if(state == true){
		printf("ADC Start Request\n");
	}else printf("ADC Stop Request\n");
}

void setRESET(const bool state) {
	printf("ADC Reset Request\n");
	uint8_t value = (uint8_t)(state ? 1 : 0);
	gpio_pin_set_dt(&reset, value);
}

void adcSetup(void){
	uint8_t *tmpBuffer;

	setRESET (true);
	setSTART (false);

	printf("Setting up ADC...\n");
	tmpBuffer = (uint8_t [4]){0x8, 0x5,0x0,0x3};
	SPI_TRX(dev, &cs_ctrl, tmpBuffer);
	k_sleep(K_MSEC(200));

	tmpBuffer = (uint8_t [4]){0x8, 0x6,0x0,0x8};
	SPI_TRX(dev, &cs_ctrl, tmpBuffer);
	k_sleep(K_MSEC(200));

	tmpBuffer = (uint8_t [4]){0x8, 0x7,0x0,0xB};
	SPI_TRX(dev, &cs_ctrl, tmpBuffer);
	k_sleep(K_MSEC(200));

	tmpBuffer = (uint8_t [4]){0x8, 0x8,0x0,0x8};
	SPI_TRX(dev, &cs_ctrl, tmpBuffer);
	k_sleep(K_MSEC(200));

	printf("Done Setting up ADC...\n");
}// Get data from ADC

void SPI_TRX(const struct device *dev, struct spi_cs_control *cs, uint8_t packet[]){
	struct spi_config config;

	config.frequency = 1000000;
	config.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8);
	config.slave = 0;
	config.cs = *cs;

	enum { datacount = 4 };
	uint8_t buff[datacount] = { packet[0], packet[1], packet[2], packet[3]};
	uint8_t rxdata[2];

	struct spi_buf tx_buf[1] = {
		{.buf = buff, .len = datacount},
	};
	struct spi_buf rx_buf[1] = {
		{.buf = rxdata, .len = datacount},
	};

	struct spi_buf_set tx_set = { .buffers = tx_buf, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = rx_buf, .count = 1 };

	int ret = spi_transceive(dev, &config, &tx_set, &rx_set);

	printf(" tx (i)  : %01x %01x %01x %01x \n",
	       buff[0], buff[1], buff[2], buff[3]);
	printf(" rx (i)  : %02x %02x \n",
	       rxdata[0], rxdata[1]);
    // Convert the 16 bit data from ADC into a UINT16_t value
	adcRX = (uint16_t)(rxdata[0] << 8 | rxdata[1]);

}

void resetVars(void){
	printf("Resetting Variable Values for Next Window\n");
	vMIN =0;
	vMAX = 0;
	sumSAMP = 0;
	vRMS = 0;
	vDC = 0;
	pk_pk = 0;
	sampleCounter = NUM_SAMPLES;
}

void calculateResult(void){

	pk_pk 		  = vMAX - vMIN;
	vRMS 		  = (sumSAMP / MAX_NUM)*sqrt(2);

	dataOutput[0] = ((vMIN/MAX_NUM) * FULL_SCALE); 		// minimum voltage converted from ADC value to voltage
	dataOutput[1] = ((vMAX/MAX_NUM) * FULL_SCALE);   	// maximum voltage converted from ADC value to voltage
	dataOutput[2] = ((pk_pk/MAX_NUM) * FULL_SCALE); 	// Pk-pk voltage from ADC value
	dataOutput[3] = ((vDC/MAX_NUM) * FULL_SCALE);		// DC voltage form ADC value
	dataOutput[4] = ((vRMS/MAX_NUM) * FULL_SCALE);      // VRMS form ADC values * sqrt(2)
	printf("vMin: %4x vMax: %4x Vpk-pk: %4x VDC: %4x vRMS: %4x\n", dataOutput[0],
	       dataOutput[1], dataOutput[2], dataOutput[3], dataOutput[4]);
	k_sleep(K_MSEC(10000)); // just using for debug to keep output visible for 10 seconds
}

void drdyEdge(const struct device *dev, struct gpio_callback *cb, uint32_t pins){
	printf("Drdy Edge GPIO Call\n");
	SPI_TRX(dev, &cs_ctrl, spiData);  // get data from the ADC and send to buffer
	dataReady = true;
}

int main(void){

	int ret;
	sampleCounter = NUM_SAMPLES; 	// set initial sample window count

	dataReady = false; 				// intially set this low until ADC is active and data is ready

	if (!device_is_ready(dev)) {
		printk("%s: device not ready.\n", dev->name);
		return 0;
	}

	// configure and set GPIO pin
	gpio_pin_configure_dt(&reset, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&start, GPIO_ACTIVE_HIGH);
	gpio_pin_configure_dt(&drdy,  GPIO_ACTIVE_HIGH);
	gpio_pin_configure_dt(&en,    GPIO_ACTIVE_HIGH);
	gpio_pin_configure_dt(&done,  GPIO_ACTIVE_HIGH);

//	gpio_pin_set_dt(&reset, 1);
//	gpio_pin_set_dt(&reset, 0);

	ret = gpio_pin_configure_dt(&drdy, GPIO_INPUT);
		if(ret !=0){
		printk("Error configuring button pin.\n");
		return 0;
		}

	ret = gpio_pin_interrupt_configure_dt(&drdy, GPIO_INT_EDGE_TO_ACTIVE);
		if(ret !=0){printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
			printk("Error configuring button pin.\n");
			return 0;
		}

	gpio_init_callback(&drdy_cb_data, drdyEdge, BIT(drdy.pin));
	gpio_add_callback(drdy.port, &drdy_cb_data);
	printk("Set up DRDY at %s pin reset%d\n", drdy.port->name, drdy.pin);

	adcSetup();
	setSTART(true);

	while (1) {

		if (sampleCounter > 0) { 			// Have we collected all samples in window?
			if(dataReady){ 					// wait for data to be ready from the ADC
				dataReady = false;			// reset until callback indicates data is ready
				cSample = adcRX; 			// push uint16_t sample value from callback into local buffer
				sumSAMP += cSample; 		// add value to total sum for this window
				sampleCounter--;            // decrement the sample count

				if(cSample> vMAX){          // Set min and max values based on samples we have already taken
					vMAX = cSample;
				}else if(cSample<vMIN){
					vMIN = cSample;
				}
			} else{ 						//do nothing since data is not ready yet
			  }
		} else{
			setSTART(false);				// now we have all samples, stop ADC sampling
			calculateResult();				// calculate results to be sent to display
			resetVars(); 					// Reset the variable values for next iteration
			k_sleep(K_MSEC(10000));
			setSTART(true); 				// Set up ADC for next window of sampling
		}
	}
	return 0;
}
