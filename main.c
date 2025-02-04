
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

#include <ti/driverlib/dl_mathacl.h>
#include "randomValues.h"
#include <ti/driverlib/dl_timera.h>
#include <ti/driverlib/dl_timerg.h>
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/dl_common.h>

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
#define SPI_1_NODE	DT_NODELABEL(spi1)
#define LED0_NODE DT_NODELABEL(led0)
#define SAMP_BITS 12
#define NUM_BITS 16
#define NUM_SAMPLES 32//pow(2,SAMP_BITS)
#define FULL_SCALE 5
#define MAX_NUM pow(2,NUM_BITS)
#define SPI_PACKET_SIZE 4
#define SAC_MASK 0x000000000000FFFF

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
float radicand = 0;
uint32_t UN = 0;
uint32_t SFACTOR = 0;
uint32_t count = 0;
float SN = 0;
uint32_t SN_Q30 = 0;
uint32_t sqrtRes_Q16 = 0;
//static const DL_TimerA_ClockConfig gTIMER_0ClockConfig = {
//	.clockSel  = DL_TIMER_CLOCK_BUSCLK,
//	.divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
//	.prescale = 0u,
//	};
//
//static const DL_TimerA_TimerConfig gTIMER_0TimerConfig = {
//	.period = TIMER_0_INST_LOAD_VALUE,
//	.timerMode = DL_TIMER_TIMER_MODE_PERIODIC,
//	.startTimer = DL_TIMER_STOP,
//	};


extern const uint16_t gRandomLUT[];
uint8_t spiData[SPI_PACKET_SIZE] = {0x0,0x0,0x0,0x0};

uint16_t adcRX, cSample;
int sampleCounter;
float SQRTRes;

int32_t vMIN, vMAX, VDC, pk_pk;
int32_t vRMS;
int32_t dataOutput[5] ;//= {0,0,0,0,0};

int64_t sacSum, sumSAMP, sacRes1, sacRes2;
int sqrtRes1, sqrtRes2, combVal;
bool  dataReady;

const DL_MathACL_operationConfig gSACConfig = {
	.opType = MATHACL_CTL_FUNC_SAC,
	.opSign = DL_MATHACL_OPSIGN_SIGNED,
	.iterations = 1,
	.scaleFactor = 0,
	.qType = DL_MATHACL_Q_TYPE_Q16
};

DL_MathACL_operationConfig gSQRTConfig = {
    .opType      = DL_MATHACL_OP_TYPE_SQRT,
    .opSign      = DL_MATHACL_OPSIGN_UNSIGNED,
    .iterations  = 5,
    .scaleFactor = 0,
    .qType       = DL_MATHACL_Q_TYPE_Q30
};

static struct gpio_callback drdy_cb_data;

void setSTART(const bool state){

	uint8_t value = (uint8_t)(state ? 1 : 0);
	gpio_pin_set_dt(&start, value);
	//if(state == true){
	//	printf("ADC Start Request\n");
	//}else printf("ADC Stop Request\n");
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
	k_sleep(K_MSEC(1000));

	tmpBuffer = (uint8_t [4]){0x8, 0x6,0x0,0x8};
	SPI_TRX(dev, &cs_ctrl, tmpBuffer);
	k_sleep(K_MSEC(1000));

	tmpBuffer = (uint8_t [4]){0x8, 0x7,0x0,0xB};
	SPI_TRX(dev, &cs_ctrl, tmpBuffer);
	k_sleep(K_MSEC(1000));

	tmpBuffer = (uint8_t [4]){0x8, 0x8,0x0,0x8};
	SPI_TRX(dev, &cs_ctrl, tmpBuffer);
	k_sleep(K_MSEC(1000));

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


	printf(" tx (i)  : %x %x %x %x \n",
	       buff[0], buff[1], buff[2], buff[3]);
	printf(" rx (i)  : %x %x \n",
	       rxdata[0], rxdata[1]);

	// Convert the 16 bit data from ADC into a UINT16_t value
	adcRX = (uint16_t)(rxdata[0] << 8 | rxdata[1]);
	printf(" ADC Value : %d \n",adcRX);
}

void resetVars(void){
	//printf("Resetting Variable Values for Next Window\n");
	vMIN 	= MAX_NUM + 1;
	vMAX 	= -MAX_NUM - 1;
	sumSAMP = 0;
	sacSum  = 0;
	pk_pk 	= 0;
	sampleCounter = NUM_SAMPLES;
	vRMS = 0;
//	sacRes1 = 0;
}

void calculateResult(void){
//	vRMS  = sqrt(sacSum >> SAMP_BITS) * FULL_SCALE;
//	VDC   = (sumSAMP >>SAMP_BITS) * FULL_SCALE;
//	vMIN  = vMIN * FULL_SCALE;
//	vMAX  = vMAX * FULL_SCALE;
//	pk_pk = vMAX - vMIN;
//
//	DL_MathACL_waitForOperation(MATHACL);
//	sacRes1 = DL_MathACL_getResultOne(MATHACL);
//	sacRes2 = DL_MathACL_getResultTwo(MATHACL);
//	sacSum = ((sacRes2) <<16) | ((sacRes1) & SAC_MASK);
//
//	printf("Sample 1 Result: %lld\n", sacRes1);
//	printf("Sample 2 Result: %lld\n", sacRes2);
//	printf("SAC Output: %lli\n", sacSum);
//	printf("VDC: %i\n", VDC);
//	printf("VMIN: %i\n", vMIN);
//	printf("VMAX: %i\n", vMAX);
//	printf("Vpk-pk: %i\n", pk_pk);

	radicand = 10.375;
    UN = floor(radicand);
    SFACTOR = 0;
    count = 2;
    SN = 0;
    do {
        SFACTOR++;
        count <<= 1;

    } while(count < UN);

    SN = radicand / (count >> 1);
    SN_Q30 = (uint32_t) (SN * (1 << 30));
    gSQRTConfig.scaleFactor = SFACTOR;

    DL_MathACL_startSqrtOperation(MATHACL, &gSQRTConfig, SN_Q30);
    DL_MathACL_waitForOperation(MATHACL);

    sqrtRes_Q16 = DL_MathACL_getResultOne(MATHACL);
	sqrtRes1 = (int)(sqrtRes_Q16>> 16);
	sqrtRes2 = (int)(sqrtRes_Q16);
	combVal = (sqrtRes1 <<16) | sqrtRes2;
	SQRTRes = (float) combVal/(1<<16);
	printf("SQRT Output: 0x%x\n", sqrtRes_Q16);
	DL_MathACL_clearResults(MATHACL);

	//
//	dataOutput[0] =  vMIN;		// minimum voltage converted from ADC value to voltage
//	dataOutput[1] =  vMAX; 		// maximum voltage converted from ADC value to voltage
//	dataOutput[2] =  pk_pk;		// Pk-pk voltage from ADC value
//	dataOutput[3] =  VDC;		// DC voltage form ADC value
//	dataOutput[4] =  vRMS;    	// VRMS form ADC values * sqrt(2)
//
//	printf("vMin: %d vMax: %d Vpk-pk: %d VDC: %d vRMS: %d\n", dataOutput[0],
//	       dataOutput[1], dataOutput[2], dataOutput[3], dataOutput[4]);
//
	k_sleep(K_MSEC(3000)); // just using for debug to keep output visible for 10 seconds
//	gpio_pin_set_dt(&done, 0); // done with this windo of data, toggle LED
}

void drdyEdge(const struct device *dev, struct gpio_callback *cb, uint32_t pins){
	printf("Drdy Edge GPIO Call\n");
	dataReady = true;
	SPI_TRX(dev, &cs_ctrl, spiData);  // get data from the ADC and send to buffer
}

int main(void){

	int ret;
	sampleCounter = NUM_SAMPLES; 	// set initial sample window count
	uint16_t rnd ;
	dataReady = false; 				// intially set this low until ADC is active and data is ready

	if (!device_is_ready(dev)) {
		printk("%s: device not ready.\n", dev->name);
		return 0;
	}

	DL_MathACL_enablePower(MATHACL);
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

		DL_MathACL_clearResults(MATHACL);

	//gpio_pin_set_dt(&en, 1);
	adcSetup();

	gpio_init_callback(&drdy_cb_data, drdyEdge, BIT(drdy.pin));
	gpio_add_callback(drdy.port, &drdy_cb_data);

	DL_Timer_enableInterrupt(TIMA0, DL_TIMERA_INTERRUPT_ZERO_EVENT);
	setSTART(true);

	while (1) {
		DL_MathACL_setOperandOne(MATHACL, 0);
		DL_MathACL_setOperandTwo(MATHACL, 0);
		rnd = rand() % 4096 + 1;
		if (sampleCounter > 0) { 			// Have we collected all samples in window?
			//if(dataReady){ 					// wait for data to be ready from the ADC
				//dataReady = false;			// reset until callback indicates data is ready
				cSample = rnd;
				DL_MathACL_setOperandTwo(MATHACL,rnd);
				DL_MathACL_waitForOperation(MATHACL);

				DL_MathACL_configOperation(MATHACL, &gSACConfig, 0, cSample);
				 				// push uint16_t sample value from callback into local buffer
				sumSAMP += cSample; 		// add value to total sum for this window
				sampleCounter--;            // decrement the sample count



				//k_sleep(K_MSEC(50));

				if(cSample> vMAX){          // Set min and max values based on samples we have already taken
					vMAX = cSample;
				}else if(cSample<vMIN){
					vMIN = cSample;
				}
			//}
//			} else{ //............		    //do nothing since data is not ready yet
//			  }
		} else{

			setSTART(false);				// now we have all samples, stop ADC sampling
			calculateResult();				// calculate results to be sent to display
			resetVars(); 					// Reset the variable values for next iteration
			gpio_pin_set_dt(&done, 1);
			//k_sleep(K_MSEC(10000));
			setSTART(true); 				// Set up ADC for next window of sampling
			gpio_pin_set_dt(&done, 0);
		}
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////


void TIMER_0_INST_ISR(void){

	setSTART(true);
	//DL_TimerA_startCounter(TIMA0);
	DL_TimerA_setLoadValue(TIMA0, gRandomLUT[sampleCounter]);


}