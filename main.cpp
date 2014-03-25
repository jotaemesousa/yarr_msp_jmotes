/*
 * main.cpp
 *
 *  Created on: May 6, 2013
 *      Author: joao
 */

#include "rf24/RF24.h"
#include <msp430.h>
#include "remote_defines.h"
#include "string.h"
#include "flash.h"
#include "math.h"

extern "C"
{
#include "rf24/spi.h"
#include "conio/conio.h"
#include "serial/serial.h"
#include "timer_msp.h"
}

// Function prototypes
void setup_adc(void);
void adc_sample(unsigned int *ADC_ptr);
int var = 0;

long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void rf24_init(RF24& radio)
{
	radio = RF24();

	// Radio pipe addresses for the 2 nodes to communicate.
	const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL};

	// Setup and configure rf radio
	radio.begin();

	// optionally, increase the delay between retries & # of retries
	radio.setRetries(15,15);

	// optionally, reduce the payload size.  seems to
	// improve reliability
	radio.setPayloadSize(sizeof(report_t));

	radio.setDataRate(RF24_250KBPS);

	// Open pipes to other nodes for communication
	radio.openWritingPipe(pipes[1]);
	radio.openReadingPipe(1,pipes[0]);

	// Start listening
	radio.startListening();

	radio.printDetails();
}

void setupPWM(void)
{
	P2DIR |= BIT2 + BIT4;
	P2SEL |= BIT2 + BIT4;

	//TA0CCR0 = 20000-1;
	TA1CCR0 = 20000-1;

	TA1CCR1 = 1500;
	TA1CCR2 = 1500;

	TA1CCTL1 = OUTMOD_7;                       // CCR1 reset/set
	TA1CCTL2 = OUTMOD_7;                       // CCR2 reset/set
	TA1CTL   = TASSEL_2 + MC_1 + ID_3;                // SMCLK, up mode
}

uint8_t getNRF24report(RF24 &radio, report_t &gamepad_report)
{
	// if there is data ready
	if (radio.available()) {
		//report_t gamepad_report;
		bool done = false;
		while (!done) {
			// Fetch the payload, and see if this was the last one.
			done = radio.read(&gamepad_report, sizeof(report_t));

			if(done)
			{
				return 0;
			}
			else
			{
				return 1;
			}
		}
	}
	return 2;
}

uint_fast8_t checkifButtonIsPressed(report_t &gamepad_report, jmotes_buttons button)
{
	if((gamepad_report.buttons & button) == button)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void drive(report_t &gamepad_report, CarParameters &settings)
{
	int y = (int8_t) gamepad_report.linear;
	int ry = (int8_t) (gamepad_report.steer);

	if(settings.steer_invert)
	{
		ry *= -1;
	}

	unsigned int  pos = 1500 + settings.steer_offset;
	if(!checkifButtonIsPressed(gamepad_report, R1_BUTTON))
	{
		if (ry > 0){
			pos = map(ry, 1, 127, 1500 + settings.steer_offset, 2000 + settings.steer_offset);
		}
		else if (ry < 0){
			pos = map(-ry, 1, 127, 1500 + settings.steer_offset, 1000 + settings.steer_offset);
		}
	}
	else
	{
		if (ry > 0){
			pos = map(ry, 1, 127, 1500 + settings.steer_offset, 1800 + settings.steer_offset);
		}
		else if (ry < 0){
			pos = map(-ry, 1, 127, 1500 + settings.steer_offset, 1200 + settings.steer_offset);
		}
	}

	TA1CCR1 = pos;

	unsigned int speed = BASE_ESC;
	if(!checkifButtonIsPressed(gamepad_report, L1_BUTTON))
	{
		if (y > 0){
			speed = map(y, 1, 127, BASE_ESC, BASE_ESC + settings.max_acc);		//2600
		}else if (y < 0){
			speed = map(-y, 1, 127, BASE_ESC, BASE_ESC - 1000);	//400
		}
	}
	else
	{
		if (y > 0){
			speed = map(y, 1, 127, BASE_ESC, BASE_ESC + settings.max_acc / 2);		//1800
		}else if (y < 0){
			speed = map(-y, 1, 127, BASE_ESC, BASE_ESC - 1000);	//1200
		}
	}

	TA1CCR2 = speed;
}

void readSettinsFromFlash(CarParameters *settings)
{
	read_SegC((unsigned char *)settings, sizeof(*settings), 0);
}

void storeSettinsToFlash(CarParameters &settings)
{
	write_SegC((unsigned char *)&settings, sizeof(settings));
}

// main loop
int main(void)
{
	WDTCTL = WDTPW + WDTHOLD;                     // Set Watchdog Timer interval to ~30ms

	BCSCTL1 = CALBC1_8MHZ;            		// Set DCO to 1MHz
	DCOCTL = CALDCO_8MHZ;

	uint32_t last_millis = millis();
	uint32_t last_millis_calibration = millis();
	default_timer();

	setupPWM();
	flash_init();

	P1DIR |= BIT0 + BIT1;

	__bis_SR_register(GIE);       // Enter LPM0, interrupts enabled

	// ____________________________________________________________
	report_t car;
	CarParameters settings;
	settings.steer_invert = 0;
	settings.steer_offset = 0;
	settings.max_acc = 0;

	readSettinsFromFlash(&settings);

	//RC_dongle car_param;
	car.steer = 0;
	car.linear = 0;
	car.buttons = 0;


	//	serial_init(57600);
	//	cio_printf(":Init UART;\n");
	//	cio_printf(":Done;\n");


	RF24 radio;

	rf24_init(radio);

	uint8_t calibration_status = 0;
	uint8_t calibration_acc_status = 0;

	for(;;)
	{

		// if there is data ready
		if(getNRF24report(radio, car)==0)
		{
			drive(car, settings);
			P1OUT &= ~BIT0;

			if((car.buttons & ASK_BIT) == ASK_BIT)
			{
				uint8_t value2send = 0;
				radio.startListening();
				radio.stopListening();
				radio.write(&value2send, sizeof(uint8_t));
				radio.startListening();
			}

			int16_t temp;
			switch (calibration_status)
			{
			case 0:
				if(checkifButtonIsPressed(car, L2_BUTTON))
				{
					calibration_status = 1;
					last_millis_calibration = millis();
				}
				break;

			case 1:
				if(checkifButtonIsPressed(car, L2_BUTTON))
				{
					if(millis() - last_millis_calibration > START_CALIBRATION_TIME_MS)
					{
						calibration_status = 2;
						calibration_acc_status = 1;
						report_t temp = {0,0,0};
						drive(temp, settings);
						P1OUT |= BIT1;
					}
				}
				else
				{
					calibration_status = 0;
					calibration_acc_status = 0;
				}
				break;

			case 2:

				while(!checkifButtonIsPressed(car, R2_BUTTON))
				{
					P1OUT &= ~BIT1;
					if(getNRF24report(radio, car) == 0)
					{
						settings.steer_offset += car.steer/20;
						settings.steer_invert = checkifButtonIsPressed(car, R1_BUTTON);

						switch (calibration_acc_status)
						{
						case 1:
							if(car.linear <= 10 && car.linear >= -10)
							{
								calibration_acc_status = 1;
							}
							else
							{
								calibration_acc_status = 2;
							}
							break;

						case 2:
							settings.max_acc = map((((car.linear) >= 0) ? car.linear : 0), 0, 120, 150, 850);

							if(settings.max_acc > 850)
							{
								settings.max_acc = 850;
							}
							break;
						default:
							break;
						}

						delay(10);
						report_t temp = {0,0,0};
						drive(temp, settings);
					}
				}
				storeSettinsToFlash(settings);

				while(car.linear != 0)
				{
					if(getNRF24report(radio, car))
					{
						car.linear = 1;
					}
				}
				calibration_status = 0;
				break;

			default:
				break;
			}

			last_millis = millis();
		}
		else
		{
			if(millis() - last_millis > TIMEOUT_MS)
			{
				report_t temp = {0,0,0};
				drive(temp, settings);
				P1OUT |= BIT0;
			}
		}
	}
	return 0;
}

void setup_adc(void)
{
	// Scan P1.3 and P1.4
	ADC10CTL0 = ADC10SHT_2 + MSC + ADC10ON + ADC10IE;
	ADC10CTL1 = INCH_4 + CONSEQ_3;
	ADC10DTC1 = 0x02;
	ADC10AE0 |= 0x18;
}

void adc_sample( unsigned int *ADC_ptr)
{
	ADC10CTL0 &= ~ENC;

	while (ADC10CTL1 & BUSY);               // Wait if ADC10 core is active

	ADC10SA = (unsigned int)ADC_ptr;     	// Data buffer start

	ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
	__bis_SR_register(GIE);        			// LPM0, ADC10_ISR will force exit
}

// ADC10 interrupt service routine
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
	__bic_SR_register_on_exit(GIE);        // Clear CPUOFF bit from 0(SR)
}

// Watchdog Timer interrupt service routine
#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void)
{

}
