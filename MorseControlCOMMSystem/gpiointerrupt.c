/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#include <ti/drivers/Timer.h>

// Constants needed for the state machine
enum States { SOS = 3, OK = 2 } state = SOS;
// This is the length for each output in number of counts the timer will cause
enum Letters { S = 5, O = 10, K = 8, SPACE = 2, RETURN = 6 };

// Words that will be outputted
const char sos[] = "S O S";
const char ok[] = "O K";

void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
    // Static variables that need to persist between each iteration
    static short timeCounter = 1;
    static short letterCount = 0;
    static char word[6] = "S O S";


    // Will go through each word one letter at a time
    switch (word[letterCount]) {
    case 'S':
        // Since this letter is just toggle back and forth between on and off each
        // cycle, it just needs to count the number of time is cycled
        if (timeCounter < S) {
            GPIO_toggle(CONFIG_GPIO_LED_0);
            timeCounter++;
        } else {
            // turns off the light and increases the letter count
            // as this is the end of the letter
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            letterCount++;
            timeCounter = 0;
        }
        break;
    case 'O':
        if (timeCounter < 11) {
            // Tests to see if this count is the 4th count
            // as the light turns off on the fourth count
            if ((timeCounter + 1) % 4) {
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
            } else {
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            }
            timeCounter++;
        } else {
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            letterCount++;
            timeCounter = 0;
        }
        break;
    case 'K':
        if (timeCounter < K) {
            // A little more complicated in this one due to the fact that
            // there isn't an easy pattern to discern from.
            if (timeCounter < 3 || (timeCounter > 5 && timeCounter < K)) {
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
            } else if (timeCounter == 4) {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            } else {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            }
            timeCounter++;
        } else {
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            letterCount++;
            timeCounter = 0;
        }
        break;
    case ' ':
        // When the letter is a space it just counts until the space time has elapsed
        if (timeCounter < SPACE) {
            timeCounter++;
        } else {
            letterCount++;
            timeCounter = 0;
        }
        break;
    case '\0':
        // a string always ends with \0 so I catch it as an indicator that
        // the word has ended
        if (timeCounter < RETURN) {
            timeCounter++;
        } else {
            // reset variables for the next word
            letterCount = 0;
            timeCounter = 0;

            // change the word based on the current state
            // Since strings always end in \0 I don't have to
            // keep track of word length and can let the switch
            // hand it.
            if (state == SOS) {
                strcpy(word, sos);
            } else if ( state == OK ) {
                strcpy(word, ok);
            }

        }
        break;
    }
}

void initTimer(void)
{
    Timer_Handle timer0;
    Timer_Params params;

    Timer_init();
    Timer_Params_init(&params);
    params.period = 500000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while(1) {}
    }
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    if (state == SOS) {
        state = OK;
    } else if (state == OK) {
        state = SOS;
    }
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    gpioButtonFxn0(index);  // Since both buttons are doing the something, calling previous button to complete this task
}


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn off user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);

    initTimer();

    return (NULL);
}
