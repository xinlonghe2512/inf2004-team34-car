/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/binary_info.h"

#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

#define BAUD_RATE 115200

#define ADC_PIN 26
#define DIGITAL_PIN 22

#define BLACK_THRESHOLD 3000

#define WHITE_THRESHOLD 400

#define BARCODE_BUF_SIZE 10

#define BARCODE_ARR_SIZE 9

#define ADC_DIFFERENCE_THRESHHOLD 100

// Global Variables
uint8_t barcodeFirstChar = 0;
uint8_t barcodeSecondChar = 0;
uint8_t barcodeThirdChar = 0;

// Barcode Type Enumeration
enum bartype
{
    THICK_BLACK, // 0
    THIN_BLACK,  // 1
    THICK_WHITE, // 2
    THIN_WHITE   // 3
};

// Barcode Array Mappings
// These arrays represent the pattern of bars for each character in Code 39 barcode format.
static char *A_ARRAY_MAP = "031312130";
static char *B_ARRAY_MAP = "130312130";
static char *C_ARRAY_MAP = "030312131";
static char *D_ARRAY_MAP = "131302130";
static char *E_ARRAY_MAP = "031302131";
static char *F_ARRAY_MAP = "130302131";
static char *G_ARRAY_MAP = "131312030";
static char *H_ARRAY_MAP = "031312031";
static char *I_ARRAY_MAP = "130312031";
static char *J_ARRAY_MAP = "131302031";
static char *K_ARRAY_MAP = "031313120";
static char *L_ARRAY_MAP = "130313120";
static char *M_ARRAY_MAP = "030313121";
static char *N_ARRAY_MAP = "131303120";
static char *O_ARRAY_MAP = "031303121";
static char *P_ARRAY_MAP = "130303121";
static char *Q_ARRAY_MAP = "131313020";
static char *R_ARRAY_MAP = "031313021";
static char *S_ARRAY_MAP = "130313021";
static char *T_ARRAY_MAP = "131303021";
static char *U_ARRAY_MAP = "021313130";
static char *V_ARRAY_MAP = "120313130";
static char *W_ARRAY_MAP = "020313131";
static char *X_ARRAY_MAP = "121303130";
static char *Y_ARRAY_MAP = "021303131";
static char *Z_ARRAY_MAP = "120303131";

// code 39 format of asterisk using enum bartype
static char *ASTERISK_ARRAY_MAP = "121303031";

// Variables for ADC and Timing
static uint32_t res = 0;
static uint16_t prevAvg = 0;
static int i = 0;
static int barcode_arr_index = 1;
char *outputBuffer;
static absolute_time_t blockStart;
static absolute_time_t blockEnd;

// Structure to Classify Voltage Readings
struct voltageClassification
{
    uint16_t voltage;
    // 0 - white
    // 1 - black
    int blackWhite;
    absolute_time_t blockStart;
    int64_t blockLength;
    enum bartype type;
} voltageClassification;

// queue for voltageclassifications of length 9
static struct voltageClassification voltageClassifications[BARCODE_BUF_SIZE];

// queue for barcode read of length 3
static char barcodeRead[3];

// function to append to barcodeRead Queue
static void appendToBarcodeRead(char barcodeChar)
{
     // This function updates the barcodeRead array with the new character.
    barcodeRead[0] = barcodeRead[1];
    barcodeRead[1] = barcodeRead[2];
    barcodeRead[2] = barcodeChar;
}

// Pattern Mappings and Character Arrays
static char *patternMappings[] = {
    "031312130", "130312130", "030312131", "131302130", "031302131",
    "130302131", "131312030", "031312031", "130312031", "131302031",
    "031313120", "130313120", "030313121", "131303120", "031303121",
    "130303121", "131313020", "031313021", "130313021", "131303021",
    "021313130", "120313130", "020313131", "121303130", "021303131",
    "120303131"};

const char characters[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ*";

// Function to Validate Barcode
static int isValidBarcode()
{
    // This function checks if the read barcode is valid.
    if (barcodeRead[0] == '*' && barcodeRead[2] == '*')
    {
        if (barcodeRead[1] != 0)
            return 1;
    }

    else if (barcodeRead[0] == 'P' && barcodeRead[2] == 'P')
    {
        if (barcodeRead[1] != 0)
            return 2;
    }

    return 0;
}

// Function to Check if Barcode is Full
static int isBarcodeFull()
{
    // This function checks if the barcodeRead array is fully populated.
    if (barcodeRead[0] != 0 && barcodeRead[1] != 0 && barcodeRead[2] != 0)
    {
        return 1;
    }

    return 0;
}

// Function to Clear Barcode Read Array
static void clearBarcodeRead()
{
    // This function resets the barcodeRead array.
    barcodeRead[0] = 0;
    barcodeRead[1] = 0;
    barcodeRead[2] = 0;
}

// function to convert array of integer to string
static char *intArrayToString(int *arr, int size)
{
    char *str = malloc(size + 1);
    for (int i = 0; i < size; i++)
    {
        str[i] = arr[i] + '0';
    }
    str[size] = '\0';
    return str;
}

static int *thickThinClassification()
{
    // calculate average block length
    int64_t totalBarLength = 0;
    for (int i = 0; i < BARCODE_ARR_SIZE; i++)
    {
        totalBarLength += voltageClassifications[i].blockLength;
    }

    int *barsRead = malloc(BARCODE_ARR_SIZE * sizeof(int));

    int64_t avgBarLength = (totalBarLength / BARCODE_ARR_SIZE);
    // assign thick thin classification
    for (int i = 0; i < BARCODE_ARR_SIZE; i++)
    {

        if (voltageClassifications[i].blackWhite)
        {
            if (voltageClassifications[i].blockLength < avgBarLength)
            {
                voltageClassifications[i].type = THIN_BLACK;
            }
            else
            {
                voltageClassifications[i].type = THICK_BLACK;
            }
        }
        else
        {
            if (voltageClassifications[i].blockLength < avgBarLength)
            {
                voltageClassifications[i].type = THIN_WHITE;
            }
            else
            {
                voltageClassifications[i].type = THICK_WHITE;
            }
        }
        barsRead[i] = voltageClassifications[i].type;
    }
    return barsRead;
}
// function to check if queue is full
static int isVoltageClassificationFull()
{
    for (int i = 0; i < BARCODE_BUF_SIZE; i++)
    {
        if (voltageClassifications[i].blackWhite == -1)
        {
            return 0;
        }
    }
    return 1;
}

// function to flush queue
static void flushVoltageClassification()
{
    barcode_arr_index = 1;
    blockStart = get_absolute_time();

    struct voltageClassification lastReading = voltageClassifications[BARCODE_BUF_SIZE - 1];

    for (int i = 0; i < BARCODE_BUF_SIZE; i++)
    {
        voltageClassifications[i].voltage = 0;
        voltageClassifications[i].blackWhite = -1;
        voltageClassifications[i].blockLength = 0;
        voltageClassifications[i].type = 0;
    }

    voltageClassifications[0] = lastReading;
}

// function to compare buffer and the barcodes
static char compareTwoArray()
{
    int *barsRead = thickThinClassification();

    if (voltageClassifications[0].blackWhite == 0)
    {
        return 0;
    }

    char *string = intArrayToString(barsRead, BARCODE_ARR_SIZE);
    free(barsRead);

    char *barcodes[] = {
        A_ARRAY_MAP,
        B_ARRAY_MAP,
        C_ARRAY_MAP,
        D_ARRAY_MAP,
        E_ARRAY_MAP,
        F_ARRAY_MAP,
        G_ARRAY_MAP,
        H_ARRAY_MAP,
        I_ARRAY_MAP,
        J_ARRAY_MAP,
        K_ARRAY_MAP,
        L_ARRAY_MAP,
        M_ARRAY_MAP,
        N_ARRAY_MAP,
        O_ARRAY_MAP,
        P_ARRAY_MAP,
        Q_ARRAY_MAP,
        R_ARRAY_MAP,
        S_ARRAY_MAP,
        T_ARRAY_MAP,
        U_ARRAY_MAP,
        V_ARRAY_MAP,
        W_ARRAY_MAP,
        X_ARRAY_MAP,
        Y_ARRAY_MAP,
        Z_ARRAY_MAP,
        ASTERISK_ARRAY_MAP};

    char characters[] = {
        'A',
        'B',
        'C',
        'D',
        'E',
        'F',
        'G',
        'H',
        'I',
        'J',
        'K',
        'L',
        'M',
        'N',
        'O',
        'P',
        'Q',
        'R',
        'S',
        'T',
        'U',
        'V',
        'W',
        'X',
        'Y',
        'Z',
        '*'};

    for (int i = 0; i < 27; i++)
    {
        if (strncmp(barcodes[i], string, BARCODE_ARR_SIZE) == 0)
        {
            free(string);
            flushVoltageClassification();
            return characters[i];
        }
    }

    return 0;
    // return 1;
}

char decodeReverse(char *string)
{

    char *barcodes[] = {
        A_ARRAY_MAP,
        B_ARRAY_MAP,
        C_ARRAY_MAP,
        D_ARRAY_MAP,
        E_ARRAY_MAP,
        F_ARRAY_MAP,
        G_ARRAY_MAP,
        H_ARRAY_MAP,
        I_ARRAY_MAP,
        J_ARRAY_MAP,
        K_ARRAY_MAP,
        L_ARRAY_MAP,
        M_ARRAY_MAP,
        N_ARRAY_MAP,
        O_ARRAY_MAP,
        P_ARRAY_MAP,
        Q_ARRAY_MAP,
        R_ARRAY_MAP,
        S_ARRAY_MAP,
        T_ARRAY_MAP,
        U_ARRAY_MAP,
        V_ARRAY_MAP,
        W_ARRAY_MAP,
        X_ARRAY_MAP,
        Y_ARRAY_MAP,
        Z_ARRAY_MAP,
        ASTERISK_ARRAY_MAP};

    char characters[] = {
        'A',
        'B',
        'C',
        'D',
        'E',
        'F',
        'G',
        'H',
        'I',
        'J',
        'K',
        'L',
        'M',
        'N',
        'O',
        'P',
        'Q',
        'R',
        'S',
        'T',
        'U',
        'V',
        'W',
        'X',
        'Y',
        'Z',
        '*'};

    for (int i = 0; i < 27; i++)
    {
        if (strncmp(barcodes[i], string, BARCODE_ARR_SIZE) == 0)
        {
            free(string);
            flushVoltageClassification();
            return characters[i];
        }
    }

    return 0;
}

// function to append queue
static void appendVoltageClassification(struct voltageClassification voltageClassification)
{
    voltageClassifications[0] = voltageClassifications[1];
    voltageClassifications[1] = voltageClassifications[2];
    voltageClassifications[2] = voltageClassifications[3];
    voltageClassifications[3] = voltageClassifications[4];
    voltageClassifications[4] = voltageClassifications[5];
    voltageClassifications[5] = voltageClassifications[6];
    voltageClassifications[6] = voltageClassifications[7];
    voltageClassifications[7] = voltageClassifications[8];
    voltageClassifications[8] = voltageClassifications[9];
    voltageClassifications[9] = voltageClassification;
    if (barcode_arr_index == BARCODE_BUF_SIZE)
    {
        char read = compareTwoArray();
        if (read != 0)
        {
            printf("%c\0", read);
            appendToBarcodeRead(read);
        }
    }
}

static void ADC_IRQ_FIFO_HANDLER()
{
    // read data from ADC FIFO
    if (!adc_fifo_is_empty())
    {
        uint16_t data = adc_fifo_get();
        res += data;
        if (i < 100)
        {
            i++;
        }
        else
        {

            uint16_t avg = res / (i);

            if (prevAvg == 0)
            {
                prevAvg = avg;
            }
            else
            {
                if (abs(prevAvg - avg) > ADC_DIFFERENCE_THRESHHOLD)
                {
                    prevAvg = avg;
                }
                else
                {
                    avg = prevAvg;
                }
            }

            i = 0;
            res = 0;

            struct voltageClassification voltageClassification;
            voltageClassification.voltage = avg;

            if (avg > BLACK_THRESHOLD || gpio_get(DIGITAL_PIN) == 1)
            {
                voltageClassification.blackWhite = 1;
            }
            else
            {
                voltageClassification.blackWhite = 0;
            }

            if (barcode_arr_index == BARCODE_BUF_SIZE)
            {

                if (voltageClassifications[BARCODE_BUF_SIZE - 1].blackWhite != voltageClassification.blackWhite)
                {
                    blockEnd = get_absolute_time();
                    voltageClassification.blockStart = blockEnd;
                    int64_t blockLength = absolute_time_diff_us(voltageClassifications[BARCODE_BUF_SIZE - 1].blockStart, blockEnd);

                    voltageClassifications[BARCODE_BUF_SIZE - 1].blockLength = blockLength / 10000;
                    appendVoltageClassification(voltageClassification);
                }
            }
            else
            {
                if (voltageClassifications[barcode_arr_index - 1].blackWhite != voltageClassification.blackWhite)
                {
                    blockEnd = get_absolute_time();
                    voltageClassification.blockStart = blockEnd;
                    if (barcode_arr_index == 0)
                    {
                        int64_t blockLength = absolute_time_diff_us(blockStart, blockEnd);
                        voltageClassification.blockLength = blockLength / 10000;
                    }
                    else
                    {
                        int64_t blockLength = absolute_time_diff_us(voltageClassifications[barcode_arr_index - 1].blockStart, blockEnd);
                        voltageClassifications[barcode_arr_index - 1].blockLength = blockLength / 10000;
                    }

                    voltageClassifications[barcode_arr_index] = voltageClassification;
                    barcode_arr_index++;
                }
            }
        }
    }
    irq_clear(ADC_IRQ_FIFO);
}

char *reverseString(const char *str)
{
    int length = strlen(str);
    char *reversed = (char *)malloc(length + 1);

    if (reversed == NULL)
    {
        return NULL; // Memory allocation failed
    }

    for (int i = 0; i < length; i++)
    {
        reversed[i] = str[length - i - 1];
    }
    reversed[length] = '\0';

    return reversed;
}

// Function to find the index of a character in the characters array
static int findCharacterIndex(char character)
{
    for (int i = 0; characters[i] != '\0'; i++)
    {
        if (characters[i] == character)
        {
            return i;
        }
    }
    return -1; // Character not found
}

// Function to check if the reversed characterArray matches any of the known barcode arrays
char checkBarcodeMatch(char *reversedCharacterArray)
{
    for (int i = 0; i < 27; i++)
    {
        if (strcmp(patternMappings[i], reversedCharacterArray) == 0)
        {
            return characters[i];
        }
    }
    return '\0'; // Return '\0' to indicate no match found
}

int main()
{
 // Initialization of ADC and GPIO.
    adc_init();

    // init the queue
    flushVoltageClassification();

    adc_gpio_init(ADC_PIN);
    adc_select_input(0);

    adc_fifo_setup(true, false, 1, false, false);
    adc_set_clkdiv(0);
    adc_irq_set_enabled(true);

    irq_clear(ADC_IRQ_FIFO);

    irq_set_exclusive_handler(ADC_IRQ_FIFO, ADC_IRQ_FIFO_HANDLER);
    irq_set_enabled(ADC_IRQ_FIFO, true);

    adc_run(true);

    sleep_ms(2000);

    // The main loop continuously checks for valid barcodes and processes them accordingly.
    while (true)
    {
         if (isValidBarcode() == 1)
        {
            printf("%c\n\r", barcodeRead[1]);
            clearBarcodeRead();
            barcodeFirstChar = 0;
            barcodeSecondChar = 0;
            barcodeThirdChar = 0;

        }
        else if (isValidBarcode() == 2)
        {
            int characterIndex = findCharacterIndex(barcodeRead[1]);
            char *characterArray = patternMappings[characterIndex];
            char *barcodeDecoded = reverseString(characterArray);
            
            printf("%c\n\r", decodeReverse(barcodeDecoded));
            clearBarcodeRead();
            barcodeFirstChar = 0;
            barcodeSecondChar = 0;
            barcodeThirdChar = 0;

        }
    }
}
