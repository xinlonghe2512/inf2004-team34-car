#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "stdint.h"
#include <math.h>
#include <string.h>
#include <inttypes.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/binary_info.h"

#include "lwipopts.h"
#include "lwip/apps/httpd.h"

#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

// Accelerometer registers
#define ACCEL_ADDR 0x19
#define CTRL_REG1_A 0x20
#define CTRL_REG4_A 0x23
#define ACCEL_X_LOW 0x28
#define ACCEL_X_HIGH 0x29
#define ACCEL_Y_LOW 0x2A
#define ACCEL_Y_HIGH 0x2B
#define ACCEL_Z_LOW 0x2C
#define ACCEL_Z_HIGH 0x2D

// Magnetometer registers
#define MAG_ADDR 0x1E
#define MR_REG_M 0x02
#define MAG_X_LOW 0x04
#define MAG_X_HIGH 0x03
#define MAG_Z_LOW 0x06
#define MAG_Z_HIGH 0x05
#define MAG_Y_LOW 0x08
#define MAG_Y_HIGH 0x07

#define SCL_PIN 5
#define SDA_PIN 4

#define CONVERSION 130


// Define number of data points for moving average
#define Data_Points_Number 10
// Define maximum size of message buffers
#define TASK_MESSAGE_BUFFER_SIZE 60 

#define ADC_CHANNEL_PIN_ONE 1
#define ADC_CHANNEL_PIN_TWO 2
#define ADC_BASE_PIN1 27 //left
#define ADC_BASE_PIN2 28 //right
#define WHITE_TRACKING_THRESHOLD 1

#define TRIGGER_PIN 2
#define ECHO_PIN 3
#define TIMEOUT_US 13050

absolute_time_t startTime;
absolute_time_t endTime;

uint slice_num0;
uint slice_num1;
volatile bool moving = true;

// set pins for 3.3V and GND
const uint BTN_PIN1 = 18; // pin for channel A
const uint BTN_PIN2 = 19; // pin for channel A
const uint BTN_PIN3 = 20; // pin for channel B
const uint BTN_PIN4 = 21; // pin for channel B

// set initial directions for both wheels
volatile bool left1 = true, left2 = false;
volatile bool left3 = false, left4 = true;
volatile bool right1 = true, right2 = false;
volatile bool right3 = false, right4 = true;

float heading;
float targetAngle;
volatile bool turningRight = false;
volatile bool turningLeft = false;

//static int num_edge_l;
//static int num_edge_r;
//static float pulse_width_l;
//static float pulse_width_r;
//static float t_distance_travelled;
struct repeating_timer timer;

int speedLeft = 9375;
int speedRight = 5250;


#define BAUD_RATE 115200

#define ADC_PIN 26
#define DIGITAL_PIN 22

#define BLACK_THRESHOLD 3000

#define WHITE_THRESHOLD 400

#define BARCODE_BUF_SIZE 10

#define BARCODE_ARR_SIZE 9

#define ADC_DIFFERENCE_THRESHHOLD 100

void stopCar(uint slice_num0, uint slice_num1);
void sendInitiationPulse(uint trigPin);
void getUltrasonicDetection(uint64_t pulseLength);
void startCar(uint slice_num0, uint slice_num1);

uint8_t barcodeFirstChar = 0;
uint8_t barcodeSecondChar = 0;
uint8_t barcodeThirdChar = 0;

enum bartype
{
    THICK_BLACK, // 0
    THIN_BLACK,  // 1
    THICK_WHITE, // 2
    THIN_WHITE   // 3
};

// code 39 format of letter F using enum bartype
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

static uint32_t res = 0;
static uint16_t prevAvg = 0;

static int i = 0;
static int barcode_arr_index = 1;

char *outputBuffer;

static absolute_time_t blockStart;
static absolute_time_t blockEnd;

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
    barcodeRead[0] = barcodeRead[1];
    barcodeRead[1] = barcodeRead[2];
    barcodeRead[2] = barcodeChar;
}

static char *patternMappings[] = {
    "031312130", "130312130", "030312131", "131302130", "031302131",
    "130302131", "131312030", "031312031", "130312031", "131302031",
    "031313120", "130313120", "030313121", "131303120", "031303121",
    "130303121", "131313020", "031313021", "130313021", "131303021",
    "021313130", "120313130", "020313131", "121303130", "021303131",
    "120303131"};

const char characters[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ*";

static int isValidBarcode()
{
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

static int isBarcodeFull()
{
    if (barcodeRead[0] != 0 && barcodeRead[1] != 0 && barcodeRead[2] != 0)
    {
        return 1;
    }

    return 0;
}

static void clearBarcodeRead()
{
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

// WIFI PORTION
//const char WIFI_SSID[] = "Ivan";
//const char WIFI_PASSWORD[] = "begmeplease";

const char WIFI_SSID[] = "Wyvern S23+";
const char WIFI_PASSWORD[] = "5ndn2n6n3e5ytsf";

// SSI tags - tag length limited to 8 bytes by default
const char *ssi_tags[] = {"barcode", "status"};

u16_t ssi_handler(int iIndex, char *pcInsert, int iInsertLen)
{
    size_t printed;
    static uint8_t barcodeCharacter;
    switch (iIndex)
    {
    case 0: // barcode placeholder
    {
        printf("read: %c %c %c \n", barcodeRead[0], barcodeRead[1], barcodeRead[2]);
        if (isValidBarcode() == 1)
        {  
            printed = snprintf(pcInsert, iInsertLen, "%c\n\r", barcodeRead[1]);

        }
        else if (isValidBarcode() == 2)
        {
            int characterIndex = findCharacterIndex(barcodeRead[1]);
            char *characterArray = patternMappings[characterIndex];
            char *barcodeDecoded = reverseString(characterArray);
            barcodeCharacter = decodeReverse(barcodeDecoded);

            printed = snprintf(pcInsert, iInsertLen, "%c\n\r", barcodeCharacter);

        }
    }
    break;
    case 1: // status
    {
        bool status = cyw43_arch_gpio_get(CYW43_WL_GPIO_LED_PIN);
        if (status == true)
        {
            printed = snprintf(pcInsert, iInsertLen, "START");
        }
        else
        {
            printed = snprintf(pcInsert, iInsertLen, "STOP");
        }
    }
    break;

    default:
        printed = 0;
        break;
    }

    return (u16_t)printed;
}

// Initialise the SSI handler
void ssi_init()
{
    // Initialise ADC (internal pin)
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

    http_set_ssi_handler(ssi_handler, ssi_tags, LWIP_ARRAYSIZE(ssi_tags));
}

// CGI handler which is run when a request for /led.cgi is detected
const char *cgi_led_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    // Check if an request for LED has been made (/led.cgi?led=x)
    if (strcmp(pcParam[0], "led") == 0)
    {
        // Look at the argument to check if LED is to be turned on (x=1) or off (x=0)
        if (strcmp(pcValue[0], "0") == 0)
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        
        else if (strcmp(pcValue[0], "1") == 0)
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    }

    // Send the index page back to the user
    return "/index.shtml";
}

const char *cgi_message_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    // Check if message is sent (/send.cgi?send=x)
    if (strcmp(pcParam[0], "send") == 0)
    {
        // Look at the argument to check if message received is to be "start" or "stop"
        if (strcmp(pcValue[0], "stop") == 0)
        {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        }
        else if (strcmp(pcValue[0], "start") == 0)
        {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        }
    }

    // Send the index page back to the user
    return "/index.shtml";
}

// tCGI Struct
// Fill this with all of the CGI requests and their respective handlers
static const tCGI cgi_handlers[] = {
    {// Html request for "/led.cgi" triggers cgi_handler
     "/led.cgi", cgi_led_handler

    },
    {"/send.cgi", cgi_message_handler},
};

void cgi_init(void)
{
    http_set_cgi_handlers(cgi_handlers, 2);
}


void wifiTask()
{

     // init the queue
     flushVoltageClassification();

    adc_gpio_init(ADC_PIN);
    //adc_select_input(0);

    adc_fifo_setup(true, false, 1, false, false);
    adc_set_clkdiv(0);
    adc_irq_set_enabled(true);

    irq_clear(ADC_IRQ_FIFO);

    irq_set_exclusive_handler(ADC_IRQ_FIFO, ADC_IRQ_FIFO_HANDLER);
    irq_set_enabled(ADC_IRQ_FIFO, true);

    adc_run(true);

    // wifi init
    cyw43_arch_init();

    cyw43_arch_enable_sta_mode();

    // Connect to the WiFI network - loop until connected
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000) != 0)
    {
        printf("Attempting to connect...\n");
    }
    // Print a success message once connected
    printf("Connected! \n");

    // Initialise web server
    httpd_init();
    printf("Http server initialised\n");

    // Configure SSI and CGI handler
    ssi_init();
    printf("SSI Handler initialised\n");
    cgi_init();
    printf("CGI Handler initialised\n");

    // ========================================================
    //adc_init();

    while (true)
    {
        printf("wifi task \n");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}



///////////Wheel Encoder////////////
void gpio_callback(uint gpio, uint32_t events) {

    if (gpio == ECHO_PIN && GPIO_IRQ_EDGE_RISE)
    {
        //printf("rise \n");
        gpio_put(TRIGGER_PIN, 0);
        startTime = get_absolute_time();
    }

    if (gpio == ECHO_PIN && GPIO_IRQ_EDGE_FALL)
    {
        //printf("fall \n");
        endTime = get_absolute_time();
        if (absolute_time_diff_us(startTime, endTime) > TIMEOUT_US)
        {
            startTime = endTime;
        }
        getUltrasonicDetection(absolute_time_diff_us(startTime, endTime));
    }

    // Put the GPIO event(s) that just happened into event_str
    // so we can print it
    // static uint32_t edge_fall_time_l;
    // static uint32_t edge_fall_time_r;
    // //gpio_event_string(event_str, events);
    // if (gpio == 6){ // Edge fall
    //     pulse_width_l = (float) (time_us_32() - edge_fall_time_l)/(1000000.0f);
    //     num_edge_l++;
    //     edge_fall_time_l = time_us_32(); // Time is in microseconds
    // } else if (gpio == 7){    // Edge rise
    //     pulse_width_r = (float) (time_us_32() - edge_fall_time_r)/(1000000.0f);
    //     num_edge_r++;
    //     edge_fall_time_r = time_us_32(); // Time is in microseconds
    // }
}

// bool print_out(struct repeating_timer *t) {
//     float speed_per_sec_l = 0; // Measure in cm/s
//     float speed_per_sec_r = 0; // Measure in cm/s
//     float distance_per_sec = 0; // Measured in cm
//     // Approximation of distance travelled. Found using diameter of wheel encoder disc (2cm), circumfrence (6.28cm), Each slit + pillar is approx. 6.28/20 = 0.314cm
//     distance_per_sec= (((num_edge_l+num_edge_r)/2)*0.314); 
//     t_distance_travelled += distance_per_sec;
//     // 0.314cm is the estimatd length of each slit
//     speed_per_sec_l = (pulse_width_l > 0) ? 0.314/pulse_width_l : 0; 
//     speed_per_sec_r = (pulse_width_r > 0) ? 0.314/pulse_width_r : 0; 
//     //printf("Total distance: %f\n", t_distance_travelled);
//     //printf("Speed using edge per sec: %f cm/s\n", distance_per_sec);
//     //printf("Speed using left pluse width: %f cm/s\n", speed_per_sec_l);
//     //printf("Speed using right pluse width: %f cm/s\n\n", speed_per_sec_r);
//     num_edge_l = 0;
//     num_edge_r = 0; //Reset number of edge per second
//     return true;
// }

//////////ULTRASONIC///////////////

// Timer to send ultrasonic initiation pulse to initiate a measurement
bool repeating_timer_callback(struct repeating_timer *t)
{
    sendInitiationPulse(TRIGGER_PIN);
    return true;
}

// Function to setup the ultrasonic pins
void setupUltrasonicPins(uint trigPin, uint echoPin)
{
    gpio_init(trigPin);
    gpio_init(echoPin);
    gpio_set_dir(trigPin, GPIO_OUT);
    gpio_set_dir(echoPin, GPIO_IN);
    gpio_pull_up(echoPin);

    // Set up interrupt for the echoPin
    gpio_set_irq_enabled_with_callback(echoPin, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

}

// Function for trigger pin to send out the initiation pulse
void sendInitiationPulse(uint trigPin)
{
    gpio_put(trigPin, 1);
    busy_wait_us(10);
    gpio_put(trigPin, 0);
}

// Function to calculate the distance from the ultrasonic
void getUltrasonicDetection(uint64_t pulseLength)
{
    // Convert the ultrasonic pulse into cm
    float ultrasonic_in_cm = pulseLength / 29 / 2;

    // Get the average of the ultrasonic for smoothing
    float ultrasonic_average = 0;
    for (int i = 0; i < 50; ++i)
    {
        ultrasonic_average = ultrasonic_average + ultrasonic_in_cm;
    }

    ultrasonic_average = (ultrasonic_average / 50);

    if(ultrasonic_average <= 15 && ultrasonic_average > 0 && moving)
    {
        stopCar(slice_num0, slice_num1);
        moving = false;
    }

    else if (ultrasonic_average > 15 && ultrasonic_average != 0 && !moving)
    {
        startCar(slice_num0, slice_num1);
        moving = true;
    }

    // Display the results obtained after smoothing
    printf("Moving Distance: %.2f cm\n\n", ultrasonic_average);
}

void UltrasonicTask()
{
    setupUltrasonicPins(TRIGGER_PIN, ECHO_PIN);

    struct repeating_timer timer;
    add_repeating_timer_ms(100, repeating_timer_callback, NULL, &timer);

    while(1)
    {
        printf("ultra task \n");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

//////////MOTOR///////////////
void SetupMotorPins()
{
    gpio_set_dir(BTN_PIN1, GPIO_IN);
    gpio_set_pulls(BTN_PIN1, left1, left2);
    gpio_set_dir(BTN_PIN2, GPIO_IN);
    gpio_set_pulls(BTN_PIN2, left3, left4);

    gpio_set_dir(BTN_PIN3, GPIO_IN);
    gpio_set_pulls(BTN_PIN3, right1, right2);
    gpio_set_dir(BTN_PIN4, GPIO_IN);
    gpio_set_pulls(BTN_PIN4, right3, right4);

    // Tell GPIO 0 and 1 they are allocated to the PWM
    gpio_set_function(0, GPIO_FUNC_PWM);
    gpio_set_function(1, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    slice_num0 = pwm_gpio_to_slice_num(0);
    pwm_set_clkdiv(slice_num0,100);
    // this code here means that it will run 12500 cycles per second
    pwm_set_wrap(slice_num0, 12500);

     // Find out which PWM slice is connected to GPIO 1 (it's slice 1)
    slice_num1 = pwm_gpio_to_slice_num(1);
    pwm_set_clkdiv(slice_num1,100);
    // this code here means that it will run 12500 cycles per second
    pwm_set_wrap(slice_num1, 12500);
}

// function to stop car
void stopCar(uint slice_num0, uint slice_num1)
{ 
    //stop the car by changing duty cycle to 0%
    pwm_set_chan_level(slice_num0, PWM_CHAN_A, 0);  // Stop Channel A
    pwm_set_chan_level(slice_num1, PWM_CHAN_B, 0);  // Stop Channel B
    pwm_set_enabled(slice_num0, true);
    pwm_set_enabled(slice_num1, true);
}

// function to start car
void startCar(uint slice_num0, uint slice_num1)
{ 
    pwm_set_chan_level(slice_num0, PWM_CHAN_A, speedRight);  // Start Channel A
    pwm_set_chan_level(slice_num1, PWM_CHAN_B, speedLeft);  // Start Channel B
    pwm_set_enabled(slice_num0, true);
    pwm_set_enabled(slice_num1, true);
}

// function to turn sharp left from stationary [REQUIRES TO STOP CAR FIRST]
void turnSharpLeft(uint slice_num0, uint slice_num1, uint BTN_PIN3, uint BTN_PIN4, bool right1, bool right2, bool right3, bool right4)
{ 
    // change direction of left wheel to turn 90 degrees
    gpio_set_pulls(BTN_PIN3, !right1, !right2);
    gpio_set_pulls(BTN_PIN4, !right3, !right4);

    pwm_set_chan_level(slice_num0, PWM_CHAN_A, 8600);  // Start Channel A
    pwm_set_chan_level(slice_num1, PWM_CHAN_B, 8600);  // Start Channel B
    pwm_set_enabled(slice_num0, true);
    pwm_set_enabled(slice_num1, true);
}

// function to turn sharp right from stationary [REQUIRES TO STOP CAR FIRST]
void turnSharpRight(uint slice_num0, uint slice_num1, uint BTN_PIN1, uint BTN_PIN2, bool left1, bool left2, bool left3, bool left4)
{ 
    // change direction of right wheel to turn 90 degrees
    gpio_set_pulls(BTN_PIN1, !left1, !left2);
    gpio_set_pulls(BTN_PIN2, !left3, !left4);

    //pwm_set_chan_level(slice_num0, PWM_CHAN_A, 12500/2);  // Start Channel A
    //pwm_set_chan_level(slice_num1, PWM_CHAN_B, 12500/2);  // Start Channel B
    pwm_set_chan_level(slice_num0, PWM_CHAN_A, 8600);  // Start Channel A
    pwm_set_chan_level(slice_num1, PWM_CHAN_B, 8600);  // Start Channel B
    pwm_set_enabled(slice_num0, true);
    pwm_set_enabled(slice_num1, true);
}

// void recenterLeft(uint slice_num0, uint slice_num1){

//     pwm_set_chan_level(slice_num0, PWM_CHAN_A, speedRight);  // Start Channel A
//     pwm_set_chan_level(slice_num1, PWM_CHAN_B, 0.7 * speedLeft);  // Start Channel B
//     pwm_set_enabled(slice_num0, true);
//     pwm_set_enabled(slice_num1, true);
// }

// void recenterRight(uint slice_num0, uint slice_num1){
//     pwm_set_chan_level(slice_num0, PWM_CHAN_A, 0.5 * speedRight);  // Start Channel A
//     pwm_set_chan_level(slice_num1, PWM_CHAN_B, speedLeft);  // Start Channel B
//     pwm_set_enabled(slice_num0, true);
//     pwm_set_enabled(slice_num1, true);
// }

//////////IR SENSOR///////////////
void SetupIRSensorPins()
{
    adc_init();

    gpio_set_function(ADC_BASE_PIN1, GPIO_FUNC_SIO);
    gpio_disable_pulls(ADC_BASE_PIN1);
    gpio_set_input_enabled(ADC_BASE_PIN1, false);

    gpio_set_function(ADC_BASE_PIN2, GPIO_FUNC_SIO);
    gpio_disable_pulls(ADC_BASE_PIN2);
    gpio_set_input_enabled(ADC_BASE_PIN2, false);
}

void ReadIRSensor()
{
    const float conversion_factor = 3.3f / (1 << 12);
    while (1) 
    {
        adc_run(false);
        adc_select_input(ADC_CHANNEL_PIN_ONE);
        uint32_t leftRead = adc_read();
        float leftResult = leftRead * conversion_factor;

        adc_select_input(ADC_CHANNEL_PIN_TWO);
        uint32_t rightRead = adc_read();
        float rightResult = rightRead * conversion_factor;

        if(leftResult < WHITE_TRACKING_THRESHOLD && rightResult > WHITE_TRACKING_THRESHOLD && !turningRight && !turningLeft)
        {
            targetAngle = heading - 90;
            turningLeft = true;
            turnSharpLeft(slice_num0, slice_num1, BTN_PIN3, BTN_PIN4, right1, right2, right3, right4);
        }

        else if (rightResult < WHITE_TRACKING_THRESHOLD && leftResult > WHITE_TRACKING_THRESHOLD && !turningRight && !turningLeft)
        {
            targetAngle = heading + 90;
            turningRight = true;
            turnSharpRight(slice_num0, slice_num1, BTN_PIN1, BTN_PIN2, left1, left2, left3, left4);
        }

        else if (rightResult > WHITE_TRACKING_THRESHOLD && leftResult > WHITE_TRACKING_THRESHOLD)
        {
            startCar(slice_num0, slice_num1);
            turningLeft = false;
            turningRight = false;
        }

        else
        {
            stopCar(slice_num0, slice_num1);
        }

        adc_run(true);
        //go back to barcode tracking
        adc_select_input(0);

        printf("ir task \n");
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

//////////MAGNETOMETER///////////////
void configureRegister(uint8_t addr, uint8_t reg, uint8_t value) 
{
    uint8_t data[2] = {reg, value};
    i2c_write_blocking(i2c0, addr, data, 2, false);
}

uint8_t readRegister(uint8_t addr, uint8_t reg) 
{
    uint8_t data;
    i2c_write_blocking(i2c0, addr, &reg, 1, true);
    i2c_read_blocking(i2c0, addr, &data, 1, false);
    return data;
}

void setupMagnetometer()
{
    i2c_init(i2c0, 100000);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SCL_PIN);
    gpio_pull_up(SDA_PIN);

    // Initialize accelerometer settings
    configureRegister(ACCEL_ADDR, CTRL_REG1_A, 0x57);
    configureRegister(ACCEL_ADDR, CTRL_REG4_A, 0x30);

    // Initialize magnetometer settings
    configureRegister(MAG_ADDR, MR_REG_M, 0x00);
}


void readAccelerometer(int16_t *x, int16_t *y, int16_t *z) 
{
    *x = (int16_t)((readRegister(ACCEL_ADDR, ACCEL_X_HIGH) << 8) | readRegister(ACCEL_ADDR, ACCEL_X_LOW));
    *y = (int16_t)((readRegister(ACCEL_ADDR, ACCEL_Y_HIGH) << 8) | readRegister(ACCEL_ADDR, ACCEL_Y_LOW));
    *z = (int16_t)((readRegister(ACCEL_ADDR, ACCEL_Z_HIGH) << 8) | readRegister(ACCEL_ADDR, ACCEL_Z_LOW));

    *x /= CONVERSION;
    *y /= CONVERSION;
    *z /= CONVERSION;
}

void readMagnetometer(int16_t *x, int16_t *y, int16_t *z) 
{
    *x = (int16_t)((readRegister(MAG_ADDR, MAG_X_HIGH) << 8) | readRegister(MAG_ADDR, MAG_X_LOW));
    *y = (int16_t)((readRegister(MAG_ADDR, MAG_Y_HIGH) << 8) | readRegister(MAG_ADDR, MAG_Y_LOW));
    *z = (int16_t)((readRegister(MAG_ADDR, MAG_Z_HIGH) << 8) | readRegister(MAG_ADDR, MAG_Z_LOW));
}

void readAllMagnetometerData()
{
    int16_t MagX, MagY, MagZ;
    int16_t accelX, accelY, accelZ;    

    while (1)
    {
        readAccelerometer(&accelX, &accelY, &accelZ);

        readMagnetometer(&MagX, &MagY, &MagZ);

        heading = atan2(MagY, MagX);
        heading *= 180.0f / M_PI;

        if(heading < 0) {
            heading += 360;
        }

        if(turningLeft || turningRight)
        {
            if(abs(heading - targetAngle) <= 1)
            {
                startCar(slice_num0, slice_num1);
                turningLeft = false;
                turningRight = false;
            }
        }

        printf("mag task \n");
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    }

void vLaunch( void) 
{

// Create the various tasks to read, calculate and print the average temperatures

    TaskHandle_t irSensorTask;
    xTaskCreate(ReadIRSensor, "IRSensorThread", configMINIMAL_STACK_SIZE , NULL, 3, &irSensorTask);

    TaskHandle_t ultrasonicSensorTask;
    xTaskCreate(UltrasonicTask, "ultrasonicThread", configMINIMAL_STACK_SIZE, NULL, 2, &ultrasonicSensorTask);

    TaskHandle_t wifiDataTask;
    xTaskCreate(wifiTask, "WifiTask", configMINIMAL_STACK_SIZE, NULL, 4, &wifiDataTask);

    TaskHandle_t magnetometerSensorTask;   
    xTaskCreate(readAllMagnetometerData, "MagetometerTask", configMINIMAL_STACK_SIZE, NULL, 1, &magnetometerSensorTask);

#if NO_SYS && configUSE_CORE_AFFINITY && configNUM_CORES > 1
    // we must bind the main task to one core (well at least while the init is called)
    // (note we only do this in NO_SYS mode, because cyw43_arch_freertos
    // takes care of it otherwise)
    vTaskCoreAffinitySet(task, 1);
#endif

    // Start the tasks and timer running
    vTaskStartScheduler();
}

int main( void )
{
    stdio_init_all();
    SetupIRSensorPins();
    SetupMotorPins();
    setupMagnetometer();

    //startCar(slice_num0, slice_num1);
    //gpio 6 -left
    //gpio 7 - right
    // gpio_set_function(6, GPIO_IN);
    // gpio_set_function(7, GPIO_IN);


    // // Configure GPIO pin 6
    // gpio_set_irq_enabled_with_callback(6, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    // // Configure GPIO pin 7
    // gpio_set_irq_enabled_with_callback(7, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // struct repeating_timer print_timer;
    // add_repeating_timer_ms(-1000, print_out, NULL, &print_timer);

    /* Configure the hardware ready to run the demo. */
    const char *rtos_name;
#if ( portSUPPORT_SMP == 1 )
    rtos_name = "FreeRTOS SMP";
#else
    rtos_name = "FreeRTOS";
#endif

#if ( portSUPPORT_SMP == 1 ) && ( configNUM_CORES == 2 )
    printf("Starting %s on both cores:\n", rtos_name);
    vLaunch();
#elif ( RUN_FREERTOS_ON_CORE == 1 )
    printf("Starting %s on core 1:\n", rtos_name);
    multicore_launch_core1(vLaunch);
    while (true);
#else
    printf("Starting %s on core 0:\n", rtos_name);
    vLaunch();
#endif
    return 0;
}