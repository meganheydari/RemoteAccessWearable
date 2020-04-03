// def for communication
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "protocol_examples_common.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
// end def

// def for LED
#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
// end def

#include <math.h>

// defines for displays
#include <stdio.h>
#include <string.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_vfs_dev.h"
#include "esp_adc_cal.h"

// Seriel
#include "esp_system.h"
#include "esp_log.h"
#include "string.h"


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "./ADXL343.h"
#include <stdio.h>
#include <stdlib.h>


#define BLINK_GPIO 21 //SCK


#define GPIO_INPUT_IO_0     4
#define GPIO_INPUT_PIN_SEL  1ULL<<GPIO_INPUT_IO_0
#define ESP_INTR_FLAG_DEFAULT 0
static xQueueHandle gpio_evt_queue = NULL;


//def for LED
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (26)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_HS_CH1_GPIO       (25)
#define LEDC_HS_CH1_CHANNEL    LEDC_CHANNEL_0

#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_LS_CH2_GPIO       (4)
#define LEDC_LS_CH2_CHANNEL    LEDC_CHANNEL_0
#define LEDC_LS_CH3_GPIO       (21)
#define LEDC_LS_CH3_CHANNEL    LEDC_CHANNEL_0

#define LEDC_TEST_CH_NUM       (4)
#define LEDC_TEST_DUTY         (4000)
#define LEDC_TEST_FADE_TIME    (3000)

// end def for led

// def for communication
#ifdef CONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#else
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#endif

#define PORT CONFIG_EXAMPLE_PORT

static const char *TAG = "example";
static const char *initPayload = "INIT";
// end def

// ====== for volatage dividor

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value


#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)


// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

// ADXL343
#define SLAVE_ADDR                         ADXL343_ADDRESS // 0x53


static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t battery_channel = ADC_CHANNEL_0;
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit1 = ADC_UNIT_1;

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define BATTERY_NO_OF_SAMPLES   10         //Multisampling
#define RANGE_NO_OF_SAMPLES   40         //Multisampling
#define ULTRA_NO_OF_SAMPLES   35
#define THER_NO_OF_SAMPLES   40

uint32_t battery_voltage; //mV
double battery_voltage_V;
double temperature; //C
int steps = 0;
// end voltage dividor

float roll = 0;
float pitch = 0;
int count = 0;
int yes = 0;


int brightness = 0;
TaskHandle_t xHandle = NULL;

static void toggle_LED_task(void *arg)
{
    // initialize led confg

    int ch;

        /*
         * Prepare and set configuration of timers
         * that will be used by LED Controller
         */
        ledc_timer_config_t ledc_timer = {
            .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
            .freq_hz = 5000,                      // frequency of PWM signal
            .speed_mode = LEDC_HS_MODE,           // timer mode
            .timer_num = LEDC_HS_TIMER,            // timer index
            .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
        };
        // Set configuration of timer0 for high speed channels
        ledc_timer_config(&ledc_timer);

        // Prepare and set configuration of timer1 for low speed channels
        ledc_timer.speed_mode = LEDC_LS_MODE;
        ledc_timer.timer_num = LEDC_LS_TIMER;
        ledc_timer_config(&ledc_timer);

        /*
         * Prepare individual configuration
         * for each channel of LED Controller
         * by selecting:
         * - controller's channel number
         * - output duty cycle, set initially to 0
         * - GPIO number where LED is connected to
         * - speed mode, either high or low
         * - timer servicing selected channel
         *   Note: if different channels use one timer,
         *         then frequency and bit_num of these channels
         *         will be the same
         */
        ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
            {
                .channel    = LEDC_HS_CH0_CHANNEL,
                .duty       = 0,
                .gpio_num   = LEDC_HS_CH0_GPIO,
                .speed_mode = LEDC_HS_MODE,
                .hpoint     = 0,
                .timer_sel  = LEDC_HS_TIMER
            },
            {
                .channel    = LEDC_HS_CH1_CHANNEL,
                .duty       = 0,
                .gpio_num   = LEDC_HS_CH1_GPIO,
                .speed_mode = LEDC_HS_MODE,
                .hpoint     = 0,
                .timer_sel  = LEDC_HS_TIMER
            },
            {
                .channel    = LEDC_LS_CH2_CHANNEL,
                .duty       = 0,
                .gpio_num   = LEDC_LS_CH2_GPIO,
                .speed_mode = LEDC_LS_MODE,
                .hpoint     = 0,
                .timer_sel  = LEDC_LS_TIMER
            },
            {
                .channel    = LEDC_LS_CH3_CHANNEL,
                .duty       = 0,
                .gpio_num   = LEDC_LS_CH3_GPIO,
                .speed_mode = LEDC_LS_MODE,
                .hpoint     = 0,
                .timer_sel  = LEDC_LS_TIMER
            },
        };

         ch = 1;

            // Set LED Controller with previously prepared configuration
        //for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
            ledc_channel_config(&ledc_channel[ch]);
       // }

        // Initialize fade service.
        ledc_fade_func_install(0);

    // === end init
    //for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
                ledc_channel[ch].channel, 11 * 444, 1000);
        ledc_fade_start(ledc_channel[ch].speed_mode,
                ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
    //}
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    //for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
                ledc_channel[ch].channel, 1 * 444, 1000);
        ledc_fade_start(ledc_channel[ch].speed_mode,
                ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
    //}
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    //for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
                ledc_channel[ch].channel, 11 * 444, 1000);
        ledc_fade_start(ledc_channel[ch].speed_mode,
                ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
    //}
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    //for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
                ledc_channel[ch].channel, 0 * 444, 200);
        ledc_fade_start(ledc_channel[ch].speed_mode,
                ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
    //}
    vTaskDelay(200 / portTICK_PERIOD_MS);

    xHandle = NULL;
    vTaskDelete(xHandle);
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

static void voltageDivider()
{
    //Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(battery_channel, atten);

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit1, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    //Continuously sample ADC1
    while (1) {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < BATTERY_NO_OF_SAMPLES; i++) {
            adc_reading += adc1_get_raw((adc1_channel_t)battery_channel);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        adc_reading /= BATTERY_NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        battery_voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        battery_voltage_V = battery_voltage/1000.00;
        //printf("Raw: %d\tVoltage: %dmV\n", adc_reading, battery_voltage);
    }
}

static void LED_light_task_init(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {

        #ifdef CONFIG_EXAMPLE_IPV4
                struct sockaddr_in dest_addr;
                dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
                dest_addr.sin_family = AF_INET;
                dest_addr.sin_port = htons(PORT);
                addr_family = AF_INET;
                ip_protocol = IPPROTO_IP;
                inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
        #else // IPV6
                struct sockaddr_in6 dest_addr;
                inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
                dest_addr.sin6_family = AF_INET6;
                dest_addr.sin6_port = htons(PORT);
                addr_family = AF_INET6;
                ip_protocol = IPPROTO_IPV6;
                inet6_ntoa_r(dest_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
        #endif

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        int err = sendto(sock, initPayload, strlen(initPayload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
        ESP_LOGI(TAG, "Message sent");
            
        while (1) {
            
        
            struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                printf("%s\n", rx_buffer);

                if (strcmp(rx_buffer,"Light") == 0 && xHandle == NULL) {

                    xTaskCreate(toggle_LED_task, "toggle_LED_task", 4096, NULL, 5, &xHandle);
                }

                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);
            }

            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

void thermistor()
{
    //Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL_3, atten);

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit1, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    double thermometerResistance = 0.0;
    double temp = 0.0;
    double a = 5, b = 1000, c = 216;
    double r_zero = 10000, t_zero = 298.15, beta = 3435, ktoc = 273.15;

    while (1) {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < 40; i++) {
            adc_reading += adc1_get_raw((adc1_channel_t)ADC_CHANNEL_3);
        }
        adc_reading /= 40;
        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        thermometerResistance = a * c / (voltage/b) - c;
        temp = (t_zero * beta)/(t_zero * log(thermometerResistance/r_zero) + beta) - ktoc;
        temperature = temp;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}
static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    int i=0;
    int count = 0;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            //printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            i++;
            if(i==50) {
              count++;
              printf("tap number: %d\n",count);
              steps = count;
              i = 0;
            }
        }
    }
}

static void send_sensor_data()
{
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {

        #ifdef CONFIG_EXAMPLE_IPV4
                struct sockaddr_in dest_addr;
                dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
                dest_addr.sin_family = AF_INET;
                dest_addr.sin_port = htons(PORT);
                addr_family = AF_INET;
                ip_protocol = IPPROTO_IP;
                inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
        #else // IPV6
                struct sockaddr_in6 dest_addr;
                inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
                dest_addr.sin6_family = AF_INET6;
                dest_addr.sin6_port = htons(PORT);
                addr_family = AF_INET6;
                ip_protocol = IPPROTO_IPV6;
                inet6_ntoa_r(dest_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
        #endif

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        while (1) {

            // Allocates storage
            char *payload = (char*)malloc(5000 * sizeof(char));
            // Prints "Hello world!" on hello_world
            sprintf(payload, "{\"thermistor\" : %.3f, \"voltage\": %.3f, \"steps\": %d}", temperature, battery_voltage_V, steps);

            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "Message sent");

        
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

void blink_task()
{
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BLINK_GPIO, 0);
    while (1) {
        bool toggle = 0;

        for (int i=0; i<7; i++) {
            gpio_set_level(BLINK_GPIO, toggle);
            vTaskDelay(1000 / portTICK_RATE_MS);
            toggle = !toggle;
        }

        for (int j=0; j<3600; j++) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);   /* wait 1h (block) */
        }
    }
}

// Function to initiate i2c -- note the MSB declaration!
static void i2c_master_init()
{
  // Debug
  printf("\n>> i2c Config\n");
  int err;

  // Port configuration
  int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

  /// Define I2C configurations
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;                              // Master mode
  conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
  err = i2c_param_config(i2c_master_port, &conf);           // Configure
  if (err == ESP_OK) {printf("- parameters: ok\n");}

  // Install I2C driver
  err = i2c_driver_install(i2c_master_port, conf.mode,
                     I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                     I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
  if (err == ESP_OK) {printf("- initialized: yes\n");}

  // Data in MSB mode
  i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
    // Data in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility  Functions //////////////////////////////////////////////////////////

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return err;
}

// Utility function to scan for i2c device
static void i2c_scanner() {
  int32_t scanTimeout = 1000;
  printf("\n>> I2C scanning ..."  "\n");
  uint8_t count = 0;
  for (uint8_t i = 1; i < 127; i++) {
    // printf("0x%X%s",i,"\n");
    if (testConnection(i, scanTimeout) == ESP_OK) {
      printf( "- Device found at address: 0x%X%s", i, "\n");
      count++;
    }
  }
  if (count == 0) {printf("- No I2C devices found!" "\n");}
}


////////////////////////////////////////////////////////////////////////////////

// ADXL343 Functions ///////////////////////////////////////////////////////////

// Get Device ID
// very similar to singl byte read, except that function does not take an input register 
// THIS CODE WAS GIVEN 
int getDeviceID(uint8_t *data) {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, ADXL343_REG_DEVID, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
    printf("%p\n", cmd);

  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

// Write one byte to register
int writeRegister(uint8_t reg, uint8_t data) {
  int ret = 0;
  // YOUR CODE HERE
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);  // 1. Start
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN); // 2.-3.
  //give the reg address
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN); // 4.-5.
  // Master writes the data and waits for ack from slave
  i2c_master_write_byte(cmd, data, ACK_CHECK_EN); 
  i2c_master_stop(cmd); // 11. Stop
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS); // This starts the I2C communication
  i2c_cmd_link_delete(cmd);

  return ret; 

}

// Read register 8 bits (1 bytes)
uint8_t readRegister(uint8_t reg) {
  uint8_t  data = 0;
  // YOUR CODE HERE
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);  
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN); 
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN); // replaced ADXL343_REG_DEVID with reg 
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);  
  i2c_master_read_byte(cmd, &data, ACK_CHECK_DIS);
  i2c_master_stop(cmd); // 11. Stop
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return data;
}

// read 16 bits (2 bytes)
int16_t read16(uint8_t reg) {
  //return ret;
  // YOUR CODE HERE
  uint8_t data1 = 0;
  uint8_t data2 = 0;
  int16_t  overall; 

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);  // 1. Start
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN); // 2.-3.
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN); // 4.-5.
  i2c_master_start(cmd); // 6.
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);  // 7.-8.
  i2c_master_read_byte(cmd, &data1, ACK_VAL); // 9.-10.
  i2c_master_read_byte(cmd, &data2, ACK_CHECK_DIS); // 9.-10.
  i2c_master_stop(cmd); // 11. Stop
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  // push data 2 into first 8 bits and 
  overall = (  ((int16_t) data2)  << 8) + ((int16_t) data1)  ;
  overall =  (~ ((int16_t) overall) + 1);
  return overall;
}

void setRange(range_t range) {
  /* Red the data format register to preserve bits */
  uint8_t format = readRegister(ADXL343_REG_DATA_FORMAT);

  /* Update the data rate */
  format &= ~0x0F;
  format |= range;

  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;

  /* Write the register back to the IC */
  writeRegister(ADXL343_REG_DATA_FORMAT, format);

}

range_t getRange(void) {
  /* Red the data format register to preserve bits */
  return (range_t)(readRegister(ADXL343_REG_DATA_FORMAT) & 0x03);
}

dataRate_t getDataRate(void) { return (dataRate_t)(readRegister(ADXL343_REG_BW_RATE) & 0x0F); }

// function to get acceleration
void getAccel(float * xp, float *yp, float *zp) 
{
  *xp = read16(ADXL343_REG_DATAX0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  *yp = read16(ADXL343_REG_DATAY0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  *zp = read16(ADXL343_REG_DATAZ0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
 // printf("X: %.2f \t Y: %.2f \t Z: %.2f\n", *xp, *yp, *zp);
}

// function to print roll and pitch
void calcRP(float x, float y, float z)
{
  // YOUR CODE HERE
  float roll = atan2(y , z) * 57.3;
  float pitch = atan2((- x) , sqrt(y * y + z * z)) * 57.3;
  // printf("CALCRP\n");
  // printf("roll: %.2f \t pitch: %.2f \n", roll, pitch);
}

void countSteps(float x, float y, float z)
{
  float troll = atan2(y , z) * 57.3;
  float tpitch = atan2((- x) , sqrt(y * y + z * z)) * 57.3;
  // printf("troll: %.2f\n", troll);
  // printf("roll: %.2f\n", roll);

  if (abs(troll - roll) > 20 || abs(tpitch - pitch) > 20) 
  {
    if (yes == 1){
      steps += 1;
      printf("Steps #: %d \n", steps);
      yes = 0;
    }
    else {
      yes = 1; 
    }

  roll = troll;
  pitch = tpitch;

  }
}

// Task to continuously poll acceleration and calculate roll and pitch
static void test_adxl343() 
{
  printf("\n>> Polling ADAXL343\n");
  while (1) {
    float xVal, yVal, zVal;
    getAccel(&xVal, &yVal, &zVal);
    calcRP(xVal, yVal, zVal);
    countSteps(xVal, yVal, zVal);
    vTaskDelay(500 / portTICK_RATE_MS);
  }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(example_connect());

    // all tasks are created here and run simultaniously.
    xTaskCreate(LED_light_task_init, "LED_light_task_init", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(voltageDivider, "voltageDivider", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(thermistor, "thermistor", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(&blink_task, "blink_task", 4096, NULL, configMAX_PRIORITIES, NULL);

    gpio_config_t io_conf;
        //disable interrupt
        io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
        //disable pull-down mode
        io_conf.pull_down_en = 0;
        //disable pull-up mode
        io_conf.pull_up_en = 0;
        //configure GPIO with the given settings
        gpio_config(&io_conf);
        //interrupt of rising edge
        io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
        //bit mask of the pins, use GPIO4/5 here
        io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
        //set as input mode
        io_conf.mode = GPIO_MODE_INPUT;
        //enable pull-up mode
        io_conf.pull_up_en = 1;
        gpio_config(&io_conf);
        //change gpio intrrupt type for one pin
        gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);
        //create a queue to handle gpio event from isr
        gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
        //start gpio task
        xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
        //install gpio isr service
        gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
        //hook isr handler for specific gpio pin
        gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
        //remove isr handler for gpio number.
        gpio_isr_handler_remove(GPIO_INPUT_IO_0);
        //hook isr handler for specific gpio pin again
        gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);

    xTaskCreate(send_sensor_data, "send_sensor_data", 4096, NULL, configMAX_PRIORITIES, NULL);

        // Routine
        i2c_master_init();
        i2c_scanner();

        // Disable interrupts
        writeRegister(ADXL343_REG_INT_ENABLE, 0);

        // Set range
        setRange(ADXL343_RANGE_16_G);

        // Enable measurements
        writeRegister(ADXL343_REG_POWER_CTL, 0x08);

    // Create task to poll ADXL343
    xTaskCreate(test_adxl343,"test_adxl343", 4096, NULL, 5, NULL);
}
