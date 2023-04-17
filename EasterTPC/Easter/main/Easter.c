
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "lib/spi_25LC040A_eeprom.h"

static const char *TIMER_TAG_5sec = "TIMER_5sec";
static const char *TIMER_TAG_100ms = "TIMER_100ms";
static const char *SPI_TAG = "SPI_TAG";
static const char *I2C_TAG = "I2C_TAG";

volatile int sampleCount = 0;
volatile uint8_t data[2];
volatile bool flagSample = false;


// ==================== I2C FUNCTIONS ======================= v1
#define I2C_MASTER_SCL_IO         22  //CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO         21  //CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM             0                   /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ        50000                /*!< I2C master clock frequency */
#define GPIO_PULLUP_ENABLE         1            
#define I2C_MASTER_TIMEOUT_MS     1000
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need */

#define MPU9250_SENSOR_ADDR      0x4d       /*!< Slave address of the MPU9250 sensor */

static esp_err_t i2c_master_init(void){

    ESP_LOGI(I2C_TAG, "Initiating I2C TL94");
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER, 
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);
    ESP_LOGI(I2C_TAG, "Initiating I2C TL94");
    printf("INSTALLING DRIVER\n");
    i2c_driver_install(i2c_master_port, conf.mode, 1024, 1024, 0);
    ESP_LOGI(I2C_TAG, "Successfully installed I2C driver");

    return 0;
}

//===================== SPI FUNCTIONS ======================== v1
    #define SPI_MASTER_HOST SPI3_HOST
    #define PIN_SPI_SS      5   
    #define PIN_SPI_SCLK    18
    #define PIN_SPI_MOSI    23
    #define PIN_SPI_MISO    19
    #define SPI_CLK_SPEED   1000000

esp_err_t spi_25LC040_init(spi_host_device_t masterHostId, int csPin, int sckPin, int mosiPin, int misoPin, int clkSpeedHz, spi_device_handle_t pDevHandle){

    esp_err_t ret;
    spi_device_handle_t spiHandle;

    spi_bus_config_t spiBusCfg = {
        .mosi_io_num = mosiPin,
        .miso_io_num = misoPin,
        .sclk_io_num = sckPin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
        .flags = SPICOMMON_BUSFLAG_MASTER
    };

    spi_device_interface_config_t spiDeviceCfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .clock_speed_hz = clkSpeedHz,
        .spics_io_num = csPin,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .queue_size = 1
    };

    ret = spi_bus_initialize(masterHostId, &spiBusCfg, SPI_DMA_DISABLED);
    ESP_ERROR_CHECK(ret);
    printf("yo\n");

    ret= spi_bus_add_device(masterHostId, &spiDeviceCfg, &spiHandle);
    ESP_ERROR_CHECK(ret);

    return ret;
}

esp_err_t spi_25LC040_free(spi_host_device_t masterHostId, spi_device_handle_t devHandle){

    esp_err_t ret;

    ret = spi_bus_free(masterHostId);
    ESP_ERROR_CHECK(ret);

    return ret;
}

esp_err_t spi_25LC040_read_byte(spi_device_handle_t devHandle, uint16_t address, uint8_t* pData){
    
    esp_err_t ret;
    spi_transaction_t spiTrans;

    uint8_t tx1 = ((address && 0x100) >> 5) || 0x03;
    uint8_t tx2 = address && 0xFF;
    uint8_t txData[2] = {tx1, tx2};

    spiTrans.length = (2 * 8);
    spiTrans.rxlength = 8;
    spiTrans.rx_buffer = pData;
    spiTrans.tx_buffer = &txData;
    
    
    ret = spi_device_polling_transmit(devHandle, &spiTrans);
    assert (ret==ESP_OK);

    return ret;
}

esp_err_t spi_25LC040_write_byte(spi_device_handle_t devHandle, uint16_t address, uint8_t data){

    esp_err_t ret;
    spi_transaction_t spiTrans;

    uint8_t tx1 = ((address & 0x100) >> 5) | 0x02;
    uint8_t tx2 = address & 0xFF;
    uint8_t tx3 = data;
    uint8_t txData[3] = {tx1, tx2, tx3};

    spiTrans.length = 24;
    spiTrans.tx_buffer = &txData;
    
    ret = spi_device_polling_transmit(devHandle, &spiTrans);
    assert (ret==ESP_OK);

    return ret;
}

esp_err_t spi_25LC040_write_page(spi_device_handle_t devHandle, uint16_t address, const uint8_t* pBuffer, uint8_t size){

    esp_err_t ret;
    spi_transaction_t spiTrans;

    uint8_t tx1 = ((address & 0x100) >> 5) | 0x02; 
    uint8_t tx2 = address & 0xFF;               // address
    
    uint8_t txData[size+2];    
    txData[0] = tx1;
    txData[1] = tx2;

    for (int i = 0; i < 16; i++){
        txData[i+2] = pBuffer[i];           // adding the pbuff to the txData
    }

    if (size > 16){
        size = 16;
        printf("\nTOO BIG, rezing to 16\n");
    } 


    spiTrans.length = (size + 2) * 8;   
    spiTrans.tx_buffer = &txData;
    
    ret = spi_device_polling_transmit(devHandle, &spiTrans);
    assert (ret==ESP_OK);

    return ret;
}

esp_err_t spi_25LC040_write_enable(spi_device_handle_t devHandle){
    
    esp_err_t ret;
    spi_transaction_t spiTrans;

    uint8_t txData = 0x06; // 0000 0110

    spiTrans.length = 8;
    spiTrans.tx_buffer = &txData;

    ret = spi_device_polling_transmit(devHandle, &spiTrans);
    assert (ret==ESP_OK);

    return ret;
}

esp_err_t spi_25LC040_write_disable(spi_device_handle_t devHandle){

    esp_err_t ret;
    spi_transaction_t spiTrans;

    uint8_t txData = 0x04; // 0000 0100

    spiTrans.length = 8;
    spiTrans.tx_buffer = &txData;
   
    ret = spi_device_polling_transmit(devHandle, &spiTrans);
    assert (ret==ESP_OK);

    return ret;
}

esp_err_t spi_25LC040_read_status(spi_device_handle_t devHandle, uint8_t* pStatus){

    esp_err_t ret;
    spi_transaction_t spiTrans;
    uint8_t txData = 0x05; // 0000 0101

    spiTrans.length = ((2 + sizeof(*pStatus)) * 8);
    spiTrans.rxlength = 8;
    spiTrans.rx_buffer = pStatus;
    spiTrans.tx_buffer = &txData;

    ret = spi_device_polling_transmit(devHandle, &spiTrans);
    assert (ret==ESP_OK);

    return ret;
}

esp_err_t spi_25LC040_write_status(spi_device_handle_t devHandle, uint8_t status){


    esp_err_t ret;
    spi_transaction_t spiTrans;

    uint8_t txData[2] = {0x01, status}; // 0000 0001 , STATUS

    spiTrans.length = 16;
    spiTrans.tx_buffer = &txData;

   
    ret = spi_device_polling_transmit(devHandle, &spiTrans);
    assert (ret==ESP_OK);

    return ret;
}


// =================== TIMER FUNCTIONS ====================== v2 
void periodicTimer_5sec_isr_handler(TimerHandle_t xTimer_5sec){
    
    ESP_LOGI(TIMER_TAG_5sec, "Handling 5sec timer");
    sampleCount = 0;
    TimerHandle_t xTimer_100ms = (TimerHandle_t) pvTimerGetTimerID(xTimer_5sec);

    xTimerStart(xTimer_100ms, 0);
    ESP_LOGI(TIMER_TAG_100ms, "Started 100ms timer");

}

void periodicTimer_100ms_isr_handler(TimerHandle_t xTimer_100ms){
    
    ESP_LOGI(TIMER_TAG_5sec, "Handling 100ms timer");
    // Read data from slave
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();   // Create a new I2C command link

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, MPU9250_SENSOR_ADDR << 1 | I2C_MASTER_WRITE, true);  // Slave Address + Write bit
        i2c_master_write_byte(cmd, 0x4d, true);                                         // Command Byte: selects the register to be read
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, MPU9250_SENSOR_ADDR << 1 | I2C_MASTER_READ, true);   // Slave Address + Read bit
        i2c_master_read(cmd, data, 2, I2C_MASTER_ACK);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

    if(++sampleCount == 3){
        xTimerStop(xTimer_100ms, 0);
        ESP_LOGI(TIMER_TAG_100ms, "Stopped 100ms timer");
    }
    ESP_LOGI(TIMER_TAG_100ms, "Sample%d data: %d %d\n", sampleCount, data[0], data[1]);
    flagSample = !flagSample;
}

void timerInit(){
    //============== 100ms timer =================
    ESP_LOGI(TIMER_TAG_100ms, "Creating 100 mili-seconds Timer");
    TimerHandle_t xTimer_100ms = xTimerCreate("Timer_100ms", 100 / portTICK_PERIOD_MS, pdTRUE, (void *) 0, periodicTimer_100ms_isr_handler);
    if (xTimer_100ms == NULL) {
        printf("Failed to create timer 100ms.\n");
    }
    ESP_LOGI(TIMER_TAG_100ms, "Successfully created Timer_100ms");

    //============== 5sec timer =================
    ESP_LOGI(TIMER_TAG_5sec, "Creating 5 seconds Periodic Timer");
    TimerHandle_t xTimer_5sec = xTimerCreate("Timer_5sec", 5000 / portTICK_PERIOD_MS, pdTRUE, (void *) xTimer_100ms, periodicTimer_5sec_isr_handler);
    if (xTimer_5sec == NULL) {
        printf("Failed to create timer 5sec.\n");
    }
    ESP_LOGI(TIMER_TAG_5sec, "Successfully created Timer_5sec");

    xTimerStart(xTimer_5sec, 0);
    ESP_LOGI(TIMER_TAG_5sec, "Successfully started Timer_5sec");

}


//======================== MAIN ================================ 
void app_main(void){   

    spi_device_handle_t spiHandle;
    spi_25LC040_init(SPI1_HOST, PIN_SPI_SS, PIN_SPI_SCLK, PIN_SPI_MOSI, PIN_SPI_MISO, SPI_CLK_SPEED, &spiHandle);
    timerInit();
    i2c_master_init();

    printf("*** INITs DONE ***\n");

    while(1){
        if(flagSample == true){
            spi_25LC040_write_byte(spiHandle, 0x0000, data[0]); 
            spi_25LC040_write_byte(spiHandle, 0x0000, data[1]);
            flagSample = false;
        }        

    }
    
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
}
    
// =================== TIMER FUNCTIONS ====================== v1
/* // void
static bool IRAM_ATTR periodicTimer_5sec_isr_handler(void *arg){
    ESP_LOGI(TIMER_TAG, "FUNCTION OK!!!\n");
   // BaseType_t high_task_awoken = pdFALSE;

    ESP_LOGI(TIMER_TAG, "FUNCTION OK!!!\n");
    printf("PRINT SOMETHING PERIODICALLY!\n");

   // return (high_task_awoken == pdTRUE);
}

void periodicTimerInit(){
    ESP_LOGI(TIMER_TAG, "START Timer init\n");

    gptimer_handle_t timerHandle;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000 // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timerHandle));    // create timer
    ESP_LOGI(TIMER_TAG, "Timer created");

    gptimer_alarm_config_t alarm_config = {
        .flags.auto_reload_on_alarm = true,
        .alarm_count = 1000000 // 5s
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timerHandle, &alarm_config));

    gptimer_event_callbacks_t alarm_cb = {
        .on_alarm = periodicTimer_5sec_isr_handler
    };
   // ESP_ERROR_CHECK(gptimer(timerHandle, periodicTimer_5sec_isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL));
    //ESP_LOGI(TIMER_TAG, "Timer ISR registered");

 
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timerHandle, &alarm_cb, NULL));

    ESP_LOGI(TIMER_TAG, "Enable timer");
    ESP_ERROR_CHECK(gptimer_enable(timerHandle));
    ESP_LOGI(TIMER_TAG, "Start timer, auto-reload at alarm event");
    ESP_LOGI(TIMER_TAG, "END Timer init\n");

}
*/