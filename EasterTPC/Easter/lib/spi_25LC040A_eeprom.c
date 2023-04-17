#include "spi_25LC040A_eeprom.h"

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

