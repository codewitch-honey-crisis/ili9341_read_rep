#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
// configured for the esp-wrover-kit
// this board has a hardwired integrated
// ILI9341 display
// change it for your configuration
#define HOST    HSPI_HOST
#define DMA_CHAN    2
#define PIN_NUM_MISO GPIO_NUM_25
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_19
#define PIN_NUM_CS   GPIO_NUM_22

#define PIN_NUM_DC   GPIO_NUM_21
#define PIN_NUM_RST  GPIO_NUM_18
#define PIN_NUM_BCKL GPIO_NUM_5

DRAM_ATTR static const struct init_cmd {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} 
 s_init_cmds[] = {
    /* Power contorl B, power control = 0, DC_ENA = 1 */
    {0xCF, {0x00, 0x83, 0X30}, 3},
    /* Power on sequence control,
    * cp1 keeps 1 frame, 1st frame enable
    * vcl = 0, ddvdh=3, vgh=1, vgl=2
    * DDVDH_ENH=1
    */
    {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
    /* Driver timing control A,
    * non-overlap=default +1
    * EQ=default - 1, CR=default
    * pre-charge=default - 1
    */
    {0xE8, {0x85, 0x01, 0x79}, 3},
    /* Power control A, Vcore=1.6V, DDVDH=5.6V */
    {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
    /* Pump ratio control, DDVDH=2xVCl */
    {0xF7, {0x20}, 1},
    /* Driver timing control, all=0 unit */
    {0xEA, {0x00, 0x00}, 2},
    /* Power control 1, GVDD=4.75V */
    {0xC0, {0x26}, 1},
    /* Power control 2, DDVDH=VCl*2, VGH=VCl*7, VGL=-VCl*3 */
    {0xC1, {0x11}, 1},
    /* VCOM control 1, VCOMH=4.025V, VCOML=-0.950V */
    {0xC5, {0x35, 0x3E}, 2},
    /* VCOM control 2, VCOMH=VMH-2, VCOML=VML-2 */
    {0xC7, {0xBE}, 1},
    /* Memory access contorl, MX=MY=0, MV=1, ML=0, BGR=1, MH=0 */
    {0x36, {0x28}, 1},
    /* Pixel format, 16bits/pixel for RGB/MCU interface */
    {0x3A, {0x55}, 1},
    /* Frame rate control, f=fosc, 70Hz fps */
    {0xB1, {0x00, 0x1B}, 2},
    /* Enable 3G, disabled */
    {0xF2, {0x08}, 1},
    /* Gamma set, curve 1 */
    {0x26, {0x01}, 1},
    /* Positive gamma correction */
    {0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
    /* Negative gamma correction */
    {0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
    /* Column address set, SC=0, EC=0xEF */
    {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
    /* Page address set, SP=0, EP=0x013F */
    {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
    /* Memory write */
    {0x2C, {0}, 0},
    /* Entry mode set, Low vol detect disabled, normal display */
    {0xB7, {0x07}, 1},
    /* Display function control */
    {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
    /* Sleep out */
    {0x11, {0}, 0x80},
    /* Display on */
    {0x29, {0}, 0x80},
    {0, {0}, 0xff},
};
// sends data to the SPI bus MOSI line
void send(spi_device_handle_t handle, const uint8_t* data,size_t size,bool cmd) {
    if(0<size) {
        spi_transaction_t t;
        memset(&t,0,sizeof(t));
        if(size>4) {
            t.tx_buffer = data;
        } else {
            t.flags = SPI_TRANS_USE_TXDATA;
            memcpy(t.tx_data,data,size);
        }
        t.user = (void*)!cmd;
        t.length = 8*size;
        ESP_ERROR_CHECK(spi_device_polling_transmit(handle,&t));
    }
}
// retrieves data from the SPI bus MISO line
// this doesn't seem to return meaningful data currently
size_t retr(spi_device_handle_t handle, uint8_t* data,size_t size,bool cmd) {
    if(0<size) {
        spi_transaction_t t;
        memset(&t,0,sizeof(t));
        // always use rx_buffer
        t.rx_buffer = data;
        t.length = t.rxlength = 8*size;
        t.user = (void*)!cmd;
        ESP_ERROR_CHECK(spi_device_polling_transmit(handle,&t));
        return (t.rxlength+7)/8;
    }
    return 0;
}
// sends all the initalization commands to the display
void init_display(spi_device_handle_t handle) {
    static const TickType_t ts = 100/portTICK_RATE_MS;
    int cmd=0;

    //Initialize non-SPI GPIOs
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

    //Reset the display
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(ts);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(ts);
    //Send all the commands
    while (s_init_cmds[cmd].databytes!=0xff) {
        send(handle,&s_init_cmds[cmd].cmd,1,true);
        send(handle,s_init_cmds[cmd].data,s_init_cmds[cmd].databytes&0x1F,false);
        if (s_init_cmds[cmd].databytes&0x80) {
            vTaskDelay(ts);
        }
        ++cmd;
    }
    //Enable backlight
    gpio_set_level(PIN_NUM_BCKL, 0);
    
}
// sets the DC line based on transaction_t.user being 0 or 1
void pre_trans_callback(spi_transaction_t* t) {
    int dc=(int)t->user;
    gpio_set_level(PIN_NUM_DC, dc!=0);
}
// sets the address window for reads or writes (see datasheet)
void set_window(spi_device_handle_t handle, uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2) {
    uint8_t tx_data[4];
    
    //Column Address Set
    uint8_t cmd = 0x2A;
    send(handle,&cmd,1,true);
    tx_data[0]=x1>>8;             //Start Col High
    tx_data[1]=x1&0xFF;           //Start Col Low
    tx_data[2]=x2>>8;             //End Col High
    tx_data[3]=x2&0xff;           //End Col Low
    send(handle,tx_data,4,false);

    //Page Address Set
    cmd = 0x2B;
    send(handle,&cmd,1,true);
    tx_data[0]=y1>>8;             //Start Page High
    tx_data[1]=y1&0xFF;           //Start Page Low
    tx_data[2]=y2>>8;             //End Page High
    tx_data[3]=y2&0xff;           //End Page Low
    send(handle,tx_data,4,false);
}
// prints the data as a string of hex
void print_hex(const uint8_t* data,size_t size) {
    for(size_t i = 0;i<size;++i) {
        printf("%02X",(int)*data++);
    }
    
}
// entry point
void app_main() {
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=GPIO_NUM_NC,
        .quadhd_io_num=GPIO_NUM_NC,
        .max_transfer_sz=8192+8,
        .flags=0,
        .intr_flags = 0
    };
    spi_device_interface_config_t devcfg={
        .command_bits=0,
        .address_bits=0,
        .dummy_bits=0,
        .mode=0,
        .duty_cycle_pos=0,
        .cs_ena_pretrans=0,
        .cs_ena_posttrans=0,
        .clock_speed_hz=10*1000*1000,//Clock out at 10 MHz
        .input_delay_ns = 0,
        .spics_io_num=(int)PIN_NUM_CS,
        .flags =0,
        .queue_size=7, //We want to be able to queue 7 transactions at a time
        .pre_cb=pre_trans_callback, //Specify pre-transfer callback to handle D/C line
        .post_cb=NULL
    };
    ESP_ERROR_CHECK(spi_bus_initialize(HOST, &buscfg,DMA_CHAN));
    spi_device_handle_t handle;
    ESP_ERROR_CHECK(spi_bus_add_device(HOST,&devcfg,&handle));
    init_display(handle);
    uint16_t fill[320];
    memset(fill,0xFF,320*2);
    set_window(handle,0,0,319,239);
    uint8_t cmd = 0x2C; // memory write
    send(handle,&cmd,1,true);
    for(int y=0;y<240;++y) {
        send(handle,(uint8_t*)fill,320*2,false);
    }
    set_window(handle,0,0,319,239);
    cmd = 0x2E; // memory read
    send(handle,&cmd,1,true);
    // dummy read (see datasheet)
    retr(handle,&cmd,1,false);
    printf("dummy value: 0x");
    print_hex(&cmd,1);
    printf("\r\n");
    // getting zeroes here, but expect FFFFFF...
    for(int y=0;y<240;++y) {
        size_t sz = retr(handle,(uint8_t*)fill,320*2,false);
        print_hex((uint8_t*)fill,sz);
        printf("\r\n");
        vTaskDelay(1);
    }
    
}