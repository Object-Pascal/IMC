/* I2C MCP23017_GPIO blinking led

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "esp_peripherals.h"
#include "periph_wifi.h"
#include "board.h"
#include "mcp23017.h"

static const char* TAG = "I2C_MCP23017";

mcp23017_t mcp23017;
SemaphoreHandle_t xMutex;

void mcp23017_task(void* pvParameters)
{
    bool on = false;
    while (1) {

	   xSemaphoreTake( xMutex, portMAX_DELAY );
	   //mcp23017_write_register(&mcp23017, MCP23017_GPIO, GPIOA, on ? 0xAA : 0x55);
	   mcp23017_write_register(&mcp23017, MCP23017_GPIO, GPIOA, on ? 0x00 : 0xFF);
	   xSemaphoreGive( xMutex );
	   on = !on;
	   
       vTaskDelay(500 / portTICK_RATE_MS);

    }
    vTaskDelete(NULL);
}

void mcp23017_task_read(void* pvParameters)
{
    while (1) {

	   vTaskDelay(5100 / portTICK_RATE_MS);
	   // Check states on input
	   uint8_t states = 0x00;
	   xSemaphoreTake( xMutex, portMAX_DELAY );
	   mcp23017_read_register(&mcp23017, MCP23017_GPIO, GPIOB, &states);
	   xSemaphoreGive( xMutex );
	   
	   ESP_LOGI(TAG, "GPIO register B states: %d", states);
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
	
	// Initialize I2C bus
	mcp23017.i2c_addr = 0x20;
	mcp23017.sda_pin = 18;
	mcp23017.scl_pin = 23;
	mcp23017.sda_pullup_en = GPIO_PULLUP_ENABLE;
	mcp23017.scl_pullup_en = GPIO_PULLUP_ENABLE;
	ESP_ERROR_CHECK(mcp23017_init(&mcp23017));
	
	xMutex = xSemaphoreCreateMutex();
	
	// Set GPIO Direction
	mcp23017_write_register(&mcp23017, MCP23017_IODIR, GPIOA, 0x00); // full port on OUTPUT
	mcp23017_write_register(&mcp23017, MCP23017_IODIR, GPIOB, 0xFF); // full port on INPUT
	mcp23017_write_register(&mcp23017, MCP23017_GPPU, GPIOB, 0xFF); // full port on INPUT
	
	
	// Create blinking led task
	xTaskCreate(mcp23017_task, "mcp23017_task", 1024 * 2, NULL, 10,
            NULL);
	xTaskCreate(mcp23017_task_read, "mcp23017_task_read", 1024 * 2, NULL, 10,
            NULL);

    while (1) {
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}