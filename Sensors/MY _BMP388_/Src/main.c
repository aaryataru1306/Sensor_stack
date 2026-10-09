#include "bmp388.h"
#include "spi_driver.h"

volatile int bmp388_status = BMP388_ERROR;
volatile int bmp388_read_status = BMP388_ERROR;

volatile uint8_t bmp388_chip_id = 0U;

volatile float bmp388_temperature_c = 0.0f;
volatile float bmp388_pressure_pa = 0.0f;

int main(void)
{
    BMP388_Data sensor_data;

    SPI1_Init();

    /* Initialize the BMP388 and verify its chip ID. */
    bmp388_status = BMP388_Init();

    if (bmp388_status == BMP388_OK)
    {
        (void)BMP388_ReadChipID((uint8_t *)&bmp388_chip_id);

        while (1)
        {
            bmp388_read_status = BMP388_ReadData(&sensor_data);

            if (bmp388_read_status == BMP388_OK)
            {
                bmp388_temperature_c = sensor_data.temperature_c;
                bmp388_pressure_pa = sensor_data.pressure_pa;
            }

            /* Inspect the variables using a debugger. */
        }
    }

    /* If initialization fails, remain here for debugging. */
    while (1)
    {
        /* Inspect bmp388_status and bmp388_chip_id. */
    }
}
