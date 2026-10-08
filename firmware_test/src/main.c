#include "stm32f411xe.h"
#include "spi_driver.h"
#include "icm20948.h"

int main(void)
{
    SPI1_Init();

    ICM20948_Status_t status =
        ICM20948_Init(ICM20948_ACCEL_FS_2G,
                      ICM20948_GYRO_FS_250DPS);

    (void)status;

    while (1)
    {
    }
}