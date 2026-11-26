#include "lis3dsh_driver.h"
#include "main.h"

/* =========================
 * LIS3DSH Register Addresses
 * ========================= */

#define LIS3DSH_WHO_AM_I_REG        0x0FU
#define LIS3DSH_CTRL_REG4           0x20U
#define LIS3DSH_CTRL_REG5           0x24U

#define LIS3DSH_OUT_X_L             0x28U
#define LIS3DSH_OUT_X_H             0x29U
#define LIS3DSH_OUT_Y_L             0x2AU
#define LIS3DSH_OUT_Y_H             0x2BU
#define LIS3DSH_OUT_Z_L             0x2CU
#define LIS3DSH_OUT_Z_H             0x2DU

#define LIS3DSH_WHO_AM_I_VALUE      0x3FU

/* SPI command bits */
#define LIS3DSH_SPI_READ_BIT        0x80U

#define LIS3DSH_SPI_TIMEOUT_MS      100U

/* Sensor configuration
 *
 * CTRL_REG4 = 0x8F
 *
 * ODR  = 800 Hz  (bits 7:4 = 1000)
 * BDU  = enabled (bit 3)
 * X    = enabled
 * Y    = enabled
 * Z    = enabled
 *
 * The MCU samples at 800 Hz via TIM2, matching the sensor ODR.
 */
#define LIS3DSH_CTRL_REG4_VALUE 0x8FU

/*
 * CTRL_REG5 = 0x00
 *
 * Full scale = ±2 g
 * Self test  = disabled
 */
#define LIS3DSH_CTRL_REG5_VALUE     0x00U


static SPI_HandleTypeDef *lis3dsh_spi = NULL;


/* =========================
 * Private function prototypes
 * ========================= */

static void LIS3DSH_CS_Low(void);
static void LIS3DSH_CS_High(void);

static LIS3DSH_Status_t LIS3DSH_ReadRegister(uint8_t reg,
                                             uint8_t *value);

static LIS3DSH_Status_t LIS3DSH_WriteRegister(uint8_t reg,
                                              uint8_t value);


/* =========================
 * Chip Select
 * ========================= */

static void LIS3DSH_CS_Low(void)
{
    HAL_GPIO_WritePin(LIS3DSH_CS_GPIO_Port,
                      LIS3DSH_CS_Pin,
                      GPIO_PIN_RESET);
}


static void LIS3DSH_CS_High(void)
{
    HAL_GPIO_WritePin(LIS3DSH_CS_GPIO_Port,
                      LIS3DSH_CS_Pin,
                      GPIO_PIN_SET);
}


/* =========================
 * Register Read
 * ========================= */

static LIS3DSH_Status_t LIS3DSH_ReadRegister(uint8_t reg,
                                             uint8_t *value)
{
    uint8_t tx_buffer[2];
    uint8_t rx_buffer[2];

    if ((lis3dsh_spi == NULL) || (value == NULL))
    {
        return LIS3DSH_STATUS_ERROR;
    }

    tx_buffer[0] = reg | LIS3DSH_SPI_READ_BIT;
    tx_buffer[1] = 0x00U;

    rx_buffer[0] = 0x00U;
    rx_buffer[1] = 0x00U;

    LIS3DSH_CS_Low();

    if (HAL_SPI_TransmitReceive(lis3dsh_spi,
                                tx_buffer,
                                rx_buffer,
                                2U,
                                LIS3DSH_SPI_TIMEOUT_MS) != HAL_OK)
    {
        LIS3DSH_CS_High();

        return LIS3DSH_STATUS_ERROR;
    }

    LIS3DSH_CS_High();

    *value = rx_buffer[1];

    return LIS3DSH_STATUS_OK;
}


/* =========================
 * Register Write
 * ========================= */

static LIS3DSH_Status_t LIS3DSH_WriteRegister(uint8_t reg,
                                              uint8_t value)
{
    uint8_t tx_buffer[2];

    if (lis3dsh_spi == NULL)
    {
        return LIS3DSH_STATUS_ERROR;
    }

    /*
     * Write operation:
     *
     * Bit 7 = 0
     *
     * Register addresses are already below 0x80,
     * therefore no read bit is added.
     */

    tx_buffer[0] = reg;
    tx_buffer[1] = value;

    LIS3DSH_CS_Low();

    if (HAL_SPI_Transmit(lis3dsh_spi,
                         tx_buffer,
                         2U,
                         LIS3DSH_SPI_TIMEOUT_MS) != HAL_OK)
    {
        LIS3DSH_CS_High();

        return LIS3DSH_STATUS_ERROR;
    }

    LIS3DSH_CS_High();

    return LIS3DSH_STATUS_OK;
}


/* =========================
 * WHO_AM_I
 * ========================= */

LIS3DSH_Status_t LIS3DSH_ReadWhoAmI(uint8_t *device_id)
{
    return LIS3DSH_ReadRegister(LIS3DSH_WHO_AM_I_REG,
                                device_id);
}


/* =========================
 * Initialization
 * ========================= */

LIS3DSH_Status_t LIS3DSH_Init(SPI_HandleTypeDef *hspi)
{
    uint8_t device_id = 0U;

    if (hspi == NULL)
    {
        return LIS3DSH_STATUS_ERROR;
    }

    lis3dsh_spi = hspi;

    /*
     * Sensor is not selected when idle.
     */
    LIS3DSH_CS_High();

    HAL_Delay(10U);


    /*
     * Step 1:
     * Verify sensor identity.
     */
    if (LIS3DSH_ReadWhoAmI(&device_id) != LIS3DSH_STATUS_OK)
    {
        return LIS3DSH_STATUS_ERROR;
    }

    if (device_id != LIS3DSH_WHO_AM_I_VALUE)
    {
        return LIS3DSH_STATUS_INVALID_ID;
    }


    /*
     * Step 2:
     * Configure output data rate and enable axes.
     */
    if (LIS3DSH_WriteRegister(LIS3DSH_CTRL_REG4,
                              LIS3DSH_CTRL_REG4_VALUE)
            != LIS3DSH_STATUS_OK)
    {
        return LIS3DSH_STATUS_ERROR;
    }


    /*
     * Step 3:
     * Configure full-scale range.
     *
     * 0x00 => ±2 g
     */
    if (LIS3DSH_WriteRegister(LIS3DSH_CTRL_REG5,
                              LIS3DSH_CTRL_REG5_VALUE)
            != LIS3DSH_STATUS_OK)
    {
        return LIS3DSH_STATUS_ERROR;
    }


    /*
     * Wait for first valid measurement.
     *
     * At 800 Hz a new sample arrives every 1.25 ms.
     */
    HAL_Delay(20U);

    return LIS3DSH_STATUS_OK;
}


/* =========================
 * Read raw XYZ acceleration
 * ========================= */

LIS3DSH_Status_t LIS3DSH_ReadRaw(LIS3DSH_RawData_t *data)
{
    uint8_t x_low;
    uint8_t x_high;

    uint8_t y_low;
    uint8_t y_high;

    uint8_t z_low;
    uint8_t z_high;


    if (data == NULL)
    {
        return LIS3DSH_STATUS_ERROR;
    }


    /*
     * Read X axis.
     */

    if (LIS3DSH_ReadRegister(LIS3DSH_OUT_X_L,
                             &x_low) != LIS3DSH_STATUS_OK)
    {
        return LIS3DSH_STATUS_ERROR;
    }

    if (LIS3DSH_ReadRegister(LIS3DSH_OUT_X_H,
                             &x_high) != LIS3DSH_STATUS_OK)
    {
        return LIS3DSH_STATUS_ERROR;
    }


    /*
     * Read Y axis.
     */

    if (LIS3DSH_ReadRegister(LIS3DSH_OUT_Y_L,
                             &y_low) != LIS3DSH_STATUS_OK)
    {
        return LIS3DSH_STATUS_ERROR;
    }

    if (LIS3DSH_ReadRegister(LIS3DSH_OUT_Y_H,
                             &y_high) != LIS3DSH_STATUS_OK)
    {
        return LIS3DSH_STATUS_ERROR;
    }


    /*
     * Read Z axis.
     */

    if (LIS3DSH_ReadRegister(LIS3DSH_OUT_Z_L,
                             &z_low) != LIS3DSH_STATUS_OK)
    {
        return LIS3DSH_STATUS_ERROR;
    }

    if (LIS3DSH_ReadRegister(LIS3DSH_OUT_Z_H,
                             &z_high) != LIS3DSH_STATUS_OK)
    {
        return LIS3DSH_STATUS_ERROR;
    }


    /*
     * LIS3DSH axis values are signed 16-bit values.
     *
     * Example:
     *
     * HIGH = 0x12
     * LOW  = 0x34
     *
     * result = 0x1234
     */

    data->x = (int16_t)(((uint16_t)x_high << 8U) |
                         (uint16_t)x_low);

    data->y = (int16_t)(((uint16_t)y_high << 8U) |
                         (uint16_t)y_low);

    data->z = (int16_t)(((uint16_t)z_high << 8U) |
                         (uint16_t)z_low);


    return LIS3DSH_STATUS_OK;
}
