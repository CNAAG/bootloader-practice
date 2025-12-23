#include <stdint.h>
#include "pmbus_i2c_slave.h"
#include "stm32g4xx.h"

#define I2C_TIMING_100K             (0x30420F13) // 100KHz
#define I2C_TIMING_400K             (0x10320309) // 400KHz
#define I2C_TIMEOUTR                (0x803F80C7) // Ttimeout=25ms Tlow:ext=8ms
#define SETTING_I2C_DEFAULT_ADDRESS (0xB0)

#define I2C1_EV_INT_PRE_PRIORITY    (5)
#define I2C1_EV_INT_SUB_PRIORITY    (0)
#define I2C1_ERROR_INT_PRE_PRIORITY (9)
#define I2C1_ERROR_INT_SUB_PRIORITY (0)

#define RX_FIFO_LEN (256U)
#define TX_FIFO_LEN (256U)

static I2C_HandleTypeDef hi2c1 = {0};

static struct {
    uint8_t  rxBuf[RX_FIFO_LEN];
    uint8_t  txBuf[TX_FIFO_LEN];
    uint16_t rxCnt;
    uint16_t txCnt;
    uint16_t txLen;              /* 本次应发长度 */
    uint8_t  cmd;                /* PMBus 命令码 */
    uint8_t  crc;                /* 实时 CRC8 值 */
    uint8_t  dir;                /* 0=写 1=读 */
    uint8_t  busy;               /* 1=交易正在进行 */
} pmbus = {0};

void I2C_Slave_Init(void)
{    
    hi2c1.Instance              = I2C1;
    hi2c1.Init.Timing           = I2C_TIMING_100K;
    hi2c1.Init.OwnAddress1      = SETTING_I2C_DEFAULT_ADDRESS;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2      = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    (void)HAL_I2C_Init(&hi2c1);

    hi2c1.Instance->TIMEOUTR &= ~(I2C_TIMEOUTR_TEXTEN | I2C_TIMEOUTR_TIMOUTEN);
    hi2c1.Instance->TIMEOUTR  = I2C_TIMEOUTR;
    hi2c1.Instance->TIMEOUTR |= (I2C_TIMEOUTR_TEXTEN | I2C_TIMEOUTR_TIMOUTEN);

    hi2c1.Instance->CR1 |= I2C_CR1_PECEN;
    __HAL_I2C_ENABLE_IT(&hi2c1,(I2C_IT_ERRI |
                                I2C_IT_STOPI |
                                I2C_IT_NACKI |
                                I2C_IT_ADDRI |
                                I2C_IT_RXI |
                                I2C_IT_TXI));
}

/* Init the low level hardware : GPIO, CLOCK, CORTEX...etc */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (hi2c->Instance == I2C1) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin         = GPIO_PIN_15; /* PA15 ------ I2C1_SCL */
        GPIO_InitStruct.Mode        = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull        = GPIO_NOPULL;
        GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate   = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin         = GPIO_PIN_7; /* PB7 ------ I2C1_SDA */
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        __HAL_RCC_I2C1_CLK_ENABLE();

        HAL_NVIC_SetPriority(I2C1_EV_IRQn, I2C1_EV_INT_PRE_PRIORITY, I2C1_EV_INT_SUB_PRIORITY);
        HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
        HAL_NVIC_SetPriority(I2C1_ER_IRQn, I2C1_ERROR_INT_PRE_PRIORITY, I2C1_ERROR_INT_SUB_PRIORITY);
        HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    }
}

/* ADDR RXNE TXIS STOPF NACKF */
void I2C1_EV_IRQHandler(void)
{
    uint32_t isr = I2C1->ISR;
    static uint8_t nAck = 0;

    if (isr & I2C_ISR_ADDR) {
        pmbus.dir   = (isr & I2C_ISR_DIR) ? 1 : 0;
        pmbus.busy  = 1;
        pmbus.rxCnt = 0;
        pmbus.txCnt = 0;
        nAck = 0;

        (void)I2C1->RXDR;            /* 先读一次，防止 RXNE 锁 */
        I2C1->ICR |= I2C_ICR_ADDRCF;
        I2C1->CR2 |= I2C_CR2_PECBYTE;
        return;
    }

    if (isr & I2C_ISR_RXNE) {
        uint8_t data = I2C1->RXDR;
        if (pmbus.rxCnt < RX_FIFO_LEN) {
            pmbus.rxBuf[pmbus.rxCnt++] = data;
            
        }
        if (pmbus.rxCnt == 1) {
            pmbus.cmd = data;
            pmbus.txLen = GetPmbusCmdDataLen(data);
        }
        return;
    }

    if (isr & I2C_ISR_TXIS) {
        if (nAck == 1) {
            
        }
        if (pmbus.txCnt < pmbus.txLen) {
            uint8_t data = pmbus.txBuf[pmbus.txCnt];
            I2C1->TXDR = data;
            pmbus.txCnt++;
        } else {
            I2C1->TXDR = I2C1->PECR;
            pmbus.txCnt++;
        }
        return;
    }

    if (isr & I2C_ISR_AF) {
        I2C1->ICR = I2C_ICR_NACKCF;
        nAck = 1;
    }

    if (isr & I2C_ISR_STOPF) {
        I2C1->ICR = I2C_ICR_STOPF;
        if (pmbus.dir == 0) {
            if (pmbus.rxCnt > 1) {


            }
        }
    }
    /* 4. 收到 STOP ---------------------------------------------------*/
    if (isr & I2C_ISR_STOPF) {
        I2C1->ICR = I2C_ICR_STOPCF;
        if (pmbus.dir == 0) {               /* 写方向：校验/执行 */
            if (pmbus.rxCnt > 1) {          /* 至少有 Command + CRC */
                uint8_t rx_crc = pmbus.rxBuf[pmbus.rxCnt - 1];
                if (rx_crc == pmbus.crc) {  /* CRC 正确 */
                    pmu_execute_cmd(pmbus.cmd,
                                    pmbus.rxBuf + 1,
                                    pmbus.rxCnt - 2);
                }
            }
        }                                   /* 读方向：CRC 已自动发出 */
        pmbus.busy = 0;                     /* 交易结束 */
        return;
    }
}

void I2C1_ER_IRQHandler(void)
{
    __HAL_I2C_DISABLE(&hi2c1);
    while (I2C1->CR1 & I2C_CR1_PE)
    {
        __HAL_I2C_DISABLE(&hi2c1);
    }
    __HAL_I2C_ENABLE(&hi2c1);
}

