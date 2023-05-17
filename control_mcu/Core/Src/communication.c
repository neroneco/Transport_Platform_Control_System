
#include "communication.h"

extern data_packet_struct Data_Packet[2];
extern 			UART_HandleTypeDef huart2;
extern int  Previous;
extern int  RX_Status;
extern char Buffer_RX[10];


//-------------------------------------------------
//         I2C communication functions
//-------------------------------------------------

void I2C_ReadBytes( I2C_HandleTypeDef* h_i2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t* buf_in, uint8_t byte_num )
{
    if (HAL_I2C_Master_Transmit(h_i2c, dev_addr, &reg_addr, 1, 1000) != HAL_OK){
        printf("[ERROR] Read I2C transmit\n");
    }
    if (HAL_I2C_Master_Receive(h_i2c, dev_addr, buf_in, byte_num, 1000) != HAL_OK){
        printf("[ERROR] Read I2C receive\n");
    }
}

void I2C_WriteBytes( I2C_HandleTypeDef* h_i2c, uint8_t dev_addr, uint8_t* buf_out, uint8_t byte_num )
{
      if (HAL_I2C_Master_Transmit(h_i2c, dev_addr, buf_out, byte_num, 1000) != HAL_OK){
          printf("[ERROR] Write I2C transmit\n");
      }
}

uint8_t I2C_ReadReg( I2C_HandleTypeDef* h_i2c, uint8_t dev_addr, uint8_t reg)
{
    uint8_t out = reg;
    uint8_t in  = 0;

    I2C_ReadBytes(h_i2c, dev_addr, &out, &in, 1);

    return in;
}

void I2C_WriteReg( I2C_HandleTypeDef* h_i2c, uint8_t dev_addr, uint8_t reg, uint8_t value )
{
    uint8_t out[2] = {reg,value};

    I2C_WriteBytes(h_i2c, dev_addr, out, 2);
}
