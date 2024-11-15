#include "MAX30102.h"

uint8_t MAX30102_Initialise (MAX30102 *dev, I2C_HandleTypeDef *i2cHandle){

    /* Set struct paramters*/
    dev->i2cHandle = i2cHandle;
    dev->_interrupt_flag = 0;

    memset(dev->_ir_samples, 0);
    memset(dev->_red_samples, 0);

    /* Store number of transcation errors (to be returned at the end of function) */
    // uint8_t errNum = 0;
    // HAL_StatusTypeDef status;
    // If I can add part/dev/mm check I can use errnum variabel to see if there's a sucessfull init

    /* Check PART ID*/

    // uint8_t regData;

    // status = MAX30102_ReadRegister(dev, MAX30102_PART_ID, &regData);
    // errNum += (status != HAL_OK);
    //if (regData != MAX30102_)

    // Not finding relevant data in the datasheet makes this hard..
}

MAX30102_WriteRegister(MAX30102 *dev uint){

}

/*
 * LOW-LEVEL FUNCTIONS
 */

