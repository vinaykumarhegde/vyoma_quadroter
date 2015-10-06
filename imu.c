#include "imu.h"
#include "filter.h"
#include "cross_studio_io.h"


S32 itgConfig(void)
{
    S8 ReturnCode = 0;
    unsigned char bufferTx1[]= {ITG_ADDR,SMPLRT_DIV,SMPLRT_VAL,DLPF_VAL};
    ReturnCode = i2c_master_write(I2C_0,bufferTx1,sizeof(bufferTx1));      // Put ITG3200 in Measure mode
    if(ReturnCode == ERROR)
        return -1;
    return 0;
}

S32 itgRegRead(S8 *Val)
{
    // Variable declaration goes here.
    U8 bufferTx[] = {ITG_ADDR,GYRO_XOUT_H};
    U8 bufferRx[] = {ITG_RDADR,0,0,0,0,0,0};
    S8 ReturnCode=0;
    ReturnCode = i2c_master_write(I2C_0,bufferTx,sizeof(bufferTx));
    if(ReturnCode == ERROR)
        return -1;
    ReturnCode = i2c_master_read(I2C_0,bufferRx,sizeof(bufferRx));             // Read 6 continuous bytes.
    if(ReturnCode == ERROR)
        return -1;
    for(int i=0;i<6;i++)
        Val[i] = bufferRx[i+1];
    return 0;

 return 0;
}

S8 itgRead(angular_rate *val)
{
    U8 ReturnCode = 0;
    S8 bufferRx[6];
    ReturnCode = itgRegRead(bufferRx);
    if(ReturnCode == -1)
        return -1;     // Indicate the error.

    val->pitch_rate = (bufferRx[0]<<8)|bufferRx[1];
    val->pitch_rate/=14.375;
    
    val->roll_rate = (bufferRx[2]<<8)|bufferRx[3];
    val->roll_rate /=14.375;
    if(0)
    {
     debug_printf("Roll_rate=%f\t",val->roll_rate);
     debug_printf("Pitch_rate=%f\t",val->pitch_rate);
    }
    return 0;
}


/*****************************************************
Accelerometer part. Please refer to individual function 
details for further details.

Written and optimized for Project Vyoma- Quadcopter research.
visit us at: http://www.projectvyoma.com 
*****************************************************/
// ADXL 345 configuration. Please see imu.h for settings.

S8 adxl345Config()
{
    S8 ReturnCode=0;
    U8 bufferTx1[] = {ADXL_ADDR,POWER_CTL,MEASURE};           // Buffer data to configure ADXL in Measure mode.
    U8 bufferTx2[] = {ADXL_ADDR,DATA_FORMAT, 0};        // Buffer data to configure ADXL in +-8g mode.
    i2c_init(I2C_0);                                          // Initialize I2C_0 for communication.
    ReturnCode = i2c_master_write(I2C_0,bufferTx1,sizeof(bufferTx1));      // Put ADXL in Measure mode
    if(ReturnCode == ERROR)
        return -1;                                            // Return message if there is a failure
    ReturnCode = i2c_master_write(I2C_0,bufferTx2,sizeof(bufferTx2));      // ADXL works in +/- 8g mode.
    if(ReturnCode == ERROR)                                   // Return message if there is a failure
        return -1;
    return 0;
}

S8 adxl345RegRead(U8 *Val)
{
    U8 bufferTx[] = {ADXL_ADDR,DATAX0};
    U8 bufferRx[] = {ADXL_RDADDR,0,0,0,0,0,0};
    S8 ReturnCode=0;
    ReturnCode = i2c_master_write(I2C_0,bufferTx,sizeof(bufferTx));
    if(ReturnCode == ERROR)
        return -1;
    ReturnCode = i2c_master_read(I2C_0,bufferRx,6);             // Read 6 continuous bytes.
    if(ReturnCode == ERROR)
        return -1;
    for(int i=0;i<6;i++)
        Val[i] = bufferRx[i+1];
    return 0;
}

S8 adxl345Read(S16 *val)
{
    U8 ReturnCode = 0;
    U8 bufferRx[6];
    ReturnCode = adxl345RegRead(bufferRx);
    if(ReturnCode == -1)
        return -1;                                        // Indicate the error.
    
    val[0] = (bufferRx[1]<<8)|bufferRx[0];
    if(val[0]&0x8000)
    {
     val[0]&=0x03ff;
     val[0]-=1024;
    }
    else
    val[0]&=0x03ff;
    
    val[1] = ((U16)bufferRx[3]<<8)|(U16)bufferRx[2];
    if(val[1]&0x8000)
    {
     val[1]&=0x03ff;
     val[1]-=1024;
    }
    else
    val[1]&=0x03ff;

    val[2] = ((U16)bufferRx[5]<<8)|(U16)bufferRx[4];
    if(val[2]&0x8000)
    {
     val[2]&=0x03ff;
     val[2]-=1024;
    }
    else
    val[2]&=0x03ff;
    
    return 0;
}