#include "Gyro.h"

GyroController *GetGyroControlFlag_Singleton(void)
{
    static GyroController *volatile g_Gyro_control_flag = NULL;

    if (g_Gyro_control_flag != NULL)
        return g_Gyro_control_flag;
    else
    {
        /* 创建单例并初始化 */
        g_Gyro_control_flag = (GyroController *)malloc(sizeof(GyroController));
        Gyro_Init(g_Gyro_control_flag);
        return g_Gyro_control_flag;
    }
}

/**
 * @brief  MCU向陀螺仪外设中写入一个字节数据，同时读取一个数据
 * @param
 * @retval
 */
uint8_t Gyro_ADC_Write_Dat(uint8_t Data)
{
    uint8_t Dat_Rec;

    HAL_GPIO_WritePin(SPI1_CS_GPIO_PORT_1, SPI1_CS_1, GPIO_PIN_RESET);   
    Dat_Rec = SPI_ReadWriteByte(hspi1, Data);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_PORT_1, SPI1_CS_1, GPIO_PIN_SET);

    return Dat_Rec;
}

/**
 * @brief  写寄存器
 * @param
 * @retval
 */
void Gyro_ADC_Write_Reg(unsigned char Command, unsigned char Data)
{
    HAL_GPIO_WritePin(SPI1_CS_GPIO_PORT_1, SPI1_CS_1, GPIO_PIN_RESET);

		
    SPI_ReadWriteByte(hspi1, Command);
	  SPI_ReadWriteByte(hspi1, 0x00);
    SPI_ReadWriteByte(hspi1, Data);

    vTaskDelay(1);
    
	  HAL_GPIO_WritePin(SPI1_CS_GPIO_PORT_1, SPI1_CS_1, GPIO_PIN_SET);
  
}

/**
 * @brief  陀螺仪ADC芯片初始化，用在SPI和Gyro的硬件初始化之后
 * @param
 * @retval
 */
void Gyro_ADC_Init(void)
{
     while (IS_ADC_RDY)
        ;
    Gyro_ADC_Write_Dat(0xf0); // Offset and Gain Self-Calibration
    vTaskDelay(2);

    while (IS_ADC_RDY)
        ;
    Gyro_ADC_Write_Dat(0xfe); // Reset to Power-Up Values
    vTaskDelay(1);

    while (IS_ADC_RDY)
        ;
    Gyro_ADC_Write_Reg(0x50, 0x06); // STATUS REGISTER Bit 2 ACAL: Auto-Calibration
    vTaskDelay(1);

    while (IS_ADC_RDY)
        ;
    Gyro_ADC_Write_Reg(0x51, 0x01); // Input Multiplexer Control Register  default
    vTaskDelay(1);

    while (IS_ADC_RDY)
        ;
    Gyro_ADC_Write_Reg(0x52, 0x00); // A/D Control Register
                                    // clock Clock Out Frequency = f CLKIN (default) ;
                                    // Sensor Detect OFF (default)
                                    // PGA = 1(default)
    vTaskDelay(1);

    while (IS_ADC_RDY)
        ;
    Gyro_ADC_Write_Reg(0x53, 0xF0); // A/D Data Rate 30,000SPS (default)  Data
                                    // rates scale linearly with fCLKIN.
    vTaskDelay(1);

    while (IS_ADC_RDY)
        ;
    Gyro_ADC_Write_Reg(0x54, 0xE0); // GPIO Control Register   Reset Value = E0h
    vTaskDelay(1);

    while (IS_ADC_RDY)
        ;
    Gyro_ADC_Write_Dat(0xf0); // Offset and Gain Self-Calibration
    vTaskDelay(1);
		
}

int delta=0;
int pre_time=0;
int now_time=0;

/**
 * @brief 返回原始的角速度寄存器值,用于陀螺仪标定
 * @param
 * @retval 返回原始的角速度寄存器值,用于陀螺仪标定
 */
int32_t Read_Gyro_Rate_Reg(void)
{
    uint8_t Reg_H, Reg_HH, Reg_HHH;
    int32_t Rate_Reg_Temp;
    while (IS_ADC_RDY) 
        ;
		HAL_GPIO_WritePin(SPI1_CS_GPIO_PORT_1, SPI1_CS_1, GPIO_PIN_RESET);
    

    SPI_ReadWriteByte(hspi1, 0x01);
    osDelay(1);
    Reg_HHH = SPI_ReadWriteByte(hspi1, 0x22); // GyroWriteDat(0x22)发送无意义，在于接收
    Reg_HH = SPI_ReadWriteByte(hspi1, 0x22);
    Reg_H = SPI_ReadWriteByte(hspi1, 0x22);
    osDelay(1);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_PORT_1, SPI1_CS_1, GPIO_PIN_SET);

    Rate_Reg_Temp = (Reg_HHH << 16) + (Reg_HH << 8) + Reg_H;

    if (Reg_HHH & 0x80)
        Rate_Reg_Temp -= 0x1000000; 
    if (GYRO_IF_DIR_INVERSE)
        Rate_Reg_Temp = -Rate_Reg_Temp;

    return Rate_Reg_Temp;
}

/**
 * @brief  translate ADS reg value to tate
 * @param  void
 * @retval uint:deg/s
 */
float Read_Gyro_Rate(GyroController *Self)
{
    int32_t Gyro_Reg = Read_Gyro_Rate_Reg();
    Self->OriginRate = Gyro_Reg * ADC_COEFF;
    float Gyro_Rate = Self->OriginRate;

    if (Gyro_Rate < 0)
        Gyro_Rate = Self->GyroCoeffNegative[2] * Gyro_Rate * Gyro_Rate *
                        Gyro_Rate +
                    Self->GyroCoeffNegative[1] * Gyro_Rate * Gyro_Rate +
                    Self->GyroCoeffNegative[0] * Gyro_Rate;
    else if (Gyro_Rate > 0)
        Gyro_Rate = Self->GyroCoeffPositive[2] * Gyro_Rate * Gyro_Rate *
                        Gyro_Rate +
                    Self->GyroCoeffPositive[1] * Gyro_Rate * Gyro_Rate +
                    Self->GyroCoeffPositive[0] * Gyro_Rate;

    Gyro_Rate -= Self->Gyro.ZeroDriftRate;
    Gyro_Rate *= ADC_CORRECT_COEFF;

    return Gyro_Rate;
}

/**
 * @brief  陀螺仪正常积分启动，需要时间较长
 * @param
 * @retval
 */
void Gyro_Zero_Drift(GyroController *Self)
{
    float Zero_Drift_Rate = 0.0f;

    for (int i = 0; i < GYRO_INVALIDE_TIME; i++)
    {
			pre_time=now_time;
		  now_time=DT_US_COUNT;
		  delta=now_time-pre_time;
      Zero_Drift_Rate = Read_Gyro_Rate(Self);
    }

    Zero_Drift_Rate = 0.0f;

    /*接下来5秒积分求均值计算零漂*/
    for (int i = 0; i < GYRO_INTEGRAL_TIME; i++)
    {
        Zero_Drift_Rate += Read_Gyro_Rate(Self);
        vTaskDelay(1);
    }
    //计算零偏和此刻的温度，用于矫正温漂
    Zero_Drift_Rate /= GYRO_INTEGRAL_TIME;
    Self->Gyro.ZeroDriftRate = Zero_Drift_Rate;
}
void Gyro_Init(GyroController *Self)
{
    Self->Gyro.Angle = 0.0f;
    Self->LastAngle = 0.0f;
    Self->Gyro.dAngle = 0.0f;
    Self->Gyro.Rate = 0.0f;
    Self->Gyro.ZeroDriftRate = 0.0f;
    Self->GyroCoeffNegative[0] = 1.00149782267262000000f;//1.00320887876701000000f;  // 1.00210532936093000000f;//1.00219154201177000000f;
    Self->GyroCoeffNegative[1] = 0.00000182369443679775f;//0.00006424997938747620f;  // 0.00003796811222844810f;//0.00007270645661039410f;
    Self->GyroCoeffNegative[2] = 0.00000011004289479470f;//0.00000196653028001516f;  // 0.00000179981794081602f;//0.00000201023445480647f;
    Self->GyroCoeffPositive[0] = 1.00249086004493000000f;//1.00399417065620000000f;  // 1.00486598170788000000f;//1.00251033404716000000f;
    Self->GyroCoeffPositive[1] = 0.00000295154486185112f;//-0.00008559299350530130f; //-0.00011541488496424900f;//-0.00007309072921035660f;
    Self->GyroCoeffPositive[2] = 0.00000014401225822969f;//0.00000206866180874066f;  // 0.00000221231502608651f;//0.00000201668114984364f;

	  HAL_GPIO_WritePin(GYRO_RESET_PORT, GYRO_RESET_PIN,GPIO_PIN_RESET);
    vTaskDelay(1);
    HAL_GPIO_WritePin(GYRO_SYNC_PORT, GYRO_SYNC_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GYRO_RESET_PORT, GYRO_RESET_PIN,GPIO_PIN_SET);

    Gyro_ADC_Init();
    Gyro_Zero_Drift(Self);
}

//************角速度处理函数**************//
/**
 * @brief  读取陀螺仪角度值，对原始角速度值进行积分，角度数据写入陀螺仪结构体
 * @param
 * @retval
 */
void Gyro_Get_Data(GyroController *Self)
{
    static bool Is_First_Cal_Angle;
    static float Gyro_Pre_Rate;
    uint16_t Now_Time;
    static uint16_t Pre_Time = 0;
    uint16_t Delta_Time;
    float Gyro_Mid_Rate;

    Self->Gyro.Rate = Read_Gyro_Rate(Self);
	
    Now_Time = DT_US_COUNT; // unit : us
    Delta_Time = Now_Time - Pre_Time;
    Pre_Time = Now_Time;

    if (Is_First_Cal_Angle == true)
    {
        Gyro_Pre_Rate = Self->Gyro.Rate; 
        Is_First_Cal_Angle = false;
    }
    else
    {
        Gyro_Mid_Rate = (Self->Gyro.Rate + Gyro_Pre_Rate) * 0.5f;
        Gyro_Pre_Rate = Self->Gyro.Rate;

        if (fabs(Gyro_Mid_Rate) <= GYRO_RATE_DEAD)
            Gyro_Mid_Rate = 0.0f;

        Self->LastAngle = Self->Gyro.Angle;
        Self->Gyro.dAngle = Gyro_Mid_Rate * Delta_Time / 1000000.0f;
        Self->Gyro.Angle += Self->Gyro.dAngle;
    }
}
