#include "KCORES_CSPS.h"

#include "stdint.h"

#include "scd_inc.h"

CSPS PowerSupply_1(0x5F, 0x57);

bool pinSta = false;
uint32_t pulsePkgStartTime = 0;
uint32_t nowTime = 0;
const uint32_t pulsePkgDeltaTime = 500;

float VoltIn;
float VoltOut;
float CurrentIn;
float CurrentOut;
float PowerIn;
float PowerOut;
float Temp1;
float Temp2;
float FanSpeed;
float isON;
int16_t FanSpeedTarget = -1;
int16_t ifON = 0;

void setup()
{
    // 使能板载LED
    pinMode(13, OUTPUT);
    pinMode(A0, OUTPUT);
    digitalWrite(A0, true);
    pinMode(A1, INPUT);


    // 串口波特率115200
    Serial.begin(115200);

    // 初始化iic
    Wire.setClock(100);

    // 初始化scd
    scd_init_1();
}


void DataRefresh()
{
    VoltIn = PowerSupply_1.getInputVoltage();
    VoltOut = PowerSupply_1.getOutputVoltage();
    CurrentIn = PowerSupply_1.getInputCurrent();
    CurrentOut = PowerSupply_1.getOutputCurrent();
    PowerIn = PowerSupply_1.getInputPower();
    PowerOut = PowerSupply_1.getOutputPower();
    Temp1 = PowerSupply_1.getTemp1();
    Temp2 = PowerSupply_1.getTemp2();
    FanSpeed = PowerSupply_1.getFanRPM();

    isON = analogRead(A1) * (5.0f / 1023.0f);
    if (ifON)
    {
        digitalWrite(A0, LOW);
    }
    else
    {
        digitalWrite(A0, HIGH);
    }

    if (FanSpeedTarget >= 0)
    {
        PowerSupply_1.setFanRPM(FanSpeedTarget);
    }
}

void loop()
{
    // 获取当前时间
    nowTime = millis();

    // 间隔一定时间发送报告包, 并闪灯
    if (nowTime - pulsePkgStartTime > pulsePkgDeltaTime)
    {
        // 以pulsePkgDeltaTime间隔自动汇报全局包
        scd_1.sco_protocol_num_next = 1;

        pulsePkgStartTime = nowTime;
        pinSta = !pinSta;
        digitalWrite(13, pinSta);
    }

    if (Serial.availableForWrite() > 8)
    {
        DataRefresh();
        Serial.write((uint8_t)scd_send1Byte(&scd_1));
        Serial.write((uint8_t)scd_send1Byte(&scd_1));
        Serial.write((uint8_t)scd_send1Byte(&scd_1));
        Serial.write((uint8_t)scd_send1Byte(&scd_1));
        Serial.write((uint8_t)scd_send1Byte(&scd_1));
        Serial.write((uint8_t)scd_send1Byte(&scd_1));
        Serial.write((uint8_t)scd_send1Byte(&scd_1));
        Serial.write((uint8_t)scd_send1Byte(&scd_1));
    }

    // scd rev
    while (Serial.available() > 0)
    {
        SCD_Rev1Byte(&scd_1, Serial.read());
    }

}

/**
 * @brief 注册宏起始，尾缀标识具体实例名称
 */
#define SCD_REG_BEGIN(TYJ_SUFFIX)                                                                      \
    volatile unsigned char recBuff##TYJ_SUFFIX[2][SCD_REVBUFF_SIZE];                                   \
    volatile unsigned char printBuff##TYJ_SUFFIX[SCD_PRINTBUFF_SIZE];                                  \
    volatile unsigned char endBuff1##TYJ_SUFFIX[4] = {0x00, 0x00, 0x81, 0x7f};                         \
    volatile unsigned char endBuff2##TYJ_SUFFIX[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x82, 0x7f}; \
    volatile unsigned char endBuff3##TYJ_SUFFIX[4] = {0x00, 0x00, 0x83, 0x7f};                         \
    static const int _begin_index##TYJ_SUFFIX = __LINE__;                                              \
    const struct TYJ_UNIT_STRUCT tyj_unit_struct##TYJ_SUFFIX[] = {

 /**
  * @brief 注册宏，中间不可加入空行或注释
  * @param TYJ_NAME 需加入的参数名称
  * @param TYJ_TYPE 需加入的参数类型
  */
 #define SCD_REG_ADD(TYJ_NAME, TYJ_TYPE)     \
     {                                       \
         (void *)&(TYJ_NAME), TYJ_##TYJ_TYPE \
                                             \
     },

  /**
   * @brief 注册宏结尾，尾缀标识具体实例名称
   */
  #define SCD_REG_END(TYJ_SUFFIX)                                                            \
    }                                                                                      \
    ;                                                                                      \
    static const int _end_index##TYJ_SUFFIX = __LINE__;                                    \
    struct SCD_CTRL_STRUCT scd##TYJ_SUFFIX;                                                \
    void scd_init##TYJ_SUFFIX()                                                            \
    {                                                                                      \
        scd##TYJ_SUFFIX._unit_struct = tyj_unit_struct##TYJ_SUFFIX;                        \
        scd##TYJ_SUFFIX.structToSendTab[0] = 0;                                            \
        scd##TYJ_SUFFIX.structToSendTab[1] = 9999;                                         \
        scd##TYJ_SUFFIX._recBuff = recBuff##TYJ_SUFFIX;                                    \
        scd##TYJ_SUFFIX._printBuff = printBuff##TYJ_SUFFIX;                                \
        scd##TYJ_SUFFIX._endbyte1 = endBuff1##TYJ_SUFFIX;                                  \
        scd##TYJ_SUFFIX._endbyte2 = endBuff2##TYJ_SUFFIX;                                  \
        scd##TYJ_SUFFIX._endbyte3 = endBuff3##TYJ_SUFFIX;                                  \
        scd##TYJ_SUFFIX.structNum = _end_index##TYJ_SUFFIX - _begin_index##TYJ_SUFFIX - 1; \
        scd##TYJ_SUFFIX.bufNum = 0;                                                        \
        scd##TYJ_SUFFIX.bytesRec = 0;                                                      \
        scd##TYJ_SUFFIX.structToSend = 0;                                                  \
        scd##TYJ_SUFFIX.byteToSend = 0;                                                    \
        scd##TYJ_SUFFIX.tempBuff = 0;                                                      \
        scd##TYJ_SUFFIX.sco_protocol_num = 0;                                              \
        scd##TYJ_SUFFIX.sco_protocol_num_next = 0;                                         \
        scd##TYJ_SUFFIX.structToSend2 = 0;                                                 \
        scd##TYJ_SUFFIX.byteToSend2 = 0;                                                   \
        scd##TYJ_SUFFIX.byteToSend3 = 0;                                                   \
        scd##TYJ_SUFFIX.structToSend3 = 0;                                                 \
        scd##TYJ_SUFFIX.dumpTarget = 0;                                                    \
        scd##TYJ_SUFFIX.dumpNumCnt = 0;                                                    \
        scd##TYJ_SUFFIX.dumpPkgCnt = 0;                                                    \
        scd##TYJ_SUFFIX.ifContPkg = 0;                                                     \
        scd##TYJ_SUFFIX.isPrintBusyFlg = 0;                                                \
        scd##TYJ_SUFFIX.byteToPrint = 0;                                                   \
        scd##TYJ_SUFFIX.byteToSend4 = 0;                                                   \
    }

   /**
    * @brief 注册宏1#scd的观测内容
    */
SCD_REG_BEGIN(_1)
SCD_REG_ADD(nowTime, uint32_t)
SCD_REG_ADD(ifON, int16_t)
SCD_REG_ADD(FanSpeedTarget, int16_t)
SCD_REG_ADD(VoltIn, float)
SCD_REG_ADD(VoltOut, float)
SCD_REG_ADD(CurrentIn, float)
SCD_REG_ADD(CurrentOut, float)
SCD_REG_ADD(PowerIn, float)
SCD_REG_ADD(PowerOut, float)
SCD_REG_ADD(Temp1, float)
SCD_REG_ADD(Temp2, float)
SCD_REG_ADD(FanSpeed, float)
SCD_REG_ADD(isON, float)
SCD_REG_END(_1)
