#ifndef PROSERVOLITE_V2_H
#define PROSERVOLITE_V2_H

#ifdef WIN32
#define EXTERNFUNC extern "C" __declspec(dllexport)
#else
#define EXTERNFUNC extern "C"
#endif

#define CAN_SUCCESS                 (0) //执行成功
#define CAN_FAILED_UNKNOW           (2) //未知错误
#define CAN_FAILED_OPERATE          (3) //操作失败
#define CAN_FAILED_DEVICEDISABLED   (4) //设备不存在
#define CAN_FAILED_MAXBYTESLIMIT    (5) //发送字节超过最大值限制
#define CAN_FAILED_NORECEIVE        (9) //没有接收到数据
#define CAN_FAILED_VERTIFYWRONG     (10)//接收数据验证错误

typedef void (*SendCallFunc)(const unsigned char *data, int size, void *);
typedef void (*ReceiveCallFunc)(const unsigned char *data, int size, void *);
typedef struct _RunAllInfo
{
    float current = 0;      //电流
    float velocity = 0;     //速度
    float position = 0;     //位置
}RunAllInfo;
enum DeviceType
{
    DeviceType_USBCAN2 = 4
};
enum BaudRate
{
    BaudRate_10 = 10,
    BaudRate_20 = 20,
    BaudRate_33 = 33,
    BaudRate_40 = 40,
    BaudRate_50 = 50,
    BaudRate_66 = 66,
    BaudRate_80 = 80,
    BaudRate_83 = 83,
    BaudRate_100 = 100,
    BaudRate_125 = 125,
    BaudRate_200 = 200,
    BaudRate_250 = 250,
    BaudRate_400 = 400,
    BaudRate_500 = 500,
    BaudRate_666 = 666,
    BaudRate_800 = 800,
    BaudRate_1000 = 1000
};

//关闭can口
EXTERNFUNC void setSendCallFunction(SendCallFunc callFunc, void *sendObject);           //设置接收回调函数
EXTERNFUNC void setReceiveCallFunction(ReceiveCallFunc callFunc, void *receiveObject);  //设置接收回调函数
EXTERNFUNC int getLastError();
EXTERNFUNC bool initDLL(DeviceType devType,int devIndex,int channel,BaudRate baudrate); //打开can口
EXTERNFUNC void freeDLL();                                                              //关闭can口

EXTERNFUNC bool getHeartBeat(int id, int timeOut = 100);                                //获得心跳状态
EXTERNFUNC unsigned getSerialNumber(int id, int timeOut = 100);                         //执行器序列号
EXTERNFUNC unsigned getHardwareVersion(int id, int timeOut = 100);                      //执行器硬件版本
EXTERNFUNC unsigned getFirmwareVersion(int id, int timeOut = 100);                      //执行器固件版本号
EXTERNFUNC float getCurrent(int id, int timeOut = 100);                                 //当前电流值(mA)
EXTERNFUNC float getVelocity(int id, int timeOut = 100);                                //当前速度值(rpm)
EXTERNFUNC float getPosition(int id, int timeOut = 100);                                //当前位置值(°)

EXTERNFUNC float getTargetCurrent(int id, int timeOut = 100);                           //获得目标电流值(mA)
EXTERNFUNC bool setTargetCurrent(int id, float current, int timeOut = 100);             //设置目标电流值(mA)
EXTERNFUNC float getTargetVelocity(int id, int timeOut = 100);                          //获得目标速度值(rpm)
EXTERNFUNC bool setTargetVelocity(int id, float velocity, int timeOut = 100);           //设置目标速度值(rpm)
EXTERNFUNC float getTargetPosition(int id, int timeOut = 100);                          //获得目标位置值(°)
EXTERNFUNC bool setTargetPosition(int id, float position, int timeOut = 100);           //设置目标位置值(°)
EXTERNFUNC float getTargetAcceleration(int id, int timeOut = 100);                      //获得目标加速度值(rpm/s)
EXTERNFUNC bool setTargetAcceleration(int id, float acceleration, int timeOut = 100);   //设置目标加速度值(rpm/s)
EXTERNFUNC float getTargetDeceleration(int id, int timeOut = 100);                      //获得目标减速度值(rpm/s)
EXTERNFUNC bool setTargetDeceleration(int id, float deceleration, int timeOut = 100);   //设置目标减速度值(rpm/s)
//EXTERNFUNC float getAccelerationTime(int id, int timeOut = 100);                        //获得加速度时间
//EXTERNFUNC bool setAccelerationTime(int id, float time, int timeOut = 100);             //设置加速度时间
EXTERNFUNC int getMode(int id, int timeOut = 100);                                      //获得控制模式
EXTERNFUNC bool setMode(int id, int mode, int timeOut = 100);                           //设置控制模式
EXTERNFUNC bool getEnabled(int id, int timeOut = 100);                                  //获得使能状态
EXTERNFUNC bool setEnabled(int id, bool enable, int timeOut = 100);                     //设置使能状态
EXTERNFUNC bool getStopRunState(int id, int timeOut = 100);                             //获得停止运行状态
EXTERNFUNC bool setStopRunState(int id, bool enable, int timeOut = 100);                //设置停止运行状态
EXTERNFUNC int getAlert(int id, int timeOut = 100);                                     //执行器告警
EXTERNFUNC float getElectronicGearRatio(int id, int timeOut = 100);                     //获得电子齿轮比
EXTERNFUNC float getVoltage(int id, int timeOut = 100);                                 //获得母线电压值
EXTERNFUNC float getProtectedVoltage(int id, int timeOut = 100);                        //获得保护电压
EXTERNFUNC bool setProtectedVoltage(int id, float voltage, int timeOut = 100);          //设置保护电压
EXTERNFUNC float getTemperature(int id, int timeOut = 100);                             //执行器当前温度
EXTERNFUNC float getProtectedTemperature(int id, int timeOut = 100);                    //执行器保护温度
EXTERNFUNC bool setProtectedTemperature(int id, float temperature, int timeOut = 100);  //执行器保护温度
EXTERNFUNC float getRecoveryTemperature(int id, int timeOut = 100);                     //执行器恢复温度
EXTERNFUNC bool setRecoveryTemperature(int id, float temperature, int timeOut = 100);   //执行器恢复温度

//EXTERNFUNC bool getCurrentLoopFilterState(int id, int timeOut = 100);                   //获得电流环滤波器状态
//EXTERNFUNC bool getVelocityLoopFilterState(int id, int timeOut = 100);                  //获得速度环滤波器状态
//EXTERNFUNC bool getPositionLoopFilterState(int id, int timeOut = 100);                  //获得位置环滤波器状态
EXTERNFUNC unsigned getPOfCurrentLoop(int id, int timeOut = 100);                          //获得电流环的p值
EXTERNFUNC bool setPOfCurrentLoop(int id, unsigned p, int timeOut = 100);                  //设置电流环的p值
EXTERNFUNC unsigned getIOfCurrentLoop(int id, int timeOut = 100);                          //获得电流环的I值
EXTERNFUNC bool setIOfCurrentLoop(int id, unsigned i, int timeOut = 100);                  //设置电流环的I值
EXTERNFUNC unsigned getPOfVelocityLoop(int id, int timeOut = 100);                         //获得速度环的p值
EXTERNFUNC bool setPOfVelocityLoop(int id , unsigned p, int timeOut = 100);                //设置速度环的p值
EXTERNFUNC unsigned getIOfVelocityLoop(int id, int timeOut = 100);                         //获得速度环的I值
EXTERNFUNC bool setIOfVelocityLoop(int id, unsigned i, int timeOut = 100);                 //设置速度环的I值
EXTERNFUNC unsigned getPOfPositionLoop(int id, int timeOut = 100);                         //获得位置环的p值
EXTERNFUNC bool setPOfPositionLoop(int id, unsigned p, int timeOut = 100);                 //设置位置环的p值
EXTERNFUNC unsigned getIOfPositionLoop(int id, int timeOut = 100);                         //获得位置环的I值
EXTERNFUNC bool setIOfPositionLoop(int id, unsigned i, int timeOut = 100);                 //设置位置环的I值

EXTERNFUNC float getIntegralLimit(int id, int timeOut = 100);                           //获得力矩环积分限制
EXTERNFUNC bool setIntegralLimit(int id, float value, int timeOut = 100);               //设置力矩环积分限制
//EXTERNFUNC float getMinOutputOfCurrentLoop(int id, int timeOut = 100);                //获得电流环的最小输出
//EXTERNFUNC bool setMinOutputOfCurrentLoop(int id, float value, int timeOut = 100);    //设置电流环的最小输出
//EXTERNFUNC float getMaxOutputOfCurrentLoop(int id, int timeOut = 100);                //获得电流环的最大输出
//EXTERNFUNC bool setMaxOutputOfCurrentLoop(int id, float value, int timeOut = 100);    //设置电流环的最大输出
//EXTERNFUNC float getMinOutputOfVelocityLoop(int id, int timeOut = 100);               //获得速度环的最小输出
//EXTERNFUNC bool setMinOutputOfVelocityLoop(int id, float value, int timeOut = 100);   //设置速度环的最小输出
//EXTERNFUNC float getMaxOutputOfVelocityLoop(int id, int timeOut = 100);               //获得速度环的最大输出
//EXTERNFUNC bool setMaxOutputOfVelocityLoop(int id, float value, int timeOut = 100);   //设置速度环的最大输出
//EXTERNFUNC float getMinOutputOfPositionLoop(int id, int timeOut = 100);               //获得位置环的最小输出
//EXTERNFUNC float setMinOutputOfPositionLoop(int id, float value, int timeOut = 100);  //设置位置环的最小输出
//EXTERNFUNC float getMaxOutputOfPositionLoop(int id, int timeOut = 100);               //获得位置环的最大输出
//EXTERNFUNC bool setMaxOutputOfPositionLoop(int id, float value, int timeOut = 100);   //设置位置环的最大输出

EXTERNFUNC float getMaxCurrent(int id, int timeOut = 100);                              //获得最大电流限制值
EXTERNFUNC bool setMaxCurrent(int id, float current, int timeOut = 100);                //设置最大电流限制值
EXTERNFUNC float getMaxVelocity(int id, int timeOut = 100);                             //获得最大速度值
EXTERNFUNC bool setMaxVelocity(int id, float velocity, int timeOut = 100);              //设置最大速度值

EXTERNFUNC float getMaxVelocityOfVelocityLadderCurve(int id, int timeOut = 100);                            //速度梯形曲线的最大速度
EXTERNFUNC bool setMaxVelocityOfVelocityLadderCurve(int id, float velocity, int timeOut = 100);             //速度梯形曲线的最大速度
EXTERNFUNC float getMaxAccelerationOfVelocityLadderCurve(int id, int timeOut = 100);                        //速度梯形曲线的最大加速度
EXTERNFUNC bool setMaxAccelerationOfVelocityLadderCurve(int id, float acceleration, int timeOut = 100);     //速度梯形曲线的最大加速度
EXTERNFUNC float getMaxDecelerationOfVelocityLadderCurve(int id, int timeOut = 100);                        //速度梯形曲线的最大减速度
EXTERNFUNC bool setMaxDecelerationOfVelocityLadderCurve(int id, float deceleration, int timeOut = 100);     //速度梯形曲线的最大减速度
EXTERNFUNC float getMaxVelocityOfPositionLadderCurve(int id, int timeOut = 100);                            //位置梯形曲线的最大速度
EXTERNFUNC bool setMaxVelocityOfPositionLadderCurve(int id, float velocity, int timeOut = 100);              //位置梯形曲线的最大速度(1：成功，5：成功，但是超过总速度限制4：超过速度限制值导致失败，0：失败)
EXTERNFUNC float getMaxAccelerationOfPositionLadderCurve(int id, int timeOut = 100);                        //位置梯形曲线的加速度
EXTERNFUNC bool setMaxAccelerationOfPositionLadderCurve(int id, float acceleration, int timeOut = 100);     //位置梯形曲线的加速度
EXTERNFUNC float getMaxDecelerationOfPositionLadderCurve(int id, int timeOut = 100);                        //位置梯形曲线的减速度
EXTERNFUNC bool setMaxDecelerationOfPositionLadderCurve(int id, float deceleration, int timeOut = 100);     //位置梯形曲线的减速度
EXTERNFUNC bool getLimitState(int id, int timeOut = 100);                               //位置限位状态
EXTERNFUNC bool setLimitState(int id, bool state, int timeOut = 100);                   //位置限位状态
EXTERNFUNC float getMaxPosition(int id, int timeOut = 100);                             //执行器位置的上限
EXTERNFUNC bool setMaxPosition(int id, float position, int timeOut = 100);              //执行器位置的上限
EXTERNFUNC float getMinPosition(int id, int timeOut = 100);                             //执行器位置的下限
EXTERNFUNC bool setMinPosition(int id, float position, int timeOut = 100);              //执行器位置的下限
EXTERNFUNC float getOriginPositionOffSet(int id, int timeOut = 100);                    //执行器位置的偏置没有Q24转换
EXTERNFUNC float getPositionOffSet(int id, int timeOut = 100);                          //执行器位置的偏置
EXTERNFUNC bool setPositionOffSet(int id, float position, int timeOut = 100);           //执行器位置的偏置
EXTERNFUNC int getSingleLapPositionRange(int id, int timeOut = 100);                    //上电时刻单圈位置范围值
EXTERNFUNC bool setSingleLapPositionRange(int id, int range, int timeOut = 100);        //上电时刻单圈位置范围值
//EXTERNFUNC float getZeroCurrent(int id, int timeOut = 100);                             //归零时电流
//EXTERNFUNC bool setZeroCurrent(int id, float current, int timeOut = 100);               //归零时电流
//EXTERNFUNC float getZeroVelocity(int id, int timeOut = 100);                            //归零时速度
//EXTERNFUNC bool setZeroVelocity(int id, float velocity, int timeOut = 100);             //归零时速度
//EXTERNFUNC float getLockedRotorEnergy(int id, int timeOut = 100);                       //堵转能量
//EXTERNFUNC bool setLockedRotorEnergy(int id, float value, int timeOut = 100);           //堵转能量
EXTERNFUNC int getCanBaudrate(int id, int timeOut = 100);                               //获得波特率
EXTERNFUNC bool setCanBaudrate(int id, int baudrate, int timeOut = 100);                //设置波特率
EXTERNFUNC bool setID(int id, int newId, int timeOut = 100);                            //执行器ID
EXTERNFUNC bool saveParas(int id, int timeOut = 100);                                   //存储参数
//EXTERNFUNC bool clearAlert(int id, int timeOut = 100);                                //清除下位机警告

//EXTERNFUNC bool getEncoderCalibrationState(int id, int timeOut = 100);
//EXTERNFUNC bool setEncoderCalibrationState(int id, bool enable, int timeOut = 100);
//EXTERNFUNC vReg_Debug = 0x70;
//EXTERNFUNC int getPoleLogarithmOfMotor(int id, int timeOut = 100);      //电极极对数
//EXTERNFUNC float getPositionOfAlignedAngle(int id, int timeOut = 100);  //机械角度电角度对齐时位置值
//EXTERNFUNC bool setPositionOfAlignedAngle(int id, float position, int timeOut = 100);
//EXTERNFUNC int getCurrentDegree(int id, int timeOut = 100);             //电角度
//EXTERNFUNC int getPhaseCurrentA(int id, int timeOut = 100);             //相电流A
//EXTERNFUNC int getPhaseCurrentB(int id, int timeOut = 100);             //相电流B
//EXTERNFUNC int getPhaseCurrentC(int id, int timeOut = 100);             //相电流C
//EXTERNFUNC int getFPGAErrorCount(int id, int timeOut = 100);            //相电流C

//EXTERNFUNC float getDegreeOfAlignedAngle(int id, int timeOut = 100);
//EXTERNFUNC bool startAlignedAngle(int id, int timeOut);
//EXTERNFUNC bool getMotorPhaseLinePolarity(int id, int timeOut);
//EXTERNFUNC bool setMotorPhaseLinePolarity(int id, float value,int timeOut);

EXTERNFUNC bool writeData(unsigned int id, unsigned char *sendData, int sendLength, unsigned char *receiveData,int *receiveLength, int timeOut = 100); //最多发送8字节，最多接收8字节

#endif // PROSERVOLITE_V2_H
