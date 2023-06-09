#include <iostream>
#include "proservoLite_V2.h"

using namespace std;

int main()
{
    int id = 1;
    initDLL(DeviceType_USBCAN2,0,0,BaudRate_1000);
    cout<<"心跳状态:"<<getHeartBeat(id)<<endl;
    cout<<"序列号:"<<getSerialNumber(id)<<endl;
    cout<<"硬件版本:"<<getHardwareVersion(id)<<endl;
    cout<<"固件版本:"<<getFirmwareVersion(id)<<endl;
    cout<<"当前电流:"<<getCurrent(id)<<endl;
    cout<<"当前速度:"<<getVelocity(id)<<endl;
    cout<<"当前位置:"<<getPosition(id)<<endl;
    cout<<"使能状态:"<<getEnabled(id)<<endl;
    setEnabled(id,true);
    //    轮廓位置模式
    {
        setMode(id,1);
        setTargetVelocity(id,10);
        setTargetCurrent(id,2000);
        setTargetAcceleration(id,1000);
        setTargetDeceleration(id,1000);
        setTargetPosition(id,180);
    }

    //    位置模式
    //    {
    //        setMode(id,5);
    //        setTargetCurrent(id,2000);
    //        setTargetPosition(id,180);
    //    }

    //    速度模式
    //    {
    //        setMode(id,3);
    //        setTargetVelocity(id,10);
    //    }

    //    电流模式
    //    {
    //        setMode(id,4);
    //        setTargetCurrent(id,1000);
    //    }

    freeDLL();
    // system("pause");
    return 0;
}