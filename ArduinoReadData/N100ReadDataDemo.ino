////////N100惯导模块接线说明///////////
//  TX1 --------------  0
//  GND ------------- GND
//  5V --------------- 5V
// N100惯导模块 --------  ArduinoUNO主板引脚


//N100惯导数据格式宏定义
#define IMU_LEN 0x38  //IMU数据长度
#define AHRS_LEN 0x30 //AHRS数据长度
#define FRAME_HEAD 0xFC //数据帧头
#define FRAME_END 0xFD //数据帧尾
#define TYPE_IMU 0x40 //IMU数据类别
#define TYPE_AHRS 0x41 //AHRS数据类别
#define IMU_TYPE_LEN 64 //当数据类型为IMU时，该数组的总长度
#define AHRS_TYPE_LEN 56 //当数据类型为AHRS时，该数组的总长度

//读取数据需要用到的相关变量
uint8_t IMU_Data[64]; //IMU数据类型长度
uint8_t AHRS_Data[56]; //AHRS数据类型长度
uint8_t Fd_data[64];  //用于存放接收串口数据
bool Data_of_IMU = 0;  //用于表示IMU数据已经就绪，可以进行解包
bool Data_of_AHRS = 0; //用于表示AHRS数据已经就绪，可以进行解包
bool ledstate;

//创建IMU数据类型的类
class IMUPacket{
 public:
    float gyroscope_x;          //unit: rad/s
    float gyroscope_y;          //unit: rad/s
    float gyroscope_z;          //unit: rad/s
    float accelerometer_x;      //m/s^2
    float accelerometer_y;      //m/s^2
    float accelerometer_z;      //m/s^2
    float magnetometer_x;       //mG
    float magnetometer_y;       //mG
    float magnetometer_z;       //mG
    float imu_temperature;      //C
    float Pressure;             //Pa
    float pressure_temperature; //C
    uint32_t Timestamp;         //us
};

//创建AHRS数据类型的类
class AHRSPacket{
 public:
  float RollSpeed;   //unit: rad/s
  float PitchSpeed;  //unit: rad/s
  float HeadingSpeed;//unit: rad/s
  float Roll;        //unit: rad
  float Pitch;       //unit: rad
  float Heading;     //unit: rad
  float Qw;//w          //Quaternion
  float Qx;//x
  float Qy;//y
  float Qz;//z
  uint32_t Timestamp; //unit: us
};

//class IMUPacket_HEX{
// public:
//    u8 gyroscope_x;          //unit: rad/s
//    u8 gyroscope_y;          //unit: rad/s
//    u8 gyroscope_z;          //unit: rad/s
//    u8 accelerometer_x;      //m/s^2
//    u8 accelerometer_y;      //m/s^2
//    u8 accelerometer_z;      //m/s^2
//    u8 magnetometer_x;       //mG
//    u8 magnetometer_y;       //mG
//    u8 magnetometer_z;       //mG
//    u8 imu_temperature;      //C
//    u8 Pressure;             //Pa
//    u8 pressure_temperature; //C
//    u8 Timestamp;         //us
//};

IMUPacket PacketIMU;   //实例化IMU数据类型对象
AHRSPacket PacketAHRS; //实例化AHRS数据类型对象

//读取N100惯导模块数据函数
void Read_N100Data(void)
{
  static uint8_t Count=0; //用于计算当前获取到的数据量
  uint8_t Usart_Receive;  //用于读取串口接收到的数据
  static uint8_t Last_Receive; //用于保存上一次的接收
  static uint8_t count=0;
  if(Serial.available()) //串口接收到数据
  {
    Usart_Receive = Serial.read(); //读取串口的数据
    Fd_data[Count] = Usart_Receive; //将接收到的数据填入数据

    //接收到上一帧的帧尾和本帧帧头则开始计数（较大程度避免数据误读浪费时间，造成数据丢包）
    if(Last_Receive==FRAME_END&&Usart_Receive==FRAME_HEAD||Count>0)
      Count++;
    else
      Count=0;

    Last_Receive = Usart_Receive; //保存本次数据
    
    //满足IMU数据长度
    if(Count==IMU_TYPE_LEN)
    {
      //数据类型、长度、帧尾均满足满足要求
      if(Fd_data[1]==TYPE_IMU&&Fd_data[2]==IMU_LEN&&Fd_data[IMU_TYPE_LEN-1]==FRAME_END)
      {
        Count = 0;//清空计数等待下次计数
        Data_of_IMU=1,memcpy(IMU_Data,Fd_data,sizeof(Fd_data));
      }
    }

    //满足AHRS数据长度要求
    if(Count==AHRS_TYPE_LEN)
    {
      if(Fd_data[1]==TYPE_AHRS&&Fd_data[2]==AHRS_LEN&&Fd_data[AHRS_TYPE_LEN-1]==FRAME_END)
      {
        Count = 0;//清空计数等待下次计数
        Data_of_AHRS=1,memcpy(AHRS_Data,Fd_data,sizeof(Fd_data));
      }
    }

    //均不满足要求，数据超出长度，放弃计数等待下次接收
    if(Count>IMU_TYPE_LEN) Count = 0;
   }
}

//数据解包函数
void DataUnpacking(void)
{
  if(Data_of_IMU==1) //IMU数据接收完毕，开始解包
  {
    if(IMU_Data[1]==TYPE_IMU&&IMU_Data[2]==IMU_LEN) //再次校验数据类型和数据长度
    {
      u8 temp[4];
      temp[0] = IMU_Data[7] , temp[1] = IMU_Data[8] , temp[2] = IMU_Data[9], temp[3] = IMU_Data[10];
      PacketIMU.gyroscope_x = HEX_to_Float_New(temp,true);

      temp[0] = IMU_Data[11] , temp[1] = IMU_Data[12] , temp[2] = IMU_Data[13], temp[3] = IMU_Data[14];
      PacketIMU.gyroscope_y = HEX_to_Float_New(temp,true);

      temp[0] = IMU_Data[15] , temp[1] = IMU_Data[16] , temp[2] = IMU_Data[17], temp[3] = IMU_Data[18];
      PacketIMU.gyroscope_z = HEX_to_Float_New(temp,true);

      temp[0] = IMU_Data[19] , temp[1] = IMU_Data[20] , temp[2] = IMU_Data[21], temp[3] = IMU_Data[22];
      PacketIMU.accelerometer_x = HEX_to_Float_New(temp,true);

      temp[0] = IMU_Data[23] , temp[1] = IMU_Data[24] , temp[2] = IMU_Data[25], temp[3] = IMU_Data[26];
      PacketIMU.accelerometer_y = HEX_to_Float_New(temp,true);

      temp[0] = IMU_Data[27] , temp[1] = IMU_Data[28] , temp[2] = IMU_Data[29], temp[3] = IMU_Data[30];
      PacketIMU.accelerometer_z = HEX_to_Float_New(temp,true);

      temp[0] = IMU_Data[31] , temp[1] = IMU_Data[32] , temp[2] = IMU_Data[33], temp[3] = IMU_Data[34];
      PacketIMU.magnetometer_x = HEX_to_Float_New(temp,true);

      temp[0] = IMU_Data[35] , temp[1] = IMU_Data[36] , temp[2] = IMU_Data[37], temp[3] = IMU_Data[38];
      PacketIMU.magnetometer_y = HEX_to_Float_New(temp,true);

      temp[0] = IMU_Data[39] , temp[1] = IMU_Data[40] , temp[2] = IMU_Data[41], temp[3] = IMU_Data[42];
      PacketIMU.magnetometer_z = HEX_to_Float_New(temp,true);

      temp[0] = IMU_Data[43] , temp[1] = IMU_Data[44] , temp[2] = IMU_Data[45], temp[3] = IMU_Data[46];
      PacketIMU.imu_temperature = HEX_to_Float_New(temp,true);

      temp[0] = IMU_Data[47] , temp[1] = IMU_Data[48] , temp[2] = IMU_Data[49], temp[3] = IMU_Data[50];
      PacketIMU.Pressure = HEX_to_Float_New(temp,true);

      temp[0] = IMU_Data[51] , temp[1] = IMU_Data[52] , temp[2] = IMU_Data[53], temp[3] = IMU_Data[54];
      PacketIMU.pressure_temperature = HEX_to_Float_New(temp,true);

      
      //下面是数据解包过程
      PacketIMU.Timestamp = timestamp(IMU_Data[58],IMU_Data[57],IMU_Data[56],IMU_Data[55]);
     
      //数据解包完毕，清空标志位等待下次数据解包
      Data_of_IMU = 0;

      //使LED闪烁提示成功接收到数据
      digitalWrite(13, HIGH);

      //使用模拟串口打印数据,默认只开通x、y、z三轴的加速度打印，需要开启其他数据选中Ctrl+/取消注释即可
//      Serial.print("gyro_x = ");
//      Serial.println(PacketIMU.gyroscope_x,7);
//      Serial.print("gyro_y = ");
//      Serial.println(PacketIMU.gyroscope_y,7);
//      Serial.print("gyro_z = ");
//      Serial.println(PacketIMU.gyroscope_z,7);
//      Serial.print("accel_X = ");
      Serial.println(PacketIMU.accelerometer_x,7);
//      Serial.print("accel_y = ");
      Serial.println(PacketIMU.accelerometer_y,7);
//      Serial.print("accel_z = ");
      Serial.println(PacketIMU.accelerometer_z,7);
//      Serial.print("mag_X = ");
//      Serial.println(PacketIMU.magnetometer_x,7);
//      Serial.print("mag_y = ");
//      Serial.println(PacketIMU.magnetometer_y,7);
//      Serial.print("mag_z = ");
//      Serial.println(PacketIMU.magnetometer_z,7);
//      Serial.print("temperature = ");
//      Serial.println(PacketIMU.imu_temperature);
//      Serial.print("Pressure = ");
//      Serial.println(PacketIMU.Pressure,7);
//      Serial.print("pressure_temperature = ");
//      Serial.println(PacketIMU.pressure_temperature,7);
//      Serial.print("Timestamp = ");
//      Serial.println(PacketIMU.Timestamp);
      Serial.println();
    }
  }

  if(Data_of_AHRS==1) //AHRS数据接收完毕，开始解包
  {
    if(AHRS_Data[1]==TYPE_AHRS&&AHRS_Data[2]==AHRS_LEN)//再次校验数据类型和数据长度
    {
      u8 temp[4];
      temp[0] = AHRS_Data[7] , temp[1] = AHRS_Data[8] , temp[2] = AHRS_Data[9], temp[3] = AHRS_Data[10];
      PacketAHRS.RollSpeed = HEX_to_Float_New(temp,true);

      temp[0] = AHRS_Data[11] , temp[1] = AHRS_Data[12] , temp[2] = AHRS_Data[13], temp[3] = AHRS_Data[14];
      PacketAHRS.PitchSpeed = HEX_to_Float_New(temp,true);

      temp[0] = AHRS_Data[15] , temp[1] = AHRS_Data[16] , temp[2] = AHRS_Data[17], temp[3] = AHRS_Data[18];
      PacketAHRS.HeadingSpeed = HEX_to_Float_New(temp,true);

      temp[0] = AHRS_Data[19] , temp[1] = AHRS_Data[20] , temp[2] = AHRS_Data[21], temp[3] = AHRS_Data[22];
      PacketAHRS.Roll = HEX_to_Float_New(temp,true);

      temp[0] = AHRS_Data[23] , temp[1] = AHRS_Data[24] , temp[2] = AHRS_Data[25], temp[3] = AHRS_Data[26];
      PacketAHRS.Pitch = HEX_to_Float_New(temp,true);

      temp[0] = AHRS_Data[27] , temp[1] = AHRS_Data[28] , temp[2] = AHRS_Data[29], temp[3] = AHRS_Data[30];
      PacketAHRS.Heading = HEX_to_Float_New(temp,true);

      temp[0] = AHRS_Data[31] , temp[1] = AHRS_Data[32] , temp[2] = AHRS_Data[33], temp[3] = AHRS_Data[34];
      PacketAHRS.Qw = HEX_to_Float_New(temp,true);

      temp[0] = AHRS_Data[35] , temp[1] = AHRS_Data[36] , temp[2] = AHRS_Data[37], temp[3] = AHRS_Data[38];
      PacketAHRS.Qx = HEX_to_Float_New(temp,true);

      temp[0] = AHRS_Data[39] , temp[1] = AHRS_Data[40] , temp[2] = AHRS_Data[41], temp[3] = AHRS_Data[42];
      PacketAHRS.Qy = HEX_to_Float_New(temp,true);

      temp[0] = AHRS_Data[43] , temp[1] = AHRS_Data[44] , temp[2] = AHRS_Data[45], temp[3] = AHRS_Data[46];
      PacketAHRS.Qz = HEX_to_Float_New(temp,true);
      PacketAHRS.Timestamp = timestamp(AHRS_Data[50],AHRS_Data[49],AHRS_Data[48],AHRS_Data[47]); //unit: us

      //数据解包完毕，清空标志位等待下次数据解包
      Data_of_AHRS = 0;

      //使LED闪烁提示成功接收到数据
      digitalWrite(13, LOW);

      //使用模拟串口打印数据,默认只开通Roll、Pitch、Heading角度打印，需要开启其他数据选中Ctrl+/取消注释即可
//      Serial.print("RollSpeed = ");
//      Serial.println(PacketAHRS.RollSpeed,7);
//      Serial.print("PitchSpeed = ");
//      Serial.println(PacketAHRS.PitchSpeed,7);
//      Serial.print("HeadingSpeed = ");
//      Serial.println(PacketAHRS.HeadingSpeed,7);
//      Serial.print("Roll = ");
      Serial.println(PacketAHRS.Roll,7);
//      Serial.print("Pitch = ");
      Serial.println(PacketAHRS.Pitch,7);
//      Serial.print("Heading = ");
      Serial.println(PacketAHRS.Heading,7);
//      Serial.print("Qw = ");
//      Serial.println(PacketAHRS.Qw,7);
//      Serial.print("Qx = ");
//      Serial.println(PacketAHRS.Qx,7);
//      Serial.print("Qy = ");
//      Serial.println(PacketAHRS.Qy,7);
//      Serial.print("Qz = ");
//      Serial.println(PacketAHRS.Qz,7);
//      Serial.print("Timestamp = ");
//      Serial.println(PacketAHRS.Timestamp);
//      Serial.println();
    }
  }
}

//16进制转浮点数实现函数
//计算机组成原理知识
float HEX_to_Float(uint8_t data1,uint8_t data2,uint8_t data3,uint8_t data4)
{
  //数据由高位到地位排序 ： data1 data2 data3 data4
  //其中接收数据时，高位存放在后面，低位存放在前面
  
  short sign; //符号计算
  unsigned long temp; //32位16进制数
  uint16_t zhishu; //存放指数
  float weishu;    //存放尾数
  float ref;       //存放计算结果
  uint16_t H_16,L_16; //存放高16位、低16位

  H_16 = data1<<8 | data2;//数据第一次融合
  L_16 = data3<<8 | data4;

  //将融合成16位的数据组合成32位数据
  temp = (unsigned long)H_16<<16|(unsigned long)L_16;

  //浮点数转化开始
  //先确定符号
  sign = (temp & 0x80000000) ? -1 : 1;  //最高位是1就是负数，0就是正数
  //计算指数
  // 127是偏移量，使得指数有正负（指数的范围是 -127 ~ +128 ）
  zhishu = ((temp >> 23) & 0xff) - 127;
  
  //获取尾数部分 将（temp & 0x7fffff）获得的值将小数点左移23位 (除以2的23次方)
  weishu = 1 + ((float)(temp & 0x7fffff) / 0x800000);
  
  //最终的公式
  ref = sign * weishu * pow(2, zhishu);
  return ref;
}

float HEX_to_Float_New(u8 *data,bool mode)
{
  float fa=0;
  u8 uc[4];

  if(mode==false)
  {
    uc[3] = data[0];
    uc[2] = data[1];
    uc[1] = data[2];
    uc[0] = data[3];
  }
  else
  {
    uc[0] = data[0];
    uc[1] = data[1];
    uc[2] = data[2];
    uc[3] = data[3];
   }
   memcpy(&fa,uc,4);
   return fa;
}

//时间戳解包函数
long long timestamp(u8 Data_1,u8 Data_2,u8 Data_3,u8 Data_4)
{
  unsigned long temp; //32位16进制数
  uint16_t H_16,L_16; //存放高16位、低16位

  H_16 = Data_1 << 8 | Data_2;
  L_16 = Data_3 << 8 | Data_4;

  //将融合成16位的数据组合成32位数据
  temp = (unsigned long)H_16<<16|(unsigned long)L_16;

  return temp;
}

//初始化
void setup()   {      
   Serial.begin(115200); //设置与N100惯导通信的波特率
   pinMode(13,OUTPUT);   //板载LED
   delay(200);           //延时等待初始化完成
}

//循环读取数据并显示
void loop()                     
{
  Read_N100Data();//通过串口读取N100惯导模块的数据
  DataUnpacking();//数据解包
}
