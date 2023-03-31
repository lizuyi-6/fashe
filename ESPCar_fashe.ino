
//B站找 ：arduino捣鼓室 欢迎大家评论区一起交流！如有不足请大家指正 共同进步！网址：https://space.bilibili.com/307396549
//根据需要下载要的库 可以根据编译查看结果
//所有的库打包在这个连接下方 https://www.bilibili.com/video/BV1r34y147SL?spm_id_from=333.999.0.0 



#include <esp_now.h>
#include <WiFi.h>

//需要的安装库
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE

    #include "Wire.h"
#endif

// MPU control/status vars
MPU6050 mpu;
bool dmpReady = false;  // 
uint8_t devStatus;      // 
uint8_t fifoBuffer[64]; //
Quaternion q;           // [w, x, y, z]         
VectorFloat gravity;    // [x, y, z]           
float ypr[3];           // [yaw, pitch, roll]   

// 记录MAC地址 // 34:94:54:25:84:74

uint8_t receiverMacAddress[] = {0x34,0x94,0x54,0x25,0x84,0x74};  

struct PacketData 
{
  byte xAxisValue;
  byte yAxisValue;
  byte zAxisValue;  
};
PacketData data;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Message sent" : "Message failed");
}

void setupMPU()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) 
  {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.setDMPEnabled(true);
      dmpReady = true;
  } 
}

void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  else
  {
    Serial.println("Succes: Initialized ESP-NOW");
  }

  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  else
  {
    Serial.println("Succes: Added peer");
  }   

  //This is to set up MPU6050 sensor
  setupMPU();  
}

void loop() 
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO. Get the Latest packet
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
  {  
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    int xAxisValue = constrain(ypr[2] * 180/M_PI, -90, 90);
    int yAxisValue = constrain(ypr[1] * 180/M_PI, -90, 90);
    int zAxisValue = constrain(ypr[0] * 180/M_PI, -90, 90);    
    data.xAxisValue = map(xAxisValue, -90, 90, 0, 254); 
    data.yAxisValue = map(yAxisValue, -90, 90, 0, 254);
    data.zAxisValue = map(zAxisValue, -90, 90, 0, 254);    

    esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *) &data, sizeof(data));

    String inputData  = inputData + "values " + xAxisValue + "  " + yAxisValue + "  " + zAxisValue;
    Serial.println(inputData);
    delay(50);            
  }
}
