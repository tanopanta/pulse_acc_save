#include <M5Stack.h>
#include <Ticker.h>
#include "utility/MPU9250.h"
#include <vector>

#include <MyPulseSensorPlayground.h>
#include <drawPulse.h>

const int PIN_INPUT = 36;
const int THRESHOLD = 550;   // Adjust this number to avoid noise when idle

const char* fname = "/pulse/out.csv";
const char* fnameAcc = "/pulse/outacc.csv";

PulseSensorPlayground pulseSensor;
DrawPulse drawPulse;
MPU9250 IMU;

Ticker tickerSensor; // センサの値を読む
Ticker tickerWriteData; // バッファにためた加速度データをCSVに書き込み

volatile bool buffSaveFlg = false;
volatile bool readSensorFlg = false;

struct sensorData {
  int accX;
  int accY;
  int accZ;
  int gyroX;
  int gyroY;
  int gyroZ;
};
std::vector<sensorData> sdBuff;
void getAcc(sensorData* pSensorData);



void setup() {
    M5.begin();
    Wire.begin();
    dacWrite(25, 0); // Speaker OFF(スピーカーノイズ対策)
    
    initPulseSensor();
    initAccSensor();


    M5.lcd.println("Push Button A to Start.");
    while(!M5.BtnA.wasReleased()) {
      M5.update(); 
    }
    M5.lcd.clear(BLACK);
    drawPulse.init();
    // 16ミリ秒ごと(62.5Hz)にセンサーリード
    tickerSensor.attach_ms(16, _readSensor);
    // 30秒ごとにフラグ（buffSaveFlg）を立てる
    tickerWriteData.attach_ms(30000, _buffSave);
    sdBuff.reserve(2048);
}

unsigned int loopcount = 0;
void loop() {
    
    //リードフラグが立っているとき読み取り
    if(readSensorFlg) {
        readSensorFlg = false;
        sensorData s;
        getAcc(&s);
        sdBuff.push_back(s);
    }

    // 脈波の立ち上がり発見時SDに書き込み（だいたい1秒に一回ぐらい）
    if(pulseSensor.sawStartOfBeat()) {
        int rri = pulseSensor.getInterBeatIntervalMs(); //心拍間隔の取得
        File file = SD.open(fname, FILE_APPEND);
        if(!file) {
          Serial.println("SD card naiyo~~~");
        } else {
          file.println(rri);
        }
        file.close();
        drawPulse.showMsg("BPM:" + String(60000/ rri));
    }

    // セーブフラグが立っているとき保存
    if(buffSaveFlg) {
        unsigned long start_time = millis();
        Serial.println("SD write...");
        File file = SD.open(fnameAcc, FILE_APPEND);
        if(!file) {
            Serial.println("SD card naiyo~~~");
        } else {
            for(int i = 0; i < sdBuff.size(); i++) {
                //char buf[64];
                //sprintf(buf, "%d, %d, %d, %d, %d, %d", sdBuff[i].accX, sdBuff[i].accY,sdBuff[i].accZ, sdBuff[i].gyroX, sdBuff[i].gyroY, sdBuff[i].gyroZ);
                file.printf("%d, %d, %d, %d, %d, %d", sdBuff[i].accX, sdBuff[i].accY,sdBuff[i].accZ, sdBuff[i].gyroX, sdBuff[i].gyroY, sdBuff[i].gyroZ);
                
            }
        }
        file.close();
        buffSaveFlg = false;
        sdBuff.clear();
        unsigned long end_time = millis();
        Serial.println(end_time - start_time);
        return;
    }

    if(loopcount++ % 20 == 0) {
        int y = pulseSensor.getLatestSample();
        drawPulse.addValue(y);
    }

    delay(1);
}


void initPulseSensor() {
    pulseSensor.analogInput(PIN_INPUT);
    pulseSensor.setThreshold(THRESHOLD);

    while (!pulseSensor.begin()) {
        Serial.println("PulseSensor.begin: failed");
        delay(500);
    }
}

void initAccSensor() {
    IMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
    delay(10);
    IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);
    delay(10);
    IMU.initMPU9250();
}

void getAcc(sensorData* pSensorData) {
    // センサから各種情報を読み取り
    IMU.readAccelData(IMU.accelCount);
    IMU.getAres();
    IMU.readGyroData(IMU.gyroCount);
    IMU.getGres();

    // 取得した加速度に解像度をかけて、バイアス値を引く
    IMU.ax = (float)IMU.accelCount[0]*IMU.aRes - IMU.accelBias[0];
    IMU.ay = (float)IMU.accelCount[1]*IMU.aRes - IMU.accelBias[1];
    IMU.az = (float)IMU.accelCount[2]*IMU.aRes - IMU.accelBias[2];

    // 取得したジャイロに解像度をかける
    IMU.gx = (float)IMU.gyroCount[0]*IMU.gRes;
    IMU.gy = (float)IMU.gyroCount[1]*IMU.gRes;
    IMU.gz = (float)IMU.gyroCount[2]*IMU.gRes;

    // 加速度・ジャイロを与えられた構造体に代入
    pSensorData -> accX = (int)(1000*IMU.ax);
    pSensorData -> accY = (int)(1000*IMU.ay);
    pSensorData -> accZ = (int)(1000*IMU.az);
    pSensorData -> gyroX = (int)(IMU.gx);
    pSensorData -> gyroY = (int)(IMU.gy);
    pSensorData -> gyroZ = (int)(IMU.gz);

}
//ハンドラ－１（センサーを読んでバッファリング）
void _readSensor() {
    readSensorFlg = true;
}

//ハンドラ－２（SD保存のフラグを管理）
void _buffSave() {
    buffSaveFlg = true;
}