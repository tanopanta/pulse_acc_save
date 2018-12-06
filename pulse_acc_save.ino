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

    drawPulse.init();

    while(!M5.BtnA.wasReleased()) {
        M5.update();
        int y = analogRead(PIN_INPUT);
        drawPulse.addValue(y);
        drawPulse.showMsg("Push Button A to Start.");
        delay(2);
    }
    M5.lcd.clear(BLACK);

    // 16ミリ秒ごと(62.5Hz)にセンサーリード
    tickerSensor.attach_ms(16, _readSensor);
    // 30秒ごとにフラグ（buffSaveFlg）を立てる
    tickerWriteData.attach_ms(30000, _buffSave);

    // バッファをヒープ領域に確保
    // (Vectorはデータが２の累乗個（ぐらい？）を超えると、再確保ー＞コピーをするので余裕を持っておけばオーバヘッドが減る)
    sdBuff.reserve(2048);
}

unsigned int loopcount = 0;
void loop() {
    //リードフラグが立っているとき読み取り
    // 処理時間　ミリ秒
    if(readSensorFlg) {
        readSensorFlg = false;
        sensorData s;
        getAcc(&s);
        sdBuff.push_back(s);

        return;
    }

    // 脈波の立ち上がり発見時SDに書き込み（だいたい1秒に一回ぐらい）
    // 処理時間30ミリ秒
    if(pulseSensor.sawStartOfBeat()) {
        int rri = pulseSensor.getInterBeatIntervalMs(); //心拍間隔の取得
        unsigned long ms = pulseSensor.getLastBeatTime();

        File file = SD.open(fname, FILE_APPEND);
        if(!file) {
          Serial.println("SD card naiyo~~~");
        } else {
          file.printf("%lu, %d\n", ms, rri);
        }
        file.close();
        
        // 心拍数を表示
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.setTextSize(3);
        M5.Lcd.printf("BPM: %03d", 60000 / rri);

        return;
    }

    // セーブフラグが立っているとき保存
    // 処理時間300ミリ秒
    if(buffSaveFlg) { 
        Serial.println("SD write...");
        File file = SD.open(fnameAcc, FILE_APPEND);
        if(!file) {
            Serial.println("SD card naiyo~~~");
        } else {
            for(int i = 0; i < sdBuff.size(); i++) {
                file.printf("%d, %d, %d, %d, %d, %d\n", sdBuff[i].accX, sdBuff[i].accY,sdBuff[i].accZ, sdBuff[i].gyroX, sdBuff[i].gyroY, sdBuff[i].gyroZ);
            }
        }
        file.close();
        buffSaveFlg = false;
        sdBuff.clear();
        return;
    }

    delay(1); // なにも処理をしなかったときだけ
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
    //IMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
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