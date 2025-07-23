/*하드웨어 
main controller : esp-wroom-32 / 업로드시 ESP32 dev module 
display : tft_st7735 1.44인치 LCD
원하는 센서 : 이산화탄소, 온도, 습도, 미세먼지, voc

sensor :
pms7003 미세먼지 센서
mh-z19b co2 센서 ->교체 
tgs2600 아날로그 가스 센서 -> ENS160으로 교체 
sht20 온습도 센서
-sensirion 센서로 통합하는 게 좋을 듯 함
SGP40	VOC 변화 감지 (습도센서 필요)	유지 가능 or SGP41/ENS160으로 확장 가능
SPS30	미세먼지 정량 측정 (PM10까지)	매우 우수, 유지 추천
SCD40	CO₂ 정량 측정 + 온습도 겸용	일반 실내 환경엔 충분, 유지 가능

부저,LED로 경고 추가

*/    

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SHT2x.h>  //  SHT20 센서용 라이브러리 / SHT2X by Rob Tillaart

// ------------------ LCD 핀 정의 ------------------
#define TFT_CS   5    // Chip Select
#define TFT_RST  27   // Reset
#define TFT_DC   26   // Data/Command

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// ------------------ PMS7003 미세먼지 센서 핀 ------------------
#define PMS_RX 16  // PMS7003 TX -> 이 핀으로 들어옴
#define PMS_TX 17  // 이 핀에서 -> PMS7003 RX로 나감
HardwareSerial pmsSerial(2); // UART2 사용

// ------------------ MH-Z19B CO₂ 센서 핀 ------------------
#define CO2_RX 3   // MH-Z19B TX -> 이 핀으로 들어옴
#define CO2_TX 1   // 이 핀에서 -> MH-Z19B RX로 나감
HardwareSerial co2Serial(1); // UART1 사용

// ------------------ CO₂ 보정 버튼 핀 ------------------
#define CALIBRATION_BTN_PIN 35
unsigned long btnPressStartTime = 0;
bool btnPressed = false;

// 보정용 전역 변수 추가 
bool isCalibrating = false;
unsigned long calibrationStartTime = 0;
const unsigned long CALIBRATION_TIME = 1200000; // 20분 = 1200000ms

// 전역 변수 추가 (파일 상단에)
unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL = 1000; // 1초

// 이전 값 저장용 변수
int prev_pm1_0 = -1, prev_pm2_5 = -1, prev_pm10 = -1, prev_co2 = -1;
float prev_temp = -99.0, prev_humidity = -99.0;

// ------------------ SHT20 온습도 센서 ------------------
SHT2x sht20;  //  SHT20 센서 객체 생성

// ------------------ 안정화 타이머 ------------------
const unsigned long PMS_STABILIZE_TIME = 30000;  // 30초 (30,000ms)
const unsigned long CO2_STABILIZE_TIME = 180000; // 3분 (180,000ms)
const unsigned long SHT20_STABILIZE_TIME = 30000; // 30초 (30,000ms)
//const unsigned long TGS2600_STABILIZE_TIME = 0; //일주일 걸림..
// 부팅 시간 기록 변수 추가
unsigned long bootTime;

// ------------------ 센서 안정화 상태 ------------------
bool pmsReady = false;
bool co2Ready = false;
bool sht20Ready = false;

// ------------------ 센서 데이터 저장 변수 ------------------
int pm1_0 = -1, pm2_5 = -1, pm10 = -1, co2 = -1;
float temperature = -99.0, humidity = -99.0;

// ------------------ 센서 Wake Up 함수 ------------------
void wakeUpPMS7003() {
  uint8_t wakeCmd[] = {0x42, 0x4D, 0xE4, 0x00, 0x01, 0x73};
  pmsSerial.write(wakeCmd, sizeof(wakeCmd));
}

// ------------------ PMS7003 데이터 읽기 함수 ------------------
void readPMData() {
  if (pmsSerial.available() >= 32) {
    if (pmsSerial.read() == 0x42 && pmsSerial.read() == 0x4D) {
      uint8_t buffer[32];
      buffer[0] = 0x42;
      buffer[1] = 0x4D;
      pmsSerial.readBytes(&buffer[2], 30);

      pm1_0 = (buffer[10] << 8) | buffer[11];
      pm2_5 = (buffer[12] << 8) | buffer[13];
      pm10  = (buffer[14] << 8) | buffer[15];
    }
  }
}

// ------------------ CO₂ 데이터 읽기 함수 ------------------
void readCO2Data() {
  uint8_t cmd_get_co2[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
  uint8_t response[9];

  co2Serial.write(cmd_get_co2, 9);  // 명령 전송
  delay(100);

  if (co2Serial.available() >= 9) {
    co2Serial.readBytes(response, 9);
    if (response[0] == 0xFF && response[1] == 0x86) {
      co2 = (response[2] << 8) + response[3]; // HIGH * 256 + LOW
    }
  }
}

// ------------------ 온습도 데이터 읽기 함수 ------------------
void readTempHumidity() {
  sht20.read();  //  한 번에 데이터 읽기
  delay(100);    //  안정적인 데이터 읽기 위해 대기

  temperature = sht20.getTemperature();
  humidity = sht20.getHumidity();

  // 값 검증: 정상 범위를 벗어나면 ERROR 값 처리
  if (temperature < -40 || temperature > 125) {
    temperature = -99.0; // ERROR 값
  }
  if (humidity < 0 || humidity > 100) {
    humidity = -99.0; // ERROR 값
  }
}

void setup() {

  pinMode(CALIBRATION_BTN_PIN, INPUT); // CO2 보정 버튼 핀 입력으로 설정

  // PMS7003 & CO2 센서 시리얼 초기화
  pmsSerial.begin(9600, SERIAL_8N1, PMS_RX, PMS_TX);
  co2Serial.begin(9600, SERIAL_8N1, CO2_RX, CO2_TX);
  wakeUpPMS7003();

  // LCD 초기화
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(0);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);
  tft.setTextSize(1);

  // I2C 통신 초기화 (SHT20)
  Wire.begin();
  sht20.begin();  // SHT20 센서 초기화

  // 부팅 시간 기록
  bootTime = millis();


  // 초기화 메시지 표시
  tft.setCursor(0, 10);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("PM1.0: Initializing...");
  tft.setCursor(0, 40);
  tft.print("PM10: Initializing...");
  tft.setCursor(0, 55);
  tft.print("CO2: Initializing...");
  tft.setCursor(0, 70);
  tft.print("TEMP: Initializing...");
  tft.setCursor(0, 85);
  tft.print("HUM: Initializing...");
}



// loop() 함수 내 센서 읽기 부분 수정
void loop() {
  unsigned long currentTime = millis();
  
  // 센서 안정화 상태 확인하고 안정화 시간이 지났으면 센서 데이터 읽기
  if (!pmsReady && (currentTime - bootTime >= PMS_STABILIZE_TIME)) {
    pmsReady = true;
  }
  if (!co2Ready && (currentTime - bootTime >= CO2_STABILIZE_TIME)) {
    co2Ready = true;
  }
  if (!sht20Ready && (currentTime - bootTime >= SHT20_STABILIZE_TIME)) {
    sht20Ready = true;
  }

  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
    // 센서 데이터 읽기
    if (pmsReady) readPMData();
    if (co2Ready) readCO2Data();
    if (sht20Ready) readTempHumidity();
    
    lastUpdateTime = currentTime;
  }

  // LCD 표시 부분
  tft.setCursor(0, 0);
  
  // PM1.0 표시
  tft.setCursor(0, 10);
  if (prev_pm1_0 != pm1_0) {
    if (pmsReady) {
      tft.setTextColor(ST77XX_BLACK);
      tft.print("PM1.0: Initializing...");
      tft.setCursor(0, 10);
      tft.setTextColor(ST77XX_WHITE);
      tft.print("PM1.0: ");
      tft.println(String(pm1_0) + " ug/m3");
    }
    prev_pm1_0 = pm1_0;
  }

  // PM10 표시
  tft.setCursor(0, 40);
  if (prev_pm10 != pm10) {
    if (pmsReady) {
      tft.setTextColor(ST77XX_BLACK);
      tft.print("PM10: Initializing...");
      tft.setCursor(0, 40);
      tft.setTextColor(ST77XX_WHITE);
      tft.print("PM10: ");
      tft.println(String(pm10) + " ug/m3");
    }
    prev_pm10 = pm10;
  }

  // CO2 표시
  tft.setCursor(0, 55);
  if (prev_co2 != co2) {
    if (co2Ready) {
      tft.setTextColor(ST77XX_BLACK);
      tft.print("CO2: Initializing...");
      tft.setCursor(0, 55);
      tft.setTextColor(ST77XX_WHITE);
      tft.print("CO2: ");
      tft.println(String(co2) + " ppm");
    }
    prev_co2 = co2;
  }

  // 온도 표시
  tft.setCursor(0, 70);
  if (prev_temp != temperature) {
    if (sht20Ready) {
      tft.setTextColor(ST77XX_BLACK);
      tft.print("TEMP: Initializing...");
      tft.setCursor(0, 70);
      tft.setTextColor(ST77XX_WHITE);
      tft.print("TEMP: ");
      tft.println(String(temperature, 1) + " C");
    }
    prev_temp = temperature;
  }

  // 습도 표시
  tft.setCursor(0, 85);
  if (prev_humidity != humidity) {
    if (sht20Ready) {
      tft.setTextColor(ST77XX_BLACK);
      tft.print("HUM: Initializing...");
      tft.setCursor(0, 85);
      tft.setTextColor(ST77XX_WHITE);
      tft.print("HUM: ");
      tft.println(String(humidity, 1) + " %");
    }
    prev_humidity = humidity;
  }

  // CO2 보정 버튼 상태 확인
  if (digitalRead(CALIBRATION_BTN_PIN) == LOW) {  // 버튼이 눌렸을 때
    if (!btnPressed) {  // 처음 눌린 순간
      btnPressed = true;
      btnPressStartTime = millis();
    }
    else if ((millis() - btnPressStartTime) >= 3000) {  // 3초 이상 눌림
      calibrateCO2Sensor();  // 보정 함수 호출
      btnPressed = false;  // 상태 초기화
    }
  }
  else {
    btnPressed = false;  // 버튼에서 손을 뗐을 때 상태 초기화
  }

}

// 보정 함수 구현
void calibrateCO2Sensor() {
    if (!isCalibrating) {
        // 보정 시작 시 명령어 전송
        byte calibrationCommand[] = {0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78};
        co2Serial.write(calibrationCommand, 9);
        
        isCalibrating = true;
        calibrationStartTime = millis();
        return;
    }

    // 보정 중 화면 표시
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(0, 0);
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    tft.println("CO2 CALIBRATION");
    tft.println("IN PROGRESS...");
    
    // 남은 시간 계산 및 표시
    unsigned long elapsedTime = millis() - calibrationStartTime;
    unsigned long remainingTime = (CALIBRATION_TIME - elapsedTime) / 1000;
    
    tft.println();
    tft.print("Time left: ");
    tft.print(remainingTime / 60);
    tft.print(":");
    tft.println(remainingTime % 60);
    
    // 20분 경과 후 보정 완료
    if (elapsedTime >= CALIBRATION_TIME) {
        isCalibrating = false;
        tft.println("\nCalibration");
        tft.println("Complete!");
        delay(2000);
    }
}