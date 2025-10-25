/*하드웨어 
main controller : NodeMCU v3 (ESP8266) / 업로드시 NodeMCU 1.0 (ESP-12E Module)

디스플레이 옵션 (선택사양):
1. TFT LCD : ST7735 1.44인치 (128x128) - SPI 통신
2. OLED : SSD1306/SH1106 (128x64) - I2C/SPI 통신  
3. E-Paper : EPD 2.9"/4.2" - SPI 통신

측정 목표 센서 :
1. 이산화탄소 (CO₂) - MH-Z19B 센서
2. 미세먼지 (PM1.0, PM2.5, PM10) - PMS 시리즈 (PMS5003/PMS7003)
3. 온습도 - SHT20 또는 AHT20 센서  
4. TVOC/eCO₂/AQI - ENS160 센서

현재 구현된 센서 :
- PMS7003 미세먼지 센서 (Software Serial - NodeMCU 핀 제한 고려)
- MH-Z19B CO₂ 센서 (Software Serial)
- SHT20 온습도 센서 (I2C)

추가 예정 센서 :
- ENS160 TVOC/공기질 센서 (I2C)
- AHT20 온습도 센서 (SHT20 대체 옵션)

주의사항:
- NodeMCU v3는 ESP8266 기반으로 ESP32 대비 GPIO 핀 수 제한
- Software Serial 사용으로 UART 센서 다중 연결
- I2C 센서는 동일 버스 공유로 주소 충돌 방지 필요

부저,LED로 경고 추가

*/    

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// 디스플레이 라이브러리 (사용할 디스플레이에 따라 선택)
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>     // TFT LCD용
// #include <Adafruit_SSD1306.h>   // OLED용 (선택사양)
// #include <GxEPD2_BW.h>          // E-Paper용 (선택사양)

#include <SoftwareSerial.h>  // NodeMCU v3용 Software Serial
#include <SHT2x.h>  //  SHT20 센서용 라이브러리 / SHT2X by Rob Tillaart

// ------------------ NodeMCU v3 핀 정의 (ESP8266) ------------------
// 사용 가능한 GPIO: D0~D8 (일부 제한사항 있음)

// ------------------ TFT LCD 핀 정의 (SPI) ------------------
#define TFT_CS   D8    // GPIO15 - Chip Select  
#define TFT_RST  D3    // GPIO0  - Reset (부팅시 주의)
#define TFT_DC   D4    // GPIO2  - Data/Command
// MOSI: D7 (GPIO13), SCK: D5 (GPIO14) - 하드웨어 SPI 사용

// OLED 핀 정의 (I2C) - 선택사양
// SDA: D2 (GPIO4), SCL: D1 (GPIO5) - 하드웨어 I2C 사용

// E-Paper 핀 정의 (SPI) - 선택사양  
// CS: D8, DC: D4, RST: D3, BUSY: D2 등

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// ------------------ PMS7003 미세먼지 센서 핀 (Software Serial) ------------------
#define PMS_RX D6  // GPIO12 - PMS7003 TX -> 이 핀으로 들어옴
#define PMS_TX D7  // GPIO13 - 이 핀에서 -> PMS7003 RX로 나감 (MOSI와 공유시 주의)
SoftwareSerial pmsSerial(PMS_RX, PMS_TX);

// ------------------ MH-Z19B CO₂ 센서 핀 (Software Serial) ------------------
#define CO2_RX D5  // GPIO14 - MH-Z19B TX -> 이 핀으로 들어옴 (SCK와 공유시 주의)
#define CO2_TX D0  // GPIO16 - 이 핀에서 -> MH-Z19B RX로 나감
SoftwareSerial co2Serial(CO2_RX, CO2_TX);

// ------------------ CO₂ 보정 버튼 핀 ------------------
#define CALIBRATION_BTN_PIN A0  // ADC0 - 아날로그 입력으로 변경

// ------------------ RGB LED 핀 정의 ------------------
#define RGB_LED_R D9   // GPIO3 - 빨간색 (RX와 공유시 주의)
#define RGB_LED_G D10  // GPIO1 - 초록색 (TX와 공유시 주의) 
#define RGB_LED_B D0   // GPIO16 - 파란색

// ------------------ 하드웨어 제어 출력 핀 ------------------
#define OUT1_PIN D1    // GPIO5 - 환기팬 제어 (I2C SCL과 공유)
#define OUT2_PIN D2    // GPIO4 - 공기청정기 제어 (I2C SDA와 공유)
#define OUT3_PIN D6    // GPIO12 - 가습기/제습기 제어 (PMS RX와 공유)
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

// ------------------ 공기질 임계값 정의 ------------------
// CO₂ 임계값 (ppm)
#define CO2_GOOD 800
#define CO2_MODERATE 1200
#define CO2_BAD 2000

// 미세먼지 임계값 (μg/m³)
#define PM25_GOOD 15
#define PM25_MODERATE 35
#define PM25_BAD 75

// 습도 임계값 (%)
#define HUMIDITY_LOW 30
#define HUMIDITY_HIGH 70

// ------------------ 하드웨어 제어 상태 변수 ------------------
bool out1_state = false;  // 환기팬 상태
bool out2_state = false;  // 공기청정기 상태  
bool out3_state = false;  // 가습기/제습기 상태

// RGB LED 상태
int led_r = 0, led_g = 0, led_b = 0;

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

// ------------------ 공기질 평가 및 제어 함수 ------------------
void evaluateAirQuality() {
  // CO₂ 기반 환기팬 제어 (OUT1)
  if (co2 > CO2_BAD) {
    out1_state = true;   // 환기팬 ON
  } else if (co2 < CO2_GOOD) {
    out1_state = false;  // 환기팬 OFF
  }
  
  // 미세먼지 기반 공기청정기 제어 (OUT2)
  if (pm2_5 > PM25_BAD) {
    out2_state = true;   // 공기청정기 ON
  } else if (pm2_5 < PM25_GOOD) {
    out2_state = false;  // 공기청정기 OFF
  }
  
  // 습도 기반 가습기/제습기 제어 (OUT3)
  if (humidity < HUMIDITY_LOW) {
    out3_state = true;   // 가습기 ON
  } else if (humidity > HUMIDITY_HIGH) {
    out3_state = true;   // 제습기 ON (또는 별도 제어 로직)
  } else {
    out3_state = false;  // OFF
  }
  
  // 하드웨어 출력 핀 제어
  digitalWrite(OUT1_PIN, out1_state ? HIGH : LOW);
  digitalWrite(OUT2_PIN, out2_state ? HIGH : LOW);  
  digitalWrite(OUT3_PIN, out3_state ? HIGH : LOW);
}

// ------------------ RGB LED 제어 함수 ------------------
void updateRGBLED() {
  // 종합 공기질 상태에 따른 RGB LED 색상 결정
  int air_quality_score = 0;
  
  // CO₂ 점수 (0-3)
  if (co2 <= CO2_GOOD) air_quality_score += 0;
  else if (co2 <= CO2_MODERATE) air_quality_score += 1;
  else if (co2 <= CO2_BAD) air_quality_score += 2;
  else air_quality_score += 3;
  
  // 미세먼지 점수 (0-3)  
  if (pm2_5 <= PM25_GOOD) air_quality_score += 0;
  else if (pm2_5 <= PM25_MODERATE) air_quality_score += 1;
  else if (pm2_5 <= PM25_BAD) air_quality_score += 2;
  else air_quality_score += 3;
  
  // RGB 색상 설정 (PWM 값: 0-255)
  if (air_quality_score <= 1) {
    // 좋음: 초록색
    led_r = 0; led_g = 255; led_b = 0;
  } else if (air_quality_score <= 3) {
    // 보통: 노란색  
    led_r = 255; led_g = 255; led_b = 0;
  } else if (air_quality_score <= 4) {
    // 나쁨: 주황색
    led_r = 255; led_g = 165; led_b = 0;
  } else {
    // 매우 나쁨: 빨간색
    led_r = 255; led_g = 0; led_b = 0;
  }
  
  // PWM 출력으로 RGB LED 제어
  analogWrite(RGB_LED_R, led_r);
  analogWrite(RGB_LED_G, led_g);
  analogWrite(RGB_LED_B, led_b);
}

void setup() {
  Serial.begin(115200);  // 디버깅용 시리얼 통신
  
  pinMode(CALIBRATION_BTN_PIN, INPUT);  // CO2 보정 버튼 (아날로그 핀)
  
  // RGB LED 핀 출력 설정
  pinMode(RGB_LED_R, OUTPUT);
  pinMode(RGB_LED_G, OUTPUT);
  pinMode(RGB_LED_B, OUTPUT);
  
  // 하드웨어 제어 출력 핀 설정
  pinMode(OUT1_PIN, OUTPUT);
  pinMode(OUT2_PIN, OUTPUT);
  pinMode(OUT3_PIN, OUTPUT);
  
  // 초기 상태: 모든 출력 OFF
  digitalWrite(OUT1_PIN, LOW);
  digitalWrite(OUT2_PIN, LOW);
  digitalWrite(OUT3_PIN, LOW);
  analogWrite(RGB_LED_R, 0);
  analogWrite(RGB_LED_G, 0);
  analogWrite(RGB_LED_B, 0);

  // PMS7003 & CO2 센서 Software Serial 초기화 (NodeMCU v3용)
  pmsSerial.begin(9600);
  co2Serial.begin(9600);
  wakeUpPMS7003();

  // TFT LCD 초기화
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(0);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);
  tft.setTextSize(1);

  // I2C 통신 초기화 (SHT20) - NodeMCU v3: SDA=D2, SCL=D1
  Wire.begin(D2, D1);  // SDA, SCL 핀 명시적 지정
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
    
    // 공기질 평가 및 하드웨어 제어
    if (pmsReady && co2Ready && sht20Ready) {
      evaluateAirQuality();
      updateRGBLED();
    }
    
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

  // CO2 보정 버튼 상태 확인 (아날로그 입력)
  int btnValue = analogRead(CALIBRATION_BTN_PIN);
  if (btnValue < 512) {  // 버튼이 눌렸을 때 (아날로그 값이 낮아짐)
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