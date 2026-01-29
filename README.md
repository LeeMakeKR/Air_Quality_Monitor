# Air Quality Monitor

**ESPHome 기반** NodeMCU v3 실내 공기질 측정 및 자동 제어 시스템

## 📋 개요

이 프로젝트는 ESPHome을 사용하여 실내 공기질을 실시간으로 모니터링하고, 측정값에 따라 환기팬, 공기청정기 등의 기기를 자동으로 제어하는 스마트 시스템입니다.

## 📊 측정 센서

### 1. 이산화탄소 (CO₂) + 온습도

- **센서**: SCD30
- **측정 범위**: 
  - CO₂: 400-40000 ppm
  - 온도: -40°C ~ +70°C
  - 습도: 0% ~ 100% RH
- **통신**: I2C
- **ESPHome 플랫폼**: `scd30`
- **특징**: CO₂, 온도, 습도를 하나의 센서로 측정
- **용도**: 실내 환기 상태 및 쾌적도 모니터링

### 2. 미세먼지 (Particulate Matter)
- **센서**: PMS 시리즈 (PMS5003/PMS7003)
- **측정 항목**: 
  - PM1.0 (초미세먼지)
  - PM2.5 (미세먼지) 
  - PM10 (가시먼지)
- **단위**: μg/m³
- **통신**: UART 시리얼 통신
- **ESPHome 플랫폼**: `pmsx003`

### 3. 온습도 (Temperature & Humidity) - 선택사항

- **센서**: SHT20 또는 AHT20
- **측정 범위**: 
  - 온도: -40°C ~ +125°C
  - 습도: 0% ~ 100% RH
- **통신**: I2C
- **ESPHome 플랫폼**: `htu21d` (SHT20 호환)
- **참고**: SCD30이 온습도를 제공하므로 선택사항

### 4. VOC (휴발성유기화합물)
- **센서**: SGP40
- **측정 항목**:
  - VOC Index (0-500)
- **통신**: I2C
- **ESPHome 플랫펼**: `sgp4x`
- **특징**: 온습도 보정으로 정확도 향상
- **용도**: 실내 공기 오염물질 감지

## 🔧 하드웨어 구성

### 메인 컨트롤러
- **NodeMCU v3** (ESP8266 기반)
- **ESPHome 플랫폼**: 펌웨어 자동 관리
- **WiFi 연결**: 원격 모니터링 및 제어

### 디스플레이 옵션 (선택사양)
| 타입 | 모델 | 해상도 | 통신 | ESPHome 지원 |
|------|------|--------|------|-------------|
| TFT LCD | ST7735 | 128x128 | SPI | ✅ `st7735` |
| OLED | SSD1306/SH1106 | 128x64 | I2C/SPI | ✅ `ssd1306` |
| E-Paper | EPD 2.9"/4.2" | 296x128/400x300 | SPI | ✅ `waveshare_epaper` |

### 센서 모듈
| 센서 | 모델 | 통신 | ESPHome 플랫폼 | 상태 |
|------|------|------|----------------|------|
| CO₂/온습도 | SCD30 | I2C | `scd30` | ✅ 구현완료 |
| 미세먼지 | PMS 시리즈 | UART | `pmsx003` | ✅ 구현완료 |
| VOC | SGP40 | I2C | `sgp4x` | ✅ 구현완료 |
| 온습도 | SHT20/AHT20 | I2C | `htu21d` | ⚠️ 선택사항 |

### 기타 구성요소
- **RGB LED**: 공기질 상태 시각 표시
- **CO₂ 보정 버튼**: 아날로그 입력 (3초 길게 누르기)
- **하드웨어 제어**: 3개 GPIO 출력 (릴레이 제어용)
- **연결성**: WiFi, Home Assistant API, 웹서버

## 🚀 ESPHome 설치 및 설정

### 1. ESPHome 설치
```bash
# pip를 통한 설치
pip install esphome

# 또는 Home Assistant Add-on 사용
# Home Assistant > Add-on Store > ESPHome 설치
```

### 2. 기기 설정
```bash
# YAML 파일 검증
esphome config air_quality_monitor.yaml

# 펌웨어 컴파일 및 업로드 (첫 설치시)
esphome run air_quality_monitor.yaml

# 무선 업데이트 (이후)
esphome upload air_quality_monitor.yaml --device air-quality-monitor.local
```

### 3. 설정 커스터마이징
`air_quality_monitor.yaml` 파일에서 다음 항목들을 수정하세요:

```yaml
wifi:
  ssid: "YOUR_WIFI_SSID"          # WiFi SSID 입력
  password: "YOUR_WIFI_PASSWORD"  # WiFi 비밀번호 입력

api:
  encryption:
    key: "YOUR_API_KEY"           # 32자리 암호화 키

ota:
  password: "YOUR_OTA_PASSWORD"   # OTA 업데이트 비밀번호
```

## 🌟 ESPHome 장점

### 🔌 Home Assistant 완벽 연동
- 자동 기기 발견 및 등록
- 실시간 센서 데이터 스트리밍
- 대시보드 자동 생성
- 자동화 규칙 쉬운 설정

### 📶 원격 모니터링
- 웹브라우저에서 실시간 확인
- 스마트폰 앱으로 어디서나 접근
- 알림 및 경고 메시지 전송

### 🔄 무선 업데이트 (OTA)
- USB 케이블 없이 코드 업데이트
- 원격지 기기도 쉽게 관리
- 자동 백업 및 복구

### 📊 고급 기능
- 히스토리 데이터 로깅
- 그래프 및 차트 표시
- 조건부 자동화 규칙
- 다른 스마트 기기와 연동

## 📌 NodeMCU v3 핀 배치

### ESPHome YAML 핀 정의
### ESPHome YAML 핀 정의
```yaml
# I2C 버스 (온습도, TVOC 센서)
i2c:
  sda: D2  # GPIO4
  scl: D1  # GPIO5

# UART (CO2, 미세먼지 센서)  
uart:
  - id: uart_co2
    tx_pin: D0  # GPIO16
    rx_pin: D5  # GPIO14
  - id: uart_pms
    tx_pin: D7  # GPIO13
    rx_pin: D6  # GPIO12

# 디스플레이 (TFT LCD)
display:
  cs_pin: D8     # GPIO15
  dc_pin: D4     # GPIO2
  reset_pin: D3  # GPIO0

# RGB LED (PWM)
output:
  red: GPIO3    # D9
  green: GPIO1  # D10  
  blue: GPIO16  # D0

# 하드웨어 제어 출력
switch:
  out1: D1  # GPIO5 (환기팬)
  out2: D2  # GPIO4 (공기청정기)
  out3: D6  # GPIO12 (가습기)
```

## 🎛️ 자동 제어 기능

### ESPHome 자동화 특징
- **30초 간격** 자동 제어 실행
- **자동 제어 모드** 스위치로 ON/OFF 가능
- **히스테리시스 제어**로 떨림 방지
- **Home Assistant**에서 원격 제어 가능

### 제어 로직 (ESPHome)
```yaml
# 30초마다 실행되는 자동화
interval:
  - interval: 30s
    then:
      # CO2 > 2000ppm → 환기팬 ON
      # CO2 < 800ppm → 환기팬 OFF
      
      # PM2.5 > 75μg/m³ → 공기청정기 ON  
      # PM2.5 < 15μg/m³ → 공기청정기 OFF
      
      # 습도 < 30% or > 70% → 가습기 ON
      # 습도 30-70% → 가습기 OFF
```

## 📱 사용 방법

### 1. 로컬 웹 인터페이스
```
http://air-quality-monitor.local
```
- 실시간 센서 데이터 확인
- 수동 기기 제어  
- 시스템 로그 확인

### 2. Home Assistant 연동
- 자동 기기 발견 및 추가
- 대시보드에서 모든 센서 확인
- 자동화 규칙 설정
- 알림 및 경고 설정

### 3. CO₂ 센서 보정
- 보정 버튼을 **3초간** 길게 누르기
- 신선한 공기 환경(실외)에서 실행
- 20분간 보정 과정 진행
- ESPHome 로그에서 진행상황 확인

## 🔧 문제해결

### WiFi 연결 실패시
1. 기기가 **"Air-Quality-Monitor"** 핫스팟 생성
2. 비밀번호: `12345678`로 연결
3. 웹브라우저에서 WiFi 설정 페이지 접속
4. 올바른 WiFi 정보 입력

### 센서 데이터가 없을 때
- ESPHome 로그 확인: `esphome logs air_quality_monitor.yaml`
- I2C 스캔 결과 확인 (주소 충돌 체크)
- UART 연결 상태 확인 (TX/RX 올바른 연결)

### Home Assistant 연동 실패
- API 암호화 키 일치 여부 확인
- 방화벽 설정 확인 (포트 6053)
- ESPHome 컴포넌트 설치 여부 확인

## 📁 프로젝트 구조

```
Air_Quality_Monitor/
├── air_quality_monitor.yaml    # ESPHome 설정 파일 (메인)
├── README.md                   # 프로젝트 문서
├── datasheets/                 # 센서 데이터시트
└── Air_Quality_Monitor/        # 기존 Arduino 코드 (참고용)
    └── Air_Quality_Monitor.ino
```

