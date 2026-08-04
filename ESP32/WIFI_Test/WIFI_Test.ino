#include "Params.h"
#include "Logger.h"
#include "Timer.h"

Logger WIFI_Logger(ssid, password);
Timer log_timer(Timer::TimerType::Millis);
Timer sampling_timer(Timer::TimerType::Millis);
Timer temp_timer(Timer::TimerType::Millis);

void setup() {
  Serial.begin(SERIAL_BAUDRATE);  // Serial 통신 시작
  WIFI_Logger.begin();

  // PSRAM 상태 확인 및 초기화
  if (psramFound()) {
    Serial.println("PSRAM available.");
    if (!psramInit()) {
      Serial.println("PSRAM initialization failed!");
      while (1) {}  // 초기화 실패 시 무한 루프
    } else {
      Serial.println("PSRAM initialized successfully!");
    }
    Serial.printf("PSRAM size: %d bytes\n", ESP.getPsramSize());  // PSRAM 크기 출력
  } else {
    Serial.println("PSRAM not available.");
    while (1) {}  // PSRAM 미탐지 시 무한 루프
  }

  WIFI_Logger.readyToLogTimeStamp();  // 시간 기록
    // 시간 측정 시작
  log_timer.start();
  sampling_timer.start();
}

void loop() {
  // sampling time이 경과했을 때만 실행
  if (sampling_timer.getDuration() >= dt * 1000) {
    // 경과 시간 출력
    // Serial.print("SamplingTime(ms):");
    // Serial.println(sampling_timer.getDuration());
    sampling_timer.start();  // sampling timer 초기화

    WIFI_Logger.handleClientRequests();  // Log Data 전송

    //============= Logging ======================================================================
    WIFI_Logger.logTimeStamp(log_timer.getDuration());  // Log the current timestamp
    //=============================================================================================
  }
}
