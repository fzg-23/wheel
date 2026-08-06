#include "Params.h"
#include "Logger.h"
#include "Timer.h"

Logger WIFI_Logger(ssid, password);
Timer log_timer(Timer::TimerType::Millis);
Timer sampling_timer(Timer::TimerType::Millis);
Timer temp_timer(Timer::TimerType::Millis);

void setup() {
  Serial.begin(SERIAL_BAUDRATE);  // 开始串行通信
  WIFI_Logger.begin();

  // 检查PSRAM状态并初始化
  if (psramFound()) {
    Serial.println("PSRAM available.");
    if (!psramInit()) {
      Serial.println("PSRAM initialization failed!");
      while (1) {}  // 初始化失败时无限循环
    } else {
      Serial.println("PSRAM initialized successfully!");
    }
    Serial.printf("PSRAM size: %d bytes\n", ESP.getPsramSize());  // PSRAM 大小输出
  } else {
    Serial.println("PSRAM not available.");
    while (1) {}  //未检测到 PSRAM 时无限循环
  }

  WIFI_Logger.readyToLogTimeStamp();  //时间记录
    // 开始测量时间
  log_timer.start();
  sampling_timer.start();
}

void loop() {
  // 仅当采样时间结束时执行
  if (sampling_timer.getDuration() >= dt * 1000) {
    // 经过时间输出
    // Serial.print("SamplingTime(ms):");
    // Serial.println(sampling_timer.getDuration());
    sampling_timer.start();  // 初始化采样定时器

    WIFI_Logger.handleClientRequests();  // 日志数据传输

    //============= Logging ======================================================================
    WIFI_Logger.logTimeStamp(log_timer.getDuration());  // Log the current timestamp
    //=============================================================================================
  }
}
