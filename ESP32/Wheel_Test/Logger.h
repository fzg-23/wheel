#ifndef LOGGER_H
#define LOGGER_H

#include <WiFi.h>     // WiFi 관련
#include <Arduino.h>  // 기본 Arduino 함수들
#include "Params.h"
#include "ps_vector.h"
#include <map>

const size_t LOG_INIT_CAP = 50000;

class Logger {
private:
  WiFiServer server;
  const char* ssid;
  const char* password;

  // 데이터 저장소: 필드명을 키로 사용
  std::map<String, ps_vector<float>> dataStorage;
  ps_vector<uint32_t> timeStamps;


public:
  Logger(const char* wifiSSID, const char* wifiPassword, int port = 80)
    : server(port), ssid(wifiSSID), password(wifiPassword) {}

  void begin() {
    WiFi.mode(WIFI_STA);  // 스테이션 모드로 설정
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print("Connecting to WiFi...");
      Serial.print(" | WiFi Status: ");
      Serial.println(WiFi.status());  // 상태 코드 출력
    }

    Serial.println("Connected to WiFi.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    server.begin();
  }

  void readyToLogTimeStamp() {
    timeStamps.reserve(LOG_INIT_CAP);
  }

  void readyToLogValue(const String& fieldName) {
    dataStorage[fieldName].reserve(LOG_INIT_CAP);
  }

  void logTimeStamp(uint32_t timestamp) {
    timeStamps.push_back(timestamp);
  }

  void logValue(const String& fieldName, float value) {
    dataStorage[fieldName].push_back(value);
  }

  void sendCSVFile(WiFiClient& client) {
    size_t dataSize = timeStamps.size();
    if (dataSize == 0) return;  // 데이터가 없으면 종료

    // HTTP 헤더를 한 번만 전송
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/csv");
    client.println("Connection: keep-alive");  // 연결을 유지하도록 설정
    client.println();

    // 데이터를 한 번에 전송하기 위한 버퍼 준비
    String csvData;
    csvData.reserve(2048);  // 버퍼 크기 예약

    // CSV 파일의 헤더 생성
    csvData += "TimeStamp,";  // 첫 번째 열: TimeStamp
    for (const auto& field : dataStorage) {
      csvData += field.first + ",";  // 각 데이터 필드 이름 추가
    }
    csvData += "\n";  // 헤더 끝

    // 데이터를 청크 단위로 전송하기 위한 설정
    size_t chunkSize = 1024;  // 한 번에 전송할 데이터 크기 (최적화 필요)
    size_t currentSize = 0;   // 현재 데이터 크기 추적

    // 데이터 생성 및 전송
    for (size_t i = 0; i < dataSize; i++) {
      // 각 행의 데이터를 CSV 형식으로 생성
      csvData += String(timeStamps[i]) + ",";  // TimeStamp 추가
      for (const auto& field : dataStorage) {
        // 각 필드에 대해 데이터를 추가 (소수점 8자리까지)
        if (i < field.second.size()) {
          csvData += String(field.second[i], 8) + ",";
        } else {
          csvData += ",";  // 데이터가 없는 경우 빈 칸 추가
        }
      }
      csvData += "\n";  // 한 줄 끝

      // 청크 크기에 도달하면 데이터를 전송
      currentSize += csvData.length();
      if (currentSize >= chunkSize) {
        client.print(csvData);  // 데이터 전송
        csvData = "";           // 버퍼 비우기
        currentSize = 0;        // 크기 초기화
      }
    }

    // 남은 데이터가 있으면 전송
    if (csvData.length() > 0) {
      client.print(csvData);
    }

    Serial.println("CSV file sent to client.");
  }


  void handleClientRequests() {
    static unsigned long lastWiFiCheck = 0;  // 마지막 Wi-Fi 상태 확인 시간
    unsigned long currentMillis = millis();

    // Wi-Fi 연결 상태 점검
    if (WiFi.status() != WL_CONNECTED && (currentMillis - lastWiFiCheck >= 10000)) {
      // 10초마다 Wi-Fi 상태를 확인
      lastWiFiCheck = currentMillis;  // 상태 확인 시간 갱신

      Serial.println("WiFi not connected. Attempting to reconnect...");
      WiFi.disconnect();
      WiFi.reconnect();

      // 재연결 상태 점검
      if (WiFi.status() != WL_CONNECTED) {
        Serial.print("Reconnecting to WiFi... ");
        Serial.println(" | WiFi Status: " + String(WiFi.status()));
      } else {
        Serial.println("WiFi reconnected successfully.");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
      }
    }

    // Wi-Fi가 연결되어 있을 경우 클라이언트 요청 처리
    if (WiFi.status() == WL_CONNECTED) {
      WiFiClient client = server.accept();

      if (client) {
        Serial.println("New Client connected.");

        // 클라이언트에서 데이터를 받을 준비가 되었는지 확인
        if (client.available()) {
          String request = client.readStringUntil('\r');
          client.clear();  // 클라이언트 버퍼 비우기

          // 클라이언트 요청 처리 (CSV 파일 요청 시)
          if (request.indexOf("/logdata") != -1) {
            Serial.println("Start sending CSV file.");
            sendCSVFile(client);  // CSV 파일 전송 함수 호출
          } else {
            Serial.println("Unknown request received.");
          }
        } else {
          Serial.println("No data received from client.");
        }

        client.stop();  // 클라이언트 연결 종료
        Serial.println("Client disconnected.");
      }
    }
  }

  void resetLogData() {  // LogData 리셋
    // 데이터 저장소의 각 벡터만 초기화
    for (auto& entry : dataStorage) {
      entry.second.clear();  // 벡터만 클리어
    }

    timeStamps.clear();  // 타임스탬프 벡터 초기화
    Serial.println("LogData has been reset.");
  }
};

#endif  // LOGGER_H
