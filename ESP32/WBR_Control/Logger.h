#ifndef LOGGER_H
#define LOGGER_H

#include <WiFi.h>     // WiFi相关
#include <Arduino.h>  // Arduino 基本功能
#include "Params.h"
#include "ps_vector.h"
#include <map>

const size_t LOG_INIT_CAP = 50000;

class Logger {
private:
  WiFiServer server;
  const char* ssid;
  const char* password;

  // 数据存储：使用字段名作为键
  std::map<String, ps_vector<float>> dataStorage;
  ps_vector<uint32_t> timeStamps;


public:
  Logger(const char* wifiSSID, const char* wifiPassword, int port = 80)
    : server(port), ssid(wifiSSID), password(wifiPassword) {}

  void begin() {
    WiFi.mode(WIFI_STA);  // 设置为电台模式
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print("Connecting to WiFi...");
      Serial.print(" | WiFi Status: ");
      Serial.println(WiFi.status());  // 状态码输出
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
    if (dataSize == 0) return;  // 如果没有数据则终止

    // 仅发送一次 HTTP 标头
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/csv");
    client.println("Connection: keep-alive");  // 设置为保持连接
    client.println();

    //准备缓冲区以立即传输数据
    String csvData;
    csvData.reserve(2048);  // 保留缓冲区大小

    // 为 CSV 文件创建标头
    csvData += "TimeStamp,";  // 第一列：时间戳
    for (const auto& field : dataStorage) {
      csvData += field.first + ",";  // 添加各个数据字段名称
    }
    csvData += "\n";  // 头尾

    // 以块的形式传输数据的设置
    size_t chunkSize = 1024;  // 一次传输多少数据（需要优化）
    size_t currentSize = 0;   // 跟踪当前数据大小

    // 数据生成和传输
    for (size_t i = 0; i < dataSize; i++) {
      // 为每行生成 CSV 格式的数据
      csvData += String(timeStamps[i]) + ",";  // 添加时间戳
      for (const auto& field : dataStorage) {
        // 为每个字段添加数据（最多 8 位小数）
        if (i < field.second.size()) {
          csvData += String(field.second[i], 8) + ",";
        } else {
          csvData += ",";  // 如果没有数据则添加空格
        }
      }
      csvData += "\n";  // 行尾

      // 当块大小达到时传输数据
      currentSize += csvData.length();
      if (currentSize >= chunkSize) {
        client.print(csvData);  // 数据传输
        csvData = "";           // 空缓冲区
        currentSize = 0;        //重置尺寸
      }
    }

    // 如果还有剩余数据则发送
    if (csvData.length() > 0) {
      client.print(csvData);
    }

    Serial.println("CSV file sent to client.");
  }


  void handleClientRequests() {
    static unsigned long lastWiFiCheck = 0;  // 上次检查 Wi-Fi 状态时间
    unsigned long currentMillis = millis();

    // 检查 Wi-Fi 连接状态
    if (WiFi.status() != WL_CONNECTED && (currentMillis - lastWiFiCheck >= 10000)) {
      // 每 10 秒检查一次 Wi-Fi 状态
      lastWiFiCheck = currentMillis;  //状态检查时间更新

      Serial.println("WiFi not connected. Attempting to reconnect...");
      WiFi.disconnect();
      WiFi.reconnect();

      // 检查重连状态
      if (WiFi.status() != WL_CONNECTED) {
        Serial.print("Reconnecting to WiFi... ");
        Serial.println(" | WiFi Status: " + String(WiFi.status()));
      } else {
        Serial.println("WiFi reconnected successfully.");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
      }
    }

    // Wi-Fi 连接时处理客户端请求
    if (WiFi.status() == WL_CONNECTED) {
      WiFiClient client = server.accept();

      if (client) {
        Serial.println("New Client connected.");

        // 验证客户端是否准备好接收数据
        if (client.available()) {
          String request = client.readStringUntil('\r');
          while (client.available()) client.read();  // 清除客户端缓冲区

          // 处理客户端请求（请求 CSV 文件时）
          if (request.indexOf("/logdata") != -1) {
            Serial.println("Start sending CSV file.");
            sendCSVFile(client);  // CSV文件传输函数调用
          } else {
            Serial.println("Unknown request received.");
          }
        } else {
          Serial.println("No data received from client.");
        }

        client.stop();  // 客户端连接终止
        Serial.println("Client disconnected.");
      }
    }
  }

  void resetLogData() {  // 日志数据重置
    // 仅初始化数据存储中的每个向量
    for (auto& entry : dataStorage) {
      entry.second.clear();  // 仅清除向量
    }

    timeStamps.clear();  // 初始化时间戳向量
    Serial.println("LogData has been reset.");
  }
};

#endif  // LOGGER_H
