#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
// =====================================================
// WIFI CONFIGURATION
// =====================================================

static const char* WIFI_SSID = "Fablab";
static const char* WIFI_PASSWORD = "0905606808";
// Static IP configuration
IPAddress local_IP(192, 168, 5, 227);
IPAddress gateway(192, 168, 5, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);
// TCP port used by PC
static constexpr uint16_t TCP_PORT = 5000;

// =====================================================
// STM32 UART CONFIGURATION
// =====================================================

// Example UART pins on ESP32-S3
// STM32 TX -> ESP32-S3 GPIO18 (RX)
// STM32 RX <- ESP32-S3 GPIO17 (TX)

static constexpr int STM32_RX_PIN = 18;
static constexpr int STM32_TX_PIN = 17;

static constexpr uint32_t STM32_BAUD = 250000;
// LED WS2812 tích hợp trên ESP32-S3
// GPIO48 thường được sử dụng trên ESP32-S3 DevKitC-1.
// Kiểm tra lại nếu board của bạn dùng chân khác.
#define WS_LED_PIN 48
#define WS_LED_COUNT 1

Adafruit_NeoPixel wsLed(
    WS_LED_COUNT,
    WS_LED_PIN,
    NEO_GRB + NEO_KHZ800
);

uint32_t lastLedUpdate = 0;
bool ledState = false;
// UART1
HardwareSerial STM32Serial(1);

// =====================================================
// TCP SERVER
// =====================================================

WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;

// =====================================================
// TRANSFER BUFFER
// =====================================================

static constexpr size_t BUFFER_SIZE = 512;

uint8_t uartBuffer[BUFFER_SIZE];
uint8_t tcpBuffer[BUFFER_SIZE];

// Statistics
uint32_t uartToTcpBytes = 0;
uint32_t tcpToUartBytes = 0;

uint32_t lastStatusTime = 0;

// =====================================================
// WIFI CONNECTION
// =====================================================
void updateStatusLED()
{
    uint32_t now = millis();

    // Nhấp nháy mỗi 500 ms
    if (now - lastLedUpdate < 500)
    {
        return;
    }

    lastLedUpdate = now;
    ledState = !ledState;

    if (!ledState)
    {
        wsLed.setPixelColor(0, wsLed.Color(0, 0, 0));
    }
    else if (WiFi.status() == WL_CONNECTED)
    {
        // Xanh lá: đã kết nối Wi-Fi
        wsLed.setPixelColor(0, wsLed.Color(0, 255, 0));
    }
    else
    {
        // Đỏ: chưa kết nối Wi-Fi
        wsLed.setPixelColor(0, wsLed.Color(255, 0, 0));
    }

    wsLed.show();
}
void connectWiFi()
{
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);

    Serial.printf(
        "\nConnecting to Wi-Fi: %s\n",
        WIFI_SSID
    );

    // Configure static IP BEFORE WiFi.begin()
    if (!WiFi.config(
            local_IP,
            gateway,
            subnet,
            primaryDNS))
    {
        Serial.println("Static IP configuration failed!");
    }

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("Wi-Fi connected!");

    Serial.print("ESP32-S3 IP: ");
    Serial.println(WiFi.localIP());

    Serial.printf(
        "TCP port: %u\n",
        TCP_PORT
    );
}

// =====================================================
// START TCP SERVER
// =====================================================

void startTCPServer()
{
    tcpServer.begin();

    Serial.println("TCP server started");
    Serial.printf(
        "Waiting for PC on port %u\n",
        TCP_PORT
    );
}

// =====================================================
// ACCEPT TCP CLIENT
// =====================================================

void acceptTCPClient()
{
    if (tcpClient && tcpClient.connected())
    {
        return;
    }

    if (tcpClient)
    {
        tcpClient.stop();
    }

    WiFiClient newClient = tcpServer.accept();

    if (newClient)
    {
        tcpClient = newClient;

        tcpClient.setNoDelay(true);

        Serial.println();
        Serial.println("PC connected!");

        Serial.print("PC IP: ");
        Serial.println(tcpClient.remoteIP());

        Serial.println(
            "Bidirectional UART <-> TCP bridge active"
        );
    }
}

// =====================================================
// STM32 UART -> PC TCP
// =====================================================

void forwardUARTToTCP()
{
    int availableBytes = STM32Serial.available();

    if (availableBytes <= 0)
    {
        return;
    }

    // Do not accumulate stale telemetry while disconnected.
    if (!tcpClient || !tcpClient.connected())
    {
        while (STM32Serial.available() > 0)
        {
            STM32Serial.read();
        }

        return;
    }

    size_t bytesToRead = min(
        (size_t)availableBytes,
        BUFFER_SIZE
    );

    size_t bytesRead = STM32Serial.readBytes(
        uartBuffer,
        bytesToRead
    );

    if (bytesRead == 0)
    {
        return;
    }

    size_t bytesWritten = tcpClient.write(
        uartBuffer,
        bytesRead
    );

    uartToTcpBytes += bytesWritten;

    // A failed or closed TCP connection will be
    // detected in the next loop iteration.
}

// =====================================================
// PC TCP -> STM32 UART
// =====================================================

void forwardTCPToUART()
{
    if (!tcpClient || !tcpClient.connected())
    {
        return;
    }

    int availableBytes = tcpClient.available();

    if (availableBytes <= 0)
    {
        return;
    }

    size_t bytesToRead = min(
        (size_t)availableBytes,
        BUFFER_SIZE
    );

    int bytesRead = tcpClient.read(
        tcpBuffer,
        bytesToRead
    );

    if (bytesRead <= 0)
    {
        return;
    }

    size_t bytesWritten = STM32Serial.write(
        tcpBuffer,
        (size_t)bytesRead
    );

    tcpToUartBytes += bytesWritten;
}

// =====================================================
// WIFI MONITOR
// =====================================================

void monitorWiFi()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        return;
    }

    Serial.println("Wi-Fi disconnected!");

    if (tcpClient)
    {
        tcpClient.stop();
    }

    WiFi.disconnect();

    connectWiFi();

    // Restart listener after reconnecting.
    tcpServer.end();
    startTCPServer();
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);
    wsLed.begin();
    wsLed.setBrightness(40);
    wsLed.clear();
    wsLed.show();
    delay(1000);

    Serial.println();
    Serial.println("================================");
    Serial.println("ESP32-S3 STM32 TCP BRIDGE");
    Serial.println("================================");

    // UART connection to STM32F411
    STM32Serial.begin(
        STM32_BAUD,
        SERIAL_8N1,
        STM32_RX_PIN,
        STM32_TX_PIN
    );

    // Prevent readBytes from blocking for long periods.
    STM32Serial.setTimeout(2);

    Serial.println("STM32 UART initialized");
    Serial.printf(
        "UART baud rate: %lu\n",
        (unsigned long)STM32_BAUD
    );

    // Connect to Wi-Fi
    connectWiFi();

    // Start TCP server
    startTCPServer();

    Serial.println();
    Serial.println("Bridge ready.");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop()
{
    // 1. Recover Wi-Fi if disconnected
    monitorWiFi();

    // 2. Accept PC TCP connection
    acceptTCPClient();

    // 3. Forward commands: PC -> STM32
    forwardTCPToUART();

    // 4. Forward telemetry: STM32 -> PC
    forwardUARTToTCP();

    // 5. Print status every 5 seconds
    uint32_t now = millis();

    if (now - lastStatusTime >= 5000)
    {
        lastStatusTime = now;

        Serial.printf(
            "WiFi=%s, TCP=%s, UART->TCP=%lu, TCP->UART=%lu\n",
            WiFi.status() == WL_CONNECTED
                ? "connected"
                : "disconnected",
            tcpClient && tcpClient.connected()
                ? "connected"
                : "waiting",
            (unsigned long)uartToTcpBytes,
            (unsigned long)tcpToUartBytes
        );
    }
    // Cập nhật trạng thái LED
    updateStatusLED();
    delay(1);
}