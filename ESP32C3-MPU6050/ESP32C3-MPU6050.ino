#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Preferences.h>
#include <esp_gap_ble_api.h>

// Debug mode settings - Thay đổi định nghĩa macro
#define DEBUG_MODE false  // Đặt thành true/false cho sản phẩm thực tế

#if DEBUG_MODE
  #define DEBUG_BEGIN(x) Serial.begin(x)
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
  #define DEBUG_PRINT_HEX(x) Serial.print(x, HEX)  // Thêm macro này
#else
  #define DEBUG_BEGIN(x)
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(...)
  #define DEBUG_PRINT_HEX(x)  // Phiên bản rỗng
#endif

// Pin definitions
#define BUZZER_PIN 1
#define LED_PIN 6
#define MPU_SDA 5
#define MPU_SCL 4
#define RESET_BUTTON_PIN 39  // Nút reset password

// MPU6050 registers
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A

// BLE UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CRASH_UUID          "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BUZZER_UUID         "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e"
#define PASSWORD_UUID       "8c3c3e3f-d8f7-413a-bf3d-7a2e5d7be87e"
#define DEVICE_LIST_UUID    "9d4d4e4f-d8f7-413a-bf3d-7a2e5d7be87e"
#define SENSITIVITY_UUID    "6e95d5e3-d8f7-413a-bf3d-7a2e5d7be87e"

// Constants
#define MAX_DEVICES 10
#define CRASH_THRESHOLD 4.0
#define DEBOUNCE_TIME 2000
#define BUTTON_HOLD_TIME 3000  // Giữ nút 3 giây để reset

// Cấu hình bộ lọc MPU6050
#define SAMPLE_FREQ 100      // Tần số lấy mẫu (Hz)
#define NUM_SAMPLES 10       // Số mẫu trong bộ lọc trung bình di động
#define IMPACT_WINDOW 5      // Cửa sổ phát hiện va chạm (số mẫu)
#define IMPACT_THRESHOLD 1.5 // Ngưỡng tăng đột biến (hệ số so với trung bình)

// Device info structure
struct DeviceInfo {
    String deviceIdentifier;  // Định danh thiết bị (brand_model_androidId)
    bool active;
};

// Global variables
BLEServer* pServer = NULL;
BLECharacteristic* pCrashChar = NULL;
BLECharacteristic* pBuzzerChar = NULL;
BLECharacteristic* pPasswordChar = NULL;
BLECharacteristic* pDeviceListChar = NULL;
BLECharacteristic* pSensitivityChar = NULL;

Preferences preferences;
bool deviceConnected = false;
String devicePassword = "";
DeviceInfo allowedDevices[MAX_DEVICES];
int deviceCount = 0;
unsigned long lastCrashTime = 0;
String currentClientId = "";
bool clientAuthenticated = false;

// Reset button variables
unsigned long buttonPressTime = 0;
bool buttonPressed = false;
bool resetInProgress = false;

// Thêm các biến trạng thái va chạm
bool crashAlarmActive = false;          // Trạng thái đang báo động
unsigned long crashDetectedTime = 0;     // Thời điểm phát hiện va chạm
unsigned long crashCooldownTime = 10000; // Thời gian nghỉ giữa các lần báo động (10 giây)
bool crashNotified = false;              // Đã gửi thông báo BLE

// MPU6050 variables
float crashThreshold = CRASH_THRESHOLD; // Default value
float accelReadings[NUM_SAMPLES] = {0};  // Bộ đệm cho bộ lọc trung bình di động
int readIndex = 0;                       // Chỉ số hiện tại trong bộ đệm
float accelSum = 0;                      // Tổng các giá trị trong bộ đệm
float filteredGForce = 0;                // Giá trị G-force đã lọc
float peakValues[IMPACT_WINDOW] = {0};   // Lưu các giá trị cao nhất để xác định va chạm
int peakIndex = 0;                       // Chỉ số trong bộ đệm peak
unsigned long lastSampleTime = 0;        // Thời điểm lấy mẫu cuối cùng
bool mpuInitialized = false;             // Trạng thái khởi tạo MPU

// Function declarations
void saveSettings();
void loadSettings();
void setupMPU6050();
float calculateGForce();
float updateFilter(float newReading);
bool detectImpact();

// Initialize all devices
void initializeDevices() {
    for (int i = 0; i < MAX_DEVICES; i++) {
        allowedDevices[i].deviceIdentifier = "";
        allowedDevices[i].active = false;
    }
}

// Check if device is allowed
bool isDeviceAllowed(String deviceIdentifier) {
    DEBUG_PRINTLN("--- Checking if device allowed ---");
    DEBUG_PRINT("Device Identifier: ");
    DEBUG_PRINTLN(deviceIdentifier);
    
    if (devicePassword == "") {
        DEBUG_PRINTLN("No password set - returning NO_PASSWORD"); 
        return false; // Trả về false để không cho phép kết nối
    }
    
    for (int i = 0; i < deviceCount; i++) {
        if (allowedDevices[i].deviceIdentifier == deviceIdentifier && allowedDevices[i].active) {
            DEBUG_PRINTLN("Device found in whitelist");
            currentClientId = deviceIdentifier; 
            return true;
        }
    }
    
    DEBUG_PRINTLN("Device NOT in whitelist");
    return false;
}

// Add device to whitelist
void addDevice(String deviceIdentifier) {
    DEBUG_PRINTLN("--- Adding device to whitelist ---");
    DEBUG_PRINT("Device Identifier: ");
    DEBUG_PRINTLN(deviceIdentifier);
    
    // Kiểm tra thiết bị đã tồn tại
    for (int i = 0; i < deviceCount; i++) {
        if (allowedDevices[i].deviceIdentifier == deviceIdentifier) {
            DEBUG_PRINTLN("Device already exists - updating");
            allowedDevices[i].active = true;
            saveSettings();
            return;
        }
    }
    
    // Thêm thiết bị mới
    if (deviceCount < MAX_DEVICES) {
        allowedDevices[deviceCount].deviceIdentifier = deviceIdentifier;
        allowedDevices[deviceCount].active = true;
        deviceCount++;
        saveSettings();
    }
}

// Remove device from whitelist
void removeDevice(String deviceIdentifier) {
    for (int i = 0; i < deviceCount; i++) {
        if (allowedDevices[i].deviceIdentifier == deviceIdentifier) {
            // Shift remaining devices
            for (int j = i; j < deviceCount - 1; j++) {
                allowedDevices[j] = allowedDevices[j + 1];
            }
            deviceCount--;
            saveSettings();
            break;
        }
    }
}

// Reset all settings
void resetAllSettings() {
    DEBUG_PRINTLN("=== RESETTING ALL SETTINGS ===");
    
    // Visual feedback - LED blink fast
    for (int i = 0; i < 10; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }
    
    // Clear preferences
    preferences.begin("bike-safety", false);
    preferences.clear();
    preferences.end();
    
    // Sound feedback
    for (int i = 0; i < 3; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(100);
        digitalWrite(BUZZER_PIN, LOW);
        delay(100);
    }
    
    DEBUG_PRINTLN("Settings cleared - Restarting...");
    delay(1000);
    ESP.restart();
}

// Load settings from NVS
void loadSettings() {
    preferences.begin("bike-safety", true);  // Read-only
    
    devicePassword = preferences.getString("password", "");
    deviceCount = preferences.getInt("deviceCount", 0);
    
    // Load crash threshold
    crashThreshold = preferences.getFloat("crash_threshold", CRASH_THRESHOLD);
    
    DEBUG_PRINTLN("=== Loading Settings ===");
    DEBUG_PRINT("Password loaded: '");
    DEBUG_PRINT(devicePassword);
    DEBUG_PRINTLN("'");
    DEBUG_PRINT("Password length: ");
    DEBUG_PRINTLN(devicePassword.length());
    DEBUG_PRINT("Password exists: ");
    DEBUG_PRINTLN(devicePassword != "" ? "Yes" : "No");
    DEBUG_PRINT("Device count: ");
    DEBUG_PRINTLN(deviceCount);
    DEBUG_PRINT("Crash threshold: ");
    DEBUG_PRINT(crashThreshold);
    DEBUG_PRINTLN("G");
    
    // Print password hex values
    if (devicePassword != "") {
        DEBUG_PRINT("Password hex: ");
        for (int i = 0; i < devicePassword.length(); i++) {
            DEBUG_PRINT_HEX(devicePassword[i]);
            DEBUG_PRINT(" ");
        }
        DEBUG_PRINTLN();
    }
    
    // Validate device count
    if (deviceCount > MAX_DEVICES) {
        DEBUG_PRINTLN("ERROR: Invalid device count, resetting");
        deviceCount = 0;
    }
    
    for (int i = 0; i < deviceCount; i++) {
        String key = "device" + String(i);
        allowedDevices[i].deviceIdentifier = preferences.getString(key.c_str(), "");
        allowedDevices[i].active = preferences.getBool((key + "_active").c_str(), true);
        
        DEBUG_PRINT("Device ");
        DEBUG_PRINT(i);
        DEBUG_PRINT(": ID=");
        DEBUG_PRINTLN(allowedDevices[i].deviceIdentifier); // Chỉ in deviceIdentifier
    }
    
    preferences.end();
    DEBUG_PRINTLN("=== Settings loaded ===");
}

// Save settings to NVS
void saveSettings() {
    preferences.begin("bike-safety", false);
    preferences.putString("password", devicePassword);
    preferences.putInt("deviceCount", deviceCount);
    preferences.putFloat("crash_threshold", crashThreshold);
    
    for (int i = 0; i < deviceCount; i++) {
        String key = "device" + String(i);
        preferences.putString(key.c_str(), allowedDevices[i].deviceIdentifier);
        preferences.putBool((key + "_active").c_str(), allowedDevices[i].active);
    }
    
    preferences.end();
    DEBUG_PRINTLN("Settings saved successfully");
}

// BLE Server callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        clientAuthenticated = false; // Reset trạng thái xác thực khi có kết nối mới
        digitalWrite(LED_PIN, HIGH);
        DEBUG_PRINTLN("=== Client connected ===");
    }
    
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        clientAuthenticated = false;
        digitalWrite(LED_PIN, LOW);
        currentClientId = "";
        DEBUG_PRINTLN("=== Client disconnected ===");
    }
};

// Password characteristic callbacks
class PasswordCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        String command = String(value.c_str());
        
        DEBUG_PRINTLN("\n=== Password command received ===");
        DEBUG_PRINT("Full command: ");
        DEBUG_PRINTLN(command);
        
        int colonIndex = command.indexOf(':');
        if (colonIndex > 0) {
            String cmd = command.substring(0, colonIndex);
            String params = command.substring(colonIndex + 1);
            
            DEBUG_PRINT("Command: ");
            DEBUG_PRINTLN(cmd);
            DEBUG_PRINT("Params: ");
            DEBUG_PRINTLN(params);
            
            if (cmd == "SETPASS") {
                // Parse: SETPASS:<password>:<identifier>
                int passwordEnd = params.indexOf(':');
                if (passwordEnd > 0) {
                    String password = params.substring(0, passwordEnd);
                    String deviceIdentifier = params.substring(passwordEnd + 1);
                    
                    if (devicePassword == "") {
                        devicePassword = password;
                        currentClientId = deviceIdentifier;
                        clientAuthenticated = true;
                        addDevice(deviceIdentifier);
                        saveSettings();
                        pCharacteristic->setValue("OK");
                        DEBUG_PRINTLN("Password set successfully");
                    } else {
                        pCharacteristic->setValue("FAIL");
                        DEBUG_PRINTLN("Password already set");
                    }
                } else {
                    pCharacteristic->setValue("FAIL");
                    DEBUG_PRINTLN("Invalid format");
                }
            }
            else if (cmd == "AUTH") {
                // Parse: AUTH:<identifier>:<password>
                int identifierEnd = params.indexOf(':');
                if (identifierEnd > 0) {
                    String deviceIdentifier = params.substring(0, identifierEnd);
                    String password = params.substring(identifierEnd + 1);
                    
                    if (password == devicePassword) {
                        currentClientId = deviceIdentifier;
                        clientAuthenticated = true;
                        addDevice(deviceIdentifier);
                        pCharacteristic->setValue("OK");
                        DEBUG_PRINTLN("Auth successful");
                    } else {
                        pCharacteristic->setValue("FAIL");
                        DEBUG_PRINTLN("Wrong password");
                    }
                } else {
                    pCharacteristic->setValue("FAIL");
                    DEBUG_PRINTLN("Invalid format");
                }
            } else if (cmd == "CHECK") {
                DEBUG_PRINTLN("--- CHECK command ---");
                String deviceId = params;
                
                DEBUG_PRINT("Checking device: ");
                DEBUG_PRINTLN(deviceId);
                
                if (devicePassword == "") {
                    pCharacteristic->setValue("NO_PASSWORD");
                    DEBUG_PRINTLN("Response: NO_PASSWORD");
                } else if (isDeviceAllowed(deviceId)) {
                    currentClientId = deviceId;
                    pCharacteristic->setValue("ALLOWED");
                    DEBUG_PRINTLN("Response: ALLOWED");
                } else {
                    pCharacteristic->setValue("NOT_ALLOWED");
                    DEBUG_PRINTLN("Response: NOT_ALLOWED");
                }
                
            } else if (cmd == "CHANGEPASS") {
                DEBUG_PRINTLN("--- CHANGEPASS command ---");
                
                if (clientAuthenticated) {
                    devicePassword = params;
                    saveSettings();
                    pCharacteristic->setValue("OK");
                    DEBUG_PRINTLN("Password changed");
                } else {
                    pCharacteristic->setValue("FAIL");
                    DEBUG_PRINTLN("Not authenticated");
                }
                
            } else if (cmd == "REMOVE") {
                DEBUG_PRINTLN("--- REMOVE command ---");
                
                if (clientAuthenticated) {
                    removeDevice(params);
                    pCharacteristic->setValue("OK");
                } else {
                    pCharacteristic->setValue("FAIL");
                }
                
            } else if (cmd == "RESET") {
                DEBUG_PRINTLN("--- RESET command ---");
                
                if (clientAuthenticated) {
                    pCharacteristic->setValue("OK");
                    delay(100);
                    resetAllSettings();
                } else {
                    pCharacteristic->setValue("FAIL");
                }
            }
        }
        
        DEBUG_PRINTLN("=== Command processing done ===\n");
    }
};

// Buzzer characteristic callbacks
class BuzzerCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        
        DEBUG_PRINTLN("\n=== Buzzer command received ===");
        DEBUG_PRINT("Current client: ");
        DEBUG_PRINTLN(currentClientId);
        DEBUG_PRINT("Client authenticated: ");
        DEBUG_PRINTLN(clientAuthenticated ? "Yes" : "No");
        
        if (value.length() > 0 && clientAuthenticated) {
            if (value[0] == '1') {
                DEBUG_PRINTLN("Buzzer activated");
                for (int i = 0; i < 5; i++) {
                    digitalWrite(BUZZER_PIN, HIGH);
                    delay(100);
                    digitalWrite(BUZZER_PIN, LOW);
                    delay(100);
                }
            } else if (value[0] == '2') {
                // Thêm lệnh reset trạng thái va chạm
                DEBUG_PRINTLN("Reset crash state command received");
                resetCrashState();
            }
        } else {
            DEBUG_PRINTLN("ERROR: Client not authenticated");
        }
    }
};

// Sensitivity characteristic callbacks
class SensitivityCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            // Parse float value từ string
            crashThreshold = atof(value.c_str());
            DEBUG_PRINT("New crash threshold: ");
            DEBUG_PRINTLN(crashThreshold);
            
            // Lưu vào preferences
            preferences.begin("bike-safety", false);
            preferences.putFloat("crash_threshold", crashThreshold);
            preferences.end();
        }
    }
};

// Setup MPU6050 with improved configuration
void setupMPU6050() {
    Wire.begin(MPU_SDA, MPU_SCL);
    Wire.setClock(400000); // Tăng tốc độ I2C lên 400kHz

    // Reset MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x80);  // Bit 7 để reset
    Wire.endTransmission(true);
    delay(100);  // Đợi reset

    // Wake up MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x01);  // Sử dụng gyro X làm clock reference
    Wire.endTransmission(true);
    delay(10);

    // Thiết lập tốc độ lấy mẫu
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(SMPLRT_DIV);
    Wire.write(0x04);  // Tốc độ lấy mẫu = 1kHz / (1 + 4) = 200Hz
    Wire.endTransmission(true);

    // Cấu hình Digital Low Pass Filter (DLPF)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(CONFIG);
    Wire.write(0x03);  // DLPF_CFG = 3, bandwidth = 44Hz
    Wire.endTransmission(true);
    
    // Set accelerometer range to ±16g
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(ACCEL_CONFIG);
    Wire.write(0x18);  // 0x18 = 0b00011000 (bits [4:3] = 0b11 cho ±16g)
    Wire.endTransmission(true);

    // Khởi tạo bộ lọc
    for (int i = 0; i < NUM_SAMPLES; i++) {
        accelReadings[i] = 0;
    }
    for (int i = 0; i < IMPACT_WINDOW; i++) {
        peakValues[i] = 0;
    }
    
    // Khởi tạo bộ lọc bằng cách đọc một số mẫu
    for (int i = 0; i < NUM_SAMPLES; i++) {
        float g = calculateRawGForce();
        updateFilter(g);
        delay(5);
    }
    
    mpuInitialized = true;
    DEBUG_PRINTLN("MPU6050 initialized with improved settings");
}

// Calculate raw G-force from accelerometer (without filtering)
float calculateRawGForce() {
    // Kiểm tra kết nối MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    byte error = Wire.endTransmission();
    if (error != 0) {
        DEBUG_PRINT("MPU6050 I2C error: ");
        DEBUG_PRINTLN(error);
        return 0;
    }
    
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (size_t)6, true);
    
    if (Wire.available() < 6) {
        DEBUG_PRINTLN("Failed to read 6 bytes from MPU6050");
        return 0;
    }
    
    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();
    
    // Hệ số chia 2048.0 cho thang đo ±16g (thay vì 16384.0 cho ±2g)
    float gx = ax / 2048.0f;
    float gy = ay / 2048.0f;
    float gz = az / 2048.0f;
    
    // Magnitude of acceleration vector
    float gForce = sqrt(gx*gx + gy*gy + gz*gz);
    
    return gForce;
}

// Update moving average filter with new G-force reading
float updateFilter(float newReading) {
    // Cập nhật tổng bằng cách trừ giá trị cũ và thêm giá trị mới
    accelSum = accelSum - accelReadings[readIndex] + newReading;
    
    // Lưu giá trị mới vào bộ đệm
    accelReadings[readIndex] = newReading;
    
    // Cập nhật chỉ số cho lần đọc tiếp theo
    readIndex = (readIndex + 1) % NUM_SAMPLES;
    
    // Tính giá trị trung bình
    filteredGForce = accelSum / NUM_SAMPLES;
    
    return filteredGForce;
}

// Detect impact using spike detection algorithm
bool detectImpact() {
    // Trước tiên kiểm tra xem giá trị hiện tại có vượt ngưỡng không
    // Nếu không vượt ngưỡng cơ bản, không cần xử lý tiếp
    if (filteredGForce < crashThreshold * 0.7f) {
        // Reset buffer nếu giá trị quá thấp để tránh nhiễu
        if (filteredGForce < 1.0f) {
            for (int i = 0; i < IMPACT_WINDOW; i++) {
                peakValues[i] = filteredGForce;
            }
        }
        return false;
    }
    
    // Lưu giá trị cao nhất hiện tại vào bộ đệm
    peakValues[peakIndex] = filteredGForce;
    peakIndex = (peakIndex + 1) % IMPACT_WINDOW;
    
    // Tìm giá trị cao nhất trong cửa sổ
    float maxVal = 0;
    for (int i = 0; i < IMPACT_WINDOW; i++) {
        if (peakValues[i] > maxVal) {
            maxVal = peakValues[i];
        }
    }
    
    // Trước khi tiếp tục, kiểm tra xem giá trị cao nhất có vượt ngưỡng không
    if (maxVal < crashThreshold) {
        return false;
    }
    
    // So sánh với baseline (đường cơ sở)
    // Tính baseline từ các giá trị trước đó trong cửa sổ cần hợp lý hơn
    float baseline = 0;
    int count = 0;
    
    // Tìm giá trị trung bình của các mẫu nhỏ hơn
    // nhưng chỉ dùng các mẫu không quá cũ
    float minVal = 100.0f;
    for (int i = 0; i < IMPACT_WINDOW; i++) {
        if (peakValues[i] < minVal && peakValues[i] > 0.5f) {
            minVal = peakValues[i];
        }
    }
    
    // Đảm bảo baseline hợp lý
    baseline = (minVal > 1.0f) ? minVal : 1.0f; // Không để baseline quá thấp
    
    // Tính tỷ lệ tăng
    float spikeRatio = maxVal / baseline;
    
    // In thông tin debug với đầy đủ chi tiết
    if (DEBUG_MODE) {
        DEBUG_PRINT("Current G: ");
        DEBUG_PRINT(filteredGForce);
        DEBUG_PRINT("G, Peak: ");
        DEBUG_PRINT(maxVal);
        DEBUG_PRINT("G, Baseline: ");
        DEBUG_PRINT(baseline);
        DEBUG_PRINT("G, Ratio: ");
        DEBUG_PRINT(spikeRatio);
        DEBUG_PRINT(", Threshold: ");
        DEBUG_PRINT(crashThreshold);
        DEBUG_PRINTLN("G");
    }
    
    // Cần thỏa mãn đồng thời các điều kiện:
    // 1. Giá trị hiện tại phải gần với giá trị peak
    // 2. Peak phải vượt ngưỡng
    // 3. Tỷ lệ phải vượt IMPACT_THRESHOLD
    bool isPeakRecent = (filteredGForce > maxVal * 0.7f); // Đảm bảo đỉnh là gần đây
    bool isAboveThreshold = (maxVal > crashThreshold);
    bool isHighRatio = (spikeRatio > IMPACT_THRESHOLD);
    
    // Chỉ khi tất cả điều kiện thỏa mãn
    if (isAboveThreshold && isHighRatio && isPeakRecent) {
        // In thông tin chi tiết khi phát hiện va chạm
        DEBUG_PRINTLN("*** CRASH CRITERIA MET ***");
        DEBUG_PRINT("- Current G: ");
        DEBUG_PRINT(filteredGForce);
        DEBUG_PRINT("G >= Peak * 0.7 (");
        DEBUG_PRINT(maxVal * 0.7f);
        DEBUG_PRINTLN("G)");
        DEBUG_PRINT("- Peak: ");
        DEBUG_PRINT(maxVal);
        DEBUG_PRINT("G > Threshold: ");
        DEBUG_PRINT(crashThreshold);
        DEBUG_PRINTLN("G");
        DEBUG_PRINT("- Ratio: ");
        DEBUG_PRINT(spikeRatio);
        DEBUG_PRINT(" > ");
        DEBUG_PRINTLN(IMPACT_THRESHOLD);
        
        return true;
    }
    
    return false;
}

// Update accelerometer readings and check for crash
void checkForCrash() {
    if (!mpuInitialized) {
        setupMPU6050();
        return;
    }
    
    // Đảm bảo tần số lấy mẫu đúng (SAMPLE_FREQ Hz)
    unsigned long currentTime = millis();
    if (currentTime - lastSampleTime < (1000 / SAMPLE_FREQ)) {
        return;
    }
    lastSampleTime = currentTime;
    
    // Đọc giá trị từ MPU6050
    float rawGForce = calculateRawGForce();
    
    // Cập nhật bộ lọc
    updateFilter(rawGForce);
    
    // In giá trị G-force mỗi giây
    static unsigned long lastGforceTime = 0;
    if (currentTime - lastGforceTime > 1000 && DEBUG_MODE) {
        lastGforceTime = currentTime;
        DEBUG_PRINT("Raw G-force: ");
        DEBUG_PRINT(rawGForce);
        DEBUG_PRINT("G, Filtered: ");
        DEBUG_PRINT(filteredGForce);
        DEBUG_PRINT("G, Threshold: ");
        DEBUG_PRINT(crashThreshold);
        DEBUG_PRINTLN("G");
    }
    
    // Kiểm tra trạng thái báo động hiện tại
    if (crashAlarmActive) {
        // Nếu đang trong trạng thái báo động, kiểm tra thời gian cooldown
        if (currentTime - crashDetectedTime > crashCooldownTime) {
            // Hết thời gian cooldown, reset trạng thái
            crashAlarmActive = false;
            crashNotified = false;
            DEBUG_PRINTLN("Crash alarm reset - ready for new detection");
        }
        return; // Bỏ qua phát hiện va chạm khi đang trong trạng thái báo động
    }
    
    // Kiểm tra va chạm với thuật toán phát hiện đỉnh
    if (detectImpact()) {
        // Chỉ xử lý nếu không trong thời gian debounce
        if (currentTime - lastCrashTime > DEBOUNCE_TIME) {
            lastCrashTime = currentTime;
            crashDetectedTime = currentTime;
            crashAlarmActive = true;  // Chuyển sang trạng thái báo động
            
            DEBUG_PRINT("CRASH DETECTED! G-force: ");
            DEBUG_PRINTLN(filteredGForce);
            
            // Chỉ gửi thông báo BLE một lần
            if (deviceConnected && !crashNotified) {
                uint8_t crashValue = 1;
                pCrashChar->setValue(&crashValue, 1);
                pCrashChar->notify();
                crashNotified = true;
                
                // Reset crash value after notification
                delay(100);
                crashValue = 0;
                pCrashChar->setValue(&crashValue, 1);
            }
            
            // Sound emergency alarm - chỉ phát một lần
            for (int i = 0; i < 10; i++) {
                digitalWrite(BUZZER_PIN, HIGH);
                delay(150);
                digitalWrite(BUZZER_PIN, LOW);
                delay(100);
            }
        }
    }
}

void resetCrashState() {
    crashAlarmActive = false;
    crashNotified = false;
    DEBUG_PRINTLN("Crash state manually reset");
}

// Check reset button
void checkResetButton() {
    bool currentState = digitalRead(RESET_BUTTON_PIN);
    
    // Nút được nhấn (LOW do pull-up)
    if (currentState == LOW) {
        if (!buttonPressed) {
            buttonPressed = true;
            buttonPressTime = millis();
            DEBUG_PRINTLN("Reset button pressed");
        } else {
            // Kiểm tra thời gian giữ nút
            unsigned long holdTime = millis() - buttonPressTime;
            
            // Feedback theo thời gian giữ
            if (holdTime > 1000 && holdTime < 2000) {
                // Nhấp nháy LED chậm
                digitalWrite(LED_PIN, millis() % 500 < 250 ? HIGH : LOW);
            } else if (holdTime > 2000 && holdTime < BUTTON_HOLD_TIME) {
                // Nhấp nháy LED nhanh
                digitalWrite(LED_PIN, millis() % 200 < 100 ? HIGH : LOW);
            } else if (holdTime > BUTTON_HOLD_TIME && !resetInProgress) {
                // Thực hiện reset
                resetInProgress = true;
                DEBUG_PRINTLN("RESET TRIGGERED BY BUTTON!");
                resetAllSettings();
            }
        }
    } else {
        // Nút được thả
        if (buttonPressed) {
            unsigned long holdTime = millis() - buttonPressTime;
            DEBUG_PRINT("Button released after ");
            DEBUG_PRINT(holdTime);
            DEBUG_PRINTLN("ms");
            
            // Reset trạng thái
            buttonPressed = false;
            resetInProgress = false;
            digitalWrite(LED_PIN, deviceConnected ? HIGH : LOW);
        }
    }
}

// Setup BLE
void setupBLE() {
    String deviceName = "BikeGuard_1234";
    BLEDevice::init(deviceName.c_str());
    
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    // Crash detection characteristic
    pCrashChar = pService->createCharacteristic(
        CRASH_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pCrashChar->addDescriptor(new BLE2902());
    
    // Buzzer control characteristic
    pBuzzerChar = pService->createCharacteristic(
        BUZZER_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pBuzzerChar->setCallbacks(new BuzzerCallbacks());
    
    // Password management characteristic
    pPasswordChar = pService->createCharacteristic(
        PASSWORD_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE
    );
    pPasswordChar->setCallbacks(new PasswordCallbacks());
    
    // Device list characteristic
    pDeviceListChar = pService->createCharacteristic(
        DEVICE_LIST_UUID,
        BLECharacteristic::PROPERTY_READ
    );
    
    // Thêm characteristic cho độ nhạy
    pSensitivityChar = pService->createCharacteristic(
        SENSITIVITY_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE
    );
    pSensitivityChar->setCallbacks(new SensitivityCallbacks());

    pService->start();
    
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);
    BLEDevice::startAdvertising();
    
    DEBUG_PRINT("BLE Device name: ");
    DEBUG_PRINTLN(deviceName);
}

void setup() {
    // Chỉ khởi tạo Serial nếu DEBUG_MODE bật
    DEBUG_BEGIN(115200);
    DEBUG_PRINTLN("\n\n=== BIKE SAFETY SYSTEM STARTING ===");
    
    // Initialize pins
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
    
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    
    // Initialize device array
    initializeDevices();
    
    // Load saved settings
    loadSettings();
    
    // Khởi tạo thư viện bộ nhớ
    preferences.begin("bike-safety", false);
    // Đọc ngưỡng va chạm từ bộ nhớ
    crashThreshold = preferences.getFloat("crash_threshold", CRASH_THRESHOLD);
    preferences.end();
    
    // Setup components
    setupMPU6050();
    setupBLE();
    
    // Khởi tạo biến thời gian lấy mẫu
    lastSampleTime = millis();
    
    // Tự kiểm tra cảm biến
    float testGForce = calculateRawGForce();
    if (testGForce > 0.5f) {
        DEBUG_PRINT("MPU6050 self-test passed: ");
        DEBUG_PRINT(testGForce);
        DEBUG_PRINTLN("G");
        mpuInitialized = true;
    } else {
        DEBUG_PRINTLN("MPU6050 self-test failed. Retrying...");
        delay(500);
        setupMPU6050();
    }
    
    DEBUG_PRINTLN("=== BIKE SAFETY SYSTEM READY ===");
    DEBUG_PRINT("Current crash threshold: ");
    DEBUG_PRINT(crashThreshold);
    DEBUG_PRINTLN("G");
    
    DEBUG_PRINTLN("Hold reset button for 3 seconds to reset all settings");
    
    // Thông báo khởi động thành công
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        digitalWrite(BUZZER_PIN, HIGH);
        delay(50);
        digitalWrite(LED_PIN, LOW);
        digitalWrite(BUZZER_PIN, LOW);
        delay(50);
    }
}

void loop() {
    static unsigned long lastStatusTime = 0;
    static unsigned long lastMpuCheckTime = 0;
    
    // Check reset button
    checkResetButton();
    
    // Check for crashes
    checkForCrash();
    
    // Kiểm tra lại MPU định kỳ nếu có lỗi
    if (!mpuInitialized && (millis() - lastMpuCheckTime > 5000)) {
        lastMpuCheckTime = millis();
        setupMPU6050();
    }
    
    // Update device list characteristic when connected
    if (deviceConnected && clientAuthenticated) {
        // Chuẩn bị chuỗi chứa danh sách thiết bị
        String deviceList;
        deviceList.reserve(deviceCount * 50); // Dự tính dung lượng
        
        for (int i = 0; i < deviceCount; i++) {
            if (allowedDevices[i].active) {
                deviceList += allowedDevices[i].deviceIdentifier + ";";
            }
        }
        pDeviceListChar->setValue(deviceList.c_str());
        
        // Gửi giá trị ngưỡng va chạm hiện tại
        char sensitivityStr[10];
        sprintf(sensitivityStr, "%.1f", crashThreshold);
        pSensitivityChar->setValue(sensitivityStr);
    }
    
    // Restart advertising if needed
    static bool oldDeviceConnected = false;
    if (!deviceConnected && oldDeviceConnected) {
        delay(500);
        pServer->startAdvertising();
    }
    oldDeviceConnected = deviceConnected;
    
    // In trạng thái chính mỗi 30 giây
    if (millis() - lastStatusTime > 30000 && DEBUG_MODE) {
        lastStatusTime = millis();
        
        // Tính giá trị trung bình, min, max
        float minVal = 100.0f;
        float maxVal = 0.0f;
        float avgVal = 0.0f;
        
        for (int i = 0; i < NUM_SAMPLES; i++) {
            if (accelReadings[i] < minVal) minVal = accelReadings[i];
            if (accelReadings[i] > maxVal) maxVal = accelReadings[i];
            avgVal += accelReadings[i];
        }
        avgVal /= NUM_SAMPLES;
        
        DEBUG_PRINTLN("\n=== System Status ===");
        DEBUG_PRINT("Connected: ");
        DEBUG_PRINTLN(deviceConnected ? "Yes" : "No");
        DEBUG_PRINT("Authenticated: ");
        DEBUG_PRINTLN(clientAuthenticated ? "Yes" : "No");
        DEBUG_PRINT("Device count: ");
        DEBUG_PRINTLN(deviceCount);
        DEBUG_PRINT("Current crash threshold: ");
        DEBUG_PRINT(crashThreshold);
        DEBUG_PRINTLN("G");
        DEBUG_PRINT("Filtered G-force: ");
        DEBUG_PRINT(filteredGForce);
        DEBUG_PRINTLN("G");
        DEBUG_PRINT("Min/Avg/Max: ");
        DEBUG_PRINT(minVal);
        DEBUG_PRINT("/");
        DEBUG_PRINT(avgVal);
        DEBUG_PRINT("/");
        DEBUG_PRINT(maxVal);
        DEBUG_PRINTLN("G");
        DEBUG_PRINT("MPU initialized: ");
        DEBUG_PRINTLN(mpuInitialized ? "Yes" : "No");
        
        // Check battery voltage if available
        #ifdef BATTERY_PIN
            float batteryVoltage = analogRead(BATTERY_PIN) * 3.3f * 2 / 4095.0f;
            DEBUG_PRINT("Battery: ");
            DEBUG_PRINT(batteryVoltage);
            DEBUG_PRINTLN("V");
        #endif
        
        DEBUG_PRINTLN("==================\n");
    }
    
    // Cập nhật thường xuyên với tốc độ phù hợp
    delay(10); // Giới hạn tốc độ lặp chính - giúp tiết kiệm pin
}