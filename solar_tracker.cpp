/*
 * ================================================================
 * DUAL AXIS SOLAR TRACKER - ESP32 CONTROLLER
 * ================================================================
 * Thuật toán: 
 * - Lọc nhiễu ADC: Exponential Moving Average (EMA)
 * - Hiệu chuẩn hệ số khuếch đại 4 LDR khi khởi động
 * - Sai số chuẩn hóa theo tổng cường độ sáng (không phụ thuộc trời nắng/râm)
 * - Điều khiển: PID dạng vận tốc (u = tốc độ quay °/s), mặc định Ki = 0 (PD)
 * - Chống rung: Deadband + lọc thông thấp khâu D (không giật khi qua deadband)
 * - Giới hạn tốc độ xoay servo (chống giật, chống sụt áp nguồn USB)
 * - Điều khiển Servo: Hardware LEDC PWM (50Hz, 13-bit)
 * ================================================================
 */

#include <Arduino.h>

// ================================================================
// 1. CẤU HÌNH CHÂN GPIO & NGOẠI VI
// ================================================================
// Cảm biến quang trở LDR (Các chân thuộc ADC1)
#define PIN_LDR_TL     34  // Top-Left (Trên - Trái)
#define PIN_LDR_TR     35  // Top-Right (Trên - Phải)
#define PIN_LDR_BL     32  // Bottom-Left (Dưới - Trái)
#define PIN_LDR_BR     33  // Bottom-Right (Dưới - Phải)

// Động cơ Servo (LEDC PWM)
#define PIN_SERVO_PAN  18  // Trục Ngang (Pan / Azimuth)
#define PIN_SERVO_TILT 19  // Trục Dọc (Tilt / Elevation)

#define CH_SERVO_PAN   0   // Kênh LEDC (dùng cho core v2.x)
#define CH_SERVO_TILT  1

// Tham số PWM LEDC
#define PWM_FREQ       50      // Tần số PWM tiêu chuẩn cho Servo (50Hz ~ Chu kỳ 20ms)
#define PWM_RES        13      // Độ phân giải 13-bit (Giá trị Duty Cycle từ 0 -> 8191)

// Core v2.x: ledcWrite() nhận số kênh; core v3.x: nhận số chân
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
#define SERVO_PAN_ID   PIN_SERVO_PAN
#define SERVO_TILT_ID  PIN_SERVO_TILT
#else
#define SERVO_PAN_ID   CH_SERVO_PAN
#define SERVO_TILT_ID  CH_SERVO_TILT
#endif

const float PWM_PERIOD_US = 1000000.0f / PWM_FREQ;   // 20000 us
const float PWM_DUTY_FULL = (float)(1UL << PWM_RES); // 8192 bước / chu kỳ

// ================================================================
// 2. CẤU HÌNH THAM SỐ THUẬT TOÁN
// ================================================================
// Hệ số lọc EMA (0.0 < ALPHA <= 1.0)
const float EMA_ALPHA = 0.15f;

// Giới hạn góc quay an toàn cho Servo (tránh kẹt cơ khí)
const float ANGLE_MIN = 10.0f;
const float ANGLE_MAX = 170.0f;

// Vị trí mặc định ban đầu (Home Position)
const float HOME_PAN  = 90.0f;
const float HOME_TILT = 40.0f;

// Độ rộng xung SG90/MG90S: 0° -> 500us, 180° -> 2500us
const float SERVO_MIN_US = 500.0f;
const float SERVO_MAX_US = 2500.0f;

// Sai số chuẩn hóa nằm trong [-1, 1]; |e| < 3% coi như đã hướng đúng nguồn sáng
const float DEADBAND = 0.03f;

// Tổng ADC 4 LDR dưới ngưỡng này -> thiếu sáng, giữ nguyên vị trí
const float MIN_LIGHT_SUM = 400.0f;

// SG90 dùng chung nguồn USB với ESP32 -> giữ tốc độ thấp để tránh brownout
const float MAX_SPEED_DEG_S = 40.0f;

// Số mẫu ADC dùng khi hiệu chuẩn LDR
const int CALIB_SAMPLES = 64;

// Đầu ra PID là tốc độ quay (°/s), góc servo = tích phân tốc độ.
// => PD ở đây tương đương PI theo vị trí; bật thêm Ki sẽ thành tích phân kép, dễ dao động.
struct PIDController {
    float Kp;        // (°/s) trên 1 đơn vị sai số chuẩn hóa
    float Ki;
    float Kd;
    float integral;
    float prevError;
    float dFiltered;
};

PIDController pidPan  = {120.0f, 0.0f, 4.0f, 0.0f, 0.0f, 0.0f};
PIDController pidTilt = {120.0f, 0.0f, 4.0f, 0.0f, 0.0f, 0.0f};

const float D_FILTER_ALPHA = 0.3f;  // Lọc thông thấp khâu D
const float INTEGRAL_LIMIT = 1.0f;  // Chống bão hòa tích phân (khi bật Ki)

// ================================================================
// 3. BIẾN TOÀN CỤC & TRẠNG THÁI HỆ THỐNG
// ================================================================
enum { LDR_TL, LDR_TR, LDR_BL, LDR_BR, NUM_LDR };
const uint8_t LDR_PINS[NUM_LDR] = {PIN_LDR_TL, PIN_LDR_TR, PIN_LDR_BL, PIN_LDR_BR};

float ldrGain[NUM_LDR] = {1.0f, 1.0f, 1.0f, 1.0f}; // Hệ số hiệu chuẩn
float ema[NUM_LDR]     = {0.0f, 0.0f, 0.0f, 0.0f}; // Giá trị LDR sau lọc EMA

// Trạng thái góc quay hiện tại của Servo
float currentPanAngle  = HOME_PAN;
float currentTiltAngle = HOME_TILT;

// Timing điều khiển loop
unsigned long lastLoopTime = 0;
const unsigned long LOOP_INTERVAL_MS = 20; // Chu kỳ lấy mẫu và điều khiển: 20ms (50Hz)

// ================================================================
// 4. XUẤT XUNG PWM CHO SERVO
// ================================================================
void servoWrite(uint8_t id, float angle) {
    angle = constrain(angle, ANGLE_MIN, ANGLE_MAX);
    float pulseUs = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * angle / 180.0f;
    ledcWrite(id, (uint32_t)(pulseUs / PWM_PERIOD_US * PWM_DUTY_FULL));
}

// ================================================================
// 5. ĐỌC CẢM BIẾN, LỌC EMA & HIỆU CHUẨN LDR
// ================================================================
float readLdr(int i) {
    return analogRead(LDR_PINS[i]) * ldrGain[i];
}

void readAndFilterSensors() {
    // EMA: S_t = alpha * X_t + (1 - alpha) * S_(t-1)
    for (int i = 0; i < NUM_LDR; i++) {
        ema[i] = EMA_ALPHA * readLdr(i) + (1.0f - EMA_ALPHA) * ema[i];
    }
}

// Yêu cầu: 4 LDR được chiếu sáng đều trong lúc hiệu chuẩn
void calibrateSensors() {
    float sum[NUM_LDR] = {0.0f, 0.0f, 0.0f, 0.0f};
    for (int n = 0; n < CALIB_SAMPLES; n++) {
        for (int i = 0; i < NUM_LDR; i++) {
            sum[i] += analogRead(LDR_PINS[i]);
        }
        delay(5);
    }

    float mean = (sum[LDR_TL] + sum[LDR_TR] + sum[LDR_BL] + sum[LDR_BR]) / NUM_LDR;
    if (mean / CALIB_SAMPLES < MIN_LIGHT_SUM / NUM_LDR) {
        Serial.println("Calibration skipped: not enough light (gain = 1.0)");
        return;
    }

    // Kẹp 0.5 - 2.0 để tránh hệ số vô lý khi 1 LDR bị che/hỏng
    for (int i = 0; i < NUM_LDR; i++) {
        ldrGain[i] = (sum[i] > 0.0f) ? constrain(mean / sum[i], 0.5f, 2.0f) : 1.0f;
    }
    Serial.printf("LDR gain [TL:%.2f TR:%.2f BL:%.2f BR:%.2f]\n",
                  ldrGain[LDR_TL], ldrGain[LDR_TR], ldrGain[LDR_BL], ldrGain[LDR_BR]);
}

// ================================================================
// 6. BỘ ĐIỀU KHIỂN PID DẠNG VẬN TỐC KẾT HỢP DEADBAND
// ================================================================
// Trả về tốc độ quay mong muốn (°/s)
float computePID(PIDController &pid, float error, float dt) {
    // D tính trên sai số thật (trước deadband) + lọc thông thấp -> không giật khi vào/ra deadband
    float derivative = (error - pid.prevError) / dt;
    pid.prevError = error;
    pid.dFiltered += D_FILTER_ALPHA * (derivative - pid.dFiltered);

    if (fabsf(error) < DEADBAND) {
        pid.integral = 0.0f;
        return 0.0f;
    }

    pid.integral = constrain(pid.integral + error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    return pid.Kp * error + pid.Ki * pid.integral + pid.Kd * pid.dFiltered;
}

// ================================================================
// 7. SETUP (KHỞI TẠO HỆ THỐNG)
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n--- ESP32 DUAL AXIS SOLAR TRACKER INITIALIZING ---");

    // ADC 12-bit, suy giảm 11dB để đo đủ dải 0 - 3.3V
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // Khởi tạo ngoại vi LEDC PWM cho Servo (Tương thích ESP32 Core v2.x & v3.x)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    // Dành cho ESP32 Arduino Core v3.x
    ledcAttach(PIN_SERVO_PAN, PWM_FREQ, PWM_RES);
    ledcAttach(PIN_SERVO_TILT, PWM_FREQ, PWM_RES);
#else
    // Dành cho ESP32 Arduino Core v2.x
    ledcSetup(CH_SERVO_PAN, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_SERVO_PAN, CH_SERVO_PAN);
    ledcSetup(CH_SERVO_TILT, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_SERVO_TILT, CH_SERVO_TILT);
#endif

    // Đưa lần lượt từng servo về Home, tránh 2 servo cùng kéo dòng gây sụt áp
    currentPanAngle  = HOME_PAN;
    currentTiltAngle = HOME_TILT;
    servoWrite(SERVO_PAN_ID, currentPanAngle);
    delay(400);
    servoWrite(SERVO_TILT_ID, currentTiltAngle);
    delay(400);

    Serial.println("Calibrating LDRs: keep all 4 sensors evenly lit...");
    delay(2000);
    calibrateSensors();

    // Khởi tạo giá trị ban đầu cho bộ lọc EMA
    for (int i = 0; i < NUM_LDR; i++) {
        ema[i] = readLdr(i);
    }

    Serial.println("System Ready! Starting PID Tracking Loop...");
    lastLoopTime = millis();
}

// ================================================================
// 8. MAIN LOOP (VÒNG LẶP CHÍNH)
// ================================================================
void loop() {
    unsigned long now = millis();
    if (now - lastLoopTime < LOOP_INTERVAL_MS) {
        return; // Đảm bảo chu kỳ lấy mẫu cố định đúng 20ms
    }

    float dt = (now - lastLoopTime) / 1000.0f; // Đổi chu kỳ ra giây (seconds)
    lastLoopTime = now;

    // STEP 1: Đọc ADC và Lọc nhiễu EMA
    readAndFilterSensors();

    // STEP 2: Tính sai số chuẩn hóa
    // Sơ đồ mạch: VCC -> LDR -> (Nút ADC) -> R_10k -> GND (Sáng cao -> ADC cao)
    float left   = ema[LDR_TL] + ema[LDR_BL];
    float right  = ema[LDR_TR] + ema[LDR_BR];
    float top    = ema[LDR_TL] + ema[LDR_TR];
    float bottom = ema[LDR_BL] + ema[LDR_BR];
    float total  = left + right;

    float errorPan = 0.0f, errorTilt = 0.0f;
    if (total >= MIN_LIGHT_SUM) {
        // e_h > 0: bên Trái sáng hơn; e_v > 0: bên Trên sáng hơn
        errorPan  = (left - right) / total;
        errorTilt = (top - bottom) / total;
    }

    // STEP 3: PID -> tốc độ quay mong muốn, giới hạn tốc độ tối đa
    float panSpeed  = computePID(pidPan, errorPan, dt);
    float tiltSpeed = computePID(pidTilt, errorTilt, dt);
    panSpeed  = constrain(panSpeed, -MAX_SPEED_DEG_S, MAX_SPEED_DEG_S);
    tiltSpeed = constrain(tiltSpeed, -MAX_SPEED_DEG_S, MAX_SPEED_DEG_S);

    // STEP 4: Cập nhật góc và kẹp trong dải 10° - 170°
    // Chiều +/- phụ thuộc lắp đặt cơ khí: e_h > 0 -> giảm Pan (sang Trái), e_v > 0 -> tăng Tilt (lên Trên)
    currentPanAngle  = constrain(currentPanAngle - panSpeed * dt, ANGLE_MIN, ANGLE_MAX);
    currentTiltAngle = constrain(currentTiltAngle + tiltSpeed * dt, ANGLE_MIN, ANGLE_MAX);

    // STEP 5: Xuất xung PWM LEDC ra Servo
    servoWrite(SERVO_PAN_ID, currentPanAngle);
    servoWrite(SERVO_TILT_ID, currentTiltAngle);

    // STEP 6: In thông số kiểm tra lên Serial Monitor (Định kỳ mỗi 200ms)
    static unsigned long lastPrintTime = 0;
    if (now - lastPrintTime > 200) {
        lastPrintTime = now;
        Serial.printf("LDR [TL:%.0f TR:%.0f BL:%.0f BR:%.0f] | ErrH:%+.1f%% ErrV:%+.1f%% | Servo [Pan:%.1f Tilt:%.1f]\n",
                      ema[LDR_TL], ema[LDR_TR], ema[LDR_BL], ema[LDR_BR],
                      errorPan * 100.0f, errorTilt * 100.0f, currentPanAngle, currentTiltAngle);
    }
}