#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_BMP085.h>
#include <BH1750.h>

// --- Khai báo phần cứng ---
#define MQ135_PIN 6 
#define I2C_SDA  8
#define I2C_SCL  9

// --- Thông số cho MQ135 (Cần chỉnh lại RZERO khi hiệu chuẩn thực tế) ---
#define RLOAD 10.0    // Điện trở tải trên board MQ135 (thường là 10kOhm)
#define RZERO 76.63   // Điện trở R0 trong môi trường sạch

// Thông số tính nồng độ khí ở thang ppm
#define PARA 116.6020682
#define PARB 2.769034857

// Thông số tính hệ số bù trừ
#define CORA .00035
#define CORB .02718
#define CORC 1.39538
#define CORD .0018
#define CORE -.003333333
#define CORF -.001923077
#define CORG 1.130128205
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_BMP085 bmp;
BH1750 lightMeter;

unsigned long lastMillis = 0;
const long interval = 2000; 

// ==========================================
// CÁC BIẾN TRẠNG THÁI CHO BỘ LỌC
// ==========================================
bool isFirstRead = true; // Dùng để khởi tạo giá trị nền ở lần chạy đầu tiên

// 1. Biến cho EMA (Nhiệt độ, Độ ẩm, Khí Gas tầng 2)
const float EMA_ALPHA = 0.2;
float ema_temp = 0;
float ema_hum = 0;
float ema_mq135 = 0;

// 2. Biến cho Kalman 1D (Áp suất)
const float KALMAN_Q = 0.15;
const float KALMAN_R = 81.0; // Phương sai nhiễu đo lường (lấy từ đánh giá biểu đồ trước đó)
float kalman_p = 1.0;        // Sai số ước lượng ban đầu
float kalman_x = 0;          // Giá trị áp suất ước lượng

// 3. Biến cho Median Cửa sổ trượt (Ánh sáng, Khí Gas tầng 1)
const int MEDIAN_N = 7;
float lux_buffer[MEDIAN_N];
float mq_buffer[MEDIAN_N];
int lux_idx = 0;
int mq_idx = 0;

// ==========================================
// CÁC HÀM XỬ LÝ LỌC NHIỄU TỐI ƯU
// ==========================================

// Hàm Lọc Median (Cửa sổ trượt, mảng N=7)
float applyRunningMedian(float newValue, float* buffer, int& index) {
    // 1. Cập nhật giá trị mới vào bộ đệm xoay vòng
    buffer[index] = newValue;
    index = (index + 1) % MEDIAN_N;

    // 2. Copy ra mảng tạm để thuật toán sắp xếp không làm hỏng thứ tự bộ đệm
    float temp[MEDIAN_N];
    for (int i = 0; i < MEDIAN_N; i++) {
        temp[i] = buffer[i];
    }

    // 3. Sắp xếp nổi bọt (Bubble sort - Rất nhẹ với mảng 7 phần tử)
    for (int i = 0; i < MEDIAN_N - 1; i++) {
        for (int j = i + 1; j < MEDIAN_N; j++) {
            if (temp[j] < temp[i]) {
                float t = temp[i];
                temp[i] = temp[j];
                temp[j] = t;
            }
        }
    }
    // Trả về giá trị đứng giữa mảng (vị trí số 3 trong mảng 0->6)
    return temp[3]; 
}

// Hàm Lọc Kalman 1D
float applyKalman(float measurement) {
    // Bước dự đoán (Predict)
    float p_pred = kalman_p + KALMAN_Q;
    
    // Bước cập nhật (Update)
    float K = p_pred / (p_pred + KALMAN_R); // Tính hệ số Kalman Gain
    kalman_x = kalman_x + K * (measurement - kalman_x);
    kalman_p = (1.0 - K) * p_pred;
    
    return kalman_x;
}

void setup() {
    Serial.begin(115200);
    // Cấu hình phân giải ADC của ESP32-S3 (12-bit)
    analogReadResolution(12);

    Wire.begin(I2C_SDA, I2C_SCL); 
    Wire.setClock(100000); 
    Wire.setTimeOut(50); 

    Serial.println("\n--- KHỞI ĐỘNG HỆ THỐNG CẢM BIẾN (CHẾ ĐỘ ỔN ĐỊNH) ---");

    if (!sht31.begin(0x44)) Serial.println("[LỖI] Không tìm thấy SHT31");
    if (!bmp.begin()) Serial.println("[LỖI] Không tìm thấy BMP180");
    if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
        Serial.println("[LỖI] Không tìm thấy BH1750");
    }

    Serial.println("Hệ thống sẵn sàng. Đang lấy mẫu (2s/lần)...\n");
}

void loop() {
    if (millis() - lastMillis >= interval) {
        lastMillis = millis();

        // --- 1. ĐỌC TÍN HIỆU GỐC TỪ CẢM BIẾN ---
        float raw_temp = sht31.readTemperature();
        float raw_hum = sht31.readHumidity();
        float raw_lux = lightMeter.readLightLevel();
        float raw_pressure = bmp.readPressure();
        float raw_mq135 = analogRead(MQ135_PIN);

        // --- KIỂM TRA LỖI & TỰ ĐỘNG PHỤC HỒI ---
        if (isnan(raw_temp) || isnan(raw_hum) || raw_pressure <= 0) {
            Serial.println("--> [CẢNH BÁO] Phát hiện nhiễu I2C, đang tự động khôi phục...");
            Wire.end(); 
            delay(10);
            Wire.begin(I2C_SDA, I2C_SCL); 
            Wire.setClock(100000);
            Wire.setTimeOut(50);
            delay(50); 
            
            sht31.begin(0x44);
            bmp.begin();
            lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
            return; // Bỏ qua chu kỳ này để tránh tính toán sai số
        } 

        // --- 2. KHỞI TẠO BỘ LỌC (Chỉ chạy 1 lần) ---
        if (isFirstRead) {
            ema_temp = raw_temp;
            ema_hum = raw_hum;
            ema_mq135 = raw_mq135;
            kalman_x = raw_pressure;
            for(int i=0; i<MEDIAN_N; i++) {
                lux_buffer[i] = raw_lux;
                mq_buffer[i] = raw_mq135;
            }
            isFirstRead = false;
        }

        // --- 3. ÁP DỤNG CÁC BỘ LỌC ĐÃ CHỌN ---
        // SHT31 (EMA)
        ema_temp = (EMA_ALPHA * raw_temp) + ((1.0 - EMA_ALPHA) * ema_temp);
        ema_hum  = (EMA_ALPHA * raw_hum) + ((1.0 - EMA_ALPHA) * ema_hum);

        // BMP180 (Kalman 1D)
        float filtered_pressure = applyKalman(raw_pressure);

        // BH1750 (Median)
        float filtered_lux = applyRunningMedian(raw_lux, lux_buffer, lux_idx);

        // MQ135 (Cascade: Median -> EMA)
        float median_mq135 = applyRunningMedian(raw_mq135, mq_buffer, mq_idx);
        ema_mq135 = (EMA_ALPHA * median_mq135) + ((1.0 - EMA_ALPHA) * ema_mq135);

        // --- 4. HIỆU CHỈNH & TÍNH TOÁN NỒNG ĐỘ MQ135 (PPM) ---
        // Chuyển giá trị ADC 12-bit đã lọc mịn thành Điện trở Rs
        float rs = ((4095.0 / ema_mq135) - 1.0) * RLOAD;

        // Tính hệ số bù trừ dựa trên Nhiệt độ và Độ ẩm thực tế (Đã qua lọc EMA)
        float correction_factor = CORE * ema_temp + CORF * ema_hum + CORG;

        // Tính Rs thực tế
        float rs_corrected = rs / correction_factor;
        
        //Tính tỉ số Rs/Ro
        float ratio = rs_corrected / RZERO;
        
        // Công thức nội suy nồng độ (CO2)
        float ppm_mq135 = PARA * pow(ratio, -PARB);

        // --- 5. TẠO FILE JSON CHUẨN ĐẦU RA (BAO GỒM RAW VÀ FILTERED) ---
        Serial.print("{");

        // Nhiệt độ
        Serial.print("\"temp_raw\":"); Serial.print(raw_temp, 2);
        Serial.print(",\"temp_filtered\":"); Serial.print(ema_temp, 2);

        // Độ ẩm
        Serial.print(",\"hum_raw\":"); Serial.print(raw_hum, 2);
        Serial.print(",\"hum_filtered\":"); Serial.print(ema_hum, 2);

        // Ánh sáng
        Serial.print(",\"lux_raw\":"); Serial.print(raw_lux, 2);
        Serial.print(",\"lux_filtered\":"); Serial.print(filtered_lux, 2);

        // Áp suất
        Serial.print(",\"pressure_raw\":"); Serial.print(raw_pressure, 2);
        Serial.print(",\"pressure_filtered\":"); Serial.print(filtered_pressure, 2);

        // Khí gas (MQ135)
        Serial.print(",\"mq135_raw_adc\":"); Serial.print(raw_mq135, 0); // Raw ADC là số nguyên
        Serial.print(",\"mq135_ppm\":"); Serial.print(ppm_mq135, 2);    // Nồng độ sau hiệu chuẩn

        // Timestamp
        Serial.print(",\"timestamp\":"); Serial.print(millis());

        Serial.println("}");
    }
}
