#include "dsp_manager.h"
#include "imu.h"

// Буферы для расчета
static float32_t fft_input[FFT_SIZE];
static float32_t fft_output[FFT_SIZE];
static float32_t magnitudes[FFT_SIZE / 2];

// Буфер для окна Ханна (чтобы убрать шумы по краям выборки)
static float32_t hann_window[FFT_SIZE];

static uint16_t sample_count = 0;
uint8_t dsp_buffer_ready = 0;

// Структура БПФ из библиотеки
static arm_rfft_fast_instance_f32 fft_handler;

void DSP_Init(void) {
    // Инициализируем структуру БПФ
    arm_rfft_fast_init_f32(&fft_handler, FFT_SIZE);
    
    // Генерируем окно Ханна (делается один раз)
    for (int i = 0; i < FFT_SIZE; i++) {
        hann_window[i] = 0.5f * (1.0f - arm_cos_f32(2.0f * 3.14159f * i / (1023.0f)));
    }
}

void DSP_AddSample(float32_t sample) {
    if (dsp_buffer_ready) return; // Ждем, пока обработают прошлую пачку

    fft_input[sample_count++] = sample;

    if (sample_count >= FFT_SIZE) {
        sample_count = 0;
        dsp_buffer_ready = 1; // Сигнализируем в main
    }
}

void DSP_Process(void) {
    // 1. Применяем окно Ханна
    arm_mult_f32(fft_input, hann_window, fft_input, FFT_SIZE);

    // 2. САМО БПФ
    arm_rfft_fast_f32(&fft_handler, fft_input, fft_output, 0);

    // 3. Считаем амплитуды
    arm_cmplx_mag_f32(fft_output, magnitudes, FFT_SIZE / 2);

    // 4. Поиск 3-х самых мощных пиков
    float32_t top_freq_indices[3] = {0};
    float32_t top_mags[3] = {0};

    for (uint32_t i = 50; i < 450; i++) {
        if (magnitudes[i] > top_mags[0]) {
            top_mags[2] = top_mags[1]; top_freq_indices[2] = top_freq_indices[1];
            top_mags[1] = top_mags[0]; top_freq_indices[1] = top_freq_indices[0];
            top_mags[0] = magnitudes[i];
            top_freq_indices[0] = (float32_t)i;
        }
    }

    // --- 5. ПЕРЕНАСТРОЙКА ТРЕХ КАСКАДОВ FMAC ---
    const float fs = 1000.0f;     // Частота дискретизации
    const float Q = 1.5f;        // Добротность (ширина выреза, 1.0 - 2.0 норм)
    const float bin_to_hz = fs / (float)FFT_SIZE; 

    for (int i = 0; i < 3; i++) {
        // Если амплитуда выше порога, настраиваем фильтр
        if (top_mags[i] > 3.0f) {
            float real_freq = top_freq_indices[i] * bin_to_hz;
            
            // Математика Notch-фильтра
            float w0 = 2.0f * 3.14159265f * real_freq / fs;
            float alpha = arm_sin_f32(w0) / (2.0f * Q);
            float cosw0 = arm_cos_f32(w0);
            float a0 = 1.0f + alpha;

            // Коэффициенты для передачи в FMAC
            // Мы делим на a0 сразу здесь
            float b0 = 1.0f / a0;
            float b1 = -2.0f * cosw0 / a0;
            float b2 = 1.0f / a0;
            float a1 = -2.0f * cosw0 / a0;
            float a2 = (1.0f - alpha) / a0;

            Update_FMAC_Coeffs(i, b0, b1, b2, a1, a2);
        } 
        else {
            // Если пика нет, ставим фильтр в режим Bypass (пропускает сигнал без изменений)
            // b0 = 1.0, остальные 0. Это даст y[n] = 1.0 * x[n]
            Update_FMAC_Coeffs(i, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        }
    }

    dsp_buffer_ready = 0; // Разрешаем новый сбор данных
}