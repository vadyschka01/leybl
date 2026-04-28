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
        hann_window[i] = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / (FFT_SIZE - 1)));
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
    // 1. Применяем окно Ханна (умножаем входные данные на "колокол")
    arm_mult_f32(fft_input, hann_window, fft_input, FFT_SIZE);

    // 2. САМО БПФ (Быстрое преобразование Фурье)
    arm_rfft_fast_f32(&fft_handler, fft_input, fft_output, 0);

    // 3. Считаем амплитуды (Magnitudes)
    arm_cmplx_mag_f32(fft_output, magnitudes, FFT_SIZE / 2);

    // 4. Поиск 3-х самых мощных пиков
    float32_t top_freqs[3] = {0};
    float32_t top_mags[3] = {0};

    // Ищем в диапазоне от 50 Гц до 450 Гц (чтобы не задеть полезный сигнал наклона)
    // Т.к. частота опроса 1000 Гц, а точек 1024, индекс массива почти равен частоте в Гц
    for (uint32_t i = 50; i < 450; i++) {
        if (magnitudes[i] > top_mags[0]) {
            // Сдвигаем старые значения
            top_mags[2] = top_mags[1]; top_freqs[2] = top_freqs[1];
            top_mags[1] = top_mags[0]; top_freqs[1] = top_freqs[0];
            // Записываем новый топ-1
            top_mags[0] = magnitudes[i];
            top_freqs[0] = (float32_t)i;
        }
    }

    // 5. ПЕРЕНАСТРОЙКА ФИЛЬТРОВ в imu.c "на лету"
    //  динамически меняем частоты notch1, notch2, notch3
    if (top_mags[0] > 10.0f) biquad_init_notch(&notch1, top_freqs[0], 1.0f, 1000.0f);
    if (top_mags[1] > 10.0f) biquad_init_notch(&notch2, top_freqs[1], 1.0f, 1000.0f);
    if (top_mags[2] > 10.0f) biquad_init_notch(&notch3, top_freqs[2], 1.0f, 1000.0f);

    dsp_buffer_ready = 0; // Разрешаем новый сбор данных
}