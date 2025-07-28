#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "ff.h"
#include "diskio.h"
#include "f_util.h"
#include "hw_config.h"
#include "my_debug.h"
#include "rtc.h"
#include "sd_card.h"
#include <string.h>

#define LED_PIN_RED 13
#define LED_PIN_GREEN 11
#define LED_PIN_BLUE 12
#define BUTTON_A 5
#define BUTTON_B 6
#define BUTTON_J 22
#define BUZZER_PIN_A 21


#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1
static int addr = 0x68;


#define I2C_PORT_DISP i2c1
#define I2C_SDA_DISP 14
#define I2C_SCL_DISP 15
#define DISPLAY_ADDRESS 0x3C

#define DEBOUNCE_TIME 300
#define BUZZER_FREQUENCY 4000/// Frequência do buzzer em Hz

void gpio_irq_handler(uint gpio, uint32_t events);

// Função para inicializar o buzzer
void pwm_init_buzzer(uint pin) {
    // Configurar o pino como saída de PWM
    gpio_set_function(pin, GPIO_FUNC_PWM);

    // Obter o slice do PWM associado ao pino
    uint slice_num = pwm_gpio_to_slice_num(pin);

    // Configurar o PWM com frequência desejada
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, clock_get_hz(clk_sys) / (BUZZER_FREQUENCY * 4096)); // Divisor de clock
    pwm_init(slice_num, &config, true);

    // Iniciar o PWM no nível baixo
    pwm_set_gpio_level(pin, 0);
}

// Função para emitir um beep com duração especificada
void beep(uint pin, uint duration_ms) {
    // Obter o slice do PWM associado ao pino
    uint slice_num = pwm_gpio_to_slice_num(pin);

    // Configurar o duty cycle para 50% (ativo)
    pwm_set_gpio_level(pin, 2048);

    // Temporização
    sleep_ms(duration_ms);

    // Desativar o sinal PWM (duty cycle 0)
    pwm_set_gpio_level(pin, 0);
}

static sd_card_t *sd_get_by_name(const char *const name)
{
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return sd_get_by_num(i);
    DBG_PRINTF("%s: unknown name %s\n", __func__, name);
    return NULL;
}
static FATFS *sd_get_fs_by_name(const char *name)
{
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return &sd_get_by_num(i)->fatfs;
    DBG_PRINTF("%s: unknown name %s\n", __func__, name);
    return NULL;
}
static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp)
{
    uint8_t buffer[6];
    uint8_t val = 0x3B;
    i2c_write_blocking(I2C_PORT, addr, &val, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buffer, 6, false);
    for (int i = 0; i < 3; i++)
        accel[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];

    val = 0x43;
    i2c_write_blocking(I2C_PORT, addr, &val, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buffer, 6, false);
    for (int i = 0; i < 3; i++)
        gyro[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];

    val = 0x41;
    i2c_write_blocking(I2C_PORT, addr, &val, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buffer, 2, false);
    *temp = (buffer[0] << 8) | buffer[1];
}


static void run_mount()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }
    FRESULT fr = f_mount(p_fs, arg1, 1);
    if (FR_OK != fr)
    {
        printf("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    sd_card_t *pSD = sd_get_by_name(arg1);
    myASSERT(pSD);
    pSD->mounted = true;
    printf("Processo de montagem do SD ( %s ) concluído\n", pSD->pcName);
}

void capture_imu_data_and_save(const char *filename, int num_samples)
{
    FIL file;
    FRESULT res;

    // 1. Abre o arquivo para escrita, criando-o se não existir.
    res = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK)
    {
        printf("ERRO: Nao foi possivel abrir o arquivo %s\n", filename);
        return;
    }

    // 2. Escreve o cabeçalho no arquivo .csv, conforme o requisito.
    const char *header = "numero_amostra,accel_x,accel_y,accel_z,giro_x,giro_y,giro_z\n";
    UINT bytes_written;
    f_write(&file, header, strlen(header), &bytes_written);

    // 3. Loop para capturar e salvar o número de amostras definido.
    for (int i = 0; i < num_samples; i++)
    {
        int16_t aceleracao[3], gyro[3], temp;

        // Lê os dados brutos do sensor MPU6050.
        mpu6050_read_raw(aceleracao, gyro, &temp);

        // Converte os dados brutos para unidades físicas (opcional, mas recomendado).
        // A sensibilidade padrão é 16384 para aceleração e 131 para giroscópio.
        float ax = aceleracao[0] / 16384.0f;
        float ay = aceleracao[1] / 16384.0f;
        float az = aceleracao[2] / 16384.0f;
        float gx = gyro[0] / 131.0f;
        float gy = gyro[1] / 131.0f;
        float gz = gyro[2] / 131.0f;

        // 4. Formata a string com todos os dados no formato CSV.
        // Aumentamos o buffer para garantir que todos os dados caibam.
        char buffer[128];
        int len = snprintf(buffer, sizeof(buffer), "%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                           i + 1, ax, ay, az, gx, gy, gz);

        // 5. Escreve a linha formatada no arquivo.
        res = f_write(&file, buffer, len, &bytes_written);
        if (res != FR_OK)
        {
            printf("ERRO: Falha ao escrever no arquivo na amostra %d\n", i + 1);
            break; // Sai do loop em caso de erro.
        }

        // Aguarda um tempo entre as amostras (ex: 100ms para 10Hz).
        sleep_ms(100);
    }

    // 6. Fecha o arquivo para garantir que todos os dados sejam salvos.
    f_close(&file);
    printf("Dados salvos com sucesso em %s\n", filename);
}

bool mount_sd_card() {
    // Pega o caminho do primeiro cartão, ex: "0:"
    const char *drive_path = sd_get_by_num(0)->pcName;
    if (!drive_path) {
        printf("ERRO: Nao foi possivel encontrar o caminho do SD card.\n");
        return false;
    }

    // Pega o ponteiro para a estrutura do sistema de arquivos associada
    FATFS *p_fs = sd_get_fs_by_name(drive_path);
    if (!p_fs) {
        printf("ERRO: Sistema de arquivos para %s nao encontrado.\n", drive_path);
        return false;
    }

    // Tenta montar o drive
    FRESULT fr = f_mount(p_fs, drive_path, 1);
    if (fr != FR_OK) {
        printf("ERRO de f_mount: %s (%d)\n", FRESULT_str(fr), fr);
        return false;
    }

    // Atualiza o status no driver do SD
    sd_card_t *pSD = sd_get_by_name(drive_path);
    pSD->mounted = true;
    printf("Cartao SD montado com sucesso.\n");
    return true;
}

/**
 * @brief Desmonta o primeiro cartão SD encontrado.
 */
void unmount_sd_card() {
    const char *drive_path = sd_get_by_num(0)->pcName;
    f_mount(NULL, drive_path, 1);
    
    sd_card_t *pSD = sd_get_by_name(drive_path);
    if (pSD) {
        pSD->mounted = false;
    }
    printf("Cartao SD desmontado.\n");
}