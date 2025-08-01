#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "ff.h"
#include "diskio.h"
#include "f_util.h"
#include "hw_config.h"
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
#define BUZZER_FREQUENCY 4000

// Estrutura para uma única amostra de dados do IMU
typedef struct
{
    uint32_t sample_num;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    datetime_t timestamp; //Armazena a data e hora
} ImuSample;

//máximo de amostras na RAM
#define MAX_SAMPLES 3000

enum MODE
{
    WAITING,
    SDMOUNT,
    READY,
    CAPTURING,
    ACESSING,
    ERROR
};

// Mensagem comunicação entre tarefas
typedef struct
{
    enum MODE new_mode;
    uint32_t sample_count; 
} DisplayMessage;
static char filename[20] = "datalog.csv";

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

//Encontra uma estrutura de cartão SD pelo seu nome
static sd_card_t *sd_get_by_name(const char *const name)
{
    // Percorre todos os cartões SD disponíveis no sistema.
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return sd_get_by_num(i); // Se os nomes correspondem, retorna o ponteiro para o cartão
    return NULL;
}

//Encontra uma FATFS pelo nome do drive.
static FATFS *sd_get_fs_by_name(const char *name)
{
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return &sd_get_by_num(i)->fatfs;
    return NULL;
}

//Lê os dados do sensor IMU MPU6050 via I2C
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

void capture_imu_data_and_save(const char *filename, int num_samples)
{
    FIL file;
    FRESULT res;

    // Abre o arquivo para escrita, criando-o se não existir.
    res = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK)
    {
        printf("ERRO: Nao foi possivel abrir o arquivo %s\n", filename);
        return;
    }

    // Escreve o cabeçalho no arquivo .csv
    const char *header = "numero_amostra,accel_x,accel_y,accel_z,giro_x,giro_y,giro_z\n";
    UINT bytes_written;
    f_write(&file, header, strlen(header), &bytes_written);

    //Loop para capturar e salvar o número de amostras definido.
    for (int i = 0; i < num_samples; i++)
    {
        int16_t aceleracao[3], gyro[3], temp;

        mpu6050_read_raw(aceleracao, gyro, &temp);

        // A sensibilidade padrão é 16384 para aceleração e 131 para giroscópio.
        float ax = aceleracao[0] / 16384.0f;
        float ay = aceleracao[1] / 16384.0f;
        float az = aceleracao[2] / 16384.0f;
        float gx = gyro[0] / 131.0f;
        float gy = gyro[1] / 131.0f;
        float gz = gyro[2] / 131.0f;

        //Formata a string com todos os dados no formato CSV.
        char buffer[128];
        int len = snprintf(buffer, sizeof(buffer), "%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                           i + 1, ax, ay, az, gx, gy, gz);

        res = f_write(&file, buffer, len, &bytes_written);
        if (res != FR_OK)
        {
            printf("ERRO: Falha ao escrever no arquivo na amostra %d\n", i + 1);
            break; 
        }

        sleep_ms(100);
    }
    f_close(&file);
    printf("Dados salvos com sucesso em %s\n", filename);
}

bool mount_sd_card() {
    // Pega o caminho do primeiro cartão
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

void unmount_sd_card() {
    const char *drive_path = sd_get_by_num(0)->pcName;
    f_mount(NULL, drive_path, 1);
    
    sd_card_t *pSD = sd_get_by_name(drive_path);
    if (pSD) {
        pSD->mounted = false;
        pSD->m_Status |= STA_NOINIT; 
    }
    printf("Cartao SD desmontado.\n");
}