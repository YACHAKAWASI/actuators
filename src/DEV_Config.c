#include "actuator_motor/DEV_Config.h"
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>

static int      i2c_fd  = -1;      // /dev/i2c-1
static uint8_t  i2c_addr = 0;      // Dirección esclavo I2C

uint32_t fd;   // legado (compatibilidad con código previo; no se usa)
int INT_PIN;   // declarado extern en el .h

// ========================= GPIO (libgpiod) — estado global =========================
#if USE_GPIOD_LIB
#include <gpiod.h>
typedef struct {
    struct gpiod_line* line;
    int mode; // -1 = sin configurar, 0 = input, 1 = output
} gpio_entry_t;

static struct gpiod_chip *g_chip = NULL; // chip GPIO abierto una vez
static gpio_entry_t gpio_map[128];       // BCM 0..127
#endif

/*********************** Utilidades ************************/

/* Detección de equipo (heurística) */
static int DEV_Equipment_Testing(void)
{
    int fd_local;
    // Leer model desde device-tree (Raspberry Pi reporta su modelo)
    fd_local = open("/proc/device-tree/model", O_RDONLY);
    if (fd_local >= 0) {
        char buf[128] = {0};
        int n = read(fd_local, buf, sizeof(buf) - 1);
        close(fd_local);
        if (n > 0) {
            for (int i = 0; i < n; ++i) if (buf[i] == '\0') buf[i] = ' ';
            printf("Current model: %s\r\n", buf);
            if (strstr(buf, "Raspberry Pi") != NULL) {
                return 'R';
            }
        }
    }

    // Fallback: /etc/issue (menos confiable)
    int i = 0;
    char value_str[64] = {0};
    fd_local = open("/etc/issue", O_RDONLY);
    printf("Current environment: ");
    if (fd_local < 0) {
        return -1;
    }
    for (i = 0; i < (int)sizeof(value_str) - 1; i++) {
        int r = read(fd_local, &value_str[i], 1);
        if (r <= 0 || value_str[i] == '\n' || value_str[i] == ' ') break;
        printf("%c", value_str[i]);
    }
    printf("\r\n");
    close(fd_local);
    return -1;
}

/* Abre el gpiochip correcto:
 * - Usa DEVICE_PACK_GPIOCHIP si está definida (recomendado en Pi 5).
 * - Si no, intenta /dev/gpiochip4 (Pi 5) y luego /dev/gpiochip0 como fallback.
 */
static struct gpiod_chip* gpio_get_chip(void)
{
#if USE_GPIOD_LIB
    if (g_chip) return g_chip;

    const char* env = getenv("ACTUATOR_MOTOR_GPIOCHIP");
    if (env) {
        g_chip = gpiod_chip_open(env);
        if (!g_chip) perror("gpiod_chip_open(env)");
    } else {
        g_chip = gpiod_chip_open("/dev/gpiochip4");
        if (!g_chip) {
            // Fallback a chip0 si 4 no existe
            g_chip = gpiod_chip_open("/dev/gpiochip0");
            if (!g_chip) perror("gpiod_chip_open(chip4/chip0)");
        }
    }
    // Inicializa el mapa
    for (int i = 0; i < 128; ++i) { gpio_map[i].line = NULL; gpio_map[i].mode = -1; }

    return g_chip;
#else
    return NULL;
#endif
}

/*********************** GPIO API ************************/

void DEV_GPIO_Mode(UWORD Pin, UWORD Mode)
{
#if USE_GPIOD_LIB
    struct gpiod_chip *chip = gpio_get_chip();
    if (!chip) return;
    if (Pin >= 128) return;

    // Si ya está en el modo solicitado, no rehacer
    if (gpio_map[Pin].line && gpio_map[Pin].mode == (int)(Mode ? 1 : 0)) return;

    // Si había una línea previa, libérala
    if (gpio_map[Pin].line) {
        gpiod_line_release(gpio_map[Pin].line);
        gpio_map[Pin].line = NULL;
        gpio_map[Pin].mode = -1;
    }

    struct gpiod_line *line = gpiod_chip_get_line(chip, Pin);
    if (!line) { fprintf(stderr, "gpiod_chip_get_line(%u) failed\n", Pin); return; }

    int ret = (Mode == 0)
        ? gpiod_line_request_input(line, "actuator_motor")
        : gpiod_line_request_output(line, "actuator_motor", 0);
    if (ret < 0) { perror("gpiod_line_request"); return; }

    gpio_map[Pin].line = line;
    gpio_map[Pin].mode = (int)(Mode ? 1 : 0);
#else
    (void)Pin; (void)Mode;
#endif
}

void DEV_Digital_Write(UWORD Pin, UBYTE Value)
{
#if USE_GPIOD_LIB
    if (Pin >= 128) return;
    if (!gpio_map[Pin].line || gpio_map[Pin].mode != 1)
        DEV_GPIO_Mode(Pin, 1); // asegura salida
    if (!gpio_map[Pin].line) return;

    if (gpiod_line_set_value(gpio_map[Pin].line, Value ? 1 : 0) < 0) {
        perror("gpiod_line_set_value");
    }
#else
    (void)Pin; (void)Value;
#endif
}

UBYTE DEV_Digital_Read(UWORD Pin)
{
    UBYTE Read_value = 0;
#if USE_GPIOD_LIB
    if (Pin >= 128) return 0;
    if (!gpio_map[Pin].line || gpio_map[Pin].mode != 0)
        DEV_GPIO_Mode(Pin, 0); // asegura entrada
    if (!gpio_map[Pin].line) return 0;

    int v = gpiod_line_get_value(gpio_map[Pin].line);
    if (v < 0) { perror("gpiod_line_get_value"); v = 0; }
    Read_value = (UBYTE)(v ? 1 : 0);
#else
    (void)Pin; Read_value = 0;
#endif
    return Read_value;
}

/*********************** Delay ************************/

void DEV_Delay_ms(UDOUBLE xms)
{
    usleep(1000 * xms);
}

/*********************** GPIO inicial ************************/

static void GPIO_Config(void)
{
    int Equipment = DEV_Equipment_Testing();
    if (Equipment == 'R') {
        INT_PIN = 4; // BCM4 (GPIO4)
    } else {
        printf("Device unrecognized, defaulting INT_PIN to BCM4.\r\n");
        INT_PIN = 4;
    }
    DEV_GPIO_Mode(INT_PIN, 0); // INPUT por defecto
}

/*********************** SPI (opcional, spidev) ************************/

void DEV_SPI_Init(void)
{
#if DEV_SPI && USE_SPIDEV_LIB
    // Implementación a demanda (spidev). No usada actualmente.
#endif
}

void DEV_SPI_WriteByte(UBYTE Value)
{
#if DEV_SPI && USE_SPIDEV_LIB
    (void)Value;
#endif
}

void DEV_SPI_Write_nByte(uint8_t *pData, uint32_t Len)
{
#if DEV_SPI && USE_SPIDEV_LIB
    (void)pData; (void)Len;
#endif
}

/*********************** I2C (i2c-dev) ************************/

void DEV_I2C_Init(uint8_t Add)
{
#if DEV_I2C && USE_I2CDEV_LIB
    if (i2c_fd >= 0) {
        close(i2c_fd);
        i2c_fd = -1;
    }

    i2c_fd = open("/dev/i2c-1", O_RDWR);
    if (i2c_fd < 0) {
        perror("open(/dev/i2c-1)");
        return;
    }
    if (ioctl(i2c_fd, I2C_SLAVE, Add) < 0) {
        perror("ioctl(I2C_SLAVE)");
        close(i2c_fd);
        i2c_fd = -1;
        return;
    }
    i2c_addr = Add;
#else
    (void)Add;
#endif
}

void I2C_Write_Byte(uint8_t Cmd, uint8_t value)
{
#if DEV_I2C && USE_I2CDEV_LIB
    if (i2c_fd < 0) return;
    uint8_t wbuf[2] = { Cmd, value };
    ssize_t n = write(i2c_fd, wbuf, 2);
    if (n != 2) {
        perror("I2C write");
    }
#else
    (void)Cmd; (void)value;
#endif
}

int I2C_Read_Byte(uint8_t Cmd)
{
    int ref = -1;
#if DEV_I2C && USE_I2CDEV_LIB
    if (i2c_fd < 0) return -1;

    // Escribir registro
    uint8_t reg = Cmd;
    if (write(i2c_fd, &reg, 1) != 1) {
        perror("I2C write(reg)");
        return -1;
    }
    // Leer 1 byte
    uint8_t rbuf = 0;
    if (read(i2c_fd, &rbuf, 1) != 1) {
        perror("I2C read(byte)");
        return -1;
    }
    ref = (int)rbuf;
#else
    (void)Cmd;
#endif
    return ref;
}

int I2C_Read_Word(uint8_t Cmd)
{
    int ref = -1;
#if DEV_I2C && USE_I2CDEV_LIB
    if (i2c_fd < 0) return -1;

    // Escribir registro
    uint8_t reg = Cmd;
    if (write(i2c_fd, &reg, 1) != 1) {
        perror("I2C write(reg)");
        return -1;
    }
    // Leer 2 bytes (LSB, MSB típico)
    uint8_t rbuf[2] = {0, 0};
    if (read(i2c_fd, rbuf, 2) != 2) {
        perror("I2C read(word)");
        return -1;
    }
    // Ensamble little-endian: low en [0], high en [1]
    ref = (int)((rbuf[1] << 8) | rbuf[0]);
#else
    (void)Cmd;
#endif
    return ref;
}

/*********************** INIT / EXIT MÓDULO ************************/

UBYTE DEV_ModuleInit(void)
{
#if USE_GPIOD_LIB
    if (!gpio_get_chip()) {
        fprintf(stderr, "GPIO init failed (libgpiod)\n");
        // No abortamos; podrías usar solo I2C
    }
#endif

#if USE_I2CDEV_LIB
    // Dirección por defecto según tu código original
    DEV_I2C_Init(0x29);
#endif

    GPIO_Config();
    return 0;
}

void DEV_ModuleExit(void)
{
#if USE_I2CDEV_LIB
    if (i2c_fd >= 0) {
        close(i2c_fd);
        i2c_fd = -1;
    }
#endif

#if USE_GPIOD_LIB
    // Libera todas las líneas pedidas
    for (int i = 0; i < 128; ++i) {
        if (gpio_map[i].line) {
            gpiod_line_release(gpio_map[i].line);
            gpio_map[i].line = NULL;
            gpio_map[i].mode = -1;
        }
    }
    if (g_chip) {
        gpiod_chip_close(g_chip);
        g_chip = NULL;
    }
#endif

#if DEV_SPI && USE_SPIDEV_LIB
    // cerrar fd_spi si se usara
#endif
}
