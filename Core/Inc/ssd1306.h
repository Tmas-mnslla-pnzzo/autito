#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>

// ── Callbacks de hardware ─────────────────────────────────
// El usuario implementa estas funciones para su plataforma.
// i2c_write: manda 'len' bytes del buffer 'data' a la direccion 'addr'.
//            retorna 0 si OK, distinto de 0 si error.
typedef uint8_t (*ssd1306_i2c_write_cb_t)(uint8_t addr, const uint8_t *data, uint8_t len);

// ── Constantes ────────────────────────────────────────────
#define SSD1306_ADDR          0x3C   // direccion I2C por defecto (0x3D si SD0 en VCC)
#define SSD1306_WIDTH         128
#define SSD1306_HEIGHT        64
#define SSD1306_PAGES         8      // HEIGHT / 8 bits por pagina
#define SSD1306_BUF_SIZE      (SSD1306_WIDTH * SSD1306_PAGES)  // 1024 bytes

// ── Handle del dispositivo ────────────────────────────────
typedef struct {
    ssd1306_i2c_write_cb_t i2c_write;   // callback de escritura I2C
    uint8_t                addr;         // direccion I2C del display
    uint8_t                buf[SSD1306_BUF_SIZE]; // frame buffer
    uint8_t                dirty;        // 1 si el buffer cambio y hay que mandar
} SSD1306_t;

// ── API ───────────────────────────────────────────────────

// Inicializa el display con la secuencia de comandos estandar.
// Debe llamarse una vez antes de cualquier otra funcion.
void SSD1306_Init(SSD1306_t *dev, ssd1306_i2c_write_cb_t i2c_write, uint8_t addr);

// Limpia el frame buffer (todo en negro). No actualiza el display.
void SSD1306_Clear(SSD1306_t *dev);

// Dibuja un pixel en (x, y). color: 1=blanco, 0=negro.
void SSD1306_DrawPixel(SSD1306_t *dev, uint8_t x, uint8_t y, uint8_t color);

// Dibuja un caracter en la posicion de columna 'col' (0-20) y fila 'row' (0-7).
// Usa font 6x8. color: 1=blanco, 0=negro.
void SSD1306_DrawChar(SSD1306_t *dev, uint8_t col, uint8_t row, char c, uint8_t color);

// Dibuja una cadena de texto a partir de (col, row).
void SSD1306_DrawText(SSD1306_t *dev, uint8_t col, uint8_t row, const char *str, uint8_t color);

// Dibuja un numero entero sin signo.
void SSD1306_DrawNumber(SSD1306_t *dev, uint8_t col, uint8_t row, uint32_t num, uint8_t color);

// Dibuja un numero entero con signo.
void SSD1306_DrawNumberSigned(SSD1306_t *dev, uint8_t col, uint8_t row, int32_t num, uint8_t color);

// Envia el frame buffer completo al display por I2C.
// Llama internamente al callback i2c_write.
void SSD1306_Update(SSD1306_t *dev);

// Envia solo una pagina (fila de 8 pixels) al display.
// Util para actualizar parcialmente sin mandar el buffer entero.
void SSD1306_UpdatePage(SSD1306_t *dev, uint8_t page);

#endif /* SSD1306_H */
