#ifndef ESP8266_H
#define ESP8266_H

#include <stdint.h>

// ── Callbacks de hardware ─────────────────────────────────
// El usuario implementa estas funciones para su plataforma.

// Escribe 'len' bytes por UART hacia el ESP8266.
typedef void    (*esp_uart_write_cb_t)(const uint8_t *data, uint8_t len);

// Lee hasta 'max_len' bytes disponibles del UART. Retorna cuantos leyo.
typedef uint8_t (*esp_uart_read_cb_t)(uint8_t *buf, uint8_t max_len);

// Retorna el tiempo actual en milisegundos (para timeouts).
typedef uint32_t (*esp_millis_cb_t)(void);

// ── Callbacks de eventos ──────────────────────────────────
typedef void (*esp_on_connected_cb_t)(void);
typedef void (*esp_on_disconnected_cb_t)(void);
typedef void (*esp_on_data_cb_t)(const uint8_t *data, uint8_t len);
typedef void (*esp_on_error_cb_t)(const char *msg);

typedef enum {
    ESP_STATE_IDLE = 0,
    ESP_STATE_RESET,
    ESP_STATE_INIT,
    ESP_STATE_SET_MODE,
    ESP_STATE_CONNECT_WIFI,
    ESP_STATE_WAIT_WIFI,
    ESP_STATE_CONNECT_TCP,
    ESP_STATE_WAIT_TCP,
    ESP_STATE_READY,
    ESP_STATE_SENDING,
    ESP_STATE_SENDING_DATA,
    ESP_STATE_ERROR,
} ESP_State_t;

typedef struct {
    const char *ssid;
    const char *password;
    const char *host;
    uint16_t    port;
} ESP_Config_t;

#define ESP_RX_BUF_SIZE  256
#define ESP_TX_BUF_SIZE  128

typedef struct {
    esp_uart_write_cb_t  uart_write;
    esp_uart_read_cb_t   uart_read;
    esp_millis_cb_t      millis;
    esp_on_connected_cb_t    on_connected;
    esp_on_disconnected_cb_t on_disconnected;
    esp_on_data_cb_t         on_data;
    esp_on_error_cb_t        on_error;
    ESP_State_t  state;
    uint32_t     timeout_ms;
    uint32_t     timeout_limit;
    uint8_t  rx_buf[ESP_RX_BUF_SIZE];
    uint16_t rx_len;
    const uint8_t *tx_pending;
    uint8_t        tx_pending_len;
    ESP_Config_t cfg;
    uint8_t wifi_connected;
    uint8_t tcp_connected;
} ESP_t;

// ── API ───────────────────────────────────────────────────

// Inicializa el handle con los callbacks de hardware y eventos.
// No envía ningun comando todavia.
void ESP_Init(ESP_t *dev,
              esp_uart_write_cb_t  uart_write,
              esp_uart_read_cb_t   uart_read,
              esp_millis_cb_t      millis,
              esp_on_connected_cb_t    on_connected,
              esp_on_disconnected_cb_t on_disconnected,
              esp_on_data_cb_t         on_data,
              esp_on_error_cb_t        on_error);

// Arranca el proceso de conexion: reset, modo station, WiFi, TCP.
// Llama una vez despues de Init. El proceso es completamente no bloqueante.
void ESP_Connect(ESP_t *dev, const ESP_Config_t *cfg);

// Motor principal de la maquina de estados. Llamar periodicamente
// desde el loop principal o desde un timer. Nunca bloquea.
void ESP_Tick(ESP_t *dev);

// Encola datos para enviar por TCP. Solo funciona si state == ESP_STATE_READY.
// Retorna 1 si acepto los datos, 0 si esta ocupado o no conectado.
uint8_t ESP_Send(ESP_t *dev, const uint8_t *data, uint8_t len);

// Retorna 1 si el canal TCP esta listo para enviar.
uint8_t ESP_IsReady(const ESP_t *dev);

// Retorna 1 si el WiFi esta conectado.
uint8_t ESP_IsWifiConnected(const ESP_t *dev);

// Fuerza un reset y reconexion desde cero.
void ESP_Reset(ESP_t *dev);

#endif /* ESP8266_H */
