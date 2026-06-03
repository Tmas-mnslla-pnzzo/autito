#include "esp8266.h"
#include <string.h>
#include <stdio.h>

// ── Helpers internos ──────────────────────────────────────

// Verifica si el buffer RX contiene una subcadena buscada.
static uint8_t rx_contains(ESP_t *dev, const char *str) {
    uint8_t slen = (uint8_t)strlen(str);
    if (dev->rx_len < slen) return 0;
    for (uint16_t i = 0; i <= dev->rx_len - slen; i++) {
        if (memcmp(&dev->rx_buf[i], str, slen) == 0) return 1;
    }
    return 0;
}

// Limpia el buffer RX.
static void rx_clear(ESP_t *dev) {
    dev->rx_len = 0;
}

// Manda un comando AT como string por UART.
static void send_cmd(ESP_t *dev, const char *cmd) {
    dev->uart_write((const uint8_t *)cmd, (uint8_t)strlen(cmd));
}

// Registra el inicio de un timeout.
static void start_timeout(ESP_t *dev, uint32_t limit_ms) {
    dev->timeout_ms    = dev->millis();
    dev->timeout_limit = limit_ms;
}

// Retorna 1 si el timeout expiro.
static uint8_t timeout_expired(ESP_t *dev) {
    return (dev->millis() - dev->timeout_ms) >= dev->timeout_limit;
}

// Transiciona al estado de error y notifica.
static void go_error(ESP_t *dev, const char *msg) {
    dev->state         = ESP_STATE_ERROR;
    dev->tcp_connected = 0;
    if (dev->on_error) dev->on_error(msg);
}

// ── API publica ───────────────────────────────────────────

void ESP_Init(ESP_t *dev,
              esp_uart_write_cb_t  uart_write,
              esp_uart_read_cb_t   uart_read,
              esp_millis_cb_t      millis,
              esp_on_connected_cb_t    on_connected,
              esp_on_disconnected_cb_t on_disconnected,
              esp_on_data_cb_t         on_data,
              esp_on_error_cb_t        on_error)
{
    memset(dev, 0, sizeof(ESP_t));
    dev->uart_write      = uart_write;
    dev->uart_read       = uart_read;
    dev->millis          = millis;
    dev->on_connected    = on_connected;
    dev->on_disconnected = on_disconnected;
    dev->on_data         = on_data;
    dev->on_error        = on_error;
    dev->state           = ESP_STATE_IDLE;
}

void ESP_Connect(ESP_t *dev, const ESP_Config_t *cfg) {
    dev->cfg            = *cfg;
    dev->wifi_connected = 0;
    dev->tcp_connected  = 0;
    dev->rx_len         = 0;
    dev->state          = ESP_STATE_RESET;
    send_cmd(dev, "AT+RST\r\n");
    start_timeout(dev, 3000);
}

void ESP_Reset(ESP_t *dev) {
    ESP_Connect(dev, &dev->cfg);
}

uint8_t ESP_IsReady(const ESP_t *dev) {
    return dev->state == ESP_STATE_READY;
}

uint8_t ESP_IsWifiConnected(const ESP_t *dev) {
    return dev->wifi_connected;
}

uint8_t ESP_Send(ESP_t *dev, const uint8_t *data, uint8_t len) {
    if (dev->state != ESP_STATE_READY || len == 0) return 0;
    dev->tx_pending     = data;
    dev->tx_pending_len = len;
    // Mandar AT+CIPSEND=n
    char cmd[24];
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d\r\n", len);
    send_cmd(dev, cmd);
    rx_clear(dev);
    dev->state = ESP_STATE_SENDING;
    start_timeout(dev, 2000);
    return 1;
}

// ── Maquina de estados ─────────────────────────────────────
// Cada estado espera una respuesta del ESP y transiciona al siguiente.
// Si el timeout expira, va a error.
void ESP_Tick(ESP_t *dev) {
    // Leer bytes disponibles del UART y acumularlos en rx_buf
    if (dev->rx_len < ESP_RX_BUF_SIZE) {
        uint8_t tmp[32];
        uint8_t n = dev->uart_read(tmp, sizeof(tmp));
        if (n > 0) {
            uint8_t space = ESP_RX_BUF_SIZE - dev->rx_len;
            if (n > space) n = space;
            memcpy(&dev->rx_buf[dev->rx_len], tmp, n);
            dev->rx_len += n;
        }
    }

    switch (dev->state) {

    case ESP_STATE_IDLE:
    case ESP_STATE_ERROR:
        // No hace nada, espera que el usuario llame ESP_Connect o ESP_Reset
        break;

    // ── Reset: esperar "ready" del ESP ─────────────────────
    case ESP_STATE_RESET:
        if (rx_contains(dev, "ready")) {
            rx_clear(dev);
            send_cmd(dev, "ATE0\r\n");   // deshabilitar echo
            dev->state = ESP_STATE_INIT;
            start_timeout(dev, 2000);
        } else if (timeout_expired(dev)) {
            // Algunos ESP mandan "ready" lento — reintentamos con AT
            rx_clear(dev);
            send_cmd(dev, "AT\r\n");
            dev->state = ESP_STATE_INIT;
            start_timeout(dev, 2000);
        }
        break;

    // ── Init: esperar OK al AT ─────────────────────────────
    case ESP_STATE_INIT:
        if (rx_contains(dev, "OK")) {
            rx_clear(dev);
            send_cmd(dev, "AT+CWMODE=1\r\n");
            dev->state = ESP_STATE_SET_MODE;
            start_timeout(dev, 2000);
        } else if (timeout_expired(dev)) {
            go_error(dev, "AT timeout");
        }
        break;

    // ── Modo station ──────────────────────────────────────
    case ESP_STATE_SET_MODE:
        if (rx_contains(dev, "OK") || rx_contains(dev, "no change")) {
            rx_clear(dev);
            // Armar comando AT+CWJAP="ssid","pass"
            char cmd[128];
            snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n",
                     dev->cfg.ssid, dev->cfg.password);
            send_cmd(dev, cmd);
            dev->state = ESP_STATE_CONNECT_WIFI;
            start_timeout(dev, 15000);   // conexion WiFi puede tardar hasta 15s
        } else if (timeout_expired(dev)) {
            go_error(dev, "CWMODE timeout");
        }
        break;

    // ── Conexion WiFi ─────────────────────────────────────
    case ESP_STATE_CONNECT_WIFI:
        if (rx_contains(dev, "WIFI GOT IP") || rx_contains(dev, "OK")) {
            dev->wifi_connected = 1;
            rx_clear(dev);
            // Armar comando AT+CIPSTART="TCP","host",port
            char cmd[128];
            snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n",
                     dev->cfg.host, dev->cfg.port);
            send_cmd(dev, cmd);
            dev->state = ESP_STATE_CONNECT_TCP;
            start_timeout(dev, 10000);
        } else if (rx_contains(dev, "FAIL") || rx_contains(dev, "ERROR")) {
            go_error(dev, "WiFi FAIL");
        } else if (timeout_expired(dev)) {
            go_error(dev, "WiFi timeout");
        }
        break;

    // ── Conexion TCP ──────────────────────────────────────
    case ESP_STATE_CONNECT_TCP:
        if (rx_contains(dev, "CONNECT")) {
            dev->tcp_connected = 1;
            rx_clear(dev);
            dev->state = ESP_STATE_READY;
            if (dev->on_connected) dev->on_connected();
        } else if (rx_contains(dev, "ERROR") || rx_contains(dev, "CLOSED")) {
            go_error(dev, "TCP FAIL");
        } else if (timeout_expired(dev)) {
            go_error(dev, "TCP timeout");
        }
        break;

    // ── Listo: monitorear desconexion y datos entrantes ───
    case ESP_STATE_READY:
        // Detectar desconexion
        if (rx_contains(dev, "CLOSED") || rx_contains(dev, "DISCONNECT")) {
            dev->tcp_connected = 0;
            rx_clear(dev);
            if (dev->on_disconnected) dev->on_disconnected();
            // Intentar reconectar TCP automaticamente
            char cmd[128];
            snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n",
                     dev->cfg.host, dev->cfg.port);
            send_cmd(dev, cmd);
            dev->state = ESP_STATE_CONNECT_TCP;
            start_timeout(dev, 10000);
            break;
        }
        // Detectar datos entrantes: "+IPD,n:datos"
        {
            const char *ipd = "+IPD,";
            if (rx_contains(dev, ipd)) {
                // Buscar el patron +IPD,len:data
                for (uint16_t i = 0; i + 5 < dev->rx_len; i++) {
                    if (memcmp(&dev->rx_buf[i], ipd, 5) == 0) {
                        uint8_t dlen = 0;
                        uint16_t j = i + 5;
                        while (j < dev->rx_len && dev->rx_buf[j] != ':') {
                            dlen = dlen * 10 + (dev->rx_buf[j] - '0');
                            j++;
                        }
                        j++;  // saltar el ':'
                        if (j + dlen <= dev->rx_len) {
                            if (dev->on_data) dev->on_data(&dev->rx_buf[j], dlen);
                            rx_clear(dev);
                        }
                        break;
                    }
                }
            }
        }
        break;

    // ── Esperando '>' para mandar datos ──────────────────
    case ESP_STATE_SENDING:
        if (rx_contains(dev, ">")) {
            rx_clear(dev);
            dev->uart_write(dev->tx_pending, dev->tx_pending_len);
            dev->state = ESP_STATE_SENDING_DATA;
            start_timeout(dev, 3000);
        } else if (rx_contains(dev, "ERROR")) {
            go_error(dev, "CIPSEND ERROR");
        } else if (timeout_expired(dev)) {
            go_error(dev, "CIPSEND timeout");
        }
        break;

    // ── Esperando SEND OK ─────────────────────────────────
    case ESP_STATE_SENDING_DATA:
        if (rx_contains(dev, "SEND OK")) {
            rx_clear(dev);
            dev->tx_pending     = 0;
            dev->tx_pending_len = 0;
            dev->state          = ESP_STATE_READY;
        } else if (rx_contains(dev, "ERROR")) {
            go_error(dev, "SEND ERROR");
        } else if (timeout_expired(dev)) {
            go_error(dev, "SEND timeout");
        }
        break;

    default:
        dev->state = ESP_STATE_IDLE;
        break;
    }
}
