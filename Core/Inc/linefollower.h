#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

#include <stdint.h>

/**
 * @file  linefollower.h
 * @brief Seguidor de línea con interpolación cuadrática y autocalibración.
 *
 * Arquitectura:
 *   - 3 sensores IR equidistantes: L (pos=-1), C (pos=0), R (pos=+1)
 *   - Autocalibración en tiempo real: min/max por sensor
 *   - Normalización: 0.0 = blanco (sin línea), 1.0 = negro (línea)
 *   - Estimación de posición por interpolación cuadrática
 *   - PID sobre el error de posición → corrección diferencial de motores
 */

/* ── Configuración ─────────────────────────────────────── */
#define LF_SENSOR_COUNT     3
#define LF_SENSOR_SPACING   1.5f   // cm entre sensores adyacentes

/* Umbral mínimo de activación — si ningún sensor supera este
   valor normalizado se considera que no hay línea visible.    */
#define LF_DETECTION_THRESHOLD  0.25f

/* Ventana de calibración: cuántos ticks de actualización
   se espera antes de considerar la calibración válida.        */
#define LF_CALIB_WARMUP_TICKS   200

/* ── Tipos ─────────────────────────────────────────────── */

/**
 * @brief Estado de detección de la línea.
 */
typedef enum {
    LF_LINE_FOUND   = 0,   ///< Línea detectada, posición válida
    LF_LINE_LOST    = 1,   ///< Sin línea visible
    LF_CALIB_WARMUP = 2,   ///< Calibración en progreso (warmup)
} _eLFStatus;

/**
 * @brief Handle principal del seguidor de línea.
 *
 * Todos los campos son privados — accedé solo a través de la API.
 */
typedef struct {
    /* Calibración */
    uint16_t cal_min[LF_SENSOR_COUNT];   ///< Mínimo ADC por sensor (sobre línea)
    uint16_t cal_max[LF_SENSOR_COUNT];   ///< Máximo ADC por sensor (sobre blanco)
    uint32_t calib_ticks;                ///< Contador de ticks de calibración

    /* Valores normalizados actuales [0.0 .. 1.0] */
    float    norm[LF_SENSOR_COUNT];

    /* Posición estimada de la línea [-1.5 .. +1.5] cm */
    float    position;

    /* Última posición válida (para recuperación tras pérdida) */
    float    last_valid_position;

    /* Estado */
    _eLFStatus status;

    /* PID */
    float    kp;
    float    ki;
    float    kd;
    float    integral;
    float    prev_error;

    /* Salidas del PID [-1.0 .. +1.0] */
    float    correction;

} _sLFHandle;

/* ── API ────────────────────────────────────────────────── */

/**
 * @brief Inicializa el handle con ganancias PID.
 * @param h    Puntero al handle.
 * @param kp   Ganancia proporcional.
 * @param ki   Ganancia integral.
 * @param kd   Ganancia derivativa.
 */
void LF_Init(_sLFHandle *h, float kp, float ki, float kd);

/**
 * @brief Resetea la calibración acumulada.
 * @param h Puntero al handle.
 */
void LF_ResetCalibration(_sLFHandle *h);

/**
 * @brief Actualiza el seguidor con nuevas lecturas ADC.
 *
 * Debe llamarse cada vez que hay datos frescos del DMA (ej. en flag_loop).
 *
 * @param h       Puntero al handle.
 * @param adc     Array de LF_SENSOR_COUNT lecturas ADC crudas [L, C, R].
 * @return        Estado actual (_eLFStatus).
 */
_eLFStatus LF_Update(_sLFHandle *h, const uint16_t *adc);

/**
 * @brief Retorna la posición estimada de la línea.
 *
 * Válida solo si LF_Update retornó LF_LINE_FOUND.
 * Rango: -1.5 (línea a la izquierda) .. 0.0 (centro) .. +1.5 (derecha) [cm]
 *
 * @param h Puntero al handle.
 * @return  Posición en cm.
 */
float LF_GetPosition(const _sLFHandle *h);

/**
 * @brief Retorna la corrección PID calculada.
 *
 * Rango: -1.0 .. +1.0
 * Positivo → girar a la derecha (reducir motor derecho).
 * Negativo → girar a la izquierda (reducir motor izquierdo).
 *
 * @param h Puntero al handle.
 * @return  Corrección normalizada.
 */
float LF_GetCorrection(const _sLFHandle *h);

/**
 * @brief Retorna el estado actual.
 * @param h Puntero al handle.
 * @return  _eLFStatus
 */
_eLFStatus LF_GetStatus(const _sLFHandle *h);

/**
 * @brief Actualiza las ganancias PID en caliente.
 * @param h   Puntero al handle.
 * @param kp  Nueva ganancia proporcional.
 * @param ki  Nueva ganancia integral.
 * @param kd  Nueva ganancia derivativa.
 */
void LF_SetPID(_sLFHandle *h, float kp, float ki, float kd);

#endif /* LINEFOLLOWER_H */
