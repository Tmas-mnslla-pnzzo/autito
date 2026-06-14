#include "linefollower.h"
#include <stdint.h>
#include <string.h>

/* ── Helpers internos ───────────────────────────────────── */

/**
 * Clamp de float entre [min, max].
 */
static inline float lf_clampf(float v, float mn, float mx)
{
    return (v < mn) ? mn : (v > mx) ? mx : v;
}

/**
 * Normaliza una lectura ADC cruda al rango [0.0 .. 1.0].
 *
 * Los sensores TCRT5000 bajan en presencia de línea negra:
 *   - cal_max = valor sobre blanco (alto)
 *   - cal_min = valor sobre negro  (bajo)
 *
 * norm = (adc - cal_min) / (cal_max - cal_min)
 * Luego invertimos: reflectance = 1.0 - norm
 * → 0.0 = blanco, 1.0 = negro (línea)
 */
static float lf_normalize(uint16_t adc, uint16_t cal_min, uint16_t cal_max)
{
    if (cal_max <= cal_min) return 0.0f;
    float norm = (float)(adc - cal_min) / (float)(cal_max - cal_min);
    norm = lf_clampf(norm, 0.0f, 1.0f);
    return 1.0f - norm;   /* invertir: negro = 1.0 */
}

/**
 * Estimación de posición por interpolación cuadrática con punto auxiliar.
 *
 * Dado que la respuesta del sensor sobre la línea forma una curva
 * aproximadamente gaussiana, ajustamos una parábola por los 3 puntos
 * (pos_i, norm_i) y calculamos el vértice — que corresponde al centro
 * real de la línea con resolución subpíxel.
 *
 * Posiciones físicas: L=-1, C=0, R=+1  (en unidades de spacing)
 *
 * Para 3 puntos equidistantes la fórmula del vértice simplifica a:
 *
 *   x_vertex = (n[L] - n[R]) / (2 * (n[L] - 2*n[C] + n[R]))
 *
 * El resultado está en unidades de spacing, luego se multiplica por
 * LF_SENSOR_SPACING para obtener cm.
 *
 * Si el denominador es muy pequeño (línea centrada o sensores saturados)
 * se usa el centroide ponderado simple como fallback.
 */
static float lf_quadratic_position(const float *n)
{
    float nL = n[0], nC = n[1], nR = n[2];

    /* Centroide ponderado simple (fallback robusto) */
    float sum  = nL + nC + nR;
    if (sum < 1e-4f) return 0.0f;   /* sin señal */

    float centroid = (-1.0f * nL + 0.0f * nC + 1.0f * nR) / sum;

    /* Interpolación cuadrática — vértice de la parábola */
    float denom = nL - 2.0f * nC + nR;
    if (denom < -1e-4f) {           /* parábola cóncava (forma normal) */
        float vertex = 0.5f * (nL - nR) / denom;
        vertex = lf_clampf(vertex, -1.5f, 1.5f);
        return vertex * LF_SENSOR_SPACING;
    }

    /* Fallback: centroide ponderado */
    return lf_clampf(centroid, -1.0f, 1.0f) * LF_SENSOR_SPACING;
}

/* ── API pública ────────────────────────────────────────── */

void LF_Init(_sLFHandle *h, float kp, float ki, float kd)
{
    memset(h, 0, sizeof(_sLFHandle));

    /* Calibración inicial conservadora */
    for (int i = 0; i < LF_SENSOR_COUNT; i++) {
        h->cal_min[i] = 0;
        h->cal_max[i] = 4095;
    }

    h->kp     = kp;
    h->ki     = ki;
    h->kd     = kd;
    h->status = LF_CALIB_WARMUP;
}

void LF_ResetCalibration(_sLFHandle *h)
{
    for (int i = 0; i < LF_SENSOR_COUNT; i++) {
        h->cal_min[i] = 4095;
        h->cal_max[i] = 0;
    }
    h->calib_ticks = 0;
    h->status = LF_CALIB_WARMUP;
}

void LF_SetPID(_sLFHandle *h, float kp, float ki, float kd)
{
    h->kp = kp;
    h->ki = ki;
    h->kd = kd;
}

_eLFStatus LF_Update(_sLFHandle *h, const uint16_t *adc)
{
    /* ── 1. Autocalibración continua ─────────────────────── */
    for (int i = 0; i < LF_SENSOR_COUNT; i++) {
        if (adc[i] < h->cal_min[i]) h->cal_min[i] = adc[i];
        if (adc[i] > h->cal_max[i]) h->cal_max[i] = adc[i];
    }
    h->calib_ticks++;

    /* Durante warmup no actuamos */
    if (h->calib_ticks < LF_CALIB_WARMUP_TICKS) {
        h->status = LF_CALIB_WARMUP;
        h->correction = 0.0f;
        return h->status;
    }

    /* ── 2. Normalización ────────────────────────────────── */
    for (int i = 0; i < LF_SENSOR_COUNT; i++) {
        h->norm[i] = lf_normalize(adc[i], h->cal_min[i], h->cal_max[i]);
    }

    /* ── 3. Detección de línea ───────────────────────────── */
    float max_activation = 0.0f;
    for (int i = 0; i < LF_SENSOR_COUNT; i++) {
        if (h->norm[i] > max_activation) max_activation = h->norm[i];
    }

    if (max_activation < LF_DETECTION_THRESHOLD) {
        /* Línea perdida — mantenemos última corrección conocida */
        h->status = LF_LINE_LOST;
        /* No reseteamos la corrección — el robot sigue girando
           hacia donde vio la línea por última vez              */
        return h->status;
    }

    /* ── 4. Estimación de posición ───────────────────────── */
    h->position = lf_quadratic_position(h->norm);
    h->last_valid_position = h->position;
    h->status = LF_LINE_FOUND;

    /* ── 5. PID ──────────────────────────────────────────── */
    float error = h->position;   /* 0 = línea centrada */

    float p = h->kp * error;

    h->integral += error;
    h->integral  = lf_clampf(h->integral, -10.0f, 10.0f); /* anti-windup */
    float i_term = h->ki * h->integral;

    float d_term  = h->kd * (error - h->prev_error);
    h->prev_error = error;

    h->correction = lf_clampf(p + i_term + d_term, -1.0f, 1.0f);

    return h->status;
}

float LF_GetPosition(const _sLFHandle *h)
{
    return h->position;
}

float LF_GetCorrection(const _sLFHandle *h)
{
    return h->correction;
}

_eLFStatus LF_GetStatus(const _sLFHandle *h)
{
    return h->status;
}
