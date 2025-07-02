#ifndef HAL_HELPERS_H
#define HAL_HELPERS_H

#include <stdio.h>
#include <ctype.h>
#include "hal.h"

// ==========================
// ===== PARAM MACROS =======
// ==========================
#define HAL_PARAM_U32_ARRAY(varname, size, prefix,pinname)               \
    do {                                                                 \
        varname = hal_malloc((size) * sizeof(hal_u32_t));               \
        if (!(varname)) return -ENOMEM;                                 \
        for (int i = 0; i < (size); i++) {                               \
            snprintf(name, sizeof(name), "%s" pinname, prefix, i);      \
            hal_param_u32_new(name, HAL_RW, &(varname)[i], comp_id);    \
        }                                                                \
    } while (0)

// --- S32 param array ---
#define HAL_PARAM_S32_ARRAY(varname, size, prefix,pinname)                     \
    do {                                                                 \
        varname = hal_malloc((size) * sizeof(hal_s32_t));               \
        if (!(varname)) return -ENOMEM;                                 \
        for (int i = 0; i < (size); i++) {                               \
            snprintf(name, sizeof(name), "%s" pinname, prefix, i);      \
            hal_param_s32_new(name, HAL_RW, &(varname)[i], comp_id);    \
        }                                                                \
    } while (0)

// --- Float param array ---
#define HAL_PARAM_FLOAT_ARRAY(varname, size, prefix,pinname)                    \
    do {                                                                  \
        varname = hal_malloc((size) * sizeof(hal_float_t));              \
        if (!(varname)) return -ENOMEM;                                  \
        for (int i = 0; i < (size); i++) {                                \
            snprintf(name, sizeof(name), "%s" pinname, prefix, i);      \
            hal_param_float_new(name, HAL_RW, &(varname)[i], comp_id);   \
        }                                                                 \
    } while (0)

// --- Bit param array ---
#define HAL_PARAM_BIT_ARRAY(varname, size, prefix,pinname)                   \
    do {                                                               \
        varname = hal_malloc((size) * sizeof(hal_bit_t));             \
        if (!(varname)) return -ENOMEM;                               \
        for (int i = 0; i < (size); i++) {                             \
            snprintf(name, sizeof(name), "%s" pinname, prefix, i);      \
            hal_param_bit_new(name, HAL_RW, &(varname)[i], comp_id);  \
        }                                                              \
    } while (0)
// ========================
// ===== PIN MACROS ======
// ========================

#define HAL_PIN_FLOAT_ARRAY(varname, size, prefix,pinname, direction)             \
    do {                                                                    \
        varname = hal_malloc((size) * sizeof(hal_float_t *));              \
        if (!(varname)) return -ENOMEM;                                    \
        for (int i = 0; i < (size); i++) {                                  \
            snprintf(name, sizeof(name), "%s" pinname, prefix, i);      \
            hal_pin_float_new(name, direction, &(varname)[i], comp_id);    \
        }                                                                   \
    } while (0)

#define HAL_PIN_S32_ARRAY(varname, size, prefix,pinname, direction)              \
    do {                                                                    \
        varname = hal_malloc((size) * sizeof(hal_bit_t *));               \
        if (!(varname)) return -ENOMEM;                                   \
        for (int i = 0; i < (size); i++) {                                 \
            snprintf(name, sizeof(name), "%s" pinname, prefix, i);      \
            hal_pin_s32_new(name, direction, &(varname)[i], comp_id);     \
        }                                                                  \
    } while (0)

#define HAL_PIN_BIT_ARRAY(varname, size, prefix,pinname, direction)              \
    do {                                                                    \
        varname = hal_malloc((size) * sizeof(hal_bit_t *));               \
        if (!(varname)) return -ENOMEM;                                   \
        for (int i = 0; i < (size); i++) {                                 \
            snprintf(name, sizeof(name), "%s" pinname, prefix, i);      \
            hal_pin_bit_new(name, direction, &(varname)[i], comp_id);     \
        }                                                                  \
    } while (0)

// === Single Parameter Macros ===

#define HAL_PARAM_FLOAT(varname, prefix,halname)                             \
    do {                                                              \
        varname = hal_malloc(sizeof(hal_float_t));                    \
        if (!(varname)) return -ENOMEM;                               \
        snprintf(name, sizeof(name), "%s%s", prefix, halname);           \
        hal_param_float_new((name), HAL_RW, varname, comp_id);     \
    } while (0)

#define HAL_PARAM_U32(varname, prefix,param_name)                         \
    do {                                                              \
        varname = hal_malloc(sizeof(hal_u32_t));                      \
        if (!(varname)) return -ENOMEM;                               \
        snprintf(name, sizeof(name), "%s%s", prefix, param_name);           \
        hal_param_u32_new((name), HAL_RW, varname, comp_id);       \
    } while (0)

#define HAL_PARAM_S32(varname, prefix,param_name)                        \
    do {                                                              \
        varname = hal_malloc(sizeof(hal_s32_t));                      \
        if (!(varname)) return -ENOMEM;                               \
        snprintf(name, sizeof(name), "%s%s", prefix, param_name);        \
        hal_param_s32_new((name), HAL_RW, varname, comp_id);       \
    } while (0)

#define HAL_PARAM_BIT(varname, prefix,param_name)                        \
    do {                                                              \
        varname = hal_malloc(sizeof(hal_bit_t));                      \
        if (!(varname)) return -ENOMEM;                               \
        snprintf(name, sizeof(name), "%s%s", prefix, param_name);        \
        hal_param_bit_new((name), HAL_RW, varname, comp_id);       \
    } while (0)


// === Single Pin Macros ===

   
#define HAL_PIN_FLOAT(varname,prefix,halname, direction)              \
    do {                                                              \
        varname = hal_malloc(sizeof(hal_float_t *));                  \
        if (!(varname)) return -ENOMEM;                               \
        snprintf(name, sizeof(name), "%s%s", prefix, halname);           \
        hal_pin_float_new((name), (direction), varname, comp_id);  \
    } while (0)

#define HAL_PIN_U32(varname,prefix, halname, direction)                    \
    do {                                                              \
        varname = hal_malloc(sizeof(hal_u32_t *));                  \
        if (!(varname)) return -ENOMEM;                               \
        snprintf(name, sizeof(name), "%s%s", prefix, halname);           \
        hal_pin_u32_new((halname), (direction), varname, comp_id);  \
    } while (0)
    
#define HAL_PIN_BIT(varname, prefix,halname, direction)                      \
    do {                                                              \
        varname = hal_malloc(sizeof(hal_bit_t *));                    \
        if (!(varname)) return -ENOMEM;                               \
        snprintf(name, sizeof(name), "%s%s", prefix, halname);           \
        hal_pin_bit_new((name), (direction), varname, comp_id);    \
    } while (0)

#endif 

#define HAL_EXPORT_FUNCT(prefix, suffix, funct)  \
    do {                                         \
        snprintf(name, sizeof(name), "%s%s", prefix, suffix); \
        hal_export_funct(name, funct, 0, 0, 0, comp_id);       \
    } while (0)


static inline void to_lowercase(char* str) {
    while (*str) {
        *str = tolower((unsigned char)*str);
        str++;
    }
}
// HAL_HELPERS_H
