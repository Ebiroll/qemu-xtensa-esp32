/*
 * Floating point coprocessor helpers.
 *
 * Copyright (c) 2011 - 2015, Max Filippov, Open Source and Linux Lab.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Open Source and Linux Lab nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "cpu.h"
#include "exec/helper-proto.h"
#include "qemu/host-utils.h"
#include "exec/cpu_ldst.h"
#include "exec/address-spaces.h"

enum {
    XTENSA_FP_I = 0x1,
    XTENSA_FP_U = 0x2,
    XTENSA_FP_O = 0x4,
    XTENSA_FP_Z = 0x8,
    XTENSA_FP_V = 0x10,
};

enum {
    XTENSA_FCR_FLAGS_SHIFT = 2,
    XTENSA_FSR_FLAGS_SHIFT = 7,
};

static const struct {
    uint32_t xtensa_fp_flag;
    int softfloat_fp_flag;
} xtensa_fp_flag_map[] = {
    { XTENSA_FP_I, float_flag_inexact, },
    { XTENSA_FP_U, float_flag_underflow, },
    { XTENSA_FP_O, float_flag_overflow, },
    { XTENSA_FP_Z, float_flag_divbyzero, },
    { XTENSA_FP_V, float_flag_invalid, },
};

void HELPER(wur_fcr)(CPUXtensaState *env, uint32_t v)
{
    static const int rounding_mode[] = {
        float_round_nearest_even,
        float_round_to_zero,
        float_round_up,
        float_round_down,
    };

    env->uregs[FCR] = v & 0xfffff07f;
    set_float_rounding_mode(rounding_mode[v & 3], &env->fp_status);
}

void HELPER(wur_fsr)(CPUXtensaState *env, uint32_t v)
{
    uint32_t flags = v >> XTENSA_FSR_FLAGS_SHIFT;
    int fef = 0;
    unsigned i;

    env->uregs[FSR] = v & 0xfffff000;
    for (i = 0; i < ARRAY_SIZE(xtensa_fp_flag_map); ++i) {
        if (flags & xtensa_fp_flag_map[i].xtensa_fp_flag) {
            fef |= xtensa_fp_flag_map[i].softfloat_fp_flag;
        }
    }
    set_float_exception_flags(fef, &env->fp_status);
}

uint32_t HELPER(rur_fsr)(CPUXtensaState *env)
{
    uint32_t flags = 0;
    int fef = get_float_exception_flags(&env->fp_status);
    unsigned i;

    for (i = 0; i < ARRAY_SIZE(xtensa_fp_flag_map); ++i) {
        if (fef & xtensa_fp_flag_map[i].softfloat_fp_flag) {
            flags |= xtensa_fp_flag_map[i].xtensa_fp_flag;
        }
    }
    return (env->uregs[FSR] & 0xfffff000) | (flags << XTENSA_FSR_FLAGS_SHIFT);
}

float32 HELPER(abs_s)(float32 v)
{
    return float32_abs(v);
}

float64 HELPER(cvtd_s)(CPUXtensaState *env, float32 v)
{
    return float32_to_float64(v, &env->fp_status);
}

float32 HELPER(neg_s)(float32 v)
{
    return float32_chs(v);
}

float32 HELPER(add_s)(CPUXtensaState *env, float32 a, float32 b)
{
    return float32_add(a, b, &env->fp_status);
}

float32 HELPER(sub_s)(CPUXtensaState *env, float32 a, float32 b)
{
    return float32_sub(a, b, &env->fp_status);
}

float32 HELPER(mul_s)(CPUXtensaState *env, float32 a, float32 b)
{
    return float32_mul(a, b, &env->fp_status);
}

float32 HELPER(madd_s)(CPUXtensaState *env, float32 a, float32 b, float32 c)
{
    return float32_muladd(b, c, a, 0,
            &env->fp_status);
}

float32 HELPER(msub_s)(CPUXtensaState *env, float32 a, float32 b, float32 c)
{
    return float32_muladd(b, c, a, float_muladd_negate_product,
            &env->fp_status);
}

float32 HELPER(maddn_s)(float32 a, float32 b, float32 c)
{
    float_status fp_status = {0};

    set_float_rounding_mode(float_round_nearest_even, &fp_status);
    return float32_muladd(b, c, a, 0, &fp_status);
}

static float32 nexp_base_s(float32 a)
{
    if (float32_is_infinity(a)) {
        return make_float32((float32_val(a) & 0x80000000) | 0x3f800000);
    } else if (float32_is_any_nan(a)) {
        return make_float32((float32_val(a) & 0x807fffff) | 0x7f800000);
    } else if (float32_is_zero(a)) {
        return make_float32((float32_val(a) & 0x80000000) | 0x3f800000);
    } else {
        uint32_t s = float32_val(a) & 0x80000000;
        uint32_t e = (float32_val(a) & 0x7f800000) >> 23;
        uint32_t m = float32_val(a) & 0x007fffff;

        if (e == 0) {
            uint32_t d = clz32(m) - 8;

            m = (m << d) & 0x007fffff;
            e = 1 - d;
        }
        return make_float32(s | ((128 - (e & 0x1)) << 23) | m);
    }
}

static const uint8_t div0[] = {
255, 253, 251, 249, 247, 245, 244, 242, 240, 238, 237, 235, 233, 232, 230, 228,
227, 225, 224, 222, 221, 219, 218, 216, 215, 213, 212, 211, 209, 208, 207, 205,
204, 203, 202, 200, 199, 198, 197, 196, 194, 193, 192, 191, 190, 189, 188, 187,
186, 185, 184, 183, 182, 181, 180, 179, 178, 177, 176, 175, 174, 173, 172, 171,
170, 169, 168, 168, 167, 166, 165, 164, 163, 163, 162, 161, 160, 159, 159, 158,
157, 156, 156, 155, 154, 153, 153, 152, 151, 151, 150, 149, 149, 148, 147, 147,
146, 145, 145, 144, 143, 143, 142, 142, 141, 140, 140, 139, 139, 138, 137, 137,
136, 136, 135, 135, 134, 133, 133, 132, 132, 131, 131, 130, 130, 129, 129, 129
};

float32 HELPER(div0_s)(float32 a)
{
    float32 norm = nexp_base_s(a);
    uint32_t s = float32_val(norm) & 0x80000000;
    uint32_t e = float32_val(norm) & 0x00800000;
    uint32_t m = (div0[(float32_val(norm) & 0x007f0000) >> 16] & 0x7f) << 16;

    return make_float32(s | (0x3e800000 + e) | m);
}

float32 HELPER(nexp01_s)(float32 a)
{
    return float32_chs(nexp_base_s(a));
}

#define EXP_ZIN 0xff

static uint32_t mkdadj_exp(float32 a)
{
    if (float32_is_infinity(a) ||
        float32_is_any_nan(a) ||
        float32_is_zero(a)) {
        return EXP_ZIN;
    } else {
        uint32_t e = (float32_val(a) & 0x7f800000) >> 23;
        uint32_t m = float32_val(a) & 0x007fffff;

        if (e == 0) {
            e = 9 - clz32(m);
        }
        return (e - 127) & ~1;
    }
}

float32 HELPER(mkdadj_s)(CPUXtensaState *env, float32 a, float32 b)
{
    uint32_t expa = mkdadj_exp(a);
    uint32_t expb = mkdadj_exp(b);
    uint32_t res;
    uint32_t exp_hi;
    uint32_t exp_lo;

    if (expa == EXP_ZIN || expb == EXP_ZIN) {
        bool is_inf = (float32_is_infinity(a) &&
                       !(float32_is_infinity(b) || float32_is_any_nan(b))) ||
            (expa != EXP_ZIN && float32_is_zero(b));
        bool is_nan = float32_is_any_nan(a) || float32_is_any_nan(b) ||
            (float32_is_zero(a) && float32_is_zero(b)) ||
            (float32_is_infinity(a) && float32_is_infinity(b));
        bool is_neg = float32_is_neg(a) ^ float32_is_neg(b);

        res = 0x180 | (is_nan << 2) | (is_inf << 1) | is_neg;
    } else {
        res = expa - expb;
    }
    exp_hi = (((res & 0x3e0) >> 2) + 127) & 0xff;
    exp_lo = (((res & 0x01f) << 3) + 127) & 0xff;

    return make_float32((exp_lo << 23) | (exp_hi << 14));
}

uint32_t HELPER(ftoi_s)(float32 v, uint32_t rounding_mode, uint32_t scale)
{
    float_status fp_status = {0};

    set_float_rounding_mode(rounding_mode, &fp_status);
    return float32_to_int32(
            float32_scalbn(v, scale, &fp_status), &fp_status);
}

uint32_t HELPER(ftoui_s)(float32 v, uint32_t rounding_mode, uint32_t scale)
{
    float_status fp_status = {0};
    float32 res;

    set_float_rounding_mode(rounding_mode, &fp_status);

    res = float32_scalbn(v, scale, &fp_status);

    if (float32_is_neg(v) && !float32_is_any_nan(v)) {
        return float32_to_int32(res, &fp_status);
    } else {
        return float32_to_uint32(res, &fp_status);
    }
}

float32 HELPER(itof_s)(CPUXtensaState *env, uint32_t v, uint32_t scale)
{
    return float32_scalbn(int32_to_float32(v, &env->fp_status),
            (int32_t)scale, &env->fp_status);
}

float32 HELPER(uitof_s)(CPUXtensaState *env, uint32_t v, uint32_t scale)
{
    return float32_scalbn(uint32_to_float32(v, &env->fp_status),
            (int32_t)scale, &env->fp_status);
}

static inline void set_br(CPUXtensaState *env, bool v, uint32_t br)
{
    if (v) {
        env->sregs[BR] |= br;
    } else {
        env->sregs[BR] &= ~br;
    }
}

void HELPER(un_s)(CPUXtensaState *env, uint32_t br, float32 a, float32 b)
{
    set_br(env, float32_unordered_quiet(a, b, &env->fp_status), br);
}

void HELPER(oeq_s)(CPUXtensaState *env, uint32_t br, float32 a, float32 b)
{
    set_br(env, float32_eq_quiet(a, b, &env->fp_status), br);
}

void HELPER(ueq_s)(CPUXtensaState *env, uint32_t br, float32 a, float32 b)
{
    int v = float32_compare_quiet(a, b, &env->fp_status);
    set_br(env, v == float_relation_equal || v == float_relation_unordered, br);
}

void HELPER(olt_s)(CPUXtensaState *env, uint32_t br, float32 a, float32 b)
{
    set_br(env, float32_lt_quiet(a, b, &env->fp_status), br);
}

void HELPER(ult_s)(CPUXtensaState *env, uint32_t br, float32 a, float32 b)
{
    int v = float32_compare_quiet(a, b, &env->fp_status);
    set_br(env, v == float_relation_less || v == float_relation_unordered, br);
}

void HELPER(ole_s)(CPUXtensaState *env, uint32_t br, float32 a, float32 b)
{
    set_br(env, float32_le_quiet(a, b, &env->fp_status), br);
}

void HELPER(ule_s)(CPUXtensaState *env, uint32_t br, float32 a, float32 b)
{
    int v = float32_compare_quiet(a, b, &env->fp_status);
    set_br(env, v != float_relation_greater, br);
}


float64 HELPER(abs_d)(float64 v)
{
    return float64_abs(v);
}

float32 HELPER(cvts_d)(CPUXtensaState *env, float64 v)
{
    return float64_to_float32(v, &env->fp_status);
}

float64 HELPER(neg_d)(float64 v)
{
    return float64_chs(v);
}

float64 HELPER(add_d)(CPUXtensaState *env, float64 a, float64 b)
{
    return float64_add(a, b, &env->fp_status);
}

float64 HELPER(sub_d)(CPUXtensaState *env, float64 a, float64 b)
{
    return float64_sub(a, b, &env->fp_status);
}

float64 HELPER(mul_d)(CPUXtensaState *env, float64 a, float64 b)
{
    return float64_mul(a, b, &env->fp_status);
}

float64 HELPER(madd_d)(CPUXtensaState *env, float64 a, float64 b, float64 c)
{
    return float64_muladd(b, c, a, 0,
            &env->fp_status);
}

float64 HELPER(msub_d)(CPUXtensaState *env, float64 a, float64 b, float64 c)
{
    return float64_muladd(b, c, a, float_muladd_negate_product,
            &env->fp_status);
}

uint32_t HELPER(ftoi_d)(float64 v, uint32_t rounding_mode, uint32_t scale)
{
    float_status fp_status = {0};

    set_float_rounding_mode(rounding_mode, &fp_status);
    return float64_to_int32(
            float64_scalbn(v, scale, &fp_status), &fp_status);
}

uint32_t HELPER(ftoui_d)(float64 v, uint32_t rounding_mode, uint32_t scale)
{
    float_status fp_status = {0};
    float64 res;

    set_float_rounding_mode(rounding_mode, &fp_status);

    res = float64_scalbn(v, scale, &fp_status);

    if (float64_is_neg(v) && !float64_is_any_nan(v)) {
        return float64_to_int32(res, &fp_status);
    } else {
        return float64_to_uint32(res, &fp_status);
    }
}

float64 HELPER(itof_d)(CPUXtensaState *env, uint32_t v, uint32_t scale)
{
    return float64_scalbn(int32_to_float64(v, &env->fp_status),
            (int32_t)scale, &env->fp_status);
}

float64 HELPER(uitof_d)(CPUXtensaState *env, uint32_t v, uint32_t scale)
{
    return float64_scalbn(uint32_to_float64(v, &env->fp_status),
            (int32_t)scale, &env->fp_status);
}

void HELPER(un_d)(CPUXtensaState *env, uint32_t br, float64 a, float64 b)
{
    set_br(env, float64_unordered_quiet(a, b, &env->fp_status), br);
}

void HELPER(oeq_d)(CPUXtensaState *env, uint32_t br, float64 a, float64 b)
{
    set_br(env, float64_eq_quiet(a, b, &env->fp_status), br);
}

void HELPER(ueq_d)(CPUXtensaState *env, uint32_t br, float64 a, float64 b)
{
    int v = float64_compare_quiet(a, b, &env->fp_status);
    set_br(env, v == float_relation_equal || v == float_relation_unordered, br);
}

void HELPER(olt_d)(CPUXtensaState *env, uint32_t br, float64 a, float64 b)
{
    set_br(env, float64_lt_quiet(a, b, &env->fp_status), br);
}

void HELPER(ult_d)(CPUXtensaState *env, uint32_t br, float64 a, float64 b)
{
    int v = float64_compare_quiet(a, b, &env->fp_status);
    set_br(env, v == float_relation_less || v == float_relation_unordered, br);
}

void HELPER(ole_d)(CPUXtensaState *env, uint32_t br, float64 a, float64 b)
{
    set_br(env, float64_le_quiet(a, b, &env->fp_status), br);
}

void HELPER(ule_d)(CPUXtensaState *env, uint32_t br, float64 a, float64 b)
{
    int v = float64_compare_quiet(a, b, &env->fp_status);
    set_br(env, v != float_relation_greater, br);
}
