/* plant_iface_model.c -- THE SWAP POINT, HIL side (PRD Sec 6.5).
 *
 * Populates the shared plant_feedback_t struct from the CLA plant model's
 * last completed frame. In Phase 2, plant_iface_adc.c populates the same
 * struct from real ADC readings instead; the control code (/control)
 * never changes.
 *
 * i_rd/i_rq from the CLA are in the rotor's own synchronously-rotating
 * dq frame; plant_feedback_t wants actual three-phase rotor currents, so
 * this does the inverse Park transform using the slip angle, integrated
 * here on the CPU side (cheap trig, no need to burn CLA cycles on it).
 */

#include <math.h>
#include "dfig_params.h"
#include "dfig_model_cla_shared.h"
#include "plant_iface.h"

static float s_theta_sl = 0.0f;   /* rotor slip angle, rad, wrapped to [-pi, pi] */

static float wrap_pi(float theta)
{
    const float two_pi = 2.0f * 3.14159265358979323846f;
    while (theta > 3.14159265358979323846f)  theta -= two_pi;
    while (theta < -3.14159265358979323846f) theta += two_pi;
    return theta;
}

/* Called once per 10 kHz frame, after the CLA task from the *previous*
 * frame has completed (see dfig_model_cla.cla header on the pipeline
 * delay). w_m is this frame's measured mechanical speed (encoder
 * M-method), used both to advance the slip angle and to feed next
 * frame's CLA trigger -- see dfig_cla_trigger() below. */
void plant_iface_model(plant_feedback_t *fb, float w_m)
{
    float w_s   = DFIG_D_W_S;
    float pp    = DFIG_POLE_PAIRS;
    float slip  = (w_s - w_m * pp) / w_s;
    float w_sl  = slip * w_s;
    float cs, sn;

    s_theta_sl = wrap_pi(s_theta_sl + w_sl * DFIG_DT);
    cs = cosf(s_theta_sl);
    sn = sinf(s_theta_sl);

    /* inverse Park: i_a = i_d*cos(theta) - i_q*sin(theta), then standard
     * 2-phase -> 3-phase (amplitude-invariant, matches dq convention
     * used throughout python/dfig_hil). */
    {
        float i_alpha = cla_out_i_rd * cs - cla_out_i_rq * sn;
        float i_beta  = cla_out_i_rd * sn + cla_out_i_rq * cs;
        const float sqrt3_2 = 0.86602540378f;

        fb->i_r_abc[0] = i_alpha;
        fb->i_r_abc[1] = -0.5f * i_alpha + sqrt3_2 * i_beta;
        fb->i_r_abc[2] = -0.5f * i_alpha - sqrt3_2 * i_beta;
    }

    fb->v_dc = DFIG_V_DC;
    fb->P_s  = cla_out_p_s;
    fb->Q_s  = cla_out_q_s;
    fb->T_e  = cla_out_t_e;
    fb->src_flags = (uint16_t)(1u << PLANT_SRC_SIMULATED_BIT);
}

/* Called once per 10 kHz frame, before the CLA task is (re-)triggered
 * for the frame after next. v_rd/v_rq are this frame's rotor voltage
 * command from the control code's inverse Park/SVPWM stage. */
void dfig_cla_trigger_inputs(float v_rd, float v_rq, float v_s, float w_m)
{
    cla_in_v_rd = v_rd;
    cla_in_v_rq = v_rq;
    cla_in_v_s  = v_s;
    cla_in_w_m  = w_m;
}
