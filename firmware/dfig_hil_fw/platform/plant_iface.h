/* The swap point (PRD-DFIG-HIL-001 Sec 6.5). All plant feedback enters the
 * control code through this struct, whether the current source is the CLA
 * plant model (HIL mode, today) or a real rotor-side converter (Phase 2).
 *
 * The PRD assumed a second CPU core (F28379D) would own the model side and
 * write this struct; on the actual F280049C (single C28x core + CLA) the
 * CLA task owns it instead, written in CLA C and shared with the C28x
 * control code through CLA-to-CPU message RAM. Field layout is unchanged
 * from the PRD so this remains the Phase 2 swap point either way. */

#ifndef PLANT_IFACE_H
#define PLANT_IFACE_H

#include <stdint.h>

#define PLANT_SRC_SIMULATED_BIT 0u  /* src_flags bit n = 1 -> simulated */

typedef struct {
    float i_r_abc[3];      /* rotor currents        */
    float v_dc;            /* DC bus voltage        */
    float P_s, Q_s, T_e;   /* stator power, torque  */
    uint16_t src_flags;    /* bit n = 1 -> simulated */
} plant_feedback_t;

#ifdef __cplusplus
extern "C" {
#endif

/* plant_iface_model.c -- HIL-mode implementation (today). Phase 2 adds a
 * plant_iface_adc() with this same signature, selected by the E14 key
 * switch per PRD Sec 6.5; /control never changes. */
void plant_iface_model(plant_feedback_t *fb, float w_m);
void dfig_cla_trigger_inputs(float v_rd, float v_rq, float v_s, float w_s, float w_m);

#ifdef __cplusplus
}
#endif

#endif
