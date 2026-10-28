/* Declarations shared between the C28x control code and the CLA plant
 * model task (dfig_model_cla.cla), mirroring the pattern C2000Ware's own
 * cla examples use for their *_shared.h files. */

#ifndef DFIG_MODEL_CLA_SHARED_H
#define DFIG_MODEL_CLA_SHARED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CPU -> CLA inputs. Written by the C28x ISR before triggering task 1. */
extern float cla_in_v_rd;
extern float cla_in_v_rq;
extern float cla_in_v_s;
extern float cla_in_w_m;

/* CLA -> CPU outputs. Read by the C28x ISR at the top of the *next*
 * frame (one 100 us pipeline delay -- see dfig_model_cla.cla header). */
extern float cla_out_i_rd;
extern float cla_out_i_rq;
extern float cla_out_p_s;
extern float cla_out_q_s;
extern float cla_out_t_e;
extern int cla_out_fault;   /* 0 = none, 1 = clamp -- plain int to match the .cla side */

/* CLA tasks, defined in dfig_model_cla.cla */
__attribute__((interrupt)) void Cla1Task1(void);
__attribute__((interrupt)) void Cla1Task7(void);

#ifdef __cplusplus
}
#endif

#endif
