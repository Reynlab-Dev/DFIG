//#############################################################################
//
// FILE:   main.c
//
// TITLE:  DFIG HIL Trainer firmware, P1 skeleton (F280049C LaunchPad).
//
// Sets up a 10 kHz (100 us) CPU Timer 0 ISR. Each frame:
//   1. Reads the CLA plant model's output from the PREVIOUS frame into
//      plant_feedback_t (plant_iface_model(), the PRD Sec 6.5 swap point).
//   2. [TODO, student-editable per PRD Sec 6.4] control code: read
//      fb->i_r_abc etc., run Park/Clarke + PI + inverse Park/SVPWM,
//      produce v_rd*/v_rq* and real ePWM duty cycles.
//   3. Writes this frame's rotor voltage command and the latest measured
//      speed into the CLA's inputs and re-triggers task 1, so the model
//      output is ready for step 1 of the frame after next.
//
// This file and dfig_model_cla.cla are the two halves of the architecture
// decision in DFIG_HANDOFF.md: the PRD assumed a second CPU core
// (F28379D) for the model; this board (F280049C) has one C28x core plus
// a CLA co-processor, so the CLA plays that role instead. See the header
// comment in dfig_model_cla.cla for the one-frame pipeline delay this
// implies, and plant_iface.h for why the struct boundary is unchanged.
//
//#############################################################################

#include "f28x_project.h"
#include "dfig_params.h"
#include "dfig_model_cla_shared.h"
#include "plant_iface.h"

// Linker-defined CLA program/const load symbols (flash -> CLA RAM copy)
extern uint32_t Cla1ProgRunStart, Cla1ProgLoadStart, Cla1ProgLoadSize;

static plant_feedback_t g_plant_feedback;
static volatile uint32_t g_frame_count = 0;

// Debug-only timing instrumentation, for scope/logic-analyzer measurement
// once this runs on real hardware -- no simulator or hardware is available
// to this session, so this is deliberately NOT a claimed/measured number
// anywhere in the repo; GPIO16 traces the CPU-side ISR (the "control"
// budget), GPIO17 traces the CLA task's actual duration (the "model"
// budget). Both are boosterpack header pins, scope-probeable without
// disturbing anything else on the LaunchPad. Remove once real numbers
// are in and this is no longer needed.
#define DEBUG_GPIO_ISR   16
#define DEBUG_GPIO_CLA   17

static void initCLA(void);
static void initDebugGpio(void);
static void initHeartbeatSci(void);
static void sciPutString(const char *s);
static void sciPutU32(uint32_t v);
__interrupt void cpuTimer0ISR(void);
__interrupt void cla1Task1DoneISR(void);

void main(void)
{
    InitSysCtrl();
    InitGpio();

    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    EALLOW;
    PieVectTable.TIMER0_INT = &cpuTimer0ISR;
    PieVectTable.CLA1_1_INT = &cla1Task1DoneISR;
    EDIS;

    initDebugGpio();
    initHeartbeatSci();
    initCLA();

    InitCpuTimers();
    // 100 MHz SYSCLK assumed (F280049C default PLL config via InitSysCtrl);
    // 100 us period -> 10 kHz frame, matches DFIG_DT in dfig_params.h.
    ConfigCpuTimer(&CpuTimer0, 100, 100);
    CpuTimer0Regs.TCR.all = 0x4000;   // start timer 0

    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;   // TIMER0
    PieCtrlRegs.PIEIER11.bit.INTx1 = 1;  // CLA1_1 (task 1 end-of-task, debug only)
    IER |= M_INT1;
    IER |= M_INT11;

    EINT;
    ERTM;

    // Debug only: prints g_frame_count over the XDS110's own USB-serial
    // bridge (SCI-A, GPIO28/29 -> onboard "Application/User UART" COM
    // port) once per second, so the running firmware can be confirmed
    // alive without a scope/logic analyzer and without disturbing state
    // over JTAG (a JTAG connect forces this device's DCSM security-init,
    // which resets the core -- this path sidesteps that entirely).
    // Remove once a scope is available and GPIO16/17 are used instead.
    uint32_t next_print = 10000;
    for (;;)
    {
        if (g_frame_count >= next_print)
        {
            sciPutString("frame_count=");
            sciPutU32(g_frame_count);
            sciPutString("\r\n");
            next_print = g_frame_count + 10000;
        }
    }
}

static void initHeartbeatSci(void)
{
    EALLOW;
    GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 1);   // GPIO28 = SCIRXDA
    GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 1);   // GPIO29 = SCITXDA
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_ASYNC);
    EDIS;

    SciaRegs.SCICCR.all = 0x0007;             // 1 stop bit, no loopback, 8-bit
    SciaRegs.SCICTL1.all = 0x0003;            // enable TX, RX, idle-line mode
    SciaRegs.SCICTL2.all = 0x0003;

    // 9600 baud at LSPCLK = SYSCLKOUT/4 = 25 MHz (default InitSysCtrl PLL
    // config + default LOSPCP divider).
    SciaRegs.SCIHBAUD.all = 0x0001;
    SciaRegs.SCILBAUD.all = 0x0044;

    SciaRegs.SCICTL1.all = 0x0023;            // relinquish SCI from reset

    SciaRegs.SCIFFTX.all = 0xE040;
    SciaRegs.SCIFFRX.all = 0x2044;
    SciaRegs.SCIFFCT.all = 0x0;
}

static void sciPutString(const char *s)
{
    while (*s != '\0')
    {
        while (SciaRegs.SCIFFTX.bit.TXFFST != 0)
        {
            // wait for space in the TX FIFO
        }
        SciaRegs.SCITXBUF.all = (uint16_t)*s;
        s++;
    }
}

static void sciPutU32(uint32_t v)
{
    char digits[11];
    int i = 0;

    if (v == 0)
    {
        sciPutString("0");
        return;
    }

    while (v > 0 && i < 10)
    {
        digits[i++] = (char)('0' + (v % 10));
        v /= 10;
    }

    while (i > 0)
    {
        char c[2];
        i--;
        c[0] = digits[i];
        c[1] = '\0';
        sciPutString(c);
    }
}

static void initDebugGpio(void)
{
    EALLOW;
    GPIO_SetupPinMux(DEBUG_GPIO_ISR, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(DEBUG_GPIO_ISR, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(DEBUG_GPIO_CLA, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(DEBUG_GPIO_CLA, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO17 = 1;
    EDIS;
}

static void initCLA(void)
{
#if defined(_FLASH)
    memcpy((uint32_t *)&Cla1ProgRunStart, (uint32_t *)&Cla1ProgLoadStart,
           (uint32_t)&Cla1ProgLoadSize);
#endif

    EALLOW;

    // CLA program in RAMLS0, data in RAMLS1 -- CLA is master of both.
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS0 = 1;
    MemCfgRegs.LSxMSEL.bit.MSEL_LS0 = 1;
    MemCfgRegs.LSxMSEL.bit.MSEL_LS1 = 1;

#pragma diag_suppress=770
    Cla1Regs.MVECT1 = (uint16_t)&Cla1Task1;
    Cla1Regs.MVECT7 = (uint16_t)&Cla1Task7;
#pragma diag_warning=770

    // Task 1 is software-triggered from cpuTimer0ISR, not by a peripheral.
    DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK1 = 0;   // software
    DmaClaSrcSelRegs.CLA1TASKSRCSEL2.bit.TASK7 = 0;   // software

    Cla1Regs.MIER.bit.INT1 = 1;
    Cla1Regs.MIER.bit.INT7 = 1;
    Cla1Regs.MCTL.bit.IACKE = 1;

    // Run the one-time coefficient-init task (task 7) now, before the
    // timer ISR starts triggering task 1.
    Cla1Regs.MIFRC.bit.INT7 = 1;

    EDIS;
}

__interrupt void cpuTimer0ISR(void)
{
    float w_m_meas;

    GpioDataRegs.GPASET.bit.GPIO16 = 1;   // debug: ISR start

    CpuTimer0.InterruptCount++;
    g_frame_count++;

    // TODO (Exercise 2 / protection.c): replace with the real eQEP
    // M-method speed estimate. Zero for now so this skeleton builds and
    // runs without the encoder wired up.
    w_m_meas = 0.0f;

    // Step 1: consume the CLA's output from the frame triggered last time
    // through this ISR (one-frame pipeline delay -- see header comment).
    plant_iface_model(&g_plant_feedback, w_m_meas);

    // Step 2 [TODO, /control]: Park/Clarke + PI + inverse Park/SVPWM on
    // g_plant_feedback -> v_rd_cmd, v_rq_cmd. Zero for now.
    //
    // w_s is nominal grid frequency until grid_events.c (sag/swell/freq
    // deviation injection, PRD Sec 6.4) is ported -- TODO.
    dfig_cla_trigger_inputs(/*v_rd=*/0.0f, /*v_rq=*/0.0f,
                             /*v_s=*/DFIG_D_V_S_PK_RATED,
                             /*w_s=*/DFIG_D_W_S, /*w_m=*/w_m_meas);

    // Step 3: re-trigger the CLA task for next frame's result.
    GpioDataRegs.GPASET.bit.GPIO17 = 1;   // debug: CLA task start
    Cla1Regs.MIFRC.bit.INT1 = 1;

    GpioDataRegs.GPACLEAR.bit.GPIO16 = 1; // debug: ISR end (CPU-side/"control" budget)
    PieCtrlRegs.PIEACK.bit.ACK1 = 1;
}

// Debug only: fires when the CLA finishes task 1. The pulse width on
// GPIO17 (set in cpuTimer0ISR above, cleared here) is the CLA model's
// actual per-frame execution time -- the number to compare against the
// ~25 us model budget from DFIG_HANDOFF.md, once this runs on hardware.
__interrupt void cla1Task1DoneISR(void)
{
    GpioDataRegs.GPACLEAR.bit.GPIO17 = 1;
    PieCtrlRegs.PIEACK.bit.ACK11 = 1;
}
