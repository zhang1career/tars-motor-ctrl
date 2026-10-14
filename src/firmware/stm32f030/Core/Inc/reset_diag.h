#ifndef RESET_DIAG_H
#define RESET_DIAG_H

#include <stdint.h>

/* Survives reset. Startup does not clear .noinit. */
#define RESET_DIAG_MAGIC 0xD1A60113U

enum
{
  RESET_DIAG_MAGIC_I = 0,
  RESET_DIAG_RCC_CSR,
  RESET_DIAG_FAULT,
  RESET_DIAG_FAULT_PC,
  RESET_DIAG_TICK_CYC_MAX,
  RESET_DIAG_TICK_OVERRUN,
  RESET_DIAG_TICK_TOTAL,
  RESET_DIAG_MAIN_TICK_SEEN,
  RESET_DIAG_TICK_GAP,
  RESET_DIAG_TICK_CYC_SUM,
  /* First trip wins until the next one. Not cleared on a matching magic. */
  RESET_DIAG_OCP_KIND,   /* 1 software or forced, 2 BKIN */
  RESET_DIAG_OCP_PHASE,  /* 0..2, or 0xFFFFFFFF if the trip had no phase */
  RESET_DIAG_OCP_LSB,    /* signed raw-2048 of that phase */
  RESET_DIAG_OCP_CTX,    /* hall in bits 7:0, kick in bit 8 */
  RESET_DIAG_OCP_TICK,   /* HAL_GetTick at the trip */
  RESET_DIAG_WORDS
};

extern volatile uint32_t g_reset_diag[RESET_DIAG_WORDS];

#endif /* RESET_DIAG_H */
