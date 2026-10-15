#ifndef FX_UDIV_H
#define FX_UDIV_H

#include <stdint.h>

/* Truncated division. `steps` is 1 or 2 Newton refinements.
 * 1 is exact for the hall edge (numerator below 2^25, divisor below
 * 2^16). 2 is exact for the voltage-limit and CCR products, up to
 * about 4e8. Either way the length does not grow with the quotient. */
uint32_t Fx_DivU(uint32_t n, uint32_t d, uint32_t steps);
int32_t Fx_DivS(int32_t n, int32_t d, uint32_t steps);

/* 9-bit reciprocal, one multiply, no Newton. The ±1 step removes a
 * one-count bias. The table can still miss by about 0.4% of the
 * quotient, so this is not an exact division. */
uint32_t Fx_QuotU(uint32_t n, uint32_t d);
int32_t Fx_QuotS(int32_t n, int32_t d);

#endif
