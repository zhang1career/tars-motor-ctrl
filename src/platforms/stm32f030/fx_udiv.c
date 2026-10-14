#include "fx_udiv.h"

/* -O2 keeps the reciprocal in registers. -Os spilled a 60-byte frame. */
#pragma GCC push_options
#pragma GCC optimize ("O2")

/* (2^32 / idx) >> 9, idx = 256..511. Two Newton steps and one
 * remainder fix match truncated division for every numerator the
 * tick forms (up to about 4e8) and every divisor that is not a
 * power of two. Powers of two are a shift. */
static const uint16_t s_rcp9[256] = {
    32768, 32640, 32513, 32388, 32263, 32140, 32017, 31895, 31775, 31655, 31536, 31418,
    31300, 31184, 31068, 30954, 30840, 30727, 30615, 30504, 30393, 30283, 30174, 30066,
    29959, 29852, 29746, 29641, 29537, 29433, 29330, 29228, 29127, 29026, 28926, 28826,
    28728, 28630, 28532, 28435, 28339, 28244, 28149, 28055, 27962, 27869, 27776, 27685,
    27594, 27503, 27413, 27324, 27235, 27147, 27060, 26973, 26886, 26800, 26715, 26630,
    26546, 26462, 26379, 26296, 26214, 26132, 26051, 25970, 25890, 25811, 25731, 25653,
    25575, 25497, 25420, 25343, 25266, 25191, 25115, 25040, 24966, 24892, 24818, 24745,
    24672, 24600, 24528, 24456, 24385, 24314, 24244, 24174, 24105, 24036, 23967, 23899,
    23831, 23763, 23696, 23629, 23563, 23497, 23431, 23366, 23301, 23237, 23172, 23109,
    23045, 22982, 22919, 22857, 22795, 22733, 22671, 22610, 22550, 22489, 22429, 22369,
    22310, 22250, 22192, 22133, 22075, 22017, 21959, 21902, 21845, 21788, 21732, 21675,
    21620, 21564, 21509, 21454, 21399, 21345, 21290, 21236, 21183, 21129, 21076, 21024,
    20971, 20919, 20867, 20815, 20763, 20712, 20661, 20610, 20560, 20510, 20460, 20410,
    20360, 20311, 20262, 20213, 20164, 20116, 20068, 20020, 19972, 19925, 19878, 19831,
    19784, 19737, 19691, 19645, 19599, 19553, 19508, 19463, 19418, 19373, 19328, 19284,
    19239, 19195, 19152, 19108, 19065, 19021, 18978, 18935, 18893, 18850, 18808, 18766,
    18724, 18682, 18641, 18600, 18558, 18517, 18477, 18436, 18396, 18355, 18315, 18275,
    18236, 18196, 18157, 18117, 18078, 18040, 18001, 17962, 17924, 17886, 17848, 17810,
    17772, 17734, 17697, 17660, 17623, 17586, 17549, 17512, 17476, 17439, 17403, 17367,
    17331, 17296, 17260, 17225, 17189, 17154, 17119, 17084, 17050, 17015, 16980, 16946,
    16912, 16878, 16844, 16810, 16777, 16743, 16710, 16677, 16644, 16611, 16578, 16545,
    16513, 16480, 16448, 16416,
};

static uint32_t fx_clz(uint32_t x)
{
  uint32_t n = 0U;

  if ((x & 0xFFFF0000U) == 0U)
  {
    n += 16U;
    x <<= 16;
  }
  if ((x & 0xFF000000U) == 0U)
  {
    n += 8U;
    x <<= 8;
  }
  if ((x & 0xF0000000U) == 0U)
  {
    n += 4U;
    x <<= 4;
  }
  if ((x & 0xC0000000U) == 0U)
  {
    n += 2U;
    x <<= 2;
  }
  if ((x & 0x80000000U) == 0U)
  {
    n += 1U;
  }
  return n;
}

/* High 32 bits of a 32x32 product. Hardware muls only; no libgcc helper. */
static uint32_t fx_mulhi(uint32_t a, uint32_t b) __attribute__((noinline));
static uint32_t fx_mulhi(uint32_t a, uint32_t b)
{
  uint32_t a0 = a & 0xFFFFU;
  uint32_t a1 = a >> 16;
  uint32_t b0 = b & 0xFFFFU;
  uint32_t b1 = b >> 16;
  uint32_t p00 = a0 * b0;
  uint32_t p01 = a0 * b1;
  uint32_t p10 = a1 * b0;
  uint32_t p11 = a1 * b1;
  uint32_t mid = (p00 >> 16) + (p01 & 0xFFFFU) + (p10 & 0xFFFFU);

  return p11 + (p01 >> 16) + (p10 >> 16) + (mid >> 16);
}

uint32_t Fx_DivU(uint32_t n, uint32_t d, uint32_t steps) __attribute__((noinline));
uint32_t Fx_DivU(uint32_t n, uint32_t d, uint32_t steps)
{
  uint32_t shift;
  uint32_t dn;
  uint32_t idx;
  uint32_t full;
  uint32_t recip;
  uint32_t q;
  uint32_t i;
  int32_t k;

  if (d == 0U)
  {
    return 0U;
  }
  shift = fx_clz(d);
  if ((d & (d - 1U)) == 0U)
  {
    return n >> (31U - shift);
  }

  dn = d << shift;
  idx = dn >> 23;
  full = ((uint32_t)s_rcp9[idx - 256U]) << 9;
  k = (int32_t)shift - 23;
  if (k >= 0)
  {
    recip = full << (uint32_t)k;
  }
  else
  {
    recip = full >> (uint32_t)(-k);
  }

  for (i = 0U; i < steps; i++)
  {
    uint32_t lo = recip * d;
    uint32_t hi = fx_mulhi(recip, d);
    uint32_t err = 0U - lo;

    recip = recip + fx_mulhi(recip, err) - recip * hi;
  }
  q = fx_mulhi(n, recip);
  if (((q + 1U) * d) <= n)
  {
    q++;
  }
  if ((q * d) > n)
  {
    q--;
  }
  return q;
}

int32_t Fx_DivS(int32_t n, int32_t d, uint32_t steps)
{
  uint32_t neg = 0U;
  uint32_t un;
  uint32_t ud;
  uint32_t q;

  if (d == 0)
  {
    return 0;
  }
  if (n < 0)
  {
    neg ^= 1U;
    un = (uint32_t)(-n);
  }
  else
  {
    un = (uint32_t)n;
  }
  if (d < 0)
  {
    neg ^= 1U;
    ud = (uint32_t)(-d);
  }
  else
  {
    ud = (uint32_t)d;
  }
  q = Fx_DivU(un, ud, steps);
  if (neg != 0U)
  {
    return -(int32_t)q;
  }
  return (int32_t)q;
}

#pragma GCC pop_options
