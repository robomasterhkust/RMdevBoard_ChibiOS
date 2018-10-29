#include "math_misc.h"
//write all non-inline function here

void lpfilter_init(lpfilterStruct* const lp,
  const float sample_freq, const float cutoff_freq)
{
  lp->data[0] = 0.0f;
  lp->data[1] = 0.0f;

  float fr = sample_freq / cutoff_freq;
  float ohm = tanf(M_PI / fr);
  float c = 1.0f + 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm;
  lp->b0 = ohm * ohm / c;
  lp->b1 = 2.0f * lp->b0;
  lp->b2 = lp->b0;
  lp->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
  lp->a2 = (1.0f - 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm) / c;
}

float lpfilter_apply(lpfilterStruct* const lp, const float input)
{
  float delay = input - lp->data[0] * lp->a1 - lp->data[1] * lp->a2;

  // don't allow bad values to propagate via the filter
  if(!isfinite(delay))
    delay = input;

  float output = delay * lp->b0 + lp->data[0] * lp->b1 + lp->data[1] * lp->b2;

  lp->data[1] = lp->data[0];
  lp->data[0] = delay;

  // return the value.  Should be no need to check limits
  return output;
}

bool threshold_count(const bool statement, const uint16_t count, uint16_t* const curr_count)
{
  if(statement && *curr_count < 65534)
    *curr_count += 1;
  else
    *curr_count = 0;

  if(*curr_count >= count)
  {
    *curr_count = 0;
    return true;
  }
  else
    return false;
}

bool state_count(const bool statement, const uint16_t count, uint16_t* const curr_count)
{
  if(statement && *curr_count < 65534)
    *curr_count += 1;
  else
    *curr_count = 0;

  if(*curr_count >= count)
  {
    *curr_count = 0;
    return true;
  }
  else
    return false;
}
