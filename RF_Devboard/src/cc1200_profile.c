#include "cc1200_profile.h"

bool cc1200_apply_profile(cc1200_t* dev, const cc1200_regpair_t* p, size_t n)
{
    if (!dev || !p || n == 0) return false;

    for (size_t i = 0; i < n; i++)
    {
        bool ok = false;
        if (p[i].is_ext)
            ok = cc1200_write_ext(dev, p[i].addr, p[i].val);
        else
            ok = cc1200_write_reg(dev, p[i].addr, p[i].val);

        if (!ok) return false;
    }
    return true;
}
