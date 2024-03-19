static float constrain(const float val, const float minval, const float maxval)
{
    return val < minval ? minval : val > maxval ? maxval : val;
}
