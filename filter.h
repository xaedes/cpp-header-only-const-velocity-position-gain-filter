#pragma once

#include <math.h>


typedef unsigned int uint;


template<typename T, uint TNum>
void copy(const T* source, T* destination)
{
    for (int i = 0; i < TNum; ++i)
    {
        destination[i] = source[i];
    }
}


template<typename T, uint TNum>
class GainFilter
{
public:
    GainFilter(T gain, double reference_dt)
        : gain(gain)
        , reference_dt(reference_dt)
        , has_values(false)
    {}

    GainFilter& filter(const T* new_values)
    {
        if (has_values)
        {
            for (uint i = 0; i < TNum; ++i)
            {
                values[i] = new_values[i] * gain + (1 - gain) * values[i];
            }
        }
        else
        {
            copyFrom(new_values);
            has_values = true;
        }
        return *this;
    }
    GainFilter& filter(double dt, const T* new_values)
    {
        if (has_values)
        {
            T the_gain = gain_for_dt(dt);
            for (uint i = 0; i < TNum; ++i)
            {
                values[i] = new_values[i] * the_gain + (1 - the_gain) * values[i];
            }
        }
        else
        {
            copyFrom(new_values);
            has_values = true;
        }
        return *this;
    }
    GainFilter& copyTo(T* target)
    {
        copy<T, TNum>(values, target);
        return *this;
    }

    GainFilter& copyFrom(const T* source)
    {
        copy<T, TNum>(source, values);
        return *this;
    }

    T gain_for_dt(double dt)
    {
        return dt / (((reference_dt * (1 - gain)) / gain) + dt);
    }

    T values[TNum];
    T gain;

    bool has_values;
    double reference_dt; // time always as double
};


template<typename T, uint TNum>
class ObservePredictFilter
{
public:
    ObservePredictFilter(T prediction_gain, double prediction_gain_dt, T correction_gain, double correction_gain_dt)
        : prediction_filter(prediction_gain, prediction_gain_dt)
        , correction_filter(correction_gain, correction_gain_dt)

    {
        for (int i = 0; i < TNum; ++i)
        {
            values[i] = 0;
        }
    }

    void correct(double dt, const T* observed)
    {
        correction_filter.copyFrom(values).filter(dt, observed).copyTo(values);
    }
    void predict(double dt, const T* prediction)
    {
        prediction_filter.copyFrom(values).filter(dt, prediction).copyTo(values);
    }

    T values[TNum];
    GainFilter<T, TNum> prediction_filter;
    GainFilter<T, TNum> correction_filter;
}; 
