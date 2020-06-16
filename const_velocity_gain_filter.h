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


template<typename T, uint TNum>
class ConstGlobalVelocityFilter
{
public:

    ConstGlobalVelocityFilter(
        T value_prediction_gain,    // high gain means trust the prediction, low gain means trust the last values
        double value_prediction_gain_dt,
        T value_correction_gain,    // high gain means trust the observation, low gain means trust the predicted values
        double value_correction_gain_dt,
        T velocity_prediction_gain, // high gain means trust the prediction, low gain means trust the last values
        double velocity_prediction_gain_dt,
        T velocity_correction_gain,  // high gain means trust the observation, low gain means trust the predicted values
        double velocity_correction_gain_dt
    )
        : value_filter(value_prediction_gain, value_prediction_gain_dt, value_correction_gain, value_correction_gain_dt)
        , velocity_filter(velocity_prediction_gain, velocity_prediction_gain_dt, velocity_correction_gain, velocity_correction_gain_dt)
        , has_last_measurement(false)
    {
        for (int i = 0; i < TNum; ++i)
        {
            // you can set another predicted velocity after instantiating
            predicted_velocity[i] = 0;
            values[i] = 0;
            velocity[i] = 0;
        }
    }

    void correct(double dt, const T* observed_values)
    {
        predict(dt);
        T observed_velocity[TNum];
        if (has_last_measurement && abs(dt) > 1e-6)
        {
            for (int i = 0; i < TNum; ++i)
            {
                observed_velocity[i] = (observed_values[i] - last_measurement[i]) / dt;
            }
            // high gain means trust the observation, low gain means trust the predicted values
            velocity_filter.correct(dt, observed_velocity);
        }
        // high gain means trust the observation, low gain means trust the predicted values
        value_filter.correct(dt, observed_values);
        copy<T, TNum>(value_filter.values, values);
        copy<T, TNum>(velocity_filter.values, velocity);
        copy<T, TNum>(observed_values, last_measurement);
        has_last_measurement = true;

    }
    void predict(double dt)
    {
        T predicted[TNum];
        for (uint i = 0; i < 3; ++i)
        {
            predicted[i] = values[i] + velocity[i] * dt;
        }
        // mix between old estimate and predicted
        // high gain means trust the prediction, low gain means trust the last values
        value_filter.predict(dt, predicted);
        velocity_filter.predict(dt, predicted_velocity);
        copy<T, TNum>(value_filter.values, values);
        copy<T, TNum>(velocity_filter.values, velocity);

    }
    T values[TNum];
    T velocity[TNum];

    ObservePredictFilter<T, TNum> value_filter;
    ObservePredictFilter<T, TNum> velocity_filter;

    T last_measurement[TNum];
    bool has_last_measurement;
    T predicted_velocity[TNum];

};

