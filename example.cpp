
#include <iostream>

#include "const_velocity_gain_filter.h"

template<typename T, uint TNum>
void print(const ConstGlobalVelocityFilter<T, TNum>& filter)
{
    std::cout << "filter.values";
    for (int i = 0; i < TNum; ++i)
    {
        std::cout << " " << filter.values[i];
    }
    std::cout << std::endl;
    std::cout << "filter.velocity";
    for (int i = 0; i < TNum; ++i)
    {
        std::cout << " " << filter.velocity[i];
    }
    std::cout << std::endl;
    std::cout << std::endl;
}

int main()
{
    double ref_dt = 0.1;
    ConstGlobalVelocityFilter<double, 3> filter(
        // value
        1, ref_dt, // high gain means trust the prediction, low gain means trust the last values
        0.99, ref_dt, // high gain means trust the observation, low gain means trust the predicted values
        // velocity
        0.0, ref_dt,  // high gain means trust the prediction, low gain means trust the last values
        0.99, ref_dt  // high gain means trust the observation, low gain means trust the predicted values
    );
    double pos[] = {
        2, 2, 2,
        1, 1, 1,
    };
    print(filter);
    filter.correct(0.1, &pos[0]); 
    print(filter);
    filter.correct(0.1, &pos[3]);
    print(filter);
    filter.predict(0.1);
    print(filter);
    filter.predict(0.1);
    print(filter);
}
