#pragma once
#include <cmath>

class AnaliticalSolver {
public:
    // Расчет точного времени падения (ToF) с учетом сопротивления воздуха
    static inline float calcTimeOfFlight(float Z0, float V0, float m, float d, float l) {
        float a = d * 9.81f * m - 2 * d * d * l * V0;
        float b = -3 * 9.81f * m * m + 3 * d * l * m * V0;
        float c = 6 * m * m * Z0;

        // Защита от деления на ноль: если коэффициент 'a' ничтожно мал
        if (std::fabs(a) < 1e-12f)
            return std::sqrt(2.0f * Z0 / 9.81f);

        float p = -b * b / (3 * a * a);
        float q = (2 * b * b * b) / (27 * a * a * a) + c / a;

        if (p >= 0)
            return std::sqrt(2.0f * Z0 / 9.81f);

        float arg = 3 * q / (2 * p) * std::sqrt(-3 / p);
        if (std::fabs(arg) > 1)
            return std::sqrt(2.0f * Z0 / 9.81f);

        float phi = std::acos(arg);
        float t   = 2 * std::sqrt(-p / 3) * std::cos((phi + 4 * static_cast<float>(M_PI)) / 3) - b / (3 * a);
        
        return t > 0 ? t : std::sqrt(2.0f * Z0 / 9.81f);
    }

    // Расчет горизонтального смещения (баллистического выноса)
    static inline float calcHDistance(float t, float V0, float m, float d, float l) {
        float l2 = l * l;
        float l4 = l2 * l2;

        float Linear_component    = t * V0;
        float Quadratic_component = - (d * std::pow(t, 2) * V0) / (2 * m);
        float Cubic_component     = (std::pow(t, 3) * (6 * d * 9.81f * l * m - 6 * std::pow(d, 2) * (-1 + l2) * V0)) / (36 * std::pow(m, 2));
        float fourth_component    = (std::pow(t, 4) * (-6 * std::pow(d, 2) * 9.81f * l * (1 + l2 + l4) * m 
                                    + 3 * std::pow(d, 3) * l2 * (1 + l2) * V0 
                                    + 6 * std::pow(d, 3) * l4 * (1 + l2) * V0)) 
                                    / (36 * std::pow(1 + l2, 2) * std::pow(m, 3));
        float fifth_component     = (std::pow(t, 5) * (3 * std::pow(d, 3) * 9.81f * std::pow(l, 3) * m 
                                    - 3 * std::pow(d, 4) * l2 * (1 + l2) * V0)) 
                                    / (36 * (1 + l2) * std::pow(m, 4));

        float total_distance = Linear_component + Quadratic_component + Cubic_component + fourth_component + fifth_component;
        return total_distance;

        

        //return t * V0
            //- (d * std::pow(t, 2) * V0) / (2 * m)
            //+ (std::pow(t, 3) * (6 * d * 9.81f * l * m
            //   - 6 * std::pow(d, 2) * (-1 + l2) * V0)) / (36 * std::pow(m, 2))
            //+ (std::pow(t, 4) * (-6 * std::pow(d, 2) * 9.81f * l * (1 + l2 + l4) * m
            //   + 3 * std::pow(d, 3) * l2 * (1 + l2) * V0
            //   + 6 * std::pow(d, 3) * l4 * (1 + l2) * V0))
            //  / (36 * std::pow(1 + l2, 2) * std::pow(m, 3))
            //+ (std::pow(t, 5) * (3 * std::pow(d, 3) * 9.81f * std::pow(l, 3) * m
            //   - 3 * std::pow(d, 4) * l2 * (1 + l2) * V0))
            //  / (36 * (1 + l2) * std::pow(m, 4));
    }
};