#pragma once
#include <iostream>
#include <cmath>
#include <random>
#include <omp.h>

#undef M_PI
#define M_PI 3.141592653589793f

const int thread_num = 32;

static int thread_finish_count[thread_num + 1]{ 0 };

static omp_lock_t lock;

extern const float  EPSILON;
const float kInfinity = std::numeric_limits<float>::max();

inline float clamp(const float &lo, const float &hi, const float &v)
{ return std::max(lo, std::min(hi, v)); }

inline  bool solveQuadratic(const float &a, const float &b, const float &c, float &x0, float &x1)
{
    float discr = b * b - 4 * a * c;
    if (discr < 0) return false;
    else if (discr == 0) x0 = x1 = - 0.5 * b / a;
    else {
        float q = (b > 0) ?
                  -0.5 * (b + sqrt(discr)) :
                  -0.5 * (b - sqrt(discr));
        x0 = q / a;
        x1 = c / q;
    }
    if (x0 > x1) std::swap(x0, x1);
    return true;
}

inline float get_random_float()
{
    static std::random_device dev;
    static std::mt19937 rng(dev());
    static std::uniform_real_distribution<float> dist(0.f, 1.f); // distribution in range [1, 6]

    return dist(rng);
}

inline void UpdateProgress(float progress)
{
    int barWidth = 70;
    printf("[");
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) printf("=");
        else if (i == pos) printf(">");
        else printf(" ");
    }
    printf("] %d %\r", int(progress * 100.0));
    fflush(stdout);
};

inline void UpdateAllProgress(float progress,int all_num)
{
    int barWidth = 70;
    for (int i = 0; i < thread_num; ++i)
    {
        if (i % 4 == 0 && i != 0) printf("\n");
        printf("Thread %d : ", omp_get_thread_num());
        int pos = barWidth * thread_finish_count[omp_get_thread_num()] / (float)all_num;
        for (int j = 0; j < barWidth; ++j)
        {
            if (j < pos)    printf("=");
            else if (j == pos) printf(">");
            else printf(" ");
        }
        printf("] %d %\r", int(progress * 100.0));
    }
    fflush(stdout);
}
