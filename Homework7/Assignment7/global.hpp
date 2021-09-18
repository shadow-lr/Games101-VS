#pragma once
#include <iostream>
#include <cmath>
#include <random>
#include <omp.h>

#undef M_PI
#define M_PI 3.141592653589793f

const int thread_num = 8;

static int thread_finish_count[thread_num + 1]{ 0 };

static omp_lock_t lock;

extern const float  EPSILON;
const float kInfinity = std::numeric_limits<float>::max();

inline float clamp(const float& lo, const float& hi, const float& v)
{
	return std::max(lo, std::min(hi, v));
}

inline  bool solveQuadratic(const float& a, const float& b, const float& c, float& x0, float& x1)
{
	float discr = b * b - 4 * a * c;
	if (discr < 0) return false;
	else if (discr == 0) x0 = x1 = -0.5 * b / a;
	else
	{
		float q = (b >= -EPSILON) ? -0.5 * (b + sqrt(discr)) : -0.5 * (b - sqrt(discr));
		x0 = q / a;
		x1 = c / q;
	}
	if (x0 - x1 >= -EPSILON) std::swap(x0, x1);
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

inline void UpdateAllProgress(float progress, int finish_num, int all_num, int per_thread_num)
{
#pragma omp critical
	{
		system("cls");
		printf("\n");
		for (int i = 0; i < thread_num; ++i)
		{
			if (i % 4 == 0 && i != 0) printf("\n");

			printf("%3d  [", i);
			float curThreadProgress = thread_finish_count[i] / (float)per_thread_num;
			int present = int(curThreadProgress * 100.0);
            int per_present_flag = 10;
			int flag_num = present / per_present_flag;

			for (int j = 0; j < per_present_flag; ++j)
			{
				if (j < flag_num) printf("|");
				else printf(" ");
			}

			printf("%-3.1f%%]", (fminf(curThreadProgress * 100.0, 100.0)));
            printf("\t");
		}
        printf("\nAll  [");

        int total_present = int(progress * 100.0);
        int per_present_flag = 10;
        int flag_num = total_present / per_present_flag;

        for (int j = 0; j < per_present_flag; ++j)
        {
            if (j < flag_num) printf("|");
            else printf(" ");
        }

        printf("%d/%d]", finish_num, all_num);
        printf("\t");
        printf("Tasks: %d; %d running", thread_num, omp_get_num_procs());

        fflush(stdout);
	}
}
