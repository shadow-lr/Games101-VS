//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include <chrono>
#include "Scene.hpp"
#include "Renderer.hpp"

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00016f;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    //int m = 0;

    // change the spp value to change sample ammount
    int spp = 500;
    std::cout << "SPP: " << spp << "\n";

    int finish_num = 0;
    int all_num = scene.height * scene.width;
    int per_thread_num = all_num / thread_num;

    // 每疫苗更新一次Progress
    int time_interval = 3;

    auto clock_start = std::chrono::system_clock::now();

    bool bOpenMSAA = false;

    int msaa_sample = 2;
    int mass_sample2 = msaa_sample * msaa_sample;

    std::vector<float> offset;

    for (int i = 0; i < msaa_sample; ++i)
    {
        offset.push_back((0.5 + i) * 1.0 / static_cast<float>(msaa_sample));
    }

    omp_set_num_threads(thread_num);
#pragma omp parallel
    {
#pragma omp for
		for (int j = 0; j < scene.height; ++j) {
			for (int i = 0; i < scene.width; ++i) {
				int write_index = j * scene.width + i;
				if (bOpenMSAA)
				{
					for (int a = 0; a < msaa_sample; ++a) {
						for (int b = 0; b < msaa_sample; ++b) {
							// generate primary ray direction
							float x = (2 * (i + offset[a]) / (float)scene.width - 1) * imageAspectRatio * scale;
							float y = (1 - 2 * (j + offset[b]) / (float)scene.height) * scale;

							Vector3f dir = normalize(Vector3f(-x, y, 1));


							for (int k = 0; k < spp; k++)
							{
								framebuffer[write_index] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
							}
						}
					}
                    framebuffer[write_index] = framebuffer[write_index] / mass_sample2;
				}
                else
                {
                    float x = (2 * (i + 0.5) / (float)scene.width - 1) * imageAspectRatio * scale;
                    float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

                    Vector3f dir = normalize(Vector3f(-x, y, 1));

                    for (int k = 0; k < spp; k++)
                    {
                        framebuffer[write_index] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
                    }
                }
				
#pragma omp critical
				finish_num += 1;

				thread_finish_count[omp_get_thread_num()] += 1;

				auto clock_now = std::chrono::system_clock::now();
				auto interval = std::chrono::duration_cast<std::chrono::seconds>(clock_now - clock_start).count();
				if (interval >= time_interval)
				{
					UpdateAllProgress(finish_num / (float)all_num, finish_num, all_num, per_thread_num);
					clock_start = clock_now;
				}
			}
		}
	}
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary_pt_CookTorrance_Glass2.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
