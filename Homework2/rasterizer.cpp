// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

#define ssaa_sample 2

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f>& positions) {
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return { id };
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i>& indices) {
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return { id };
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f>& cols) {
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return { id };
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f) {
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(double x, double y, const Vector3f* _v) {
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]

    Vector2f v01 = Vector2f(_v[1].x() - _v[0].x(), _v[1].y() - _v[0].y());
    Vector2f v12 = Vector2f(_v[2].x() - _v[1].x(), _v[2].y() - _v[1].y());
    Vector2f v20 = Vector2f(_v[0].x() - _v[2].x(), _v[0].y() - _v[2].y());

    Vector2f v0P = Vector2f(x - _v[0].x(), y - _v[0].y());
    Vector2f v1P = Vector2f(x - _v[1].x(), y - _v[1].y());
    Vector2f v2P = Vector2f(x - _v[2].x(), y - _v[2].y());

    double c1 = v01.x() * v0P.y() - v01.y() * v0P.x();
    double c2 = v12.x() * v1P.y() - v12.y() * v1P.x();
    double c3 = v20.x() * v2P.y() - v20.y() * v2P.x();

    if ((c1 >= 0 && c2 >= 0 && c3 >= 0) || (c1 <= 0 && c2 <= 0 && c3 <= 0))
        return true;

    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v) {
    float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) /
        (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() -
            v[2].x() * v[1].y());
    float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) /
        (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() -
            v[0].x() * v[2].y());
    float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) /
        (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() -
            v[1].x() * v[0].y());
    return { c1, c2, c3 };
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type) {
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind) {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto& vert : v) {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i) {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    int xmin = MIN(MIN(floor(v[0].x()), floor(v[0].x())), floor(v[2].x()));
    int xmax = MAX(MAX(floor(v[0].x()), floor(v[1].x())), floor(v[2].x()));

    int ymin = MIN(MIN(floor(v[0].y()), floor(v[1].y())), floor(v[2].y()));
    int ymax = MAX(MAX(floor(v[0].y()), floor(v[1].y())), floor(v[2].y()));

    int sample_num = 8;
    std::vector<float> offset;

	for (int i = 0; i < sample_num; ++i)
	{
		offset.push_back((0.5 + i) * 1.0 / static_cast<float>(sample_num));
	}

    int index;

    // MSAA
   /* for (int x = xmin; x <= xmax; x++) {
        for (int y = ymin; y <= ymax; y++) {
            int in_num = 0;
			for (int i = 0; i < sample_num; ++i) {
				for (int j = 0; j < sample_num; ++j) {
					if (insideTriangle(x + offset[i], y + offset[j], t.v)) {
						++in_num;
					}
				}
			}
            if (in_num > 0 &&insideTriangle(x + 0.5, y + 0.5, t.v)) {
                auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated =
                    alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                index = get_index(x, y);

                if (index < frame_buf.size() && depth_buf[index] > z_interpolated) {
                    depth_buf[index] = z_interpolated;
                    Eigen::Vector3f point;
                    point << static_cast<float>(x), static_cast<float>(y), z_interpolated;
                    set_pixel(point, t.getColor() * in_num / (sample_num * sample_num));
                }

            }
        }
    }*/

    float sampling_period = 1.0f / ssaa_sample;

    // 2x2SSAA
	for (int x = xmin; x <= xmax; x++) {
		for (int y = ymin; y <= ymax; y++) {
            int in_num = 0;
            Eigen::Vector3f color_sum;
            for (int i = 0; i < ssaa_sample; ++i) {
                for (int j = 0; j < ssaa_sample; ++j) {
                    // 中心点
                    float new_x = x + (i + 0.5) * sampling_period;
                    float new_y = y + (j + 0.5) * sampling_period;

                    if (insideTriangle(new_x, new_y, t.v)) {
                        auto [alpha, beta, gamma] = computeBarycentric2D(new_x, new_y, t.v);
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated =
                            alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;

                        // 左下角的点
                        int depth_buf_x, depth_buf_y;
                        depth_buf_x = x * ssaa_sample + i;
                        depth_buf_y = y * ssaa_sample + j;

                        if (z_interpolated < depth_buf[get_index(depth_buf_x, depth_buf_y)]) {
                            depth_buf[get_index(depth_buf_x, depth_buf_y)] = z_interpolated;
                            Vector3f temp_point = { depth_buf_x * 1.0f,depth_buf_y * 1.0f,0.0f };
                            Vector3f color = t.getColor();
                            set_temp_pixel(temp_point, color);

                        }
                    }
                }
            }
        }
	}

    for (int x = xmin; x <= xmax; x++)
    {
        for (int y = ymin; y <= ymax; y++)
        {
            Eigen::Vector3f color{ 0,0,0 };
            Eigen::Vector3f point{ x * 1.0f, y * 1.0f, 0 };

            for (int i = 0; i < ssaa_sample; ++i)
            {
                for (int j = 0; j < ssaa_sample; ++j)
                {
                    int depth_buf_x, depth_buf_y;
                    depth_buf_x = x * ssaa_sample + i;
                    depth_buf_y = y * ssaa_sample + j;
                    color += temp_frame_buf[get_index(depth_buf_x, depth_buf_y)];
                }
            }
            color /= (ssaa_sample * ssaa_sample);
            set_pixel(point, color);
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m) {
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v) {
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p) {
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff) {
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color) {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth) {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
    std::fill(temp_frame_buf.begin(), temp_frame_buf.end(), Eigen::Vector3f{ 0,0,0 });
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h) {
    frame_buf.resize(w * h);
    temp_frame_buf.resize(w * h * ssaa_sample * ssaa_sample);
    depth_buf.resize(w * h * ssaa_sample * ssaa_sample);
}

int rst::rasterizer::get_index(int x, int y) {
    // MSAA
    //return (height - 1 - y) * width + x;
    return (height * ssaa_sample - 1 - y) * width * ssaa_sample + x;
}

Eigen::Vector3f rst::rasterizer::get_pixed(Eigen::Vector3f& point) {
    auto ind = (height - 1 - point.y()) * width + point.x();
    return frame_buf[ind];
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color) {
    //old index: auto ind = point.y() + point.x() * width;
    // MSAA
    auto ind = (height - 1 - point.y()) * width + point.x();
    frame_buf[ind] = color;
}

void rst::rasterizer::set_temp_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    // 4x4SSAA
    auto ind = (height * ssaa_sample - 1 - point.y()) * width * ssaa_sample + point.x();
    temp_frame_buf[ind] = color;
}

// clang-format on