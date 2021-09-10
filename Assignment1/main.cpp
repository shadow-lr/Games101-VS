#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <iostream>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos) {
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
            -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle) {
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    Eigen::Matrix4f rotation;

    rotation_angle *= MY_PI / 180.0f;

    rotation << cos(rotation_angle), -sin(rotation_angle), 0, 0,
            sin(rotation_angle), cos(rotation_angle), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    model = rotation * model;

    return model;
}

Eigen::Matrix4f get_model_matrix(Vector3f axis, float angle) {
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f rotation;
    Eigen::Matrix3f mul;


    float radian = angle * MY_PI / 180.0f;
    float length = sqrt(axis.x() * axis.x() + axis.y() * axis.y() + axis.z() * axis.z());

    axis.x() /= length;
    axis.y() /= length;
    axis.z() /= length;


    mul << 0, -axis[2], axis[1],
            axis[2], 0, -axis[0],
            -axis[1], axis[0], 0;

    rotation =
            Eigen::Matrix3f::Identity() * cos(radian) + (1 - cos(radian)) * axis * axis.transpose() + sin(radian) * mul;

    model << rotation(0, 0), rotation(0, 1), rotation(0, 2), 0,
            rotation(1, 0), rotation(1, 1), rotation(1, 2), 0,
            rotation(2, 0), rotation(2, 1), rotation(2, 2), 0,
            0, 0, 0, 1;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar) {
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f temp;

    float x00 = 1.0f / (aspect_ratio * tan(0.5f * eye_fov * MY_PI / 180.0f));
    float x11 = 1.0f / (tan(0.5f * eye_fov * MY_PI / 180.0f));
    float x22 = (-zFar - zNear) / (zNear - zFar);

    float x23 = (2.0f * zNear * zFar) / (zNear - zFar);
    float x33 = 1.0f;

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    temp << x00, 0, 0, 0,
            0, x11, 0, 0,
            0, 0, x22, x23,
            0, 0, x33, 0;

    projection = temp * projection;

    return projection;
}

int main(int argc, const char **argv) {
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2,  0, -2},
                                     {0,  2, -2},
                                     {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    Vector3f axis(1, 0, 0);

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(axis, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(axis, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(60, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        } else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
