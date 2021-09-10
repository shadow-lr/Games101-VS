#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    if (control_points.size() <= 1)
    {
        return control_points[0];
    }

    std::vector<cv::Point2f> temp_points;

    for (int i = 0; i < control_points.size() - 1; ++i)
    {
        auto point = (1 - t) * control_points[i] + control_points[i + 1] * t;
        temp_points.emplace_back(point);
    }
    
    return recursive_bezier(temp_points, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    auto& p_0 = control_points[0];
    auto& p_1 = control_points[1];
    auto& p_2 = control_points[2];
    auto& p_3 = control_points[3];

    std::vector<cv::Point2f> handing_points{ p_0,p_1,p_2,p_3 };
    std::vector<cv::Point2f> temp_points{ p_0,p_1,p_2,p_3 };

    int sample_num = 2;

	for (double t = 0.0; t <= 1.0; t += 0.001)
	{
        handing_points = temp_points;
        auto final_point = recursive_bezier(handing_points, t);

        for (int i = 0; i < sample_num; ++i)
        {
            for (int j = 0; j < sample_num; ++j)
            {
                int y = final_point.y + i;
                int x = final_point.x + j;
                double distance = sqrt(pow(y - final_point.y, 2) + pow(x - final_point.x, 2)) /* / sqrt(2)*/;
                window.at<cv::Vec3b>(y, x)[1] = MIN(window.at<cv::Vec3b>(y, x)[1] + 255 * MAX(2 - exp(distance), 0.0), 255.0);
                //window.at<cv::Vec3b>(y, x)[1] = (1 - distance) * window.at<cv::Vec3b>(y, x)[1] + distance * 255.0;
            }
        }
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            //naive_bezier(control_points, window);
			bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
