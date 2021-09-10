### 完成的点

- 提交的格式正确，包含所有必须的文件。代码可以编译和运行
- De Casteljau 算法： 对于给定的控制点，你的代码能够产生正确的 Bézier 曲线
- 实现对 Bézier 曲线的反走样。(对于一个曲线上的点，不只把它对应于一个像 素，你需要根据到像素中心的距离来考虑与它相邻的像素的颜色。)

在recursive_bezier函数中进行递归调用

```cpp
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
```

在bezier函数中完成了曲线t参数的遍历和调用绘制绘制点的函数

```cpp
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
```



