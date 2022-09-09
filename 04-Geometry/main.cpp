#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

#ifndef DIR_PATH
#error  path not define!
#else
#define S_PATH(str) DIR_PATH##str
#endif


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
    // 贝塞尔曲线代数公式
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
    // Implement de Casteljau's algorithm
    int n = control_points.size();

    auto calPoint = [](cv::Point2f p0, cv::Point2f p1, float t) {
		cv::Point2f p;
		p.x = p0.x + (p1.x - p0.x) * t;
		p.y = p0.y + (p1.y - p0.y) * t;
        return p;
    };

    if (n > 2)
    {
        std::vector<cv::Point2f> next_points;
        for (int i = 0; i < n - 1; ++i)
        {
            next_points.emplace_back(calPoint(control_points[i], control_points[i + 1], t));
        }
        return recursive_bezier(next_points, t);
    }
    
    return calPoint(control_points[0], control_points[1], t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    // de Casteljau算法
	for (double t = 0.0; t <= 1.0; t += 0.001)
	{
        cv::Point2f point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
	}
}

/*
对于提高部分，对实现方法不做硬性规定，只要你根据距离来考虑相邻元素的颜色即可，请大家大胆发挥。
如果出现“断裂”的现象可以适当降低t的间隔，“断裂”现象不会影响你的得分。
*/

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
            naive_bezier(control_points, window);
			bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            //cv::imwrite(S_PATH("./image/my_bezier_curve.png"), window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
