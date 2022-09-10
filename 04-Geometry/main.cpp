#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

#ifndef DIR_PATH
#error  path not define!
#else
#define S_PATH(str) DIR_PATH##str
#endif

std::vector<cv::Point2f> control_points;

#define BEZIER_NUM 4

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < BEZIER_NUM)
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


// 反走样参考: https://zhuanlan.zhihu.com/p/464122963
static const float antialiasingArr[2][2] =
{
    {}
};

void bezier_antialiasing(const std::vector<cv::Point2f>& control_points, cv::Mat& window)
{
	// de Casteljau算法
    // 反走样求得到的点像素周围4个点
	for (double t = 0.0; t <= 1.0; t += 0.001)
	{
		cv::Point2f p = recursive_bezier(control_points, t);

		// 点 p 最相邻的四个点
		cv::Point2i p0(p.x - std::floor(p.x) < 0.5 ? std::floor(p.x) : std::ceil(p.x),
			p.y - std::floor(p.y) < 0.5 ? std::floor(p.y) : std::ceil(p.y));
		std::vector<cv::Point2i> ps = { p0, cv::Point2i(p0.x - 1, p0.y),
			cv::Point2i(p0.x, p0.y - 1), cv::Point2i(p0.x - 1, p0.y - 1),
		};

		// distance
		float sum_d = 0.f;
		float max_d = sqrt(2);
		std::vector<float> dis;
		for (const auto pi : ps)
		{
			float d = max_d - std::sqrt(std::pow((pi.x - p.x), 2) + std::pow((pi.y - p.y), 2));
			dis.emplace_back(d);
			sum_d += d;
		}

		// color
		// dis 0 -> 255
		// dis sqrt(2) -> 0
		// assign colors
		for (int i = 0; i < 4; i++) {
			//float k = dis[i] / sum_d;
			float k = dis[i] / max_d;
			window.at<cv::Vec3b>(ps[i].y, ps[i].x)[1] = std::min(255.f, window.at<cv::Vec3b>(ps[i].y, ps[i].x)[1] + 255.f * k);
		};

		//window.at<cv::Vec3b>(p.y, p.x)[1] = 255;
	}
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

	// Test
	control_points.emplace_back(cv::Point2f(130.0f,390.0f));
	control_points.emplace_back(cv::Point2f(241.0f,259.0f));
	control_points.emplace_back(cv::Point2f(490.0f,271.0f));
	control_points.emplace_back(cv::Point2f(548.0f,447.0f));


	while (key != 27)
	{
		for (auto& point : control_points)
		{
			cv::circle(window, point, 3, { 255, 255, 255 }, 3);
		}

		if (control_points.size() == BEZIER_NUM)
		{
			naive_bezier(control_points, window);
			//bezier(control_points, window);
			//bezier_antialiasing(control_points, window);

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
