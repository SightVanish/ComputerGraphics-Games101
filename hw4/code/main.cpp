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
    // TODO: Implement de Casteljau's algorithm
    if(control_points.size()==1)    return control_points[0];
    std::vector<cv::Point2f> points;
    for(int i=0; i<control_points.size()-1; i++)
        points.push_back(t*control_points[i]+(1-t)*control_points[i+1]);
    return recursive_bezier(points, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for(double t=0.0; t<=1.0; t+=0.0001)
    {
        auto point=recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255; // show the point in green
        // anti-alising
        // the effect of anti-alising depends on the sample points you select
        std::vector<cv::Point2f> sample_points;
        float x=point.x-std::floor(point.x);
        float y=point.y-std::floor(point.y);
        float offset_x=x<0.5f?-1.0f:1.0f;
        float offset_y=y<0.5f?-1.0f:1.0f;
        sample_points.push_back(cv::Point2f(std::floor(point.x)+0.5f, std::floor(point.y)+0.5f));
        sample_points.push_back(cv::Point2f(std::floor(point.x)+offset_x, std::floor(point.y)+0.5f));
        sample_points.push_back(cv::Point2f(std::floor(point.x)+0.5f, std::floor(point.y)+offset_y));
        sample_points.push_back(cv::Point2f(std::floor(point.x)+offset_x, std::floor(point.y)+offset_y));

        cv::Point2f d = sample_points[0] - point;
        float base = sqrt(d.x*d.x + d.y*d.y);
        for(cv::Point2f p:sample_points)
        {
            cv::Point2f t=p-point;
            float distance=sqrt(t.x*t.x+t.y*t.y);
            cv::Vec3d color=window.at<cv::Vec3b>(p.y, p.x);
            color[1]=std::max((float)color[1], 255*(distance/base));
            color[1]=255;
            window.at<cv::Vec3b>(p.y, p.x)[1]=color[1];
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
            // naive_bezier(control_points, window);
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
