#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
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

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    vector<cv::Point2f> q;
    if (control_points.size() == 1) return control_points[0];
    for (int i = 0;i < control_points.size() - 1;i++)
        q.push_back(control_points[i] * (1 - t) + control_points[i + 1] * t);
    return recursive_bezier(q,t);

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    float step = 0.001;
    float t = 0;
    int dx[] = {0,0,1,-1},dy[] = {1,-1,0,0};
    while (t <= 1.0f){
        auto point = recursive_bezier(control_points,t);
        t += step;
        int x = point.x,y = point.y;
        float dist = 1000000.0f;
        int now_x = x ,now_y = y;
        for (int j = 0;j < 4;j++){
            int nx = x + dx[j],ny = y + dy[j];
            if (nx < 0 && nx >= 800 && ny < 0 && ny >= 800) continue;
            float tmp = pow(point.x - nx,2) + pow(point.y - ny,2);
            if (tmp < dist){
                dist = tmp;
                now_x = nx ,now_y = ny;
            } 
        }
         cout << now_x<< " " << now_y << endl;
        // window.at<cv::Vec3b>(now_x, now_y)[1] = 255;
        cout << point.x << " " << point.y << endl;
         window.at<cv::Vec3b>(now_y, now_x)[1] = 255;
    }
}

int main() 
{
    cv::Mat window = cv::Mat(800, 800, CV_8UC3, cv::Scalar(0));
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
            cv::imwrite("_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
