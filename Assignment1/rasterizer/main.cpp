#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;
using namespace std;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float radian = rotation_angle / 180;
    // model << cos(radian * MY_PI), - sin(radian) , 0.0 ,0.0 , sin(radian) , cos(radian) , 0.0 , 0.0 , 0.0, 0.0 ,1.0 , 0.0, 0.0 , 0.0 , 0.0 ,1.0 ;
    model(0,0) = cos(radian * MY_PI);
    model(0,1) = - sin(radian * MY_PI);
    model(1,0) = sin(radian * MY_PI);
    model(1,1) = cos(radian * MY_PI);
    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    return model;
}

Eigen::Matrix4f get_rotation(Vector3f aixs,float angle){
    aixs.normalize();
    Vector3f t = Vector3f(1,1,1) + aixs;
    t.normalize();
    Vector3f w = t.cross(aixs);
    // cout << w << endl;
    w.normalize();
    Vector3f v = aixs.cross(w);
    // cout << v << endl;
    v.normalize();
    float radian = angle / 180 * MY_PI;

    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    // model << cos(radian * MY_PI), - sin(radian) , 0.0 ,0.0 , sin(radian) , cos(radian) , 0.0 , 0.0 , 0.0, 0.0 ,1.0 , 0.0, 0.0 , 0.0 , 0.0 ,1.0 ;
    model(1,1) = cos(radian);
    model(1,2) = -sin(radian);
    model(2,1) = sin(radian);
    model(2,2) = cos(radian);

    Eigen::Matrix4f coor;
    coor << aixs(0), w(0), v(0), 0, aixs(1), w(1), v(1), 0, aixs(2), w(2), v(2), 0, 0, 0, 0, 1;  
    return coor * model * coor.inverse();
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    zNear = -zNear;
    zFar = -zFar;
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    projection = Eigen::Matrix4f::Zero();
    eye_fov  = eye_fov / 180 * MY_PI;
    // projection(0,0) = 1.0 / (tan(eye_fov * 0.5)* aspect_ratio);
    // projection(1,1) = 1.0 / tan(eye_fov * 0.5);
    // projection(2,2) = zFar / (zFar - zNear);
    // projection(2,3) = 1.0f;
    // projection(3,2) = (zNear * zFar) / (zNear - zFar);
    projection(0,0) = zNear;
    projection(1,1) = zNear;
    projection(2,2) = zNear + zFar;
    projection(2,3) = -zNear * zFar;
    projection(3,2) = 1.0;
    projection(3,3) = 0.0;

    Eigen::Matrix4f ort = Eigen::Matrix4f::Identity();;

    float t = tan(eye_fov / 2) * abs(zNear);
    float r = t * aspect_ratio;
    ort(0,0) = 1.0 / r;
    ort(1,1) = 1.0 / t;
    ort(2,2) = 2.0 / (zFar - zNear);
    ort(2,3) = (zNear + zFar) / 2;
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    projection = ort * projection;
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(Vector3f(0,0,1),angle));
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

        // r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(Vector3f(0,0,1),angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
