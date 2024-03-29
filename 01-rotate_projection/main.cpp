#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

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

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    model(0, 0) = cos(rotation_angle);
    model(0, 1) = -sin(rotation_angle);
    model(1, 0) = sin(rotation_angle);
    model(1, 1) = cos(rotation_angle);

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
#if 0
    zNear = -zNear;
    zFar = -zFar;

    projection(0, 0) = zNear / (abs(zNear) * aspect_ratio * tan(eye_fov / 2.0f));
    projection(1, 1) = zNear / (abs(zNear) * tan(eye_fov / 2.0f));
    projection(2, 2) = (zNear + zFar) / (zNear - zFar);
    projection(3, 3) = 0;
	projection(3, 2) = 1;
    projection(2, 3) = -2.0f * zNear * zFar / (zNear - zFar);

#else
    
	zNear = -zNear;
	zFar = -zFar;

    // zNear and zFar > 0, r= -l, t = -b
    float t = abs(zNear) * tan(eye_fov * MY_PI / 360.0f);
    float r = t * aspect_ratio;
	Eigen::Matrix4f scaleMat;
	Eigen::Matrix4f moveMat;
	Eigen::Matrix4f orthoMat;

    //缩放矩阵
    scaleMat <<
        1.0f / r, 0, 0, 0,
        0, 1.0f / t, 0, 0,
        0, 0, 2.0f / (zNear - zFar), 0,
		0, 0, 0, 1;

    //平移矩阵
    moveMat <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, -(zNear + zFar)/2.0f,
		0, 0, 0, 1;

    //透视矩阵
    orthoMat <<
		zNear, 0, 0, 0,
		0, zNear, 0, 0,
		0, 0, zNear + zFar, -zNear * zFar,
		0, 0, 1, 0;

    projection = scaleMat * moveMat * orthoMat * projection;
#endif

    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f rotate;
    float alpha = angle * MY_PI / 180;

    // 罗德里格旋转公式
    Eigen::Matrix3f N_mat;
    N_mat <<
        0, -axis.z(), axis.y(),
        axis.z(), 0, -axis.x(),
        -axis.y(), axis.x(), 0;

    Eigen::Matrix3f res = cos(alpha)* Eigen::Matrix3f::Identity() + (1 - cos(alpha)) * axis * axis.transpose() + sin(alpha) * N_mat;

    rotate <<
        res(0, 0), res(0, 1), res(0, 2), 0,
        res(1, 0), res(1, 1), res(1, 2), 0,
        res(2, 0), res(2, 1), res(2, 2), 0,
        0, 0, 0, 1;

    return rotate;
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

        r.set_model(get_model_matrix(angle));
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

        r.set_model(get_model_matrix(angle));
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
