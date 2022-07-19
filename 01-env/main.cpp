#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>

#include<iostream>
#include<corecrt_math_defines.h>

using namespace std;

int main(){

    // Basic Example of cpp
    std::cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    std::cout << a << std::endl;
    std::cout << a/b << std::endl;
    std::cout << std::sqrt(b) << std::endl;
    std::cout << std::acos(-1) << std::endl;
    std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;

    // Example of vector
    std::cout << "Example of vector \n";
    // vector definition
    Eigen::Vector3f v(1.0f,2.0f,3.0f);
    Eigen::Vector3f w(1.0f,0.0f,0.0f);
    // vector output
    std::cout << "Example of output \n";
    std::cout << v << std::endl;
    // vector add
    std::cout << "Example of add \n";
    std::cout << v + w << std::endl;
    // vector scalar multiply
    std::cout << "Example of scalar multiply \n";
    std::cout << v * 3.0f << std::endl;
    std::cout << 2.0f * v << std::endl;

    // Example of matrix
    std::cout << "Example of matrix \n";
    // matrix definition
    Eigen::Matrix3f i,j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of output \n";
    std::cout << i << std::endl;
    std::cout << j << std::endl;
    // matrix add i + j
    // matrix scalar multiply i * 2.0
    // matrix multiply i * j
    // matrix multiply vector i * v

    // 点积
    Eigen::Vector3f v2(1.0f, 2.0f, 3.0f);
    Eigen::Vector3f w2(1.0f, 0.0f, 0.0f);
    std::cout << "Dot product: " << v2.dot(w2) << std::endl;

    // 叉乘
	std::cout << "output" << v2.cross(w2) << std::endl;

    // 矩阵乘法
    std::cout << i * j << std::endl;
    std::cout << i * v << std::endl;


	// 作业：给定一个点 P=(2,1), 将该点绕原点先逆时针旋转 45◦，
    // 再平移 (1,2), 计算出
	// 变换后点的坐标（要求用齐次坐标进行计算）。
    //Eigen::Vector3f p0(2.0f, 1.0f, 0.0f);
	//Eigen::Matrix3f pM = p0.matrix();
    //Eigen::Isometry3f rotateMat = Eigen::Isometry3f::Identity();
    //rotateMat.rotate()
    //Eigen::Vector3f res = p0.eulerAngles(0, 0, 45.0f / 180.0f * acos(-1));
    Eigen::Vector3f p0(2.0f, 1.0f, 0.0f);

	Eigen::AngleAxisf rotationVector2(M_PI / 4, Eigen::Vector3f(0, 0, 1));
	Eigen::Matrix3f rotationMatrix2 = Eigen::Matrix3f::Identity();
    rotationMatrix2 = rotationVector2.toRotationMatrix();
    cout << rotationMatrix2* p0 << endl;


    return 0;
}