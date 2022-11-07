//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_INTERSECTION_H
#define RAYTRACING_INTERSECTION_H
#include "Vector.hpp"
#include "Material.hpp"
class Object;
class Sphere;

struct Intersection
{
    Intersection(){
        happened=false;
        coords=Vector3f();
        normal=Vector3f();
        distance= std::numeric_limits<double>::max();
        obj =nullptr;
        m=nullptr;
    }
    bool happened; // 是否产生（符合要求的）交点
    Vector3f coords; // 交点坐标
    Vector3f tcoords; // 插值的纹理坐标
    Vector3f normal; // 交点所在平面法线
    Vector3f emit; // 材质的自发光颜色
    double distance; // 射线的传播距离，即t值
    Object* obj; // 交点所在物体的物体类型
    Material* m; // 交点所在物体的材料类型
};
#endif //RAYTRACING_INTERSECTION_H
