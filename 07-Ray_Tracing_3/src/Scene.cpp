//
// Created by Göksu Güvendiren on 2019-05-14.
//
#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

/**
 * 在场景的所有光源上按面积 uniform 的 sample 一个点，
 * 并计算该 sample 的概率密度。
 * 算法首先遍历场景中的所有光源（实际上场景中只有一个光源）并计算出光源面积的和，
 * 然后使用随机数的方法取到一个随机的光源，对这个光源进行采样。
 * 即，随机取到该光源上的一个点，返回Intersection信息，并计算pdf概率密度的值（面积的倒数）：
 */
void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
// Ref:
// https://blog.csdn.net/weixin_45951701/article/details/127197404
// https://zhuanlan.zhihu.com/p/542134949
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection it = this->intersect(ray); // 得到点 p
    if (!it.happened) // 光线与物体存在相交
        return Vector3f(0);

    /**
     * 判断交点处是否为光源，由于光源也是一个Object对象，
     * 因此需要判断其是否有emission属性。
     * 若我们的primary ray直射到了光源（即depth = 0），
     * 则表示由摄像机可直接看到光源，因此直接返回光源颜色
     */
    if (it.m->hasEmission() && depth == 0)
        return it.m->getEmission();

    Vector3f L_dir(0);
    Vector3f L_indir(0);

    // 对光源进行采样，得到光源上随机一点x的信息和概率密度函数值
    Intersection light_pos = Intersection();
    float pdf_light = 0.0f;
    sampleLight(light_pos, pdf_light);

    // 射线方向, p to light
    Vector3f collision_light = (light_pos.coords - it.coords).normalized();

    // 创建一条从交点p射向光源点light_pos的射线，判断交点是否为光源
    Ray light_to_object_ray(it.coords, collision_light);
    Intersection light_ray_inter = Scene::intersect(light_to_object_ray);

    Vector3f L_i = light_pos.emit; // 从光源发出的入射光线
    Vector3f wo = ray.direction; // view to p
    Vector3f ws = collision_light; // p to light(x')
    Vector3f N = it.normal; // N表示交点的法线方向
    Vector3f NN = light_pos.normal; // NN表示光源上x点的法线方向

    Vector3f f_r = it.m->eval(wo, ws, N); // BRDF 散射函数
    float cos_theta = dotProduct(ws, N); // (p to light(x')) 与 p的法线夹角
    float cos_theta_x = dotProduct(-ws, NN);  // (light(x') to p) 与 light(x')法线夹角
    float dis_pow2 = dotProduct(light_pos.coords - it.coords, light_pos.coords - it.coords);

    // 判断光源中间是否被遮挡, 若被遮挡, 则无法照亮交点p, 即判断距离差
    if (light_ray_inter.distance - collision_light.norm() > -0.005)
    {
        L_dir = L_i * f_r * cos_theta * cos_theta_x / dis_pow2 / pdf_light;
    }

    // Contribution from other reflectors. 非光源的贡献
    // 引入俄罗斯轮盘赌
    float P_RR = RussianRoulette;
    if (get_random_float() < P_RR)
    {
        // 1.对半球进行采样得到一个随机方向wi（漫反射均匀分布）
        Vector3f wi = it.m->sample(ray.direction, it.normal).normalized();
        Ray p_to_wi_ray(it.coords, wi);
        Intersection islight = Scene::intersect(p_to_wi_ray);
        if (islight.happened && !islight.m->hasEmission()) // 若交点不是光源，才继续进行计算
        {
            float pdf = it.m->pdf(ray.direction, wi, it.normal);
            L_indir = castRay(p_to_wi_ray, depth + 1) * f_r * dotProduct(wi, it.normal) / pdf / P_RR;
        }
    }

    return L_dir + L_indir;
}