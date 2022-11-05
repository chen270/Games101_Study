#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds()); // 整体的包围盒
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        // 个数为 1 时，创建叶子节点
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        // 数为 2 时，创建左右叶子节点
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        // 若容器内有两个以上的Object对象, 则需要使用算法将它们分成两部分

        /**
         * 1.对于每一个Object对象的包围盒，计算出其包围盒的中心，
         *   以这些包围盒的中心坐标建立一个新的包围盒
         */
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid()); // Centroid:矩心


        /**
         * 2.确认上面生成的包围盒在XYZ哪个维度上的跨度最大，然后在该维度上，
         *   以每个Object对象的包围盒的中心为比较对象，对Object对象进行排序.
         */
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0: // 最宽在x轴
            // sort排序，比如这里对比两个包围盒质心的x坐标，小的排左边，大的排右边
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1: // 最宽在y轴
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2: // 最宽在z轴
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        /**
         * 将排序后的Object对象平分到两个容器中，
           分别作为递归调用recursiveBuild()的参数，
           将返回值分别赋给左右子节点
         */
        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        // 包围盒为左右子节点的和
        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

static float getCubeSurfaceArea(const Vector3f& s, const Vector3f& e)
{
    float a = std::fabs(s.x - e.x);
    float b = std::fabs(s.y - e.y);
    float c = std::fabs(s.z - e.z);
    return 2 * (a * b + a * c + b * c);
}

BVHBuildNode* BVHAccel::recursiveBuild_BY_SAH(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds()); // 整体的包围盒

    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        // 个数为 1 时，创建叶子节点
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        // 数为 2 时，创建左右叶子节点
        node->left = recursiveBuild(std::vector{ objects[0] });
        node->right = recursiveBuild(std::vector{ objects[1] });

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        // 若容器内有两个以上的Object对象, 则需要使用算法将它们分成两部分

        /**
         * 1.对于每一个Object对象的包围盒，计算出其包围盒的中心，
         *   以这些包围盒的中心坐标建立一个新的包围盒
         */
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
            Union(centroidBounds, objects[i]->getBounds().Centroid()); // Centroid:矩心

	    /**
	     * 2.确认上面生成的包围盒在XYZ哪个维度上的跨度最大，然后在该维度上，
	     *   以每个Object对象的包围盒的中心为比较对象，对Object对象进行排序.
	     */
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0: // 最宽在x轴
            // sort排序，比如这里对比两个包围盒质心的x坐标，小的排左边，大的排右边
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                    f2->getBounds().Centroid().x;
                });
            break;
        case 1: // 最宽在y轴
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                    f2->getBounds().Centroid().y;
                });
            break;
        case 2: // 最宽在z轴
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                    f2->getBounds().Centroid().z;
                });
            break;
        }

        const bool useSAB = true;
        const int B = 12; // 桶数
		int mincostIndex = 0;
        if (useSAB) // SAH
        {
			float S_C = centroidBounds.SurfaceArea(); // 总面积
			//int B = 10; // 桶数
			float minCost = std::numeric_limits<float>::infinity(); //最小花费

            for (int i = 0; i < B; ++i)
            {
				auto beginning = objects.begin();
                auto middling = objects.begin() + (objects.size() * i / B);
				auto ending = objects.end();

				auto leftshapes = std::vector<Object*>(beginning, middling);
				auto rightshapes = std::vector<Object*>(middling, ending);

                //求左右包围盒:
                Bounds3 leftBound, rightBound;
				for (int j = 0; j < leftshapes.size(); ++j)
                    leftBound = Union(leftBound, objects[j]->getBounds().Centroid());
				for (int j = 0; j < rightshapes.size(); ++j)
                    rightBound = Union(rightBound, objects[j]->getBounds().Centroid());
                
                float S_A = leftBound.SurfaceArea();
                float S_B = rightBound.SurfaceArea();

                float costTmp = S_A * leftshapes.size() / S_C + S_B * rightshapes.size() / S_C + 0.125f;
                if (minCost > costTmp)
                {
                    minCost = costTmp;
                    mincostIndex = i;
                }
            }
        }

        /**
         * 将排序后的Object对象平分到两个容器中，
           分别作为递归调用recursiveBuild()的参数，
           将返回值分别赋给左右子节点
         */
        auto beginning = objects.begin();
		auto middling = useSAB ? (objects.begin() + (objects.size() * mincostIndex / B)) :
			                     (objects.begin() + (objects.size() / 2));
		//auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        // 包围盒为左右子节点的和
        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection it;

    if (nullptr == node)
        return it;

    std::array<int, 3> dirIsNeg = { int(ray.direction.x > 0), int(ray.direction.y > 0), int(ray.direction.z > 0) };
    // 与包围盒没有相交
    if (!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
        return it;

    if (nullptr == node->left && nullptr == node->right)
        return node->object->getIntersection(ray); // leaf node

    Intersection it1 = getIntersection(node->left, ray);
    Intersection it2 = getIntersection(node->right, ray);

    return it1.distance > it2.distance ? it2 : it1;
}