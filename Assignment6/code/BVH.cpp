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
    if (splitMethod == SplitMethod::NAIVE){        
        root = recursiveBuild(primitives);
        std::cout << "Build In BVH" << std::endl;
    }
    else {
        root = BuildAsSAH(primitives);
        std::cout << "Build In SAH" << std::endl;
    }
    std::cout << "move  " <<  primitives.size() << std::endl; 

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    
    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::BuildAsSAH(std::vector<Object*> objects){
    BVHBuildNode* node = new BVHBuildNode();
    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = BuildAsSAH(std::vector{objects[0]});
        node->right = BuildAsSAH(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }
        int B = 10;
        int mincostIndex = -1;
        double mincost = std::numeric_limits<double>::max();
        float area = centroidBounds.SurfaceArea();

        for (int i = 1;i < B;i++){
            auto beginning = objects.begin();
            auto middling = objects.begin() + (objects.size() * i / B);
            auto ending = objects.end();

            auto leftshapes = std::vector<Object*>(beginning, middling);
            auto rightshapes = std::vector<Object*>(middling, ending);

            Bounds3 leftBounds,rightBounds;
            for (int j = 0;j < leftshapes.size();j++) leftBounds = Union(leftBounds,leftshapes[j]->getBounds());
            for (int j  =  0;j < rightshapes.size();j++) rightBounds = Union(rightBounds,rightshapes[j]->getBounds());
            double leftarea = leftBounds.SurfaceArea() , rightarea = rightBounds.SurfaceArea();

            double temp = (leftarea / area) * leftshapes.size() + (rightarea / area) * rightshapes.size();

            if (temp < mincost) mincost = temp,mincostIndex = i;

        }
        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() * mincostIndex / B);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = BuildAsSAH(leftshapes);
        node->right = BuildAsSAH(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}


BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();
    // std::cout << objects.size() << std::endl;
    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

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
    Intersection isect;
    if (!node) return isect;
    std::array<int, 3> dirIsNeg = {(int)(ray.direction.x > 0),(int)(ray.direction.y > 0),(int)(ray.direction.z > 0)};
    if (node->bounds.IntersectP(ray,ray.direction_inv,dirIsNeg)){
        if (node->left == nullptr && node->right == nullptr)  {
            return node->object->getIntersection(ray);
        }
        Intersection left_intersection = BVHAccel::getIntersection(node->left,ray);
        Intersection right_intersection = BVHAccel::getIntersection(node->right,ray);
        return left_intersection.distance > right_intersection.distance?right_intersection:left_intersection;
    }
    return isect;
}