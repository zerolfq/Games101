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
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection intersection = Scene::intersect(ray);
    Vector3f hitPoint = intersection.coords;
    Vector3f normal = intersection.normal;
    Material* m = intersection.m;
    Vector3f L_dir, L_indir;
    if (intersection.happened){
        if (intersection.m->hasEmission()) //一、交点是光源：
            return intersection.m->getEmission();
        
        float light_pdf = 0;
        sampleLight(intersection,light_pdf);
        Vector3f x = intersection.coords;
        Vector3f light_normal = intersection.normal;
        Vector3f ws = x - hitPoint;
        Vector3f emit = intersection.emit;
        ws = normalize(ws);
        Intersection light_inter = Scene::intersect(Ray(hitPoint,ws));
        // dotProduct(light_inter.coords - hitPoint,light_inter.coords - hitPoint) >= dotProduct(x - hitPoint,x - hitPoint);
        if (dotProduct(light_inter.coords - hitPoint,light_inter.coords - hitPoint) >= dotProduct(x - hitPoint,x - hitPoint)) {
            L_dir = emit * m->eval(ray.direction,ws,normal) * dotProduct(ws,normal) * dotProduct(-ws,light_normal) / dotProduct(hitPoint - x,hitPoint - x) / light_pdf;
        }

        if (get_random_float() <= RussianRoulette){
            Vector3f wi = m->sample(ray.direction,normal);
            wi = normalize(wi);
            Vector3f wo = ray.direction;
            Intersection object_inter = Scene::intersect(Ray(hitPoint,wi));
            if (object_inter.happened && !object_inter.obj->hasEmit()) {
                L_indir = Scene::castRay(Ray(hitPoint,wi),0) * m->eval(wo,wi,normal) * dotProduct(wi,normal) / m->pdf(wo,wi,normal) / RussianRoulette;
            }

        } 
    }
    return L_dir + L_indir;
}