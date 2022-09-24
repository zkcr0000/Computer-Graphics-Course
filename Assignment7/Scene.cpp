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

    /*
    shade(p, wo)
    # Contribution from the light source.
        Uniformly sample the light with "sampleLight(inter, pdf_light)"
        x: light source sampling position)
        p: path ray hitting object position
        w_i_light: (p - x) incoming light ray 
        w_o: ray outgoing light direction to the camera
        N_p: object normal vector
        N_light: light normal vector
        L_i_light: light incoming radiance
        brdf : BRDF given incoming and output direction

        Shoot a ray from p to x
        If the ray is not blocked in the middle
            L_dir = emit * eval(wo, ws, N) * dot(ws, N) * dot(ws, NN) / |x-p|^2 / pdf_light
    # Contribution from other reflectors.
    L_indir = 0.0

    Test Russian Roulette with probability RussianRoulette wi = sample(wo, N)
    Trace a ray r(p, wi)
    If ray r hit a non-emitting object at q
        L_indir = shade(q, wi) * eval(wo, wi, N) * dot(wi, N) / pdf(wo, wi, N) / RussianRoulette
    
    Return L_dir + L_indir
    */

    // Since we have Russian Roulette, we don't need to check this anymore.
    // if (depth > this->maxDepth) {
    //     return Vector3f(0.0f,0.0f,0.0f);
    // }

    /*
    Intersection p_intersection = intersect(ray);
    if (!p_intersection.happened) {
        return Vector3f();
    }

    if (p_intersection.m->hasEmission()) {
        return p_intersection.m->getEmission();
    }

    Vector3f l_dir; // contribution from light source
    Vector3f l_indir; // contribution from non-emitted object

    // Uniform sample light
    Intersection x_intersection;
    float pdf_light = 0.0f;
    sampleLight(x_intersection, pdf_light);

    Vector3f p = p_intersection.coords;
    Vector3f x = x_intersection.coords;
    Vector3f w_i_light = (x-p).normalized();
    float p_x_distance = (x-p).norm();
    Vector3f N_p = p_intersection.normal.normalized();
    Vector3f N_light = x_intersection.normal.normalized();

    // radiance of spherical light
    Vector3f l_i_light = x_intersection.emit;

    // Shoot a ray from p to x to check if blocked
    Ray p_x_ray(p, w_i_light); 
    Intersection p_x_ray_intersection = intersect(p_x_ray);

    // 0.0001 is the error acceptance for intersection
    if(p_x_ray_intersection.distance - p_x_distance > -0.0001) {
        Vector3f brdf = p_intersection.m->eval(ray.direction, p_x_ray.direction, N_p);
        // brdf assuming diffuse model, for RGB component
        l_dir = l_i_light * brdf  
            * dotProduct(p_x_ray.direction, N_p)
            * dotProduct(-p_x_ray.direction, N_light)
            / std::pow(p_x_distance, 2)
            / pdf_light;
    }

    // Probability of RussianRoulette, whether we want to shoot the next ray
    float p_RR = 0.8;
    // There is probability RussianRoulette
    if(get_random_float() > p_RR) {
        return l_dir;
    }

    // random sample a direction wiht some pdf
    Vector3f wi_dir = p_intersection.m->sample(ray.direction, N_p).normalized();
    Ray wi_ray(p_intersection.coords, wi_dir);
    // If ray r hit a non-emitting object at q
    Intersection wi_inter = intersect(wi_ray);
    if (wi_inter.happened && (!wi_inter.m->hasEmission())) {
        Vector3f brdf = p_intersection.m->eval(ray.direction, wi_ray.direction, N_p);
        l_indir = castRay(wi_ray, depth + 1) * brdf
            * dotProduct(wi_ray.direction, N_p)
            / p_intersection.m->pdf(ray.direction, wi_ray.direction, N_p)
            / p_RR;
    }

    */


    Intersection p_inter = intersect(ray);
    if (!p_inter.happened) {
        return Vector3f();
    }
    if (p_inter.m->hasEmission()) {
        return p_inter.m->getEmission();
    }

    float EPLISON = 0.0001;
    Vector3f l_dir;
    Vector3f l_indir;
    
    // sampleLight(inter, pdf_light)
    Intersection x_inter;
    float pdf_light = 0.0f;
    sampleLight(x_inter, pdf_light);    
    
    // Get x, ws, NN, emit from inter
    Vector3f p = p_inter.coords;
    Vector3f x = x_inter.coords;
    Vector3f ws_dir = (x - p).normalized();
    float ws_distance = (x - p).norm();
    Vector3f N = p_inter.normal.normalized();
    Vector3f NN = x_inter.normal.normalized();
    Vector3f emit = x_inter.emit;

    // Shoot a ray from p to x
    Ray ws_ray(p, ws_dir); 
    Intersection ws_ray_inter = intersect(ws_ray);
    // If the ray is not blocked in the middle
    if(ws_ray_inter.distance - ws_distance > -EPLISON) {
        l_dir = emit * p_inter.m->eval(ray.direction, ws_ray.direction, N) 
            * dotProduct(ws_ray.direction, N)
            * dotProduct(-ws_ray.direction, NN)
            / std::pow(ws_distance, 2)
            / pdf_light;
    }
    
    // Test Russian Roulette with probability RussianRoulette
    if(get_random_float() > RussianRoulette) {
        return l_dir;
    }

    l_indir = 0.0;

    Vector3f wi_dir = p_inter.m->sample(ray.direction, N).normalized();
    Ray wi_ray(p_inter.coords, wi_dir);
    // If ray r hit a non-emitting object at q
    Intersection wi_inter = intersect(wi_ray);
    if (wi_inter.happened && (!wi_inter.m->hasEmission())) {
        l_indir = castRay(wi_ray, depth + 1) * p_inter.m->eval(ray.direction, wi_ray.direction, N)
            * dotProduct(wi_ray.direction, N)
            / p_inter.m->pdf(ray.direction, wi_ray.direction, N)
            / RussianRoulette;
    }
    return l_dir+l_indir;
}
