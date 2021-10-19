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
    Vector3f L_dir(0,0,0);
    Vector3f L_indir(0,0,0);

    // find whether ray hit anything
    Intersection inter = intersect(ray);

    // if hit nothing->return nothing
    if(!inter.happened) return Vector3f(0,0,0);

    if(inter.m->hasEmission())
    {
        // if(depth==0)    return inter.m->getEmission(); // if this ray hit light source directly, return directly.
        // else return Vector3f(0,0,0); // if thie ray hit light source(but not directly), we do not consider light source(we will consider it later)
        return inter.m->getEmission();
    }

    // sample light source
    Intersection lightInter;
    float pdf_light=0.0f;
    sampleLight(lightInter, pdf_light);

    Vector3f normal=inter.normal; // this is the object(hit) normal
    Vector3f object2light=lightInter.coords-inter.coords; // object->light source
    float objectLight_distance=object2light.norm();
    object2light=object2light.normalized();


    Ray light(inter.coords, object2light); // ray--start from object to light

    Intersection object2lightInter=intersect(light); // shoot light ray from object

    // if light ray hit light source directly
    if(object2lightInter.happened && (object2lightInter.coords-lightInter.coords).norm()<0.1)
    {
        // L_dir = emit * eval (wo , ws , N) * dot (ws , N) * dot (ws ,NN) / |x-p |^2 / pdf_light
        L_dir = lightInter.emit*inter.m->eval(ray.direction, object2light, normal)*dotProduct(object2light, normal)*dotProduct(-object2light, lightInter.normal)/(objectLight_distance*objectLight_distance)/pdf_light;
    }


    // hit other object
    // RR--get_random_float will directly return a float in 0-1
    if(get_random_float() < RussianRoulette)
    {
        // construct out ray
        // from object, sample object-0>outside 
        Vector3f outDirection = inter.m->sample(ray.direction, normal).normalized();
        Ray outRay(inter.coords, outDirection);
        Intersection outRayInter = intersect(outRay);

        // if out ray hit something but not light source--indirectly
        if(outRayInter.happened && !outRayInter.m->hasEmission())
        {
            // L_indir = shade (q, wi) * eval (wo , wi , N) * dot (wi , N)/ pdf (wo , wi , N) / RussianRoulette
            L_indir = castRay(outRay, depth+1)*inter.m->eval(ray.direction, outDirection, normal)*dotProduct(outDirection, normal)/inter.m->pdf(ray.direction, outDirection, normal)/RussianRoulette;
            // note: when we recursively call this funtion, depth+=1
        }
    }
    return L_dir+L_indir;
}