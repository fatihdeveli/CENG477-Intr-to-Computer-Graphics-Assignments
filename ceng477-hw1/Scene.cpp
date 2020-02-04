#include "Scene.h"
#include "Camera.h"
#include "Light.h"
#include "Material.h"
#include "Shape.h"
#include "tinyxml2.h"
#include "Image.h"
#include "VectorOperations.h"
#include <cmath>

using namespace tinyxml2;

/* 
 * Must render the scene from each camera's viewpoint and create an image.
 * You can use the methods of the Image class to save the image as a PPM file. 
 */
void Scene::renderScene(void)
{
	/***********************************************
     * Implemented                                 *
     ***********************************************/
	using namespace VectorOperations;
    // Create an image for each camera
    for (auto camera : cameras) {

        int nx = camera->imgPlane.nx;
        int ny = camera->imgPlane.ny;

        Image image(nx, ny);

        for (int i = 0; i < nx; i++) {
            for (int j = 0; j < ny; j++) {
                // Compute the viewing ray r (from e to s)
                Ray r = camera->getPrimaryRay(i, j);

                Vector3f totalLight = shading(r, 0);
                Color color;
                color.red = totalLight.r > 255 ? 255 : (unsigned char) totalLight.r;
                color.grn = totalLight.g > 255 ? 255 : (unsigned char) totalLight.g;
                color.blu = totalLight.b > 255 ? 255 : (unsigned char) totalLight.b;
                image.setPixelValue(i, j, color);
            }
        }
        image.saveImage(camera->imageName);
    }
}

Vector3f Scene::shading(const Ray &r, int recursionDepth) const {
    using namespace VectorOperations;

    if (recursionDepth >= maxRecursionDepth) {
        return {0,0,0};
    }

    float t_min = MAXFLOAT;
    Shape* obj = nullptr;
    ReturnVal intersectionPoint;

    for (auto object : this->objects) {
        ReturnVal returnVal = object->intersect(r);
        if (returnVal.intersect) { // Check if r intersects the object
            if (returnVal.t < t_min) {
                t_min = returnVal.t;
                obj = object;
                intersectionPoint = returnVal;
            }
        }
    }

    if (obj) { // Viewing ray intersected with an object
        // Ambient shading
        Vector3f ambientReflectanceCoefficient = materials[obj->matIndex - 1]->ambientRef;
        Vector3f totalLight = {ambientLight.r * ambientReflectanceCoefficient.r,
                               ambientLight.g * ambientReflectanceCoefficient.g,
                               ambientLight.b * ambientReflectanceCoefficient.b};

        for (auto light : lights) {
            // Compute the shadow ray s from intersection point to light
            // Offset the shadow ray origin by epsilon
            Vector3f shadowRayOrigin = intersectionPoint.intersectPoint;
            Vector3f shadowRayDirection = vectorSubtract(light->position, shadowRayOrigin);
            Vector3f offset = vectorWithConstantMult(shadowRayDirection, shadowRayEps);
            shadowRayOrigin = vectorSum(shadowRayOrigin, offset);

            Ray s(shadowRayOrigin, shadowRayDirection);

            bool inShadow = false;
            for (auto object : objects) {
                // If s intersects the object before the light source
                ReturnVal shadowRayIntersection = object->intersect(s);
                if (shadowRayIntersection.intersect && shadowRayIntersection.t < s.gett(light->position)) {
                    // Continue the light loop
                    inShadow = true;
                    break;
                }
            }

            if (!inShadow) {
                // Add diffuse and specular components for this light source
                // Pixel color += L_d + Ls
                // Calculate diffuse shading with k_d*cos(theta)*contribution
                Vector3f lightContribution = light->computeLightContribution(intersectionPoint.intersectPoint);
                Vector3f diffuseReflectanceCoefficient = materials[obj->matIndex - 1]->diffuseRef;
                Vector3f normal = normalize(intersectionPoint.surf_normal);
                Vector3f w_i = normalize(vectorSubtract(light->position, s.origin));
                float cosTheta = dotProduct(normal, w_i);
                if (cosTheta < 0) cosTheta = 0;
                Vector3f outgoingRadiance = {diffuseReflectanceCoefficient.r * cosTheta * lightContribution.r,
                                             diffuseReflectanceCoefficient.g * cosTheta * lightContribution.g,
                                             diffuseReflectanceCoefficient.b * cosTheta * lightContribution.b};

                totalLight.r += outgoingRadiance.r;
                totalLight.g += outgoingRadiance.g;
                totalLight.b += outgoingRadiance.b;

                // Calculate specular shading with k_s * (cos(alpha))^p * contribution
                Vector3f specularReflectanceCoefficient = materials[obj->matIndex - 1]->specularRef;
                // w_i: shadowRayDirection;
                // w_o: -ray.direction
                Vector3f w_o = normalize(vectorWithConstantMult(r.direction, -1));
                // Half vector
                Vector3f h = normalize(vectorSum(normalize(shadowRayDirection), w_o)); // w_i + w_o normalized

                float cosAlpha = dotProduct(normalize(intersectionPoint.surf_normal), h);
                if (cosAlpha < 0) cosAlpha = 0;
                float multiplier = powf(cosAlpha, materials[obj->matIndex - 1]->phongExp);
                outgoingRadiance = {specularReflectanceCoefficient.r * multiplier * lightContribution.r,
                                    specularReflectanceCoefficient.g * multiplier * lightContribution.g,
                                    specularReflectanceCoefficient.b * multiplier * lightContribution.b};

                totalLight.r += outgoingRadiance.r;
                totalLight.g += outgoingRadiance.g;
                totalLight.b += outgoingRadiance.b;
            }
            // Add mirror component
            Vector3f mirrorRefCoefficient = materials[obj->matIndex - 1]->mirrorRef;
            if (mirrorRefCoefficient.r > 0 || mirrorRefCoefficient.g > 0 || mirrorRefCoefficient.b > 0) {
                // w_r = -w_o + 2.n.(n.w_o) // All unit vectors
                // Calculate k_m
                Vector3f normal = normalize(intersectionPoint.surf_normal);
                Vector3f w_o = normalize(vectorWithConstantMult(r.direction, -1));
                Vector3f w_r = vectorSum(normalize(r.direction),vectorWithConstantMult(normal,2*(dotProduct(normal, w_o))));
                w_r = normalize(w_r);

                // Offset the origin to avoid self-intersection of reflection ray
                offset = vectorWithConstantMult(w_r, shadowRayEps);
                Vector3f reflectionRayOrigin = vectorSum(intersectionPoint.intersectPoint, offset);
                Ray reflectionRay(reflectionRayOrigin, w_r);
                Vector3f mirrorReflection = shading(reflectionRay, ++recursionDepth);
                mirrorReflection = {mirrorReflection.x * mirrorRefCoefficient.x,
                                    mirrorReflection.y * mirrorRefCoefficient.y,
                                    mirrorReflection.z * mirrorRefCoefficient.z};
                totalLight = vectorSum(totalLight, mirrorReflection);
            }
        }
        return totalLight;
    }
    else {
        return backgroundColor;
    }
}

// Parses XML file. 
Scene::Scene(const char *xmlPath)
{
	const char *str;
	XMLDocument xmlDoc;
	XMLError eResult;
	XMLElement *pElement;

	maxRecursionDepth = 1;
	shadowRayEps = 0.001;

	eResult = xmlDoc.LoadFile(xmlPath);

	XMLNode *pRoot = xmlDoc.FirstChild();

	pElement = pRoot->FirstChildElement("MaxRecursionDepth");
	if(pElement != nullptr)
		pElement->QueryIntText(&maxRecursionDepth);

	pElement = pRoot->FirstChildElement("BackgroundColor");
	str = pElement->GetText();
	sscanf(str, "%f %f %f", &backgroundColor.r, &backgroundColor.g, &backgroundColor.b);

	pElement = pRoot->FirstChildElement("ShadowRayEpsilon");
	if(pElement != nullptr)
		pElement->QueryFloatText(&shadowRayEps);

	pElement = pRoot->FirstChildElement("IntersectionTestEpsilon");
	if(pElement != nullptr)
		eResult = pElement->QueryFloatText(&intTestEps);

	// Parse cameras
	pElement = pRoot->FirstChildElement("Cameras");
	XMLElement *pCamera = pElement->FirstChildElement("Camera");
	XMLElement *camElement;
	while(pCamera != nullptr)
	{
        int id;
        char imageName[64];
        Vector3f pos, gaze, up;
        ImagePlane imgPlane;

		eResult = pCamera->QueryIntAttribute("id", &id);
		camElement = pCamera->FirstChildElement("Position");
		str = camElement->GetText();
		sscanf(str, "%f %f %f", &pos.x, &pos.y, &pos.z);
		camElement = pCamera->FirstChildElement("Gaze");
		str = camElement->GetText();
		sscanf(str, "%f %f %f", &gaze.x, &gaze.y, &gaze.z);
		camElement = pCamera->FirstChildElement("Up");
		str = camElement->GetText();
		sscanf(str, "%f %f %f", &up.x, &up.y, &up.z);
		camElement = pCamera->FirstChildElement("NearPlane");
		str = camElement->GetText();
		sscanf(str, "%f %f %f %f", &imgPlane.left, &imgPlane.right, &imgPlane.bottom, &imgPlane.top);
		camElement = pCamera->FirstChildElement("NearDistance");
		eResult = camElement->QueryFloatText(&imgPlane.distance);
		camElement = pCamera->FirstChildElement("ImageResolution");	
		str = camElement->GetText();
		sscanf(str, "%d %d", &imgPlane.nx, &imgPlane.ny);
		camElement = pCamera->FirstChildElement("ImageName");
		str = camElement->GetText();
		strcpy(imageName, str);

		cameras.push_back(new Camera(id, imageName, pos, gaze, up, imgPlane));

		pCamera = pCamera->NextSiblingElement("Camera");
	}

	// Parse materals
	pElement = pRoot->FirstChildElement("Materials");
	XMLElement *pMaterial = pElement->FirstChildElement("Material");
	XMLElement *materialElement;
	while(pMaterial != nullptr)
	{
		materials.push_back(new Material());

		int curr = materials.size() - 1;
	
		eResult = pMaterial->QueryIntAttribute("id", &materials[curr]->id);
		materialElement = pMaterial->FirstChildElement("AmbientReflectance");
		str = materialElement->GetText();
		sscanf(str, "%f %f %f", &materials[curr]->ambientRef.r, &materials[curr]->ambientRef.g, &materials[curr]->ambientRef.b);
		materialElement = pMaterial->FirstChildElement("DiffuseReflectance");
		str = materialElement->GetText();
		sscanf(str, "%f %f %f", &materials[curr]->diffuseRef.r, &materials[curr]->diffuseRef.g, &materials[curr]->diffuseRef.b);
		materialElement = pMaterial->FirstChildElement("SpecularReflectance");
		str = materialElement->GetText();
		sscanf(str, "%f %f %f", &materials[curr]->specularRef.r, &materials[curr]->specularRef.g, &materials[curr]->specularRef.b);
		materialElement = pMaterial->FirstChildElement("MirrorReflectance");
		if(materialElement != nullptr)
		{
			str = materialElement->GetText();
			sscanf(str, "%f %f %f", &materials[curr]->mirrorRef.r, &materials[curr]->mirrorRef.g, &materials[curr]->mirrorRef.b);
		}
				else
		{
			materials[curr]->mirrorRef.r = 0.0;
			materials[curr]->mirrorRef.g = 0.0;
			materials[curr]->mirrorRef.b = 0.0;
		}
		materialElement = pMaterial->FirstChildElement("PhongExponent");
		if(materialElement != nullptr)
			materialElement->QueryIntText(&materials[curr]->phongExp);

		pMaterial = pMaterial->NextSiblingElement("Material");
	}

	// Parse vertex data
	pElement = pRoot->FirstChildElement("VertexData");
	int cursor = 0;
	Vector3f tmpPoint;
	str = pElement->GetText();
	while(str[cursor] == ' ' || str[cursor] == '\t' || str[cursor] == '\n')
		cursor++;
	while(str[cursor] != '\0')
	{
		for(int cnt = 0 ; cnt < 3 ; cnt++)
		{
			if(cnt == 0)
				tmpPoint.x = atof(str + cursor);
			else if(cnt == 1)
				tmpPoint.y = atof(str + cursor);
			else
				tmpPoint.z = atof(str + cursor);
			while(str[cursor] != ' ' && str[cursor] != '\t' && str[cursor] != '\n')
				cursor++; 
			while(str[cursor] == ' ' || str[cursor] == '\t' || str[cursor] == '\n')
				cursor++;
		}
		vertices.push_back(tmpPoint);
	}

	// Parse objects
	pElement = pRoot->FirstChildElement("Objects");
	
	// Parse spheres
	XMLElement *pObject = pElement->FirstChildElement("Sphere");
	XMLElement *objElement;
	while(pObject != nullptr)
	{
		int id;
		int matIndex;
		int cIndex;
		float R;

		eResult = pObject->QueryIntAttribute("id", &id);
		objElement = pObject->FirstChildElement("Material");
		eResult = objElement->QueryIntText(&matIndex);
		objElement = pObject->FirstChildElement("Center");
		eResult = objElement->QueryIntText(&cIndex);
		objElement = pObject->FirstChildElement("Radius");
		eResult = objElement->QueryFloatText(&R);

		objects.push_back(new Sphere(id, matIndex, cIndex, R, &vertices));

		pObject = pObject->NextSiblingElement("Sphere");
	}

	// Parse triangles
	pObject = pElement->FirstChildElement("Triangle");
	while(pObject != nullptr)
	{
		int id;
		int matIndex;
		int p1Index;
		int p2Index;
		int p3Index;

		eResult = pObject->QueryIntAttribute("id", &id);
		objElement = pObject->FirstChildElement("Material");
		eResult = objElement->QueryIntText(&matIndex);
		objElement = pObject->FirstChildElement("Indices");
		str = objElement->GetText();
		sscanf(str, "%d %d %d", &p1Index, &p2Index, &p3Index);

		objects.push_back(new Triangle(id, matIndex, p1Index, p2Index, p3Index, &vertices));

		pObject = pObject->NextSiblingElement("Triangle");
	}

	// Parse meshes
	pObject = pElement->FirstChildElement("Mesh");
	while(pObject != nullptr)
	{
		int id;
		int matIndex;
		int p1Index;
		int p2Index;
		int p3Index;
		int cursor = 0;
		int vertexOffset = 0;
		vector<Triangle> faces;
		vector<int> *meshIndices = new vector<int>;

		eResult = pObject->QueryIntAttribute("id", &id);
		objElement = pObject->FirstChildElement("Material");
		eResult = objElement->QueryIntText(&matIndex);
		objElement = pObject->FirstChildElement("Faces");
		objElement->QueryIntAttribute("vertexOffset", &vertexOffset);
		str = objElement->GetText();
		while(str[cursor] == ' ' || str[cursor] == '\t' || str[cursor] == '\n')
			cursor++;
		while(str[cursor] != '\0')
		{
			for(int cnt = 0 ; cnt < 3 ; cnt++)
			{
				if(cnt == 0)
					p1Index = atoi(str + cursor) + vertexOffset;
				else if(cnt == 1)
					p2Index = atoi(str + cursor) + vertexOffset;
				else
					p3Index = atoi(str + cursor) + vertexOffset;
				while(str[cursor] != ' ' && str[cursor] != '\t' && str[cursor] != '\n')
					cursor++; 
				while(str[cursor] == ' ' || str[cursor] == '\t' || str[cursor] == '\n')
					cursor++;
			}
			faces.push_back(*(new Triangle(-1, matIndex, p1Index, p2Index, p3Index, &vertices)));
			meshIndices->push_back(p1Index);
			meshIndices->push_back(p2Index);
			meshIndices->push_back(p3Index);
		}

		objects.push_back(new Mesh(id, matIndex, faces, meshIndices, &vertices));

		pObject = pObject->NextSiblingElement("Mesh");
	}

	// Parse lights
	int id;
	Vector3f position;
	Vector3f intensity;
	pElement = pRoot->FirstChildElement("Lights");

	XMLElement *pLight = pElement->FirstChildElement("AmbientLight");
	XMLElement *lightElement;
	str = pLight->GetText();
	sscanf(str, "%f %f %f", &ambientLight.r, &ambientLight.g, &ambientLight.b);

	pLight = pElement->FirstChildElement("PointLight");
	while(pLight != nullptr)
	{
		eResult = pLight->QueryIntAttribute("id", &id);
		lightElement = pLight->FirstChildElement("Position");
		str = lightElement->GetText();
		sscanf(str, "%f %f %f", &position.x, &position.y, &position.z);
		lightElement = pLight->FirstChildElement("Intensity");
		str = lightElement->GetText();
		sscanf(str, "%f %f %f", &intensity.r, &intensity.g, &intensity.b);

		lights.push_back(new PointLight(position, intensity));

		pLight = pLight->NextSiblingElement("PointLight");
	}
}

