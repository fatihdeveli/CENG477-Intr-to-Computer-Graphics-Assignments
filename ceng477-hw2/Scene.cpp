#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <fstream>
#include <cmath>
#include <algorithm>

#include "Scene.h"
#include "Camera.h"
#include "Color.h"
#include "Model.h"
#include "Rotation.h"
#include "Scaling.h"
#include "Translation.h"
#include "Triangle.h"
#include "Vec3.h"
#include "tinyxml2.h"
#include "Helpers.h"

#define TO_RADIAN 0.01745327777777

using namespace tinyxml2;
using namespace std;

/*
	Transformations, clipping, culling, rasterization are done here.
	You can define helper functions inside Scene class implementation.
*/
void Scene::forwardRenderingPipeline(Camera *camera)
{
	// TODO: Implement this function.

	for (auto model : models) {
	    Matrix4 modelingTransformationMatrix = getModelingTransformationMatrix(model);
	    Matrix4 cameraTransformationMatrix = getCameraTransformationMatrix(camera);

	    Matrix4 Mcam_Mmodel = multiplyMatrixWithMatrix(cameraTransformationMatrix, modelingTransformationMatrix);

	    Matrix4 Mprojection_Mcam_Mmodel;
	    if (projectionType) { // Perspective
	        Matrix4 Mper = getPerspectiveProjectionMatrix(camera);
            Mprojection_Mcam_Mmodel = multiplyMatrixWithMatrix(Mper, Mcam_Mmodel);
	    }
	    else { // Ortographic
	        Matrix4 Mort = getOrthographicProjectionMatrix(camera);
	        Mprojection_Mcam_Mmodel = multiplyMatrixWithMatrix(Mort, Mcam_Mmodel);
	    }

	    // Perspective divide and Viewport inside in loop, which visits all vertices of each triangle
	    for(auto triangle : model->triangles){
	        Vec3* firstVertex = vertices[triangle.getFirstVertexId()-1];
            Vec3* secondVertex = vertices[triangle.getSecondVertexId()-1];
            Vec3* thirdVertex = vertices[triangle.getThirdVertexId()-1];

	        Vec4 firstVertexVec4(firstVertex->x, firstVertex->y, firstVertex->z, 1, firstVertex->colorId);
            Vec4 secondVertexVec4(secondVertex->x, secondVertex->y, secondVertex->z, 1, secondVertex->colorId);
            Vec4 thirdVertexVec4(thirdVertex->x, thirdVertex->y, thirdVertex->z, 1, thirdVertex->colorId);

            firstVertexVec4 = multiplyMatrixWithVec4(Mprojection_Mcam_Mmodel, firstVertexVec4);
            secondVertexVec4 = multiplyMatrixWithVec4(Mprojection_Mcam_Mmodel, secondVertexVec4);
            thirdVertexVec4 = multiplyMatrixWithVec4(Mprojection_Mcam_Mmodel, thirdVertexVec4);

            perspectiveDivide(firstVertexVec4);
            perspectiveDivide(secondVertexVec4);
            perspectiveDivide(thirdVertexVec4);

            Vec3 transformedFirstVertex = viewPortTransformation(camera, firstVertexVec4);
            Vec3 transformedSecondVertex = viewPortTransformation(camera, secondVertexVec4);
            Vec3 transformedThirdVertex = viewPortTransformation(camera, thirdVertexVec4);

            if (cullingEnabled) {
                Vec3 edge1 = subtractVec3(transformedSecondVertex, transformedFirstVertex);
                Vec3 edge2 = subtractVec3(transformedThirdVertex, transformedFirstVertex);

                Vec3 normal = crossProductVec3(edge2, edge1);
                normal = normalizeVec3(normal);

                Vec3 viewingVector = transformedSecondVertex;
                viewingVector = normalizeVec3(viewingVector);

                if (dotProductVec3(viewingVector, normal) > 0) {
                    continue;
                }
            }

            transformedFirstVertex.colorId = firstVertex->colorId;
            transformedSecondVertex.colorId = secondVertex->colorId;
            transformedThirdVertex.colorId = thirdVertex->colorId;

            if (model->type) { // Solid rendering
                triangleRasterization(transformedFirstVertex, transformedSecondVertex, transformedThirdVertex);
            }
            else { // Wireframe rendering
                lineRasterization(transformedFirstVertex, transformedSecondVertex);
                lineRasterization(transformedSecondVertex, transformedThirdVertex);
                lineRasterization(transformedThirdVertex, transformedFirstVertex);
            }

            int x = (int) transformedFirstVertex.x;
            int y = (int) transformedFirstVertex.y;
            if (x <= camera->horRes && x >= 0 && y >= 0 && y <= camera->verRes) {
                Color* color = colorsOfVertices[transformedFirstVertex.colorId-1];
                image[x][y].r = color->r;
                image[x][y].g = color->g;
                image[x][y].b = color->b;
            }

            x = (int) transformedSecondVertex.x;
            y = (int) transformedSecondVertex.y;
            if (x <= camera->horRes && x >= 0 && y >= 0 && y <= camera->verRes) {
                Color* color = colorsOfVertices[transformedSecondVertex.colorId-1];
                image[x][y].r = color->r;
                image[x][y].g = color->g;
                image[x][y].b = color->b;
            }

            x = (int) transformedThirdVertex.x;
            y = (int) transformedThirdVertex.y;
            if (x <= camera->horRes && x >= 0 && y >= 0 && y <= camera->verRes) {
                Color* color = colorsOfVertices[transformedThirdVertex.colorId-1];
                image[x][y].r = color->r;
                image[x][y].g = color->g;
                image[x][y].b = color->b;
            }
	    }
	}
}




Matrix4 Scene::getModelingTransformationMatrix(Model *model) {
    Matrix4 result = getIdentityMatrix();
    for (int i = 0; i < model->transformationIds.size(); i++) {
        char type = model->transformationTypes[i];
        int id = model->transformationIds[i];
        result = multiplyMatrixWithMatrix(getMatrixOfType(type, id),result );
    }
    return result;
}


Matrix4 Scene::getMatrixOfType(char type, int id) {
    if(type == 'r'){
        return getRotationMatrix(id);
    }
    else if (type == 't'){
        return getTranslationMatrix(id);
    }
    else if(type == 's'){
        return getScalingMatrix(id);
    }
    return nullptr;
}

Matrix4 Scene::getTranslationMatrix(int id) {
    for (auto translation : translations) {
        if (translation->translationId == id) {
            double translationMatrix[4][4] = {{1, 0, 0, translation->tx},
                                              {0, 1, 0, translation->ty},
                                              {0, 0, 1, translation->tz},
                                              {0, 0, 0, 1}};
            return Matrix4(translationMatrix);
        }
    }
    return nullptr;
}

Matrix4 Scene::getScalingMatrix(int id) {
    for (auto scaling : scalings) {
        if (scaling->scalingId == id) {
            double scalingMatrix[4][4] = {{scaling->sx, 0, 0, 0},
                                          {0, scaling->sy, 0, 0},
                                          {0, 0, scaling->sz, 0},
                                          {0, 0, 0, 1}};
            return Matrix4(scalingMatrix);
        }
    }
    return nullptr;
}

Matrix4 Scene::getRotationMatrix(int id) {
    for (auto rotation : rotations) {
        if (rotation->rotationId == id) {
            // To find v, set the smallest component of u (absolute) to zero and swap the other two while negating one.

            Vec3 u(rotation->ux, rotation->uy, rotation->uz, -1);

            u = normalizeVec3(u);

            vector<double> uVector{abs(u.getElementAt(0)), abs(u.getElementAt(1)), abs(u.getElementAt(2))};

            auto result = std::min_element(uVector.begin(), uVector.end());
            int minIndex = distance(uVector.begin(), result);

            Vec3 v = generateVFromU(u, minIndex);

            Vec3 w = crossProductVec3(u, v);
            v = normalizeVec3(v);
            w = normalizeVec3(w);

            double angle = rotation->angle * TO_RADIAN;
            double rotationMatrix[4][4] = {{1, 0, 0, 0},
                                           {0, cos(angle), -sin(angle), 0},
                                           {0, sin(angle), cos(angle), 0},
                                           {0, 0, 0, 1}};
            Matrix4 Rx(rotationMatrix);
            double M_matrix[4][4] = {{u.x, u.y, u.z, 0},
                              {v.x, v.y, v.z, 0},
                              {w.x, w.y, w.z, 0},
                              {0, 0, 0, 1}};
            Matrix4 M(M_matrix);

            double M_inverseMatrix[4][4] = {{u.x, v.x, w.x, 0},
                                            {u.y, v.y, w.y, 0},
                                            {u.z, v.z, w.z, 0},
                                            {0, 0, 0, 1}};
            Matrix4 M_inverse(M_inverseMatrix);

            return multiplyMatrixWithMatrix(M_inverse, multiplyMatrixWithMatrix(Rx, M));
        }
    }
    return nullptr;
}

Vec3 Scene::generateVFromU(const Vec3 &u, int minIndex) {
    Vec3 v(u);

    v.setElementAt(minIndex, 0);

    double temp;
    switch (minIndex) {
        case 0:
            temp = v.getElementAt(1);
            v.setElementAt(1, v.getElementAt(2));
            v.setElementAt(2, -temp);
            break;

        case 1:
            temp = v.getElementAt(0);
            v.setElementAt(0, v.getElementAt(2));
            v.setElementAt(2, -temp);
            break;

        case 2:
            temp = v.getElementAt(0);
            v.setElementAt(0, v.getElementAt(1));
            v.setElementAt(1, -temp);
            break;

        default:
            break;
    }
    return v;
}

Matrix4 Scene::getCameraTransformationMatrix(Camera *camera) {
    double ux = camera->u.x;
    double uy = camera->u.y;
    double uz = camera->u.z;

    double vx = camera->v.x;
    double vy = camera->v.y;
    double vz = camera->v.z;

    double wx = camera->w.x;
    double wy = camera->w.y;
    double wz = camera->w.z;

    double ex = camera->pos.x;
    double ey = camera->pos.y;
    double ez = camera->pos.z;

    double sum1 = ux*ex + uy*ey + uz*ez;
    double sum2 = vx*ex + vy*ey + vz*ez;
    double sum3 = wx*ex + wy*ey + wz*ez;

    double cameraTransformation[4][4] = {
            {ux, uy, uz, -sum1},
            {vx, vy, vz, -sum2},
            {wx, wy, wz, -sum3},
            {0, 0, 0, 1}
    };

    return Matrix4(cameraTransformation);
}

Matrix4 Scene::getPerspectiveProjectionMatrix(Camera *camera) {
    double n = camera->near;
    double f = camera->far;
    double l = camera->left;
    double r = camera->right;
    double t = camera->top;
    double b = camera->bottom;

    double matrix[4][4] = {{2*n/(r-l), 0,         (r+l)/(r-l),  0},
                           {0,         2*n/(t-b), (t+b)/(t-b),  0},
                           {0,         0,         -(f+n)/(f-n), -2*f*n/(f-n)},
                           {0,         0,         -1,           0}};

    return Matrix4(matrix);
}


Matrix4 Scene::getOrthographicProjectionMatrix(Camera *camera) {
    double n = camera->near;
    double f = camera->far;
    double l = camera->left;
    double r = camera->right;
    double t = camera->top;
    double b = camera->bottom;

    double matrix[4][4] = {{2/(r-l), 0, 0, -(r+l)/(r-l)},
                           {0, 2/(t-b), 0, -(t+b)/(t-b)},
                           {0, 0, -2/(f-n), -(f+n)/(f-n)},
                           {0, 0, 0, 1}};

    return Matrix4(matrix);
}

void Scene::perspectiveDivide(Vec4 &vec) {
    if (vec.t != 0) {
        vec.x = vec.x/vec.t;
        vec.y = vec.y/vec.t;
        vec.z = vec.z/vec.t;
        vec.t = 1;
    }
}

Vec3 Scene::viewPortTransformation(Camera* camera, const Vec4 &vec) {
    double nx = camera->horRes;
    double ny = camera->verRes;
    Vec4 transformationMatrix[3] = {Vec4(nx/2, 0,    0,   (nx-1)/2, -1),
                                    Vec4(0,    ny/2, 0,   (ny-1)/2, -1),
                                    Vec4(0,    0,    0.5, 0.5, -1)};

    Vec3 result;
    for (int i = 0; i < 3; i++) {
        Vec4 row = transformationMatrix[i];
        result.setElementAt(i,row.x*vec.x + row.y*vec.y + row.z*vec.z + row.t*vec.t);
    }
    return result;
}

double lineEquation(int x, int y, int x0, int y0, int x1, int y1) {
    return x*(y0-y1) + y*(x1-x0) + x0*y1 - y0*x1;
}

void Scene::triangleRasterization(const Vec3 &v0, const Vec3 &v1, const Vec3 &v2){
    int x0 = (int) v0.x;
    int y0 = (int) v0.y;
    int x1 = (int) v1.x;
    int y1 = (int) v1.y;
    int x2 = (int) v2.x;
    int y2 = (int) v2.y;

    Color* c0 = colorsOfVertices[v0.colorId-1];
    Color* c1 = colorsOfVertices[v1.colorId-1];
    Color* c2 = colorsOfVertices[v2.colorId-1];

    int y_min = min(y0, min(y1, y2));
    int y_max = max(y0, max(y1, y2));
    int x_min = min(x0, min(x1, x2));
    int x_max = max(x0, max(x1, x2));


    for (int y = y_min; y <= y_max; y++) {
        for (int x = x_min; x <= x_max; x++) {
            double alpha = lineEquation(x, y, x1, y1, x2, y2) / lineEquation(x0, y0, x1, y1, x2, y2);
            double beta = lineEquation(x, y, x2, y2, x0, y0) / lineEquation(x1, y1, x2, y2, x0, y0);
            double gamma = lineEquation(x, y, x0, y0, x1, y1) / lineEquation(x2, y2, x0, y0, x1, y1);
            if (alpha >= 0 && beta >= 0 && gamma >= 0) {
                Color c;
                c.r = round(alpha*c0->r + beta*c1->r + gamma*c2->r);
                c.g = round(alpha*c0->g + beta*c1->g + gamma*c2->g);
                c.b = round(alpha*c0->b + beta*c1->b + gamma*c2->b);
                image[x][y] = c;
            }
        }
    }
}

void Scene::lineRasterization(Vec3 &v0, Vec3 &v1){
    int x0 = (int) v0.x;
    int y0 = (int) v0.y;
    int x1 = (int) v1.x;
    int y1 = (int) v1.y;

    double m = (double) (y1-y0)/(x1-x0);

    if (m > 0 && m <= 1) {
        int y = y0;
        double d = (y0-y1) + 0.5*(x1-x0);
        const Color* c0 = colorsOfVertices[v0.colorId-1];
        const Color* c1 = colorsOfVertices[v1.colorId-1];
        Color c = *c0;
        Color dc;

        double x1_x0 = v1.x - v0.x;
        double y0_y1 = v0.y - v1.y;
        dc.r = (c1->r - c0->r) / x1_x0;
        dc.g = (c1->g - c0->g) / x1_x0;
        dc.b = (c1->b - c0->b) / x1_x0;

        for (int x = x0; x <= x1; x++) {
            if (d < 0) {
                y++;
                d += y0_y1 + x1_x0;
            }
            else {
                d += (y0_y1);
            }
            c.r += dc.r;
            c.g += dc.g;
            c.b += dc.b;
            image[x][y] = c;
        }
    }

    else if(m<0 && m>-1){
        int y = y0;
        double d = (y0-y1) + 0.5*(x1-x0);
        const Color* c0 = colorsOfVertices[v0.colorId-1];
        const Color* c1 = colorsOfVertices[v1.colorId-1];
        Color c = *c0;
        Color dc;

        double x1_x0 = v1.x - v0.x;
        double y0_y1 = v0.y - v1.y;
        dc.r = (c1->r - c0->r) / x1_x0;
        dc.g = (c1->g - c0->g) / x1_x0;
        dc.b = (c1->b - c0->b) / x1_x0;

        for (int x = x0; x <= x1; x++) {
            if (d > 0) {
                y--;
                d += y0_y1 - x1_x0;
            }
            else {
                d += (y0_y1);
            }
            c.r += dc.r;
            c.g += dc.g;
            c.b += dc.b;
            image[x][y] = c;
        }


    }

    else if(m>1){
        int x = x0;
        double d = (x0-x1) + 0.5*(y1-y0);
        const Color* c0 = colorsOfVertices[v0.colorId-1];
        const Color* c1 = colorsOfVertices[v1.colorId-1];
        Color c = *c0;
        Color dc;

        double y1_y0 = y1 - y0;
        double x0_x1 = x0 - x1;
        dc.r = (c1->r - c0->r) / y1_y0;
        dc.g = (c1->g - c0->g) / y1_y0;
        dc.b = (c1->b - c0->b) / y1_y0;

        for (int y = y0; y <= y1; y++) {
            if (d < 0) {
                x++;
                d += x0_x1 + y1_y0;
            }
            else {
                d += (x0_x1);
            }
            c.r += dc.r;
            c.g += dc.g;
            c.b += dc.b;
            image[x][y] = c;
        }
    }
    else if(m<-1){
        int x = x0;
        double d = (x0-x1) + 0.5*(y1-y0);
        const Color* c0 = colorsOfVertices[v0.colorId-1];
        const Color* c1 = colorsOfVertices[v1.colorId-1];
        Color c = *c0;
        Color dc;

        double y1_y0 = y1 - y0;
        double x0_x1 = x0 - x1;
        dc.r = (c1->r - c0->r) / y1_y0;
        dc.g = (c1->g - c0->g) / y1_y0;
        dc.b = (c1->b - c0->b) / y1_y0;

        for (int y = y0; y <= y1; y++) {
            if (d > 0) {
                x--;
                d += x0_x1 - y1_y0;
            }
            else {
                d += (x0_x1);
            }
            c.r += dc.r;
            c.g += dc.g;
            c.b += dc.b;
            image[x][y] = c;
        }
    }
}

/*
	Parses XML file
*/
Scene::Scene(const char *xmlPath)
{
	const char *str;
	XMLDocument xmlDoc;
	XMLElement *pElement;

	xmlDoc.LoadFile(xmlPath);

	XMLNode *pRoot = xmlDoc.FirstChild();

	// read background color
	pElement = pRoot->FirstChildElement("BackgroundColor");
	str = pElement->GetText();
	sscanf(str, "%lf %lf %lf", &backgroundColor.r, &backgroundColor.g, &backgroundColor.b);

	// read culling
	pElement = pRoot->FirstChildElement("Culling");
	if (pElement != NULL)
		pElement->QueryBoolText(&cullingEnabled);

	// read projection type
	pElement = pRoot->FirstChildElement("ProjectionType");
	if (pElement != NULL)
		pElement->QueryIntText(&projectionType);

	// read cameras
	pElement = pRoot->FirstChildElement("Cameras");
	XMLElement *pCamera = pElement->FirstChildElement("Camera");
	XMLElement *camElement;
	while (pCamera != NULL)
	{
		Camera *cam = new Camera();

		pCamera->QueryIntAttribute("id", &cam->cameraId);

		camElement = pCamera->FirstChildElement("Position");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->pos.x, &cam->pos.y, &cam->pos.z);

		camElement = pCamera->FirstChildElement("Gaze");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->gaze.x, &cam->gaze.y, &cam->gaze.z);

		camElement = pCamera->FirstChildElement("Up");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->v.x, &cam->v.y, &cam->v.z);

		cam->gaze = normalizeVec3(cam->gaze);
		cam->u = crossProductVec3(cam->gaze, cam->v);
		cam->u = normalizeVec3(cam->u);

		cam->w = inverseVec3(cam->gaze);
		cam->v = crossProductVec3(cam->u, cam->gaze);
		cam->v = normalizeVec3(cam->v);

		camElement = pCamera->FirstChildElement("ImagePlane");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf %lf %lf %lf %d %d",
			   &cam->left, &cam->right, &cam->bottom, &cam->top,
			   &cam->near, &cam->far, &cam->horRes, &cam->verRes);

		camElement = pCamera->FirstChildElement("OutputName");
		str = camElement->GetText();
		cam->outputFileName = string(str);

		cameras.push_back(cam);

		pCamera = pCamera->NextSiblingElement("Camera");
	}

	// read vertices
	pElement = pRoot->FirstChildElement("Vertices");
	XMLElement *pVertex = pElement->FirstChildElement("Vertex");
	int vertexId = 1;

	while (pVertex != NULL)
	{
		Vec3 *vertex = new Vec3();
		Color *color = new Color();

		vertex->colorId = vertexId;

		str = pVertex->Attribute("position");
		sscanf(str, "%lf %lf %lf", &vertex->x, &vertex->y, &vertex->z);

		str = pVertex->Attribute("color");
		sscanf(str, "%lf %lf %lf", &color->r, &color->g, &color->b);

		vertices.push_back(vertex);
		colorsOfVertices.push_back(color);

		pVertex = pVertex->NextSiblingElement("Vertex");

		vertexId++;
	}

	// read translations
	pElement = pRoot->FirstChildElement("Translations");
	XMLElement *pTranslation = pElement->FirstChildElement("Translation");
	while (pTranslation != NULL)
	{
		Translation *translation = new Translation();

		pTranslation->QueryIntAttribute("id", &translation->translationId);

		str = pTranslation->Attribute("value");
		sscanf(str, "%lf %lf %lf", &translation->tx, &translation->ty, &translation->tz);

		translations.push_back(translation);

		pTranslation = pTranslation->NextSiblingElement("Translation");
	}

	// read scalings
	pElement = pRoot->FirstChildElement("Scalings");
	XMLElement *pScaling = pElement->FirstChildElement("Scaling");
	while (pScaling != NULL)
	{
		Scaling *scaling = new Scaling();

		pScaling->QueryIntAttribute("id", &scaling->scalingId);
		str = pScaling->Attribute("value");
		sscanf(str, "%lf %lf %lf", &scaling->sx, &scaling->sy, &scaling->sz);

		scalings.push_back(scaling);

		pScaling = pScaling->NextSiblingElement("Scaling");
	}

	// read rotations
	pElement = pRoot->FirstChildElement("Rotations");
	XMLElement *pRotation = pElement->FirstChildElement("Rotation");
	while (pRotation != NULL)
	{
		Rotation *rotation = new Rotation();

		pRotation->QueryIntAttribute("id", &rotation->rotationId);
		str = pRotation->Attribute("value");
		sscanf(str, "%lf %lf %lf %lf", &rotation->angle, &rotation->ux, &rotation->uy, &rotation->uz);

		rotations.push_back(rotation);

		pRotation = pRotation->NextSiblingElement("Rotation");
	}

	// read models
	pElement = pRoot->FirstChildElement("Models");

	XMLElement *pModel = pElement->FirstChildElement("Model");
	XMLElement *modelElement;
	while (pModel != NULL)
	{
		Model *model = new Model();

		pModel->QueryIntAttribute("id", &model->modelId);
		pModel->QueryIntAttribute("type", &model->type);

		// read model transformations
		XMLElement *pTransformations = pModel->FirstChildElement("Transformations");
		XMLElement *pTransformation = pTransformations->FirstChildElement("Transformation");

		pTransformations->QueryIntAttribute("count", &model->numberOfTransformations);

		while (pTransformation != NULL)
		{
			char transformationType;
			int transformationId;

			str = pTransformation->GetText();
			sscanf(str, "%c %d", &transformationType, &transformationId);

			model->transformationTypes.push_back(transformationType);
			model->transformationIds.push_back(transformationId);

			pTransformation = pTransformation->NextSiblingElement("Transformation");
		}

		// read model triangles
		XMLElement *pTriangles = pModel->FirstChildElement("Triangles");
		XMLElement *pTriangle = pTriangles->FirstChildElement("Triangle");

		pTriangles->QueryIntAttribute("count", &model->numberOfTriangles);

		while (pTriangle != NULL)
		{
			int v1, v2, v3;

			str = pTriangle->GetText();
			sscanf(str, "%d %d %d", &v1, &v2, &v3);

			model->triangles.push_back(Triangle(v1, v2, v3));

			pTriangle = pTriangle->NextSiblingElement("Triangle");
		}

		models.push_back(model);

		pModel = pModel->NextSiblingElement("Model");
	}
}

/*
	Initializes image with background color
*/
void Scene::initializeImage(Camera *camera)
{
	if (this->image.empty())
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			vector<Color> rowOfColors;

			for (int j = 0; j < camera->verRes; j++)
			{
				rowOfColors.push_back(this->backgroundColor);
			}

			this->image.push_back(rowOfColors);
		}
	}
	// if image is filled before, just change color rgb values with the background color
	else
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			for (int j = 0; j < camera->verRes; j++)
			{
				this->image[i][j].r = this->backgroundColor.r;
				this->image[i][j].g = this->backgroundColor.g;
				this->image[i][j].b = this->backgroundColor.b;
			}
		}
	}
}

/*
	If given value is less than 0, converts value to 0.
	If given value is more than 255, converts value to 255.
	Otherwise returns value itself.
*/
int Scene::makeBetweenZeroAnd255(double value)
{
	if (value >= 255.0)
		return 255;
	if (value <= 0.0)
		return 0;
	return (int)(value);
}

/*
	Writes contents of image (Color**) into a PPM file.
*/
void Scene::writeImageToPPMFile(Camera *camera)
{
	ofstream fout;

	fout.open(camera->outputFileName.c_str());

	fout << "P3" << endl;
	fout << "# " << camera->outputFileName << endl;
	fout << camera->horRes << " " << camera->verRes << endl;
	fout << "255" << endl;

	for (int j = camera->verRes - 1; j >= 0; j--)
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			fout << makeBetweenZeroAnd255(this->image[i][j].r) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].g) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].b) << " ";
		}
		fout << endl;
	}
	fout.close();
}

/*
	Converts PPM image in given path to PNG file, by calling ImageMagick's 'convert' command.
	os_type == 1 		-> Ubuntu
	os_type == 2 		-> Windows
	os_type == other	-> No conversion
*/
void Scene::convertPPMToPNG(string ppmFileName, int osType)
{
	string command;

	// call command on Ubuntu
	if (osType == 1)
	{
		command = "convert " + ppmFileName + " " + ppmFileName + ".png";
		system(command.c_str());
	}

	// call command on Windows
	else if (osType == 2)
	{
		command = "magick convert " + ppmFileName + " " + ppmFileName + ".png";
		system(command.c_str());
	}

	// default action - don't do conversion
	else
	{
	}
}