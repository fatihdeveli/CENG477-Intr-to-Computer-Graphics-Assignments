#ifndef _SCENE_H_
#define _SCENE_H_

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "Camera.h"
#include "Color.h"
#include "Model.h"
#include "Rotation.h"
#include "Scaling.h"
#include "Translation.h"
#include "Triangle.h"
#include "Vec3.h"
#include "Vec4.h"
#include "Matrix4.h"

using namespace std;

class Scene
{
public:
	Color backgroundColor;
	bool cullingEnabled;
	int projectionType;

	vector< vector<Color> > image;
	vector< Camera* > cameras;
	vector< Vec3* > vertices;
	vector< Color* > colorsOfVertices;
	vector< Scaling* > scalings;
	vector< Rotation* > rotations;
	vector< Translation* > translations;
	vector< Model* > models;

	Scene(const char *xmlPath);

	void initializeImage(Camera* camera);
	void forwardRenderingPipeline(Camera* camera);
	int makeBetweenZeroAnd255(double value);
	void writeImageToPPMFile(Camera* camera);
	void convertPPMToPNG(string ppmFileName, int osType);

private:
    Matrix4 getTranslationMatrix(int id);
    Matrix4 getScalingMatrix(int id);
    Matrix4 getRotationMatrix(int id);
    Matrix4 getModelingTransformationMatrix(Model* model);
    Matrix4 getMatrixOfType(char type, int id);
    static Matrix4 getCameraTransformationMatrix(Camera *camera);
    static Matrix4 getPerspectiveProjectionMatrix(Camera *camera);
    static Matrix4 getOrthographicProjectionMatrix(Camera *camera);

    static Vec3 generateVFromU(const Vec3 &u, int minIndex);

    static void perspectiveDivide(Vec4 &vec);

    static Vec3 viewPortTransformation(Camera* camera, const Vec4 &vec);

    void lineRasterization(Vec3 &v1, Vec3 &v2);
    void triangleRasterization(const Vec3 &v0, const Vec3 &v1, const Vec3 &v2);
};

#endif
