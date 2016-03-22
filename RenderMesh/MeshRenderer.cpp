#include <iostream>
#include <vector>
#include <ANN\ANN.h>
#include <GL\glut.h>

#include "MeshRendered.h"

static MeshRenderer *theRenderer = NULL;

static void fnRedrawMesh()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0, 0, 0, 1);
	CV_Assert(theRenderer != NULL);
	MeshRenderer::Mesh& theMesh = theRenderer->m_mesh;
	CV_Assert(theMesh.vertexCoords.size() == theMesh.vertexNormals.size());
	glBegin(GL_TRIANGLES);
	for (int id = 0; id < theMesh.facets.size(); id++)
	{
		for (int i = 0; i < 3; i++)
		{
			Eigen::Vector3f normal = theMesh.vertexNormals[theMesh.facets[id][i]];
			Eigen::Vector3f vertex = theMesh.vertexCoords[theMesh.facets[id][i]];
			glNormal3f(normal(0), normal(1), normal(2));
			glVertex3f(vertex(0), vertex(1), vertex(2));
		}
	}
	glEnd();
	glutSwapBuffers();
}

static void fnRedrawHeatMap()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0, 0, 0, 1);
	CV_Assert(theRenderer != NULL);
	MeshRenderer::Mesh& theMesh = theRenderer->m_mesh;
	CV_Assert(theMesh.vertexCoords.size() == theMesh.vertexNormals.size()
		&& theMesh.vertexCoords.size() == theMesh.vertexColors.size());
	glBegin(GL_TRIANGLES);
	for (int id = 0; id < theMesh.facets.size(); id++)
	{
		for (int i = 0; i < 3; i++)
		{
			Eigen::Vector3f normal = theMesh.vertexNormals[theMesh.facets[id][i]];
			Eigen::Vector3f vertex = theMesh.vertexCoords[theMesh.facets[id][i]];
			Eigen::Vector3f color  = theMesh.vertexColors[theMesh.facets[id][i]];
			glColor3f(color(0), color(1), color(2));
			glNormal3f(normal(0), normal(1), normal(2));
			glVertex3f(vertex(0), vertex(1), vertex(2));
		}
	}
	glEnd();
	glutSwapBuffers();
}

static void fnUpdatePose()
{
	//*********************************** PROJECTION **************************************// 
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	CV_Assert(theRenderer != NULL);
	float fx = theRenderer->m_fx, fy = theRenderer->m_fy, 
		  cx = theRenderer->m_cx, cy = theRenderer->m_cy, 
		  width = theRenderer->m_width, height = theRenderer->m_height;
	fx *= theRenderer->m_currentZoomScale; fy *= theRenderer->m_currentZoomScale;
	float neardist = 0.1, fardist = 2.0;			// in meters
	glFrustum(
		(0 - cx) * neardist / fx, (width - cx) * neardist / fx,
		(0 - cy) * neardist / fy, (height - cy) * neardist / fy,
		neardist, fardist);

	//*********************************** MODELVIEW **************************************// 
	CV_Assert(theRenderer->m_keyframes.size() >= 1);
	int cam0 = theRenderer->m_currentCamID,
		cam1 = (theRenderer->m_currentCamID + 1 + theRenderer->m_keyframes.size()) % theRenderer->m_keyframes.size();
	float alpha = theRenderer->m_currentInterpAlpha;
	Eigen::Isometry3f 
		T0 = theRenderer->m_keyframes[cam0].T,		// local->world
		T1 = theRenderer->m_keyframes[cam1].T;		// local->world

	Eigen::Vector3f eye0	= T0.translation().matrix(), eye1 = T1.translation().matrix();
	Eigen::Vector3f eye		= (1.0f - alpha) * eye0 + alpha * eye1;
	Eigen::Vector3f ray0	= T0.rotation().matrix() * Eigen::Vector3f(0, 0, 1), ray1 = T1.rotation().matrix() * Eigen::Vector3f(0, 0, 1);
	Eigen::Vector3f ray		= (1.0f - alpha) * ray0 + alpha * ray1;
	Eigen::Vector3f center	= eye + ray;
	Eigen::Vector3f up0		= T0.rotation().matrix() * Eigen::Vector3f(0, -1, 0), up1 = T1.rotation().matrix() * Eigen::Vector3f(0, -1, 0);
	Eigen::Vector3f up		= (1.0f - alpha) * up0 + alpha * up1;

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(eye(0), eye(1), eye(2), center(0), center(1), center(2), up(0), up(1), up(2));
}

static void fnResetView()
{
	GLfloat light_position[] = { 0.0, -1.0, -4.0, 0.0 };
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glDepthFunc(GL_LESS);
	glEnable(GL_DEPTH_TEST);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glEnable(GL_COLOR_MATERIAL);	// In order to use glColor under with lighting turned on

	CV_Assert(theRenderer != NULL);
	theRenderer->m_currentCamID = 0;
	theRenderer->m_currentZoomScale = 1.0f;
	theRenderer->m_currentInterpAlpha = 0.0f;
	fnUpdatePose();
}

void MeshRenderer::Mesh::LoadFromPLY(const char *filePathPLY)
{
	void ParsePLY(const char *filePath, std::vector<Eigen::Vector3f> &vertices, std::vector<Eigen::Vector3i> &facets);
	ParsePLY(filePathPLY, vertexCoords, facets);

	// Compute normals.
	vertexNormals.resize(vertexCoords.size());
	std::fill(vertexNormals.begin(), vertexNormals.end(), Eigen::Vector3f(0, 0, 0));
	std::vector<int> adjFacetCnts(vertexNormals.size());
	std::fill(adjFacetCnts.begin(), adjFacetCnts.end(), 0);
	std::vector<Eigen::Vector3f> facetNormals(facets.size());

	for (int i = 0; i < facets.size(); i++)
	{
		int i0 = facets[i][0], i1 = facets[i][1], i2 = facets[i][2];
		Eigen::Vector3f AB = (vertexCoords[i1] - vertexCoords[i0]);
		Eigen::Vector3f AC = (vertexCoords[i2] - vertexCoords[i0]);
		facetNormals[i] = AB.cross(AC);
		vertexNormals[i0] += facetNormals[i]; 
		vertexNormals[i1] += facetNormals[i]; 
		vertexNormals[i2] += facetNormals[i];
		adjFacetCnts[i0]++; 
		adjFacetCnts[i1]++; 
		adjFacetCnts[i2]++;
	}

	for (int i = 0; i < vertexNormals.size(); i++)
	{
		vertexNormals[i] = -vertexNormals[i] / (float)adjFacetCnts[i];
		vertexNormals[i].normalize();
	}
}

void MeshRenderer::Mesh::ComputeVertexColor(MeshRenderer::Mesh &meshGT, const float minErrVal, const float maxErrVal)
{
	std::vector<double> errors(this->vertexCoords.size());

	ANNpointArray annPoints = annAllocPts(meshGT.vertexCoords.size(), 3);
	for (int i = 0; i < meshGT.vertexCoords.size(); i++)	// points in GT
	{
		annPoints[i][0] = meshGT.vertexCoords[i](0); annPoints[i][1] = meshGT.vertexCoords[i](1); annPoints[i][2] = meshGT.vertexCoords[i](2);
	}

	ANNkd_tree *tree = new ANNkd_tree(annPoints, meshGT.vertexCoords.size(), 3);
	ANNpoint query = annAllocPt(3);
	ANNidx nnIdx[1];
	ANNdist dists[1];
	for (int i = 0; i < this->vertexCoords.size(); i++)
	{
		query[0] = this->vertexCoords[i](0); query[1] = this->vertexCoords[i](1); query[2] = this->vertexCoords[i](2);
		tree->annkSearch(query, 1, nnIdx, dists);
		errors[i] = dists[0];
	}
	delete tree;

	/*
		std::vector<double> tmp = errors;
		std::sort(tmp.begin(), tmp.end(), [](const double &a, const double b) {
			return a < b;
		});
		minErrVal = tmp[0.1 * tmp.size()];
		maxErrVal = tmp[0.7 * tmp.size()];
		printf("min-val, max-val = [%f, %f]\n", minErrVal, maxErrVal);
	*/

	vertexColors.resize(this->vertexCoords.size());
	for (int i = 0; i < vertexColors.size(); i++)
	{
		vertexColors[i] = ([](float value, float minimum, float maximum) {	
			/****************** This is a lambda function *******************/
			// Clamp value in to [minimum, maximum] and visualize the interval as heat map.
			// See http://stackoverflow.com/questions/20792445/calculate-rgb-value-for-a-range-of-values-to-create-heat-map
			value = std::max(minimum, std::min(value, maximum));
			float ratio = 2 * (value - minimum) / (maximum - minimum);
			float b = int(std::max(0.0f, 255 * (1 - ratio)));
			float r = int(std::max(0.0f, 255 * (ratio - 1)));
			float g = 255 - b - r;
			return Eigen::Vector3f(r / 255.0f, g / 255.0f, b / 255.0f);
		})(errors[i], minErrVal, maxErrVal);
	}
}

MeshRenderer::MeshRenderer(const char *filePathMesh, const char *keyframeFolder, const char *outputFolder)
	: m_filePathMesh(filePathMesh), m_keyframeFolder(keyframeFolder), m_outputFolder(outputFolder)
{
	// TODO: accept sensor type as an argument instead of hard-coded in this function.
#if 0		//asus xtion
	m_fx = 525.0f; m_fy = 525.0f; m_cx = 319.5f; m_cy = 239.5f;
	m_dMin = 0.5f; m_dMax = 1.5f;
#endif
#if 1		//primesense carmine 1.09
	m_fx = 574.053f; m_fy = 574.053f; m_cx = 320.0f; m_cy = 240.0f;
	m_dMin = 0.05f; m_dMax = 1.5f;
#endif
#if 0		//real sense
	m_fx = 631.247864f; m_fy = 631.247864f; m_cx = 318.404053f; m_cy = 229.425339f;
	m_dMin = 0.15f; m_dMax = 1.5f;
#endif
#if 0		//spitfire
	m_fx = 262.197662f; m_fy = 262.780609f; m_cx = 187.031265f; m_cy = 136.834137f;
	m_dMin = 0.15f; m_dMax = 1.5f;
#endif
#if 0		//Grigio TOF camera
	m_fx = 254.959473f; m_fy = 254.959473f; m_cx = 194.459137f; m_cy = 144.893631f;
	m_dMin = 0.05f; m_dMax = 1.0f;
#endif
#if 0		//realsense F200
	m_fx = 590.476196f; m_fy = 590.476196f; m_cx = 321.676697f; m_cy = 257.888885f;
	m_dMin = 0.05f; m_dMax = 1.5f;
#endif

	this->m_currentCamID = 0;
	this->m_currentZoomScale = 1.0f;
	this->m_currentInterpAlpha = 0.0f;
	this->m_framesPerInterval = 1;
}

void MeshRenderer::RenderMesh()
{
	m_mesh.LoadFromPLY(m_filePathMesh);
	LoadKeyframes();

	theRenderer = this;

	glutInitWindowSize(m_width, m_height);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	char fakeParam[] = "fakeParam";
	char *fakeArgv[] = { fakeParam, NULL };
	int fakeArgc = 1;
	glutInit(&fakeArgc, fakeArgv);
	glutCreateWindow("RenderMesh");
	glutDisplayFunc(fnRedrawMesh);
	fnResetView();

	char cmd[256], filePath[256];
	sprintf_s(cmd, 256, "mkdir %s", m_outputFolder);
	system(cmd);
	printf("Saving meshes");
	for (int fid = 0, cam = 0; cam < m_keyframes.size(); cam++) for (int i = 0; i < m_framesPerInterval; i++)
	{
		printf(".");
		theRenderer->m_currentCamID = cam;
		theRenderer->m_currentInterpAlpha = (float)i / (float)m_framesPerInterval;
		theRenderer->m_currentZoomScale = 1.0f;
		glViewport(0, 0, m_width, m_height);
		fnUpdatePose();
		fnRedrawMesh();
		fnRedrawMesh();						// Redraw two times to ensure both buffers 
											// are holding the current rendered viewpoint.
		cv::Mat canvas(m_height, m_width, CV_8UC3);
		glPixelStorei(GL_PACK_ALIGNMENT, 1);
		glReadPixels(0, 0, m_width, m_height, GL_RGB, GL_UNSIGNED_BYTE, canvas.data);
		cv::flip(canvas, canvas, 0);		// Positive value means flipping around y-axis;
		cv::cvtColor(canvas, canvas, CV_RGB2BGR);
		sprintf_s(filePath, 256, "%s\\%08d.png", m_outputFolder, fid);
		cv::imwrite(filePath, canvas);
		fid++;
	}
	printf("done!\n");
}

void MeshRenderer::RenderHeatMap(const char *filePathGTMesh, const float minVisErrVal, const float maxVisErrVal)
{
	m_mesh.LoadFromPLY(m_filePathMesh);
	Mesh gtMesh;
	gtMesh.LoadFromPLY(filePathGTMesh);
	CV_Assert(0 <= minVisErrVal && minVisErrVal < maxVisErrVal);
	m_mesh.ComputeVertexColor(gtMesh, minVisErrVal, maxVisErrVal);
	LoadKeyframes();

	theRenderer = this;

	glutInitWindowSize(m_width, m_height);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	char fakeParam[] = "fakeParam";
	char *fakeArgv[] = { fakeParam, NULL };
	int fakeArgc = 1;
	glutInit(&fakeArgc, fakeArgv);
	glutCreateWindow("RenderMesh");
	glutDisplayFunc(fnRedrawMesh);
	fnResetView();

	char cmd[256], filePath[256];
	sprintf_s(cmd, 256, "mkdir %s", m_outputFolder);
	system(cmd);
	printf("Saving heat meshes");
	for (int fid = 0, cam = 0; cam < m_keyframes.size(); cam++) for (int i = 0; i < m_framesPerInterval; i++)
	{
		printf(".");
		theRenderer->m_currentCamID = cam;
		theRenderer->m_currentInterpAlpha = (float)i / (float)m_framesPerInterval;
		theRenderer->m_currentZoomScale = 1.0f;
		glViewport(0, 0, m_width, m_height);
		fnUpdatePose();
		fnRedrawHeatMap();
		fnRedrawHeatMap();					// Redraw two times to ensure both buffers 
											// are holding the current rendered viewpoint.
		cv::Mat canvas(m_height, m_width, CV_8UC3);
		glPixelStorei(GL_PACK_ALIGNMENT, 1);
		glReadPixels(0, 0, m_width, m_height, GL_RGB, GL_UNSIGNED_BYTE, canvas.data);
		cv::flip(canvas, canvas, 0);		// Positive value means flipping around y-axis;
		cv::cvtColor(canvas, canvas, CV_RGB2BGR);
		sprintf_s(filePath, 256, "%s\\%08d.png", m_outputFolder, fid);
		cv::imwrite(filePath, canvas);
		fid++;
	}
	printf("done!\n");
}

void MeshRenderer::LoadKeyframes()
{
	m_keyframes.reserve(1000);
	int fid = 0;
	printf("loading keyframes");
	while (true)
	{
		printf(".");
		KEYFRAME kf;

		char fileName[256];
		sprintf_s(fileName, 256, "%s\\%08d.png", m_keyframeFolder, fid);
		kf.color = cv::imread(fileName);
		if (kf.color.data == NULL) break;

		sprintf_s(fileName, 256, "%s\\%08d.bin", m_keyframeFolder, fid);
		FILE *fp;
		fopen_s(&fp, fileName, "rb");
		fread(&m_width, sizeof(int), 1, fp); fread(&m_height, sizeof(int), 1, fp);
		int bytesPerPixel, stride;
		fread(&stride, sizeof(int), 1, fp); fread(&bytesPerPixel, sizeof(int), 1, fp);
		cv::Mat depth(m_height, m_width, CV_16SC1);
		fread(depth.data, m_width * m_height * sizeof(unsigned short), 1, fp);
		fclose(fp);

		kf.depth.create(m_height, m_width, CV_32FC1);
		unsigned short *data = (unsigned short*)depth.data;
		for (int y = 0, off = 0; y < m_height; y++) for (int x = 0; x < m_width; x++, off++)
		{
			float d = (float)data[off] * 0.001f;
			kf.depth.at<float>(y, x) = d;
		}

		kf.T = Eigen::Isometry3f::Identity();
		sprintf_s(fileName, 256, "%s\\%08d.txt", m_keyframeFolder, fid);
		if (fopen_s(&fp, fileName, "rt") != 0)
			break;
		Eigen::Matrix<float, 6, 1> xx;
		for (int j = 0; j < 6; j++)
			fscanf_s(fp, "%f ", &xx[j]);
		fclose(fp);
		Sophus::SE3f se3 = Sophus::SE3f::exp(xx);
		kf.T.matrix() = se3.matrix();
		m_keyframes.push_back(kf);

		fid++;
	}
	m_keyframes.shrink_to_fit();
	printf("done(%d)!\n", (int)m_keyframes.size());
}

