#pragma once;

#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <Eigen\Core>
#include <Eigen\Dense>
#include <sophus\se3.hpp>

class MeshRenderer
{
public:
	struct KEYFRAME
	{
		int fid;				// frame id
		cv::Mat color, depth;	// color.channels == 3, bgr, color.channels == 1, ir or gray, depth.type = float
		Eigen::Isometry3f T;	// local->world
	};

	struct Mesh
	{
	public:
		void LoadFromPLY(const char *filePathPLY);
		void ComputeVertexColor(MeshRenderer::Mesh &gtMesh, const float minErrVal, const float maxErrVal);

	public:
		std::vector<Eigen::Vector3f> vertexCoords;
		std::vector<Eigen::Vector3f> vertexNormals;
		std::vector<Eigen::Vector3f> vertexColors;
		std::vector<Eigen::Vector3i> facets;
	};

public:
	MeshRenderer(const char *filePathMesh, const char *keyframeFolder, const char *outputFolder);
	void RenderMesh();
	void MeshRenderer::RenderHeatMap(const char *filePathGTMesh, const float minVisErrVal, const float maxVisErrVal);

protected:
	void LoadKeyframes();

public:
	Mesh					m_mesh;
	std::vector<KEYFRAME>	m_keyframes;
	const char				*m_filePathMesh;
	const char				*m_keyframeFolder;
	const char				*m_outputFolder;

public:
	float		m_fx, m_fy, m_cx, m_cy, m_dMin, m_dMax;
	int			m_width, m_height;
	int			m_framesPerInterval;
	float		m_currentZoomScale, m_currentInterpAlpha;
	int			m_currentCamID;
};