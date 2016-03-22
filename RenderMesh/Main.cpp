#include <string>
#include <tclap/CmdLine.h>
#include "MeshRendered.h"


#ifdef _DEBUG
#pragma comment(lib, "opencv_core2411d.lib")
#pragma comment(lib, "opencv_highgui2411d.lib")
#pragma comment(lib, "opencv_imgproc2411d.lib")
#pragma comment(lib, "../ann_1.1.2/MS_Win32/dll/Debug/ANN.lib")
#else
#pragma comment(lib, "opencv_core2411.lib")
#pragma comment(lib, "opencv_highgui2411.lib")
#pragma comment(lib, "opencv_imgproc2411.lib")
#pragma comment(lib, "../ann_1.1.2/MS_Win32/dll/Release/ANN.lib")
#endif


int main(int argc, char** argv)
{
	try {
		TCLAP::CmdLine cmd("This program takes a ply mesh model and a set of keyframes camera poses as input, and outputs the rendered viewpoints at the position each keyframe, and optionally the interpolated viewpoints between neighboring keyframes", ' ', "1.0");

		TCLAP::ValueArg<std::string> meshPathArg("", "mesh", "path to input mesh", true, "", "std::string", cmd);
		TCLAP::ValueArg<std::string> keyframeFolderArg("", "kf", "path to keyframe folder", true, "", "std::string", cmd);
		TCLAP::ValueArg<std::string> outputFolderArg("", "out", "path to output folder", true, "", "std::string", cmd);
		TCLAP::ValueArg<std::string> gtMeshPathArg("", "gt-mesh", "path to ground-truth mesh. will rendered mesh errors instead of mesh if enabled", false, "", "std::string", cmd);
		TCLAP::ValueArg<double> minVisErrorArg("", "min-vis-error", "min value of the visualized heat map", false, 0.00000, "double", cmd);
		TCLAP::ValueArg<double> maxVisErrorArg("", "max-vis-error", "max value of the visualized heat map", false, 0.00016, "double", cmd);
		TCLAP::ValueArg<int> framesPerIntervalArg("", "fpi", "number of frames rendered between viewpoints adjacent keyframes", false, 1, "int", cmd);
		
		cmd.parse(argc, argv);

		std::string meshPath		= meshPathArg.getValue();
		std::string keyframeFolder	= keyframeFolderArg.getValue();
		std::string outputFolder	= outputFolderArg.getValue();
		std::string gtMeshPath		= gtMeshPathArg.getValue();
		double minVisError			= minVisErrorArg.getValue();
		double maxVisError			= maxVisErrorArg.getValue();
		int framesPerInterval		= framesPerIntervalArg.getValue();

		std::cout << meshPath			<< "\n";
		std::cout << keyframeFolder		<< "\n";
		std::cout << meshPath			<< "\n";
		std::cout << gtMeshPath			<< "\n";
		std::cout << minVisError		<< "\n";
		std::cout << maxVisError		<< "\n";
		std::cout << framesPerInterval	<< "\n";

		MeshRenderer renderer(meshPath.c_str(), keyframeFolder.c_str(), outputFolder.c_str());
		renderer.m_framesPerInterval = framesPerInterval;

		if (gtMeshPath != "")
		{
			renderer.RenderHeatMap(gtMeshPath.c_str(), minVisError, maxVisError);
		}
		else
		{
			renderer.RenderMesh();
		}
	}
	catch (TCLAP::ArgException &e)  // catch any exceptions
	{
		std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
	}
}