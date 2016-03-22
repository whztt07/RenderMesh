#include <stdio.h> 
#include <stdlib.h>
#include <vector>
#include <string>

#include <Eigen\Core>
#include <Eigen\Dense>

#include "rply.h"

static int vertex_cb(p_ply_argument argument) 
{
	long idx, subidx;
	void *pdata;
	ply_get_argument_user_data(argument, &pdata, &subidx);
	ply_get_argument_element(argument, NULL, &idx);
	std::vector<Eigen::Vector3f>& vertices = *(std::vector<Eigen::Vector3f>*)pdata;
	vertices[idx](subidx) = ply_get_argument_value(argument);
	return 1;
}

static int face_cb(p_ply_argument argument) 
{
	long idx, subidx;
	void *pdata;
	ply_get_argument_user_data(argument, &pdata, NULL);
	ply_get_argument_element(argument, NULL, &idx);
	ply_get_argument_property(argument, NULL, NULL, &subidx);
	if (subidx < 0)		// the first element (i.e. the #vertices in the facet) will have subidx -1.
		return 1;
	std::vector<Eigen::Vector3i>& facets = *(std::vector<Eigen::Vector3i>*)pdata;
	facets[idx](subidx) = ply_get_argument_value(argument);
	return 1;
}

void ParsePLY(const char *filePath, std::vector<Eigen::Vector3f> &vertices, std::vector<Eigen::Vector3i> &facets)
{
	p_ply ply = ply_open(filePath, NULL, 0, NULL);
	if (!ply)
	{
		fprintf(stderr, "cannot open ply file %s\n", filePath);
		exit(-1);
	}
	if (!ply_read_header(ply))
	{
		fprintf(stderr, "cannot parse ply header %s\n", filePath);
		exit(-1);
	}

	long numVertices = ply_set_read_cb(ply, "vertex", "x", vertex_cb, NULL, 0);
	printf("[ParsePly] num-vertices = %d\n", (int)numVertices);
	vertices.resize(numVertices);
	ply_set_read_cb(ply, "vertex", "x", vertex_cb, &vertices, 0);
	ply_set_read_cb(ply, "vertex", "y", vertex_cb, &vertices, 1);
	ply_set_read_cb(ply, "vertex", "z", vertex_cb, &vertices, 2);

	long numFacets = ply_set_read_cb(ply, "face", "vertex_indices", face_cb, NULL, 0);
	ply_set_read_cb(ply, "face", "vertex_indices", face_cb, &facets, 0);
	if (numFacets == 0)
	{
		numFacets = ply_set_read_cb(ply, "face", "vertex_index", face_cb, NULL, 0);
		ply_set_read_cb(ply, "face", "vertex_index", face_cb, &facets, 0);
	}
	
	printf("[ParsePly] num-facets = %d\n", (int)numFacets);
	facets.resize(numFacets);
	
	if (!ply_read(ply))
	{
		fprintf(stderr, "ply body inconsitent with header %s\n", filePath);
		exit(-1);
	}
	ply_close(ply);
}

#if 0
int main(int argc, char **argv)
{
	if (argc < 2)
	{
		printf("usage: %s.exe filePathPly\n", argv[0]);
		exit(-1);
	}

	std::vector<Eigen::Vector3f> vertices;
	std::vector<Eigen::Vector3i> facets;

	ParsePly(argv[1], vertices, facets);
	
	for (int i = 0; i < vertices.size(); i++)
		printf("%f, %f, %f\n", vertices[i](0), vertices[i](1), vertices[i](2));
	for (int i = 0; i < facets.size(); i++)
		printf("%d, %d, %d\n", facets[i](0), facets[i](1), facets[i](2));
}
#endif