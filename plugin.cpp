#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>

#include <boost/algorithm/string/predicate.hpp>

#include <icpPointToPlane.h>

#include "simPlusPlus/Plugin.h"
#include "plugin.h"
#include "stubs.h"

void match(SScriptCallBack *p, const char *cmd, match_in *in, match_out *out)
{
    int modl_sz = 0;
    const float *modl_pts = simGetPointCloudPoints(in->model_handle, &modl_sz, 0);
    if(!modl_pts) throw std::string("model point cloud empty");
    double *modl = new double[3 * modl_sz];
    for(size_t i = 0; i < 3 * modl_sz; i++) modl[i] = modl_pts[i];

    int tmpl_sz = 0;
    const float *tmpl_pts = simGetPointCloudPoints(in->template_handle, &tmpl_sz, 0);
    if(!tmpl_pts) throw std::string("model point cloud empty");
    double *tmpl = new double[3 * tmpl_sz];
    for(size_t i = 0; i < 3 * tmpl_sz; i++) tmpl[i] = tmpl_pts[i];

    Matrix R = Matrix::eye(3);
    Matrix t(3,1);

    IcpPointToPlane icp(modl, modl_sz, 3);
    icp.fit(tmpl, tmpl_sz, R, t, in->outlier_treshold);

    out->m.resize(12);
    for(int row = 0; row < 3; row++)
    {
        for(int j = 0; j < 3; j++)
            out->m[row * 4 + j] = R.val[row][j];
        out->m[row * 4 + 3] = t.val[row][0];
    }
}

void matchToShape(SScriptCallBack *p, const char *cmd, matchToShape_in *in, matchToShape_out *out)
{
    // create a point cloud from given shape
    double voxel_size = 0.005;
    int model_cloud = simCreatePointCloud(voxel_size, 1, 0, 1, 0);
    simInsertObjectIntoPointCloud(model_cloud, in->model_handle, 0, voxel_size, NULL, NULL);

    out->m = match(0, model_cloud, in->template_handle, in->outlier_treshold);

    simRemoveObject(model_cloud);
}

class Plugin : public sim::Plugin
{
public:
    void onStart()
    {
        if(!registerScriptStuff())
            throw std::runtime_error("failed to register script stuff");

        simSetModuleInfo(PLUGIN_NAME, 0, "ICP (iterative closest point algorithm) Plugin", 0);
        simSetModuleInfo(PLUGIN_NAME, 1, BUILD_DATE, 0);
    }
};

SIM_PLUGIN(PLUGIN_NAME, PLUGIN_VERSION, Plugin)
