#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>

#include <icpPointToPlane.h>

#include <simPlusPlus/Plugin.h>
#include "config.h"
#include "plugin.h"
#include "stubs.h"

class Plugin : public sim::Plugin
{
public:
    void onInit()
    {
        if(!registerScriptStuff())
            throw std::runtime_error("failed to register script stuff");

        setExtVersion("ICP (iterative closest point algorithm) Plugin");
        setBuildDate(BUILD_DATE);
    }

    void match(match_in *in, match_out *out)
    {
        std::vector<double> modl = sim::getPointCloudPoints(in->model_handle);
        std::vector<double> tmpl = sim::getPointCloudPoints(in->template_handle);
        if(modl.empty()) throw std::runtime_error("model point cloud empty");
        if(tmpl.empty()) throw std::runtime_error("model point cloud empty");

        Matrix R = Matrix::eye(3);
        Matrix t(3,1);

        IcpPointToPlane icp(modl.data(), modl.size() / 3, 3);
        icp.fit(tmpl.data(), tmpl.size() / 3, R, t, in->outlier_treshold);

        out->m.resize(12);
        for(int row = 0; row < 3; row++)
        {
            for(int j = 0; j < 3; j++)
                out->m[row * 4 + j] = R.val[row][j];
            out->m[row * 4 + 3] = t.val[row][0];
        }
    }

    void matchToShape(matchToShape_in *in, matchToShape_out *out)
    {
        // create a point cloud from given shape
        double voxel_size = 0.005;
        int model_cloud = sim::createPointCloud(voxel_size, 1, 0, 1);
        sim::insertObjectIntoPointCloud(model_cloud, in->model_handle, 0, voxel_size);

        match_in args;
        args._ = in->_;
        args.model_handle = model_cloud;
        args.template_handle = in->template_handle;
        args.outlier_treshold = in->outlier_treshold;
        match_out ret;
        match(&args, &ret);
        out->m = ret.m;

        sim::removeObjects({model_cloud});
    }
};

SIM_PLUGIN(Plugin)
#include "stubsPlusPlus.cpp"
