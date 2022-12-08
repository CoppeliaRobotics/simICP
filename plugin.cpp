#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>

#include <icpPointToPlane.h>

#include "simPlusPlus/Plugin.h"
#include "config.h"
#include "plugin.h"
#include "stubs.h"

class Plugin : public sim::Plugin
{
public:
    void onStart()
    {
        if(!registerScriptStuff())
            throw std::runtime_error("failed to register script stuff");

        setExtVersion("ICP (iterative closest point algorithm) Plugin");
        setBuildDate(BUILD_DATE);
    }

    void match(match_in *in, match_out *out)
    {
        int modl_sz = 0;
        const double *modl = simGetPointCloudPoints(in->model_handle, &modl_sz, 0);
        if(!modl) throw std::runtime_error("model point cloud empty");

        int tmpl_sz = 0;
        const double *tmpl = simGetPointCloudPoints(in->template_handle, &tmpl_sz, 0);
        if(!tmpl) throw std::runtime_error("model point cloud empty");

        Matrix R = Matrix::eye(3);
        Matrix t(3,1);

        std::vector<double> _modl(modl,modl+modl_sz*3);
        IcpPointToPlane icp(_modl.data(), modl_sz, 3);
        std::vector<double> _tmpl(tmpl,tmpl+tmpl_sz*3);
        icp.fit(_tmpl.data(), tmpl_sz, R, t, in->outlier_treshold);

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
        int model_cloud = simCreatePointCloud(voxel_size, 1, 0, 1, 0);
        simInsertObjectIntoPointCloud(model_cloud, in->model_handle, 0, voxel_size, NULL, NULL);

        match_in args;
        args._ = in->_;
        args.model_handle = model_cloud;
        args.template_handle = in->template_handle;
        args.outlier_treshold = in->outlier_treshold;
        match_out ret;
        match(&args, &ret);
        out->m = ret.m;

        simRemoveObject(model_cloud);
    }
};

SIM_PLUGIN(PLUGIN_NAME, PLUGIN_VERSION, Plugin)
#include "stubsPlusPlus.cpp"
