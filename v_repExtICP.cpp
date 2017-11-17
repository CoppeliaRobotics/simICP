#include "v_repExtICP.h"
#include "v_repLib.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>

#include <boost/algorithm/string/predicate.hpp>

#ifdef _WIN32
    #ifdef QT_COMPIL
        #include <direct.h>
    #else
        #include <shlwapi.h>
        #pragma comment(lib, "Shlwapi.lib")
    #endif
#endif /* _WIN32 */
#if defined (__linux) || defined (__APPLE__)
    #include <unistd.h>
#define _stricmp strcasecmp
#endif /* __linux || __APPLE__ */

#define PLUGIN_VERSION 1

LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind

#include <icpPointToPlane.h>

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

VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer, int reservedInt)
{
    char curDirAndFile[1024];
#ifdef _WIN32
    #ifdef QT_COMPIL
        _getcwd(curDirAndFile, sizeof(curDirAndFile));
    #else
        GetModuleFileNameA(NULL, curDirAndFile, 1023);
        PathRemoveFileSpecA(curDirAndFile);
    #endif
#elif defined (__linux) || defined (__APPLE__)
    getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif

    std::string currentDirAndPath(curDirAndFile);
    std::string temp(currentDirAndPath);
#ifdef _WIN32
    temp+="\\v_rep.dll";
#elif defined (__linux)
    temp+="/libv_rep.so";
#elif defined (__APPLE__)
    temp+="/libv_rep.dylib";
#endif /* __linux || __APPLE__ */
    vrepLib = loadVrepLibrary(temp.c_str());
    if(vrepLib == NULL)
    {
        std::cout << "Error, could not find or correctly load the V-REP library. Cannot start 'Octomap' plugin.\n";
        return 0;
    }
    if(getVrepProcAddresses(vrepLib)==0)
    {
        std::cout << "Error, could not find all required functions in the V-REP library. Cannot start 'Octomap' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return 0;
    }

    int vrepVer;
    simGetIntegerParameter(sim_intparam_program_version, &vrepVer);
    if(vrepVer < 30203) // if V-REP version is smaller than 3.02.03
    {
        std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start 'Octomap' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return 0;
    }

    if(!registerScriptStuff())
    {
        std::cout << "Initialization failed.\n";
        unloadVrepLibrary(vrepLib);
        return 0;
    }

    return PLUGIN_VERSION; // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

VREP_DLLEXPORT void v_repEnd()
{
    unloadVrepLibrary(vrepLib);
}

VREP_DLLEXPORT void* v_repMessage(int message, int* auxiliaryData, void* customData, int* replyData)
{
    // Keep following 4 lines at the beginning and unchanged:
    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode, &errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode, sim_api_errormessage_ignore);
    void *retVal = NULL;

    // Keep following unchanged:
    simSetIntegerParameter(sim_intparam_error_report_mode, errorModeSaved);
    return retVal;
}

