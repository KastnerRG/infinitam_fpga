
#include "tangoMain.h"

#include "../../../../../Engine/TangoEngine.h"

#include <sstream>

//#include <Eigen/Eigen>

#include "tango_client_api.h"
#include "tango_support_api.h"


using namespace std;



TangoMain::TangoMain():
        tango_config_(nullptr)
{
}

TangoMain::~TangoMain()
{
    tango_config_ = nullptr;
};




bool TangoMain::CheckTangoVersion(JNIEnv* env, jobject activity, int min_tango_version)
{
    int version;
    TangoErrorType err = TangoSupport_GetTangoVersion(env, activity, &version);
    return err == TANGO_SUCCESS && version >= min_tango_version;
}

bool TangoMain::OnTangoServiceConnected(JNIEnv* env, jobject binder)
{
    TangoErrorType ret = TangoService_setBinder(env, binder);
    if (ret != TANGO_SUCCESS)
    {
        LOGE(
                "PointCloudApp: Failed to set Binder Tango service with"
                        "error code: %d",
                ret);
        return false;
    }

    TangoSupport_initializeLibrary();
    return true;
}


int TangoMain::TangoSetupConfig()
{
    tango_config_ = TangoService_getConfig(TANGO_CONFIG_DEFAULT);
    if(tango_config_ == nullptr)
    {
        LOGE("Failed to get default config form");
        return TANGO_ERROR;
    }


    int ret = TangoConfig_setBool(tango_config_, "config_enable_auto_recovery", true);
    if(ret != TANGO_SUCCESS)
    {
        LOGE("config_enable_auto_recovery() failed with error code: %d", ret);
        return ret;
    }

    // Enable depth.
    ret = TangoConfig_setBool(tango_config_, "config_enable_depth", true);
    if(ret != TANGO_SUCCESS)
    {
        LOGE("config_enable_depth() failed with error code: %d", ret);
        return ret;
    }

    ret = TangoConfig_setInt32(tango_config_, "config_depth_mode", TANGO_POINTCLOUD_XYZC );
    if(ret != TANGO_SUCCESS)
    {
        LOGE("config_depth_mode() failed with error code: %d", ret);
        return ret;
    }

    ret = TangoConfig_setBool(tango_config_, "config_enable_color_camera", true);
    if(ret != TANGO_SUCCESS)
    {
        LOGE("config_enable_color_camera() failed with error code: %d", ret);
        return ret;
    }

    return ret;
}


int TangoMain::TangoConnectCallbacks()
{
    InfiniTAM::Engine::TangoEngine* tango_engine = InfiniTAM::Engine::TangoEngine::getInstance();


    // Get max number of points
    int32_t max_point_cloud_elements;
    int ret = TangoConfig_getInt32(tango_config_, "max_point_cloud_elements",
                               &max_point_cloud_elements);
    if(ret != TANGO_SUCCESS) {
        LOGE("Failed to query maximum number of point cloud elements.");
        return ret;
    }

    // Initialize
    tango_engine->initialize(max_point_cloud_elements);

    // Connect callbacks
    return tango_engine->connectCallbacks();
}


bool TangoMain::TangoConnect()
{
    TangoErrorType err = TangoService_connect(this, tango_config_);
    if(err != TANGO_SUCCESS)
    {
        LOGE("Failed to connect to the Tango service with error code: %d", err);
        return false;
    }

    return true;
}

void TangoMain::TangoDisconnect()
{
    TangoConfig_free(tango_config_);
    tango_config_ = nullptr;
    TangoService_disconnect();
}


void TangoMain::setConfigBool(const char *key, bool value)
{
    int ret = TangoConfig_setBool(tango_config_, key, value);
    if(ret != TANGO_SUCCESS)
    {
        LOGE("%s() failed with error code: %d", key, ret);
    }
}


bool TangoMain::getConfigBool(const char *key)
{
    bool value = false;
    int ret = TangoConfig_getBool(tango_config_, key, &value);

    if (ret != TANGO_SUCCESS)
    {
        LOGE("PointCloudApp: %s() failed with error" "code: %d", key, ret);
    }

    return value;
}







