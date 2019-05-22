
#ifndef TANGO_MAIN_H_
#define TANGO_MAIN_H_


#include <jni.h>

#include "logging.h"

//#include "ImageDataBuffer.h"
//#include "DataBuffer.h"
//#include "DataWriter.h"
//#include "TangoImages.h"

typedef void* TangoConfig;


class TangoMain
{
public:
    TangoMain();
    ~TangoMain();



    // Check that the installed version of the Tango API is up to date.
    //
    // @return returns true if the application version is compatible with the
    //         Tango Core version.
    bool CheckTangoVersion(JNIEnv* env, jobject caller_activity,
                           int min_tango_version);

    // Called when Tango Service is connected successfully.
    bool OnTangoServiceConnected(JNIEnv* env, jobject binder);

    // Setup the configuration file for the Tango Service.
    int TangoSetupConfig();

    // Connect the onPoseAvailable callback.
    int TangoConnectCallbacks();

    // Connect to Tango Service.
    // This function will start the Tango Service pipeline, in this case, it will
    // start Motion Tracking and Depth Sensing callbacks.
    bool TangoConnect();

    // Disconnect from Tango Service, release all the resources that the app is
    // holding from the Tango Service.
    void TangoDisconnect();


    void setConfigBool(const char* key, bool value);
    bool getConfigBool(const char* key);



private:


    TangoConfig tango_config_;

};

#endif  // TANGO_MAIN_H_
