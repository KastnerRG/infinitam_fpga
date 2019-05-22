//
// Created by qkgautier on 8/4/16.
//

#include "JavaMethods.h"


bool JavaMethods::Initialize(JNIEnv * env)
{
    jint j_status = env->GetJavaVM(&javaVm_);
    if(j_status != JNI_OK){ return false; }

    jclass localClass = env->FindClass("uk/ac/ox/robots/InfiniTAM/InfiniTAMMainScreen");
    mainscreen_class_ = (jclass) env->NewGlobalRef(localClass);

    return true;
}


void JavaMethods::setMemoryUsage(float freeMem, float totalMem)
{
    JVMAttach attached(javaVm_);

    if(!attached.g_env)
    {
        LOGE("Unable to attach thread to JVM");
        return;
    }
    jmethodID java_fct = attached.g_env->GetStaticMethodID(mainscreen_class_, "setMemoryUsage", "(FF)V");

    attached.g_env->CallStaticVoidMethod(mainscreen_class_, java_fct, freeMem, totalMem);
}


void JavaMethods::setAlgorithmFramerate(float hz)
{
    JVMAttach attached(javaVm_);

    if(!attached.g_env)
    {
        LOGE("Unable to attach thread to JVM");
        return;
    }
    jmethodID java_fct = attached.g_env->GetStaticMethodID(mainscreen_class_, "setAlgorithmFramerate", "(F)V");

    attached.g_env->CallStaticVoidMethod(mainscreen_class_, java_fct, hz);
}



