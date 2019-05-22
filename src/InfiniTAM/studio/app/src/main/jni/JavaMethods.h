//
// Created by qkgautier on 8/4/16.
//

#pragma once

#include <jni.h>

#include "logging.h"

class JavaMethods
{
public:

    static JavaMethods& GetInstance()
    {
        static JavaMethods javaMethods_;
        return javaMethods_;
    }

    bool Initialize(JNIEnv* env);

    void setMemoryUsage(float freeMem, float totalMem);
    void setAlgorithmFramerate(float hz);

private:
    JavaMethods(){}
    ~JavaMethods(){}

private:
    JavaVM* javaVm_;
    jclass mainscreen_class_;


    struct JVMAttach
    {
        JVMAttach(JavaVM* vm, const char* name = nullptr):
                jvm(vm),
                g_env(nullptr),
                attached(false)
        {
            if(!vm){ return; }

            int getEnvStat = vm->GetEnv((void **) &g_env, JNI_VERSION_1_6);

            if (getEnvStat == JNI_EDETACHED)
            {
                JavaVMAttachArgs args;
                args.version = JNI_VERSION_1_6;
                if(name)
                    args.name = name;
                else
                    args.name = "native_thread";
                args.group = nullptr;


                if (vm->AttachCurrentThread(&g_env, &args) != 0)
                {
                    LOGE("Failed to attach thread to Java VM");
                }
                attached = true;
            }
            else if (getEnvStat == JNI_EVERSION)
            {
                LOGE("GetEnv: version not supported");
            }
            else if (getEnvStat == JNI_OK)
            {
            }
        }

        ~JVMAttach(){ if(attached){ jvm->DetachCurrentThread(); } }

        JavaVM* jvm;
        JNIEnv *g_env;
        bool attached;
    };
};

