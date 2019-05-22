//
// Created by qkgautier on 10/26/15.
//

#ifndef JNI_TANGO_H
#define JNI_TANGO_H

#include <jni.h>

#include "tangoMain.h"

#ifdef __cplusplus
extern "C" {
#endif


static TangoMain tango_main;



JNIEXPORT jboolean JNICALL
Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMApplication_checkTangoVersion(
        JNIEnv* env, jobject, jobject activity, jint min_tango_version) {
    return tango_main.CheckTangoVersion(env, activity, min_tango_version);
}

JNIEXPORT jboolean JNICALL
Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMApplication_onTangoServiceConnected(
        JNIEnv* env, jobject /*caller_object*/, jobject binder) {
    return tango_main.OnTangoServiceConnected(env, binder);
}

JNIEXPORT jint JNICALL
Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMApplication_setupConfig(
        JNIEnv*, jobject) {
    return tango_main.TangoSetupConfig();
}

JNIEXPORT jboolean JNICALL
Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMApplication_connect(JNIEnv*, jobject) {
    return tango_main.TangoConnect();
}

JNIEXPORT jint JNICALL
Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMApplication_connectCallbacks(
        JNIEnv*, jobject) {
    return tango_main.TangoConnectCallbacks();
}

JNIEXPORT void JNICALL
Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMApplication_disconnect(
        JNIEnv*, jobject) {
    tango_main.TangoDisconnect();
}



//JNIEXPORT void JNICALL
//Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMApplication_toggleRecording(JNIEnv*, jobject)
//{
//    tango_main.data_writer_.toggleRecording();
//}
//
//JNIEXPORT void JNICALL
//Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMApplication_startRecording(JNIEnv*, jobject)
//{
//    tango_main.data_writer_.startRecording();
//}
//
//JNIEXPORT void JNICALL
//Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMApplication_stopRecording(JNIEnv*, jobject)
//{
//    tango_main.data_writer_.stopRecording();
//}
//
//JNIEXPORT bool JNICALL
//Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMApplication_isRecording(JNIEnv*, jobject)
//{
//    return tango_main.data_writer_.isRecording();
//}
//
//JNIEXPORT void JNICALL
//Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMApplication_setRecordingName(JNIEnv* env, jobject, jstring name)
//{
//    const char* nativetext = env->GetStringUTFChars(name, nullptr);
//    tango_main.data_writer_.setRecordingName(nativetext);
//    env->ReleaseStringUTFChars(name, nativetext);
//}
//
//JNIEXPORT void JNICALL
//Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMApplication_setRecordAll(JNIEnv* env, jobject, jboolean saveAll)
//{
//    int saveType = DataWriter::SAVE_ALL;
//    if(!saveAll)
//    {
//        saveType = DataWriter::SAVE_POSE | DataWriter::SAVE_DEPTH | DataWriter::SAVE_COLOR_FOR_DEPTH;
//    }
//    tango_main.data_writer_.setSaveType(saveType);
//}




#ifdef __cplusplus
}
#endif

#endif //JNI_TANGO_H
