// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "InfiniTAMApp.h"
#include "JavaMethods.h"

#include <unistd.h>
#include <jni.h>

extern "C" {

JNIEXPORT void JNICALL Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMRenderer_InitGL(JNIEnv *env, jobject thiz)
{
	(InfiniTAMApp::Instance())->InitGL();
}

JNIEXPORT void JNICALL Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMRenderer_ResizeGL(JNIEnv *env, jobject thiz, int x, int y)
{
	(InfiniTAMApp::Instance())->ResizeGL(x,y);
}

JNIEXPORT void JNICALL Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMRenderer_RenderGL(JNIEnv *env, jobject thiz)
{
	(InfiniTAMApp::Instance())->RenderGL();
}

JNIEXPORT void JNICALL Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMApplication_InitializeNativeApp(JNIEnv *env, jobject thiz, jstring java_libdir)
{
	const char *native_libdir = env->GetStringUTFChars(/*env,*/ java_libdir, NULL);
	chdir(native_libdir);
	InfiniTAMApp::Instance();
	env->ReleaseStringUTFChars(/*env,*/ java_libdir, native_libdir);
	JavaMethods::GetInstance().Initialize(env);
}

JNIEXPORT void JNICALL Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMProcessor_StartProcessing(
		JNIEnv *env, jobject thiz,
		int useLiveCamera, float voxelSize, int icp_quality)
{
	(InfiniTAMApp::Instance())->StartProcessing(useLiveCamera, voxelSize, icp_quality);
}

JNIEXPORT int JNICALL Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMProcessor_ProcessFrame(JNIEnv *env, jobject thiz)
{
	return (InfiniTAMApp::Instance())->ProcessFrame()?1:0;
}

JNIEXPORT void JNICALL Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMProcessor_ResetScene(JNIEnv *env, jobject thiz)
{
	(InfiniTAMApp::Instance())->ResetScene();
}

JNIEXPORT void JNICALL Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMProcessor_SetIntegration(JNIEnv *env, jobject thiz, jboolean on)
{
	(InfiniTAMApp::Instance())->SetIntegration(on);
}

JNIEXPORT void JNICALL Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMProcessor_SetMainProcessing(JNIEnv *env, jobject thiz, jboolean on)
{
	(InfiniTAMApp::Instance())->SetMainProcessing(on);
}

JNIEXPORT void JNICALL Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMProcessor_SaveScene(JNIEnv *env, jobject thiz, jboolean mesh)
{
	(InfiniTAMApp::Instance())->SaveScene(mesh);
}

JNIEXPORT void JNICALL Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMProcessor_Stop(JNIEnv *env, jobject thiz)
{
	(InfiniTAMApp::Instance())->Stop();
}

JNIEXPORT void JNICALL Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMProcessor_onTouchEvent(
		JNIEnv*, jobject, int touch_count, int event, float x0, float y0, float x1, float y1)
{
	InfiniTAMApp::Instance()->GetTouchHandler().onTouchEvent(touch_count, event, x0, y0, x1, y1);
}

JNIEXPORT void JNICALL Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMProcessor_ResetView(
		JNIEnv*, jobject)
{
	(InfiniTAMApp::Instance())->ResetView();
}

JNIEXPORT void JNICALL Java_uk_ac_ox_robots_InfiniTAM_InfiniTAMProcessor_SetScreenSize(
		JNIEnv*, jobject, float x, float y)
{
	InfiniTAMApp::Instance()->GetTouchHandler().SetScreenSize(x, y);
}


}

