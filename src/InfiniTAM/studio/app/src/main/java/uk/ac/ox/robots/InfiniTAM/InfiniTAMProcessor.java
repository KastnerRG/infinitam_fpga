// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

package uk.ac.ox.robots.InfiniTAM;

import android.opengl.GLSurfaceView;
import android.opengl.GLSurfaceView.Renderer;
import javax.microedition.khronos.opengles.GL10;
import javax.microedition.khronos.egl.EGLConfig;
import android.util.Log;

class InfiniTAMRenderer implements GLSurfaceView.Renderer
{
	public void onDrawFrame(GL10 gl) 
	{
		RenderGL();
	}

	public void onSurfaceChanged(GL10 gl, int width, int height) 
	{		
		ResizeGL(width, height);
	}

	public void onSurfaceCreated(GL10 gl, EGLConfig config) 
	{
		InitGL();
	}

	public static native void InitGL();
	public static native void ResizeGL(int newWidth, int newHeight);
	public static native void RenderGL();
}

class InfiniTAMProcessor implements Runnable
{
	private InfiniTAMView view;
	boolean stopRequested;

	private boolean settingsSet;
	private float settingVoxelSize;
	private int settingIcpQuality;
	private float screenX, screenY;

	InfiniTAMProcessor()
	{
		view = null;
		stopRequested = false;
		settingsSet = false;
	}

	public void attachVisualisation(InfiniTAMView _view)
	{
		view = _view;
	}

	public void detachVisualisation(InfiniTAMView _view)
	{
		view = null;
	}

	public void requestStop()
	{
		Stop();
		stopRequested = true;
	}

	public void setSettings(float voxelSize, int icpQuality, float screen_x, float screen_y){
		settingVoxelSize = voxelSize;
		settingIcpQuality = icpQuality;
		screenX = screen_x; screenY = screen_y;
		settingsSet = true;
	}

	public void run()
	{
		if(!settingsSet){ return; }

		int usb_fd = InfiniTAMApplication.getApplication().waitForUSBPermission();
		stopRequested = false;

		SetScreenSize(screenX, screenY);
		StartProcessing((usb_fd >= 0)?1:0, settingVoxelSize, settingIcpQuality);

		while (true) {
			int ret = ProcessFrame();
			if (view != null) view.requestRender();
			if (ret == 0) break;
			if (stopRequested) break;
		}
	}

	public static native void StartProcessing(int useLiveCamera, float voxelSize, int icp_quality);
	public static native int ProcessFrame();
	public static native void ResetScene();
	public static native void SetIntegration(boolean on);
	public static native void SetMainProcessing(boolean on);
	public static native void SaveScene(boolean mesh);
	public static native void SetScreenSize(float x, float y);
	public static native void onTouchEvent(int touch_count, int event, float x0, float y0, float x1, float y1);
	public static native void ResetView();
	public static native void Stop();
}


class InfiniTAMViewProcessor implements Runnable {
	private InfiniTAMView view;

	boolean stopRequested;

	InfiniTAMViewProcessor() {
		view = null;
		stopRequested = false;
	}

	public void attachVisualisation(InfiniTAMView _view) {
		view = _view;
	}

	public void detachVisualisation(InfiniTAMView _view) {
		view = null;
	}

	public void requestStop() {
		stopRequested = true;
	}


	public void run() {

		stopRequested = false;

		while (true) {
			if (view != null) view.requestRender();
			if (stopRequested) break;
		}
	}
}


