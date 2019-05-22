// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

package uk.ac.ox.robots.InfiniTAM;

import android.app.Activity;
import android.app.ActivityManager;
import android.app.Dialog;
import android.content.ComponentName;
import android.content.ServiceConnection;
import android.graphics.Point;
import android.os.Bundle;
import android.content.Context;
import android.opengl.GLSurfaceView;
import android.os.Handler;
import android.os.IBinder;
import android.util.AttributeSet;
import android.util.Log;
import android.view.Display;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;


import com.projecttango.examples.cpp.util.TangoInitializationHelper;





public class InfiniTAMMainScreen extends Activity
{
	private InfiniTAMView view;

	private Thread processingThread;
	private Thread viewProcessingThread;
	private InfiniTAMProcessor processor;
	private InfiniTAMViewProcessor viewProcessor;
	private IMUReceiver imuReceiver;

	private static final int  MIN_TANGO_CORE_VERSION = 9377;

	protected static InfiniTAMMainScreen mCurrentActivity;

	private Point mScreenSize = new Point();



	// Project Tango Service connection.
	ServiceConnection mTangoServiceConnection = new ServiceConnection() {
		public void onServiceConnected(ComponentName name, IBinder service) {
			// Synchronization around MainActivity object is to avoid
			// Project Tango disconnect in the middle of the connecting operation.
			synchronized (InfiniTAMMainScreen.this) {
				InfiniTAMApplication.onTangoServiceConnected(service);

				// Setup the configuration for the TangoService.
				InfiniTAMApplication.setupConfig();

				// Connect the onPoseAvailable callback.
				InfiniTAMApplication.connectCallbacks();

				// Connect to Tango Service (returns true on success).
				// Starts Motion Tracking and Area Learning.
				if (!InfiniTAMApplication.connect()) {
					runOnUiThread(new Runnable() {
						@Override
						public void run() {
							// End the activity and let the user know something went wrong.
							Toast.makeText(InfiniTAMMainScreen.this, "Connect Tango Service Error",
									Toast.LENGTH_LONG).show();
							finish();
							return;
						}
					});
				}

				// At this point we could call Project Tango functions.
				// For example:
				// TangoService_connect(config);
			}
		}
		public void onServiceDisconnected(ComponentName name) {
			// Handle this if you need to gracefully shut down/retry in the event
			// that Project Tango itself crashes/gets upgraded while running.
		}
	};




	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState)
	{
		super.onCreate(savedInstanceState);

		mCurrentActivity = this;

		// Check if the Tango Core is out of date.
		if (!InfiniTAMApplication.checkTangoVersion(this, MIN_TANGO_CORE_VERSION)) {
			Toast.makeText(this, "Tango Core out of date, please update in Play Store",
					Toast.LENGTH_LONG).show();
			finish();
			return;
		}


		processor = new InfiniTAMProcessor();
		viewProcessor = new InfiniTAMViewProcessor();


//		setContentView(view);
		setContentView(R.layout.mainscreen);

		// Setup buttons and menus
		setupUI();


		// Setup display parameters
		Display display = getWindowManager().getDefaultDisplay();
		display.getSize(mScreenSize);
	}

	@Override
	protected void onStart() {
		super.onStart();

		view = (InfiniTAMView)findViewById(R.id.infinitamView);
		//view = new InfiniTAMView(this);

//		InfiniTAMApplication.setupConfig();

		showOptions();
	}

	@Override
	protected void onResume()
	{
		super.onResume();
		view.onResume();
//		processor.attachVisualisation(view);
		viewProcessor.attachVisualisation(view);
//		processor.resume();

		TangoInitializationHelper.bindTangoService(this, mTangoServiceConnection);

		processingThread = new Thread(processor);
		processingThread.start();

		viewProcessingThread = new Thread(viewProcessor);
		viewProcessingThread.start();
	}

	@Override
	protected void onPause() {
		viewProcessor.detachVisualisation(view);
//		processor.detachVisualisation(view);
		super.onPause();
		view.onPause();
//		processor.pause();

		InfiniTAMApplication.disconnect();
		synchronized (this) {
			unbindService(mTangoServiceConnection);
		}
	}

	@Override
	protected void onStop() {
		super.onStop();
		viewProcessor.requestStop();
		processor.requestStop();
		try {
			viewProcessingThread.join();
			processingThread.join();

		}catch (Exception e)
		{
			Log.e("InfiniTAM", e.toString());
		}
	}

	@Override
	protected void onDestroy()
	{
		super.onDestroy();
	}

	@Override
	public boolean onTouchEvent(MotionEvent event) {
		// Pass the touch event to the native layer for camera control.
		// Single touch to rotate the camera around the device.
		// Two fingers to zoom in and out.
		int pointCount = event.getPointerCount();
		if (pointCount == 1) {
			float normalizedX = event.getX(0) / mScreenSize.x;
			float normalizedY = event.getY(0) / mScreenSize.y;
			InfiniTAMProcessor.onTouchEvent(1, event.getActionMasked(),
					normalizedX, normalizedY, 0.0f, 0.0f);

		}
		if (pointCount == 2) {
			if (event.getActionMasked() == MotionEvent.ACTION_POINTER_UP) {
				int index = event.getActionIndex() == 0 ? 1 : 0;
				float normalizedX = event.getX(index) / mScreenSize.x;
				float normalizedY = event.getY(index) / mScreenSize.y;
				InfiniTAMProcessor.onTouchEvent(1, MotionEvent.ACTION_DOWN,
						normalizedX, normalizedY, 0.0f, 0.0f);
			} else {
				float normalizedX0 = event.getX(0) / mScreenSize.x;
				float normalizedY0 = event.getY(0) / mScreenSize.y;
				float normalizedX1 = event.getX(1) / mScreenSize.x;
				float normalizedY1 = event.getY(1) / mScreenSize.y;
				InfiniTAMProcessor.onTouchEvent(2, event.getActionMasked(),
						normalizedX0, normalizedY0,
						normalizedX1, normalizedY1);
			}
		}
		return true;
	}


	protected void setupUI()
	{
		// Reset scene
		final ImageButton resetButton = (ImageButton)findViewById(R.id.button_reset);
		resetButton.setOnClickListener(new View.OnClickListener() {
			public void onClick(View view) {
				resetButton.setVisibility(View.INVISIBLE);

				Thread thread = new Thread(){
					public void run() {
						InfiniTAMProcessor.ResetScene();
						runOnUiThread(new Runnable() {
							public void run() {
								resetButton.setVisibility(View.VISIBLE);
							}
						});

					}
				};
				thread.start();

			}
		});

		// Fusion ON/OFF
		final ToggleButton fusionButton = (ToggleButton)findViewById(R.id.toggleButton_fusion);
		fusionButton.setOnClickListener(new View.OnClickListener() {
			public void onClick(View view) {
				InfiniTAMProcessor.SetIntegration(fusionButton.isChecked());
			}
		});

		// Save scene
		final ImageButton saveButton = (ImageButton)findViewById(R.id.imageButton_save);
		saveButton.setOnClickListener(new View.OnClickListener() {
			public void onClick(View view) {
				save(saveButton, false);
			}
		});

		final ImageButton saveButton_mesh = (ImageButton)findViewById(R.id.imageButton_saveMesh);
		saveButton_mesh.setOnClickListener(new View.OnClickListener() {
			public void onClick(View view) {
				save(saveButton_mesh, true);
			}
		});

		final ImageButton resetViewButton = (ImageButton)findViewById(R.id.imageButton_resetView);
		resetViewButton.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View view) {
				InfiniTAMProcessor.ResetView();
			}
		});

		final ImageButton stopButton = (ImageButton)findViewById(R.id.imageButton_stop);
		stopButton.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View view) {
				stopButton.setVisibility(View.INVISIBLE);
				InfiniTAMProcessor.Stop();
			}
		});

	}

	protected void save(final ImageButton saveButton, final boolean mesh) {
		saveButton.setVisibility(View.INVISIBLE);

		Toast.makeText(InfiniTAMMainScreen.this, "Saving data", Toast.LENGTH_LONG).show();

		Thread thread = new Thread() {
			public void run() {
				InfiniTAMProcessor.SetMainProcessing(false);
				InfiniTAMProcessor.SaveScene(mesh);
				InfiniTAMProcessor.SetMainProcessing(true);
				runOnUiThread(new Runnable() {
					public void run() {
						saveButton.setVisibility(View.VISIBLE);
					}
				});

			}
		};
		thread.start();
	}

	protected void showOptions()
	{
		final Dialog dialog = new Dialog(this);
		dialog.setContentView(R.layout.options_dialog);
		dialog.setTitle("Options");

		final EditText voxelSizeInput = (EditText) dialog.findViewById(R.id.editText_optionsVoxelSize);
		final SeekBar  icpSeekBar     = (SeekBar)  dialog.findViewById(R.id.seekBar_icpQuality);
		icpSeekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
			public void onProgressChanged(SeekBar seekBar, int i, boolean b) {
				TextView icpText = (TextView) dialog.findViewById(R.id.textView_icpQualityNum);
				icpText.setText(String.valueOf(i));
			}
			public void onStartTrackingTouch(SeekBar seekBar) {}
			public void onStopTrackingTouch(SeekBar seekBar) {}
		});

		Button dialogButton = (Button) dialog.findViewById(R.id.button_optionsOk);

		dialogButton.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {

				float voxelSize = Float.parseFloat(voxelSizeInput.getText().toString()) / 1000.0f;
				int icpQuality  = icpSeekBar.getProgress();

				dialog.dismiss();

				processor.setSettings(voxelSize, icpQuality, mScreenSize.x, mScreenSize.y);

				onResume();
			}
		});

		dialog.show();
	}


	public static void setMemoryUsage(float freeMem, float totalMem){

		final String text_mem = "Free mem: " + String.format("%.0f", freeMem / 1000000.0) + "MB";
//				+ " / " + String.format("%.0f", totalMem / 1000000.0) + "MB";

		mCurrentActivity.runOnUiThread(new Runnable() {
			@Override
			public void run() {
				TextView view_mem = (TextView) mCurrentActivity.findViewById(R.id.textView_memory);
				view_mem.setText(text_mem);
			}
		});
	}

	public static void setAlgorithmFramerate(float hz){

		final String text_mem = "" + String.format("%.1f", hz) + "Hz";

		mCurrentActivity.runOnUiThread(new Runnable() {
			@Override
			public void run() {
				TextView view_mem = (TextView) mCurrentActivity.findViewById(R.id.textView_memory);
				view_mem.setText(text_mem);
			}
		});
	}

} // class

class InfiniTAMView extends GLSurfaceView
{
	private final InfiniTAMRenderer mRenderer = new InfiniTAMRenderer();

	public InfiniTAMView(Context context) {
		super(context);
		init();
	}

	public InfiniTAMView(Context context, AttributeSet attrs) {
		super(context, attrs);
		init();
	}

	private void init() {
		// Create an OpenGL ES 2.0 context.
		setEGLContextClientVersion(1);

		// Set the Renderer for drawing on the GLSurfaceView
		setRenderer(mRenderer);

		// Render the view only when there is a change in the drawing data
		setRenderMode(GLSurfaceView.RENDERMODE_WHEN_DIRTY);
	}

}

