package cpc.robotics.vision;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.os.Build;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.ViewGroup;

import org.opencv.BuildConfig;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.Closeable;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

import static android.view.View.VISIBLE;

/**
 * Based on OpenCV JavaCameraView using the V1 Camera API. This version works without needing to
 * be associated with a SurfaceView. Conversely, a SurfaceView can be provided when preview
 * rendering is required.
 */
public class VisionHelper implements Closeable {

    private static final int MAGIC_TEXTURE_ID = 10;
    private static final String TAG = "VisionHelper";

    private byte mBuffer[];
    private Mat[] mFrameChain;
    private int mChainIdx = 0;
    private Thread mThread;
    private volatile boolean stopThread = false;
    private static final int STOPPED = 0;
    private static final int STARTED = 1;

    private volatile boolean mCameraFrameReady = false;
    private boolean mSurfaceExist = false;

    private int mState = STOPPED;
    private Bitmap mCacheBitmap;
    private final Object mStateSyncObject = new Object();
    private final Object mFrameSyncObject = new Object();

    protected CopyOnWriteArrayList<VisionExtension> extensions = new CopyOnWriteArrayList<>();
    protected int mPreviewFormat = ImageFormat.NV21;
    protected int mWidth;
    protected int mHeight;
    protected int mCameraIndex; // e.. = Camera.CameraInfo.CAMERA_FACING_BACK;
    protected Context mContext;
    protected boolean mEnabled = false; // need to explicitly enable when ready

    protected Camera mCamera = null;
    protected CameraViewFrameImpl[] mCameraFrame;
    private SurfaceTexture mSurfaceTexture;
    private SurfaceView mView = null;

    /**
     * This class interface is abstract representation of single frame from camera for onCameraFrame callback
     * Attention: Do not use objects, that represents this interface out of onCameraFrame callback!
     */
    public interface CameraViewFrame {

        /**
         * This method returns RGBA Mat with frame
         */
        public Mat rgba();
    };

    /**
     * Actual (hidden) implementation of CameraViewFrame
     */
    private class CameraViewFrameImpl implements CameraViewFrame {

        @Override
        public Mat rgba() {
            if (mPreviewFormat == ImageFormat.NV21)
                Imgproc.cvtColor(mYuvFrameData, mRgba, Imgproc.COLOR_YUV2RGBA_NV21, 4);
            else if (mPreviewFormat == ImageFormat.YV12)
                Imgproc.cvtColor(mYuvFrameData, mRgba, Imgproc.COLOR_YUV2RGB_I420, 4);  // COLOR_YUV2RGBA_YV12 produces inverted colors
            else
                throw new IllegalArgumentException("Preview Format can be NV21 or YV12");

            return mRgba;
        }

        public CameraViewFrameImpl(Mat Yuv420sp, int width, int height) {
            super();
            mWidth = width;
            mHeight = height;
            mYuvFrameData = Yuv420sp;
            mRgba = new Mat();
        }

        public void release() {
            mRgba.release();
        }

        private Mat mYuvFrameData;
        private Mat mRgba;
    };

    /**
     * Worker thread feeds frames from camera to callback.
     */
    private class CameraWorker implements Runnable {

        @Override
        public void run() {
            while(!stopThread) {
                boolean hasFrame = false;
                synchronized (mFrameSyncObject) {
                    try {
                        while (!mCameraFrameReady) {
                            mFrameSyncObject.wait();
                        }
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        stopThread = true;
                    }
                    if (mCameraFrameReady)
                    {
                        mChainIdx = 1 - mChainIdx;
                        mCameraFrameReady = false;
                        hasFrame = true;
                    }
                }

                if (!stopThread && hasFrame) {
                    if (!mFrameChain[1 - mChainIdx].empty())
                        deliverAndDrawFrame(mCameraFrame[1 - mChainIdx]);
                }
            }
            Log.d(TAG, "Finish processing thread");
        }
    }

    private final Camera.PreviewCallback previewCallbacks = new Camera.PreviewCallback() {

        /**
         * Camera callback for the camera 'preview' with is actually to capture a buffer to allow
         * OpenCV processing.
         * @param frame Frame data
         * @param camera Camera frame was received from
         */
        @Override
        public void onPreviewFrame(byte[] frame, Camera camera) {
            synchronized (mFrameSyncObject) {
                mFrameChain[mChainIdx].put(0, 0, frame);
                mCameraFrameReady = true;
                mFrameSyncObject.notify();
            }
            if (mCamera != null)
                mCamera.addCallbackBuffer(mBuffer);
        }
    };

    private final SurfaceHolder.Callback surfaceHolderCallbacks = new SurfaceHolder.Callback() {

        @Override
        public void surfaceCreated(SurfaceHolder surfaceHolder) {
            // deferred to surfaceChanged
        }

        @Override
        public void surfaceChanged(SurfaceHolder surfaceHolder, int i, int i1, int i2) {
            Log.d(TAG, "surfaceChanged event");
            synchronized(mStateSyncObject) {
                if (!mSurfaceExist) {
                    mSurfaceExist = true;
                    checkCurrentState();
                } else {
                    /** Surface changed. We need to stop camera and restart with new parameters */
                /* Pretend that old surface has been destroyed */
                    mSurfaceExist = false;
                    checkCurrentState();
                /* Now use new surface. Say we have it now */
                    mSurfaceExist = true;
                    checkCurrentState();
                }
            }
        }

        @Override
        public void surfaceDestroyed(SurfaceHolder surfaceHolder) {
            synchronized(mStateSyncObject) {
                mSurfaceExist = false;
                checkCurrentState();
            }
        }
    };

    /**
     * Construct a VisionHelper bound to a Context.
     * @param context Context, should be an Activity
     * @param cameraId Specify which camera, e.g. CAMERA_FACING_BACK
     * @param width Maximum width of camera capture
     * @param height Maximum height of camera capture
     */
    public VisionHelper(Context context, int cameraId, int width, int height) {
        mContext = context;
        mCameraIndex = cameraId;
        mWidth = width;
        mHeight = height;
        OpenCVLoader.initDebug(); // Ensure OpenCV is loaded
    }

    public Context getContext() {
        return mContext;
    }

    public Activity getActivity() {
        return (Activity)mContext;
    }

    /**
     * Allows try(VisionHelper var = new VisionHelper(...)) { ... } block
     * Ensures resources are released, and camera is available for re-use.
     */
    @Override
    public void close() {
        disable();
        for(VisionExtension extension : extensions) {
            extension.close();
        }
        extensions.clear();
    }

    /**
     * Add extensions in order of execution. Extensions are not
     * @param extensions
     */
    public void addExtensions(VisionExtension ... extensions) {
        for (VisionExtension extension : extensions) {
            if (this.extensions.addIfAbsent(extension)) {
                extension.setVisionHelper(this);
                extension.setActive(mCamera != null);
            }
        }
        // Defer onCreate so that extensions can be added in groups
        for (VisionExtension extension : extensions) {
            extension.onCreate();
        }
    }

    /**
     * Add a single extension and return it to caller.
     * @param extension extension to add
     * @return extension passed in
     */
    public VisionExtension addExtension(VisionExtension extension) {
        addExtensions(extension);
        return extension;
    }

    /**
     * Get an extension by class - find first extension that matches the specified class, or returns
     * null.
     * @param cls VisionExtension to retrieve
     * @return VisionExtension instance or null.
     */
    public <E extends VisionExtension> E getExtension(Class<E> cls) {
        for (VisionExtension extension : this.extensions) {
            if (cls.isInstance(extension)) {
                return (E)extension;
            }
        }
        return null;
    }

    /**
     * Get or create extension by class.
     * @param cls VisionExtension to retrieve or create
     * @return existing or new instance.
     */
    public VisionExtension addExtension(Class<? extends VisionExtension> cls) {
        VisionExtension extension = getExtension(cls);
        if (extension == null) {
            try {
                extension = cls.newInstance();
            } catch (InstantiationException|IllegalAccessException e) {
                throw new RuntimeException(e.getMessage(), e);
            }
            return addExtension(extension);
        } else {
            return extension;
        }
    }

    /**
     * This method is invoked when camera preview has started. After this method is invoked
     * the frames will start to be delivered to client via the onCameraFrame() callback.
     * @param width -  the width of the frames that will be delivered
     * @param height - the height of the frames that will be delivered
     */
    public void onCameraViewStarted(int width, int height) {
    }

    /**
     * This method is invoked when camera preview has been stopped for some reason.
     * No frames will be delivered via onCameraFrame() callback after this method is called.
     */
    public void onCameraViewStopped() {
    }

    /**
     * This method is invoked when delivery of the frame needs to be done.
     * The returned values - is a modified frame which needs to be displayed on the screen.
     */
    public Mat onCameraFrame(CameraViewFrame inputFrame) {
        Mat img = inputFrame.rgba();
        for(VisionExtension extension : extensions) {
            img = extension.frame(img);
        }
        return img;
    }

    /**
     * Associate a surface view with this vision helper for preview
     * @param view View to associate
     */
    public void setView(SurfaceView view) {
        mView = view;
        view.getHolder().addCallback(this.surfaceHolderCallbacks);
        checkCurrentState();
    }

    /**
     * Retrieve view associated with this vision helper
     * @return associated view
     */
    public SurfaceView getView() {
        return mView;
    }

    /**
     * Public exposed method to indicate camera operation is desired. It's not actually enabled
     * unless other conditions are met when a view is attached.
     */
    public final void enable() {
        synchronized(mStateSyncObject) {
            mEnabled = true;
            checkCurrentState();
        }
    }

    /**
     * Disable operation of camera.
     */
    public final void disable() {
        synchronized(mStateSyncObject) {
            mEnabled = false;
            checkCurrentState();
        }
    }

    /**
     * Prior to first enable, desired width. After camera is enabled, actual width.
     * @return width
     */
    public int getWidth()
    {
        return mWidth;
    }

    /**
     * Prior to first enable, desired height. After camera is enabled, actual height.
     * @return height
     */
    public int getHeight()
    {
        return mHeight;
    }

    /**
     * Underlying V1 camera if camera is enabled, else null.
     * @return camera
     */
    public Camera getCamera() { return mCamera; }

    /**
     * Called when mSyncObject lock is held
     */
    private final void checkCurrentState() {
        int targetState;

        if (mEnabled && (mView == null || (mSurfaceExist && mView.getVisibility() == VISIBLE))) {
            Log.d(TAG, "target=STARTED");
            targetState = STARTED;
        } else {
            Log.d(TAG, "target=STOPPED");
            targetState = STOPPED;
        }

        if (targetState != mState) {
            /* The state change detected. Need to exit the current state and enter target state */
            processExitState(mState);
            mState = targetState;
            processEnterState(mState);
        }
    }

    private final void processEnterState(int state) {
        Log.d(TAG, "call processEnterState: " + state);
        switch(state) {
            case STARTED:
                onEnterStartedState();
                onCameraViewStarted(mWidth, mHeight);
                break;
            case STOPPED:
                onEnterStoppedState();
                onCameraViewStopped();
                break;
        };
    }

    private final void processExitState(int state) {
        Log.d(TAG, "call processExitState: " + state);
        switch(state) {
            case STARTED:
                onExitStartedState();
                break;
            case STOPPED:
                onExitStoppedState();
                break;
        };
    }

    private final void onEnterStoppedState() {
        /* nothing to do */
    }

    private final void onExitStoppedState() {
        /* nothing to do */
    }

    // NOTE: The order of bitmap constructor and camera connection is important for android 4.1.x
    // Bitmap must be constructed before surface
    private final void onEnterStartedState() {
        Log.d(TAG, "call onEnterStartedState");
        /* Connect camera */
        if (!connectCamera(getWidth(), getHeight())) {
            AlertDialog ad = new AlertDialog.Builder(mContext).create();
            ad.setCancelable(false); // This blocks the 'BACK' button
            ad.setMessage("It seems that you device does not support camera (or it is locked). Application will be closed.");
            ad.setButton(DialogInterface.BUTTON_NEUTRAL,  "OK", new DialogInterface.OnClickListener() {
                public void onClick(DialogInterface dialog, int which) {
                    dialog.dismiss();
                    ((Activity) mContext).finish();
                }
            });
            ad.show();

        }
    }

    private void onExitStartedState() {
        disconnectCamera();
        if (mCacheBitmap != null) {
            mCacheBitmap.recycle();
        }
    }

    /**
     * Private helper used to initialize the underlying camera stream
     * @param width
     * @param height
     * @return true if initialized
     */
    private boolean initializeCamera(int width, int height) {
        Log.d(TAG, "Initialize camera");
        boolean result = true;
        synchronized (this) {
            mCamera = null;
            Camera.CameraInfo cameraInfo = new Camera.CameraInfo();
            int localCameraIndex = mCameraIndex;
            for (int camIdx = 0; camIdx < Camera.getNumberOfCameras(); ++camIdx) {
                Camera.getCameraInfo( camIdx, cameraInfo );
                if (cameraInfo.facing == mCameraIndex) {
                    localCameraIndex = cameraInfo.facing;
                    break;
                }
            }
            Log.d(TAG, "Trying to open camera (" + Integer.valueOf(localCameraIndex) + ")");
            try {
                mCamera = Camera.open(localCameraIndex);
            } catch (RuntimeException e) {
                Log.e(TAG, "Camera #" + localCameraIndex + "failed to open: " + e.getLocalizedMessage());
            }

            if (mCamera == null)
                return false;

            /* Now set camera parameters */
            try {
                Camera.Parameters params = mCamera.getParameters();
                Log.d(TAG, "getSupportedPreviewSizes()");
                List<Camera.Size> sizes = params.getSupportedPreviewSizes();

                if (sizes != null) {
                    /* Select the size that fits surface considering maximum size allowed */
                    Size frameSize = calculateCameraFrameSize(sizes, width, height);

                    /* Image format NV21 causes issues in the Android emulators */
                    if (Build.FINGERPRINT.startsWith("generic")
                            || Build.FINGERPRINT.startsWith("unknown")
                            || Build.MODEL.contains("google_sdk")
                            || Build.MODEL.contains("Emulator")
                            || Build.MODEL.contains("Android SDK built for x86")
                            || Build.MANUFACTURER.contains("Genymotion")
                            || (Build.BRAND.startsWith("generic") && Build.DEVICE.startsWith("generic"))
                            || "google_sdk".equals(Build.PRODUCT)) {
                        params.setPreviewFormat(ImageFormat.YV12);  // "generic" or "android" = android emulator
                    } else {
                        params.setPreviewFormat(ImageFormat.NV21);
                    }
                    mPreviewFormat = params.getPreviewFormat();

                    Log.d(TAG, "Set preview size to " + Integer.valueOf((int)frameSize.width) + "x" + Integer.valueOf((int)frameSize.height));
                    params.setPreviewSize((int)frameSize.width, (int)frameSize.height);

                    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.ICE_CREAM_SANDWICH && !android.os.Build.MODEL.equals("GT-I9100")) {
                        params.setRecordingHint(true);
                    }

                    List<String> FocusModes = params.getSupportedFocusModes();
                    if (FocusModes != null && FocusModes.contains(Camera.Parameters.FOCUS_MODE_CONTINUOUS_VIDEO)) {
                        params.setFocusMode(Camera.Parameters.FOCUS_MODE_CONTINUOUS_VIDEO);
                    }

                    mCamera.setParameters(params);
                    params = mCamera.getParameters();

                    mWidth = params.getPreviewSize().width;
                    mHeight = params.getPreviewSize().height;

                    int size = mWidth * mHeight;
                    size  = size * ImageFormat.getBitsPerPixel(params.getPreviewFormat()) / 8;
                    mBuffer = new byte[size];

                    mCamera.addCallbackBuffer(mBuffer);
                    mCamera.setPreviewCallbackWithBuffer(this.previewCallbacks);

                    mFrameChain = new Mat[2];
                    mFrameChain[0] = new Mat(mHeight + (mHeight/2), mWidth, CvType.CV_8UC1);
                    mFrameChain[1] = new Mat(mHeight + (mHeight/2), mWidth, CvType.CV_8UC1);

                    allocateCache();

                    mCameraFrame = new CameraViewFrameImpl[2];
                    mCameraFrame[0] = new CameraViewFrameImpl(mFrameChain[0], mWidth, mHeight);
                    mCameraFrame[1] = new CameraViewFrameImpl(mFrameChain[1], mWidth, mHeight);

                    mSurfaceTexture = new SurfaceTexture(MAGIC_TEXTURE_ID);
                    mCamera.setPreviewTexture(mSurfaceTexture);

                    /* Finally we are ready to start the preview */
                    Log.d(TAG, "startPreview");
                    mCamera.startPreview();
                } else {
                    result = false;
                }
            } catch (Exception e) {
                result = false;
                e.printStackTrace();
            }
        }

        return result;
    }

    /**
     * Release the camera resources previously allocated.
     */
    private final void releaseCamera() {
        synchronized (this) {
            if (mCamera != null) {
                mCamera.stopPreview();
                mCamera.setPreviewCallback(null);

                mCamera.release();
            }
            mCamera = null;
            if (mFrameChain != null) {
                mFrameChain[0].release();
                mFrameChain[1].release();
            }
            if (mCameraFrame != null) {
                mCameraFrame[0].release();
                mCameraFrame[1].release();
            }
        }
    }

    protected void allocateCache() {
        // NOTE: On Android 4.1.x the function must be called before SurfaceTexture constructor!
        mCacheBitmap = Bitmap.createBitmap(mWidth, mHeight, Bitmap.Config.ARGB_8888);
    }

    /**
     * Connect camera and start feeding frames to callbacks.
     * @param width Maximum width
     * @param height Maximum height
     * @return true if success.
     */
    protected boolean connectCamera(int width, int height) {

        // Instantiate camera, create thread, enable extensions

        // initialize camera
        Log.d(TAG, "Connecting to camera");
        if (!initializeCamera(width, height)) {
            return false;
        }

        mCameraFrameReady = false;

        // Connect extensions prior to starting worker thread
        for(VisionExtension extension : extensions) {
            extension.setActive(true);
        }

        // start worker thread
        Log.d(TAG, "Starting processing thread");
        stopThread = false;
        mThread = new Thread(new CameraWorker());
        mThread.start();
        return true;
    }

    /**
     * Disconnect camera cleanly
     */
    protected void disconnectCamera() {
        // Stop extensions, stop thread, release camera
        Log.d(TAG, "Disconnecting from camera");
        for(VisionExtension extension : extensions) {
            extension.setActive(false);
        }
        try {
            if (mThread != null) {
                Log.d(TAG, "Interrupting camera worker");
                stopThread = true;
                mThread.interrupt();
            }
            Log.d(TAG, "Notify thread");
            synchronized (mFrameSyncObject) {
                mFrameSyncObject.notify();
            }
            Log.d(TAG, "Waiting for thread");
            if (mThread != null)
                mThread.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } finally {
            mThread =  null;
        }

        /* Now release camera */
        releaseCamera();

        mCameraFrameReady = false;
    }

    /**
     * This helper method can be called by subclasses to select camera preview size.
     * It goes over the list of the supported preview sizes and selects the maximum one which
     * fits both values set via setMaxFrameSize() and surface frame allocated for this view
     * @param supportedSizes
     * @param maxWidth
     * @param maxHeight
     * @return optimal frame size
     */
    protected Size calculateCameraFrameSize(List<Camera.Size> supportedSizes, int maxWidth, int maxHeight) {
        int calcWidth = 0;
        int calcHeight = 0;
        long calcArea = 0;

        for (Camera.Size size : supportedSizes) {
            int width = size.width;
            int height = size.height;
            long area = width*height;

            if (area > calcArea && width <= maxWidth && height <= maxHeight) {
                calcWidth = width;
                calcHeight = height;
                calcArea = area;
            }
        }

        return new Size(calcWidth, calcHeight);
    }

    /**
     * This method shall be called by the subclasses when they have valid
     * object and want it to be delivered to external client (via callback) and
     * then displayed on the screen.
     * @param frame - the current frame to be delivered
     */
    protected void deliverAndDrawFrame(CameraViewFrame frame) {
        Mat modified;

        modified = onCameraFrame(frame);
        if (mView instanceof SurfaceView) {
            SurfaceView view = (SurfaceView)mView;

            boolean bmpValid = true;
            if (modified != null) {
                try {
                    Utils.matToBitmap(modified, mCacheBitmap);
                } catch(Exception e) {
                    Log.e(TAG, "Mat type: " + modified);
                    Log.e(TAG, "Bitmap type: " + mCacheBitmap.getWidth() + "*" + mCacheBitmap.getHeight());
                    Log.e(TAG, "Utils.matToBitmap() throws an exception: " + e.getMessage());
                    bmpValid = false;
                }
            }

            if (bmpValid && mCacheBitmap != null) {
                Canvas canvas = view.getHolder().lockCanvas();
                if (canvas != null) {
                    float scale;
                    float width = canvas.getWidth();
                    float height = canvas.getHeight();
                    scale = Math.max(0, Math.min(height / mHeight, width / mWidth));
                    canvas.drawColor(0, android.graphics.PorterDuff.Mode.CLEAR);
                    if (BuildConfig.DEBUG) {
                        Log.d(TAG, "mStretch value: " + scale);
                    }
                    int offX = (int)(canvas.getWidth() - scale*mCacheBitmap.getWidth()) / 2;
                    int offY = (int)(canvas.getHeight() - scale*mCacheBitmap.getHeight()) / 2;

                    // scale
                    canvas.drawBitmap(mCacheBitmap,
                            new Rect(0,0,mCacheBitmap.getWidth(), mCacheBitmap.getHeight()),
                            new Rect(offX, offY,
                                    offX + (int)(scale*mCacheBitmap.getWidth()),
                                    offY + (int)(scale*mCacheBitmap.getHeight())), null);
                    view.getHolder().unlockCanvasAndPost(canvas);
                }
            }

        }
    }
}
