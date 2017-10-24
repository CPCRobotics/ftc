package cpc.robotics.colorcheck;

import android.hardware.Camera;
import android.os.Bundle;
import android.view.WindowManager;
import android.widget.SeekBar;
import android.widget.TextView;

import org.lasarobotics.vision.util.ScreenOrientation;
import org.lasarobotics.vision.util.color.ColorHSV;

import cpc.robotics.vision.BlurExtension;
import cpc.robotics.vision.CameraControlExtension;
import cpc.robotics.vision.ImageRotationExtension;
import cpc.robotics.vision.VisionEnabledActivity;
import cpc.robotics.vision.VisionHelper;

public class ViewColor extends VisionEnabledActivity implements SeekBar.OnSeekBarChangeListener {

    private VisionHelper vision;
    private final BlurExtension blur = new BlurExtension();
    private final FeedbackExtension feedback = new FeedbackExtension();
    private final ImageRotationExtension rotation = new ImageRotationExtension();
    private final CameraControlExtension cameraControl = new CameraControlExtension();
    private SeekBar hueLower, hueUpper, satLower, satUpper, valLower, valUpper;
    private TextView summaryText;

    public ViewColor() {
        super();
    }

    /**
     * Called when the activity is first created.
     */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_view_color);
        hueLower = connectSeekBar(R.id.hueLower);
        hueUpper = connectSeekBar(R.id.hueUpper);
        satLower = connectSeekBar(R.id.satLower);
        satUpper = connectSeekBar(R.id.satUpper);
        valLower = connectSeekBar(R.id.valLower);
        valUpper = connectSeekBar(R.id.valUpper);
        summaryText = (TextView)findViewById(R.id.summaryText);

        vision = new VisionHelper(this, Camera.CameraInfo.CAMERA_FACING_BACK, 500, 500);
        vision.addExtensions(blur, feedback, rotation, cameraControl);
        blur.setBlurWidth(3);
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        //rotation.setActivityOrientationFixed(ScreenOrientation.LANDSCAPE);
        rotation.setZeroOrientation(ScreenOrientation.LANDSCAPE_REVERSE);
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();

        //
        // Configures VisionHelper to output preview to CameraTestSurfaceView
        //
        initializeVision(R.id.previewSurface, vision);

        //
        // Note that the camera is opened when activity is 'resumed', see base class
        //
        updateRange();
    }

    @Override
    public void onProgressChanged(SeekBar seekBar, int i, boolean b) {
        updateRange();
    }

    @Override
    public void onStartTrackingTouch(SeekBar seekBar) {

    }

    @Override
    public void onStopTrackingTouch(SeekBar seekBar) {

    }

    protected SeekBar connectSeekBar(int id) {
        SeekBar ctrl = (SeekBar)findViewById(id);
        ctrl.setOnSeekBarChangeListener(this);
        return ctrl;
    }

    protected int [] readPair(SeekBar lower, SeekBar upper) {
        return new int[] { lower.getProgress(), upper.getProgress() };
    }

    protected int [] readWrapPair(SeekBar lower, SeekBar upper) {
        int [] pair = readPair(lower, upper);
        if (pair[0] > pair[1]) {
            pair[1] += 256;
        }
        return pair;
    }

    protected int [] readBoundPair(SeekBar lower, SeekBar upper) {
        int[] pair = readPair(lower, upper);
        if (pair[0] > pair[1]) {
            int x = pair[0];
            pair[0] = pair[1];
            pair[1] = x;
        }
        return pair;
    }

    void updateRange() {
        int [] hue = readWrapPair(hueLower, hueUpper);
        int [] sat = readBoundPair(satLower, satUpper);
        int [] val = readBoundPair(valLower, valUpper);
        ColorHSV lower = new ColorHSV(hue[0], sat[0], val[0]);
        ColorHSV upper = new ColorHSV(hue[1], sat[1], val[1]);
        feedback.setRange(lower, upper);
        summaryText.setText(feedback.toString());
    }
}
