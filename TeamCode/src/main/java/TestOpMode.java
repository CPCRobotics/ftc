import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorManager;


@Autonomous(name="EagleBot: Test")
@Disabled
public class TestOpMode extends OpMode
{
    SensorManager sensorMgr;
    Sensor Gyro;
    @Override
    public void init()
    {

        sensorMgr = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
        Gyro = sensorMgr.getDefaultSensor(Sensor.TYPE_GYROSCOPE );

    }

    @Override
    public void start()
    {
        telemetry.addData("gmag",0);
    }

    @Override
    public void loop()
    {

    }
}
