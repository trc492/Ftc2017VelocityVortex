package samples;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Test: Loop Timing", group="3543TestSamples")
public class TimingTest extends LinearOpMode
{
    private static final String TAG = "TrcDbg";
    private static final double DRIVE_POWER = 0.2;
    private static final double TURN_POWER = 0.5;
    private static final DcMotor.Direction LEFTWHEEL_DIRECTION = DcMotor.Direction.FORWARD;
    private static final DcMotor.Direction RIGHTWHEEL_DIRECTION = DcMotor.Direction.REVERSE;

    public void runOpMode()
    {
        DcMotor lfWheel = hardwareMap.dcMotor.get("lfWheel");
        DcMotor rfWheel = hardwareMap.dcMotor.get("rfWheel");
        DcMotor lrWheel = hardwareMap.dcMotor.get("lrWheel");
        DcMotor rrWheel = hardwareMap.dcMotor.get("rrWheel");
        lfWheel.setDirection(LEFTWHEEL_DIRECTION);
        lrWheel.setDirection(LEFTWHEEL_DIRECTION);
        rfWheel.setDirection(RIGHTWHEEL_DIRECTION);
        rrWheel.setDirection(RIGHTWHEEL_DIRECTION);

        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyroSensor");

        waitForStart();

        long prevTime = 0;
        long minInterval = Long.MAX_VALUE;
        long maxInterval = Long.MIN_VALUE;
        long loopCount = 0;
        long totalTime = 0;
        long startTime = System.currentTimeMillis();
        while (opModeIsActive())
        {
            long currTime = System.currentTimeMillis();
            if (prevTime != 0)
            {
                long interval = currTime - prevTime;
                totalTime += interval;
                if (interval < minInterval)
                {
                    minInterval = interval;
                }
                else if (interval > maxInterval)
                {
                    maxInterval = interval;
                }
                //
                // Turning right and checking gyro.
                //
                lfWheel.setPower(TURN_POWER);
                lrWheel.setPower(TURN_POWER);
                rfWheel.setPower(-TURN_POWER);
                rrWheel.setPower(-TURN_POWER);
                Log.i(TAG, String.format(
                        "%6d:[%7.3f: %6d] heading=%d",
                        loopCount, (currTime - startTime)/1000.0, interval, gyro.getIntegratedZValue()));
                //
                // Driving forward and checking encoders.
                //
//                lfWheel.setPower(DRIVE_POWER);
//                rfWheel.setPower(DRIVE_POWER);
//                lrWheel.setPower(DRIVE_POWER);
//                rrWheel.setPower(DRIVE_POWER);
//                Log.i(TAG, String.format(
//                        "%6d:[%7.3f: %6d] lf=%d, rf=%d, lr=%d, rr=%d",
//                        loopCount, (currTime - startTime)/1000.0, interval,
//                        lfWheel.getCurrentPosition(), rfWheel.getCurrentPosition(),
//                        lrWheel.getCurrentPosition(), rrWheel.getCurrentPosition()));
            }
            prevTime = currTime;
            loopCount++;
        }
        Log.i(TAG, String.format(
                "Summary: MinInterval=%d, MaxInterval=%s, AvgInterval=%d",
                minInterval, maxInterval, Math.round((double)totalTime/loopCount)));
    }
}
