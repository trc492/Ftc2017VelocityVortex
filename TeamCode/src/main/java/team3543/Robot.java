package team3543;

import android.widget.TextView;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import ftclib.FtcDcMotor;
import ftclib.FtcMRGyro;
import ftclib.FtcMRI2cColorSensor;
import ftclib.FtcOpMode;
import ftclib.FtcUltrasonicSensor;
import hallib.HalDashboard;
import hallib.HalUtil;
import trclib.TrcAnalogTrigger;
import trclib.TrcDriveBase;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot;
import trclib.TrcSensor;

public class Robot implements TrcPidController.PidInput, TrcAnalogTrigger.TriggerHandler
{
    //
    // Global objects.
    //
    public FtcOpMode opmode;
    public HardwareMap hardwareMap;
    public HalDashboard dashboard;
    public FtcRobotControllerActivity activity;
    //
    // Sensors.
    //
    public FtcMRGyro gyro;
    public ColorSensor beaconColorSensor;
    public FtcMRI2cColorSensor lineFollowColorSensor;
    public FtcUltrasonicSensor sonarSensor;
    public double prevSonarValue;

    //
    // DriveBase subsystem.
    //
    public FtcDcMotor leftFrontWheel;
    public FtcDcMotor rightFrontWheel;
    public FtcDcMotor leftRearWheel;
    public FtcDcMotor rightRearWheel;
    public TrcDriveBase driveBase;

    public TrcPidController encoderPidCtrl;
    public TrcPidController gyroPidCtrl;
    public TrcPidDrive pidDrive;

    public double[] color4Zones = {RobotInfo.COLOR_BLACK, RobotInfo.COLOR_BLUE,
                                    RobotInfo.COLOR_RED, RobotInfo.COLOR_WHITE};
    public double[] color2Zones = {RobotInfo.COLOR_RED, RobotInfo.COLOR_WHITE};
    public TrcPidController sonarPidCtrl;
    public TrcPidController colorPidCtrl;
    public TrcPidDrive pidLineFollow;
    public TrcAnalogTrigger colorTrigger;

    // subsystems
    public PartAccel partAccel;
    public ButtonPusher leftPusher;
    public ButtonPusher rightPusher;
    public BallPickUp ballPickUp;


    public Robot(TrcRobot.RunMode runMode)
    {
        opmode = FtcOpMode.getInstance();
        hardwareMap = opmode.hardwareMap;
        dashboard = opmode.getDashboard();
        activity = (FtcRobotControllerActivity)hardwareMap.appContext;
        hardwareMap.logDevices();
        dashboard.setTextView((TextView)activity.findViewById(FtcSampleCode.R.id.textOpMode));
        //
        // Initialize sensors.
        //
        gyro = new FtcMRGyro("gyroSensor");
        gyro.calibrate();
        beaconColorSensor = hardwareMap.colorSensor.get("colorSensor");
        beaconColorSensor.enableLed(false);
        lineFollowColorSensor = new FtcMRI2cColorSensor("i2cColorSensor", 0x40, false);
        lineFollowColorSensor.setLEDEnabled(true);
        sonarSensor = new FtcUltrasonicSensor("legoSonarSensor");
        sonarSensor.setScale(RobotInfo.SONAR_INCHES_PER_CM);
        //
        // DriveBase subsystem.
        //
        leftFrontWheel = new FtcDcMotor("leftFrontWheel");
        rightFrontWheel = new FtcDcMotor("rightFrontWheel");
        leftRearWheel = new FtcDcMotor("leftRearWheel");
        rightRearWheel = new FtcDcMotor("rightRearWheel");
        leftFrontWheel.setInverted(true);
        leftRearWheel.setInverted(true);
        driveBase = new TrcDriveBase(
                leftFrontWheel, leftRearWheel, rightFrontWheel, rightRearWheel, gyro);
        driveBase.setYPositionScale(RobotInfo.DRIVE_INCHES_PER_COUNT);
        //
        // PID Drive.
        //
        encoderPidCtrl = new TrcPidController(
                "encoderPidCtrl",
                RobotInfo.DRIVE_KP, RobotInfo.DRIVE_KI,
                RobotInfo.DRIVE_KD, RobotInfo.DRIVE_KF,
                RobotInfo.DRIVE_TOLERANCE, RobotInfo.DRIVE_SETTLING,
                this);
        gyroPidCtrl = new TrcPidController(
                "gyroPidCtrl",
                RobotInfo.GYRO_KP, RobotInfo.GYRO_KI,
                RobotInfo.GYRO_KD, RobotInfo.GYRO_KF,
                RobotInfo.GYRO_TOLERANCE, RobotInfo.GYRO_SETTLING,
                this);
        pidDrive = new TrcPidDrive("pidDrive", driveBase, null, encoderPidCtrl, gyroPidCtrl);
        //
        // PID Line following.
        //
        sonarPidCtrl = new TrcPidController(
                "sonarPidCtrl",
                RobotInfo.SONAR_KP, RobotInfo.SONAR_KI,
                RobotInfo.SONAR_KD, RobotInfo.SONAR_KF,
                RobotInfo.SONAR_TOLERANCE, RobotInfo.SONAR_SETTLING,
                this);
        sonarPidCtrl.setAbsoluteSetPoint(true);
        sonarPidCtrl.setInverted(true);
        colorPidCtrl = new TrcPidController(
                "colorPidCtrl",
                RobotInfo.COLOR_KP, RobotInfo.COLOR_KI,
                RobotInfo.COLOR_KD, RobotInfo.COLOR_KF,
                RobotInfo.COLOR_TOLERANCE, RobotInfo.COLOR_SETTLING,
                this);
        colorPidCtrl.setAbsoluteSetPoint(true);
        pidLineFollow = new TrcPidDrive(
                "pidLineFollow", driveBase, null, sonarPidCtrl, colorPidCtrl);
        colorTrigger = new TrcAnalogTrigger(
                "colorTrigger", lineFollowColorSensor, 0, color4Zones, this);
        //
        // Create subsystems here.
        //
        partAccel = new PartAccel("shooter");

        leftPusher = new ButtonPusher("leftPusher");
        rightPusher = new ButtonPusher("rightPusher");

        ballPickUp = new BallPickUp("ballPickUp");
    }   //Robot

    public void startMode(TrcRobot.RunMode runMode)
    {
        gyro.resetZIntegrator();
        gyro.setEnabled(true);
        lineFollowColorSensor.setLEDEnabled(true);
        sonarSensor.setEnabled(true);
        prevSonarValue = (Double)sonarSensor.getData(0).value;
        driveBase.resetPosition();
    }   //startMode

    public void stopMode(TrcRobot.RunMode runMode)
    {
        FtcOpMode.getOpModeTracer().traceInfo(
                FtcOpMode.getOpModeName(), "Stopping: %.3f", HalUtil.getCurrentTime());
        gyro.setEnabled(false);
        lineFollowColorSensor.setLEDEnabled(false);
        sonarSensor.setEnabled(false);
    }   //stopMode

    //
    // Implements TrcPidController.PidInput
    //

    @Override
    public double getInput(TrcPidController pidCtrl)
    {
        double input = 0.0;

        if (pidCtrl == encoderPidCtrl)
        {
            input = driveBase.getYPosition();
        }
        else if (pidCtrl == gyroPidCtrl)
        {
            input = driveBase.getHeading();
        }
        else if (pidCtrl == sonarPidCtrl)
        {
            input = (Double)sonarSensor.getData(0).value;
            //
            // The Lego Ultrasonic sensor occasionally returns a zero.
            // This is causing havoc to PID control. Let's detect that
            // and discard it and reuse the previous value instead.
            //
            if (input == 0.0)
            {
                input = prevSonarValue;
            }
            else
            {
                prevSonarValue = input;
            }
        }
        else if (pidCtrl == colorPidCtrl)
        {
            TrcSensor.SensorData data = lineFollowColorSensor.getWhiteValue();
            input = data.value != null? (double)(Integer)data.value: 0.0;
            //
            // Give it a deadband to minimize fish tailing.
            //
            if (Math.abs(input - RobotInfo.COLOR_LINE_EDGE_LEVEL) < RobotInfo.COLOR_LINE_EDGE_DEADBAND)
            {
                input = RobotInfo.COLOR_LINE_EDGE_LEVEL;
            }
        }

        return input;
    }   //getInput

    //
    // Implements TrcAnalogTrigger.TriggerHandler
    //

    @Override
    public void AnalogTriggerEvent(TrcAnalogTrigger analogTrigger, int zoneIndex, double zoneValue)
    {
        if (analogTrigger == colorTrigger && pidDrive.isEnabled())
        {
            FtcOpMode.getOpModeTracer().traceInfo("Robot", "Entering zone %d (%.0f).",
                                                  zoneIndex, zoneValue);
            if (zoneIndex > 0)
            {
                //
                // Encountering blue, red or white line, abort PID drive.
                //
                pidDrive.cancel();
            }
        }
    }   //AnalogTriggerEvent

}   //class Robot
