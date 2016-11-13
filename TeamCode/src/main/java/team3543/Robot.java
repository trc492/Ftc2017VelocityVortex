package team3543;

import android.widget.TextView;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import ftclib.FtcDcMotor;
import ftclib.FtcMRGyro;
import ftclib.FtcMRI2cColorSensor;
import ftclib.FtcOpMode;
import ftclib.FtcServo;
import hallib.HalDashboard;
import trclib.TrcAnalogTrigger;
import trclib.TrcDriveBase;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot;

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
    public FtcMRI2cColorSensor lineDetectionSensor;
    //
    // DriveBase subsystem.
    //
    public FtcDcMotor leftFrontWheel;
    public FtcDcMotor rightFrontWheel;
    public FtcDcMotor leftRearWheel;
    public FtcDcMotor rightRearWheel;
    public TrcDriveBase driveBase;

    public TrcPidController encoderXPidCtrl;
    public TrcPidController encoderYPidCtrl;
    public TrcPidController gyroPidCtrl;
    public TrcPidDrive pidDrive;

    public double[] lightZones = {RobotInfo.LINE_DARK_LEVEL, RobotInfo.LINE_WHITE_LEVEL};
    public TrcAnalogTrigger lineTrigger;
    //
    // Other subsystems.
    //
    public Shooter shooter;
    public FtcServo leftButtonPusher;
    public FtcServo rightButtonPusher;
    public FtcDcMotor ballPickUp;
    public FtcDcMotor conveyor;

    public Robot(TrcRobot.RunMode runMode)
    {
        //
        // Initialize global objects.
        //
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
        lineDetectionSensor = new FtcMRI2cColorSensor("lineDetectionSensor", 0x40, false);
        lineDetectionSensor.setLEDEnabled(true);
        //
        // Initialize DriveBase.
        //
        leftFrontWheel = new FtcDcMotor("leftFrontWheel");
        rightFrontWheel = new FtcDcMotor("rightFrontWheel");
        leftRearWheel = new FtcDcMotor("leftRearWheel");
        rightRearWheel = new FtcDcMotor("rightRearWheel");
        leftFrontWheel.setInverted(true);
        leftRearWheel.setInverted(true);
        driveBase = new TrcDriveBase(
                leftFrontWheel, leftRearWheel, rightFrontWheel, rightRearWheel, gyro);
        driveBase.setXPositionScale(RobotInfo.ENCODER_X_INCHES_PER_COUNT);
        driveBase.setYPositionScale(RobotInfo.ENOCDER_Y_INCHES_PER_COUNT);
        //
        // Initialize PID drive.
        //
        encoderXPidCtrl = new TrcPidController(
                "encoderXPidCtrl",
                RobotInfo.ENCODER_X_KP, RobotInfo.ENCODER_X_KI,
                RobotInfo.ENCODER_X_KD, RobotInfo.ENCODER_X_KF,
                RobotInfo.ENCODER_X_TOLERANCE, RobotInfo.ENCODER_X_SETTLING,
                this);
        encoderYPidCtrl = new TrcPidController(
                "encoderYPidCtrl",
                RobotInfo.ENCODER_Y_KP, RobotInfo.ENCODER_Y_KI,
                RobotInfo.ENCODER_Y_KD, RobotInfo.ENCODER_Y_KF,
                RobotInfo.ENCODER_Y_TOLERANCE, RobotInfo.ENCODER_Y_SETTLING,
                this);
        gyroPidCtrl = new TrcPidController(
                "gyroPidCtrl",
                RobotInfo.GYRO_KP, RobotInfo.GYRO_KI,
                RobotInfo.GYRO_KD, RobotInfo.GYRO_KF,
                RobotInfo.GYRO_TOLERANCE, RobotInfo.GYRO_SETTLING,
                this);
        pidDrive = new TrcPidDrive("pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroPidCtrl);
        lineTrigger = new TrcAnalogTrigger(
                "lineTrigger", lineDetectionSensor, 0, lightZones, this);

        //
        // Initialize other subsystems.
        //

        shooter = new Shooter("shooter");

        leftButtonPusher = new FtcServo("leftButtonPusherServo");
        leftButtonPusher.setPosition(RobotInfo.BUTTON_PUSHER_RETRACT_POSITION);
        rightButtonPusher = new FtcServo("rightButtonPusherServo");
        rightButtonPusher.setPosition(RobotInfo.BUTTON_PUSHER_RETRACT_POSITION);

        ballPickUp = new FtcDcMotor("pickUpMotor");
        ballPickUp.setInverted(true);

        conveyor = new FtcDcMotor("conveyorMotor");
    }   //Robot

    public void startMode(TrcRobot.RunMode runMode)
    {
        gyro.resetZIntegrator();
        gyro.setEnabled(true);
        lineDetectionSensor.setLEDEnabled(true);
        driveBase.resetPosition();
    }   //startMode

    public void stopMode(TrcRobot.RunMode runMode)
    {
        lineDetectionSensor.setLEDEnabled(false);
        gyro.setEnabled(false);
        shooter.stop();
    }   //stopMode

    //
    // Implements TrcPidController.PidInput
    //

    @Override
    public double getInput(TrcPidController pidCtrl)
    {
        double input = 0.0;

        if (pidCtrl == encoderXPidCtrl)
        {
            input = driveBase.getXPosition();
        }
        else if (pidCtrl == encoderYPidCtrl)
        {
            input = driveBase.getYPosition();
        }
        else if (pidCtrl == gyroPidCtrl)
        {
            input = driveBase.getHeading();
        }

        return input;
    }   //getInput

    //
    // Implements TrcAnalogTrigger.TriggerHandler
    //

    @Override
    public void AnalogTriggerEvent(TrcAnalogTrigger analogTrigger, int zoneIndex, double zoneValue)
    {
        if (analogTrigger == lineTrigger && pidDrive.isEnabled())
        {
            FtcOpMode.getGlobalTracer().traceInfo("Robot", "Entering zone %d (%.0f).",
                                                  zoneIndex, zoneValue);
            if (zoneIndex > 0)
            {
                //
                // Encountering white line, abort PID drive.
                //
                pidDrive.cancel();
            }
        }
    }   //AnalogTriggerEvent

}   //class Robot