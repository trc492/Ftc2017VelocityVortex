package team3543;

import android.widget.TextView;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import ftclib.FtcDcMotor;
import ftclib.FtcMRGyro;
import ftclib.FtcOpMode;
import ftclib.FtcServo;
import hallib.HalDashboard;
import hallib.HalUtil;
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
//    public FtcOpticalDistanceSensor lineDetectionSensor;

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

    public double[] lightZones = {RobotInfo.LINE_LEVEL_DARK, RobotInfo.LINE_LEVEL_WHITE};
    public TrcAnalogTrigger lineTrigger;

    // subsystems
    public Shooter shooter;
    public FtcServo leftButtonPusher;
    public FtcServo rightButtonPusher;
    public FtcDcMotor ballPickUp;
    public FtcServo ballGate;
    public FtcDcMotor conveyor;

    public Robot(TrcRobot.RunMode runMode)
    {
        opmode = FtcOpMode.getInstance();
        hardwareMap = opmode.hardwareMap;
        dashboard = opmode.getDashboard();
        activity = (FtcRobotControllerActivity)hardwareMap.appContext;
        hardwareMap.logDevices();
        dashboard.setTextView((TextView)activity.findViewById(FtcSampleCode.R.id.textOpMode));

        //10/25/16 senors/gyro/button pushers are not hooked up yet

        //sensors
        initSensors();

        //drivebase
        initDriveBase();

        //pid drives for FtcAuto
        initPidDrives();

        //init subsystems
        initSubsystems();
    }   //Robot

    public void startMode(TrcRobot.RunMode runMode)
    {
        FtcOpMode.getOpModeTracer().traceInfo(
                FtcOpMode.getOpModeName(), "Starting: %.3f", HalUtil.getCurrentTime());

        startSensors();
        startSubsystems();
        startDriveBase();
    }   //startMode

    public void stopMode(TrcRobot.RunMode runMode)
    {
        FtcOpMode.getOpModeTracer().traceInfo(
                FtcOpMode.getOpModeName(), "Stopping: %.3f", HalUtil.getCurrentTime());


        stopSensors();
        stopDriveBase();
        stopSubsystems();
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

    private void initSensors() {
        //
        // Initialize sensors.
        //
        gyro = new FtcMRGyro("gyroSensor");
        gyro.calibrate();
        beaconColorSensor = hardwareMap.colorSensor.get("colorSensor");
        beaconColorSensor.enableLed(false);
//        lineDetectionSensor = new FtcOpticalDistanceSensor("odsSensor");
    }

    private void startSensors() {
        gyro.resetZIntegrator();
        gyro.setEnabled(true);
    }

    private void stopSensors() {
        gyro.setEnabled(false);
    }

    private void initPidDrives() {
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
//        lineTrigger = new TrcAnalogTrigger(
//                "lineTrigger", lineDetectionSensor, 0, lightZones, this);
    }

    private void initDriveBase() {
        //
        // Create the motors.
        //
        leftFrontWheel = new FtcDcMotor("leftFrontWheel");
        rightFrontWheel = new FtcDcMotor("rightFrontWheel");
        leftRearWheel = new FtcDcMotor("leftRearWheel");
        rightRearWheel = new FtcDcMotor("rightRearWheel");
        leftFrontWheel.setInverted(true); //the left front wheel is inverted, uncomment this after the hardware is fixed
        leftRearWheel.setInverted(true);

        //need to initialize with the gyro once gyro is installed
        driveBase = new TrcDriveBase(
                leftFrontWheel, leftRearWheel, rightFrontWheel, rightRearWheel, gyro);
        driveBase.setYPositionScale(RobotInfo.DRIVE_INCHES_PER_COUNT);
    }

    private void startDriveBase() {
        driveBase.resetPosition();
    }

    private void stopDriveBase() {
        //nothing to do for now
    }
    private void initSubsystems() {
        //
        // Create and initialize subsystems here.
        //
        shooter = new Shooter("shooter");

        leftButtonPusher = new FtcServo("leftButtonPusherServo");
        leftButtonPusher.setPosition(RobotInfo.BUTTON_PUSHER_RETRACT_POSITION);

        rightButtonPusher = new FtcServo("rightButtonPusherServo");
        rightButtonPusher.setPosition(RobotInfo.BUTTON_PUSHER_RETRACT_POSITION);

        ballPickUp = new FtcDcMotor("pickUpMotor");
        ballPickUp.setInverted(true);

        ballGate = new FtcServo("gateServo");
        ballGate.setPosition(RobotInfo.BALLGATE_CLOSE_POSITION);

        conveyor = new FtcDcMotor("conveyorMotor");
    }

    private void startSubsystems() {
        /*
        partAccel.reset();
        conveyor.reset();

        ballPickUpMotor.resetPosition();
        ballPickUpMotor.setPower(0.0);

        leftPusherServo.setPosition(RobotInfo.BUTTON_PUSHER_RETRACT_POSITION);
        rightPusherServo.setPosition(RobotInfo.BUTTON_PUSHER_RETRACT_POSITION);
        */
    }
    private void stopSubsystems(){
        /*
        partAccel.reset();
        conveyor.reset();
        ballPickUpMotor.setPower(0.0);
        leftPusherServo.setPosition(RobotInfo.BUTTON_PUSHER_RETRACT_POSITION);
        rightPusherServo.setPosition(RobotInfo.BUTTON_PUSHER_RETRACT_POSITION);
        */
    }

}   //class Robot