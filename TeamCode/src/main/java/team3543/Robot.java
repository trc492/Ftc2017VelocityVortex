/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team3543;

import android.widget.TextView;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import ftclib.FtcAnalogGyro;
import ftclib.FtcAndroidTone;
import ftclib.FtcDcMotor;
import ftclib.FtcMRGyro;
import ftclib.FtcMRI2cColorSensor;
import ftclib.FtcMRRangeSensor;
import ftclib.FtcOpMode;
import ftclib.FtcOpticalDistanceSensor;
import ftclib.FtcRobotBattery;
import ftclib.FtcServo;
import hallib.HalDashboard;
import trclib.TrcAnalogTrigger;
import trclib.TrcDbgTrace;
import trclib.TrcDriveBase;
import trclib.TrcGyro;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot;

public class Robot implements TrcPidController.PidInput, TrcAnalogTrigger.TriggerHandler
{
    public static final boolean USE_RANGE_SENSOR = true;
    public static final boolean USE_LINE_DETECTOR = true;
    public static final boolean USE_ODS_LINE_DETECTOR = true;
    public static final boolean USE_COLOR_SENSOR = true;
    public static final boolean USE_ANALOG_GYRO = true;

    private static final String moduleName = "Robot";
    //
    // Global objects.
    //
    public FtcOpMode opmode;
    public HardwareMap hardwareMap;
    public HalDashboard dashboard;
    public FtcRobotControllerActivity activity;
    public TrcDbgTrace tracer;

    //
    // Sensors.
    //
    public TrcGyro gyro = null;
    public ColorSensor beaconColorSensor = null;
    public FtcMRI2cColorSensor lineDetectionSensor = null;
    public FtcOpticalDistanceSensor odsLineDetector = null;
    public FtcMRRangeSensor rangeSensor = null;
    private double prevRangeValue = 0.0;
    //
    // DriveBase subsystem.
    //
    public FtcDcMotor leftFrontWheel = null;
    public FtcDcMotor rightFrontWheel = null;
    public FtcDcMotor leftRearWheel = null;
    public FtcDcMotor rightRearWheel = null;
    public TrcDriveBase driveBase = null;
    public FtcAndroidTone androidTone = null;

    public TrcPidController encoderXPidCtrl = null;
    public TrcPidController encoderYPidCtrl = null;
    public TrcPidController gyroPidCtrl = null;
    public TrcPidController rangePidCtrl = null;
    public TrcPidDrive pidDrive = null;
    public TrcPidDrive rangePidDrive = null;

    public TrcAnalogTrigger lineTrigger = null;
    public TrcAnalogTrigger sonarTrigger = null;
    //
    // Other subsystems.
    //
    public FtcRobotBattery battery = null;
    public Shooter shooter = null;
    public FtcServo leftButtonPusher = null;
    public FtcServo rightButtonPusher = null;
    public FtcDcMotor ballPickUp = null;
    public FtcDcMotor conveyor = null;

    public Robot(TrcRobot.RunMode runMode)
    {
        //
        // Initialize global objects.
        //
        opmode = FtcOpMode.getInstance();
        hardwareMap = opmode.hardwareMap;
        dashboard = HalDashboard.getInstance();
        activity = (FtcRobotControllerActivity)hardwareMap.appContext;
        hardwareMap.logDevices();
        dashboard.setTextView((TextView)activity.findViewById(FtcSampleCode.R.id.textOpMode));
        tracer = FtcOpMode.getGlobalTracer();
        //
        // Initialize sensors.
        //
        if (USE_ANALOG_GYRO)
        {
            gyro = new FtcAnalogGyro("analogGyro", RobotInfo.ANALOG_GYRO_VOLT_PER_DEG_PER_SEC);
            ((FtcAnalogGyro)gyro).calibrate();
        }
        else
        {
            gyro = new FtcMRGyro("gyroSensor");
            ((FtcMRGyro)gyro).calibrate();
        }

        if (USE_COLOR_SENSOR)
        {
            beaconColorSensor = hardwareMap.colorSensor.get("colorSensor");
            beaconColorSensor.enableLed(false);
        }

        if (USE_LINE_DETECTOR)
        {
            if (USE_ODS_LINE_DETECTOR)
            {
                odsLineDetector = new FtcOpticalDistanceSensor("odsLineDetector");
            }
            else
            {
                lineDetectionSensor = new FtcMRI2cColorSensor("lineDetectionSensor", 0x40, false);
                lineDetectionSensor.setLEDEnabled(true);
            }
        }

        if (USE_RANGE_SENSOR)
        {
            rangeSensor = new FtcMRRangeSensor("rangeSensor");
        }
        //
        // Initialize DriveBase.
        //
        leftFrontWheel = new FtcDcMotor("leftFrontWheel");
        rightFrontWheel = new FtcDcMotor("rightFrontWheel");
        leftRearWheel = new FtcDcMotor("leftRearWheel");
        rightRearWheel = new FtcDcMotor("rightRearWheel");

        leftFrontWheel.motor.setMaxSpeed(RobotInfo.DRIVE_MAX_SPEED);
        rightFrontWheel.motor.setMaxSpeed(RobotInfo.DRIVE_MAX_SPEED);
        leftRearWheel.motor.setMaxSpeed(RobotInfo.DRIVE_MAX_SPEED);
        rightRearWheel.motor.setMaxSpeed(RobotInfo.DRIVE_MAX_SPEED);
        leftFrontWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);
        rightFrontWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);
        leftRearWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);
        rightRearWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);

        leftFrontWheel.setInverted(true);
        leftRearWheel.setInverted(true);

        driveBase = new TrcDriveBase(leftFrontWheel, leftRearWheel, rightFrontWheel, rightRearWheel, gyro);
        driveBase.setXPositionScale(RobotInfo.ENCODER_X_INCHES_PER_COUNT);
        driveBase.setYPositionScale(RobotInfo.ENOCDER_Y_INCHES_PER_COUNT);
        //
        // Initialize tone device.
        //
        androidTone = new FtcAndroidTone("AndroidTone");
        //
        // Initialize PID drive.
        //
        encoderXPidCtrl = new TrcPidController(
                "encoderXPidCtrl",
                RobotInfo.ENCODER_X_KP, RobotInfo.ENCODER_X_KI, RobotInfo.ENCODER_X_KD, RobotInfo.ENCODER_X_KF,
                RobotInfo.ENCODER_X_TOLERANCE, RobotInfo.ENCODER_X_SETTLING,
                this);
        encoderYPidCtrl = new TrcPidController(
                "encoderYPidCtrl",
                RobotInfo.ENCODER_Y_KP, RobotInfo.ENCODER_Y_KI, RobotInfo.ENCODER_Y_KD, RobotInfo.ENCODER_Y_KF,
                RobotInfo.ENCODER_Y_TOLERANCE, RobotInfo.ENCODER_Y_SETTLING,
                this);
        gyroPidCtrl = new TrcPidController(
                "gyroPidCtrl",
                RobotInfo.GYRO_KP, RobotInfo.GYRO_KI, RobotInfo.GYRO_KD, RobotInfo.GYRO_KF,
                RobotInfo.GYRO_TOLERANCE, RobotInfo.GYRO_SETTLING,
                this);
        gyroPidCtrl.setAbsoluteSetPoint(true);
        gyroPidCtrl.setNoOscillation(true);
        if (USE_RANGE_SENSOR)
        {
            rangePidCtrl = new TrcPidController(
                    "rangePidCtrl",
                    RobotInfo.RANGE_KP, RobotInfo.RANGE_KI, RobotInfo.RANGE_KD, RobotInfo.RANGE_KF,
                    RobotInfo.RANGE_TOLERANCE, RobotInfo.RANGE_SETTLING,
                    this);
            rangePidCtrl.setAbsoluteSetPoint(true);
            rangePidCtrl.setNoOscillation(true);
        }

        pidDrive = new TrcPidDrive("pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroPidCtrl);
        pidDrive.setStallTimeout(RobotInfo.PIDDRIVE_STALL_TIMEOUT);
        pidDrive.setBeep(androidTone);

        if (USE_LINE_DETECTOR)
        {
            double[] lightZones = {RobotInfo.LINE_DARK_LEVEL, RobotInfo.LINE_WHITE_LEVEL};

            if (USE_ODS_LINE_DETECTOR)
            {
                lineTrigger = new TrcAnalogTrigger("lineTrigger", odsLineDetector, 0, lightZones, this);
            }
            else
            {
                lineTrigger = new TrcAnalogTrigger("lineTrigger", lineDetectionSensor, 0, lightZones, this);
            }
        }

        if (USE_RANGE_SENSOR)
        {
            double[] sonarZones = {RobotInfo.WALL_NEAR, RobotInfo.WALL_FAR};
            rangePidDrive = new TrcPidDrive("rangePidDrive", driveBase, rangePidCtrl, encoderYPidCtrl, gyroPidCtrl);
            rangePidDrive.setStallTimeout(RobotInfo.PIDDRIVE_STALL_TIMEOUT);
            rangePidDrive.setBeep(androidTone);
            sonarTrigger = new TrcAnalogTrigger("sonarTrigger", rangeSensor, 0, sonarZones, this);
        }

        //
        // Initialize other subsystems.
        //

        battery = new FtcRobotBattery(leftFrontWheel.motor.getController());

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

        if (USE_LINE_DETECTOR && !USE_ODS_LINE_DETECTOR)
        {
            lineDetectionSensor.setLEDEnabled(true);
        }

        driveBase.resetPosition();
    }   //startMode

    public void stopMode(TrcRobot.RunMode runMode)
    {
        shooter.stop();

        gyro.setEnabled(false);

        if (USE_LINE_DETECTOR && !USE_ODS_LINE_DETECTOR)
        {
            lineDetectionSensor.setLEDEnabled(false);
        }
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
        else if (pidCtrl == rangePidCtrl)
        {
            input = rangeSensor.sensor.getDistance(DistanceUnit.INCH);
            //
            // The range sensor sometimes does not detect the echo and returns 255cm (approx. 100.4 inches).
            // If so, throw it away and use the previous value instead.
            //
            if (input > 100.0)
            {
                input = prevRangeValue;
            }
            else
            {
                prevRangeValue = input;
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
        tracer.traceInfo(moduleName, "%s: Entering zone %d (%.2f).", analogTrigger.toString(), zoneIndex, zoneValue);
        if (analogTrigger == lineTrigger && pidDrive.isEnabled())
        {
            if (zoneIndex > 0)
            {
                //
                // Encountering white line, abort PID drive.
                //
                pidDrive.cancel();
            }
        }
        else if (analogTrigger == sonarTrigger && pidDrive.isEnabled())
        {
            if (zoneIndex == 0)
            {
                //
                // Getting close to wall, abort PID drive.
                //
                pidDrive.cancel();
            }
        }
    }   //AnalogTriggerEvent

    public void setTurnPID(double xDistance, double yDistance, double heading)
    {
        double degrees = Math.abs(heading - driveBase.getHeading());

        if (xDistance != 0.0 || yDistance != 0)
        {
            gyroPidCtrl.setOutputRange(-RobotInfo.TURN_POWER_LIMIT, RobotInfo.TURN_POWER_LIMIT);
            gyroPidCtrl.setPID(RobotInfo.GYRO_KP, RobotInfo.GYRO_KI, RobotInfo.GYRO_KD, 0.0);
        }
        else if (degrees < RobotInfo.SMALL_TURN_THRESHOLD)
        {
            //
            // We are turning a small angle, use stronger PID.
            //
            gyroPidCtrl.setOutputRange(-RobotInfo.TURN_POWER_LIMIT, RobotInfo.TURN_POWER_LIMIT);
            gyroPidCtrl.setPID(RobotInfo.GYRO_SMALL_TURN_KP, RobotInfo.GYRO_SMALL_TURN_KI,
                               RobotInfo.GYRO_SMALL_TURN_KD, 0.0);
        }
        else if (degrees < RobotInfo.LARGE_TURN_THRESHOLD)
        {
            //
            // We are turning a medium angle, use normal PID.
            //
            gyroPidCtrl.setOutputRange(-RobotInfo.TURN_POWER_LIMIT, RobotInfo.TURN_POWER_LIMIT);
            gyroPidCtrl.setPID(RobotInfo.GYRO_KP, RobotInfo.GYRO_KI, RobotInfo.GYRO_KD, 0.0);
        }
        else
        {
            //
            // We are turning a large angle, use weaker PID.
            //
            gyroPidCtrl.setOutputRange(-RobotInfo.TURN_POWER_LIMIT, RobotInfo.TURN_POWER_LIMIT);
            gyroPidCtrl.setPID(RobotInfo.GYRO_LARGE_TURN_KP, RobotInfo.GYRO_LARGE_TURN_KI,
                               RobotInfo.GYRO_LARGE_TURN_KD, 0.0);
        }
    }

}   //class Robot