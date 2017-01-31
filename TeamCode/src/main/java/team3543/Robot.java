/*
 * Copyright (c) 2016 Titan Robotics Club (http://www.titanrobotics.com)
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

import android.speech.tts.TextToSpeech;
import android.widget.TextView;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import ftclib.FtcAnalogGyro;
import ftclib.FtcAndroidTone;
import ftclib.FtcDcMotor;
import ftclib.FtcMRColorSensor;
import ftclib.FtcMRGyro;
import ftclib.FtcMRRangeSensor;
import ftclib.FtcOpMode;
import ftclib.FtcOpticalDistanceSensor;
import ftclib.FtcRobotBattery;
import ftclib.FtcServo;
import hallib.HalDashboard;
import trclib.TrcAnalogTrigger;
import trclib.TrcDbgTrace;
import trclib.TrcDriveBase;
import trclib.TrcEvent;
import trclib.TrcGyro;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot;
import trclib.TrcUtil;

public class Robot implements TrcPidController.PidInput, TrcAnalogTrigger.TriggerHandler
{
    static final boolean USE_RANGE_SENSOR = true;
    static final boolean USE_LINE_DETECTOR = true;
    static final boolean USE_ODS_LINE_DETECTOR = true;
    static final boolean USE_COLOR_SENSOR = true;
    private static final boolean USE_ANALOG_GYRO = true;
    private static final boolean USE_SPEECH = true;

    private static final String moduleName = "Robot";
    //
    // Global objects.
    //
    public HalDashboard dashboard;
    public TrcDbgTrace tracer;
    //
    // Text To Speech.
    //
    public TextToSpeech textToSpeech = null;
    //
    // Sensors.
    //
    TrcGyro gyro = null;
    FtcMRColorSensor beaconColorSensor = null;
    FtcMRColorSensor lineDetectionSensor = null;
    FtcOpticalDistanceSensor odsLineDetector = null;
    private FtcMRRangeSensor rangeSensor = null;
    private double prevRangeValue = 0.0;
    //
    // DriveBase subsystem.
    //
    FtcDcMotor leftFrontWheel = null;
    FtcDcMotor rightFrontWheel = null;
    FtcDcMotor leftRearWheel = null;
    FtcDcMotor rightRearWheel = null;
    TrcDriveBase driveBase = null;
    FtcRobotBattery battery = null;

    TrcPidController encoderXPidCtrl = null;
    TrcPidController encoderYPidCtrl = null;
    TrcPidController gyroPidCtrl = null;
    TrcPidController rangePidCtrl = null;
    TrcPidDrive pidDrive = null;
    TrcPidDrive rangePidDrive = null;

    TrcAnalogTrigger<FtcOpticalDistanceSensor.DataType> odsTrigger = null;
    TrcAnalogTrigger<FtcMRColorSensor.DataType> colorTrigger = null;
    double targetHeading = 0.0;
    //
    // Other subsystems.
    //
    Shooter shooter = null;
    FtcServo leftButtonPusher = null;
    FtcServo rightButtonPusher = null;
    FtcDcMotor ballPickUp = null;
    FtcDcMotor conveyor = null;

    public Robot(TrcRobot.RunMode runMode)
    {
        //
        // Initialize global objects.
        //
        HardwareMap hardwareMap = FtcOpMode.getInstance().hardwareMap;
        dashboard = HalDashboard.getInstance();
        FtcRobotControllerActivity activity = (FtcRobotControllerActivity)hardwareMap.appContext;
        hardwareMap.logDevices();
        dashboard.setTextView((TextView)activity.findViewById(FtcSampleCode.R.id.textOpMode));
        tracer = FtcOpMode.getGlobalTracer();
        //
        // Text To Speech.
        //
        if (USE_SPEECH)
        {
            textToSpeech = FtcOpMode.getInstance().getTextToSpeech();
        }
        //
        // Initialize sensors.
        //
        if (USE_ANALOG_GYRO)
        {
            gyro = new FtcAnalogGyro("analogGyro", RobotInfo.ANALOG_GYRO_VOLT_PER_DEG_PER_SEC);
            ((FtcAnalogGyro)gyro).calibrate();
            gyro.setScale(0, RobotInfo.ANALOG_GYRO_SCALE);
        }
        else
        {
            gyro = new FtcMRGyro("gyroSensor");
            ((FtcMRGyro)gyro).calibrate();
        }

        if (USE_COLOR_SENSOR)
        {
            beaconColorSensor = new FtcMRColorSensor("colorSensor");
            beaconColorSensor.sensor.enableLed(false);
//            beaconColorSensor.setDeviceEnabled(false);
        }

        if (USE_LINE_DETECTOR)
        {
            if (USE_ODS_LINE_DETECTOR)
            {
                odsLineDetector = new FtcOpticalDistanceSensor("odsLineDetector");
            }
            else
            {
                lineDetectionSensor = new FtcMRColorSensor("lineDetectionSensor");
                lineDetectionSensor.sensor.setI2cAddress(I2cAddr.create8bit(0x40));
                lineDetectionSensor.sensor.enableLed(true);
//                lineDetectionSensor.setDeviceEnabled(false);
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
        driveBase.setYPositionScale(RobotInfo.ENCODER_Y_INCHES_PER_COUNT);

        battery = new FtcRobotBattery(leftFrontWheel.motor.getController());
        //
        // Initialize tone device.
        //
        FtcAndroidTone androidTone = new FtcAndroidTone("AndroidTone");
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
        gyroPidCtrl.setOutputRange(-RobotInfo.TURN_POWER_LIMIT, RobotInfo.TURN_POWER_LIMIT);
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
                odsTrigger = new TrcAnalogTrigger<>(
                        "odsTrigger", odsLineDetector, 0, FtcOpticalDistanceSensor.DataType.RAW_LIGHT_DETECTED,
                        lightZones, this);
            }
            else
            {
                colorTrigger = new TrcAnalogTrigger<>(
                        "colorTrigger", lineDetectionSensor, 0, FtcMRColorSensor.DataType.WHITE, lightZones, this);
            }
        }

        if (USE_RANGE_SENSOR)
        {
            rangePidDrive = new TrcPidDrive("rangePidDrive", driveBase, rangePidCtrl, encoderYPidCtrl, gyroPidCtrl);
            rangePidDrive.setStallTimeout(RobotInfo.PIDDRIVE_STALL_TIMEOUT);
            rangePidDrive.setBeep(androidTone);
        }

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

        //
        // Wait for gyro calibration to complete if not already.
        //
        while (gyro.isCalibrating())
        {
            TrcUtil.sleep(10);
        }
        //
        // Tell the driver initialization is complete.
        //
        if (textToSpeech != null)
        {
            textToSpeech.speak("Initialization complete!", TextToSpeech.QUEUE_FLUSH, null);
        }
    }   //Robot

    void startMode(TrcRobot.RunMode runMode)
    {
        if (USE_LINE_DETECTOR && !USE_ODS_LINE_DETECTOR)
        {
            lineDetectionSensor.sensor.enableLed(true);
        }
        //
        // Since our gyro is analog, we need to enable its integrator.
        //
        gyro.setEnabled(true);
        //
        // Reset all X, Y and heading values.
        //
        driveBase.resetPosition();
        targetHeading = 0.0;
    }   //startMode

    void stopMode(TrcRobot.RunMode runMode)
    {
        shooter.stop();
        //
        // Disable the gyro integrator.
        //
        gyro.setEnabled(false);

        if (USE_LINE_DETECTOR && !USE_ODS_LINE_DETECTOR)
        {
            lineDetectionSensor.sensor.enableLed(false);
        }

        if (textToSpeech != null)
        {
            textToSpeech.stop();
            textToSpeech.shutdown();
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
            if (input > 100.0 && prevRangeValue <= 100.0)
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
        if ((analogTrigger == odsTrigger || analogTrigger == colorTrigger) && pidDrive.isActive())
        {
            if (zoneIndex > 0)
            {
                //
                // Detected white line, abort PID drive.
                //
                tracer.traceInfo(moduleName, "%s: Found white line.", analogTrigger.toString());
                pidDrive.cancel();
            }
        }
    }   //AnalogTriggerEvent

    private void setDrivePID(double xDistance, double yDistance, double heading)
    {
        double degrees = Math.abs(heading - driveBase.getHeading());
        xDistance = Math.abs(xDistance);
        yDistance = Math.abs(yDistance);
        //
        // No oscillation if turn-only.
        //
        boolean noOscillation = degrees != 0.0 && xDistance == 0.0 && yDistance == 0.0;
        gyroPidCtrl.setNoOscillation(noOscillation);
        tracer.traceInfo("setDrivePID", "NoOscillation=%s", Boolean.toString(noOscillation));
        if (xDistance != 0.0 && xDistance < RobotInfo.SMALL_X_THRESHOLD)
        {
            //
            // Small X movement, use stronger X PID to overcome friction.
            //
            encoderXPidCtrl.setPID(
                    RobotInfo.ENCODER_SMALL_X_KP, RobotInfo.ENCODER_SMALL_X_KI, RobotInfo.ENCODER_SMALL_X_KD, 0.0);
        }
        else
        {
            //
            // Use normal X PID.
            //
            encoderXPidCtrl.setPID(RobotInfo.ENCODER_X_KP, RobotInfo.ENCODER_X_KI, RobotInfo.ENCODER_X_KD, 0.0);
        }

        if (yDistance != 0.0 && yDistance < RobotInfo.SMALL_Y_THRESHOLD)
        {
            //
            // Small Y movement, use stronger Y PID to overcome friction.
            //
            encoderYPidCtrl.setPID(
                    RobotInfo.ENCODER_SMALL_Y_KP, RobotInfo.ENCODER_SMALL_Y_KI, RobotInfo.ENCODER_SMALL_Y_KD, 0.0);
        }
        else
        {
            //
            // Use normal Y PID.
            //
            encoderYPidCtrl.setPID(RobotInfo.ENCODER_Y_KP, RobotInfo.ENCODER_Y_KI, RobotInfo.ENCODER_Y_KD, 0.0);
        }

        if (degrees != 0.0 && degrees < RobotInfo.SMALL_TURN_THRESHOLD)
        {
            //
            // Small turn, use stronger turn PID to overcome friction.
            //
            gyroPidCtrl.setPID(
                    RobotInfo.GYRO_SMALL_TURN_KP, RobotInfo.GYRO_SMALL_TURN_KI, RobotInfo.GYRO_SMALL_TURN_KD, 0.0);
        }
        else
        {
            //
            // Use normal Y PID.
            //
            gyroPidCtrl.setPID(RobotInfo.GYRO_KP, RobotInfo.GYRO_KI, RobotInfo.GYRO_KD, 0.0);
        }
    }   //setDrivePID

    void setPIDDriveTarget(
            double xDistance, double yDistance, double heading, boolean holdTarget, TrcEvent event)
    {
        setDrivePID(xDistance, yDistance, heading);
        pidDrive.setTarget(xDistance, yDistance, heading, holdTarget, event);
    }   //setPIDDriveTarget

    void traceStateInfo(double elapsedTime, String stateName, double xDistance, double yDistance, double heading)
    {
        tracer.traceInfo(
                moduleName,
                "[%5.3f] %17s: xPos=%6.2f/%6.2f,yPos=%6.2f/%6.2f,heading=%6.1f/%6.1f,range=%5.2f,volt=%5.2fV(%5.2fV)",
                elapsedTime, stateName,
                driveBase.getXPosition(), xDistance, driveBase.getYPosition(), yDistance,
                driveBase.getHeading(), heading, getInput(rangePidCtrl),
                battery.getCurrentVoltage(), battery.getLowestVoltage());
    }   //traceStateInfo

    double selectParameter(boolean startNear, FtcAuto.Alliance alliance, double nearRed, double nearBlue,
                           double farRed, double farBlue)
    {
        if (startNear)
        {
            if (alliance == FtcAuto.Alliance.RED_ALLIANCE)
            {
                return nearRed;
            }
            else
            {
                return nearBlue;
            }
        }
        else
        {
            if (alliance == FtcAuto.Alliance.RED_ALLIANCE)
            {
                return farRed;
            }
            else
            {
                return farBlue;
            }
        }
    }   //selectParameter

}   //class Robot
