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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import ftclib.FtcChoiceMenu;
import ftclib.FtcGamepad;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import ftclib.FtcValueMenu;
import trclib.TrcAnalogInput;
import trclib.TrcDbgTrace;
import trclib.TrcEvent;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

@TeleOp(name="Test", group="3543Test")
public class FtcTest extends FtcTeleOp implements FtcMenu.MenuButtons, FtcGamepad.ButtonHandler
{
    private static final boolean debugXPid = false;
    private static final boolean debugYPid = false;
    private static final boolean debugGyroPid = false;
    private static final boolean debugRangePid = true;
    private TrcDbgTrace tracer = FtcOpMode.getGlobalTracer();

    private enum Test
    {
        SENSORS_TEST,
        MOTORS_TEST,
        Y_TIMED_DRIVE,
        X_TIMED_DRIVE,
        Y_DISTANCE_DRIVE,
        X_DISTANCE_DRIVE,
        RANGE_DRIVE,
        GYRO_TURN
    }   //enum Test

    private enum State
    {
        START,
        DONE
    }   //enum State

    //
    // State machine.
    //
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine sm;
    //
    // Menu choices.
    //
    private Test test = Test.SENSORS_TEST;
    private double driveTime = 0.0;
    private double driveDistance = 0.0;
    private double rangeDistance = 0.0;
    private double turnDegrees = 0.0;

    private int motorIndex = 0;
    private double ballGatePos = RobotInfo.BALLGATE_DOWN_POSITION;
    private double buttonPusherPos = 0.0;

    //
    // Implements FtcOpMode interface.
    //

    @Override
    public void initRobot()
    {
        super.initRobot();
        //
        // Miscellaneous.
        //
        event = new TrcEvent("TestEvent");
        timer = new TrcTimer("TestTimer");
        sm = new TrcStateMachine("TestSM");
        //
        // Choice menus.
        //
        doMenus();
        sm.start(State.START);
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    //
    // Must override TeleOp so it doesn't fight with us.
    //
    @Override
    public void runPeriodic(double elapsedTime)
    {
        //
        // Allow TeleOp to run so we can control the robot in test sensor mode.
        //
        if (test == Test.SENSORS_TEST)
        {
            super.runPeriodic(elapsedTime);
        }
    }   //runPeriodic

    @Override
    public void runContinuous(double elapsedTime)
    {
        State state = (State)sm.getState();
        dashboard.displayPrintf(8, "%s: %s", test.toString(), state != null? state.toString(): "STOPPED!");

        switch (test)
        {
            case SENSORS_TEST:
                doSensorsTest();
                break;

            case MOTORS_TEST:
                doMotorsTest();
                break;

            case Y_TIMED_DRIVE:
                doTimedDrive(0.0, 0.2, 0.0, driveTime);
                break;

            case X_TIMED_DRIVE:
                doTimedDrive(1.0, 0.0, 0.0, driveTime);
                break;

            case Y_DISTANCE_DRIVE:
                doPidDrive(0.0, driveDistance, 0.0);
                break;

            case X_DISTANCE_DRIVE:
                doPidDrive(driveDistance, 0.0, 0.0);
                break;

            case RANGE_DRIVE:
                if (robot.USE_RANGE_SENSOR)
                {
                    doRangeDrive(rangeDistance);
                }
                break;

            case GYRO_TURN:
                doPidDrive(0.0, 0.0, turnDegrees);
                break;
        }
    }   //runContinuous

    //
    // Implements FtcMenu.MenuButtons interface.
    //

    @Override
    public boolean isMenuUpButton()
    {
        return gamepad1.dpad_up;
    }   //isMenuUpButton

    @Override
    public boolean isMenuDownButton()
    {
        return gamepad1.dpad_down;
    }   //isMenuDownButton

    @Override
    public boolean isMenuEnterButton()
    {
        return gamepad1.a;
    }   //isMenuEnterButton

    @Override
    public boolean isMenuBackButton()
    {
        return gamepad1.dpad_left;
    }   //isMenuBackButton

    private void doMenus()
    {
        FtcChoiceMenu testMenu = new FtcChoiceMenu("Tests:", null, this);
        FtcValueMenu driveTimeMenu = new FtcValueMenu("Drive time:", testMenu, this, 1.0, 10.0, 1.0, 4.0, " %.0f sec");
        FtcValueMenu driveDistanceMenu = new FtcValueMenu(
                "Drive distance:", testMenu, this, -10.0, 10.0, 0.5, 4.0, " %.1f ft");
        FtcValueMenu rangeDistanceMenu = new FtcValueMenu(
                "Range distance:", testMenu, this, 0.5, 12.0, 0.5, 6.0, " %.0f in");
        FtcValueMenu turnDegreesMenu = new FtcValueMenu(
                "Turn degrees:", testMenu, this, -360.0, 360.0, 5.0, 45.0, " %.0f deg");

        testMenu.addChoice("Sensors test", Test.SENSORS_TEST);
        testMenu.addChoice("Motors test", Test.MOTORS_TEST);
        testMenu.addChoice("Y Timed drive", Test.Y_TIMED_DRIVE, driveTimeMenu);
        testMenu.addChoice("X Timed drive", Test.X_TIMED_DRIVE, driveTimeMenu);
        testMenu.addChoice("Y Distance drive", Test.Y_DISTANCE_DRIVE, driveDistanceMenu);
        testMenu.addChoice("X Distance drive", Test.X_DISTANCE_DRIVE, driveDistanceMenu);
        testMenu.addChoice("Range drive", Test.RANGE_DRIVE, rangeDistanceMenu);
        testMenu.addChoice("Degrees turn", Test.GYRO_TURN, turnDegreesMenu);

        FtcMenu.walkMenuTree(testMenu);

        test = (Test)testMenu.getCurrentChoiceObject();
        driveTime = driveTimeMenu.getCurrentValue();
        driveDistance = driveDistanceMenu.getCurrentValue();
        rangeDistance = rangeDistanceMenu.getCurrentValue();
        turnDegrees = turnDegreesMenu.getCurrentValue();

        dashboard.displayPrintf(0, "Test: %s", testMenu.getCurrentChoiceText());
    }   //doMenus

    /**
     * This method reads all sensors and prints out their values. This is a very useful diagnostic tool to check
     * if all sensors are working properly. For encoders, since test sensor mode is also teleop mode, you can
     * operate the gamepads to turn the motors and check the corresponding encoder counts.
     */
    private void doSensorsTest()
    {
        final int LABEL_WIDTH = 100;
        //
        // Read all sensors and display on the dashboard.
        // Drive the robot around to sample different locations of the field.
        //
        dashboard.displayPrintf(3, LABEL_WIDTH, "FrontEnc: ", "l=%.0f,r=%.0f",
                                robot.leftFrontWheel.getPosition(), robot.rightFrontWheel.getPosition());
        dashboard.displayPrintf(4, LABEL_WIDTH, "RearEnc: ", "l=%.0f,r=%.0f",
                                robot.leftRearWheel.getPosition(), robot.rightRearWheel.getPosition());
        dashboard.displayPrintf(5, LABEL_WIDTH, "Shooter: ", "Pos=%.3f,touch=%s",
                                robot.shooter.getPosition(), Boolean.toString(robot.shooter.isTouchActive()));
        dashboard.displayPrintf(6, LABEL_WIDTH, "ServoPos: ", "BallGate=%.2f,Pusher=%.2f",
                                ballGatePos, buttonPusherPos);
        if (robot.USE_ANALOG_GYRO)
        {
            dashboard.displayPrintf(9, LABEL_WIDTH, "Gyro: ", "Rate=%.3f,Heading=%.1f",
                                    (Double)robot.analogGyro.getZRotationRate().value,
                                    (Double)robot.analogGyro.getZHeading().value);
        }
        else
        {
            dashboard.displayPrintf(9, LABEL_WIDTH, "Gyro: ", "Rate=%.1f,Heading=%.1f",
                                    (Double)robot.gyro.getZRotationRate().value,
                                    (Double)robot.gyro.getZHeading().value);
        }

        if (robot.USE_COLOR_SENSOR)
        {
            dashboard.displayPrintf(10, LABEL_WIDTH, "Beacon: ", "RGBAH=[%d,%d,%d,%d,%x]",
                                    robot.beaconColorSensor.red(), robot.beaconColorSensor.green(),
                                    robot.beaconColorSensor.blue(),
                                    robot.beaconColorSensor.alpha(), robot.beaconColorSensor.argb());
        }

        if (robot.USE_LINE_DETECTOR)
        {
            if (robot.USE_ODS_LINE_DETECTOR)
            {
                dashboard.displayPrintf(11, LABEL_WIDTH, "Line: ", "light=%.3f",
                                        (Double) robot.odsLineDetector.getRawData(
                                                0, TrcAnalogInput.DataType.INPUT_DATA).value);
            }
            else
            {
                dashboard.displayPrintf(11, LABEL_WIDTH, "Line: ", "color=%d,white=%d",
                                        (Integer) robot.lineDetectionSensor.getColorNumber().value,
                                        (Integer) robot.lineDetectionSensor.getWhiteValue().value);
            }
        }

        if (robot.USE_RANGE_SENSOR)
        {
            dashboard.displayPrintf(12, LABEL_WIDTH, "Range: ", "%.3f in",
                                    robot.rangeSensor.sensor.getDistance(DistanceUnit.INCH.INCH));
        }
    }   //doSensorsTest

    /**
     * This method runs each of the four wheels in sequence for a fixed number of seconds. It is for diagnosing
     * problems with the drive train. At the end of the run, you should check the amount of encoder counts each
     * wheel has accumulated. They should be about the same. If not, you need to check the problem wheel for
     * friction or chain tension etc. You can also use this test to check if a motor needs to be "inverted"
     * (i.e. turning in the wrong direction).
     */
    private void doMotorsTest()
    {
        dashboard.displayPrintf(9, "Motors Test: index=%d", motorIndex);
        dashboard.displayPrintf(10, "Enc: lf=%.0f, rf=%.0f",
                robot.leftFrontWheel.getPosition(), robot.rightFrontWheel.getPosition());
        dashboard.displayPrintf(11, "Enc: lr=%.0f, rr=%.0f",
                robot.leftRearWheel.getPosition(), robot.rightRearWheel.getPosition());

        if (sm.isReady())
        {
            State state = (State)sm.getState();
            switch (state)
            {
                case START:
                    //
                    // Spin a wheel for 5 seconds.
                    //
                    switch (motorIndex)
                    {
                        case 0:
                            robot.leftFrontWheel.setPower(0.5);
                            robot.rightFrontWheel.setPower(0.0);
                            robot.leftRearWheel.setPower(0.0);
                            robot.rightRearWheel.setPower(0.0);
                            break;

                        case 1:
                            robot.leftFrontWheel.setPower(0.0);
                            robot.rightFrontWheel.setPower(0.5);
                            robot.leftRearWheel.setPower(0.0);
                            robot.rightRearWheel.setPower(0.0);
                            break;

                        case 2:
                            robot.leftFrontWheel.setPower(0.0);
                            robot.rightFrontWheel.setPower(0.0);
                            robot.leftRearWheel.setPower(0.5);
                            robot.rightRearWheel.setPower(0.0);
                            break;

                        case 3:
                            robot.leftFrontWheel.setPower(0.0);
                            robot.rightFrontWheel.setPower(0.0);
                            robot.leftRearWheel.setPower(0.0);
                            robot.rightRearWheel.setPower(0.5);
                            break;
                    }
                    motorIndex = motorIndex + 1;
                    timer.set(5.0, event);
                    sm.waitForSingleEvent(event, motorIndex < 4? State.START: State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    robot.leftFrontWheel.setPower(0.0);
                    robot.rightFrontWheel.setPower(0.0);
                    robot.leftRearWheel.setPower(0.0);
                    robot.rightRearWheel.setPower(0.0);
                    sm.stop();
                    break;
            }
        }
    }   //doMotorsTest

    /**
     * This method runs the Drive Base with the given power for the given amount of time. This test is designed
     * to calibrate the encoder scaling factors. For example, if you drive the robot forward for 4 seconds, this
     * test will print out the average encoder count of all four wheels. Measure how far the robot has gone and
     * the encoder scaling factor can be calculated: INCHES_PER_COUNT = MeasuredDistanceInInches/AverageEncoderCount.
     * Note that to prevent any wheel slipping, use lower power values and runs longer time so the calculation will
     * be more accurate.
     *
     * @param xPower specifies motor power to move in the X direction.
     * @param yPower specifies motor power to move in the Y direction.
     * @param turnPower specifies motor power to turn.
     * @param time specifies how long in seconds should the robot run.
     */
    private void doTimedDrive(double xPower, double yPower, double turnPower, double time)
    {
        double lfEnc = robot.leftFrontWheel.getPosition();
        double rfEnc = robot.rightFrontWheel.getPosition();
        double lrEnc = robot.leftRearWheel.getPosition();
        double rrEnc = robot.rightRearWheel.getPosition();
        dashboard.displayPrintf(9, "Timed Drive: %.0f sec", time);
        dashboard.displayPrintf(10, "Enc:lf=%.0f,rf=%.0f", lfEnc, rfEnc);
        dashboard.displayPrintf(11, "Enc:lr=%.0f,rr=%.0f", lrEnc, rrEnc);
        dashboard.displayPrintf(12, "average=%f", (lfEnc + rfEnc + lrEnc + rrEnc)/4.0);
        dashboard.displayPrintf(13, "xPos=%.1f,yPos=%.1f,heading=%.1f",
                robot.driveBase.getXPosition(),
                robot.driveBase.getYPosition(),
                robot.driveBase.getHeading());

        if (sm.isReady())
        {
            State state = (State)sm.getState();
            switch (state)
            {
                case START:
                    //
                    // Drive the robot with the specified power and set a timer for the given time.
                    //
                    robot.driveBase.mecanumDrive_Cartesian(xPower, yPower, turnPower);
                    timer.set(time, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done, stop the robot.
                    //
                    robot.driveBase.stop();
                    sm.stop();
                    break;
            }
        }
    }   //doTimedDrive

    /**
     * This method drives the robot to the specified target distance. This test is designed for tuning PID constants.
     * This test can only be run after the proper encoder scaling factors have been determined. Run this test with
     * a specific distance (e.g. 96 inches). At the end of the run, check the dashboard display to see how far the
     * robot has actually gone and the corresponding error. Tuned the PID constants to make the error to go near zero
     * (error should be less than tolerance).
     *
     * @param xDistance specifies the target distance in inches in the X direction.
     * @param yDistance specifies the target distance in inches in the Y direction.
     * @param rotation specifies the rotation target in degrees.
     */
    private void doPidDrive(double xDistance, double yDistance, double rotation)
    {
        final String funcName = "doPidDrive";

        dashboard.displayPrintf(9, "xPos=%.1f,yPos=%.1f,heading=%.1f",
                robot.getInput(robot.encoderXPidCtrl),
                robot.getInput(robot.encoderYPidCtrl),
                robot.getInput(robot.gyroPidCtrl));
        robot.encoderXPidCtrl.displayPidInfo(10);
        robot.encoderYPidCtrl.displayPidInfo(12);
        robot.gyroPidCtrl.displayPidInfo(14);
        if (debugXPid || debugYPid || debugGyroPid || debugRangePid)
        {
            tracer.traceInfo("Battery", "Voltage=%5.2fV (%5.2fV)",
                             robot.battery.getCurrentVoltage(), robot.battery.getLowestVoltage());
        }
        if (debugXPid || xDistance != 0.0)
        {
            robot.encoderXPidCtrl.printPidInfo(tracer);
        }
        if (debugYPid || yDistance != 0.0)
        {
            robot.encoderYPidCtrl.printPidInfo(tracer);
        }
        if (debugGyroPid || rotation != 0.0)
        {
            robot.gyroPidCtrl.printPidInfo(tracer);
        }
        if (debugRangePid)
        {
            robot.rangePidCtrl.printPidInfo(tracer);
        }

        if (sm.isReady())
        {
            State state = (State)sm.getState();
            switch (state)
            {
                case START:
                    //
                    // Drive the given distance or degrees.
                    //
                    robot.battery.setTaskEnabled(true);
                    robot.pidDrive.setStallTimeout(0.0);
                    robot.setTurnPID(xDistance*12.0, yDistance*12.0, rotation);
                    robot.pidDrive.setTarget(xDistance*12.0, yDistance*12.0, rotation, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    robot.battery.setTaskEnabled(false);
                    sm.stop();
                    break;
            }
        }
    }   //doPidDrive

    /**
     * This method drives the robot to the specified distance to the wall using the Range Sensor. This test is
     * designed for tuning PID constants for the Range Sensor. Run this test with a specific distance to the wall
     * (e.g. 5 inches). At the end of the run, check the dashboard display to see how far the robot is from the wall
     * and the corresponding error. Tuned the PID constants to make the error to go near zero (error should be less
     * than tolerance).
     *
     * @param rangeDistance specifies the target distance in inches to the wall.
     */
    private void doRangeDrive(double rangeDistance)
    {
        final String funcName = "doRangeDrive";

        dashboard.displayPrintf(9, "xPos=%.1f,yPos=%.1f,heading=%.1f",
                                robot.getInput(robot.rangePidCtrl),
                                robot.getInput(robot.encoderYPidCtrl),
                                robot.getInput(robot.gyroPidCtrl));
        robot.rangePidCtrl.displayPidInfo(10);
        robot.encoderYPidCtrl.displayPidInfo(12);
        robot.gyroPidCtrl.displayPidInfo(14);
        if (debugGyroPid || debugRangePid)
        {
            tracer.traceInfo("Battery", "Voltage=%5.2f (%5.2f)",
                             robot.battery.getCurrentVoltage(), robot.battery.getLowestVoltage());
        }
        if (debugGyroPid)
        {
            robot.gyroPidCtrl.printPidInfo(tracer);
        }
        if (debugRangePid)
        {
            robot.rangePidCtrl.printPidInfo(tracer);
        }

        if (sm.isReady())
        {
            State state = (State) sm.getState();
            switch (state)
            {
                case START:
                    //
                    // Drive the to the given wall distance.
                    //
                    robot.battery.setTaskEnabled(true);
                    robot.setTurnPID(rangeDistance, 0.0, 0.0);
                    robot.rangePidDrive.setTarget(rangeDistance, 0.0, 0.0, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    robot.battery.setTaskEnabled(false);
                    sm.stop();
                    break;
            }
        }
    }   //doRangeDrive

    //
    // Implements FtcGamepad.ButtonHandler interface.
    //

    @Override
    public void gamepadButtonEvent(FtcGamepad gamepad, int button, boolean pressed)
    {
        boolean processed = false;

        dashboard.displayPrintf(7, "%s: %04x->%s",
                gamepad.toString(), button, pressed? "Pressed": "Released");

        if (gamepad == driverGamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_DPAD_UP:
                    if (pressed)
                    {
                        ballGatePos += 0.01;
                        if (ballGatePos > 1.0) ballGatePos = 1.0;
                        robot.shooter.setBallGatePosition(ballGatePos);
                    }
                    processed = true;
                    break;

                case FtcGamepad.GAMEPAD_DPAD_DOWN:
                    if (pressed)
                    {
                        ballGatePos -= 0.01;
                        if (ballGatePos < 0.0) ballGatePos = 0.0;
                        robot.shooter.setBallGatePosition(ballGatePos);
                    }
                    processed = true;
                    break;

                case FtcGamepad.GAMEPAD_DPAD_LEFT:
                    if (pressed)
                    {
                        buttonPusherPos -= 0.05;
                        if (buttonPusherPos < 0.0) buttonPusherPos = 0.0;
                        robot.leftButtonPusher.setPosition(buttonPusherPos);
                        robot.rightButtonPusher.setPosition(buttonPusherPos);
                    }
                    processed = true;
                    break;

                case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                    if (pressed)
                    {
                        buttonPusherPos += 0.05;
                        if (buttonPusherPos > 1.0) buttonPusherPos = 1.0;
                        robot.leftButtonPusher.setPosition(buttonPusherPos);
                        robot.rightButtonPusher.setPosition(buttonPusherPos);
                    }
                    processed = true;
                    break;
            }
        }
        else if (gamepad == operatorGamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    break;

                case FtcGamepad.GAMEPAD_X:
                    break;

                case FtcGamepad.GAMEPAD_B:
                    break;

                case FtcGamepad.GAMEPAD_LBUMPER:
                    break;

                case FtcGamepad.GAMEPAD_RBUMPER:
                    break;

                case FtcGamepad.GAMEPAD_BACK:
                    break;

                case FtcGamepad.GAMEPAD_START:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_UP:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_DOWN:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_LEFT:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                    break;
            }
        }

        if (!processed)
        {
            super.gamepadButtonEvent(gamepad, button, pressed);
        }
    }   //gamepadButtonEvent

}   //class FtcTest
