package team3543;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftclib.FtcChoiceMenu;
import ftclib.FtcGamepad;
import ftclib.FtcMenu;
import ftclib.FtcValueMenu;
import trclib.TrcEvent;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

@TeleOp(name="Test", group="3543Test")
public class FtcTest extends FtcTeleOp implements FtcMenu.MenuButtons, FtcGamepad.ButtonHandler
{
    private enum Test
    {
        SENSORS_TEST,
        MOTORS_TEST,
        Y_TIMED_DRIVE,
        X_TIMED_DRIVE,
        Y_DISTANCE_DRIVE,
        X_DISTANCE_DRIVE,
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
    private double turnDegrees = 0.0;

    private int motorIndex = 0;
    private double ballGatePos = RobotInfo.BALLGATE_CLOSE_POSITION;
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
                doTimedDrive(0.2, 0.0, 0.0, driveTime);
                break;

            case Y_DISTANCE_DRIVE:
                doPidDrive(0.0, driveDistance, 0.0);
                break;

            case X_DISTANCE_DRIVE:
                doPidDrive(driveDistance, 0.0, 0.0);
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
        FtcValueMenu driveTimeMenu = new FtcValueMenu("Drive time:", testMenu, this,
                                                      1.0, 10.0, 1.0, 8.0, " %.0f sec");
        FtcValueMenu driveDistanceMenu = new FtcValueMenu("Drive distance:", testMenu, this,
                                                          1.0, 10.0, 1.0, 8.0, " %.0f ft");
        FtcValueMenu turnDegreesMenu =
                new FtcValueMenu("Turn degrees:", testMenu, this,
                                 -360.0, 360.0, 90.0, 360.0, " %.0f deg");

        testMenu.addChoice("Sensors test", Test.SENSORS_TEST);
        testMenu.addChoice("Motors test", Test.MOTORS_TEST);
        testMenu.addChoice("Y Timed drive", Test.Y_TIMED_DRIVE, driveTimeMenu);
        testMenu.addChoice("X Timed drive", Test.X_TIMED_DRIVE, driveTimeMenu);
        testMenu.addChoice("Y Distance drive", Test.Y_DISTANCE_DRIVE, driveDistanceMenu);
        testMenu.addChoice("X Distance drive", Test.X_DISTANCE_DRIVE, driveDistanceMenu);
        testMenu.addChoice("Degrees turn", Test.GYRO_TURN, turnDegreesMenu);

        FtcMenu.walkMenuTree(testMenu);

        test = (Test)testMenu.getCurrentChoiceObject();
        driveTime = driveTimeMenu.getCurrentValue();
        driveDistance = driveDistanceMenu.getCurrentValue();
        turnDegrees = turnDegreesMenu.getCurrentValue();

        dashboard.displayPrintf(0, "Test: %s", testMenu.getCurrentChoiceText());
    }   //doMenus

    private void doSensorsTest()
    {
        final int LABEL_WIDTH = 100;
        //
        // Read all sensors and display on the dashboard.
        // Drive the robot around to sample different locations of the field.
        //
        dashboard.displayPrintf(9, LABEL_WIDTH, "FrontEnc: ", "l=%.0f,r=%.0f",
                                robot.leftFrontWheel.getPosition(),
                                robot.rightFrontWheel.getPosition());
        dashboard.displayPrintf(10, LABEL_WIDTH, "RearEnc: ", "l=%.0f,r=%.0f",
                                robot.leftRearWheel.getPosition(),
                                robot.rightRearWheel.getPosition());
        dashboard.displayPrintf(11, LABEL_WIDTH, "Gyro: ", "Rate=%.1f,Heading=%.1f",
                                robot.gyro.getZRotationRate().value,
                                robot.gyro.getZHeading().value);
        dashboard.displayPrintf(12, LABEL_WIDTH, "Shooter: ", "Pos=%.3f,Speed=%.3f",
                                robot.shooter.getPosition(), robot.shooter.getSpeed());
        dashboard.displayPrintf(13, LABEL_WIDTH, "Beacon: ", "RGBAH=[%d,%d,%d,%d,%x]",
                                robot.beaconColorSensor.red(),
                                robot.beaconColorSensor.green(),
                                robot.beaconColorSensor.blue(),
                                robot.beaconColorSensor.alpha(),
                                robot.beaconColorSensor.argb());
        /*
        dashboard.displayPrintf(14, LABEL_WIDTH, "Line: ", "color=%d,white=%d",
                                (Integer)robot.lineDetectionSensor.getColorNumber().value,
                                (Integer)robot.lineDetectionSensor.getWhiteValue().value);
                */
        dashboard.displayPrintf(15, LABEL_WIDTH, "ServoPos: ", "BallGate=%.2f,Pusher=%.2f",
                                ballGatePos, buttonPusherPos);
    }   //doSensorsTest

    private void doMotorsTest()
    {
        dashboard.displayPrintf(9, "Motors Test: index=%d", motorIndex);
        dashboard.displayPrintf(10, "Enc: lf=%.0f, rf=%.0f",
                                robot.leftFrontWheel.getPosition(),
                                robot.rightFrontWheel.getPosition());
        dashboard.displayPrintf(11, "Enc: lr=%.0f, rr=%.0f",
                                robot.leftRearWheel.getPosition(),
                                robot.rightRearWheel.getPosition());

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
                    sm.addEvent(event);
                    sm.waitForEvents(motorIndex < 4? State.START: State.DONE);
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
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
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

    private void doPidDrive(double xDistance, double yDistance, double rotation)
    {
        dashboard.displayPrintf(9, "xPos=%.1f,yPos=%.1f,heading=%.1f",
                                robot.driveBase.getXPosition(),
                                robot.driveBase.getYPosition(),
                                robot.driveBase.getHeading());
        robot.encoderXPidCtrl.displayPidInfo(10);
        robot.encoderYPidCtrl.displayPidInfo(12);
        robot.gyroPidCtrl.displayPidInfo(14);

        if (sm.isReady())
        {
            State state = (State)sm.getState();
            switch (state)
            {
                case START:
                    //
                    // Drive the given Y distance.
                    //
                    robot.pidDrive.setTarget(xDistance*12.0, yDistance*12.0, rotation, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    sm.stop();
                    break;
            }
        }
    }   //doPidDrive

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
                        robot.ballGate.setPosition(ballGatePos);
                    }
                    processed = true;
                    break;

                case FtcGamepad.GAMEPAD_DPAD_DOWN:
                    if (pressed)
                    {
                        ballGatePos -= 0.01;
                        if (ballGatePos < 0.0) ballGatePos = 0.0;
                        robot.ballGate.setPosition(ballGatePos);
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
