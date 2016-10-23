package team3543;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftclib.FtcGamepad;
import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcRobot;

@TeleOp(name="TeleOp", group="3543TeleOp")
public class FtcTeleOp extends FtcOpMode implements FtcGamepad.ButtonHandler
{
    private enum DriveMode
    {
        TANK_MODE,
        MECANUM_MODE_ONE_STICK,
        MECANUM_MODE_TWO_STICKS
    }   //enum DriveMode

    protected HalDashboard dashboard;
    protected Robot robot;

    private FtcGamepad driverGamepad;
    private FtcGamepad operatorGamepad;

    private boolean invertedDrive = false;
    private DriveMode   driveMode = DriveMode.MECANUM_MODE_TWO_STICKS;
    //
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void initRobot()
    {
        //
        // Initializing robot objects.
        //
        robot = new Robot(TrcRobot.RunMode.TELEOP_MODE);
        dashboard = robot.dashboard;
        //
        // Initializing Gamepads.
        //
        driverGamepad = new FtcGamepad("DriverGamepad", gamepad1, this);
        operatorGamepad = new FtcGamepad("OperatorGamepad", gamepad2, this);
        driverGamepad.setYInverted(true);
        operatorGamepad.setYInverted(true);
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        dashboard.clearDisplay();
        robot.startMode(TrcRobot.RunMode.TELEOP_MODE);
        //
        // There is an issue with the gamepad objects that may not be valid
        // before waitForStart() is called. So we call the setGamepad method
        // here to update their references in case they have changed.
        //
        driverGamepad.setGamepad(gamepad1);
        operatorGamepad.setGamepad(gamepad2);
    }   //startMode

    @Override
    public void stopMode()
    {
        robot.stopMode(TrcRobot.RunMode.TELEOP_MODE);
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        //
        // DriveBase subsystem.
        //
        double x=0.0,y=0.0,rotation=0.0;

        switch(driveMode)
        {
            case TANK_MODE:
            {
                x = driverGamepad.getLeftStickY(true);
                y = driverGamepad.getRightStickY(true);
                break;
            }
            case MECANUM_MODE_ONE_STICK:
            {
                x = driverGamepad.getLeftStickX(true);
                y = driverGamepad.getLeftStickY(true);
                rotation = driverGamepad.getRightTrigger(true)-driverGamepad.getLeftTrigger(true);
                break;
            }
            case MECANUM_MODE_TWO_STICKS:
            default:
            {
                x = driverGamepad.getRightStickX(true);
                y = driverGamepad.getLeftStickY(true);
                rotation = driverGamepad.getRightTrigger(true)-driverGamepad.getLeftTrigger(true);
            }

        }
        if (driveMode != DriveMode.TANK_MODE)
            robot.driveBase.mecanumDrive_Cartesian(x,y,rotation);
        else
            robot.driveBase.tankDrive(x,y);

        dashboard.displayPrintf(1, "mode=%s,x=%.2f,y=%.2f,rot=%.2f",driveMode.toString(),x,y,rotation);
        dashboard.displayPrintf(2, "yPos=%.2f,heading=%.2f",
                                robot.driveBase.getYPosition(), robot.driveBase.getHeading());
    }   //runPeriodic

    //
    // Implements FtcGamepad.ButtonHandler interface.
    //

    @Override
    public void gamepadButtonEvent(FtcGamepad gamepad, int button, boolean pressed)
    {
        dashboard.displayPrintf(7, "%s: %04x->%s",
                gamepad.toString(), button, pressed? "Pressed": "Released");
        if (gamepad == driverGamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    if (pressed)
                        driveMode = DriveMode.MECANUM_MODE_ONE_STICK;
                    break;

                case FtcGamepad.GAMEPAD_B:
                    if (pressed)
                        driveMode = DriveMode.MECANUM_MODE_TWO_STICKS;
                    break;

                case FtcGamepad.GAMEPAD_X:
                    if (pressed)
                        driveMode = DriveMode.TANK_MODE;
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    break;

                case FtcGamepad.GAMEPAD_RBUMPER:
                    invertedDrive = pressed;
                    break;
            }
        }
        else if (gamepad == operatorGamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    if (pressed)
                        robot.partAccel.openFire(false);
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    if (pressed) {
                        robot.ballPickUp.stopPickUp();
                        robot.conveyer.stopConveyer();
                    }
                    break;

                case FtcGamepad.GAMEPAD_X:
                    if (pressed) {
                        robot.ballPickUp.startPickUp();
                        robot.conveyer.startConveyer();
                    }
                    break;

                case FtcGamepad.GAMEPAD_B:
                    if (pressed)
                        robot.partAccel.stopFire();
                    break;

                case FtcGamepad.GAMEPAD_LBUMPER:
                    if (pressed)
                        robot.leftPusher.extend();
                    else
                        robot.leftPusher.retract();
                    break;

                case FtcGamepad.GAMEPAD_RBUMPER:
                    if (pressed)
                        robot.rightPusher.extend();
                    else
                        robot.rightPusher.retract();
                    break;

                case FtcGamepad.GAMEPAD_START:
                    if (pressed)
                        robot.conveyer.openGate();
                    else
                        robot.conveyer.closeGate();
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
    }   //gamepadButtonEvent

}   //class FtcTeleOp
