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
        MECANUM_MODE,
    }   //enum DriveMode

    protected HalDashboard dashboard;
    protected Robot robot;

    protected FtcGamepad driverGamepad;
    protected FtcGamepad operatorGamepad;

    private boolean invertedDrive = false;
    private DriveMode driveMode = DriveMode.MECANUM_MODE;
    private boolean conveyorOn = false;
    private boolean ballGateOpened = false;
    private boolean leftPusherExtended = false;
    private boolean rightPusherExtended = false;

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
        switch(driveMode)
        {
            case TANK_MODE:
                double leftPower = driverGamepad.getLeftStickY(true);
                double rightPower = driverGamepad.getRightStickY(true);
                robot.driveBase.tankDrive(leftPower, rightPower, invertedDrive);
                dashboard.displayPrintf(1, "Tank:left=%.2f,right=%.2f,inverted=%s",
                        leftPower, rightPower, Boolean.toString(invertedDrive));
                break;

            case MECANUM_MODE:
                double x = driverGamepad.getLeftStickX(true);
                double y = driverGamepad.getRightStickY(true);
                double rot = driverGamepad.getRightTrigger(true) - driverGamepad.getLeftTrigger(true);
                robot.driveBase.mecanumDrive_Cartesian(x, y, rot, invertedDrive);
                dashboard.displayPrintf(1, "Mecanum:x=%.2f,y=%.2f,rot=%.2f,inverted=%s",
                        x, y, rot, Boolean.toString(invertedDrive));
                break;
        }
        dashboard.displayPrintf(2, "xPos=%.2f,yPos=%.2f,heading=%.2f",
                robot.driveBase.getXPosition(), robot.driveBase.getYPosition(), robot.driveBase.getHeading());
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
                        driveMode = DriveMode.MECANUM_MODE;
                    break;

                case FtcGamepad.GAMEPAD_B:
                    if (pressed)
                        driveMode = DriveMode.TANK_MODE;
                    break;

                case FtcGamepad.GAMEPAD_X:
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
                        robot.shooter.fireOneShot();
                    break;

                case FtcGamepad.GAMEPAD_B:
                    if (pressed)
                        robot.shooter.setPower(RobotInfo.SHOOTER_POWER);
                    else
                        robot.shooter.setPower(0.0);
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    if (pressed)
                    {
                        ballGateOpened = !ballGateOpened;
                        robot.shooter.setBallGatePosition(
                                ballGateOpened? RobotInfo.BALLGATE_OPEN_POSITION: RobotInfo.BALLGATE_CLOSE_POSITION);
                    }
                    break;

                case FtcGamepad.GAMEPAD_X:
                    if (pressed)
                    {
                        conveyorOn = !conveyorOn;
                        robot.ballPickUp.setPower(conveyorOn? RobotInfo.BALL_PICKUP_MOTOR_POWER: 0.0);
                        robot.conveyor.setPower(conveyorOn? RobotInfo.CONVEYOR_MOTOR_POWER: 0.0);
                    }
                    break;

                case FtcGamepad.GAMEPAD_LBUMPER:
                    if (pressed)
                    {
                        leftPusherExtended = !leftPusherExtended;
                        robot.leftButtonPusher.setPosition(
                                leftPusherExtended? RobotInfo.BUTTON_PUSHER_EXTEND_POSITION:
                                                    RobotInfo.BUTTON_PUSHER_RETRACT_POSITION);
                    }
                    break;

                case FtcGamepad.GAMEPAD_RBUMPER:
                    if (pressed)
                    {
                        rightPusherExtended = !rightPusherExtended;
                        robot.rightButtonPusher.setPosition(
                                rightPusherExtended? RobotInfo.BUTTON_PUSHER_EXTEND_POSITION:
                                                     RobotInfo.BUTTON_PUSHER_RETRACT_POSITION);
                    }
                    break;

                case FtcGamepad.GAMEPAD_BACK:
                    break;

                case FtcGamepad.GAMEPAD_START:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_UP:
                    if (pressed)
                        robot.shooter.fireContinuous(true);
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
