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

    private enum ConveyorMode
    {
        CONVEYOR_OFF,
        CONVEYOR_FORWARD,
        CONVEYOR_REVERSE
    }   //enum ConveyorMode

    protected HalDashboard dashboard;
    protected Robot robot;

    FtcGamepad driverGamepad;
    private FtcGamepad operatorGamepad;

    private double drivePowerScale = 1.0;
    private boolean invertedDrive = false;
    private DriveMode driveMode = DriveMode.MECANUM_MODE;
    private ConveyorMode conveyorMode = ConveyorMode.CONVEYOR_OFF;

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
                double leftPower = driverGamepad.getLeftStickY(true)*drivePowerScale;
                double rightPower = driverGamepad.getRightStickY(true)*drivePowerScale;
                robot.driveBase.tankDrive(leftPower, rightPower, invertedDrive);
                dashboard.displayPrintf(1, "Tank:left=%.2f,right=%.2f,inverted=%s",
                                        leftPower, rightPower, Boolean.toString(invertedDrive));
                break;

            case MECANUM_MODE:
                double x = driverGamepad.getLeftStickX(true)*drivePowerScale;
                double y = driverGamepad.getRightStickY(true)*drivePowerScale;
                double rot = (driverGamepad.getRightTrigger(true) - driverGamepad.getLeftTrigger(true))*drivePowerScale;
                robot.driveBase.mecanumDrive_Cartesian(x, y, rot, invertedDrive);
                dashboard.displayPrintf(1, "Mecanum:x=%.2f,y=%.2f,rot=%.2f,inverted=%s",
                                        x, y, rot, Boolean.toString(invertedDrive));
                break;
        }
        dashboard.displayPrintf(2, "xPos=%.2f,yPos=%.2f,heading=%.2f",
                                robot.driveBase.getXPosition(), robot.driveBase.getYPosition(),
                                robot.driveBase.getHeading());
    }   //runPeriodic

    //
    // Implements FtcGamepad.ButtonHandler interface.
    //

    @Override
    public void gamepadButtonEvent(FtcGamepad gamepad, int button, boolean pressed)
    {
        dashboard.displayPrintf(7, "%s: %04x->%s", gamepad.toString(), button, pressed? "Pressed": "Released");
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

                case FtcGamepad.GAMEPAD_LBUMPER:
                    drivePowerScale = pressed? 0.5: 1.0;
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
                    //
                    // Load particle.
                    //
                    robot.shooter.setBallGatePosition(
                            pressed? RobotInfo.BALLGATE_UP_POSITION : RobotInfo.BALLGATE_DOWN_POSITION);
                    break;

                case FtcGamepad.GAMEPAD_B:
                    //
                    // Manual firing.
                    //
                    robot.shooter.setPower(pressed? RobotInfo.SHOOTER_POWER: 0.0);
                    break;

                case FtcGamepad.GAMEPAD_X:
                    //
                    // Load particle, arm and fire the particle.
                    // This is basically combining button A and B.
                    //
                    if (pressed)
                    {
                        robot.shooter.loadAndFireOneShot();
                    }
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    //
                    // Arm and fire the particle.
                    //
                    if (pressed)
                    {
                        robot.shooter.fireOneShot();
                    }
                    break;

                case FtcGamepad.GAMEPAD_LBUMPER:
                    robot.leftButtonPusher.setPosition(
                            pressed? RobotInfo.BUTTON_PUSHER_EXTEND_POSITION: RobotInfo.BUTTON_PUSHER_RETRACT_POSITION);
                    break;

                case FtcGamepad.GAMEPAD_RBUMPER:
                    robot.rightButtonPusher.setPosition(
                            pressed? RobotInfo.BUTTON_PUSHER_EXTEND_POSITION: RobotInfo.BUTTON_PUSHER_RETRACT_POSITION);
                    break;

                case FtcGamepad.GAMEPAD_BACK:
                    break;

                case FtcGamepad.GAMEPAD_START:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_UP:
                    if (pressed)
                    {
                        conveyorMode = conveyorMode == ConveyorMode.CONVEYOR_FORWARD?
                                ConveyorMode.CONVEYOR_OFF: ConveyorMode.CONVEYOR_FORWARD;
                        if (conveyorMode == ConveyorMode.CONVEYOR_FORWARD)
                        {
                            robot.ballPickUp.setPower(RobotInfo.BALL_PICKUP_MOTOR_POWER);
                            robot.conveyor.setPower(RobotInfo.CONVEYOR_MOTOR_POWER);
                        }
                        else
                        {
                            robot.ballPickUp.setPower(0.0);
                            robot.conveyor.setPower(0.0);
                        }
                    }
                    break;

                case FtcGamepad.GAMEPAD_DPAD_DOWN:
                    if (pressed)
                    {
                        conveyorMode = conveyorMode == ConveyorMode.CONVEYOR_REVERSE?
                                ConveyorMode.CONVEYOR_OFF: ConveyorMode.CONVEYOR_REVERSE;
                        if (conveyorMode == ConveyorMode.CONVEYOR_REVERSE)
                        {
                            robot.ballPickUp.setPower(-RobotInfo.BALL_PICKUP_MOTOR_POWER);
                            robot.conveyor.setPower(-RobotInfo.CONVEYOR_MOTOR_POWER);
                        }
                        else
                        {
                            robot.ballPickUp.setPower(0.0);
                            robot.conveyor.setPower(0.0);
                        }
                    }
                    break;

                case FtcGamepad.GAMEPAD_DPAD_LEFT:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                    /*
                    if (pressed)
                    {
                        robot.shooter.fireContinuous(true);
                    }
                    */
                    break;
            }
        }
    }   //gamepadButtonEvent

}   //class FtcTeleOp
