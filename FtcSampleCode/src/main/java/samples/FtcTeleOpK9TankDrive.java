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

package samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import ftclib.FtcGamepad;
import ftclib.FtcOpMode;
import trclib.TrcRobot;

@TeleOp(name="TeleOp: K9Bot Tank Drive", group="3543TeleOpSamples")
@Disabled
public class FtcTeleOpK9TankDrive extends FtcOpMode implements FtcGamepad.ButtonHandler
{
    private K9Robot robot;
    private FtcGamepad gamepad;

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void initRobot()
    {
        robot = new K9Robot(TrcRobot.RunMode.TELEOP_MODE);
        //
        // Initializing Gamepads.
        //
        gamepad = new FtcGamepad("Gamepad", gamepad1, this);
        gamepad.setYInverted(true);
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        robot.startMode(TrcRobot.RunMode.TELEOP_MODE);
        //
        // There is an issue with the gamepad objects that may not be valid
        // before waitForStart() is called. So we call the setGamepad method
        // here to update their references in case they have changed.
        //
        gamepad.setGamepad(gamepad1);
    }   //startMode

    @Override
    public void stopMode()
    {
        robot.stopMode(TrcRobot.RunMode.TELEOP_MODE);
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        final int LABEL_WIDTH = 200;
        //
        // DriveBase subsystem.
        //
        double left = gamepad.getLeftStickY(true);
        double right = gamepad.getRightStickY(true);
        robot.driveBase.tankDrive(left, right);

        robot.dashboard.displayPrintf(1, LABEL_WIDTH, "Text: ", "*** Robot Data ***");
        robot.dashboard.displayPrintf(2, LABEL_WIDTH, "arm: ", "%.2f", robot.armServo.getPosition());
        robot.dashboard.displayPrintf(3, LABEL_WIDTH, "claw: ", "%.2f", robot.clawServo.getPosition());
        robot.dashboard.displayPrintf(4, LABEL_WIDTH, "left power: ", "%.2f", left);
        robot.dashboard.displayPrintf(5, LABEL_WIDTH, "right power: ", "%.2f", right);
    }   //runPeriodic

    //
    // Implements FtcGamepad.ButtonHandler interface.
    //

    @Override
    public void gamepadButtonEvent(FtcGamepad gamepad, int button, boolean pressed)
    {
        if (gamepad == this.gamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    if (pressed)
                    {
                        robot.arm.setPosition(K9Robot.ARM_MAX_RANGE, K9Robot.SERVO_STEPRATE);
                    }
                    else
                    {
                        robot.arm.stop();
                    }
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    if (pressed)
                    {
                        robot.arm.setPosition(K9Robot.ARM_MIN_RANGE, K9Robot.SERVO_STEPRATE);
                    }
                    else
                    {
                        robot.arm.stop();
                    }
                    break;

                case FtcGamepad.GAMEPAD_X:
                    if (pressed)
                    {
                        robot.claw.setPosition(K9Robot.CLAW_MAX_RANGE, K9Robot.SERVO_STEPRATE);
                    }
                    else
                    {
                        robot.claw.stop();
                    }
                    break;

                case FtcGamepad.GAMEPAD_B:
                    if (pressed)
                    {
                        robot.claw.setPosition(K9Robot.CLAW_MIN_RANGE, K9Robot.SERVO_STEPRATE);
                    }
                    else
                    {
                        robot.claw.stop();
                    }
                    break;
            }
        }
    }   //gamepadButtonEvent

}   //class FtcTeleOpK9TankDrive
