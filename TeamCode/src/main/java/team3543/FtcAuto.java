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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Date;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import ftclib.FtcValueMenu;
import trclib.TrcDbgTrace;
import trclib.TrcRobot;

@Autonomous(name="Autonomous", group="3543Auto")
public class FtcAuto extends FtcOpMode implements FtcMenu.MenuButtons
{
    public enum Alliance
    {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance

    public enum Strategy
    {
        AUTO_100_1,
        AUTO_100_2,
        AUTO_40_NEAR,
        AUTO_40_FAR,
        DISTANCE_DRIVE,
        TIMED_DRIVE,
        DO_NOTHING
    }   //enum Strategy

    public enum ParkOption
    {
        DO_NOTHING,
        PARK_CORNER,
        PARK_CENTER
    }   //enum ParkOption

    private static final String moduleName = "FtcAuto";

    private TrcDbgTrace tracer = FtcOpMode.getGlobalTracer();
    private Robot robot;

    private TrcRobot.RobotCommand autoCommand = null;
    private Alliance alliance = Alliance.RED_ALLIANCE;
    private double delay = 0.0;
    private int numParticles = 2;
    private ParkOption parkOption = ParkOption.PARK_CORNER;
    private Strategy strategy = Strategy.AUTO_100_1;
    private int beaconButtons = 2;
    private double driveDistance = 0.0;
    private double driveTime = 0.0;
    private double drivePower = 0.0;

    //
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void initRobot()
    {
        //
        // Initializing robot objects.
        //
        robot = new Robot(TrcRobot.RunMode.AUTO_MODE);
        //
        // Choice menus.
        //
        doMenus();
        //
        // Strategies.
        //
        switch (strategy)
        {
            case AUTO_100_1:
                autoCommand = new CmdAuto100(robot, alliance, delay, numParticles, parkOption, beaconButtons, true);
                break;

            case AUTO_100_2:
                autoCommand = new CmdAuto100(robot, alliance, delay, numParticles, parkOption, beaconButtons, false);
                break;

            case AUTO_40_NEAR:
                autoCommand = new CmdAuto40(robot, alliance, delay, numParticles, parkOption, true);
                break;

            case AUTO_40_FAR:
                autoCommand = new CmdAuto40(robot, alliance, delay, numParticles, parkOption, false);
                break;

            case DISTANCE_DRIVE:
                autoCommand = new CmdPidDrive(robot, delay, 0.0, driveDistance*12.0, 0.0);
                break;

            case TIMED_DRIVE:
                autoCommand = new CmdTimedDrive(robot, delay, driveTime, 0.0, drivePower, 0.0);
                break;

            case DO_NOTHING:
            default:
                autoCommand = null;
                break;
        }
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        robot.startMode(TrcRobot.RunMode.AUTO_MODE);
        robot.battery.setEnabled(true);
        Date date = new Date();
        tracer.traceInfo(moduleName, "%s: ***** Starting autonomous *****", date.toString());
        robot.dashboard.clearDisplay();
    }   //startMode

    @Override
    public void stopMode()
    {
        robot.stopMode(TrcRobot.RunMode.AUTO_MODE);
        robot.battery.setEnabled(false);
    }   //stopMode

    @Override
    public void runContinuous(double elapsedTime)
    {
        if (autoCommand != null)
        {
            autoCommand.cmdPeriodic(elapsedTime);
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
        //
        // Create menus.
        //
        FtcChoiceMenu<Alliance> allianceMenu = new FtcChoiceMenu<>("Alliance:", null, this);
        FtcValueMenu delayMenu = new FtcValueMenu("Delay time:", allianceMenu, this, 0.0, 15.0, 1.0, 0.0, " %.0f sec");
        FtcValueMenu numParticlesMenu = new FtcValueMenu(
                "Shoot particles:", delayMenu, this, 0.0, 2.0, 1.0, 2.0, " %.0f");
        FtcChoiceMenu<ParkOption> parkOptionMenu = new FtcChoiceMenu<>("Park options:", numParticlesMenu, this);
        FtcChoiceMenu<Strategy> strategyMenu = new FtcChoiceMenu<>("Strategies:", parkOptionMenu, this);
        FtcValueMenu beaconButtonsMenu = new FtcValueMenu(
                "Push beacon buttons:", strategyMenu, this, 0.0, 2.0, 1.0, 2.0, " %.0f");
        FtcValueMenu driveDistanceMenu = new FtcValueMenu(
                "Distance:", strategyMenu, this, -10.0, 10.0, 0.5, 5.0, " %.0f ft");
        FtcValueMenu driveTimeMenu = new FtcValueMenu(
                "Drive time:", strategyMenu, this, 0.0, 15.0, 1.0, 5.0, " %.0f sec");
        FtcValueMenu drivePowerMenu = new FtcValueMenu(
                "Drive power:", driveTimeMenu, this, -1.0, 1.0, 0.1, 0.5, " %.1f");
        //
        // Populate menus.
        //
        delayMenu.setChildMenu(numParticlesMenu);
        numParticlesMenu.setChildMenu(parkOptionMenu);
        driveTimeMenu.setChildMenu(drivePowerMenu);

        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE, delayMenu);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE, delayMenu);

        parkOptionMenu.addChoice("Do nothing", ParkOption.DO_NOTHING, strategyMenu);
        parkOptionMenu.addChoice("Park center", ParkOption.PARK_CENTER, strategyMenu);
        parkOptionMenu.addChoice("Park corner", ParkOption.PARK_CORNER, strategyMenu);

        strategyMenu.addChoice("Auto 100pt 1", Strategy.AUTO_100_1, beaconButtonsMenu);
        strategyMenu.addChoice("Auto 100pt 2", Strategy.AUTO_100_2, beaconButtonsMenu);
        strategyMenu.addChoice("Auto 40pt near", Strategy.AUTO_40_NEAR);
        strategyMenu.addChoice("Auto 40pt far", Strategy.AUTO_40_FAR);
        strategyMenu.addChoice("Distance Drive", Strategy.DISTANCE_DRIVE, driveDistanceMenu);
        strategyMenu.addChoice("Timed Drive", Strategy.TIMED_DRIVE, driveTimeMenu);
        strategyMenu.addChoice("Do nothing", Strategy.DO_NOTHING);
        //
        // Traverse menus.
        //
        FtcMenu.walkMenuTree(allianceMenu);
        //
        // Fetch choices.
        //
        alliance = allianceMenu.getCurrentChoiceObject();
        delay = delayMenu.getCurrentValue();
        numParticles = (int)numParticlesMenu.getCurrentValue();
        parkOption = parkOptionMenu.getCurrentChoiceObject();
        strategy = strategyMenu.getCurrentChoiceObject();
        beaconButtons = (int)beaconButtonsMenu.getCurrentValue();
        driveDistance = driveDistanceMenu.getCurrentValue();
        driveTime = driveTimeMenu.getCurrentValue();
        drivePower = drivePowerMenu.getCurrentValue();
        //
        // Show choices.
        //
        robot.dashboard.displayPrintf(1, "-- Auto Strategy: %s --", strategyMenu.getCurrentChoiceText());
        robot.dashboard.displayPrintf(2, "Alliance=%s,Delay=%.0f sec", alliance.toString(), delay);
        robot.dashboard.displayPrintf(3, "Auto100: NumParticles=%d,BeaconButtons=%d", numParticles, beaconButtons);
        robot.dashboard.displayPrintf(4, "Auto40: ParkOption=%s", parkOption.toString());
        robot.dashboard.displayPrintf(5, "Drive: distance=%.0f ft,Time=%.0f,Power=%.1f",
                                      driveDistance, driveTime, drivePower);
    }   //doMenus

}   //class FtcAuto
