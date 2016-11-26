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

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import ftclib.FtcValueMenu;
import hallib.HalDashboard;
import trclib.TrcRobot;

@Autonomous(name="Autonomous", group="3543Auto")
public class FtcAuto extends FtcOpMode implements FtcMenu.MenuButtons
{
    public enum Alliance
    {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance

    public enum StartPosition
    {
        NEAR,
        FAR
    }   //enum StartPosition

    public enum Strategy
    {
        DO_NOTHING,
        BEACON,
        DEFENSE
    }   //enum Strategy

    public enum ParkOption
    {
        DO_NOTHING,
        PARK_CORNER,
        PARK_CENTER
    }   //enum ParkOption

    private HalDashboard dashboard;
    private Robot robot;

    private TrcRobot.AutoStrategy autoStrategy = null;
    private Alliance alliance = Alliance.RED_ALLIANCE;
    private StartPosition startPos = StartPosition.NEAR;
    private double delay = 0.0;
    private Strategy strategy = Strategy.BEACON;
    private int shootParticles = 2;
    private int beaconButtons = 2;
    private double driveDistance = 0.0;
    private ParkOption parkOption = ParkOption.DO_NOTHING;

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
        dashboard = robot.dashboard;
        //
        // Choice menus.
        //
        doMenus();
        //
        // Strategies.
        //
        switch (strategy)
        {
            case BEACON:
                autoStrategy = new AutoBeacon(
                        robot, alliance, startPos, delay, shootParticles, beaconButtons, parkOption);
                break;

            case DEFENSE:
                autoStrategy = new AutoDefense(robot, delay, driveDistance);
                break;

            case DO_NOTHING:
            default:
                autoStrategy = null;
                break;
        }
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        dashboard.clearDisplay();
        robot.startMode(TrcRobot.RunMode.AUTO_MODE);
    }   //startMode

    @Override
    public void stopMode()
    {
        robot.stopMode(TrcRobot.RunMode.AUTO_MODE);
    }   //stopMode

    @Override
    public void runContinuous(double elapsedTime)
    {
        if (autoStrategy != null)
        {
            autoStrategy.autoPeriodic(elapsedTime);
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
        FtcChoiceMenu allianceMenu = new FtcChoiceMenu("Alliance:", null, this);
        FtcChoiceMenu startPosMenu = new FtcChoiceMenu("Start position:", allianceMenu, this);
        FtcValueMenu delayMenu = new FtcValueMenu(
                "Delay time: ", startPosMenu, this, 0.0, 15.0, 1.0, 0.0, " %.0f sec");
        FtcChoiceMenu strategyMenu = new FtcChoiceMenu("Strategies:", delayMenu, this);
        FtcValueMenu distanceMenu = new FtcValueMenu(
                "Distance: ", strategyMenu, this, 1.0, 10.0, 1.0, 1.0, " %.0f ft");
        FtcValueMenu shootParticlesMenu = new FtcValueMenu(
                "Shoot particles: ", strategyMenu, this, 0.0, 2.0, 1.0, 2.0, "%.0f");
        FtcValueMenu beaconButtonsMenu = new FtcValueMenu(
                "Push beacon buttons: ", shootParticlesMenu, this, 0.0, 2.0, 1.0, 2.0, "%.0f");
        FtcChoiceMenu parkOptionMenu = new FtcChoiceMenu("Park options", beaconButtonsMenu, this);

        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE, startPosMenu);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE, startPosMenu);

        startPosMenu.addChoice("Near", StartPosition.NEAR, delayMenu);
        startPosMenu.addChoice("Far", StartPosition.FAR, delayMenu);

        delayMenu.setChildMenu(strategyMenu);

        strategyMenu.addChoice("Do nothing", Strategy.DO_NOTHING);
        strategyMenu.addChoice("Beacon", Strategy.BEACON, shootParticlesMenu);
        strategyMenu.addChoice("Defense", Strategy.DEFENSE, distanceMenu);

        shootParticlesMenu.setChildMenu(beaconButtonsMenu);
        beaconButtonsMenu.setChildMenu(parkOptionMenu);

        parkOptionMenu.addChoice("Do nothing", ParkOption.DO_NOTHING);
        parkOptionMenu.addChoice("Park center", ParkOption.PARK_CENTER);
        parkOptionMenu.addChoice("Park corner", ParkOption.PARK_CORNER);

        FtcMenu.walkMenuTree(allianceMenu);

        alliance = (Alliance)allianceMenu.getCurrentChoiceObject();
        startPos = (StartPosition)startPosMenu.getCurrentChoiceObject();
        delay = delayMenu.getCurrentValue();
        strategy = (Strategy)strategyMenu.getCurrentChoiceObject();
        driveDistance = distanceMenu.getCurrentValue();
        shootParticles = (int)shootParticlesMenu.getCurrentValue();
        beaconButtons = (int)beaconButtonsMenu.getCurrentValue();
        parkOption = (ParkOption)parkOptionMenu.getCurrentChoiceObject();

        dashboard.displayPrintf(0, "Auto Strategy: %s (%s)", strategyMenu.getCurrentChoiceText(), alliance.toString());
        dashboard.displayPrintf(1, "Start position: %s", startPos.toString());
        dashboard.displayPrintf(2, "Delay = %.0f sec", delay);
        dashboard.displayPrintf(3, "Defense: distance=%.0f ft", driveDistance);
        dashboard.displayPrintf(
                4, "Beacon: ShootParticles=%d,BeaconButtons=%d,ParkOption=%s",
                shootParticles, beaconButtons, parkOption.toString());
    }   //doMenus

}   //class FtcAuto
