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

import java.util.Date;

import ftclib.FtcOpMode;
import trclib.TrcAnalogInput;
import trclib.TrcDbgTrace;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

public class AutoBeacon implements TrcRobot.AutoStrategy
{
    private enum State
    {
        MOVE_CENTER_VORTEX,
        AIM_CENTER_VORTEX,
        GOTO_CENTER_VORTEX,
        SHOOT_PARTICLES,
        MOVE_OUT,
        TURN_TO_CAPBALL,
        KNOCK_OUT_CAPBALL,
        BACKUP,
        TURN_TO_CENTER1,
        PARK_CENTER1,
        TURN_TO_CORNER1,
        PARK_CORNER1,
        TURN_TO_WALL,
        GOTO_WALL,
        PARALLEL_WALL,
        PUSH_BEACON_BUTTONS,
        DONE
    }   //enum State

    private static final String moduleName = "AutoBeacon";

    private TrcDbgTrace tracer = FtcOpMode.getGlobalTracer();

    private Robot robot;
    private FtcAuto.Alliance alliance;
    private FtcAuto.StartPosition startPos;
    private double delay;
    private int shootParticles;
    private int beaconButtons;
    private FtcAuto.ParkOption parkOption;
    private CmdPushBeaconButtons2 pushBeaconButtons;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;
    private boolean particleLoaded = true;
    private double heading = 0.0;

    public AutoBeacon(
            Robot robot,
            FtcAuto.Alliance alliance,
            FtcAuto.StartPosition startPos,
            double delay,
            int shootParticles,
            int beaconButtons,
            FtcAuto.ParkOption parkOption)
    {
        this.robot = robot;
        this.alliance = alliance;
        this.startPos = startPos;
        this.delay = delay;
        this.shootParticles = shootParticles;
        this.beaconButtons = beaconButtons;
        this.parkOption = parkOption;
        pushBeaconButtons = new CmdPushBeaconButtons2(robot, alliance, beaconButtons, parkOption);
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        robot.battery.setTaskEnabled(true);
        sm.start(startPos == FtcAuto.StartPosition.NEAR? State.SHOOT_PARTICLES: State.MOVE_CENTER_VORTEX);
        Date date = new Date();
        tracer.traceInfo(moduleName, "%s: ********** Starting autonomous **********", date.toString());
    }

    @Override
    public void autoPeriodic(double elapsedTime)
    {
        //
        // Print debug info.
        //
        robot.dashboard.displayPrintf(
                1, "State: %s", sm.isReady()? (sm.getState()).toString(): "Disabled");
        if (Robot.USE_LINE_DETECTOR)
        {
            if (Robot.USE_ODS_LINE_DETECTOR)
            {
                robot.dashboard.displayPrintf(2, "LineDetect: light=%.3f",
                                              (Double)robot.odsLineDetector.getRawData(
                                                      0, TrcAnalogInput.DataType.INPUT_DATA).value);
            }
            else
            {
                robot.dashboard.displayPrintf(2, "LineDetect: light=%d", robot.lineDetectionSensor.sensor.alpha());
            }
        }

        if (Robot.USE_COLOR_SENSOR)
        {
            robot.dashboard.displayPrintf(3, "RGBAH: [%d,%d,%d,%d,%x]",
                                          robot.beaconColorSensor.sensor.red(), robot.beaconColorSensor.sensor.green(),
                                          robot.beaconColorSensor.sensor.blue(), robot.beaconColorSensor.sensor.alpha(),
                                          robot.beaconColorSensor.sensor.argb());
        }

        if (robot.pidDrive.isEnabled())
        {
            robot.encoderXPidCtrl.displayPidInfo(4);
            robot.encoderYPidCtrl.displayPidInfo(6);
            robot.gyroPidCtrl.displayPidInfo(8);
        }

        if (sm.isReady())
        {
            State state = sm.getState();
            State nextState;
            double xDistance;
            double yDistance;

            robot.traceStateInfo(elapsedTime, state.toString(), heading);
            switch (state)
            {
                case MOVE_CENTER_VORTEX:
                    //
                    // This state is called only for FAR StartPosition.
                    //
                    xDistance = 0.0;
                    yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? 6.0: 8.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.AIM_CENTER_VORTEX);
                    break;

                case AIM_CENTER_VORTEX:
                    //
                    // This state is called only for FAR StartPosition.
                    //
                    xDistance = yDistance = 0.0;
                    heading = alliance == FtcAuto.Alliance.RED_ALLIANCE? -45.0: 43.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.GOTO_CENTER_VORTEX);
                    break;

                case GOTO_CENTER_VORTEX:
                    //
                    // This state is called only for FAR StartPosition.
                    //
                    xDistance = 0.0;
                    yDistance = 6.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.SHOOT_PARTICLES);
                    break;

                case SHOOT_PARTICLES:
                    tracer.traceInfo(state.toString(), "NumParticles=%d", shootParticles);
                    //
                    // Fire a particle if any.
                    //
                    if (shootParticles > 0)
                    {
                        if (particleLoaded)
                        {
                            robot.shooter.fireOneShot(event);
                            particleLoaded = false;
                        }
                        else
                        {
                            robot.shooter.loadAndFireOneShot(event);
                        }
                        shootParticles--;
                        sm.waitForSingleEvent(event, State.SHOOT_PARTICLES);
                    }
                    //
                    // Do delay if any.
                    //
                    else
                    {
                        nextState = startPos == FtcAuto.StartPosition.NEAR? State.MOVE_OUT: State.TURN_TO_CAPBALL;
                        if (delay > 0.0 && delay - elapsedTime > 0)
                        {
                            timer.set(delay - elapsedTime, event);
                            sm.waitForSingleEvent(event, nextState);
                        }
                        else
                        {
                            sm.setState(nextState);
                        }
                    }
                    break;

                case MOVE_OUT:
                    xDistance = 0.0;
                    yDistance = 32.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_TO_CAPBALL);
                    break;

                case TURN_TO_CAPBALL:
                    xDistance = yDistance = 0.0;
                    heading = robot.selectParameter(
                            startPos, alliance,
                            //NEAR
                            -30.0, 30.0,
                            //FAR
                            -54.0, 50.0);

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.KNOCK_OUT_CAPBALL);
                    break;

                case KNOCK_OUT_CAPBALL:
                    nextState = beaconButtons == 0? State.BACKUP: State.TURN_TO_WALL;
                    xDistance = 0.0;
                    yDistance = robot.selectParameter(
                            startPos, alliance,
                            //NEAR (old value = 64.0) 45.0 for pushBeacon1
                            beaconButtons != 0? 52.0: 52.0, beaconButtons != 0? 70.0: 52.0,
                            //FAR
                            80.0, 80.0);

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case BACKUP:
                    nextState = parkOption == FtcAuto.ParkOption.PARK_CENTER? State.TURN_TO_CENTER1:
                                parkOption == FtcAuto.ParkOption.PARK_CORNER? State.TURN_TO_CORNER1: State.DONE;
                    xDistance = 0.0;
                    if (parkOption == FtcAuto.ParkOption.PARK_CORNER)
                    {
                        yDistance = robot.selectParameter(
                                startPos, alliance,
                                //NEAR
                                -24.0, -17.0,
                                //FAR
                                -24.0, -20.0);
                    }
                    else
                    {
                        yDistance = robot.selectParameter(
                                startPos, alliance,
                                //NEAR
                                -64.0, -58.0,
                                //FAR
                                -46.0, -50.0);
                    }

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case TURN_TO_CENTER1:
                    xDistance = yDistance = 0.0;
                    heading = robot.selectParameter(
                            startPos, alliance,
                            //NEAR
                            13.0, -11.0,
                            //FAR
                            -0.3, -2.0);

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.PARK_CENTER1);
                    break;

                case PARK_CENTER1:
                    xDistance = 0.0;
                    yDistance = robot.selectParameter(
                            startPos, alliance,
                            //NEAR
                            50.0, 50.0,
                            //FAR
                            22.0, 32.0);

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case TURN_TO_CORNER1:
                    xDistance = yDistance = 0.0;
                    heading = alliance == FtcAuto.Alliance.RED_ALLIANCE? -130.0: 135.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.PARK_CORNER1);
                    break;

                case PARK_CORNER1:
                    xDistance = 0.0;
                    yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? 52.0: 52.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case TURN_TO_WALL:
                    xDistance = yDistance = 0.0;
                    heading = alliance == FtcAuto.Alliance.RED_ALLIANCE? -90.0: 90.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.GOTO_WALL);
                    break;

                case GOTO_WALL:
                    xDistance = 0.0;
                    yDistance = 22.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.PARALLEL_WALL);
                    break;

                case PARALLEL_WALL:
                    xDistance = yDistance = 0.0;
                    heading = alliance == FtcAuto.Alliance.RED_ALLIANCE? 0.0: 180.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.PUSH_BEACON_BUTTONS);
                    break;

                case PUSH_BEACON_BUTTONS:
                    if (pushBeaconButtons.cmdPeriodic(elapsedTime))
                    {
                        sm.setState(State.DONE);
                    }
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
    }   //autoPeriodic

}   //class AutoBeacon
