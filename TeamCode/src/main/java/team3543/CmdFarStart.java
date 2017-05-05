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

import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

public class CmdFarStart implements TrcRobot.RobotCommand
{
    private static final boolean debugXPid = false;
    private static final boolean debugYPid = false;
    private static final boolean debugTurnPid = false;

    private enum State
    {
        MOVE_OUT,
        TURN_TO_CENTER_VORTEX,
        GET_CLOSER,
        SHOOT_PARTICLES,
        TURN_TO_CAPBALL,
        KNOCK_OUT_CAPBALL,
        DONE
    }   //enum State

    private static final String moduleName = "CmdFarStart";

    private Robot robot;
    private FtcAuto.Alliance alliance;
    private double delay;
    private int numParticles;
    private boolean noCapBall;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;
    private boolean particleLoaded = true;

    public CmdFarStart(Robot robot, FtcAuto.Alliance alliance, double delay, int numParticles, boolean noCapBall)
    {
        this.robot = robot;
        this.alliance = alliance;
        this.delay = delay;
        this.numParticles = numParticles;
        this.noCapBall = noCapBall;
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.MOVE_OUT);
    }   //CmdFarStart

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean done = !sm.isEnabled();
        //
        // Print debug info.
        //
        State state = sm.getState();
        robot.dashboard.displayPrintf(1, "State: %s", state != null? sm.getState().toString(): "Disabled");

        if (sm.isReady())
        {
            state = sm.getState();
            double xDistance = 0.0, yDistance = 0.0;

            switch (state)
            {
                case MOVE_OUT:
                    //
                    // Move out a little away from the wall so we can turn.
                    //
                    xDistance = 0.0;
                    yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? 6.0: 8.0;

                    robot.setPIDDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_TO_CENTER_VORTEX);
                    break;

                case TURN_TO_CENTER_VORTEX:
                    //
                    // Turn to aim at the center vortex.
                    //
                    xDistance = yDistance = 0.0;
                    robot.targetHeading = alliance == FtcAuto.Alliance.RED_ALLIANCE? -45.0: 43.0;

                    robot.setPIDDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.GET_CLOSER);
                    break;

                case GET_CLOSER:
                    //
                    // We are a little too far to shoot, get a little closer.
                    //
                    xDistance = 0.0;
                    yDistance = 12.0;

                    robot.setPIDDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.SHOOT_PARTICLES);
                    break;

                case SHOOT_PARTICLES:
                    //
                    // Shoot the selected number of particles.
                    //
                    robot.tracer.traceInfo(state.toString(), "NumParticles=%d", numParticles);
                    //
                    // Fire a particle if any.
                    //
                    if (numParticles > 0)
                    {
                        if (particleLoaded)
                        {
                            //
                            // The first particle is preloaded in the shooter so no need to load to save time.
                            //
                            robot.shooter.fireOneShot(event);
                            particleLoaded = false;
                        }
                        else
                        {
                            //
                            // Need to load the second particle before shooting.
                            //
                            robot.shooter.loadAndFireOneShot(event);
                        }
                        numParticles--;
                        sm.waitForSingleEvent(event, State.SHOOT_PARTICLES);
                    }
                    else if (!noCapBall)
                    {
                        //
                        // Do delay if any.
                        //
                        if (delay > 0.0 && delay - elapsedTime > 0)
                        {
                            timer.set(delay - elapsedTime, event);
                            sm.waitForSingleEvent(event, State.TURN_TO_CAPBALL);
                        }
                        else
                        {
                            sm.setState(State.TURN_TO_CAPBALL);
                        }
                    }
                    else
                    {
                        //
                        // We don't want to displace Cap Ball, go forward a little and stop!
                        //
                        xDistance = 0.0;
                        yDistance = 52.0;

                        robot.encoderYPidCtrl.setOutputRange(-0.5, 0.5);
                        robot.setPIDDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
                        sm.waitForSingleEvent(event, State.DONE);
                    }
                    break;

                case TURN_TO_CAPBALL:
                    //
                    // Turn the robot so that it will run and graze the Cap Ball off its base.
                    //
                    xDistance = yDistance = 0.0;
                    robot.targetHeading = alliance == FtcAuto.Alliance.RED_ALLIANCE? -54.0: 48.0;

                    robot.setPIDDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.KNOCK_OUT_CAPBALL);
                    break;

                case KNOCK_OUT_CAPBALL:
                    //
                    // Run and graze the Cap Ball.
                    //
                    xDistance = 0.0;
                    yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? 80.0: 80.0;

                    robot.setPIDDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    robot.encoderYPidCtrl.setOutputRange(-1.0, 1.0);
                    sm.stop();
                    done = true;
                    break;
            }
            robot.traceStateInfo(elapsedTime, state.toString(), xDistance, yDistance, robot.targetHeading);
        }

        if (robot.pidDrive.isActive() && (debugXPid || debugYPid || debugTurnPid))
        {
            robot.tracer.traceInfo("Battery", "Voltage=%5.2fV (%5.2fV)",
                                   robot.battery.getVoltage(), robot.battery.getLowestVoltage());

            if (debugXPid)
            {
                robot.encoderXPidCtrl.printPidInfo(robot.tracer);
            }

            if (debugYPid)
            {
                robot.encoderYPidCtrl.printPidInfo(robot.tracer);
            }

            if (debugTurnPid)
            {
                robot.gyroPidCtrl.printPidInfo(robot.tracer);
            }
        }

        return done;
    }   //cmdPeriodic

}   //class CmdFarStart
