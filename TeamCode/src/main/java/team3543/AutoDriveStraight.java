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

import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcDbgTrace;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

public class AutoDriveStraight implements TrcRobot.AutoStrategy
{
    private enum State
    {
        DO_DELAY,
        DRIVE_DISTANCE,
        DONE
    }   //enum State

    private static final String moduleName = "AutoDriveStraight";

    private HalDashboard dashboard;
    private TrcDbgTrace tracer = FtcOpMode.getGlobalTracer();

    private Robot robot;
    private double delay;
    private double distance;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine sm;

    public AutoDriveStraight(Robot robot, double delay, double distance)
    {
        this.robot = robot;
        this.dashboard = robot.dashboard;
        this.delay = delay;
        this.distance = distance;
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine(moduleName);
        sm.start(State.DO_DELAY);
    }

    @Override
    public void autoPeriodic(double elapsedTime)
    {
        dashboard.displayPrintf(1, moduleName + ": delay=%.0f, distance=%.0f",
                                delay, distance);

        if (sm.isReady())
        {
            State state = (State)sm.getState();
            tracer.traceInfo(moduleName, "State: %s [%.3f]", state.toString(), elapsedTime);
            dashboard.displayPrintf(7, "State: %s [%.3f]", state.toString(), elapsedTime);

            switch (state)
            {
                case DO_DELAY:
                    if (delay == 0.0)
                    {
                        sm.setState(State.DRIVE_DISTANCE);
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.DRIVE_DISTANCE);
                    }
                    break;

                case DRIVE_DISTANCE:
                    //
                    // Drive the set distance.
                    //
                    robot.pidDrive.setTarget(distance*12.0, 0.0, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
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
    }

}   //class AutoDriveStraight
