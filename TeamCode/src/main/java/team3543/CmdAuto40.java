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

import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;

public class CmdAuto40 implements TrcRobot.RobotCommand
{
    private enum State
    {
        START,
        BACKUP,
        TURN_TO_CENTER,
        PARK_CENTER,
        TURN_TO_CORNER,
        PARK_CORNER,
        DONE
    }   //enum State

    private static final String moduleName = "CmdAuto40";

    private Robot robot;
    private FtcAuto.Alliance alliance;
    private FtcAuto.ParkOption parkOption;
    private boolean startNear;
    private TrcRobot.RobotCommand startCmd;
    private TrcEvent event;
    private TrcStateMachine<State> sm;

    public CmdAuto40(
            Robot robot,
            FtcAuto.Alliance alliance,
            double delay,
            int numParticles,
            FtcAuto.ParkOption parkOption,
            boolean startNear)
    {
        this.robot = robot;
        this.alliance = alliance;
        this.parkOption = parkOption;
        this.startNear = startNear;
        startCmd = startNear? new CmdNearStart(robot, alliance, delay, numParticles, true):
                              new CmdFarStart(robot, alliance, delay, numParticles);
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START);
    }   //CmdAuto40

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean done = false;
        //
        // Print debug info.
        //
        State state = sm.getState();
        if (state != State.START)
        {
            robot.dashboard.displayPrintf(1, "State: %s", state != null? state.toString(): "Disabled");
        }

        if (robot.pidDrive.isEnabled())
        {
            robot.encoderXPidCtrl.displayPidInfo(2);
            robot.encoderYPidCtrl.displayPidInfo(4);
            robot.gyroPidCtrl.displayPidInfo(6);
        }

        if (sm.isReady())
        {
            state = sm.getState();
            State nextState;
            double xDistance, yDistance;

            if (state != State.START)
            {
                robot.traceStateInfo(elapsedTime, state.toString());
            }

            switch (state)
            {
                case START:
                    if (startCmd.cmdPeriodic(elapsedTime))
                    {
                        sm.setState(State.BACKUP);
                    }
                    break;

                case BACKUP:
                    nextState = parkOption == FtcAuto.ParkOption.PARK_CENTER? State.TURN_TO_CENTER:
                                parkOption == FtcAuto.ParkOption.PARK_CORNER? State.TURN_TO_CORNER: State.DONE;
                    xDistance = 0.0;
                    if (parkOption == FtcAuto.ParkOption.PARK_CORNER)
                    {
                        yDistance = robot.selectParameter(
                                startNear, alliance,
                                //NEAR
                                -32.0, -17.0,
                                //FAR
                                -24.0, -20.0);
                    }
                    else
                    {
                        yDistance = robot.selectParameter(
                                startNear, alliance,
                                //NEAR
                                -64.0, -58.0,
                                //FAR
                                -46.0, -50.0);
                    }

                    robot.setTurnPID(xDistance, yDistance, robot.targetHeading);
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case TURN_TO_CENTER:
                    xDistance = yDistance = 0.0;
                    robot.targetHeading = robot.selectParameter(
                            startNear, alliance,
                            //NEAR
                            0.0, -11.0,
                            //FAR
                            -0.3, -2.0);

                    robot.setTurnPID(xDistance, yDistance, robot.targetHeading);
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.PARK_CENTER);
                    break;

                case PARK_CENTER:
                    xDistance = 0.0;
                    yDistance = robot.selectParameter(
                            startNear, alliance,
                            //NEAR
                            48.0, 50.0,
                            //FAR
                            22.0, 32.0);

                    robot.setTurnPID(xDistance, yDistance, robot.targetHeading);
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case TURN_TO_CORNER:
                    xDistance = yDistance = 0.0;
                    robot.targetHeading = alliance == FtcAuto.Alliance.RED_ALLIANCE? 45.0: 135.0;

                    robot.setTurnPID(xDistance, yDistance, robot.targetHeading);
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.PARK_CORNER);
                    break;

                case PARK_CORNER:
                    xDistance = 0.0;
                    yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? -42.0: 42.0;

                    robot.setTurnPID(xDistance, yDistance, robot.targetHeading);
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    done = true;
                    sm.stop();
                    break;
            }
        }

        return done;
    }   //cmdPeriodic

}   //class CmdAuto40
