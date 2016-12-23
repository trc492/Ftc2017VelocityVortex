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

import trclib.TrcAnalogInput;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;

public class Auto100 implements TrcRobot.AutoStrategy
{
    private enum State
    {
        NEAR_START,
        TURN_TO_WALL,
        GOTO_WALL,
        PARALLEL_WALL,
        PUSH_BEACON_BUTTONS,
        DONE
    }   //enum State

    private static final String moduleName = "Auto100";

    private Robot robot;
    private FtcAuto.Alliance alliance;
    private TrcRobot.RobotCommand pushBeaconButtonsCmd;
    private CmdNearStart nearStartCmd;
    private TrcEvent event;
    private TrcStateMachine<State> sm;

    public Auto100(
            Robot robot,
            FtcAuto.Alliance alliance,
            double delay,
            int numParticles,
            TrcRobot.RobotCommand pushBeaconButtonsCmd)
    {
        this.robot = robot;
        this.alliance = alliance;
        this.pushBeaconButtonsCmd = pushBeaconButtonsCmd;
        boolean shortRun = alliance == FtcAuto.Alliance.RED_ALLIANCE &&
                           pushBeaconButtonsCmd instanceof CmdPushBeaconButtons1;
        nearStartCmd = new CmdNearStart(robot, alliance, delay, numParticles, shortRun);
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.NEAR_START);
    }   //Auto100

    @Override
    public void autoPeriodic(double elapsedTime)
    {
        //
        // Print debug info.
        //
        robot.dashboard.displayPrintf(1, "State: %s", sm.isReady()? (sm.getState()).toString(): "Disabled");

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
            double xDistance, yDistance;

            if (state != State.NEAR_START && state != State.PUSH_BEACON_BUTTONS)
            {
                robot.traceStateInfo(elapsedTime, state.toString());
            }

            switch (state)
            {
                case NEAR_START:
                    if (nearStartCmd.cmdPeriodic(elapsedTime))
                    {
                        sm.setState(State.TURN_TO_WALL);
                    }
                    break;

                case TURN_TO_WALL:
                    xDistance = yDistance = 0.0;
                    robot.targetHeading = alliance == FtcAuto.Alliance.RED_ALLIANCE? -90.0: 90.0;

                    robot.setTurnPID(xDistance, yDistance, robot.targetHeading);
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.GOTO_WALL);
                    break;

                case GOTO_WALL:
                    xDistance = 0.0;
                    yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE &&
                                pushBeaconButtonsCmd instanceof CmdPushBeaconButtons1? 22.0: 16.0;

                    robot.setTurnPID(xDistance, yDistance, robot.targetHeading);
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.PARALLEL_WALL);
                    break;

                case PARALLEL_WALL:
                    xDistance = yDistance = 0.0;
                    robot.targetHeading = alliance == FtcAuto.Alliance.RED_ALLIANCE? 0.0: 180.0;

                    robot.setTurnPID(xDistance, yDistance, robot.targetHeading);
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.PUSH_BEACON_BUTTONS);
                    break;

                case PUSH_BEACON_BUTTONS:
                    if (pushBeaconButtonsCmd.cmdPeriodic(elapsedTime))
                    {
                        sm.setState(State.DONE);
                    }
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
    }   //autoPeriodic

}   //class Auto100
