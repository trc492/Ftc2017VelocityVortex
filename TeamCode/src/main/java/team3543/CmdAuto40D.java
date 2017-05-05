/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
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

import android.speech.tts.TextToSpeech;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.Locale;

import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdAuto40D implements TrcRobot.RobotCommand
{
    private static final boolean debugXPid = false;
    private static final boolean debugYPid = false;
    private static final boolean debugTurnPid = false;

    private enum State
    {
        START,
        TURN_TO_CENTER,
        FIND_NEAR_TARGET1,
        FIND_NEAR_TARGET2,
        PARK_CENTER1,
        WAIT1,
        GO_ACROSS,
        FIND_FAR_TARGET1,
        FIND_FAR_TARGET2,
        WAIT2,
        PARK_CENTER2,
        PARK_CENTER3,
        DONE
    }   //enum State

    private static final String moduleName = "CmdAuto40D";

    private Robot robot;
    private FtcAuto.Alliance alliance;
    private double delay;
    private FtcAuto.ParkOption parkOption;
    private TrcRobot.RobotCommand startCmd;
    private VuforiaVision vuforiaVision;
    private TrcTimer timer;
    private TrcEvent timerEvent;
    private TrcEvent driveEvent;
    private TrcStateMachine<State> sm;
    private double deepDistance = 15.0;

    CmdAuto40D(Robot robot, FtcAuto.Alliance alliance, double delay, int numParticles, FtcAuto.ParkOption parkOption)
    {
        this.robot = robot;
        this.alliance = alliance;
        this.delay = delay;
        this.parkOption = parkOption;
        startCmd = new CmdFarStart(robot, alliance, delay, numParticles, true);
        vuforiaVision = new VuforiaVision(robot);
        vuforiaVision.setEnabled(true);
        timer = new TrcTimer(moduleName);
        timerEvent = new TrcEvent(moduleName + ".timer");
        driveEvent = new TrcEvent(moduleName + ".drive");
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START);
        //// It is critical to turn very accurately in this strategy.
        //
        robot.gyroPidCtrl.setTargetTolerance(0.5);
    }   //CmdAuto40D

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean done = !sm.isEnabled();
        //
        // Print debug info.
        //
        State state = sm.getState();
        if (state != State.START)
        {
            robot.dashboard.displayPrintf(1, "State: %s", state != null? state.toString(): "Disabled");
        }

        if (robot.pidDrive.isActive())
        {
            robot.encoderXPidCtrl.displayPidInfo(2);
            robot.encoderYPidCtrl.displayPidInfo(4);
            robot.gyroPidCtrl.displayPidInfo(6);
        }

        if (sm.isReady())
        {
            State nextState;
            state = sm.getState();
            double xDistance = 0.0, yDistance = 0.0;
            boolean printStateInfo = true;
            VectorF targetPos;
            int target;

            switch (state)
            {
                case START:
                    //
                    // The robot starts at the far position. It runs the CmdFarStart which is common segment shared
                    // by several autonomous strategies. This state will run that segment until its completion when
                    // it returns true, then we move on to the next state. startCmd would shoot a specified number
                    // of particles and stop close to the center vortex.
                    //
                    printStateInfo = false;
                    if (startCmd.cmdPeriodic(elapsedTime))
                    {
                        sm.setState(State.TURN_TO_CENTER);
                    }
                    break;

                case TURN_TO_CENTER:
                    //
                    // Turn to face the center vortex.
                    //
                    xDistance = yDistance = 0.0;
                    robot.targetHeading = alliance == FtcAuto.Alliance.RED_ALLIANCE? 0.0: 180.0;

                    robot.setPIDDriveTarget(xDistance, yDistance, robot.targetHeading, false, driveEvent);
                    sm.waitForSingleEvent(driveEvent, State.FIND_NEAR_TARGET1);
                    break;

                case FIND_NEAR_TARGET1:
                    timer.set(1.0, timerEvent);
                    sm.setState(State.FIND_NEAR_TARGET2);
                    break;

                case FIND_NEAR_TARGET2:
                    //
                    // Looking for near target: Gears for red alliance and Wheels for blue alliance.
                    //
                    target = alliance == FtcAuto.Alliance.RED_ALLIANCE?
                            VuforiaVision.TARGET_GEARS: VuforiaVision.TARGET_WHEELS;
                    if ((targetPos = vuforiaVision.getTargetPosition(target)) != null)
                    {
//                        robot.gyroPidCtrl.setTargetTolerance(2.0);
                        double xTargetDistance = targetPos.get(2) / RobotInfo.MM_PER_INCH;
//                        double yTargetDistance = targetPos.get(0) / RobotInfo.MM_PER_INCH;
                        xDistance = 52.0 - xTargetDistance;
                        yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? 16.0: -16.0;
//                        yDistance = 6.0 - yTargetDistance;
                        robot.tracer.traceInfo("Auto40DF", "xDistance=%.2f, yDistance=%.2f, heading=%.1f",
                                               xDistance, yDistance, robot.targetHeading);
                        robot.textToSpeech.speak(String.format(Locale.US, "%s found at %d inches",
                                                               vuforiaVision.getTargetName(target),
                                                               (int) xTargetDistance),
                                                 TextToSpeech.QUEUE_FLUSH, null);

                        robot.setPIDDriveTarget(xDistance, yDistance, robot.targetHeading, false, driveEvent);
                        robot.gyroPidCtrl.setNoOscillation(true);
                        sm.waitForSingleEvent(driveEvent, State.WAIT1);
                    }
                    else if (timerEvent.isSignaled())
                    {
                        sm.setState(State.PARK_CENTER1);
                    }
                    break;

                case PARK_CENTER1:
                    //
                    // We only come here because we couldn't find the vision target within the timeout period. So we
                    // will just go forward a little and park at the center vortex assuming we are right on it.
                    //
//                    robot.gyroPidCtrl.setTargetTolerance(0.5);
                    xDistance = 0.0;
                    yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? 16.0: -16.0;

                    robot.setPIDDriveTarget(xDistance, yDistance, robot.targetHeading, false, driveEvent);
                    robot.gyroPidCtrl.setNoOscillation(true);
                    sm.waitForSingleEvent(driveEvent, State.WAIT1);
                    break;

                case WAIT1:
                    //
                    // Wait the specified delay.
                    //
                    if (delay < 10.0) delay = 10.0;
                    delay -= elapsedTime;
                    if (delay > 0.0)
                    {
                        timer.set(delay, timerEvent);
                        sm.setState(State.GO_ACROSS);
                    }
                    else
                    {
                        sm.setState(State.GO_ACROSS);
                    }
                    break;

                case GO_ACROSS:
                    xDistance = 0.0;
                    yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? 40.0: -40.0;

                    robot.encoderYPidCtrl.setOutputRange(-0.3, 0.3);
                    robot.setPIDDriveTarget(xDistance, yDistance, robot.targetHeading, false, driveEvent);
                    sm.waitForSingleEvent(driveEvent, State.FIND_FAR_TARGET1);
                    break;

                case FIND_FAR_TARGET1:
                    robot.encoderYPidCtrl.setOutputRange(-1.0, 1.0);
                    timer.set(1.0, timerEvent);
                    sm.setState(State.FIND_FAR_TARGET2);
                    break;

                case FIND_FAR_TARGET2:
                    //
                    // Looking for far target: Tools for red alliance and Legos for blue alliance.
                    //
                    target = alliance == FtcAuto.Alliance.RED_ALLIANCE?
                            VuforiaVision.TARGET_TOOLS: VuforiaVision.TARGET_LEGOS;
                    nextState = parkOption == FtcAuto.ParkOption.PARK_CENTER? State.WAIT2: State.DONE;
                    if ((targetPos = vuforiaVision.getTargetPosition(target)) != null)
                    {
                        double targetDistance = targetPos.get(2)/RobotInfo.MM_PER_INCH;
                        xDistance = 56.0 - targetDistance;
                        yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? deepDistance: -deepDistance - 12.0;
                        robot.tracer.traceInfo("Auto40DF", "xDistance=%.2f, yDistance=%.2f, heading=%.1f",
                                               xDistance, yDistance, robot.targetHeading);
                        robot.textToSpeech.speak(String.format(Locale.US, "%s found at %d inches",
                                                               vuforiaVision.getTargetName(target),
                                                               (int)targetDistance),
                                                 TextToSpeech.QUEUE_FLUSH, null);

                        robot.setPIDDriveTarget(xDistance, yDistance, robot.targetHeading, false, driveEvent);
                        sm.waitForSingleEvent(driveEvent, nextState);
                    }
                    else if (timerEvent.isSignaled())
                    {
                        deepDistance = 0.0;
                        sm.setState(nextState);
                    }
                    break;

                case WAIT2:
                    double delay = 25.0 - elapsedTime;
                    if (delay > 0.0)
                    {
                        timer.set(delay, timerEvent);
                        sm.waitForSingleEvent(timerEvent, State.PARK_CENTER2);
                    }
                    else
                    {
                        sm.setState(State.PARK_CENTER2);
                    }
                    break;

                case PARK_CENTER2:
                    if (deepDistance > 0.0)
                    {
                        xDistance = yDistance = 0.0;
                        robot.targetHeading = alliance == FtcAuto.Alliance.RED_ALLIANCE? -21.0: 210.0;
                        nextState = State.PARK_CENTER3;
                    }
                    else
                    {
                        xDistance = 0.0;
                        yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? -28.0: 28.0;
                        nextState = State.DONE;
                    }
                    robot.setPIDDriveTarget(xDistance, yDistance, robot.targetHeading, false, driveEvent);
                    robot.gyroPidCtrl.setNoOscillation(true);
                    sm.waitForSingleEvent(driveEvent, nextState);
                    break;

                case PARK_CENTER3:
//                    robot.gyroPidCtrl.setTargetTolerance(2.0);
                    xDistance = 0.0;
                    yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? -48.0: 48.0;

                    robot.setPIDDriveTarget(xDistance, yDistance, robot.targetHeading, false, driveEvent);
                    robot.gyroPidCtrl.setNoOscillation(true);
                    sm.waitForSingleEvent(driveEvent, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    vuforiaVision.setEnabled(false);
                    done = true;
                    sm.stop();
                    break;
            }

            if (printStateInfo)
            {
                robot.traceStateInfo(elapsedTime, state.toString(), xDistance, yDistance, robot.targetHeading);
            }
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

}   //class CmdAuto40D
