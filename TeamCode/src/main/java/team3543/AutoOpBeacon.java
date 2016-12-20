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
import trclib.TrcAnalogInput;
import trclib.TrcDbgTrace;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

public class AutoOpBeacon implements TrcRobot.AutoStrategy
{
    private enum State
    {
        MOVE_CENTER_VORTEX,
        AIM_CENTER_VORTEX,
        GOTO_CENTER_VORTEX,
        SHOOT_PARTICLES,
        GOTO_LINE,
        DO_DELAY,
        GOTO_OP,
        TURN_TO_BEACON,
        GOTO_BEACON,
        PARALLEL_WALL,
        ALIGN_WALL1,
        ALIGN_WALL2,
        FIND_LINE,
        PUSH_BUTTON,
        RETRACT,
        NEXT_BEACON,
        TURN_TO_CENTER,
        GOTO_CENTER,
        DONE
    }   //enum State

    private static final String moduleName = "AutoOpBeacon";

    private TrcDbgTrace tracer = FtcOpMode.getGlobalTracer();

    private Robot robot;
    private FtcAuto.Alliance alliance;
    private double delay;
    private int shootParticles;
    private int beaconButtons;
    private FtcAuto.ParkOption parkOption;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine sm;
    private boolean particleLoaded = true;
    private boolean leftPusherExtended = false;
    private boolean rightPusherExtended = false;
    private double heading = 0.0;
    private int remainingBeaconButtons = 0;
    private double timeout = 0.0;

    public AutoOpBeacon(
            Robot robot,
            FtcAuto.Alliance alliance,
            double delay,
            int shootParticles,
            int beaconButtons,
            FtcAuto.ParkOption parkOption)
    {
        this.robot = robot;
        this.alliance = alliance;
        this.delay = delay;
        this.shootParticles = shootParticles;
        this.beaconButtons = beaconButtons;
        this.remainingBeaconButtons = beaconButtons;
        this.parkOption = parkOption;
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine(moduleName);
        robot.battery.setTaskEnabled(true);
        sm.start(State.MOVE_CENTER_VORTEX);
        tracer.traceInfo(moduleName, "********** Starting autonomous **********");
    }

    @Override
    public void autoPeriodic(double elapsedTime)
    {
        //
        // Print debug info.
        //
        robot.dashboard.displayPrintf(
                1, "State: %s", sm.isReady()? ((State)sm.getState()).toString(): "Disabled");
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
            State state = (State)sm.getState();
            double xDistance;
            double yDistance;

            robot.traceStateInfo(elapsedTime, state.toString(), heading);
            switch (state)
            {
                case MOVE_CENTER_VORTEX:
                    xDistance = 0.0;
                    yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? 6.0: 8.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.AIM_CENTER_VORTEX);
                    break;

                case AIM_CENTER_VORTEX:
                    xDistance = yDistance = 0.0;
                    heading = alliance == FtcAuto.Alliance.RED_ALLIANCE? -45.0: 43.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.GOTO_CENTER_VORTEX);
                    break;

                case GOTO_CENTER_VORTEX:
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
                    else
                    {
                        xDistance = yDistance = 0.0;
                        heading = alliance == FtcAuto.Alliance.RED_ALLIANCE? -180.0: 0.0;

                        robot.setTurnPID(xDistance, yDistance, heading);
                        robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                        sm.waitForSingleEvent(event, State.GOTO_LINE);
                    }
                    break;

                case GOTO_LINE:
                    xDistance = 0.0;
                    yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? -8.0: 8.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.DO_DELAY);
                    break;

                case DO_DELAY:
                    delay = Math.max(delay, 10.0);
                    if (elapsedTime < delay)
                    {
                        timer.set(delay - elapsedTime, event);
                        sm.waitForSingleEvent(event, State.GOTO_OP);
                    }
                    else
                    {
                        sm.setState(State.GOTO_OP);
                    }
                    break;

                case GOTO_OP:
                    xDistance = 0.0;
                    yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? -72.0: 72.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_TO_BEACON);
                    break;

                case TURN_TO_BEACON:
                    xDistance = yDistance = 0.0;
                    heading = alliance == FtcAuto.Alliance.RED_ALLIANCE? -135.0: 45.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.GOTO_BEACON);
                    break;

                case GOTO_BEACON:
                    xDistance = 0.0;
                    yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? -12.0: 12.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.PARALLEL_WALL);
                    break;

                case PARALLEL_WALL:
                    xDistance = yDistance = 0.0;
                    heading = alliance == FtcAuto.Alliance.RED_ALLIANCE? -270.0: 0.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.ALIGN_WALL1);
                    break;

                case ALIGN_WALL1:
                    if (Robot.USE_RANGE_SENSOR)
                    {
                        robot.rangeSensor.setDeviceEnabled(true);
                    }
                    robot.driveBase.mecanumDrive_Cartesian(-1.0, 0.0, 0.0);
                    timeout = elapsedTime + 1.0;
                    sm.setState(State.ALIGN_WALL2);
                    break;

                case ALIGN_WALL2:
                    if (robot.getInput(robot.rangePidCtrl) < 3.5 || elapsedTime >= timeout)
                    {
                        robot.driveBase.mecanumDrive_Cartesian(0.0, 0.0, 0.0);
                        sm.setState(State.FIND_LINE);
                        if (Robot.USE_RANGE_SENSOR)
                        {
                            robot.rangeSensor.setDeviceEnabled(false);
                        }

                        if (Robot.USE_COLOR_SENSOR)
                        {
                            robot.beaconColorSensor.setDeviceEnabled(true);
                        }
                    }
                    break;

                case FIND_LINE:
                    xDistance = 0.0;
                    yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? -20.0: 20.0;
                    heading = robot.driveBase.getHeading();

                    if (Robot.USE_LINE_DETECTOR)
                    {
                        robot.lineTrigger.setEnabled(true);
                    }

                    robot.encoderYPidCtrl.setOutputRange(-0.12, 0.12);
                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.PUSH_BUTTON);
                    break;

                case PUSH_BUTTON:
                    robot.encoderYPidCtrl.setOutputRange(-1.0, 1.0);
                    if (Robot.USE_LINE_DETECTOR)
                    {
                        robot.lineTrigger.setEnabled(false);
                    }

                    int redValue = 0;
                    int greenValue = 0;
                    int blueValue = 0;
                    if (Robot.USE_COLOR_SENSOR)
                    {
                        redValue = robot.beaconColorSensor.sensor.red();
                        greenValue = robot.beaconColorSensor.sensor.green();
                        blueValue = robot.beaconColorSensor.sensor.blue();
                        robot.beaconColorSensor.setDeviceEnabled(false);
                    }
                    boolean isRed = redValue > blueValue && redValue > greenValue;
                    boolean isBlue = blueValue > redValue && blueValue > greenValue;
                    robot.dashboard.displayPrintf(
                            14, "[%d,%d,%d]isRed=%s,isBlue=%s",
                            redValue, greenValue, blueValue, Boolean.toString(isRed), Boolean.toString(isBlue));
                    tracer.traceInfo(
                            state.toString(), "[%d,%d,%d]isRed=%s,isBlue=%s",
                            redValue, greenValue, blueValue, Boolean.toString(isRed), Boolean.toString(isBlue));
                    //
                    // Determine which button to push and do it.
                    //
                    if (alliance == FtcAuto.Alliance.RED_ALLIANCE && isRed ||
                        alliance == FtcAuto.Alliance.BLUE_ALLIANCE && isBlue)
                    {
                        robot.leftButtonPusher.setPosition(RobotInfo.BUTTON_PUSHER_EXTEND_POSITION);
                        leftPusherExtended = true;
                    }
                    else if (alliance == FtcAuto.Alliance.RED_ALLIANCE && isBlue ||
                             alliance == FtcAuto.Alliance.BLUE_ALLIANCE && isRed)
                    {
                        robot.rightButtonPusher.setPosition(RobotInfo.BUTTON_PUSHER_EXTEND_POSITION);
                        rightPusherExtended = true;
                    }
                    robot.dashboard.displayPrintf(
                            15, "leftPusher=%s, rightPusher=%s",
                            Boolean.toString(leftPusherExtended), Boolean.toString(rightPusherExtended));
                    //
                    // It takes sometime for the button pusher to extend, set a timer to wait for it.
                    // TODO: add code to check color change and not wait the full 1.5 sec.
                    //
                    if (leftPusherExtended || rightPusherExtended)
                    {
                        timer.set(1.5, event);
                        sm.waitForSingleEvent(event, State.RETRACT);
                    }
                    else
                    {
                        sm.setState(State.NEXT_BEACON);
                    }
                    break;

                case RETRACT:
                    //
                    // Release the button pusher and retract the hanging hook.
                    //
                    if (leftPusherExtended)
                    {
                        robot.leftButtonPusher.setPosition(RobotInfo.BUTTON_PUSHER_RETRACT_POSITION);
                        leftPusherExtended = false;
                    }

                    if (rightPusherExtended)
                    {
                        robot.rightButtonPusher.setPosition(RobotInfo.BUTTON_PUSHER_RETRACT_POSITION);
                        rightPusherExtended = false;
                    }
                    //
                    // It takes sometime for the button pusher to retract, set a timer to wait for it.
                    //
                    timer.set(0.2, event);
                    sm.waitForSingleEvent(event, State.NEXT_BEACON);
                    break;

                case NEXT_BEACON:
                    if (remainingBeaconButtons == 2)
                    {
                        xDistance = 0.0;
                        yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? -48.0: 48.0;

                        robot.setTurnPID(xDistance, yDistance, heading);
                        robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                        remainingBeaconButtons--;
                        sm.waitForSingleEvent(event, State.ALIGN_WALL1);
                    }
                    else if (parkOption == FtcAuto.ParkOption.DO_NOTHING)
                    {
                        //
                        // Stay there, we are done!
                        //
                        sm.setState(State.DONE);
                    }
                    else
                    {
                        //
                        // We are going somewhere. let's get off the wall so we can turn.
                        //
                        xDistance = 12.0;
                        yDistance = 0.0;

                        robot.setTurnPID(xDistance, yDistance, heading);
                        robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                        sm.waitForSingleEvent(event, State.TURN_TO_CENTER);
                    }
                    break;

                case TURN_TO_CENTER:
                    xDistance = yDistance = 0.0;
                    if (beaconButtons == 2)
                    {
                        heading = alliance == FtcAuto.Alliance.RED_ALLIANCE? -135.0: -45.0;
                    }
                    else
                    {
                        heading = alliance == FtcAuto.Alliance.RED_ALLIANCE? -315.0: 45.0;
                    }

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.GOTO_CENTER);
                    break;

                case GOTO_CENTER:
                    xDistance = 0.0;
                    yDistance = beaconButtons == 2 && alliance == FtcAuto.Alliance.RED_ALLIANCE ||
                                beaconButtons == 1 && alliance == FtcAuto.Alliance.BLUE_ALLIANCE? 36.0: -36.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
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

}   //class AutoOpBeacon
