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

class CmdAuto100 implements TrcRobot.RobotCommand
{
    private static final boolean debugXPid = false;
    private static final boolean debugYPid = false;
    private static final boolean debugTurnPid = false;

    private enum State
    {
        NEAR_START,
        TURN_TO_WALL,
        GOTO_WALL,
        PARALLEL_WALL,
        ALIGN_WALL1,
        ALIGN_WALL2,
        ALIGN_WALL3,
        FIND_LINE,
        PUSH_BUTTON1,
        PUSH_BUTTON2,
        RETRACT,
        NEXT_BEACON,
        GOTO_VORTEX,
        TURN_TO_VORTEX,
        PARK_VORTEX,
        DONE
    }   //enum State

    private static final String moduleName = "CmdAuto100";

    private Robot robot;
    private FtcAuto.Alliance alliance;
    private FtcAuto.ParkOption parkOption;
    private int numBeaconButtons;
    private int remainingBeaconButtons;
    private boolean nearBeacon;
    private CmdNearStart nearStartCmd;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;
    private boolean leftPusherExtended = false;
    private boolean rightPusherExtended = false;

    CmdAuto100(Robot robot,
               FtcAuto.Alliance alliance,
               double delay,
               int numParticles,
               FtcAuto.ParkOption parkOption,
               int numBeaconButtons,
               boolean usePath1)
    {
        this.robot = robot;
        this.alliance = alliance;
        this.parkOption = parkOption;
        this.numBeaconButtons = numBeaconButtons;

        remainingBeaconButtons = numBeaconButtons;
        nearBeacon = !usePath1 || numBeaconButtons == 1;
        nearStartCmd = new CmdNearStart(robot, alliance, delay, numParticles, nearBeacon);

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.NEAR_START);
    }   //CmdAuto100

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean done = !sm.isEnabled();
        //
        // Print debug info.
        //
        State state = sm.getState();
        if (state != State.NEAR_START)
        {
            robot.dashboard.displayPrintf(1, "State: %s", state != null? state.toString(): "Disabled");
        }

        if (Robot.USE_LINE_DETECTOR)
        {
            if (Robot.USE_ODS_LINE_DETECTOR)
            {
                robot.dashboard.displayPrintf(2, "LineDetect: light=%.3f",
                                              robot.odsLineDetector.sensor.getRawLightDetected());
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

        if (robot.pidDrive.isActive())
        {
            robot.encoderXPidCtrl.displayPidInfo(4);
            robot.encoderYPidCtrl.displayPidInfo(6);
            robot.gyroPidCtrl.displayPidInfo(8);
        }

        if (sm.isReady())
        {
            state = sm.getState();
            State nextState;
            double xDistance = 0.0, yDistance = 0.0;
            int redValue, greenValue, blueValue;
            boolean isRed, isBlue;
            boolean printStateInfo = true;

            switch (state)
            {
                case NEAR_START:
                    //
                    // The robot starts at near the corner vortex. nearStartCmd is a common segment shared by
                    // several autonomous strategies. This state will run that segment until its completion when
                    // it returns true, then we move on to the next state. nearStartCmd would shoot a specified
                    // number of particles and displace the Cap Ball.
                    //
                    printStateInfo = false;
                    if (nearStartCmd.cmdPeriodic(elapsedTime))
                    {
                        sm.setState(State.TURN_TO_WALL);
                    }
                    break;

                case TURN_TO_WALL:
                    //
                    // After displacing the Cap Ball, turn the robot towards the wall.
                    //
                    xDistance = yDistance = 0.0;
                    robot.targetHeading = alliance == FtcAuto.Alliance.RED_ALLIANCE? -90.0: 90.0;

                    robot.setPIDDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.GOTO_WALL);
                    break;

                case GOTO_WALL:
                    //
                    // Go forward to the wall.
                    //
                    xDistance = 0.0;
                    yDistance = nearBeacon? 22.0: 18.0;

                    robot.setPIDDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.PARALLEL_WALL);
                    break;

                case PARALLEL_WALL:
                    //
                    // Turn the robot to parallel the wall.
                    //
                    xDistance = yDistance = 0.0;
                    robot.targetHeading = alliance == FtcAuto.Alliance.RED_ALLIANCE? 0.0: 180.0;

                    robot.setPIDDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.ALIGN_WALL1);
                    break;

                case ALIGN_WALL1:
                    //
                    // Strafe left with full power to hit the wall. If the robot was misaligned with the wall, this
                    // will align it again. We use the range sensor as a stopping condition. In addition, we also
                    // set a timeout time to make sure we will move on even if the range sensor has malfunctioned.
                    //
                    robot.driveBase.mecanumDrive_Cartesian(-1.0, 0.0, 0.0);
                    timer.set(1.5, event);
                    sm.setState(State.ALIGN_WALL2);
                    break;

                case ALIGN_WALL2:
                    //
                    // Continue to strafe left until either the range sensor said we are less than 3.5-inch away
                    // from the wall or the timeout has expired.
                    //
                    if (robot.getInput(robot.rangePidCtrl) < 3.5)
                    {
                        timer.set(0.5, event);
                        sm.setState(State.ALIGN_WALL3);
                    }
                    else if (event.isSignaled())
                    {
                        sm.setState(State.ALIGN_WALL3);
                    }
                    break;

                case ALIGN_WALL3:
                    //
                    // We are done aligning with the wall. Stop the robot and delay just a little to allow the gyro
                    // to settle.
                    //
                    robot.driveBase.mecanumDrive_Cartesian(0.0, 0.0, 0.0);

//                    if (Robot.USE_COLOR_SENSOR)
//                    {
//                        robot.beaconColorSensor.setDeviceEnabled(true);
//                    }

                    timer.set(0.1, event);
                    sm.waitForSingleEvent(event, State.FIND_LINE);
                    break;

                case FIND_LINE:
                    //
                    // We should be aligned with the wall, maintain the current heading when running along the wall
                    // looking for the white line. Limit the robot to only 13% power so we won't miss the line or
                    // overshoot too much when detecting the line.
                    //
                    xDistance = 0.0;
                    yDistance = nearBeacon && alliance == FtcAuto.Alliance.RED_ALLIANCE ||
                                !nearBeacon && alliance == FtcAuto.Alliance.BLUE_ALLIANCE? -35.0: 35.0;
                    if (numBeaconButtons == 2 && remainingBeaconButtons == 1)
                    {
                        //
                        // We are going to the 2nd of the 2 beacons, so the direction is reversed.
                        //
                        yDistance = -yDistance;
                        nearBeacon = !nearBeacon;
                    }
                    robot.targetHeading = robot.driveBase.getHeading();

                    if (Robot.USE_LINE_DETECTOR)
                    {
                        if (Robot.USE_ODS_LINE_DETECTOR)
                        {
                            robot.odsTrigger.setEnabled(true);
                        }
                        else
                        {
                            robot.colorTrigger.setEnabled(true);
                        }
                    }

                    robot.encoderYPidCtrl.setOutputRange(-0.13, 0.13);
                    robot.setPIDDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.PUSH_BUTTON1);
                    break;

                case PUSH_BUTTON1:
                    //
                    // We found the line. Let's check the beacon color and press the appropriate button.
                    //
                    robot.encoderYPidCtrl.setOutputRange(-1.0, 1.0);

                    if (Robot.USE_LINE_DETECTOR)
                    {
                        if (Robot.USE_ODS_LINE_DETECTOR)
                        {
                            robot.odsTrigger.setEnabled(false);
                        }
                        else
                        {
                            robot.colorTrigger.setEnabled(false);
                        }
                    }

                    if (Robot.USE_COLOR_SENSOR)
                    {
                        redValue = robot.beaconColorSensor.sensor.red();
                        greenValue = robot.beaconColorSensor.sensor.green();
                        blueValue = robot.beaconColorSensor.sensor.blue();
                    }

                    isRed = redValue > blueValue && redValue > greenValue;
                    isBlue = blueValue > redValue && blueValue > greenValue;
                    robot.dashboard.displayPrintf(
                            14, "[%d,%d,%d]isRed=%s,isBlue=%s",
                            redValue, greenValue, blueValue, Boolean.toString(isRed), Boolean.toString(isBlue));
                    robot.tracer.traceInfo(
                            state.toString(), "[%d,%d,%d]isRed=%s,isBlue=%s",
                            redValue, greenValue, blueValue, Boolean.toString(isRed), Boolean.toString(isBlue));
                    //
                    // Determine which button to push and do it.
                    //
                    if (alliance == FtcAuto.Alliance.RED_ALLIANCE && isRed ||
                        alliance == FtcAuto.Alliance.BLUE_ALLIANCE && isBlue)
                    {
                        //
                        // It takes sometime for the button pusher to extend, set a timer to wait for it.
                        //
//                        robot.beaconColorSensor.setDeviceEnabled(false);
                        robot.leftButtonPusher.setPosition(RobotInfo.BUTTON_PUSHER_EXTEND_POSITION);
                        leftPusherExtended = true;
                        timer.set(1.5, event);
                        sm.waitForSingleEvent(event, State.RETRACT);
                    }
                    else if (alliance == FtcAuto.Alliance.RED_ALLIANCE && isBlue ||
                             alliance == FtcAuto.Alliance.BLUE_ALLIANCE && isRed)
                    {
                        //
                        // Since the color sensor was detecting a different color than our alliance color, we don't
                        // have to set a timer and blindly wait. We could just wait for the color change. Once the
                        // color has changed to our alliance color, we could abort the button pusher and move onto
                        // the next state. That will save some time. However, we are still setting the timeout for
                        // safety measure. In case the color doesn't change, 1.5-second is all we can afford to
                        // waste.
                        //
                        robot.rightButtonPusher.setPosition(RobotInfo.BUTTON_PUSHER_EXTEND_POSITION);
                        rightPusherExtended = true;
                        timer.set(1.5, event);
                        sm.setState(State.PUSH_BUTTON2);
                    }
                    else
                    {
                        //
                        // Cannot determine the color, skip it.
                        //
                        sm.setState(State.NEXT_BEACON);
                    }
                    robot.dashboard.displayPrintf(
                            15, "leftPusher=%s, rightPusher=%s",
                            Boolean.toString(leftPusherExtended), Boolean.toString(rightPusherExtended));
                    break;

                case PUSH_BUTTON2:
                    //
                    // We will only come to this state if the color sensor detecting a color different from our
                    // alliance color. We keep checking the color of the beacon. If it has changed to our color
                    // or time has expired, move on to the next state.
                    //
                    if (Robot.USE_COLOR_SENSOR)
                    {
                        redValue = robot.beaconColorSensor.sensor.red();
                        greenValue = robot.beaconColorSensor.sensor.green();
                        blueValue = robot.beaconColorSensor.sensor.blue();
                        isRed = redValue > blueValue && redValue > greenValue;
                        isBlue = blueValue > redValue && blueValue > greenValue;
                    }
                    boolean timedOut = event.isSignaled();
                    robot.tracer.traceInfo(
                            state.toString(), "[%d,%d,%d]isRed=%s,isBlue=%s,expired=%s",
                            redValue, greenValue, blueValue, Boolean.toString(isRed), Boolean.toString(isBlue),
                            Boolean.toString(timedOut));
                    if (timedOut ||
                        alliance == FtcAuto.Alliance.RED_ALLIANCE && isRed ||
                        alliance == FtcAuto.Alliance.BLUE_ALLIANCE && isBlue)
                    {
//                        robot.beaconColorSensor.setDeviceEnabled(false);
                        sm.setState(State.RETRACT);
                    }
                    break;

                case RETRACT:
                    //
                    // We need to retract the pusher a little bit before start moving so it doesn't get caught on
                    // by the beacon.
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

                    timer.set(0.2, event);
                    sm.waitForSingleEvent(event, State.NEXT_BEACON);
                    break;

                case NEXT_BEACON:
                    //
                    // Determine if we are done pushing beacon buttons or if we have another beacon button to push.
                    //
                    if (remainingBeaconButtons == 2)
                    {
                        //
                        // We have another button to push, go to the next beacon.
                        //
                        xDistance = 0.0;
                        yDistance = nearBeacon && alliance == FtcAuto.Alliance.RED_ALLIANCE ||
                                    !nearBeacon && alliance == FtcAuto.Alliance.BLUE_ALLIANCE? 40.0: -40.0;

                        robot.setPIDDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
                        remainingBeaconButtons--;
                        sm.waitForSingleEvent(event, State.ALIGN_WALL1);
                    }
                    else if (parkOption == FtcAuto.ParkOption.DO_NOTHING)
                    {
                        //
                        // We are done pushing beacon buttons and the ParkOption is none, so we are done.
                        //
                        sm.setState(State.DONE);
                    }
                    else
                    {
                        //
                        // We are going to park somewhere. let's get off the wall so we can run or turn to our
                        // parking place.
                        // If we are at near beacon and want to park at the center, we are already aligned to the
                        // center vortex, we just need to turn towards it.
                        //
                        xDistance = nearBeacon && parkOption != FtcAuto.ParkOption.PARK_CENTER? 24.0: 12.0;
                        yDistance = 0.0;
                        nextState = nearBeacon && parkOption == FtcAuto.ParkOption.PARK_CENTER?
                                        State.TURN_TO_VORTEX: State.GOTO_VORTEX;

                        robot.setPIDDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
                        sm.waitForSingleEvent(event, nextState);
                    }
                    break;

                case GOTO_VORTEX:
                    //
                    // Go towards the vortexes.
                    //
                    xDistance = yDistance = 0.0;
                    if (nearBeacon)
                    {
                        //
                        // If we come here, we are at near beacon and want to park at the corner. We just run
                        // straight to it and we are done.
                        //
                        yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? -64.0: 30.0;
                        nextState = State.DONE;
                    }
                    else if (parkOption == FtcAuto.ParkOption.PARK_CENTER)
                    {
                        //
                        // We are at far beacon and want to park at the center, turn toward the center.
                        //
                        robot.targetHeading = alliance == FtcAuto.Alliance.RED_ALLIANCE? -45.0: 225.0;
                        nextState = State.PARK_VORTEX;
                    }
                    else
                    {
                        //
                        // We are at far beacon and want to park at the corner, run straight to it and we are done.
                        //
                        yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? -84.0: 84.0;
                        nextState = State.DONE;
                    }

                    robot.setPIDDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case TURN_TO_VORTEX:
                    //
                    // If we come here, we are at near beacon and want to park at the center vortex. Turn the robot
                    // to face the vortexes.
                    //
                    xDistance = yDistance = 0.0;
                    robot.targetHeading = alliance == FtcAuto.Alliance.RED_ALLIANCE? 90.0: 270.0;

                    robot.setPIDDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.PARK_VORTEX);
                    break;

                case PARK_VORTEX:
                    //
                    // If we come here, we are at near or far beacon and want to park at the center vortex.
                    //
                    xDistance = 0.0;
                    yDistance = nearBeacon? 52.0: alliance == FtcAuto.Alliance.RED_ALLIANCE? -52.0: 52.0;

                    robot.setPIDDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
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

}   //class CmdAuto100
