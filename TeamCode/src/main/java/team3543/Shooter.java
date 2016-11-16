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

import ftclib.FtcDcMotor;
import ftclib.FtcServo;
import ftclib.FtcTouchSensor;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPidMotor;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;
import trclib.TrcTimer;

public class Shooter implements TrcTaskMgr.Task, TrcPidController.PidInput
{
    private enum ShooterState
    {
        LOAD_PARTICLE,
        ARM_AND_FIRE,
        PULL_BACK,
        DONE
    }   //enum ShooterState

    private String instanceName;
    private FtcDcMotor shooterMotor;
    private TrcPidController pidCtrl;
    private TrcPidMotor pidMotor;
    private FtcTouchSensor touchSensor;
    private FtcServo ballGate;
    private TrcStateMachine sm;
    private TrcTimer timer;
    private TrcEvent event;
    private TrcEvent completionEvent = null;
    private boolean continuousModeOn = false;

    public Shooter(String instanceName)
    {
        this.instanceName = instanceName;

        shooterMotor = new FtcDcMotor("shooterMotor");
        shooterMotor.setInverted(true);
        shooterMotor.setBrakeModeEnabled(true);
        pidCtrl = new TrcPidController(
                instanceName,
                RobotInfo.SHOOTER_KP, RobotInfo.SHOOTER_KI, RobotInfo.SHOOTER_KD, RobotInfo.SHOOTER_KF,
                RobotInfo.SHOOTER_TOLERANCE, RobotInfo.SHOOTER_SETTLING, this);
        pidCtrl.setNoOscillation(true);
        pidMotor = new TrcPidMotor(instanceName, shooterMotor, pidCtrl);
        pidMotor.setPositionScale(RobotInfo.SHOOTER_DEGREES_PER_COUNT);
        touchSensor = new FtcTouchSensor("shooterTouchSensor");

        ballGate = new FtcServo("ballGateServo");
        ballGate.setPosition(RobotInfo.BALLGATE_CLOSE_POSITION);

        sm = new TrcStateMachine(instanceName);
        timer = new TrcTimer(instanceName);
        event = new TrcEvent(instanceName);
    }

    private void setEnabled(boolean enabled)
    {
        if (enabled)
        {
            TrcTaskMgr.getInstance().registerTask(instanceName, this, TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
            TrcTaskMgr.getInstance().registerTask(instanceName, this, TrcTaskMgr.TaskType.STOP_TASK);
        }
        else
        {
            TrcTaskMgr.getInstance().unregisterTask(this, TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
            TrcTaskMgr.getInstance().unregisterTask(this, TrcTaskMgr.TaskType.STOP_TASK);
        }
    }

    public void stop()
    {
        if (sm.isEnabled())
        {
            sm.stop();
        }
        setEnabled(false);
        continuousModeOn = false;
        pidMotor.cancel();
    }

    public double getPosition()
    {
        return pidMotor.getPosition();
    }

    public boolean isTouchActive()
    {
        return touchSensor.isActive();
    }

    public void setPower(double power)
    {
        shooterMotor.setPower(power);
    }

    public void setBallGatePosition(double position)
    {
        ballGate.setPosition(position);
    }

    private void fire(ShooterState startState, boolean continuous, TrcEvent event)
    {
        continuousModeOn = continuous;
        this.completionEvent = event;
        if (sm.isEnabled())
        {
            stop();
        }
        sm.start(startState);
        setEnabled(true);
    }

    public void fireOneShot(TrcEvent event)
    {
        fire(ShooterState.ARM_AND_FIRE, false, event);
    }

    public void fireOneShot()
    {
        fire(ShooterState.ARM_AND_FIRE, false, null);
    }

    public void loadAndFireOneShot(TrcEvent event)
    {
        fire(ShooterState.LOAD_PARTICLE, false, event);
    }

    public void loadAndFireOneShot()
    {
        fire(ShooterState.LOAD_PARTICLE, false, null);
    }

    public void fireContinuous(boolean on)
    {
        continuousModeOn = on;
        if (on)
        {
            fire(ShooterState.LOAD_PARTICLE, true, null);
        }
    }

    //
    // Implements TrcTaskMgr.Task.
    //

    @Override
    public void startTask(TrcRobot.RunMode runMode)
    {
    }

    @Override
    public void stopTask(TrcRobot.RunMode runMode)
    {
        stop();
    }

    @Override
    public void prePeriodicTask(TrcRobot.RunMode runMode)
    {
    }

    @Override
    public void postPeriodicTask(TrcRobot.RunMode runMode)
    {
    }

    @Override
    public void preContinuousTask(TrcRobot.RunMode runMode)
    {
    }

    @Override
    public void postContinuousTask(TrcRobot.RunMode runMode)
    {
        pidCtrl.displayPidInfo(3);

        if (sm.isReady())
        {
            ShooterState state = (ShooterState)sm.getState();

            switch (state)
            {
                case LOAD_PARTICLE:
                    ballGate.setPosition(RobotInfo.BALLGATE_OPEN_POSITION);
                    timer.set(RobotInfo.SHOOTER_BALLGATE_OPEN_TIME, event);
                    sm.addEvent(event);
                    sm.waitForEvents(ShooterState.ARM_AND_FIRE);
                    break;

                case ARM_AND_FIRE:
                    ballGate.setPosition(RobotInfo.BALLGATE_CLOSE_POSITION);
                    shooterMotor.setPower(RobotInfo.SHOOTER_POWER);
                    if (touchSensor.isActive())
                    {
                        shooterMotor.setPower(0.0);
                        shooterMotor.resetPosition();
                        timer.set(RobotInfo.SHOOTER_PAUSE_TIME, event);
                        sm.addEvent(event);
                        sm.waitForEvents(ShooterState.PULL_BACK);
                    }
                    break;

                case PULL_BACK:
                    pidMotor.setTarget(RobotInfo.SHOOTER_PULLBACK_TARGET, event, 0.5);
                    sm.addEvent(event);
                    sm.waitForEvents(continuousModeOn? ShooterState.LOAD_PARTICLE: ShooterState.DONE);
                    break;

                default:
                case DONE:
                    if (completionEvent != null)
                    {
                        completionEvent.set(true);
                        completionEvent = null;
                    }
                    sm.stop();
                    setEnabled(false);
                    break;
            }
        }
    }

    //
    // Implements TrcPidController.PidInput
    //

    @Override
    public double getInput(TrcPidController pidCtrl)
    {
        double input = 0.0;

        if (pidCtrl == this.pidCtrl)
        {
            input = pidMotor.getPosition();
        }

        return input;
    }   //getInput

}   //class Shooter
