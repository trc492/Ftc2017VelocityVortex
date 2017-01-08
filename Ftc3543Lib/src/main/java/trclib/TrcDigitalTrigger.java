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

package trclib;

/**
 * This class implements a digital trigger. A digital trigger consists of a digital input device. It monitors the
 * device state and calls the notification handler if the state changes.
 */
public class TrcDigitalTrigger implements TrcTaskMgr.Task
{
    private static final String moduleName = "TrcDigitalTrigger";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    /**
     * This interface contains the method for the trigger event handler.
     */
    public interface TriggerHandler
    {
        /**
         * This method is called when the digital input device has changed state.
         *
         * @param digitalTrigger specifies this DigitalTrigger instance as the source of the event.
         * @param active specifies true if the digital device state is active, false otherwise.
         */
        void DigitalTriggerEvent(TrcDigitalTrigger digitalTrigger, boolean active);

    }   //interface TriggerHandler

    private String instanceName;
    private TrcDigitalInput digitalInput;
    private TriggerHandler eventHandler;
    private boolean prevState = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param digitalInput specifies the digital input device.
     * @param eventHandler specifies the object that will be called to handle the digital input device state change.
     */
    public TrcDigitalTrigger(final String instanceName, TrcDigitalInput digitalInput, TriggerHandler eventHandler)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        if (digitalInput == null || eventHandler == null)
        {
            throw new NullPointerException("DigitalInput/EventHandler must be provided");
        }

        this.instanceName = instanceName;
        this.digitalInput = digitalInput;
        this.eventHandler = eventHandler;
    }   //TrcDigitalTrigger

    /**
     * This method enables/disables the task that monitors the device state.
     *
     * @param enabled specifies true to enable the task, false to disable.
     */
    public void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }

        TrcTaskMgr taskMgr = TrcTaskMgr.getInstance();
        if (enabled)
        {
            taskMgr.registerTask(instanceName, this, TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }
        else
        {
            taskMgr.unregisterTask(this, TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }
    }   //setEnabled

    //
    // Implements TrcTaskMgr.Task
    //

    @Override
    public void startTask(TrcRobot.RunMode runMode)
    {
    }   //startTask

    @Override
    public void stopTask(TrcRobot.RunMode runMode)
    {
    }   //stopTask

    @Override
    public void prePeriodicTask(TrcRobot.RunMode runMode)
    {
    }   //prePeriodicTask

    @Override
    public void postPeriodicTask(TrcRobot.RunMode runMode)
    {
    }   //postPeriodicTask

    /**
     * This method is called periodically to check if the digital input device has changed state.
     *
     * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
     */
    @Override
    public void preContinuousTask(TrcRobot.RunMode runMode)
    {
        final String funcName = "preContinuousTask";
        boolean currState = digitalInput.isActive();

        if (currState != prevState)
        {
            if (eventHandler != null)
            {
                eventHandler.DigitalTriggerEvent(this, currState);
            }
            prevState = currState;

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "%s triggered (state=%s)", instanceName, Boolean.toString(currState));
            }
        }
    }   //preContinuousTask

    @Override
    public void postContinuousTask(TrcRobot.RunMode runMode)
    {
    }   //postContinuousTask

}   //class TrcDigitalTrigger
