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

import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;

/**
 * This class implements a platform independent I2C device. Typically, this class is extended by a platform dependent
 * I2C device class. The platform dependent I2C device class must implement the abstract methods required by this
 * class. The abstract methods allow this class to perform platform independent operations on the I2C device.
 */
public abstract class TrcI2cDevice implements TrcTaskMgr.Task
{
    private static final String moduleName = "TrcI2cDevice";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    /**
     * This method checks if the I2C port is ready for bus transaction.
     *
     * @return true if port is ready, false otherwise.
     */
    public abstract boolean isPortReady();

    /**
     * This method checks if the I2C port is in write mode.
     *
     * @return true if port is in write mode, false otherwise.
     */
    public abstract boolean isPortInWriteMode();

    /**
     * This method sends a read command to the device.
     *
     * @param regAddress specifies the register address.
     * @param length specifies the number of bytes to read.
     */
    public abstract void sendReadCommand(int regAddress, int length);

    /**
     * This method sends a write command to the device.
     *
     * @param regAddress specifies the register address.
     * @param length specifies the number of bytes to write.
     * @param data specifies the data buffer containing the data to write to the device.
     */
    public abstract void sendWriteCommand(int regAddress, int length, byte[] data);

    /**
     * This method retrieves the data read from the device.
     *
     * @return byte array containing the data read.
     */
    public abstract byte[] getData();

    /**
     * The client of this class provides this interface if it wants to be notified when a read or write operation
     * has been completed.
     */
    public interface CompletionHandler
    {
        /**
         * This method is called when the read operation has been completed.
         *
         * @param regAddress specifies the starting register address.
         * @param length specifies the number of bytes read.
         * @param timestamp specified the timestamp of the data retrieved.
         * @param data specifies the data byte array.
         * @param timedout specifies true if the operation was timed out, false otherwise.
         * @return true if the request should be repeated, false otherwise.
         */
        boolean readCompletion(int regAddress, int length, double timestamp, byte[] data, boolean timedout);

        /**
         * This method is called when the write operation has been completed.
         *
         * @param regAddress specifies the starting register address.
         * @param length specifies the number of bytes read.
         * @param timedout specifies true if the operation was timed out, false otherwise.
         */
        void writeCompletion(int regAddress, int length, boolean timedout);

    }   //interface CompletionHandler

    /**
     * Specifies the Port Command state machine states.
     */
    private enum PortCommandState
    {
        //
        // Check the queue for next request.
        //
        START,
        //
        // Send the port command to the device.
        //
        SEND_PORT_COMMAND,
        //
        // Wait for port command to complete.
        //
        WAIT_PORT_COMMAND_COMPLETE,
        //
        // The request has been completed or we timed out.
        //
        PORT_COMMAND_COMPLETED,
        //
        // No more request in the queue, stop the state machine.
        //
        DONE
    }   //enum PortCommandState

    /**
     * This class implements an I2C device request. It can be a read or write request. This is implicitly indicated
     * by the writeBuffer field. The presence of a writeBuffer indicates it is a write request. It is a read request
     * otherwise.
     */
    private class Request
    {
        private int regAddress;
        private int length;
        private byte[] writeBuffer;
        private CompletionHandler handler;
        private double timeout;
        private boolean expired;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param regAddress specifies the register address.
         * @param length specifies the number of bytes to read or write.
         * @param writeBuffer specifies the write buffer, null if read operation.
         * @param handler specifies the completion handler to call when done. Can be null if none needed.
         * @param timeout specifies the timeout time. It can be set to 0 if there is no timeout.
         */
        public Request(int regAddress, int length, byte[] writeBuffer, CompletionHandler handler, double timeout)
        {
            this.regAddress = regAddress;
            this.length = length;
            this.writeBuffer = writeBuffer;
            this.handler = handler;
            this.timeout = timeout;
            this.expired = false;
        }   //Request

    }   //class Request

    private String instanceName;
    private TrcStateMachine<PortCommandState> portCommandSM;
    private Queue<Request> requestQueue = new LinkedList<>();
    private Request currRequest = null;
    private double expiredTime = 0.0;
    private byte[] dataRead = null;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcI2cDevice(String instanceName)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        portCommandSM = new TrcStateMachine<>(instanceName);
    }   //TrcI2cDevice

    /**
     * This method returns the instance name of the device.
     *
     * @return instance name.
     */
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method enables/disables the internal port command state machine and its task.
     *
     * @param enabled specifies true to enable the state machine and task, false otherwise.
     */
    private void setTaskEnabled(boolean enabled)
    {
        final String funcName = "setTaskEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (enabled)
        {
            TrcTaskMgr.getInstance().registerTask(instanceName, this, TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
            portCommandSM.start(PortCommandState.START);
        }
        else
        {
            portCommandSM.stop();
            TrcTaskMgr.getInstance().unregisterTask(this, TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }
    }   //setTaskEnabled

    /**
     * This method queues the read request.
     *
     * @param regAddress specifies the register address to read from.
     * @param length specifies the number of bytes to read.
     * @param handler specifies the completion handler to call when done. Can be null if none needed.
     * @param timeout specifies the timeout for the operation in seconds.
     */
    public void read(int regAddress, int length, CompletionHandler handler, double timeout)
    {
        final String funcName = "read";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "addr=%x,len=%d", regAddress, length);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        requestQueue.add(new Request(regAddress, length, null, handler, timeout));
        //
        // If the PortCommand state machine is not already active, start it.
        //
        if (!portCommandSM.isEnabled())
        {
            setTaskEnabled(true);
        }
    }   //read

    /**
     * This method queues the read request.
     *
     * @param regAddress specifies the register address to read from.
     * @param length specifies the number of bytes to read.
     * @param handler specifies the completion handler to call when done. Can be null if none needed.
     */
    public void read(int regAddress, int length, CompletionHandler handler)
    {
        read(regAddress, length, handler, 0.0);
    }   //read

    /**
     * This method queues the read request.
     *
     * @param regAddress specifies the register address to read from.
     * @param length specifies the number of bytes to read.
     */
    public void read(int regAddress, int length)
    {
        read(regAddress, length, null, 0.0);
    }   //read

    /**
     * This method queues the write request.
     *
     * @param regAddress specifies the register address to write to.
     * @param length specifies the number of bytes to read.
     * @param writeBuffer specifies the buffer containing the data to be written to the device.
     * @param handler specifies the completion handler to call when done. Can be null if none needed.
     * @param timeout specifies the timeout for the operation in seconds.
     */
    public void write(int regAddress, int length, byte[] writeBuffer, CompletionHandler handler, double timeout)
    {
        final String funcName = "write";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "addr=%x,len=%d", regAddress, length);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        requestQueue.add(new Request(regAddress, length, writeBuffer, handler, timeout));
        //
        // If the PortCommand state machine is not already active, start it.
        //
        if (!portCommandSM.isEnabled())
        {
            setTaskEnabled(true);
        }
    }   //write

    /**
     * This method queues the write request.
     *
     * @param regAddress specifies the register address to write to.
     * @param length specifies the number of bytes to read.
     * @param writeBuffer specifies the buffer containing the data to be written to the device.
     * @param handler specifies the completion handler to call when done.
     *                Can be null if none needed.
     */
    public void write(int regAddress, int length, byte[] writeBuffer, CompletionHandler handler)
    {
        write(regAddress, length, writeBuffer, handler, 0.0);
    }   //write

    /**
     * This method queues the write request.
     *
     * @param regAddress specifies the register address to write to.
     * @param length specifies the number of bytes to read.
     * @param writeBuffer specifies the buffer containing the data to be written to the device.
     */
    public void write(int regAddress, int length, byte[] writeBuffer)
    {
        write(regAddress, length, writeBuffer, null, 0.0);
    }   //write

    /**
     * This method sends a byte command to the device.
     *
     * @param regAddress specifies the register address to write to.
     * @param command specifies the command byte.
     */
    public void sendByteCommand(int regAddress, byte command)
    {
        final String funcName = "sendByteCommand";
        byte[] data = new byte[1];

        data[0] = command;
        write(regAddress, 1, data);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "command=%x", command);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //sendByteCommand

    /**
     * This method sends a 16-bit command to the device.
     *
     * @param regAddress specifies the register address to write to.
     * @param command specifies the 16-bit command.
     */
    public void sendWordCommand(int regAddress, short command)
    {
        final String funcName = "sendWordCommand";
        byte[] data = new byte[2];

        data[0] = (byte)(command & 0xff);
        data[1] = (byte)(command >> 8);
        write(regAddress, 2, data);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "command=%x", command);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //sendWordCommand

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
     * This method is called periodically to run the PortCommand state machines.
     *
     * @param runMode specifies the competition mode that is running.
     */
    @Override
    public void preContinuousTask(TrcRobot.RunMode runMode)
    {
        final String funcName = "preContinuousTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "runMode=%s", runMode.toString());
        }

        if (portCommandSM.isReady())
        {
            PortCommandState state = portCommandSM.getState();
            switch (state)
            {
                case START:
                    //
                    // Dequeue a request from the beginning of the queue.
                    //
                    currRequest = requestQueue.poll();
                    if (currRequest == null)
                    {
                        //
                        // There is no request in the queue, we are done.
                        //
                        portCommandSM.setState(PortCommandState.DONE);
                        break;
                    }
                    else
                    {
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "%s", state.toString());
                        }
                        expiredTime = currRequest.timeout;
                        if (expiredTime != 0.0)
                        {
                            expiredTime += TrcUtil.getCurrentTime();
                        }
                        currRequest.expired = false;
                        portCommandSM.setState(PortCommandState.SEND_PORT_COMMAND);
                        state = portCommandSM.getState();
                    }
                    //
                    // Intentionally falling through to next case.
                    //
                case SEND_PORT_COMMAND:
                    //
                    // Wait for the port to become ready before sending the command.
                    //
                    if (isPortReady())
                    {
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "%s: Request(addr=%x,len=%d,%s)",
                                               state.toString(), currRequest.regAddress,
                                               currRequest.length, currRequest.writeBuffer == null? "read": "write");
                        }

                        dataRead = null;
                        if (currRequest.writeBuffer == null)
                        {
                            //
                            // It's a read request, setup a read command.
                            //
                            sendReadCommand(currRequest.regAddress, currRequest.length);
                        }
                        else
                        {
                            //
                            // It's a write request, setup a write command.
                            //
                            sendWriteCommand(currRequest.regAddress, currRequest.length, currRequest.writeBuffer);
                        }
                        portCommandSM.setState(PortCommandState.WAIT_PORT_COMMAND_COMPLETE);
                    }
                    else if (expiredTime != 0.0 && TrcUtil.getCurrentTime() > expiredTime)
                    {
                        currRequest.expired = true;
                        portCommandSM.setState(PortCommandState.PORT_COMMAND_COMPLETED);
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "%s: Port timed out, busy with another request.",
                                               state.toString());
                        }
                    }
                    break;

                case WAIT_PORT_COMMAND_COMPLETE:
                    //
                    // Wait for the port command to complete or timed out.
                    //
                    if (isPortReady())
                    {
                        if (currRequest.writeBuffer != null)
                        {
                            //
                            // It is a write request, the request is completed.
                            //
                            portCommandSM.setState(PortCommandState.PORT_COMMAND_COMPLETED);
                            if (debugEnabled)
                            {
                                dbgTrace.traceInfo(funcName, "%s: write command completed.", state.toString());
                            }
                        }
                        else
                        {
                            dataRead = getData();
                            //
                            // It is a read request. For some reason, even when isPortReady() returns true, the data
                            // may not be ready. So we need to check the buffer length against the requested length.
                            // If it's not ready, remain in this state until we have valid data or timed out.
                            //
                            if (dataRead.length == currRequest.length)
                            {
                                //
                                // We have valid data, the request is completed.
                                //
                                portCommandSM.setState(PortCommandState.PORT_COMMAND_COMPLETED);
                                if (debugEnabled)
                                {
                                    dbgTrace.traceInfo(funcName, "%s: read command completed. %s",
                                                       state.toString(), Arrays.toString(dataRead));
                                }
                            }
                            else if (expiredTime != 0.0 && TrcUtil.getCurrentTime() > expiredTime)
                            {
                                currRequest.expired = true;
                                portCommandSM.setState(PortCommandState.PORT_COMMAND_COMPLETED);
                                if (debugEnabled)
                                {
                                    dbgTrace.traceInfo(funcName, "%s: Port command timed out.", state.toString());
                                }
                            }
                        }
                    }
                    else if (expiredTime != 0.0 && TrcUtil.getCurrentTime() > expiredTime)
                    {
                        currRequest.expired = true;
                        portCommandSM.setState(PortCommandState.PORT_COMMAND_COMPLETED);
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "%s: Port command timed out.", state.toString());
                        }
                    }
                    break;

                case PORT_COMMAND_COMPLETED:
                    //
                    // The port command is complete, call completion handler if any.
                    //
                    if (debugEnabled)
                    {
                        dbgTrace.traceInfo(funcName, "%s: Command completed (timeout=%s).",
                                           state.toString(), Boolean.toString(currRequest.expired));
                    }

                    if (currRequest.handler != null)
                    {
                        if (currRequest.writeBuffer == null)
                        {
                            if (currRequest.handler.readCompletion(currRequest.regAddress, currRequest.length,
                                                                   TrcUtil.getCurrentTime(), dataRead,
                                                                   currRequest.expired))
                            {
                                //
                                // Repeat this read request.
                                //
                                requestQueue.add(currRequest);
                            }
                        }
                        else
                        {
                            currRequest.handler.writeCompletion(currRequest.regAddress, currRequest.length,
                                                                currRequest.expired);
                        }
                    }
                    portCommandSM.setState(PortCommandState.START);
                    break;

                case DONE:
                default:
                    //
                    // There is no more request in the queue, stop the state machine.
                    //
                    if (debugEnabled)
                    {
                        dbgTrace.traceInfo(funcName, "%s", state.toString());
                    }
                    setTaskEnabled(false);
                    break;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //preContinuousTask

    @Override
    public void postContinuousTask(TrcRobot.RunMode runMode)
    {
    }   //postContinuousTask

}   //class TrcI2cDevice
