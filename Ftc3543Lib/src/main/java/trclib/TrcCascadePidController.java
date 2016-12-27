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

public class TrcCascadePidController extends TrcPidController implements TrcPidController.PidInput
{
    private static final String moduleName = "TrcCascadePidController";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    public interface CascadeInput
    {
        double getPrimaryInput(TrcCascadePidController cascadeCtrl);
        double getSecondaryInput(TrcCascadePidController cascadeCtrl);
    }   //interface CascadeInput

    public TrcPidController secondaryCtrl;
    private CascadeInput cascadeInput;

    public TrcCascadePidController(
            final String instanceName,
            double       primaryKp,
            double       primaryKi,
            double       primaryKd,
            double       primaryKf,
            double       primaryTolerance,
            double       primarySettlingTime,
            double       secondaryKp,
            double       secondaryKi,
            double       secondaryKd,
            double       secondaryKf,
            double       secondaryTolerance,
            double       secondarySettlingTime,
            CascadeInput cascadeInput)
    {
        super(instanceName + ".primary",
              primaryKp, primaryKi, primaryKd, primaryKf,
              primaryTolerance, primarySettlingTime);
        super.setPidInput(this);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        secondaryCtrl = new TrcPidController(instanceName + ".secondary",
                                             secondaryKp, secondaryKi, secondaryKd, secondaryKf,
                                             secondaryTolerance, secondarySettlingTime, this);
        this.cascadeInput = cascadeInput;
    }   //TrcCascadePidController

    @Override
    public void reset()
    {
        final String funcName = "reset";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        secondaryCtrl.reset();
        super.reset();
    }   //reset

    @Override
    public double getOutput()
    {
        final String funcName = "getOutput";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        double primaryOutput = super.getOutput();
        secondaryCtrl.setTarget(primaryOutput);
        double secondaryOutput = secondaryCtrl.getOutput();

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=(primary:%f,secondary:%f",
                               primaryOutput, secondaryOutput);
        }

        return secondaryOutput;
    }   //getOutput

    //
    // Implements TrcPidController.PidInput
    //

    @Override
    public double getInput(TrcPidController pidCtrl)
    {
        double input = 0.0;

        if (pidCtrl == this)
        {
            input = cascadeInput.getPrimaryInput(this);
        }
        else if (pidCtrl == secondaryCtrl)
        {
            input = cascadeInput.getSecondaryInput(this);
        }

        return input;
    }   //getInput

}   //class TrcCascadePidController
