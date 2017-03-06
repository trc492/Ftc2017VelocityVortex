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

package trclib;

/**
 * This class implements a platform independent vision task. When enabled, it grabs a frame from the video source,
 * calls the provided object detector to process the frame and overlays rectangles on the detected objects in the
 * image. This class is to be extended by a platform dependent vision processor who will provide the video input
 * and output. 
 *
 * @param <I> specifies the type of the input image.
 * @param <O> specifies the type of the detected objects.
 */
public class TrcVisionTask<I, O> implements TrcThread.PeriodicTask
{
    private static final String moduleName = "TrcVisionTask";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    /**
     * This interface provides methods to grab image from the video input, render image to video output and detect
     * objects from the acquired image.
     *
     * @param <I> specifies the type of the input image.
     * @param <O> specifies the type of the detected object.
     */
    public interface VisionProcessor<I, O>
    {
        /**
         * This method is called to grab an image frame from the video input.
         *
         * @param image specifies the frame buffer to hold the captured image.
         * @return true if frame is successfully captured, false otherwise.
         */
        boolean grabFrame(I image);

        /**
         * This method is called to detect objects in the acquired image frame.
         *
         * @param image specifies the image to be processed.
         * @param detectedObjects specifies the preallocated buffer to hold the detected objects, can be null if no
         *        preallocated buffer required.
         * @return detected objects, null if none detected.
         */
        O detectObjects(I image, O detectedObjects);

    }   //interface VisionProcessor

    private TrcDbgTrace tracer = null;
    private long totalTime = 0;
    private long totalFrames = 0;
    private double taskStartTime = 0.0;

    private final String instanceName;
    private VisionProcessor<I, O> visionProcessor;
    private I[] imageBuffers;
    private O[] detectedObjectBuffers;
    private int imageIndex = 0;
    private int bufferIndex = 0;
    private TrcThread<O> visionTask;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param visionProcessor specifies the vision processor object.
     * @param imageBuffers specifies an array of image buffers.
     * @param detectedObjectBuffers specifies an array of buffers to hold the detected objects.
     */
    public TrcVisionTask(
        final String instanceName, VisionProcessor<I, O> visionProcessor, I[] imageBuffers, O[] detectedObjectBuffers)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        this.visionProcessor = visionProcessor;
        this.imageBuffers = imageBuffers;
        this.detectedObjectBuffers = detectedObjectBuffers;
        visionTask = new TrcThread<>(instanceName, this);
    }   //TrcVisionTask

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method enables/disables vision processing performance report.
     *
     * @param tracer specifies a tracer to enable performance report, null to disable.
     */
    public void setPerfReportEnabled(TrcDbgTrace tracer)
    {
        this.tracer = tracer;
    }   //setPerfReportEnabled

    /**
     * This method is called to terminate the vision task. Once this is called, no other method in this class
     * should be called except for isTaskTerminated().
     */
    public void terminateTask()
    {
        visionTask.terminateTask();
    }   //terminateTask

    /**
     * This method checks if the vision task has been terminated.
     *
     * @return true if vision task is terminated, false otherwise.
     */
    public boolean isTaskTerminated()
    {
        return visionTask.isTaskTerminated();
    }   //isTaskTerminated

    /**
     * This method enables/disables the vision task. As long as the task is enabled, it will continue to
     * acquire/process images.
     *
     * @param enabled specifies true to enable vision task, false to disable.
     */
    public void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
        }

        visionTask.setTaskEnabled(enabled);
        if (enabled)
        {
            totalTime = 0;
            totalFrames = 0;
            taskStartTime = TrcUtil.getCurrentTime();
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setEnabled

    /**
     * This method returns the state of the vision task.
     *
     * @return true if the vision task is enabled, false otherwise.
     */
    public boolean isEnabled()
    {
        final String funcName = "isEnabled";
        boolean enabled = visionTask.isTaskEnabled();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(enabled));
        }

        return enabled;
    }   //isEnabled

    /**
     * This method sets the vision task processing interval.
     *
     * @param interval specifies the processing interval in msec. If 0, process as fast as the CPU can run.
     */
    public void setProcessingInterval(long interval)
    {
        final String funcName = "setProcessingInterval";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "interval=%dms", interval);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        visionTask.setProcessingInterval(interval);
    }   //setProcessInterval

    /**
     * This method returns the vision task processing interval.
     *
     * @return vision task processing interval in msec.
     */
    public long getProcessingInterval()
    {
        final String funcName = "getProcessingInterval";
        long interval = visionTask.getProcessingInterval();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", interval);
        }

        return interval;
    }   //getProcessingInterval

    //
    // Implements TrcThread.PeriodicTask interface.
    //

    /**
     * This method runs the vision processing task.
     */
    @Override
    public void runPeriodic()
    {
        final String funcName = "runPeriodic";
        double startTime;
        double elapsedTime;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK);
        }

        if (visionProcessor.grabFrame(imageBuffers[imageIndex]))
        {
            //
            // Capture an image and subject it for object detection. The object detector produces an array of
            // rectangles representing objects detected.
            //
            startTime = TrcUtil.getCurrentTimeMillis();
            visionProcessor.detectObjects(
                imageBuffers[imageIndex], detectedObjectBuffers != null? detectedObjectBuffers[bufferIndex]: null);
            elapsedTime = TrcUtil.getCurrentTimeMillis() - startTime;
            totalTime += elapsedTime;
            totalFrames++;
            if (tracer != null)
            {
                tracer.traceInfo(funcName, "Average processing time = %.3f msec, Frame rate = %.1f",
                    (double)totalTime/totalFrames, totalFrames/(TrcUtil.getCurrentTime() - taskStartTime));
            }
            //
            // Switch to the next buffer so that we won't clobber the info while the client is accessing it.
            //
            imageIndex = (imageIndex + 1)%imageBuffers.length;
            if (detectedObjectBuffers != null)
            {
                visionTask.setData(detectedObjectBuffers[bufferIndex]);
                bufferIndex = (bufferIndex + 1)%detectedObjectBuffers.length;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //runPeriodic

}   //class TrcVisionTask
