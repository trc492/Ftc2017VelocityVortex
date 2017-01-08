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

import java.util.ArrayList;
import java.util.HashSet;

/**
 * This class provides methods for the callers to register/unregister cooperative multi-tasking tasks. It manages
 * these tasks and will work with the cooperative multi-tasking scheduler to run these tasks.
 */
public class TrcTaskMgr
{
    private static final String moduleName = "TrcTaskMgr";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    /**
     * These are the task type TrcTaskMgr supports:
     */
    public enum TaskType
    {
        /**
         * START_TASK is called one time before a competition mode is about to start.
         */
        START_TASK,

        /**
         * STOP_TASK is called one time before a competition mode is about to end.
         */
        STOP_TASK,

        /**
         * PREPERIODIC_TASK is called periodically at a rate about 50Hz before runPeriodic().
         */
        PREPERIODIC_TASK,

        /**
         * POSTPERIODIC_TASK is called periodically at a rate about 50Hz after runPeriodic().
         */
        POSTPERIODIC_TASK,

        /**
         * PRECONTINUOUS_TASK is called periodically at a rate as fast as the scheduler is able to loop and is run
         * before runContinuous() typically 10 msec interval.
         */
        PRECONTINUOUS_TASK,

        /**
         * POSTCONTINUOUS_TASK is called periodically at a rate as fast as the schedule is able to loop and is run
         * after runContinuous() typically 10 msec interval.
         */
        POSTCONTINUOUS_TASK

    }   //enum TaskType

    /**
     * Any class that is registering as a cooperative multi-tasking task must implement this interface.
     */
    public interface Task
    {
        /**
         * This method contains code that will initialize the task before a competition mode is about to start.
         * Typically, if the task is a robot subsystem, you may put last minute mode specific initialization code
         * here. Most of the time, you don't put any code here because all initialization is done in initRobot().
         * But sometimes, you may want to delay a certain initialization until right before competition starts.
         * For example, you may want to reset the gyro heading right before competition starts to prevent drifting.
         *
         * @param runMode specifies the competition mode that is about to start (e.g. Autonomous, TeleOp, Test).
         */
        void startTask(TrcRobot.RunMode runMode);

        /**
         * This method contains code that will clean up the task before a competition mode is about to end. Typically,
         * if the task is a robot subsystem, you may put code to stop the robot here. Most of the time, you don't put
         * any code here because the system will cut power to all the motors after a competition mode has ended.
         *
         * @param runMode specifies the competition mode that is about to end (e.g. Autonomous, TeleOp, Test).
         */
        void stopTask(TrcRobot.RunMode runMode);

        /**
         * This method contains code that will run before runPeriodic() is called. Typically, you will put code that
         * deals with any input or sensor readings here so that the code in runPeriodic() will be able to make use of
         * the input/sensor readings produced by the code here.
         *
         * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
         */
        void prePeriodicTask(TrcRobot.RunMode runMode);

        /**
         * This method contains code that will run after runPeriodic() is called. Typically, you will put code that
         * deals with actions such as programming the motors here.
         *
         * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
         */
        void postPeriodicTask(TrcRobot.RunMode runMode);

        /**
         * This method contains code that will run before runContinuous() is called. Typically, you will put code that
         * deals with any input or sensor readings that requires more frequent processing here such as integrating the
         * gyro rotation rate to heading.
         *
         * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
         */
        void preContinuousTask(TrcRobot.RunMode runMode);

        /**
         * This method contains code that will run after runContinuous() is called. Typically, you will put code that
         * deals with actions that requires more frequent processing.
         *
         * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
         */
        void postContinuousTask(TrcRobot.RunMode runMode);

    }   //interface Task

    /**
     * This class implements TaskObject that will be created whenever a class is registered as a cooperative
     * multi-tasking task. The created task objects will be entered into an array list of task objects to be
     * scheduled by the scheduler.
     */
    private static class TaskObject
    {
        private HashSet<TaskType> taskTypes;
        private final String taskName;
        private Task task;

        /**
         * Constructor: Creates an instgance of the task object with the given name
         * and the given task type.
         *
         * @param taskName specifies the instance name of the task.
         * @param task specifies the object that implements the TrcTaskMgr.Task interface.
         */
        public TaskObject(final String taskName, Task task)
        {
            taskTypes = new HashSet<>();
            this.taskName = taskName;
            this.task = task;
        }   //TaskObject

        /**
         * This method adds the given task type to the task object.
         *
         * @param type specifies the task type.
         * @return true if successful, false if the task with that task type is already registered in the task list.
         */
        public boolean addTaskType(TaskType type)
        {
            return taskTypes.add(type);
        }   //addTaskType

        /**
         * This method removes the given task type from the task object.
         *
         * @param type specifies the task type.
         * @return true if successful, false if the task with that type is not found the task list.
         */
        public boolean removeTaskType(TaskType type)
        {
            return taskTypes.remove(type);
        }   //removeTaskType

        /**
         * This method checks if the given task is associated with this task object.
         *
         * @param task specifies the task to be checked against.
         * @return true if it is the same task, false otherwise.
         */
        public boolean isSame(Task task)
        {
            return task == this.task;
        }   //isSame

        /**
         * This method checks if the given task type is registered with this task object.
         *
         * @param type specifies the task type to be checked against.
         * @return true if this task is registered as the given type, false otherwise.
         */
        public boolean hasType(TaskType type)
        {
            return taskTypes.contains(type);
        }   //hasType

        /**
         * This method checks if this task object has no registered task type.
         *
         * @return true if this task has no task type, false otherwise.
         */
        public boolean hasNoType()
        {
            return taskTypes.isEmpty();
        }   //hasNoType

        /**
         * This method returns the instance name of the task.
         *
         * @return instance name of the class.
         */
        public String toString()
        {
            return taskName;
        }   //toString

        /**
         * This method returns the class object that was associated with this task object.
         *
         * @return class object associated with the task.
         */
        public Task getTask()
        {
            return task;
        }   //getTask

    }   //class TaskObject

    private static TrcTaskMgr instance = null;
    private ArrayList<TaskObject> taskList = new ArrayList<>();

    /**
     * Constructor: Creates an instance of the task manager. Typically, there is only one global instance of
     * task manager. Any class that needs to call task manager can call its static method getInstance().
     */
    public TrcTaskMgr()
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        instance = this;
    }   //TrcTaskMgr

    /**
     * This method returns the global instance of TrcTaskMgr.
     *
     * @return global instance of TrcTaskMgr.
     */
    public static TrcTaskMgr getInstance()
    {
        return instance;
    }   //getInstance

    /**
     * This method registers a class object as a cooperative multi-tasking task with the given task type.
     *
     * @param taskName specifies the instance name of the task.
     * @param task specifies the class object associated with the task.
     * @param type specifies the task type.
     */
    public void registerTask(final String taskName, Task task, TaskType type)
    {
        final String funcName = "registerTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "task=%s,type=%s", taskName, type.toString());
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        //
        // Check if the task object already exist. If not, create a new task object and add it to the task object list.
        //
        TaskObject taskObj = findTask(task);
        if (taskObj == null)
        {
            taskObj = new TaskObject(taskName, task);
            taskList.add(taskObj);
        }

        //
        // Register the task type with the task object.
        //
        taskObj.addTaskType(type);
    }   //registerTask

    /**
     * This method unregisters a task type from a task object associated with the given task class.
     *
     * @param task specifies the class objhect associated with the task.
     * @param type specifies the task type.
     */
    public void unregisterTask(Task task, TaskType type)
    {
        final String funcName = "unregisterTask";
        TaskObject taskObj = findTask(task);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "task=%s,type=%s", taskObj != null ? taskObj.toString() : "unknown", type.toString());
        }

        //
        // If we found the task object associated with the given task, unregister the task type from it and if the
        // task object has no more task type, remove it from the task list.
        //
        if (taskObj != null)
        {
            taskObj.removeTaskType(type);
            if (taskObj.hasNoType())
            {
                taskList.remove(taskObj);
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //unregisterTask

    /**
     * This method enumerates the task list and calls all the tasks that matches the given task type.
     *
     * @param type specifies the task type to be executed.
     * @param mode specifies the robot run mode.
     */
    public void executeTaskType(TaskType type, TrcRobot.RunMode mode)
    {
        final String funcName = "executeTaskType";

        for (int i = 0; i < taskList.size(); i++)
        {
            TaskObject taskObj = taskList.get(i);
            if (taskObj.hasType(type))
            {
                Task task = taskObj.getTask();
                switch (type)
                {
                    case START_TASK:
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Executing StartTask %s", taskObj.toString());
                        }
                        task.startTask(mode);
                        break;

                    case STOP_TASK:
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Executing StopTask %s", taskObj.toString());
                        }
                        task.stopTask(mode);
                        break;

                    case PREPERIODIC_TASK:
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Executing PrePeriodicTask %s", taskObj.toString());
                        }
                        task.prePeriodicTask(mode);
                        break;

                    case POSTPERIODIC_TASK:
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Executing PostPeriodicTask %s", taskObj.toString());
                        }
                        task.postPeriodicTask(mode);
                        break;

                    case PRECONTINUOUS_TASK:
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Executing PreContinuousTask %s", taskObj.toString());
                        }
                        task.preContinuousTask(mode);
                        break;

                    case POSTCONTINUOUS_TASK:
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Executing PostContinuousTask %s", taskObj.toString());
                        }
                        task.postContinuousTask(mode);
                        break;
                }
            }
        }
    }   //executeTaskType

    /**
     * This method finds the given task in the task list and return it.
     *
     * @param task specifies the task to look for.
     * @return true if found, false otherwise.
     */
    private TaskObject findTask(Task task)
    {
        for (int i = 0; i < taskList.size(); i++)
        {
            TaskObject taskObj = taskList.get(i);
            if (taskObj.isSame(task))
            {
                return taskObj;
            }
        }
        return null;
    }   //findTask

}   //class TaskMgr
