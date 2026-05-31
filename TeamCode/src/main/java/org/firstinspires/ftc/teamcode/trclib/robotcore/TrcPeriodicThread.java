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

package org.firstinspires.ftc.teamcode.trclib.robotcore;

import java.util.function.Consumer;

/**
 * This class implements a periodic thread that runs a task at a specified interval.
 *
 * @param <T> specifies the type of the task context data.
 */
public class TrcPeriodicThread<T>
{
    private final String instanceName;
    private final Consumer<T> task;
    private volatile T data;
    private volatile long processingIntervalMs;
    private volatile boolean taskEnabled;
    private volatile boolean terminated;
    private final Thread thread;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name of the thread.
     * @param task specifies the task to run periodically.
     * @param data specifies the context data passed to the task.
     * @param priority specifies the thread priority.
     */
    public TrcPeriodicThread(String instanceName, Consumer<T> task, T data, int priority)
    {
        this.instanceName = instanceName;
        this.task = task;
        this.data = data;
        this.processingIntervalMs = 0;
        this.taskEnabled = false;
        this.terminated = false;

        thread = new Thread(this::periodicRun, instanceName);
        thread.setPriority(priority);
        thread.setDaemon(true);
        thread.start();
    }   //TrcPeriodicThread

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This is the thread body that runs the task periodically.
     */
    private void periodicRun()
    {
        while (!terminated)
        {
            if (taskEnabled)
            {
                task.accept(data);
            }

            long interval = processingIntervalMs;
            if (interval > 0)
            {
                try
                {
                    Thread.sleep(interval);
                }
                catch (InterruptedException e)
                {
                    if (!terminated)
                    {
                        Thread.currentThread().interrupt();
                    }
                }
            }
            else
            {
                Thread.yield();
            }
        }
    }   //periodicRun

    /**
     * This method sets the processing interval of the periodic thread.
     *
     * @param intervalMs specifies the processing interval in milliseconds. If 0, run as fast as possible.
     */
    public void setProcessingInterval(long intervalMs)
    {
        this.processingIntervalMs = intervalMs;
    }   //setProcessingInterval

    /**
     * This method returns the processing interval of the periodic thread.
     *
     * @return processing interval in milliseconds.
     */
    public long getProcessingInterval()
    {
        return processingIntervalMs;
    }   //getProcessingInterval

    /**
     * This method enables/disables the periodic task.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setTaskEnabled(boolean enabled)
    {
        this.taskEnabled = enabled;
    }   //setTaskEnabled

    /**
     * This method checks if the periodic task is enabled.
     *
     * @return true if enabled, false otherwise.
     */
    public boolean isTaskEnabled()
    {
        return taskEnabled;
    }   //isTaskEnabled

    /**
     * This method sets the context data for the task.
     *
     * @param data specifies the context data.
     */
    public void setData(T data)
    {
        this.data = data;
    }   //setData

    /**
     * This method terminates the periodic thread.
     */
    public void terminateTask()
    {
        terminated = true;
        thread.interrupt();
    }   //terminateTask

}   //class TrcPeriodicThread
