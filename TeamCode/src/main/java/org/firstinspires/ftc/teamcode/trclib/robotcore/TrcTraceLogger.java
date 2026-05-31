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

package org.firstinspires.ftc.teamcode.trclib.robotcore;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

/**
 * This class implements a trace logger that writes trace messages to a log file.
 */
public class TrcTraceLogger
{
    private final String logFileName;
    private BufferedWriter writer;
    private volatile boolean enabled;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param logFileName specifies the full path of the log file.
     */
    public TrcTraceLogger(String logFileName)
    {
        this.logFileName = logFileName;
        this.enabled = false;
        try
        {
            writer = new BufferedWriter(new FileWriter(logFileName, true));
            enabled = true;
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }   //TrcTraceLogger

    /**
     * This method returns the log file name.
     *
     * @return log file name.
     */
    @Override
    public String toString()
    {
        return logFileName;
    }   //toString

    /**
     * This method enables/disables the trace logger.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public synchronized void setEnabled(boolean enabled)
    {
        this.enabled = enabled;
        if (!enabled && writer != null)
        {
            try
            {
                writer.close();
            }
            catch (IOException e)
            {
                e.printStackTrace();
            }
            writer = null;
        }
    }   //setEnabled

    /**
     * This method checks if the trace logger is enabled.
     *
     * @return true if enabled, false otherwise.
     */
    public boolean isEnabled()
    {
        return enabled;
    }   //isEnabled

    /**
     * This method logs a message to the log file.
     *
     * @param msg specifies the message to be logged.
     */
    public synchronized void logMessage(String msg)
    {
        if (enabled && writer != null)
        {
            try
            {
                writer.write(msg);
                writer.newLine();
                writer.flush();
            }
            catch (IOException e)
            {
                e.printStackTrace();
            }
        }
    }   //logMessage

}   //class TrcTraceLogger
