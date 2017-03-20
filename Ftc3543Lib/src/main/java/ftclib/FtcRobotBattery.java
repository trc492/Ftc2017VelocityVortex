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

package ftclib;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import trclib.TrcRobotBattery;

/**
 * This class extends the TrcRobotBattery which provides a task to monitor the robot battery levels and the methods to
 * access the highest and the lowest battery levels during the monitoring session.
 */
public class FtcRobotBattery extends TrcRobotBattery
{
    private VoltageSensor sensor;

    /**
     * Constructor: create an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     */
    public FtcRobotBattery(HardwareMap hardwareMap)
    {
        super();
        sensor = hardwareMap.voltageSensor.iterator().next();
    }   //FtcRobotBattery

    /**
     * Constructor: create an instance of the object.
     */
    public FtcRobotBattery()
    {
        this(FtcOpMode.getInstance().hardwareMap);
    }   //FtcRobotBattery

    /**
     * This method returns the robot battery voltage.
     *
     * @return current battery voltage in volts.
     */
    @Override
    public double getVoltage()
    {
        return sensor.getVoltage();
    }   //getVoltage

    /**
     * This method returns the robot battery current.
     *
     * @return current battery current in amps.
     */
    @Override
    public double getCurrent()
    {
        throw new UnsupportedOperationException("The system does not support current info.");
    }   //getCurrent

    /**
     * This method returns the robot battery power.
     *
     * @return current battery power in watts.
     */
    @Override
    public double getPower()
    {
        throw new UnsupportedOperationException("The system does not support power info.");
    }   //getPower

}   //class FtcRobotBattery
