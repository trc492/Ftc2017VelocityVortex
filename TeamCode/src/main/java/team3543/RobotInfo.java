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

public class RobotInfo
{
    //
    // DriveBase subsystem.
    //
    public static final double ENCODER_X_KP                     = 0.1;
    public static final double ENCODER_X_KI                     = 0.0;
    public static final double ENCODER_X_KD                     = 0.0;
    public static final double ENCODER_X_KF                     = 0.0;
    public static final double ENCODER_X_TOLERANCE              = 2.0;
    public static final double ENCODER_X_SETTLING               = 0.2;
    public static final double ENCODER_X_INCHES_PER_COUNT       = 0.0093;

    public static final double ENCODER_Y_KP                     = 0.03;
    public static final double ENCODER_Y_KI                     = 0.0;
    public static final double ENCODER_Y_KD                     = 0.0;
    public static final double ENCODER_Y_KF                     = 0.0;
    public static final double ENCODER_Y_TOLERANCE              = 2.0;
    public static final double ENCODER_Y_SETTLING               = 0.2;
    public static final double ENOCDER_Y_INCHES_PER_COUNT       = (103.0/9212.25);

    public static final double GYRO_KP                          = 0.015;
    public static final double GYRO_KI                          = 0.0;
    public static final double GYRO_KD                          = 0.1;
    public static final double GYRO_KF                          = 0.0;
    public static final double GYRO_TOLERANCE                   = 2.0;
    public static final double GYRO_SETTLING                    = 0.2;

    //
    // Line detection.
    //
    public static final double LINE_DARK_LEVEL                  = 2.0;
    public static final double LINE_WHITE_LEVEL                 = 21.0;

    //
    // Shooter subsystem.
    //
    public static final double SHOOTER_KP                       = 0.02;
    public static final double SHOOTER_KI                       = 0.0;
    public static final double SHOOTER_KD                       = 0.0;
    public static final double SHOOTER_KF                       = 0.0;
    public static final double SHOOTER_TOLERANCE                = 2.0;
    public static final double SHOOTER_SETTLING                 = 0.2;
    public static final double SHOOTER_POWER                    = 1.0;
    public static final double SHOOTER_BALLGATE_OPEN_TIME       = 0.7;
    public static final double SHOOTER_PAUSE_TIME               = 0.1;
    public static final double SHOOTER_DEGREES_PER_COUNT        = (360.0/1680.0);
    public static final double SHOOTER_PULLBACK_TARGET          = 80.0; //in degrees

    public static final double SHOOTER_SIMPLE_PULLBACK_TARGET   = 200.0; //initial pull back distance
    public static final double SHOOTER_SIMPLE_ONESHOT_TARGET    = 1667.0 - SHOOTER_SIMPLE_PULLBACK_TARGET; //encoder distance for one shot minus pull back
    public static final double SHOOTER_SIMPLE_OPEN_GATE_DELAY   = 1.0; //delay for opening ball gate (in second)
    public static final double SHOOTER_SIPMLE_CLOSE_GATE_DELAY  = 1.0; //delay for closing ball gate
    public static final double SHOOTER_SIPMLE_AFTER_FIRE_DELAY  = 0.5;

    //
    // Ball gate subsystem.
    //
    public static final double BALLGATE_CLOSE_POSITION          = 0.70;
    public static final double BALLGATE_OPEN_POSITION           = 0.56;

    //
    // Button pusher subsystem.
    //
    public static final double BUTTON_PUSHER_RETRACT_POSITION   = 0.2;
    public static final double BUTTON_PUSHER_EXTEND_POSITION    = 0.8;

    //
    // Ball pickup subsystem.
    //
    public static final double BALL_PICKUP_MOTOR_POWER          = 1.0;

    //
    // Conveyor subsystem.
    //
    public static final double CONVEYOR_MOTOR_POWER             = 1.0;

}   //class RobotInfo
