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
    // Tone device.
    //
    public static final double TONE_ATTACK                      = 0.0;      // in seconds
    public static final double TONE_DECAY                       = 0.0;      // in seconds
    public static final double TONE_SUSTAIN                     = 1.0;      // in proportion
    public static final double TONE_RELEASE                     = 0.02;     // in seconds
    //
    // DriveBase subsystem.
    //
    public static final double ENCODER_X_KP                     = 0.15;
    public static final double ENCODER_X_KI                     = 0.0;
    public static final double ENCODER_X_KD                     = 0.0;
    public static final double ENCODER_X_KF                     = 0.0;
    public static final double ENCODER_X_TOLERANCE              = 2.0;
    public static final double ENCODER_X_SETTLING               = 0.2;
    public static final double ENCODER_X_INCHES_PER_COUNT       = 0.0126615810903615;

    public static final double ENCODER_Y_KP                     = 0.01;
    public static final double ENCODER_Y_KI                     = 0.0;
    public static final double ENCODER_Y_KD                     = 0.0;
    public static final double ENCODER_Y_KF                     = 0.0;
    public static final double ENCODER_Y_TOLERANCE              = 2.0;
    public static final double ENCODER_Y_SETTLING               = 0.2;
    public static final double ENOCDER_Y_INCHES_PER_COUNT       = 0.01667;

    public static final double GYRO_KP                          = 0.0048;
    public static final double GYRO_KI                          = 0.0;
    public static final double GYRO_KD                          = 0.0;
    public static final double GYRO_KF                          = 0.0;
    public static final double GYRO_TOLERANCE                   = 2.0;
    public static final double GYRO_SETTLING                    = 0.2;

    public static final double RANGE_KP                         = 0.085;
    public static final double RANGE_KI                         = 0.0;
    public static final double RANGE_KD                         = 0.0;
    public static final double RANGE_KF                         = 0.0;
    public static final double RANGE_TOLERANCE                  = 0.5;
    public static final double RANGE_SETTLING                   = 0.2;

    public static final double PIDDRIVE_STALL_TIMEOUT           = 0.5;

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

    //
    // Ball gate subsystem.
    //
    public static final double BALLGATE_CLOSE_POSITION          = 0.70;
    public static final double BALLGATE_OPEN_POSITION           = 0.56;

    //
    // Button pusher subsystem.
    //
    public static final double BUTTON_PUSHER_RETRACT_POSITION   = (45.0/255.0);//(55.0/255.0);
    public static final double BUTTON_PUSHER_EXTEND_POSITION    = (200.0/255.0);

    //
    // Ball pickup subsystem.
    //
    public static final double BALL_PICKUP_MOTOR_POWER          = 1.0;

    //
    // Conveyor subsystem.
    //
    public static final double CONVEYOR_MOTOR_POWER             = 1.0;

}   //class RobotInfo
