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

package team3543;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import FtcSampleCode.R;
import ftclib.FtcVuforia;

public class VuforiaVision
{
    private Robot robot;
    private FtcVuforia vuforia;

    public VuforiaVision(Robot robot)
    {
        final float ROBOT_WIDTH = 18*RobotInfo.MM_PER_INCH;                 // in mm
        final float FTC_FIELD_WIDTH = (12*12 - 2)*RobotInfo.MM_PER_INCH;    // in mm
        final float TARGET_HEIGHT = 160.0f;                                 // in mm
        final String VUFORIA_LICENSE_KEY =
                "AdCwzDH/////AAAAGeDkDS3ukU9+lIXc19LMh+cKk29caNhOl8UqmZOymRGwVwT1ZN8uaPdE3Q+zceDu9AKNsqL9qLblSFV" +
                "/x8Y3jfOZdjMFs0CQSQOEyWv3xfJsdSmevXDQDQr+4KI31HY2YSf/KB/kyxfuRMk4Pi+vWS+oLl65o7sWPiyFgzoM74ENyb" +
                "j4FgteD/2b6B+UFuwkHWKBNpp18wrpkaiFfr/FCbRFcdWP5mrjlEZM6eOj171dybw97HPeZbGihnnxOeeUv075O7P167AVq" +
                "aiPy2eRK7OCubR32KXOqQKoyF6AXp+qu2cOTApXS5dqOOseEm+HE4eMF0S2Pld3i5AWBIR+JlPXDuc9LwoH2Q8iDwUK1+4g";
        final int CAMERAVIEW_ID = R.id.cameraMonitorViewId;
        final VuforiaLocalizer.CameraDirection CAMERA_DIR = VuforiaLocalizer.CameraDirection.FRONT;
        final String TRACKABLES_FILE = "FTC_2016-17";
        //
        // Note that the order of the targets must match the order in the FTC_2016-17.xml file.
        //
        final FtcVuforia.Target[] targets =
        {
            //
            // Blue alliance near beacon.
            //
            new FtcVuforia.Target(
                    "wheels", 90.0f, 0.0f, 0.0f, 12.0f*RobotInfo.MM_PER_INCH, FTC_FIELD_WIDTH/2.0f, TARGET_HEIGHT),
            //
            // Red alliance far beacon.
            //
            new FtcVuforia.Target(
                    "tools", 90.0f, 0.0f, 90.0f, -FTC_FIELD_WIDTH/2.0f, 30.0f*RobotInfo.MM_PER_INCH, TARGET_HEIGHT),
            //
            // Blue alliance far beacon.
            //
            new FtcVuforia.Target(
                    "legos", 90.0f, 0.0f, 0.0f, -30.0f*RobotInfo.MM_PER_INCH, FTC_FIELD_WIDTH/2.0f, TARGET_HEIGHT),
            //
            // Red alliance near beacon.
            //
            new FtcVuforia.Target(
                    "gears", 90.0f, 0.0f, 90.0f, -FTC_FIELD_WIDTH/2.0f, -12.0f*RobotInfo.MM_PER_INCH, TARGET_HEIGHT)
        };

        this.robot = robot;
        vuforia = new FtcVuforia(VUFORIA_LICENSE_KEY, CAMERAVIEW_ID, CAMERA_DIR, TRACKABLES_FILE, targets.length);
        //
        // Phone location: Mounted on the left side center of the robot with the front camera facing outward.
        //
        OpenGLMatrix phoneLocationOnRobot = vuforia.locationMatrix(90.0f, 0.0f, 0.0f, 0.0f, ROBOT_WIDTH/2.0f, 0.0f);
        vuforia.setTargets(targets, phoneLocationOnRobot);
    }   //VuforiaVision

    public void setEnabled(boolean enabled)
    {
        vuforia.setTrackingEnabled(enabled);
    }   //setEnabled

    public String getTargetName(int index)
    {
        return vuforia.getTarget(index).getName();
    }   //getTargetName

    public VectorF getTargetPosition(int index)
    {
        VectorF targetPos = null;
        VuforiaTrackable target = vuforia.getTarget(index);

        if (vuforia.isTargetVisible(target))
        {
            OpenGLMatrix pose = vuforia.getTargetPose(target);
            if (pose != null)
            {
                targetPos = pose.getTranslation();
                robot.dashboard.displayPrintf(2, "%s: %6.2f,%6.2f,%6.2f",
                                              target.getName(),
                                              targetPos.get(0)/RobotInfo.MM_PER_INCH,
                                              targetPos.get(1)/RobotInfo.MM_PER_INCH,
                                              targetPos.get(2)/RobotInfo.MM_PER_INCH);
                robot.tracer.traceInfo("TargetPos", "%s: %6.2f, %6.2f, %6.2f",
                                       target.getName(),
                                       targetPos.get(0)/RobotInfo.MM_PER_INCH,
                                       targetPos.get(1)/RobotInfo.MM_PER_INCH,
                                       targetPos.get(2)/RobotInfo.MM_PER_INCH);
            }
        }

        return targetPos;
    }   //getTargetPosition

    public OpenGLMatrix getRobotLocation(int index)
    {
        VuforiaTrackable target = vuforia.getTarget(index);
        OpenGLMatrix robotLocation = vuforia.getRobotLocation(target);

        if (robotLocation != null)
        {
            robot.dashboard.displayPrintf(3, "RobotLoc: %s", robotLocation.formatAsTransform());
            robot.tracer.traceInfo("RobotLoc", "%s: %s", target.getName(), robotLocation.formatAsTransform());
        }

        return robotLocation;
    }   //getRobotLocation

}   //class VuforiaVision
