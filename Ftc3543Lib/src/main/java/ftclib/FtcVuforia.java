/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

package ftclib;

import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This class makes using Vuforia a little easier by minimizing the number of calls to it. It only exposes the
 * minimum things you need to set for the FTC competition. If you want to do more complex stuff, you may want
 * to not use this and call Vuforia directly so you can customize other stuff.
 */
public class FtcVuforia
{
    private VuforiaLocalizer.Parameters params;
    private VuforiaTrackables targetList;

    /**
     * Constructor: Create an instance of this object. It initializes Vuforia with the specified target images and
     * other parameters.
     *
     * @param licenseKey specifies the Vuforia license key.
     * @param cameraViewId specifies the camera view ID on the activity.
     * @param cameraDir specifies which camera to use (front or back).
     * @param trackablesFile specifies the XML file that contains the target info.
     * @param numTargets specifies the number of simultaneous trackable targets.
     */
    public FtcVuforia(
            String licenseKey, int cameraViewId, VuforiaLocalizer.CameraDirection cameraDir,
            String trackablesFile, int numTargets)
    {
        params = new VuforiaLocalizer.Parameters(cameraViewId);
        params.vuforiaLicenseKey = licenseKey;
        params.cameraDirection = cameraDir;
        VuforiaLocalizer localizer = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, numTargets);
        targetList = localizer.loadTrackablesFromAsset(trackablesFile);
    }   //FtcVuforia

    /**
     * This method enables/disables target tracking.
     *
     * @param enabled specifies true to enable target tracking, false otherwise.
     */
    public void setTrackingEnabled(boolean enabled)
    {
        if (enabled)
        {
            targetList.activate();
        }
        else
        {
            targetList.deactivate();
        }
    }   //setTrackingEnabled

    /**
     * This method sets the properties of the specified target.
     *
     * @param index specifies the target index in the XML file.
     * @param name specifies the target name.
     * @param locationOnField specifies the target location on the field, can be null if no robot tracking.
     * @param phoneLocationOnRobot specifies the phone location on the robot, can be null if no robot tracking.
     */
    public void setTargetInfo(int index, String name, OpenGLMatrix locationOnField, OpenGLMatrix phoneLocationOnRobot)
    {
        VuforiaTrackable target = targetList.get(index);
        target.setName(name);

        if (locationOnField != null)
        {
            target.setLocation(locationOnField);
        }

        if (phoneLocationOnRobot != null)
        {
            ((VuforiaTrackableDefaultListener) target.getListener()).setPhoneInformation(
                    phoneLocationOnRobot, params.cameraDirection);
        }
    }   //setTargetInfo

    /**
     * This method sets the properties of the specified target.
     *
     * @param index specifies the target index in the XML file.
     * @param name specifies the target name.
     */
    public void setTargetInfo(int index, String name)
    {
        setTargetInfo(index, name, null, null);
    }   //setTargetInfo

    /**
     * This method creates a location matrix that can be used to relocate an object to its final location
     * by rotating and translating the object from the origin of the field. It is doing the operation in
     * the order of the parameters. In other words, it will first rotate the object on the X-axis, then
     * rotate on the Y-axis, then rotate on the Z-axis, then translate on the X-axis, then translate on
     * the Y-axis and finally translate on the Z-axis.
     *
     * @param rotateX specifies rotation on the X-axis.
     * @param rotateY specifies rotation on the Y-axis.
     * @param rotateZ specifies rotation on the Z-axis.
     * @param translateX specifies translation on the X-axis.
     * @param translateY specifies translation on the Y-axis.
     * @param translateZ specifies translation on the Z-axis.
     * @return returns the location matrix.
     */
    public OpenGLMatrix locationMatrix(
            float rotateX, float rotateY, float rotateZ, float translateX, float translateY, float translateZ)
    {
        return OpenGLMatrix.translation(translateX, translateY, translateZ)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, rotateX, rotateY, rotateZ));
    }   //locationMatrix

    /**
     * This method returns the list of trackable targets.
     *
     * @return list of trackable targets.
     */
    public VuforiaTrackables getTargetList()
    {
        return targetList;
    }   //getTargetList

    /**
     * This method returns the target object with the specified index in the target list.
     *
     * @param index specifies the target index in the list.
     * @return target.
     */
    public VuforiaTrackable getTarget(int index)
    {
        return targetList.get(index);
    }   //getTarget

    /**
     * This method returns the target object with the specified target name.
     *
     * @param name specifies the name of the target.
     * @return target.
     */
    public VuforiaTrackable getTarget(String name)
    {
        VuforiaTrackable target = null;

        for (int i = 0; i < targetList.size(); i++)
        {
            target = targetList.get(i);
            if (name.equals(target.getName()))
            {
                break;
            }
            else
            {
                target = null;
            }
        }

        return target;
    }   //getTarget

    /**
     * This method determines if the target is visible.
     *
     * @param target specifies the target object.
     * @return true if the target is in view, false otherwise.
     */
    public boolean isTargetVisible(VuforiaTrackable target)
    {
        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener)target.getListener();
        return listener.isVisible();
    }   //isTargetVisible

    /**
     * This method returns the position matrix of the specified target.
     *
     * @param target specifies the target to get the position matrix.
     * @return position matrix of the specified target.
     */
    public OpenGLMatrix getTargetPose(VuforiaTrackable target)
    {
        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener)target.getListener();
        return listener.getPose();
    }   //getTargetPose

    /**
     * This method determines the robot location by the given target.
     *
     * @param target specifies the target to be used to determine robot location.
     * @return robot location matrix.
     */
    public OpenGLMatrix getRobotLocation(VuforiaTrackable target)
    {
        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener)target.getListener();
        return listener.getRobotLocation();
    }   //getRobotLocation

}   //class FtcVuforia
