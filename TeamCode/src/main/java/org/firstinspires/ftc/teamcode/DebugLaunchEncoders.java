/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="DebugLaunchEncoders", group="Decode Challenge")

public class DebugLaunchEncoders extends LinearOpMode {

    // Hardware
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx launcherRight = null;
    private DcMotorEx launcherLeft = null;

    // RPM Presets
    private static final double RPM_PRESET_A = 2100.0;
    private static final double RPM_PRESET_B = 2400.0;
    private static final double RPM_PRESET_X = 2700.0;
    private static final double RPM_PRESET_Y = 3000.0;
    private double targetRPM = RPM_PRESET_B;  // Default target

    @Override
    public void runOpMode() {

        // Initialize launcher motors with extended interface for velocity control
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcher_right");
        launcherLeft = hardwareMap.get(DcMotorEx.class, "launcher_left");

        launcherRight.setDirection(DcMotor.Direction.FORWARD);
        launcherLeft.setDirection(DcMotor.Direction.REVERSE);

        // Configure launcher motors for velocity control using encoders
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Target RPM", targetRPM);
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // RPM Presets
            if (gamepad2.y) {
                targetRPM = RPM_PRESET_Y;
            } else if (gamepad2.x) {
                targetRPM = RPM_PRESET_X;
            } else if (gamepad2.b) {
                targetRPM = RPM_PRESET_B;
            } else if (gamepad2.a) {
                targetRPM = RPM_PRESET_A;
            }

            // save the trigger as a bool, so we know f it is on / off more easily.
            // if it is != 0 (it is a float)
            boolean launchTrigger = gamepad2.right_trigger != 0.0f;
            double targetRadPerSec = 0;

            // setVelocity is radians / second
            if (launchTrigger) {

                // OLD CODE
                // targetVelocity = (targetRPM / 60.0) * TICKS_PER_REV;

                // Formula to get radians / sec id => Target RPM * (2 PI / 60 )
                targetRadPerSec = targetRPM * 2.0 * Math.PI / 60.0;

                launcherRight.setVelocity(targetRadPerSec);
                launcherLeft.setVelocity(targetRadPerSec);
            } else {
                launcherRight.setVelocity(0);
                launcherLeft.setVelocity(0);
            }

            // Get actual velocity for telemetry
            // getVelocity is radians / second
            double actualRadPerSecRight = launcherRight.getVelocity();
            double actuaRadPerSecLeft = launcherLeft.getVelocity();

            // OLD CODE
            // double actualRPMRight = (actualVelocityRight / TICKS_PER_REV) * 60.0;
            // double actualRPMLeft = (actualVelocityLeft / TICKS_PER_REV) * 60.0;

            double actualRPMRight = actualRadPerSecRight * 60.0 / ( 2 * Math.PI );
            double actualRPMLeft = actuaRadPerSecLeft * 60.0 / ( 2 * Math.PI );

            //Telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addLine();
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Actual RPM Right", actualRPMRight);
            telemetry.addData("Actual RPM Left ", actualRPMLeft);
            telemetry.addLine();
            telemetry.addData("Target Rad/Sec", targetRadPerSec);
            telemetry.addData("Actual Rad/Sec Right", actualRadPerSecRight);
            telemetry.addData("Actual Rad/Sec Left ", actuaRadPerSecLeft);
            telemetry.addLine();
            telemetry.addData("Trigger", "%b", launchTrigger);
            telemetry.addLine();

            telemetry.update();
        }

    }

}
