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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@TeleOp(name="DecodeTeleOp", group="Decode Challenge")

public class DecodeTeleOp extends LinearOpMode {

    // Hardware
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcherRight = null;
    private DcMotorEx launcherLeft = null;
    private CRServo topLeft = null;
    private CRServo topRight = null;

    // AprilTag
    private static final int DESIRED_TAG_ID = -1;  // -1 for ANY tag
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    // Launcher Constants
    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_RPM = 3500.0;
    
    // RPM Presets
    private static final double RPM_PRESET_A = 2100.0;
    private static final double RPM_PRESET_B = 2400.0;
    private static final double RPM_PRESET_X = 2700.0;
    private static final double RPM_PRESET_Y = 3000.0;
    private static final double RPM_ADJUST_STEP = 50.0;  // D-pad adjustment

    // State
    private double targetRPM = 2400.0;  // Default target
    private double movementSpeedDivisor = 2.0;  // Drive speed divisor
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    @Override
    public void runOpMode() {
        // Initialize drive motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Initialize launcher motors with extended interface for velocity control
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcher_right");
        launcherLeft = hardwareMap.get(DcMotorEx.class, "launcher_left");

        // Initialize servos
        topLeft = hardwareMap.get(CRServo.class, "top_left");
        topRight = hardwareMap.get(CRServo.class, "top_right");

        // Set motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        launcherRight.setDirection(DcMotor.Direction.FORWARD);
        launcherLeft.setDirection(DcMotor.Direction.REVERSE);

        // Configure launcher motors for velocity control using encoders
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize AprilTag
        initAprilTag();
        setManualExposure(6, 150);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Target RPM", targetRPM);
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            boolean targetFound = false;
            desiredTag = null;
            double tagDistance = 0;

            //AprilTag Detection
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        targetFound = true;
                        desiredTag = detection;
                        tagDistance = desiredTag.ftcPose.y;  // Distance in inches
                        break;
                    }
                }
            }

            //Drive Control
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = -gamepad1.right_stick_x;

            // Auto-aim at AprilTag
            if (gamepad1.right_bumper && targetFound) {
                yaw = Range.clip(-desiredTag.ftcPose.bearing * 0.01, -0.03, 0.03);
            }

            // Drive speed presets
            if (gamepad1.y) {
                movementSpeedDivisor = 3.0;
            } else if (gamepad1.x) {
                movementSpeedDivisor = 2.5;
            } else if (gamepad1.b) {
                movementSpeedDivisor = 2.0;
            } else if (gamepad1.a) {
                movementSpeedDivisor = 1.5;
            }

            // Calculate wheel powers
            double leftFrontPower = (axial + lateral + yaw) / movementSpeedDivisor;
            double rightFrontPower = (axial - lateral - yaw) / movementSpeedDivisor;
            double leftBackPower = (axial - lateral + yaw) / movementSpeedDivisor;
            double rightBackPower = (axial + lateral - yaw) / movementSpeedDivisor;

            // Normalize wheel powers
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Apply drive power
            leftFrontDrive.setPower(rightFrontPower);
            rightFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(rightBackPower);
            rightBackDrive.setPower(leftBackPower);

            //Launcher Control
            
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

            // RPM adjustment
            if (gamepad2.dpad_up && !dpadUpPressed) {
                targetRPM = Math.min(targetRPM + RPM_ADJUST_STEP, MAX_RPM);
                dpadUpPressed = true;
            } else if (!gamepad2.dpad_up) {
                dpadUpPressed = false;
            }

            if (gamepad2.dpad_down && !dpadDownPressed) {
                targetRPM = Math.max(targetRPM - RPM_ADJUST_STEP, 0);
                dpadDownPressed = true;
            } else if (!gamepad2.dpad_down) {
                dpadDownPressed = false;
            }

            // Auto-calculate RPM from AprilTag distance
            double launcherRPM = targetRPM;
            if (gamepad2.right_bumper && targetFound) {
                double y = tagDistance;
                launcherRPM = (-0.0285 * y * y) + (14.4 * y) + 1304;
                launcherRPM = Range.clip(launcherRPM, 1000, MAX_RPM);
            }

            double triggerValue = gamepad2.right_trigger;
            double targetVelocity = 0;
            
            if (triggerValue > 0.1) {

                targetVelocity = (launcherRPM / 60.0) * TICKS_PER_REV;
                launcherRight.setVelocity(targetVelocity);
                launcherLeft.setVelocity(targetVelocity);
            } else {
                launcherRight.setVelocity(0);
                launcherLeft.setVelocity(0);
            }

            // Get actual velocity for telemetry
            double actualVelocityRight = launcherRight.getVelocity();
            double actualVelocityLeft = launcherLeft.getVelocity();
            double actualRPMRight = (actualVelocityRight / TICKS_PER_REV) * 60.0;
            double actualRPMLeft = (actualVelocityLeft / TICKS_PER_REV) * 60.0;

            //Feeder Servo Control
            topLeft.setPower(-gamepad2.left_stick_y);
            topRight.setPower(gamepad2.left_stick_y);

            //Telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Drive Speed", "1/%.1f", movementSpeedDivisor);
            telemetry.addLine();
            telemetry.addData("Target RPM", "%.0f", launcherRPM);
            telemetry.addData("Targeting RPM", "%4f", targetVelocity);
            telemetry.addData("Actual RPM", "L:%.0f R:%.0f", actualRPMLeft, actualRPMRight);
            telemetry.addData("Trigger", "%.2f", triggerValue);
            telemetry.addLine();
            if (targetFound) {
                telemetry.addData("AprilTag", "ID %d @ %.1f in", desiredTag.id, tagDistance);
                telemetry.addData("Bearing", "%.1f°", desiredTag.ftcPose.bearing);
            } else {
                telemetry.addData("AprilTag", "Not detected");
            }
            telemetry.addData("Feeder", "%.2f", gamepad2.left_stick_y);
            telemetry.update();
        }

        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    //Initialize the AprilTag processor.
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(5);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Manually set the camera gain and exposure.
     */
    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) {
            return;
        }

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
