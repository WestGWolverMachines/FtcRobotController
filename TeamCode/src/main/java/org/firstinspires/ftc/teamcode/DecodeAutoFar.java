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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="DecodeAutoFar", group="Decode Challenge", preselectTeleOp="DecodeTeleOp")
public class DecodeAutoFar extends LinearOpMode {

   
    
    //Number of balls to launch
    private static final int BALL_COUNT = 2;
    
    //Target launcher RPM for far launch
    private static final double TARGET_RPM = 3000.0;
    
    //RPM drop threshold to detect a launch
    private static final double LAUNCH_DETECT_DROP = 0.15;
    
    //Time to wait for RPM to recover after launch (ms)
    private static final int RPM_RECOVERY_TIME_MS = 500;
    
    /** Maximum time to wait for a single ball launch (ms) */
    private static final int MAX_LAUNCH_WAIT_MS = 5000;
    
    /** Distance to drive forward after launching (inches) */
    private static final double DRIVE_FORWARD_INCHES = 48.0;
    
    //Drive speed during forward movement (0-1)
    private static final double DRIVE_SPEED = 0.4;
    
    //Encoder ticks per motor revolution (GoBilda 6000 RPM)
    private static final double LAUNCHER_TICKS_PER_REV = 28.0;
    
    //Encoder ticks per inch of travel
    private static final double DRIVE_TICKS_PER_INCH = 41.8;

    // Hardware
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcherRight = null;
    private DcMotorEx launcherLeft = null;
    private CRServo topLeft = null;
    private CRServo topRight = null;
    
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize hardware
        initHardware();

        // Display configuration during init
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Ball Count", BALL_COUNT);
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Drive Distance", "%.1f inches", DRIVE_FORWARD_INCHES);
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
           
            double targetVelocity = (TARGET_RPM / 60.0) * LAUNCHER_TICKS_PER_REV;
            launcherRight.setVelocity(targetVelocity);
            launcherLeft.setVelocity(targetVelocity);
            
            waitForLauncherReady(TARGET_RPM, 3000);

            for (int ball = 1; ball <= BALL_COUNT && opModeIsActive(); ball++) {
                
                launchBall();
                
                if (ball < BALL_COUNT) {
                    // Wait for RPM to recover before next ball
                    sleep(RPM_RECOVERY_TIME_MS);
                    waitForLauncherReady(TARGET_RPM, 2000);
                }
            }

            stopFeeder();
            launcherRight.setVelocity(0);
            launcherLeft.setVelocity(0);
            
            driveForward(DRIVE_FORWARD_INCHES, DRIVE_SPEED);

            telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
            telemetry.update();
        }
    }

    /**
     * Initialize all hardware.
     */
    private void initHardware() {
        // Drive motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Reset drive encoders
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Launcher motors
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcher_right");
        launcherLeft = hardwareMap.get(DcMotorEx.class, "launcher_left");
        launcherRight.setDirection(DcMotor.Direction.FORWARD);
        launcherLeft.setDirection(DcMotor.Direction.REVERSE);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Feeder servos
        topLeft = hardwareMap.get(CRServo.class, "top_left");
        topRight = hardwareMap.get(CRServo.class, "top_right");
    }

    /**
     * Wait for launcher to reach target RPM.
     */
    private void waitForLauncherReady(double targetRPM, int timeoutMs) {
        ElapsedTime timer = new ElapsedTime();
        double targetVelocity = (targetRPM / 60.0) * LAUNCHER_TICKS_PER_REV;
        double threshold = targetVelocity * 0.95;  // 95% of target
        
        while (opModeIsActive() && timer.milliseconds() < timeoutMs) {
            double currentVelocity = (launcherRight.getVelocity() + launcherLeft.getVelocity()) / 2.0;
            double currentRPM = (currentVelocity / LAUNCHER_TICKS_PER_REV) * 60.0;
            
            telemetry.addData("Launcher", "%.0f / %.0f RPM", currentRPM, targetRPM);
            telemetry.update();
            
            if (currentVelocity >= threshold) {
                return;  // Ready!
            }
            sleep(50);
        }
    }

    /**
     * Launch a single ball using RPM drop detection.
     * Runs feeder until RPM drops (indicating ball passed through) then recovers.
     */
    private void launchBall() {
        ElapsedTime launchTimer = new ElapsedTime();
        double targetVelocity = (TARGET_RPM / 60.0) * LAUNCHER_TICKS_PER_REV;
        double dropThreshold = targetVelocity * (1.0 - LAUNCH_DETECT_DROP);
        boolean ballLaunched = false;
        
    
        topLeft.setPower(-1.0);
        topRight.setPower(1.0);
        
        
        while (opModeIsActive() && launchTimer.milliseconds() < MAX_LAUNCH_WAIT_MS && !ballLaunched) {
            double currentVelocity = (launcherRight.getVelocity() + launcherLeft.getVelocity()) / 2.0;
            double currentRPM = (currentVelocity / LAUNCHER_TICKS_PER_REV) * 60.0;
            
            telemetry.addData("Launching", "RPM: %.0f (drop at %.0f)", currentRPM, 
                    (dropThreshold / LAUNCHER_TICKS_PER_REV) * 60.0);
            telemetry.update();
            
            if (currentVelocity < dropThreshold) {
                // RPM dropped
                ballLaunched = true;
                telemetry.addData("Launch", "Detected!");
                telemetry.update();
            }
            sleep(20);
        }
        
        stopFeeder();
    }

    /**
     * Stop the feeder servos.
     */
    private void stopFeeder() {
        topLeft.setPower(0);
        topRight.setPower(0);
    }

    /**
     * Drive forward a specified distance using rear wheel encoders.
     */
    private void driveForward(double inches, double speed) {
        int targetTicks = (int)(inches * DRIVE_TICKS_PER_INCH);
        
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        leftBackDrive.setTargetPosition(targetTicks);
        rightBackDrive.setTargetPosition(targetTicks);
        
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);
        
        while (opModeIsActive() && (leftBackDrive.isBusy() || rightBackDrive.isBusy())) {
            telemetry.addData("Driving", "L:%d R:%d / %d", 
                    leftBackDrive.getCurrentPosition(), 
                    rightBackDrive.getCurrentPosition(),
                    targetTicks);
            telemetry.update();
            sleep(50);
        }
        
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
