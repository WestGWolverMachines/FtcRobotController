package org.firstinspires.ftc.teamcode;



import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "waypoints", group = "Biobuzz")

@Configurable

public class Waypoints extends LinearOpMode {

    //pid values
    public static double Kp_pos = 0.08;
    public static double Kp_heading = 0.08;

    // movement tolerance
    public static double ToleranceInches = 1.0;
    public static double toleranceDegrees = 5.0;

    //Odometry hardware position offset
    public static double ODO_X_OFFSET_MM = 0.0;
    public static double ODO_Y_OFFSET_MM = 0.0;

    //waypoints to move to
    public static double[][] waypoints = {
            {24,  0,   0},   // Move Forward 24 inches
            {0, 0,  0},   // Strafe Left 24 inches, turn 90 degrees
            {24,   0,   0}    // Return to origin (0,0) and face forward
    };
    public int currentWaypoint = 0;

    private DcMotor lf, lb, rf, rb;
    private GoBildaPinpointDriver odo;




    @Override
    public void runOpMode() {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());


        lf = hardwareMap.get(DcMotor.class, "left_front_drive");
        lb = hardwareMap.get(DcMotor.class, "left_back_drive");
        rf = hardwareMap.get(DcMotor.class, "right_front_drive");
        rb = hardwareMap.get(DcMotor.class, "right_back_drive");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);

        configurePinpoint();


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            odo.update();
            Pose2D currentpose = odo.getPosition();


            if (currentWaypoint < waypoints.length) {
                double[] wp = waypoints[currentWaypoint];
                Pose2D targetpose = new Pose2D(
                        DistanceUnit.INCH,
                        wp[0],
                        wp[1],
                        AngleUnit.DEGREES,
                        wp[2]
                );
                boolean reached = drive(currentpose, targetpose);

                if(reached){
                    setMotorPowers(0,0,0,0);
                }
            } else {
                setMotorPowers(0,0,0,0);
                telemetry.addData("Status", "Path Complete!");
            }

            telemetry.addData("Waypoint", currentWaypoint + " / " + waypoints.length);
            telemetry.addData("X", currentpose.getX(DistanceUnit.INCH));
            telemetry.addData("Y", currentpose.getY(DistanceUnit.INCH));
            telemetry.addData("Heading", Math.toDegrees(currentpose.getHeading(AngleUnit.DEGREES)));
            telemetry.update();
        }

    }

    public void configurePinpoint(){

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        /*



         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        odo.resetPosAndIMU();
    }


    private boolean drive(Pose2D currentPose, Pose2D targetPose){
        double errorX = targetPose.getX(DistanceUnit.INCH) - currentPose.getX(DistanceUnit.INCH);
        double errorY = targetPose.getY(DistanceUnit.INCH) - currentPose.getY(DistanceUnit.INCH);

        double errorHeading = targetPose.getHeading(AngleUnit.DEGREES) - currentPose.getHeading(AngleUnit.DEGREES);
        while (errorHeading > 180) errorHeading -= 360;
        while (errorHeading <= -180) errorHeading += 360;

        double DistanceToTarget = Math.hypot(errorX, errorY);
        if(DistanceToTarget < ToleranceInches && errorHeading < toleranceDegrees){
            return true;
        }

        double heading = currentPose.getHeading(AngleUnit.RADIANS);
        double rotX = errorX * Math.cos(-heading) - errorY * Math.sin(-heading);
        double rotY = errorX * Math.sin(-heading) + errorY * Math.cos(-heading);


        double axial = rotX * Kp_pos;
        double lateral = rotY * Kp_pos;
        double yaw = errorHeading * Kp_heading;

        // Calculate wheel powers
        double lfPower = (axial - lateral + yaw);
        double rfPower = (axial + lateral - yaw);
        double lbPower = (axial + lateral + yaw);
        double rbPower = (axial - lateral - yaw);

        // Normalize wheel powers
        double max = Math.max(Math.abs(lfPower), Math.abs(rfPower));
        max = Math.max(max, Math.abs(lbPower));
        max = Math.max(max, Math.abs(rbPower));
        if (max > 1.0) {
            lfPower /= max;
            rfPower /= max;
            lbPower /= max;
            rbPower /= max;
        }

        // Apply drive power
        setMotorPowers(lfPower, rfPower, lbPower, rbPower);

        return false;
    }

    private void setMotorPowers(double fl, double fr, double bl, double br) {
        lf.setPower(fl/2);
        rf.setPower(fr/2);
        lb.setPower(bl/2);
        rb.setPower(br/2);
    }

    }
