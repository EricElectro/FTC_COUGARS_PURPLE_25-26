package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name="Auto", group = "Example")

public class Auto extends LinearOpMode
{
    // --- Constants for auto-driving & tag handling ---
    final double DESIRED_DISTANCE = 60.0;   // Distance you want to stop from the scoring tag (cm)
    final double SPEED_GAIN  = 0.05;        // How strongly robot drives forward when far away
    final double TURN_GAIN   = 0.01;        // How strongly robot turns to face tag
    final double MAX_AUTO_SPEED = 0.75;     // Max forward speed in auto
    final double MAX_AUTO_TURN  = 0.5;      // Max turning speed in auto
    
    // --- Hardware declarations ---
    private DcMotor LeftDrive;
    private DcMotor RightDrive;
    private DcMotor FlyWheel_Big;
    private DcMotor FlyWheel_Small;
    private Servo Top_Servo;
    
    // --- AprilTag variables ---
    private static final boolean USE_WEBCAM = true;
    private int DESIRED_TAG_ID = -1;         // Scoring tag (goal)
    private int DESIRED_ORDER_TAG_ID = -1;   // Loading station tag (unused but tracked)
    private static double Dist = 9999;       // Distance to target tag
    private static double Angle = 0;         // Angle (yaw) to target tag
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    private AprilTagDetection desiredOrderTag = null;
    boolean targetFound = false; // Found goal tag?
    boolean orderFound = false;  // Found loading tag?
    
    // --- Autonomous State Machine ---
    enum AutoState { SEARCH, APPROACH_GOAL, SHOOTING, GO_TO_LOADING, LOADING, RETURN_TO_GOAL }
    AutoState state = AutoState.SEARCH;

    // Shooting cycle control
    int ballsShotThisCycle = 0;
    final int MAX_BALLS_PER_CYCLE = 3;

    long Time_To_Charge = 3000;     // Flywheel charge time for shooting (ms)
    boolean teamIsRed = true;       // Controls which AprilTag ID is target
    ElapsedTime runtime = new ElapsedTime();

    @Override public void runOpMode()
    {
        // --- Initialize camera & AprilTag pipeline ---
        initAprilTag();

        // Map hardware
        LeftDrive = hardwareMap.get(DcMotor.class, "Left Motor");
        RightDrive = hardwareMap.get(DcMotor.class, "Right Motor");
        FlyWheel_Big = hardwareMap.get(DcMotor.class, "FlyWheel_Big");
        FlyWheel_Small = hardwareMap.get(DcMotor.class, "FlyWheel_Small");
        Top_Servo = hardwareMap.get(Servo.class, "Top_Servo");

        // Motor directions
        LeftDrive.setDirection(DcMotor.Direction.FORWARD);
        RightDrive.setDirection(DcMotor.Direction.REVERSE);
        FlyWheel_Big.setDirection(DcMotor.Direction.REVERSE);
        FlyWheel_Small.setDirection(DcMotor.Direction.FORWARD);

        // Set which AprilTag is target depending on team
        InitiateTeamTags(teamIsRed ? "RED" : "BLUE");
        
        // Optional camera exposure tuning
        if (USE_WEBCAM) setManualExposure(6, 150);

        telemetry.addData("Status", "Init complete. Team: %s", teamIsRed ? "RED" : "BLUE");
        telemetry.addData("Goal Tag", DESIRED_TAG_ID);
        telemetry.addData("Order Tag (loading)", DESIRED_ORDER_TAG_ID);
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Move forward a bit at the start (maybe leaving tile)
        LeftDrive.setPower(0.5);
        RightDrive.setPower(0.5);
        sleep(1000);
        LeftDrive.setPower(0);
        RightDrive.setPower(0);

        // Reset tag flags
        targetFound = false;
        orderFound = false;
        
        // ---------------------------
        // MAIN AUTONOMOUS LOOP
        // ---------------------------
        while (opModeIsActive())
        {
            desiredTag = null;
            desiredOrderTag = null;
            
            // --- Read all detected AprilTags ---
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {

                    // Check if this is the scoring tag
                    if (detection.id == DESIRED_TAG_ID) {
                        desiredTag = detection;
                        targetFound = true;

                    // Loading station tag detection
                    } else if (detection.id != 20 && detection.id != 24) {
                        desiredOrderTag = detection;
                        orderFound = true;
                    }
                }
            }

            // Update distance & angle if goal tag found
            if (targetFound && desiredTag != null) {
                Dist = desiredTag.ftcPose.range;
                Angle = desiredTag.ftcPose.yaw;
            }

            // --------------------------
            // STATE MACHINE LOGIC
            // --------------------------
            switch (state) {

                // ---------------- SEARCH ----------------
                case SEARCH:
                    telemetry.addData("State","SEARCH");

                    // If both tags known → move to approach phase
                    if (targetFound && orderFound) {
                        state = AutoState.APPROACH_GOAL;
                        telemetry.addData("Transition","Found goal → APPROACH_GOAL");

                    // Otherwise slowly rotate to scan the field
                    } else {
                        RunRobot(0, 0.75 * (teamIsRed ? 1 : -1));
                    }
                    break;

                // ---------------- APPROACH GOAL ----------------
                case APPROACH_GOAL:
                    telemetry.addData("State","APPROACH_GOAL");

                    // If you lose sight of goal tag, go back to SEARCH
                    if (!targetFound || desiredTag == null) {
                        state = AutoState.SEARCH;
                        break;
                    }

                    // Compute positional errors relative to AprilTag
                    double rangeError = desiredTag.ftcPose.range - DESIRED_DISTANCE;
                    double headingError = desiredTag.ftcPose.bearing;
                    double yawError = desiredTag.ftcPose.yaw;

                    // Proportional drive + turn control
                    double drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    double turn  = Range.clip(yawError * TURN_GAIN / 2, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                    telemetry.addData("Transition",Math.abs(rangeError) + " " + Math.abs(yawError));

                    // When close enough → start shooting
                    if (Math.abs(rangeError) < 5.0 && Math.abs(yawError) < 5.0) {
                        LeftDrive.setPower(0);
                        RightDrive.setPower(0);
                        ballsShotThisCycle = 0;
                        state = AutoState.SHOOTING;

                        telemetry.addData("Transition","APPROACH_GOAL → SHOOTING");

                    } else {
                        RunRobot(drive, turn);
                    }
                    break;

                // ---------------- SHOOTING ----------------
                case SHOOTING:
                    telemetry.addData("State","SHOOTING");

                    // Shoot 3 balls one by one
                    if (ballsShotThisCycle < 3) {
                        ballsShotThisCycle++;
                        telemetry.addData("Shooting","Ball %d", ballsShotThisCycle);

                        ShootOneBall();
                        sleep(500);

                    } else {
                        ballsShotThisCycle = 0;
                        state = AutoState.GO_TO_LOADING;

                        telemetry.addData("Transition","SHOOTING → GO_TO_LOADING");
                    }
                    break;

                // ---------------- GO TO LOADING TAG ----------------
                case GO_TO_LOADING:
                    telemetry.addData("State","GO_TO_LOADING");

                    if (orderFound && desiredOrderTag != null) {

                        // Compute how far we are from loading station
                        double loadRangeError = desiredOrderTag.ftcPose.range - 24.0;
                        double loadHeadingError = desiredOrderTag.ftcPose.bearing;

                        double driveLoad = Range.clip(loadRangeError * SPEED_GAIN, -0.5, 0.5);
                        double turnLoad  = Range.clip(loadHeadingError * TURN_GAIN, -0.3, 0.3);

                        // When close enough → begin loading
                        if (Math.abs(loadRangeError) < 10.0) {
                            LeftDrive.setPower(0);
                            RightDrive.setPower(0);
                            state = AutoState.LOADING;

                            telemetry.addData("Transition","GO_TO_LOADING → LOADING");

                        } else {
                            RunRobot(driveLoad, turnLoad);
                        }

                    // If loading tag not found, do a small search pattern
                    } else {
                        RunRobot(0.4, 0);
                        sleep(600);
                        RunRobot(0, 0.4 * (teamIsRed ? 1 : -1));
                        sleep(500);
                        RunRobot(0,0);
                    }
                    break;

                // ---------------- LOADING BALLS (wait or intake) ----------------
                case LOADING:
                    // Nothing coded here yet — robot simply waits?
                    break;

                // ---------------- RETURN TO GOAL ----------------
                case RETURN_TO_GOAL:
                    telemetry.addData("State","RETURN_TO_GOAL");

                    if (targetFound && desiredTag != null) {

                        // Compute errors again for returning to goal
                        double rError = desiredTag.ftcPose.range - DESIRED_DISTANCE;
                        double hError = desiredTag.ftcPose.bearing;

                        double d = Range.clip(rError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                        double t = Range.clip(hError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                        // Close enough → shoot again
                        if (Math.abs(rError) < 6.0 && Math.abs(desiredTag.ftcPose.yaw) < 8.0) {
                            LeftDrive.setPower(0);
                            RightDrive.setPower(0);
                            ballsShotThisCycle = 0;
                            state = AutoState.SHOOTING;

                            telemetry.addData("Transition","RETURN_TO_GOAL → SHOOTING");

                        } else {
                            RunRobot(d, t);
                        }

                    // If we lose the tag, slowly rotate to find it
                    } else {
                        RunRobot(0, 0.3 * (teamIsRed ? -1 : 1));
                    }
                    break;

                // ---------------- DEFAULT (fail-safe) ----------------
                default:
                    LeftDrive.setPower(0);
                    RightDrive.setPower(0);
                    break;
            }
            
            // Update telemetry every loop
            telemetry.addData("DetectedGoal", targetFound);
            telemetry.addData("DetectedLoad", orderFound);
            telemetry.addData("Dist", "%.1f", Dist);
            telemetry.addData("Yaw", "%.1f", Angle);
            telemetry.update();
        }

        telemetry.addData("Auto", "Complete");
        telemetry.update();
    }

    // ----------------------------------------------------------
    // ROBOT MOVEMENT HELPER
    // ----------------------------------------------------------
    public void RunRobot(double x, double yaw) {
        // Differential drive mixing: forward + turning
        double LeftPower =  x + yaw;
        double RightPower = x - yaw;

        // Normalize if any power > 1.0
        double max = Math.max(Math.abs(LeftPower), Math.abs(RightPower));
        if (max > 1.0) {
            LeftPower /= max;
            RightPower /= max;
        }

        LeftDrive.setPower(LeftPower);
        RightDrive.setPower(RightPower);
    }
    
    // ----------------------------------------------------------
    // APRILTAG INITIALIZATION
    // ----------------------------------------------------------
    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2); // Reduces image resolution to improve FPS

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
    
    // ----------------------------------------------------------
    // MANUAL CAMERA EXPOSURE / GAIN SETTINGS
    // ----------------------------------------------------------
    private void setManualExposure(int exposureMS, int gain) {

        if (visionPortal == null) return;

        // Wait until camera is streaming before setting exposure
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();

            while (!isStopRequested() &&
                  (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }

            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        if (!isStopRequested()) {

            // Switch camera to manual exposure mode
            ExposureControl exposureControl =
                    visionPortal.getCameraControl(ExposureControl.class);

            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }

            // Apply exposure
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);

            // Apply gain
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    // ----------------------------------------------------------
    // SHOOT A SINGLE BALL
    // ----------------------------------------------------------
    private void ShootOneBall()
    {
        // Spin up large flywheel
        FlyWheel_Big.setPower(0.8);
        FlyWheel_Small.setPower(0.0);
        Top_Servo.setPosition(0.0);

        // Let flywheel charge up
        sleep((int)Time_To_Charge);

        // Open servo gate
        Top_Servo.setPosition(0.5);

        // Kick ball with small flywheel
        FlyWheel_Small.setPower(1.0);
        sleep(500);

        // Reset servo & wheels
        Top_Servo.setPosition(0.0);
        FlyWheel_Small.setPower(0.0);
        sleep(500);
        FlyWheel_Big.setPower(0.0);
        sleep(500);
    }

    // ----------------------------------------------------------
    // SELECT TARGET APRILTAG BASED ON TEAM
    // ----------------------------------------------------------
    private void InitiateTeamTags(String teamColor)
    {
        // Red uses Tag 24, Blue uses Tag 20
        if (teamColor != null && teamColor.equalsIgnoreCase("RED")) {
            DESIRED_TAG_ID = 24;
        } else {
            DESIRED_TAG_ID = 20;
        }
    }
}
