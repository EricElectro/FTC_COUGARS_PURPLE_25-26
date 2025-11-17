package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
import com.qualcomm.robotcore.hardware.IMU;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name="Auto", group = "Example")

public class Auto extends LinearOpMode
{
    final double DESIRED_DISTANCE = 60.0;
    final double SPEED_GAIN  = 0.05;
    final double TURN_GAIN   = 0.01;
    final double MAX_AUTO_SPEED = 0.75;
    final double MAX_AUTO_TURN  = 0.5;
    
    private DcMotor LeftDrive;
    private DcMotor RightDrive;
    private DcMotor FlyWheel_Big;
    private DcMotor FlyWheel_Small;
    private Servo Top_Servo;
    private IMU imu;
    
    private static final boolean USE_WEBCAM = true;
    private int DESIRED_TAG_ID = -1;
    private int DESIRED_ORDER_TAG_ID = -1;
    private static double Dist = 9999;
    private static double Angle = 0;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    private AprilTagDetection desiredOrderTag = null;
    boolean targetFound = false;
    boolean orderFound = false;
    
    private void initiate()
    {
        LeftDrive = hardwareMap.get(DcMotor.class, "Left Motor");
        RightDrive = hardwareMap.get(DcMotor.class, "Right Motor");
        FlyWheel_Big = hardwareMap.get(DcMotor.class, "FlyWheel_Big");
        FlyWheel_Small = hardwareMap.get(DcMotor.class, "FlyWheel_Small");
        Top_Servo = hardwareMap.get(Servo.class, "Top_Servo");
        imu = hardwareMap.get(IMU.class, "imu");
        LeftDrive.setDirection(DcMotor.Direction.FORWARD);
        RightDrive.setDirection(DcMotor.Direction.REVERSE);
        FlyWheel_Big.setDirection(DcMotor.Direction.REVERSE);
        FlyWheel_Small.setDirection(DcMotor.Direction.FORWARD);
        IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        );
        imu.initialize(parameters);
    }


    enum AutoState { SEARCH, APPROACH_GOAL, SHOOTING, GO_TO_LOADING, LOADING, RETURN_TO_GOAL}
    AutoState state = AutoState.SEARCH;

    int ballsShotThisCycle = 0;
    final int MAX_BALLS_PER_CYCLE = 3;
    long Time_To_Charge = 3000;
    boolean teamIsRed = true;
    ElapsedTime runtime = new ElapsedTime();
    @Override public void runOpMode()
    {
        initAprilTag();
        initiate();
        InitiateTeamTags(teamIsRed ? "RED" : "BLUE");
        
        if (USE_WEBCAM) setManualExposure(1, 2); // Indoors (6, 150)
        
        telemetry.addData("Status", "Init complete. Team: %s", teamIsRed ? "RED" : "BLUE");
        telemetry.addData("Goal Tag", DESIRED_TAG_ID);
        telemetry.addData("Order Tag (loading)", DESIRED_ORDER_TAG_ID);
        telemetry.update();
        waitForStart();
        runtime.reset();

        LeftDrive.setPower(0.5);
        RightDrive.setPower(0.5);
        sleep(500);
        LeftDrive.setPower(0);
        RightDrive.setPower(0);
        targetFound = false;
        orderFound = false;
        
        while (opModeIsActive())
        {
            desiredTag = null;
            desiredOrderTag = null;
            
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if (detection.id == DESIRED_TAG_ID) {
                        desiredTag = detection;
                        targetFound = true;
                    } else if (detection.id != 20 && detection.id != 24) {
                        desiredOrderTag = detection;
                        orderFound = true;
                    }
                }
            }
            if (targetFound && desiredTag != null) {
                Dist = desiredTag.ftcPose.range;
                Angle = desiredTag.ftcPose.yaw;
            }
            switch (state) {
                case SEARCH:
                    /*telemetry.addData("State","SEARCH");
                    if (targetFound && orderFound) {
                        state = AutoState.APPROACH_GOAL;
                        telemetry.addData("Transition","Found goal -> APPROACH_GOAL");
                    } else {
                        RunRobot(0, 0.75 * (teamIsRed ? 1 : -1));
                    }
                    break;*/
                    YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
                    double yawDeg = ypr.getYaw(AngleUnit.DEGREES);
                    double pitchDeg = ypr.getPitch(AngleUnit.DEGREES);
                    double rollDeg = ypr.getRoll(AngleUnit.DEGREES);
                    telemetry.addData("Status", yawDeg + " Yaw");
                    telemetry.addData("Status", pitchDeg + " Pitch");
                    telemetry.addData("Status", rollDeg + " Roll");
                    telemetry.update();
                    
                case APPROACH_GOAL:
                    telemetry.addData("State","APPROACH_GOAL");
                    if (!targetFound || desiredTag == null) {
                        state = AutoState.SEARCH;
                        break;
                    }
                    double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                    double headingError = desiredTag.ftcPose.bearing;
                    double yawError = desiredTag.ftcPose.yaw;
                    double drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    double turn  = Range.clip(yawError * TURN_GAIN / 2, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    telemetry.addData("Transition",Math.abs(rangeError) + " " + Math.abs(yawError));
                    if (Math.abs(rangeError) < 5.0 && Math.abs(yawError) < 5.0) {
                        LeftDrive.setPower(0);
                        RightDrive.setPower(0);
                        ballsShotThisCycle = 0;
                        state = AutoState.SHOOTING;
                        telemetry.addData("Transition","APPROACH_GOAL -> SHOOTING");
                    } else {
                        RunRobot(drive, turn);
                    }
                    break;
                case SHOOTING:
                    telemetry.addData("State","SHOOTING");
                    if (ballsShotThisCycle < 3) {
                        ballsShotThisCycle++;
                        telemetry.addData("Shooting","Ball %d", ballsShotThisCycle);
                        ShootOneBall();
                        sleep(500);
                    } else {
                        ballsShotThisCycle = 0;
                        state = AutoState.GO_TO_LOADING;
                        telemetry.addData("Transition","SHOOTING -> GO_TO_LOADING");
                    }
                    break;
                case GO_TO_LOADING:
                    telemetry.addData("State","GO_TO_LOADING");
                    if (orderFound && desiredOrderTag != null) {
                        double loadRangeError = desiredOrderTag.ftcPose.range - 24.0;
                        double loadHeadingError = desiredOrderTag.ftcPose.bearing;
                        double driveLoad = Range.clip(loadRangeError * SPEED_GAIN, -0.5, 0.5);
                        double turnLoad  = Range.clip(loadHeadingError * TURN_GAIN, -0.3, 0.3);
                        if (Math.abs(loadRangeError) < 10.0) {
                            LeftDrive.setPower(0);
                            RightDrive.setPower(0);
                            state = AutoState.LOADING;
                            telemetry.addData("Transition","GO_TO_LOADING -> LOADING");
                        } else {
                            RunRobot(driveLoad, turnLoad);
                        }
                    } else {
                        RunRobot(0.4, 0);
                        sleep(600);
                        RunRobot(0, 0.4 * (teamIsRed ? 1 : -1));
                        sleep(500);
                        RunRobot(0,0);
                    }
                    break;
                case LOADING:
                    break;
                case RETURN_TO_GOAL:
                    telemetry.addData("State","RETURN_TO_GOAL");
                    if (targetFound && desiredTag != null) {
                        double rError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                        double hError = desiredTag.ftcPose.bearing;
                        double d = Range.clip(rError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                        double t = Range.clip(hError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                        if (Math.abs(rError) < 6.0 && Math.abs(desiredTag.ftcPose.yaw) < 8.0) {
                            LeftDrive.setPower(0);
                            RightDrive.setPower(0);
                            ballsShotThisCycle = 0;
                            state = AutoState.SHOOTING;
                            telemetry.addData("Transition","RETURN_TO_GOAL -> SHOOTING");
                        } else {
                            RunRobot(d, t);
                        }
                    } else {
                        RunRobot(0, 0.3 * (teamIsRed ? -1 : 1));
                    }
                    break;
                default:
                    LeftDrive.setPower(0);
                    RightDrive.setPower(0);
                    break;
            }
            
            telemetry.addData("DetectedGoal", targetFound);
            telemetry.addData("DetectedLoad", orderFound);
            telemetry.addData("Dist", "%.1f", Dist);
            telemetry.addData("Yaw", "%.1f", Angle);
            telemetry.update();
        }
        telemetry.addData("Auto", "Complete");
        telemetry.update();
    }

    public void RunRobot(double x, double yaw) {
        double LeftPower =  x + yaw;
        double RightPower = x - yaw;
        double max = Math.max(Math.abs(LeftPower), Math.abs(RightPower));
        if (max > 1.0) {
            LeftPower /= max;
            RightPower /= max;
        }
        LeftDrive.setPower(LeftPower);
        RightDrive.setPower(RightPower);
    }
    
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
    
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

        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    
    private void ShootOneBall()
    {
        FlyWheel_Big.setPower(0.8);
        FlyWheel_Small.setPower(0.0);
        Top_Servo.setPosition(0.0);
        sleep((int)Time_To_Charge);
        Top_Servo.setPosition(0.5);
        FlyWheel_Small.setPower(1.0);
        sleep(500);
        Top_Servo.setPosition(0.0);
        FlyWheel_Small.setPower(0.0);
        sleep(500);
        FlyWheel_Big.setPower(0.0);
        sleep(500);
    }

    private void InitiateTeamTags(String teamColor)
    {
        if (teamColor != null && teamColor.equalsIgnoreCase("RED")) { DESIRED_TAG_ID = 24; } else { DESIRED_TAG_ID = 20; }
    }
}









