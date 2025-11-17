package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DriveAndShoot", group = "Examples")
public class DriveAndShoot extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor FlyWheel_Big;
    private DcMotor FlyWheel_Small;
    private Servo Top_Servo;

    private VisionPortal visionportal;
    private WebcamName webcam;
    
    boolean Harry = false;
    boolean Inverted = false;

    double FlyWheelControl = 100;
    double WheelControl = 100;

    double timer1 = 0;
    double timer2 = 0;
    
    @Override
    public void runOpMode() {
        
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        leftMotor = hardwareMap.get(DcMotor.class, "Left Motor");
        rightMotor = hardwareMap.get(DcMotor.class, "Right Motor");
        FlyWheel_Big = hardwareMap.get(DcMotor.class, " FlyWheel_Big");
        FlyWheel_Small = hardwareMap.get(DcMotor.class, "FlyWheel_Small");
        Top_Servo = hardwareMap.get(Servo.class, "Top_Servo");
        
        visionportal = VisionPortal.easyCreateWithDefaults(webcam);
        
        FlyWheel_Big.setDirection(DcMotor.Direction.FORWARD);
        FlyWheel_Small.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        
        while (opModeIsActive()) {
            
            double y1 = -gamepad1.right_stick_y;
            double y2 = -gamepad1.left_stick_y;
            double x1 = -gamepad1.right_stick_x;
            double x2 = -gamepad1.left_stick_x;
            double RWheelPower =((y2-x2)*WheelControl)/100;
            double LWheelPower =((y2+x2)*WheelControl)/100;
            double PowerT = ((-gamepad1.right_trigger)*FlyWheelControl)/100;
            double PowerA = (-gamepad1.left_trigger);
            boolean triggerValue = gamepad1.left_bumper;
            
            if (Inverted == false) 
            {
                FlyWheel_Big.setDirection(DcMotor.Direction.FORWARD);
                FlyWheel_Small.setDirection(DcMotor.Direction.REVERSE);
            } else {
                FlyWheel_Big.setDirection(DcMotor.Direction.REVERSE);
                FlyWheel_Small.setDirection(DcMotor.Direction.FORWARD);
            }
            
            if (Harry)
            {
                RWheelPower =((y1-x2)*WheelControl)/100;
                LWheelPower =((y1+x2)*WheelControl)/100;
            }
            
            leftMotor.setPower(LWheelPower);
            rightMotor.setPower(RWheelPower);
            FlyWheel_Big.setPower(PowerT);
            FlyWheel_Small.setPower(PowerA);
            
            if (triggerValue == true) {
                Top_Servo.setPosition(.3);
            } else {
                Top_Servo.setPosition(0);
            }
            
            Wheel_Motors_Speed_Control();
            Fly_Wheel_Motors_Speed_Control();
            
            telemetry.addData("Left Power", LWheelPower);
            telemetry.addData("Right Power", RWheelPower);
            telemetry.addData("FlyWheel Big", PowerT);
            telemetry.addData("FlyWheel Small", PowerA);
            telemetry.addData("Top Servo", triggerValue);
            telemetry.addData("Fly Wheel Speed", FlyWheelControl+"%");
            telemetry.addData("Fly Wheel Speed", WheelControl+"%");
            telemetry.addData("HarryMode: ", Harry);
            telemetry.addData("Inverted: ", Inverted);
            telemetry.update();
        }
    webcam.close();
    }
    
    public void Fly_Wheel_Motors_Speed_Control () {
        if (gamepad1.dpad_up) {
            if  (timer1 == 0) {
                if (FlyWheelControl != 100) {
                   FlyWheelControl+=5;
                   timer1 = 1500;
                }
            } else {
                timer1-=1;
            }
        }
        if (gamepad1.dpad_down) {
            if  (timer1 == 0) {
                if (FlyWheelControl != 0) {
                   FlyWheelControl-=5;
                   timer1 = 1500;
                }
            } else {
                timer1-=1;
            }
        }
    }
    
    public void Wheel_Motors_Speed_Control () {
        if (gamepad1.dpad_right) {
            if  (timer2 == 0) {
                if (Harry) {Harry = false;} else {Harry = true;}
                timer2 = 3000;
            } else {
                timer2-=1;
            }
        }
        if (gamepad1.xWasPressed()) {
            if (Inverted) {Inverted = false;} else {Inverted = true;}
        }
    }
}



