package org.firstinspires.ftc.teamcode.;




import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;




import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;




@TeleOp(name = "roblox_copy_1")
public class roblox_copy_1 extends LinearOpMode {
//ur e bike is not coming



    // ===== AURA AIDEN'S SERVO =====
    public Servo LEFT_INTAKE, RIGHT_INTAKE;




    public static final double MID_SERVO   =  0.5;
    public static final double INTAKE_SPEED   =  0.25;
    double intakeOffset = 0;




    // ===== DRIVE =====
    DcMotor LF, RF, LB, RB;




    // ===== IMU =====
    IMU imu;




    // ===== CONTROL STATES =====
    boolean invertedControls = false;
    boolean lastInvert = false;
    boolean lastYawReset = false;




    ElapsedTime buttonDelay = new ElapsedTime();




    @Override
    public void runOpMode() {




        // ===== HARDWARE MAP =====
        LF = hardwareMap.dcMotor.get("FL");
        LB = hardwareMap.dcMotor.get("BL");
        RF = hardwareMap.dcMotor.get("FR");
        RB = hardwareMap.dcMotor.get("BR");




        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);




        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        LEFT_INTAKE  = hardwareMap.get(Servo.class, "left_hand");
        RIGHT_INTAKE = hardwareMap.get(Servo.class, "right_hand");






        // ===== IMU INIT =====
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));




        waitForStart();




        while (opModeIsActive()) {




            // ===== INPUT =====
            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;




            // ===== AURA AIDEN'S SERVO INPUT ====
            boolean hasRun = false;


            while (opModeIsActive()) {
                if (!hasRun) {
                    // Code you want to run once
                    intakeOffset = Range.clip(intakeOffset, -0.5, 0.5);
                    LEFT_INTAKE.setPosition(INTAKE_SPEED + intakeOffset);
                    RIGHT_INTAKE.setPosition(INTAKE_SPEED - intakeOffset);


                    hasRun = true; // Prevent it from running again
                }
            }


            while (gamepad1.right_bumper)
                intakeOffset += INTAKE_SPEED;
            while (gamepad1.left_bumper)
                intakeOffset -= INTAKE_SPEED;




            // ===== SPEED CONTROL =====
            double driveScale = 1.0;
            if (gamepad1.right_trigger > 0.1) driveScale = 0.3;
            else if (gamepad1.left_trigger > 0.1) driveScale = 0.6;




            // ===== INVERT CONTROLS =====
            if (gamepad1.b && !lastInvert && buttonDelay.seconds() > 0.3) {
                invertedControls = !invertedControls;
                buttonDelay.reset();
            }
            lastInvert = gamepad1.b;




            if (invertedControls) {
                lx = -lx;
                ly = -ly;
            }




            // ===== RESET YAW =====
            if (gamepad1.y && !lastYawReset) {
                imu.resetYaw();
            }
            lastYawReset = gamepad1.y;




            // ===== FIELD CENTRIC DRIVE =====
            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);




            double cosA = Math.cos(-yaw);
            double sinA = Math.sin(-yaw);




            double rotX = lx * cosA - ly * sinA;
            double rotY = lx * sinA + ly * cosA;




            setDrive(
                    (rotY + rotX + rx) * driveScale,
                    (rotY - rotX + rx) * driveScale,
                    (rotY - rotX - rx) * driveScale,
                    (rotY + rotX - rx) * driveScale
            );




            // ===== TELEMETRY =====
            telemetry.addData("Inverted", invertedControls);
            telemetry.update();
        }
    }




    // ===== DRIVE HELPER =====
    private void setDrive(double fl, double bl, double fr, double br) {
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(bl)),
                Math.max(Math.abs(fr), Math.abs(br)));




        if (max > 1.0) {
            fl /= max;
            bl /= max;
            fr /= max;
            br /= max;
        }




        LF.setPower(fl);
        LB.setPower(bl);
        RF.setPower(fr);
        RB.setPower(br);
    }
}




