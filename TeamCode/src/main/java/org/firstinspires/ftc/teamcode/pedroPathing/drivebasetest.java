package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// This TeleOp controls the full robot:
//   Gamepad 1 → Drive (field-centric), Intake rollers, Grabbers, Slides
//   Gamepad 2 → Arm (Elbow, Wrist, Claw)
@TeleOp(name = "FULL_ROBOT_TEST")
public class drivebasetest extends LinearOpMode {

    // =====================================================================
    //  HARDWARE DECLARATIONS
    //  These variables hold references to physical hardware on the robot.
    //  The names in hardwareMap.get("...") must match your Driver Hub config.
    // =====================================================================

    // --- Drive Motors (Mecanum wheels) ---
    DcMotor LF, RF, LB, RB; // Left-Front, Right-Front, Left-Back, Right-Back

    // --- Linear Slide Motors ---
    // DcMotorEx gives access to encoder position, useful for slide height feedback
    DcMotorEx SLIDE_LEFT, SLIDE_RIGHT;

    // --- IMU (built into Control Hub) ---
    // Used for field-centric driving — robot moves relative to field, not itself
    IMU imu;

    // --- Intake Servos (spin to intake/eject game pieces) ---
    Servo LEFT_INTAKE, RIGHT_INTAKE;

    // --- Grabber Servos (open/close a claw or gripper on the intake side) ---
    Servo LEFT_GRABBER, RIGHT_GRABBER;

    // --- Arm Servos ---
    Servo ELBOW; // Rotates the arm up/down
    Servo WRIST; // Tilts the end effector
    Servo CLAW;  // Opens/closes the scoring claw


    // =====================================================================
    //  TUNABLE CONSTANTS
    //  All values below are marked // TUNE: — change them to dial in behavior.
    //  Servo positions range from 0.0 to 1.0 (0° to ~270° depending on servo).
    // =====================================================================

    // --- Drive Scale ---
    // TUNE: Lower this (e.g. 0.7) to slow the robot down for precision driving
    static final double DRIVE_SCALE = 1.0;

    // --- Intake Servo Positions ---
    // The two intake servos face opposite directions, so one runs "forward"
    // while the other runs at (1.0 - position) to spin the same physical direction.
    // TUNE: Swap INTAKE_FORWARD and INTAKE_REVERSE if intake spins the wrong way
    static final double INTAKE_FORWARD = 1.0; // Spin inward (collect pieces)
    static final double INTAKE_REVERSE = 0.0; // Spin outward (eject pieces)
    static final double INTAKE_STOP    = 0.5; // Continuous servo stop position (must be exactly 0.5)

    // --- Grabber Servo Positions ---
    // TUNE: Adjust GRABBER_OPEN if the grabber doesn't open wide enough
    // TUNE: Adjust GRABBER_CLOSED if it crushes the piece or doesn't grip firmly
    static final double GRABBER_OPEN     = 0.20; // Grabber fully open
    static final double GRABBER_CLOSED   = 0.1; // Grabber fully closed / gripping
    static final double GRABBER_STARTPOS = 0.1; // Position at init (before match starts)

    // --- Elbow Servo Positions ---
    // Controls the main arm rotation (up/down)
    // TUNE: ELBOW_GROUND — how far down the arm goes to pick from the floor
    // TUNE: ELBOW_TRANSFER — mid-position for handing off a piece to the scoring claw
    // TUNE: ELBOW_SCORE — raised position for scoring on the high basket/bar
    static final double ELBOW_GROUND   = 0.10; // Arm pointing down toward ground
    static final double ELBOW_TRANSFER = 0.45; // Arm at transfer/handoff height
    static final double ELBOW_SCORE    = 0.85; // Arm raised to scoring position
    static final double ELBOW_STARTPOS = 0.50; // Init position

    // --- Wrist Servo Positions ---
    // Tilts the end effector to keep the claw level at different arm angles
    // TUNE: WRIST_FLAT — wrist angle when collecting from the ground
    // TUNE: WRIST_SCORE — wrist angle when scoring (may need to tilt to drop piece)
    static final double WRIST_FLAT     = 0.20; // Wrist flat / parallel to ground
    static final double WRIST_SCORE    = 0.75; // Wrist tilted for scoring orientation
    static final double WRIST_STARTPOS = 0.50; // Init position

    // --- Claw Servo Positions ---
    // TUNE: CLAW_OPEN — how far the claw opens (wider = easier to grab, but slower)
    // TUNE: CLAW_CLOSED — how far it closes (tighter = firmer grip; don't over-tighten)
    static final double CLAW_OPEN      = 0.52; // Claw open to receive/release piece
    static final double CLAW_CLOSED    = 0.50; // Claw closed and gripping
    static final double CLAW_STARTPOS  = 0.52;  // Init position (fully retracted)

    // --- Slide Power ---
    // TUNE: Lower this if slides are jerky or skip; raise it if they're too slow
    static final double SLIDE_POWER = 0.7;


    // =====================================================================
    //  OP MODE ENTRY POINT
    // =====================================================================
    @Override
    public void runOpMode() {

        // =================================================================
        //  HARDWARE MAP — connects variable names to physical devices
        //  The string must exactly match the name in the Driver Hub config
        // =================================================================

        // Drive motors
        LF = hardwareMap.dcMotor.get("FL");
        LB = hardwareMap.dcMotor.get("BL");
        RF = hardwareMap.dcMotor.get("FR");
        RB = hardwareMap.dcMotor.get("BR");

        // Slide motors (DcMotorEx = extended motor with encoder access)
        SLIDE_LEFT  = hardwareMap.get(DcMotorEx.class, "slide_left");
        SLIDE_RIGHT = hardwareMap.get(DcMotorEx.class, "slide_right");

        // IMU (built into Control Hub or Expansion Hub)
        imu = hardwareMap.get(IMU.class, "imu");

        // Intake continuous rotation servos
        LEFT_INTAKE  = hardwareMap.get(Servo.class, "left_intake");
        RIGHT_INTAKE = hardwareMap.get(Servo.class, "right_intake");

        // Grabber servos (intake-side claw)
        LEFT_GRABBER  = hardwareMap.get(Servo.class, "leftgrabber");
        RIGHT_GRABBER = hardwareMap.get(Servo.class, "rightgrabber");

        // Arm servos
        ELBOW = hardwareMap.get(Servo.class, "elbow");
        WRIST = hardwareMap.get(Servo.class, "wrist");
        CLAW  = hardwareMap.get(Servo.class, "claw");


        // =================================================================
        //  DRIVE MOTOR CONFIGURATION
        //  Motors on the same side spin opposite physical directions because
        //  they are mirror-mounted. Reverse one side so setPower(+) always
        //  means "forward" on both sides.
        //  NOTE: If your robot drives sideways or backward, flip directions here.
        // =================================================================
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);

        // BRAKE = wheels lock when power = 0 (better for precision, stops drifting)
        // FLOAT = wheels spin freely when power = 0 (smoother but drifts more)
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // =================================================================
        //  SLIDE MOTOR CONFIGURATION
        //  Both motors lift the same mechanism — one is physically reversed,
        //  so we reverse it in software so positive power = slides go UP.
        // =================================================================
        SLIDE_LEFT.setDirection(DcMotorSimple.Direction.FORWARD);
        SLIDE_RIGHT.setDirection(DcMotorSimple.Direction.REVERSE);

        SLIDE_LEFT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SLIDE_RIGHT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoder counts to 0 at startup so slide position is consistent
        SLIDE_LEFT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SLIDE_RIGHT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // RUN_WITHOUT_ENCODER = manual power control (driver uses dpad to drive slides)
        // Switch to RUN_TO_POSITION later if you want automatic height presets
        SLIDE_LEFT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SLIDE_RIGHT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // =================================================================
        //  IMU CONFIGURATION
        //  Tell the SDK how the Control Hub is physically mounted on the robot.
        //  LogoFacingDirection = which way the REV logo faces
        //  UsbFacingDirection  = which way the USB port faces
        //  TUNE: Change these if your hub is mounted differently
        // =================================================================
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));


        // =================================================================
        //  INITIAL SERVO POSITIONS (set before match starts)
        //  Servos jump to these positions the moment the code initializes.
        //  Make sure these are safe positions that won't cause collisions.
        // =================================================================
        LEFT_INTAKE.setPosition(INTAKE_STOP);
        RIGHT_INTAKE.setPosition(INTAKE_STOP);

        LEFT_GRABBER.setPosition(GRABBER_CLOSED);
        RIGHT_GRABBER.setPosition(GRABBER_CLOSED);

        ELBOW.setPosition(ELBOW_STARTPOS);
        WRIST.setPosition(WRIST_STARTPOS);
        CLAW.setPosition(CLAW_STARTPOS);

        // Wait here until the Driver Hub's play button is pressed
        waitForStart();


        // =================================================================
        //  MAIN LOOP — runs repeatedly every ~10ms while the match is active
        // =================================================================
        while (opModeIsActive()) {

            // =============================================================
            //  GAMEPAD 1 — DRIVE (Field-Centric Mecanum)
            //
            //  Field-centric means the robot moves relative to the field:
            //  pushing the stick "forward" always drives away from the driver,
            //  regardless of which direction the robot is facing.
            //
            //  Controls:
            //    Left stick X/Y  → strafe / forward-backward translation
            //    Right stick X   → rotation (turning left/right)
            // =============================================================

            double lx = gamepad1.left_stick_x;          // Strafe input (-1 left, +1 right)
            double ly = -gamepad1.left_stick_y;          // Forward input (inverted: stick up = +1)
            double rx = gamepad1.right_stick_x;          // Rotation input

            // Get current robot heading from IMU (in radians, -π to +π)
            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the stick input by the negative of the robot's yaw.
            // This converts driver-relative input into field-relative movement.
            double cosA = Math.cos(-yaw);
            double sinA = Math.sin(-yaw);

            double rotX = lx * cosA - ly * sinA; // Field-relative strafe
            double rotY = lx * sinA + ly * cosA; // Field-relative forward

            // Mecanum wheel mixing formula:
            //   FL = forward + strafe + rotate
            //   BL = forward - strafe + rotate
            //   FR = forward - strafe - rotate
            //   BR = forward + strafe - rotate
            // setDrive() also normalizes values so no wheel exceeds 1.0 power
            setDrive(
                    (rotY + rotX + rx) * DRIVE_SCALE, // FL
                    (rotY - rotX + rx) * DRIVE_SCALE, // BL
                    (rotY - rotX - rx) * DRIVE_SCALE, // FR
                    (rotY + rotX - rx) * DRIVE_SCALE  // BR
            );


            // =============================================================
            //  GAMEPAD 1 — INTAKE ROLLERS
            //  Continuous rotation servos — 0.5 = stop, 1.0 / 0.0 = spin
            //  LEFT and RIGHT face opposite directions, so RIGHT gets mirrored
            //  using (1.0 - position).
            //
            //  X button → spin inward (collect game piece)
            //  A button → spin outward (eject game piece)
            //  No button → stop
            // =============================================================
            if (gamepad1.x) {
                // Collect: left spins INTAKE_FORWARD, right mirrors it
                LEFT_INTAKE.setPosition(INTAKE_FORWARD);
                RIGHT_INTAKE.setPosition(1.0 - INTAKE_FORWARD);
            } else if (gamepad1.a) {
                // Eject: reverse both rollers
                LEFT_INTAKE.setPosition(INTAKE_REVERSE);
                RIGHT_INTAKE.setPosition(1.0 - INTAKE_REVERSE);
            } else {
                // Idle: stop both rollers
                LEFT_INTAKE.setPosition(INTAKE_STOP);
                RIGHT_INTAKE.setPosition(1.0 - INTAKE_STOP);
            }


            // =============================================================
            //  GAMEPAD 1 — INTAKE GRABBERS
            //  Standard position servos that grip/release a game piece
            //  after intake rollers bring it in.
            //
            //  Right Bumper → open grabbers
            //  Left Bumper  → close grabbers
            // =============================================================
            if (gamepad1.right_bumper) {
                LEFT_GRABBER.setPosition(GRABBER_OPEN);
                RIGHT_GRABBER.setPosition(GRABBER_OPEN);
            } else if (gamepad1.left_bumper) {
                LEFT_GRABBER.setPosition(GRABBER_CLOSED);
                RIGHT_GRABBER.setPosition(GRABBER_CLOSED);
            }
            // NOTE: No else here — grabbers hold their last position when no button pressed


            // =============================================================
            //  GAMEPAD 2 — ELBOW (Arm rotation)
            //  Moves the scoring arm to preset angles.
            //
            //  B → Score position  (arm up high)
            //  X → Ground position (arm down low)
            //  Y → Transfer position (mid-height handoff)
            //
            //  TUNE: Adjust ELBOW_GROUND / ELBOW_TRANSFER / ELBOW_SCORE above
            // =============================================================
            if (gamepad2.b) {
                ELBOW.setPosition(ELBOW_SCORE);
            } else if (gamepad2.x) {
                ELBOW.setPosition(ELBOW_GROUND);
            } else if (gamepad2.y) {
                ELBOW.setPosition(ELBOW_TRANSFER);
            }


            // =============================================================
            //  GAMEPAD 2 — WRIST
            //  Tilts the end effector to keep claw at the right angle.
            //
            //  Right Bumper → Score angle (tilt to drop into basket/bar)
            //  Left Bumper  → Flat angle  (level for pickup)
            //
            //  TUNE: Adjust WRIST_FLAT / WRIST_SCORE above
            // =============================================================
            if (gamepad2.right_bumper) {
                WRIST.setPosition(WRIST_SCORE);
            } else if (gamepad2.left_bumper) {
                WRIST.setPosition(WRIST_FLAT);
            }


            // =============================================================
            //  GAMEPAD 2 — CLAW (Scoring claw open/close)
            //
            //  A → Open claw  (release piece into scoring area)
            //  B is NOT used for claw here — B is used for ELBOW_SCORE above.
            //      (Original code had a conflict: B was mapped to both
            //       ELBOW_SCORE and CLAW_CLOSED simultaneously. Fixed below.)
            //
            //  Right Trigger (> 0.5) → Close claw (grip piece)
            //
            //  TUNE: Adjust CLAW_OPEN / CLAW_CLOSED above
            // =============================================================
            if (gamepad2.a) {
                CLAW.setPosition(CLAW_OPEN);
            } else if (gamepad2.right_trigger > 0.5) {
                // Using right trigger for close instead of B, which conflicts with elbow
                CLAW.setPosition(CLAW_CLOSED);
            }


            // =============================================================
            //  GAMEPAD 1 — LINEAR SLIDES
            //  Manual power control — driver holds the button to move slides.
            //  Slides brake (hold position) when no button is pressed.
            //
            //  D-Pad Up   → Extend slides (up)
            //  D-Pad Down → Retract slides (down)
            //
            //  TUNE: Adjust SLIDE_POWER above (0.0–1.0)
            //  TIP: Add encoder-based limits here later to prevent over-extension
            // =============================================================
            double slidePower = 0;

            if (gamepad1.dpad_up) {
                slidePower = SLIDE_POWER;   // Positive = extend up
            } else if (gamepad1.dpad_down) {
                slidePower = -SLIDE_POWER;  // Negative = retract down
            }
            // slidePower stays 0 if no dpad button → motors brake and hold position

            SLIDE_LEFT.setPower(slidePower);
            SLIDE_RIGHT.setPower(slidePower);


            // =============================================================
            //  TELEMETRY — Displayed on the Driver Hub screen during the match
            //  Useful for debugging servo positions and slide heights in real time
            // =============================================================
            telemetry.addData("Slides L (ticks)", SLIDE_LEFT.getCurrentPosition());
            telemetry.addData("Slides R (ticks)", SLIDE_RIGHT.getCurrentPosition());
            telemetry.addData("Claw Position",    CLAW.getPosition());
            telemetry.addData("Elbow Position",   ELBOW.getPosition());
            telemetry.addData("Wrist Position",   WRIST.getPosition());
            telemetry.update();
        }
    }


    // =====================================================================
    //  HELPER: setDrive(fl, bl, fr, br)
    //
    //  Sets power to all four drive motors.
    //  Normalizes values so that if any wheel power exceeds 1.0,
    //  all four are scaled down proportionally.
    //  This preserves the intended direction while staying within motor limits.
    // =====================================================================
    private void setDrive(double fl, double bl, double fr, double br) {

        // Find the largest absolute power value across all four wheels
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(bl)),
                Math.max(Math.abs(fr), Math.abs(br)));

        // If any motor would exceed 1.0, scale all four down by the same factor
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