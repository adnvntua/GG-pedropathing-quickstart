package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Registers this class as a TeleOp opmode visible in the Driver Hub menu
@TeleOp(name = "GearGurus_TeleOp")
public class roblox_copy_1 extends LinearOpMode {

    // =========================================================================
    // ===== TUNING CONSTANTS — Edit ONLY these values to tune the robot! =====
    //   All positions and powers are collected here so you never have to hunt
    //   through the code to change a number.
    // =========================================================================

    // --- SLIDE POSITIONS (encoder ticks) ------------------------------------
    // Run the robot, use the manual stick to find the right tick counts,
    // then plug those numbers in here.
    static final int    SLIDE_GROUND        = 0;       // Fully retracted / resting
    static final int    SLIDE_LOW           = 300;     // Low basket height
    static final int    SLIDE_MID           = 700;     // Mid-scoring height
    static final int    SLIDE_HIGH          = 1400;    // High basket / max extension

    // Power sent to slide motors while driving to a preset (0.0–1.0).
    // Start low (0.4) and raise slowly. Too high = jerky/overshooting.
    static final double SLIDE_POWER         = 0.7;

    // Power sent during MANUAL stick override (usually same as above or lower)
    static final double SLIDE_MANUAL_POWER  = 0.6;

    // --- INTAKE SERVO POSITIONS (continuous-rotation servos) ----------------
    // 1.0 = full speed one way, 0.5 = stopped, 0.0 = full speed other way.
    // If intake spins the wrong direction, swap FORWARD and REVERSE values.
    static final double INTAKE_FORWARD      = 1.0;    // Spin inward (collecting)
    static final double INTAKE_REVERSE      = 0.0;    // Spin outward (ejecting)
    static final double INTAKE_STOP         = 0.5;    // Stopped (neutral)

    // --- ELBOW SERVO POSITIONS (0.0–1.0) ------------------------------------
    // 0.0 = fully one direction, 1.0 = fully other direction.
    // Adjust until elbow sits at the right angle for each task.
    static final double ELBOW_GROUND        = 0.10;   // Aimed down for floor pickup
    static final double ELBOW_TRANSFER      = 0.45;   // Mid-height for sample handoff
    static final double ELBOW_SCORE         = 0.85;   // Raised for scoring into basket

    // --- WRIST SERVO POSITIONS (0.0–1.0) ------------------------------------
    static final double WRIST_FLAT          = 0.20;   // Horizontal for picking up
    static final double WRIST_SCORE         = 0.75;   // Angled for basket scoring

    // --- CLAW SERVO POSITIONS (0.0–1.0) ------------------------------------
    static final double CLAW_OPEN           = 0.25;   // Open to grab or release
    static final double CLAW_CLOSE          = 0.70;   // Closed to grip sample

    // --- GRABBER SERVO POSITIONS
    static final double GRABBER_OPEN = 0.20;
    static final double GRABBER_CLOSED = 0.70;

    // How much a servo moves PER LOOP when held for fine adjustment (right stick).
    // Smaller = finer tuning. Good range: 0.005 (very fine) – 0.03 (coarser).
    static final double SERVO_INCREMENT     = 0.015;

    // =========================================================================
    // ===== HARDWARE DECLARATIONS =====
    // =========================================================================

    // Mecanum drive motors
    DcMotor   LF, RF, LB, RB;

    // Linear slide motors — DcMotorEx gives access to encoder target-position API
    DcMotorEx SLIDE_LEFT, SLIDE_RIGHT;

    // Continuous-rotation intake rollers
    Servo LEFT_INTAKE, RIGHT_INTAKE;

    // Grabber servos
    Servo LEFT_GRABBER, RIGHT_GRABBER;

    // Arm servos
    Servo ELBOW;   // Rotates the arm up/down
    Servo WRIST;   // Rotates the wrist/sample orientation
    Servo CLAW;    // Opens and closes the grabber

    // IMU (built into the Control Hub) — used for field-centric driving
    IMU imu;

    // =========================================================================
    // ===== RUNTIME STATE VARIABLES =====
    // =========================================================================

    // --- Drive state ---
    boolean invertedControls  = false; // True = controls flipped 180° (useful when robot faces driver)
    boolean lastInvert        = false; // Last known B-button state (edge detection)
    boolean lastYawReset      = false; // Last known Y-button state (edge detection)

    // --- Intake state ---
    boolean intakeRunning     = false; // True while intake is spinning inward
    boolean intakeReversed    = false; // True while intake is ejecting
    boolean lastIntakeToggle  = false; // Last known X-button state (gamepad1)
    boolean lastIntakeReverse = false; // Last known A-button state (gamepad1)

    // --- Claw state ---
    boolean clawOpen          = false; // True = claw is open
    boolean lastClawToggle    = false; // Last known A-button state (gamepad2)

    // Live servo positions — updated by preset buttons AND fine-tune stick
    double elbowPos           = ELBOW_TRANSFER; // Starts at transfer position
    double wristPos           = WRIST_FLAT;     // Starts flat for pickup

    // Shared debounce timer — resets whenever ANY toggle button is pressed.
    // This prevents a single button press from registering multiple times.
    // NOTE: All toggles share this timer, so pressing two toggles within 0.3s
    //       may cause one to be ignored. This is intentional to avoid double-presses.
    ElapsedTime buttonDelay   = new ElapsedTime();

    // =========================================================================
    // ===== MAIN OPMODE ENTRY POINT =====
    // =========================================================================
    @Override
    public void runOpMode() {

        // =====================================================================
        // ===== HARDWARE MAP — String names must match your Driver Hub config =====
        // =====================================================================

        // --- Drive motors ---
        LF = hardwareMap.dcMotor.get("FL");  // Front-Left motor port name
        LB = hardwareMap.dcMotor.get("BL");  // Back-Left motor port name
        RF = hardwareMap.dcMotor.get("FR");  // Front-Right motor port name
        RB = hardwareMap.dcMotor.get("BR");  // Back-Right motor port name

        // Reverse left-side motors so all wheels spin the same relative direction.
        // If your robot drives sideways instead of forward, swap REVERSE/FORWARD here.
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.REVERSE); // Keeping original directions from working code
        RB.setDirection(DcMotorSimple.Direction.FORWARD);

        // BRAKE: wheels resist movement when power is cut (prevents rolling)
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Slide motors (DcMotorEx for encoder position control) ---
        SLIDE_LEFT  = hardwareMap.get(DcMotorEx.class, "slideleft");

        // One motor must be reversed so both extend the slides in the same direction.
        // If slides fight each other, swap FORWARD/REVERSE on one of them.
        SLIDE_LEFT.setDirection(DcMotorSimple.Direction.FORWARD);

        // BRAKE: slides hold their position when power is removed (no drift)
        SLIDE_LEFT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Zero the encoders at startup — all preset positions are relative to this
        SLIDE_LEFT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // RUN_USING_ENCODER lets the motor use encoder feedback for smoother power
        SLIDE_LEFT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- Intake continuous-rotation servos ---
        LEFT_INTAKE  = hardwareMap.get(Servo.class, "leftintake");
        RIGHT_INTAKE = hardwareMap.get(Servo.class, "rightintake");

        // --- Grabber servos
        LEFT_GRABBER = hardwareMap.get(Servo.class, "leftgrabber");
        RIGHT_GRABBER = hardwareMap.get(Servo.class, "rightgrabber");

        // --- Arm / claw servos ---
        ELBOW = hardwareMap.get(Servo.class, "elbow"); // Arm pivot up/down
        WRIST = hardwareMap.get(Servo.class, "wrist"); // Sample orientation
        CLAW  = hardwareMap.get(Servo.class, "claw");  // Gripper open/close

        // --- IMU initialization ---
        imu = hardwareMap.get(IMU.class, "imu");
        // Tell the SDK how the Control Hub is mounted on the robot.
        // Change these if your hub is oriented differently.
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        // --- Set safe starting positions before match begins ---
        ELBOW.setPosition(elbowPos);          // Start at transfer position
        WRIST.setPosition(wristPos);          // Start flat
        CLAW.setPosition(CLAW_CLOSE);         // Start clamped so nothing falls out
        LEFT_INTAKE.setPosition(INTAKE_STOP); // Intake stopped
        RIGHT_INTAKE.setPosition(INTAKE_STOP);
        LEFT_GRABBER.setPosition(GRABBER_CLOSED);
        RIGHT_GRABBER.setPosition(GRABBER_CLOSED);

        waitForStart(); // Hang here until the referee presses START on the Driver Hub

        // =====================================================================
        // ===== MAIN CONTROL LOOP — runs every ~20ms while opmode is active =====
        // =====================================================================
        while (opModeIsActive()) {

            // =================================================================
            // GAMEPAD 1 — DRIVER
            // Controls: Drive, intake, speed modes, invert, yaw reset
            // =================================================================

            // Read driver joystick axes
            double lx = gamepad1.left_stick_x;    // Positive = strafe RIGHT
            double ly = -gamepad1.left_stick_y;   // Positive = drive FORWARD (stick Y is inverted)
            double rx  = gamepad1.right_stick_x;  // Positive = rotate CLOCKWISE

            // ----- SPEED MODES -----
            // Default = 100% | Left trigger = 60% (careful) | Right trigger = 30% (precise)
            double driveScale = 1.0;
            if      (gamepad1.right_trigger > 0.1) driveScale = 0.30; // Precision mode
            else if (gamepad1.left_trigger  > 0.1) driveScale = 0.60; // Medium mode

            // ----- INVERT CONTROLS (B button) -----
            // Flips the drive direction — handy when the robot is oriented away from driver.
            // Edge detection: !lastInvert means we only fire on the FIRST frame the button is held.
            if (gamepad1.b && !lastInvert && buttonDelay.seconds() > 0.3) {
                invertedControls = !invertedControls; // Toggle the flag
                buttonDelay.reset();                  // Restart debounce window
            }
            lastInvert = gamepad1.b; // Save state for next loop's edge detection

            if (invertedControls) {
                lx = -lx; // Flip strafe direction
                ly = -ly; // Flip forward/back direction
            }

            // ----- RESET YAW (Y button) -----
            // Redefines "forward" for field-centric to wherever the robot is currently pointing.
            // Use this if the robot spins out and field-centric gets confused.
            if (gamepad1.y && !lastYawReset) {
                imu.resetYaw(); // Zero the heading — robot's current direction becomes "forward"
            }
            lastYawReset = gamepad1.y;

            // ----- INTAKE TOGGLE (X = start/stop, A = reverse/eject) -----
            // X button: cycle the intake between running and stopped
            if (gamepad1.x && !lastIntakeToggle && buttonDelay.seconds() > 0.3) {
                intakeRunning  = !intakeRunning; // Flip running state
                intakeReversed = false;          // Clear reverse when normally toggling
                buttonDelay.reset();
            }
            lastIntakeToggle = gamepad1.x;

            // A button: toggle eject mode (spits sample out)
            if (gamepad1.a && !lastIntakeReverse && buttonDelay.seconds() > 0.3) {
                intakeReversed = !intakeReversed;           // Flip eject state
                if (intakeReversed) intakeRunning = true;   // Auto-start if not already running
                buttonDelay.reset();
            }
            lastIntakeReverse = gamepad1.a;

            // Determine the final speed value to send to both intake servos
            double intakeSpeed;
            if      (!intakeRunning)  intakeSpeed = INTAKE_STOP;    // Off
            else if (intakeReversed)  intakeSpeed = INTAKE_REVERSE; // Ejecting
            else                      intakeSpeed = INTAKE_FORWARD; // Collecting

            // Left and Right intake face opposite directions, so mirror the position.
            // 1.0 - x flips the value: 1.0 becomes 0.0, 0.5 stays 0.5, etc.
            LEFT_INTAKE.setPosition(intakeSpeed);
            RIGHT_INTAKE.setPosition(1.0 - intakeSpeed); // Mirrored so both spin toward center

            // ----- FIELD-CENTRIC DRIVE -----
            // Get the robot's current absolute heading from the IMU
            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the joystick vector by the negative yaw to align with the field.
            // This means pushing the stick "up" always drives toward the far end of the field,
            // regardless of which way the robot is facing.
            double cosA = Math.cos(-yaw); // X component of the rotation matrix
            double sinA = Math.sin(-yaw); // Y component of the rotation matrix

            double rotX = lx * cosA - ly * sinA; // Field-relative strafe
            double rotY = lx * sinA + ly * cosA; // Field-relative forward

            // Calculate each mecanum wheel's power using standard holonomic equations:
            //   Front-Left  = forward + strafe + rotate
            //   Back-Left   = forward - strafe + rotate
            //   Front-Right = forward - strafe - rotate
            //   Back-Right  = forward + strafe - rotate
            setDrive(
                    (rotY + rotX + rx) * driveScale,  // FL
                    (rotY - rotX + rx) * driveScale,  // BL
                    (rotY - rotX - rx) * driveScale,  // FR
                    (rotY + rotX - rx) * driveScale   // BR
            );

            // =================================================================
            // GAMEPAD 2 — OPERATOR
            // Controls: Slides, elbow, wrist, claw
            // =================================================================

            // ----- SLIDE PRESET POSITIONS (D-Pad) -----
            // Each D-Pad direction sends the slides to a tunable target tick count.
            // The slides will automatically stop when they reach the target.
            if (gamepad2.dpad_up) {
                setSlideTo(SLIDE_HIGH);   // Extend fully for high basket
            } else if (gamepad2.dpad_right) {
                setSlideTo(SLIDE_MID);    // Mid-height scoring
            } else if (gamepad2.dpad_left) {
                setSlideTo(SLIDE_LOW);    // Low scoring height
            } else if (gamepad2.dpad_down) {
                setSlideTo(SLIDE_GROUND); // Fully retract slides
            }

            // ----- MANUAL SLIDE OVERRIDE (Left Stick Y, gamepad2) -----
            // Lets the operator nudge the slides past or between preset positions.
            // Pushing the stick overrides the encoder target and uses direct power.
            double manualSlide = -gamepad2.left_stick_y; // Up on stick = positive (extend)
            if (Math.abs(manualSlide) > 0.1) {           // Deadband: ignore stick drift < 10%
                // Switch to raw power mode — no encoder target, just direct drive
                SLIDE_LEFT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                SLIDE_RIGHT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                SLIDE_LEFT.setPower(manualSlide  * SLIDE_MANUAL_POWER); // Scale by manual power constant
                SLIDE_RIGHT.setPower(manualSlide * SLIDE_MANUAL_POWER);
            } else if (SLIDE_LEFT.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                // Stick was released, and we were in manual mode — stop and hold
                SLIDE_LEFT.setPower(0);
                SLIDE_RIGHT.setPower(0);
                // NOTE: To add PID holding later, call setSlideTo(currentPosition) here
                //       so the motor holds position via RUN_TO_POSITION.
            }

            // ----- CLAW TOGGLE (A button, gamepad2) -----
            // Single press opens the claw; press again to close.
            if (gamepad2.a && !lastClawToggle && buttonDelay.seconds() > 0.3) {
                clawOpen = !clawOpen;                                 // Flip claw state
                CLAW.setPosition(clawOpen ? CLAW_OPEN : CLAW_CLOSE); // Move to matching position
                buttonDelay.reset();
            }
            lastClawToggle = gamepad2.a;

            // ----- ELBOW PRESETS (Face buttons, gamepad2) -----
            // B = scoring position (arm up)
            // X = ground position (arm down for pickup)
            // Y = transfer position (arm mid-height to hand off to claw)
            if      (gamepad2.b) elbowPos = ELBOW_SCORE;
            else if (gamepad2.x) elbowPos = ELBOW_GROUND;
            else if (gamepad2.y) elbowPos = ELBOW_TRANSFER;

            // Fine-tune elbow with Right Stick Y (gamepad2)
            // Hold the stick to slowly creep the elbow — great for dialing in exact angles.
            double elbowAdjust = -gamepad2.right_stick_y; // Up on stick = increase position value
            if (Math.abs(elbowAdjust) > 0.1) {            // Deadband
                elbowPos += elbowAdjust * SERVO_INCREMENT; // Nudge by INCREMENT each loop
                elbowPos  = Math.max(0.0, Math.min(1.0, elbowPos)); // Clamp within servo range
            }

            ELBOW.setPosition(elbowPos); // Send final position to servo every loop

            // ----- WRIST PRESETS (Bumpers, gamepad2) -----
            // Right bumper = wrist to scoring angle
            // Left bumper  = wrist flat for floor pickup
            if      (gamepad2.right_bumper) wristPos = WRIST_SCORE;
            else if (gamepad2.left_bumper)  wristPos = WRIST_FLAT;

            // Fine-tune wrist with Right Stick X (gamepad2)
            // Tilt the stick sideways to slowly rotate the wrist.
            double wristAdjust = gamepad2.right_stick_x;
            if (Math.abs(wristAdjust) > 0.1) {            // Deadband
                wristPos += wristAdjust * SERVO_INCREMENT; // Nudge by INCREMENT each loop
                wristPos  = Math.max(0.0, Math.min(1.0, wristPos)); // Clamp within servo range
            }

            WRIST.setPosition(wristPos); // Send final position to servo every loop

            // =================================================================
            // TELEMETRY — Live data on the Driver Hub screen
            // Use this to read current positions when tuning constants above
            // =================================================================
            telemetry.addLine("=== DRIVE ===");
            telemetry.addData("  Inverted Controls", invertedControls);
            telemetry.addData("  Speed Scale",       "%.0f%%", driveScale * 100);

            telemetry.addLine("=== SLIDES ===");
            telemetry.addData("  Left  Ticks (current)", SLIDE_LEFT.getCurrentPosition());
            telemetry.addData("  Right Ticks (current)", SLIDE_RIGHT.getCurrentPosition());
            telemetry.addData("  Mode",                  SLIDE_LEFT.getMode());
            // ^ Use these tick values to set SLIDE_LOW / MID / HIGH constants above

            telemetry.addLine("=== INTAKE ===");
            telemetry.addData("  Running",  intakeRunning);
            telemetry.addData("  Reversed", intakeReversed);

            telemetry.addLine("=== ARM ===");
            telemetry.addData("  Elbow Position", "%.3f", elbowPos);
            telemetry.addData("  Wrist Position", "%.3f", wristPos);
            telemetry.addData("  Claw Open",      clawOpen);
            // ^ Use these position values to set ELBOW_*/WRIST_*/CLAW_* constants above

            telemetry.update(); // Push all telemetry lines to the Driver Hub display
        }
    }

    // =========================================================================
    // HELPER: Drive slides to a target encoder position
    //   RUN_TO_POSITION mode automatically stops the motor when it arrives.
    //   To add PID later, replace this method with a PIDF controller that reads
    //   getCurrentPosition() and adjusts power dynamically each loop.
    // =========================================================================
    private void setSlideTo(int targetTicks) {
        // Tell both motors where to go (in encoder ticks)
        SLIDE_LEFT.setTargetPosition(targetTicks);
        SLIDE_RIGHT.setTargetPosition(targetTicks);

        // RUN_TO_POSITION: the motor uses its internal controller to reach the target.
        // It will hold position once it arrives (as long as power > 0).
        SLIDE_LEFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SLIDE_RIGHT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power — motor internally limits itself to not overshoot the target.
        // This is NOT a speed cap; it determines how aggressively the motor drives toward the target.
        SLIDE_LEFT.setPower(SLIDE_POWER);
        SLIDE_RIGHT.setPower(SLIDE_POWER);
    }

    // =========================================================================
    // HELPER: Apply power to all four mecanum drive motors
    //   If any motor value exceeds ±1.0, scale all four down proportionally.
    //   This preserves the intended strafe/forward/turn ratio at high speeds.
    // =========================================================================
    private void setDrive(double fl, double bl, double fr, double br) {
        // Find the largest absolute power among all four wheels
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(bl)),
                Math.max(Math.abs(fr), Math.abs(br)));

        // If any motor exceeds the legal range of ±1.0, scale everyone down together
        if (max > 1.0) {
            fl /= max; // Each value divided by the same max keeps the ratio intact
            bl /= max;
            fr /= max;
            br /= max;
        }

        // Send final clamped power values to the motors
        LF.setPower(fl);
        LB.setPower(bl);
        RF.setPower(fr);
        RB.setPower(br);
    }
}