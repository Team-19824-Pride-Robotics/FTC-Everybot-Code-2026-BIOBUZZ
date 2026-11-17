package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TeleOp Linear OpMode", group="Linear OpMode")
public class teleopControlOpModeLinear extends LinearOpMode {


    // ElapsedTime tracks how long the OpMode has been running.
    private ElapsedTime runtime = new ElapsedTime();

//    private DcMotor leftFrontDrive  = null;
//    private DcMotor leftBackDrive   = null;
//    private DcMotor rightFrontDrive = null;
//    private DcMotor rightBackDrive  = null;

//    private DcMotor intake    = null;
    private DcMotor catapult1 = null;
    private DcMotor catapult2 = null;

// ============================================================
// SERVO HARDWARE DECLARATIONS
// ============================================================

    // Set to true for continuous-rotation servos, false for positional servos.
    private static final boolean USE_CR_SERVOS = false;

    private CRServo rightFlowerRampCR = null;
    private CRServo leftFlowerRampCR  = null;
    private Servo   rightFlowerRamp   = null;
    private Servo   leftFlowerRamp    = null;

    // "static final" means this value never changes — it is a constant.
    private static final double SERVO_RANGE_DEG = 270.0;

    // Adjust independently to account for differences in servo horn installation.
    private static final double LEFT_SERVO_START_DEG  = 240.0; //(135+105)
    private static final double RIGHT_SERVO_START_DEG =  30.0; //(135-105)

    private static final double SERVO_TRAVEL_DEG = 105.0;

    // Intermediate angle to filter Nectar away from the robot.
    private static final double SERVO_MID_DEG = 50.0;

// ============================================================

    // "double" stores decimal numbers. Motor power ranges from -1.0 (full reverse) to 1.0 (full forward).
    private static final double INTAKE_IN_POWER  =  1.0;
    private static final double INTAKE_OUT_POWER = -1.0;
    private static final double INTAKE_OFF_POWER =  0.0;
    private double intakePower = INTAKE_OFF_POWER;

    // Triggers are analog (0.0–1.0). Comparing to a threshold converts them to a yes/no value.
    private static final double TRIGGER_THRESHOLD  =  0.2;

    // Sign depends on motor wiring and directions set in init.
    private static final double SHOOT_4_POWER      = -1.00;
    private static final double SHOOT_3_POWER      = -0.96;
    private static final double SHOOT_2_POWER      = -0.89;
    private static final double SHOOT_1_POWER      = -0.85;
    private static final double CATAPULT_DOWN_POWER =  1.0;
    private static final double CATAPULT_HOLD_POWER =  0.0;

    // An enum is a fixed set of named states. Used here to track what the catapult is doing.
    private enum CatapultModes {UP, DOWN, HOLD}
    private CatapultModes pivotMode;
    private static final double catapultRunDuration    = 250; // milliseconds
    private static final double catapultReturnDuration = 300; // milliseconds — increase if catapult doesn't fully return down
    // "boolean" can only be true or false.
    private boolean catapultRunning = false;
    private ElapsedTime catapultTimer = new ElapsedTime();
    private double catapultActivePower = 0;
    private boolean previousUpButton = false;

    private static final double SERVO_POWER = 0.5;

    // Converts a physical angle in degrees to a servo position value (0.0–1.0).
    private static double angleToServo(double angleDeg) {
        return angleDeg / SERVO_RANGE_DEG;
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // hardwareMap links a Java variable to a physical port. Names MUST match the Robot Configuration exactly.
//        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
//        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
//        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");
//        intake    = hardwareMap.get(DcMotor.class, "intake");
        catapult1 = hardwareMap.get(DcMotor.class, "catapult1");
        catapult2 = hardwareMap.get(DcMotor.class, "catapult2");

        // Reverse right-side motors to account for mirrored drivetrain mounting.
        // If driving behaves backward, verify motor wiring and these direction settings.
//        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

//        intake.setDirection(DcMotor.Direction.FORWARD);    // Forward should INTAKE.
        catapult1.setDirection(DcMotor.Direction.FORWARD); // Forward should pivot DOWN.
        catapult2.setDirection(DcMotor.Direction.REVERSE); // Reverse should pivot DOWN.

// ============================================================
// SERVO HARDWARE INIT
// ============================================================
        if (USE_CR_SERVOS) {
            rightFlowerRampCR = hardwareMap.get(CRServo.class, "right_servo");
            leftFlowerRampCR  = hardwareMap.get(CRServo.class, "left_servo");
            leftFlowerRampCR.setDirection(CRServo.Direction.FORWARD);
            rightFlowerRampCR.setDirection(CRServo.Direction.REVERSE);
            leftFlowerRampCR.setPower(0.0);
            rightFlowerRampCR.setPower(0.0);
        } else {
            rightFlowerRamp = hardwareMap.get(Servo.class, "right_servo");
            leftFlowerRamp  = hardwareMap.get(Servo.class, "left_servo");
            leftFlowerRamp.setPosition(angleToServo(LEFT_SERVO_START_DEG));
            rightFlowerRamp.setPosition(angleToServo(RIGHT_SERVO_START_DEG));
        }
// ============================================================

        // Pause here until the driver presses START on the Driver Station.
        waitForStart();
        runtime.reset();

        double leftFrontPower  = 0;
        double rightFrontPower = 0;
        double leftBackPower   = 0;
        double rightBackPower  = 0;

        // opModeIsActive() returns false when the driver presses STOP, ending the loop.
        while (opModeIsActive()) {
            double max;

            // Gamepad stick Y is negative when pushed forward; negate so forward = positive axial.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = -gamepad1.left_stick_x;  // Remove negative sign if strafing wrong direction
            double yaw     = -gamepad1.right_stick_x; // Remove negative sign if turning wrong direction

            // Triggers are analog; comparing to TRIGGER_THRESHOLD treats them as a digital button.
            boolean intakeInButton  = gamepad1.left_trigger > TRIGGER_THRESHOLD;
            boolean intakeOutButton = gamepad1.left_bumper;
            // && means AND — both must be true. If both are held, cancel the inward command.
            if (intakeInButton && intakeOutButton) {
                intakeInButton = false;
            }

            boolean rampDownButton = gamepad1.dpad_down;
            boolean rampUpButton   = gamepad1.dpad_up;
            boolean rampMidButton  = gamepad1.dpad_left;
            if (rampDownButton && rampUpButton && rampMidButton) {
                rampDownButton = false;
            }

            boolean catapult4ballUpButton = gamepad1.right_trigger > TRIGGER_THRESHOLD;
            boolean catapultDownButton    = gamepad1.right_bumper;
            boolean catapult3ballUpButton = gamepad1.y;
            boolean catapult2ballUpButton = gamepad1.b;
            boolean catapult1ballUpButton = gamepad1.a;
            // If the down button is held, cancel all launch buttons to prevent conflicts.
            if (catapultDownButton) {
                catapult4ballUpButton = false;
                catapult3ballUpButton = false;
                catapult2ballUpButton = false;
                catapult1ballUpButton = false;
            }

// ============================================================
// DRIVETRAIN: Mecanum wheel mixing
// ============================================================
            // Each wheel gets a different mix of axial, lateral, and yaw to produce mecanum motion.
            leftFrontPower  = axial + lateral + yaw;
            rightFrontPower = axial - lateral - yaw;
            leftBackPower   = axial - lateral + yaw;
            rightBackPower  = axial + lateral - yaw;

            // If any wheel exceeds 1.0, scale all four down proportionally to preserve the ratio.
            // /= means divide this variable by the value on the right and store the result.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

// ============================================================
// INTAKE
// ============================================================
            if (intakeInButton) {
                intakePower = INTAKE_IN_POWER;
            } else if (intakeOutButton) {
                intakePower = INTAKE_OUT_POWER;
            } else {
                intakePower = INTAKE_OFF_POWER;
            }

// ============================================================
// CATAPULT: timed launch followed by automatic return
// ============================================================
            // || means OR — true if any one of these buttons is pressed.
            boolean upButtonPressed = catapult4ballUpButton || catapult3ballUpButton || catapult2ballUpButton || catapult1ballUpButton;
            // ! means NOT. newUpPress is only true on the first loop where a button is pressed (rising edge).
            boolean newUpPress = upButtonPressed && !previousUpButton;

            if (newUpPress && !catapultRunning) {
                catapultRunning = true;
                catapultTimer.reset(); // Restart the timer from zero when a shot begins.
                if      (catapult4ballUpButton) catapultActivePower = SHOOT_4_POWER;
                else if (catapult3ballUpButton) catapultActivePower = SHOOT_3_POWER;
                else if (catapult2ballUpButton) catapultActivePower = SHOOT_2_POWER;
                else if (catapult1ballUpButton) catapultActivePower = SHOOT_1_POWER;
                else                           catapultActivePower = CATAPULT_HOLD_POWER;
            }

            if (catapultRunning) {
                // catapultTimer.milliseconds() returns how long ago reset() was called.
                if (catapultTimer.milliseconds() < catapultRunDuration) {
                    pivotMode = CatapultModes.UP;
                    catapult1.setPower(catapultActivePower);
                    catapult2.setPower(catapultActivePower);
                } else if (catapultTimer.milliseconds() < catapultRunDuration + catapultReturnDuration) { // increase catapultReturnDuration if catapult doesn't pull down
                    pivotMode = CatapultModes.DOWN;
                    catapult1.setPower(CATAPULT_DOWN_POWER);
                    catapult2.setPower(CATAPULT_DOWN_POWER);
                } else {
                    catapultRunning = false; // Reset so a future press can launch again.
                    pivotMode = CatapultModes.HOLD;
                    catapult1.setPower(CATAPULT_HOLD_POWER);
                    catapult2.setPower(CATAPULT_HOLD_POWER);
                }
            } else if (catapultDownButton) { // Manual return only available when no sequence is active.
                pivotMode = CatapultModes.DOWN;
                catapult1.setPower(CATAPULT_DOWN_POWER);
                catapult2.setPower(CATAPULT_DOWN_POWER);
            } else {
                pivotMode = CatapultModes.HOLD;
                catapult1.setPower(CATAPULT_HOLD_POWER);
                catapult2.setPower(CATAPULT_HOLD_POWER);
            }

            String catapult_mode_str;
            if      (pivotMode == CatapultModes.UP)   catapult_mode_str = "UP";
            else if (pivotMode == CatapultModes.DOWN)  catapult_mode_str = "DOWN";
            else                                       catapult_mode_str = "HOLD";

// ============================================================
// SERVO CONTROL
// ============================================================
            if (USE_CR_SERVOS) {
                if (rampUpButton && !rampDownButton) {
                    leftFlowerRampCR.setPower(SERVO_POWER);
                    rightFlowerRampCR.setPower(SERVO_POWER);
                } else if (rampDownButton && !rampUpButton) {
                    leftFlowerRampCR.setPower(-SERVO_POWER);
                    rightFlowerRampCR.setPower(-SERVO_POWER);
                } else {
                    leftFlowerRampCR.setPower(0.0);
                    rightFlowerRampCR.setPower(0.0);
                }
            } else {
                if (rampDownButton) {
                    leftFlowerRamp.setPosition(angleToServo(LEFT_SERVO_START_DEG - SERVO_TRAVEL_DEG));
                    rightFlowerRamp.setPosition(angleToServo(RIGHT_SERVO_START_DEG + SERVO_TRAVEL_DEG));
                } else if (rampUpButton) {
                    leftFlowerRamp.setPosition(angleToServo(LEFT_SERVO_START_DEG));
                    rightFlowerRamp.setPosition(angleToServo(RIGHT_SERVO_START_DEG));
                } else if (rampMidButton) {
                    leftFlowerRamp.setPosition(angleToServo(LEFT_SERVO_START_DEG - SERVO_MID_DEG));
                    rightFlowerRamp.setPosition(angleToServo(RIGHT_SERVO_START_DEG + SERVO_MID_DEG));
                }
            }
// ============================================================

// ============================================================
// WRITE EFFECTORS
// ============================================================
            // All setPower() calls for drive and intake happen here. Catapult and servo outputs are written in their own sections above.
//            leftFrontDrive.setPower(leftFrontPower);
//            rightFrontDrive.setPower(rightFrontPower);
//            leftBackDrive.setPower(leftBackPower);
//            rightBackDrive.setPower(rightBackPower);
//            intake.setPower(intakePower);

// ============================================================
// TELEMETRY
// ============================================================
            // Save this loop's button state so next loop can detect a new press.
            previousUpButton = upButtonPressed;

            // telemetry sends labeled data to the Driver Station screen.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
//            telemetry.addData("Intake", "%4.2f", intake.getPower());
            telemetry.addData("Catapult MODE", "%s", catapult_mode_str);
            telemetry.addData("Shot power", "%4.2f", catapultActivePower);
            if (!USE_CR_SERVOS) {
                telemetry.addData("Right Servo Position", rightFlowerRamp.getPosition());
                telemetry.addData("Left Servo Position",  leftFlowerRamp.getPosition());
            }
            telemetry.update();
        }
    }
}
