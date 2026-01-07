/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
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
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * StarterBotTeleopWithToggleBrake (Logitech F310 friendly)
 *
 * - Preserves original teleop functionality (arcade drive, launcher velocity control,
 *   feeder servos and launch state machine).
 * - Adds a toggleable, robust "hold position" brake bound to L1 (left bumper on Logitech F310).
 *     * Press L1 once to toggle Brake Mode ON, press again to toggle OFF.
 * - No rumble or other confirmation is performed when toggling (per request).
 *
 * Brake implementation details:
 * - When enabled we:
 *     * save the drive motors' current run mode,
 *     * record the current encoder positions,
 *     * set the motors to RUN_TO_POSITION with their current position as the target,
 *     * set motor power to 1.0 so the controller actively holds position,
 *     * ensure zeroPowerBehavior is BRAKE and repeatedly force power to the motors while
 *       brake mode is active.
 * - When disabled we:
 *     * restore the motors' previous run modes and allow normal arcade driving to resume.
 *
 * Note: This actively holds position; extreme physical forces may still move the robot if
 * the motors/gearbox/current limit cannot supply the torque required.
 */

@TeleOp(name = "StarterBotTeleop (Toggle Brake L1 - F310)", group = "StarterBot")
//@Disabled
public class StarterBotTeleop extends OpMode {
    final double FEED_TIME_SECONDS = 0.5; // The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; // Servo stop
    final double FULL_SPEED = 0.75;

    final double LAUNCHER_TARGET_VELOCITY = 1350;
    final double LAUNCHER_MIN_VELOCITY = 1300;

    // Drive & mechanism members
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    // Timers & state
    ElapsedTime feederTimer = new ElapsedTime();

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchState launchState;

    // Drive telemetry
    double leftPower;
    double rightPower;

    // Brake toggle state
    private boolean brakeActive = false;
    private DcMotor.RunMode leftPrevMode = null;
    private DcMotor.RunMode rightPrevMode = null;

    // Manual rising-edge detection state (works for Logitech F310 mapping)
    private boolean prevLeftBumper = false;   // for L1 toggle
    private boolean prevRightBumper = false;  // for right bumper launch

    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        // Hardware map
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        // Motor directions
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Launcher encoder mode & PIDF
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        launcher.setZeroPowerBehavior(BRAKE);

        // Drive brake behavior by default (safe)
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);

        // Initialize feeders
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() { }

    @Override
    public void start() { }

    @Override
    public void loop() {
        // Read current bumper states from the Gamepad (Logitech F310 left bumper => gamepad1.left_bumper)
        boolean curLeftBumper = gamepad2.left_bumper;
        boolean curRightBumper = gamepad2.right_bumper;

        // L1 rising-edge => toggle brake (no rumble or confirmation)
        if (curLeftBumper && !prevLeftBumper) {
            toggleBrakeMode();
        }
        prevLeftBumper = curLeftBumper;

        // If brake active, enforce hold; otherwise allow normal driving
        if (brakeActive) {
            enforceHoldPosition();
        } else {
            // Driving using Logitech F310 joystick axes
            arcadeDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x);
        }

        // Launcher manual control
        if (gamepad2.y) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad2.b) {
            launcher.setVelocity(STOP_SPEED);
        }

        // Right bumper rising-edge => request a shot (preserves original behaviour)
        boolean shotRequested = (curRightBumper && !prevRightBumper);
        prevRightBumper = curRightBumper;
        launch(shotRequested);

        // Telemetry (kept minimal)
        telemetry.addData("LaunchState", launchState);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Launcher speed", launcher.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    // --- Brake implementation ------------------------------------------------

    private void toggleBrakeMode() {
        if (!brakeActive) {
            enableBrakeMode();
        } else {
            disableBrakeMode();
        }
    }

    private void enableBrakeMode() {
        // Save previous modes
        leftPrevMode = leftDrive.getMode();
        rightPrevMode = rightDrive.getMode();

        // Capture current encoder positions
        int leftPos = leftDrive.getCurrentPosition();
        int rightPos = rightDrive.getCurrentPosition();

        // Set targets to current positions
        leftDrive.setTargetPosition(leftPos);
        rightDrive.setTargetPosition(rightPos);

        // Use RUN_TO_POSITION to actively hold position
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Ensure strong braking behavior
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);

        // Command full power so controller actively resists external forces
        leftDrive.setPower(1.0);
        rightDrive.setPower(1.0);

        brakeActive = true;
    }

    private void disableBrakeMode() {
        // Stop power before changing modes
        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);

        // Restore previous run modes (or default to RUN_USING_ENCODER)
        if (leftPrevMode != null) {
            leftDrive.setMode(leftPrevMode);
        } else {
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (rightPrevMode != null) {
            rightDrive.setMode(rightPrevMode);
        } else {
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Keep zeroPowerBehavior as BRAKE (safe default)
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);

        brakeActive = false;
    }

    private void enforceHoldPosition() {
        // Re-apply target positions and full power every loop to ensure hold
        int leftTarget = leftDrive.getTargetPosition();
        int rightTarget = rightDrive.getTargetPosition();

        leftDrive.setTargetPosition(leftTarget);
        rightDrive.setTargetPosition(rightTarget);

        // Ensure run mode is RUN_TO_POSITION
        if (leftDrive.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (rightDrive.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        leftDrive.setPower(1.0);
        rightDrive.setPower(1.0);
    }

    // --- Driving & launching -----------------------------------------------

    void arcadeDrive(double forward, double rotate) {
        leftPower = forward + rotate;
        rightPower = forward - rotate;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }
}