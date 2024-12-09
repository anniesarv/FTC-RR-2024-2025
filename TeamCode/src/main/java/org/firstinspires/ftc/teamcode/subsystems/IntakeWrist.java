package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class IntakeWrist extends SubsystemBase {

    private final Servo horizontalWrist;
    private final CRServo intakeLeft;
    private final CRServo intakeRight;

    // constants for wrist positions
    private static final double WRIST_UP_POSITION = 0.0; // Adjust based on your servo configuration
    private static final double WRIST_DOWN_POSITION = 1.0; // Adjust based on your servo configuration

    // power constants for intake/outtake servos
    private static final double INTAKE_POWER = 1.0; // Full power for intake
    private static final double OUTTAKE_POWER = -1.0; // Reverse power for outtake

    public IntakeWrist(RobotHardware robotHardware) {
        this.horizontalWrist = robotHardware.horizontalWrist;
        this.intakeLeft = robotHardware.intakeLeft;
        this.intakeRight = robotHardware.intakeRight;

        // Initialize wrist servo to the up position
        horizontalWrist.setPosition(WRIST_UP_POSITION);

        // Stop intake CR servos initially
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
    }

    // Lower the wrist
    public void lowerWrist() {
        horizontalWrist.setPosition(WRIST_DOWN_POSITION);
    }

    // Raise the wrist
    public void raiseWrist() {
        horizontalWrist.setPosition(WRIST_UP_POSITION);
    }

    // Start the intake
    public void startIntake() {
        intakeLeft.setPower(INTAKE_POWER);
        intakeRight.setPower(INTAKE_POWER);
    }

    // Reverse the intake to outtake
    public void startOuttake() {
        intakeLeft.setPower(OUTTAKE_POWER);
        intakeRight.setPower(OUTTAKE_POWER);
    }

    // Stop the intake
    public void stopIntake() {
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
    }
}
