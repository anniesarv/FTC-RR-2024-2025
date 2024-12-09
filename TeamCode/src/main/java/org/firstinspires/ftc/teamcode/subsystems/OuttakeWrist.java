package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class OuttakeWrist extends SubsystemBase {

    private final Servo verticalWrist;
    private final Servo microServo;

    // Constants for vertical wrist positions
    private static final double WRIST_UP_POSITION = 1.0; // Adjust based on your servo configuration
    private static final double WRIST_DOWN_POSITION = 0.0; // Adjust based on your servo configuration

    // Constants for outtake servo positions
    private static final double OUTTAKE_OPEN_POSITION = 1.0; // Open position for outtake
    private static final double OUTTAKE_CLOSED_POSITION = 0.0; // Closed position

    public OuttakeWrist(RobotHardware robotHardware) {
        this.verticalWrist = robotHardware.verticalWrist;
        this.microServo = robotHardware.microServo;

        // Initialize the vertical wrist and outtake servo to default positions
        verticalWrist.setPosition(WRIST_UP_POSITION);
        microServo.setPosition(OUTTAKE_CLOSED_POSITION);
    }

    // Move the wrist down
    public void lowerWrist() {
        verticalWrist.setPosition(WRIST_DOWN_POSITION);
    }

    // Move the wrist up
    public void raiseWrist() {
        verticalWrist.setPosition(WRIST_UP_POSITION);
    }

    // Open the outtake to release an object
    public void openOuttake() {
        microServo.setPosition(OUTTAKE_OPEN_POSITION);
    }

    // Close the outtake
    public void closeOuttake() {
        microServo.setPosition(OUTTAKE_CLOSED_POSITION);
    }
}
