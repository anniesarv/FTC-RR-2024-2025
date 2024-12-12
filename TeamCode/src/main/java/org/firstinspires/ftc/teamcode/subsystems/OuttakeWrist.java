package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class OuttakeWrist extends SubsystemBase {

    private final Servo verticalWrist;
    private final Servo microServo;

    private static final double WRIST_UP_POSITION = 1.0; // Adjust based on your servo configuration
    private static final double WRIST_DOWN_POSITION = 0.0; // Adjust based on your servo configuration

    private static final double OUTTAKE_OPEN_POSITION = 1.0; // Open position for outtake
    private static final double OUTTAKE_CLOSED_POSITION = 0.0; // Closed position

    public OuttakeWrist(RobotHardware robotHardware) {
        this.verticalWrist = robotHardware.verticalWrist;
        this.microServo = robotHardware.microServo;

        verticalWrist.setPosition(WRIST_UP_POSITION);
        microServo.setPosition(OUTTAKE_CLOSED_POSITION);
    }

    public void lowerWrist() {
        verticalWrist.setPosition(WRIST_DOWN_POSITION);
    }

    public void raiseWrist() {
        verticalWrist.setPosition(WRIST_UP_POSITION);
    }

    public void openClaw() {
        microServo.setPosition(OUTTAKE_OPEN_POSITION);
    }

    public void closeClaw() {
        microServo.setPosition(OUTTAKE_CLOSED_POSITION);
    }
}
