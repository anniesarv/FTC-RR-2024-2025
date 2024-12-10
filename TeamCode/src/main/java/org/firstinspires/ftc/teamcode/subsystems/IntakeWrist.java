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
    private static final double WRIST_UP_POSITION = 1.0;
    private static final double WRIST_DOWN_POSITION = 0.0;
    private static final double INTAKE_POWER = -1.0;
    private static final double OUTTAKE_POWER = 1.0;

    public IntakeWrist(RobotHardware robotHardware) {
        this.horizontalWrist = robotHardware.horizontalWrist;
        this.intakeLeft = robotHardware.intakeLeft;
        this.intakeRight = robotHardware.intakeRight;

        horizontalWrist.setPosition(WRIST_UP_POSITION);

        intakeLeft.setPower(0);
        intakeRight.setPower(0);
    }

    public void lowerWrist() {
        horizontalWrist.setPosition(WRIST_DOWN_POSITION);
    }

    public void raiseWrist() {
        horizontalWrist.setPosition(WRIST_UP_POSITION);
    }

    public void startIntake() {
            intakeLeft.setPower(INTAKE_POWER);
            intakeRight.setPower(-INTAKE_POWER);

        System.out.println("Starting intake: Left = " + INTAKE_POWER + ", Right = " + (-INTAKE_POWER));

    }

    public void startOuttake() {
            intakeLeft.setPower(OUTTAKE_POWER);
            intakeRight.setPower(-OUTTAKE_POWER);

    }

    public void stopIntake() {
            intakeLeft.setPower(0);
            intakeRight.setPower(0);


    }
}
