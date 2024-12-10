package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalArm;

public class MoveHorizontalArmToPosition extends CommandBase {

    private final HorizontalArm horizontalArm;
    private final int targetPosition;

    public MoveHorizontalArmToPosition(HorizontalArm horizontalArm, int targetPosition) {
        this.horizontalArm = horizontalArm;
        this.targetPosition = targetPosition;

        addRequirements(horizontalArm);
    }

    @Override
    public void initialize() {
        // when the command starts, move the arm to the target position
        horizontalArm.moveToPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(horizontalArm.motorHA.getCurrentPosition() - targetPosition) < 10; // 10 is a tolerance value
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the arm when the command ends
        horizontalArm.stopArm();
    }
}
