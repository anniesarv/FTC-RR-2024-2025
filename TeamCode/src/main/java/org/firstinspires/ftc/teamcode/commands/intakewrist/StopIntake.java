package org.firstinspires.ftc.teamcode.commands.intakewrist;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeWrist;

public class StopIntake extends CommandBase {

    private final IntakeWrist intakeWrist;

    public StopIntake(IntakeWrist intakeWrist) {
        this.intakeWrist = intakeWrist;

        addRequirements(intakeWrist);
    }


    @Override
    public void initialize() {
        intakeWrist.stopIntake();

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        // not sure what to put into here

    }

    @Override
    public boolean isFinished() {
        return false; // run continuously during teleop
    }
}
