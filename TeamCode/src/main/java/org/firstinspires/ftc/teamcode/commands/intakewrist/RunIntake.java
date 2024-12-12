package org.firstinspires.ftc.teamcode.commands.intakewrist;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeWrist;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeWrist;

import java.util.function.DoubleSupplier;

public class RunIntake extends CommandBase {

    private final IntakeWrist intakeWrist;

    // private final OuttakeWrist outtakeWrist;

    public RunIntake(IntakeWrist intakeWrist) {
        this.intakeWrist = intakeWrist;
       // this.outtakeWrist = outtakeWrist;


        addRequirements(intakeWrist);
    }


    @Override
    public void initialize() {
        intakeWrist.startIntake();

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        // stop mechanisms when the command ends ?
       // intakeWrist.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false; // run continuously during teleop
    }
}
