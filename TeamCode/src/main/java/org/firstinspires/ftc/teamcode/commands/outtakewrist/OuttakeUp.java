package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeWrist;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeWrist;

public class OuttakeUp extends CommandBase {

    //private final IntakeWrist intakeWrist;
    private final OuttakeWrist outtakeWrist;

    public OuttakeUp(OuttakeWrist outtakeWrist) {
        this.outtakeWrist = outtakeWrist;

        addRequirements(outtakeWrist);
    }

    @Override
    public void execute() {
        //intakeWrist.startOuttake();
        outtakeWrist.raiseWrist();


    }

    @Override
    public void end(boolean interrupted) {
        // stop mechanisms when the command ends ?
        //intakeWrist.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false; // run continuously during teleop
    }
}
