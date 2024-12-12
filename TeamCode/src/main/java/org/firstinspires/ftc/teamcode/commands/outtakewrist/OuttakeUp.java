package org.firstinspires.ftc.teamcode.commands.outtakewrist;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeWrist;

public class OuttakeUp extends CommandBase {

    //private final IntakeWrist intakeWrist;
    private final OuttakeWrist outtakeWrist;

    public OuttakeUp(OuttakeWrist outtakeWrist) {
        this.outtakeWrist = outtakeWrist;

        addRequirements(outtakeWrist);
    }


    @Override
    public void initialize() {
        outtakeWrist.raiseWrist();
    }
    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false; // run continuously during teleop
    }
}
