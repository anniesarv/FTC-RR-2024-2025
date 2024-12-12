package org.firstinspires.ftc.teamcode.commands.outtakewrist;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeWrist;

public class OpenClaw extends CommandBase {

    //private final IntakeWrist intakeWrist;
      private final OuttakeWrist outtakeWrist;

    public OpenClaw(OuttakeWrist outtakeWrist) {
         this.outtakeWrist = outtakeWrist;

        addRequirements(outtakeWrist);
    }



    @Override
    public void initialize() {
        outtakeWrist.openClaw();

    }

    @Override
    public void execute() {

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
