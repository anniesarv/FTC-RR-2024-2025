package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeWrist;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeWrist;

public class OpenClaw extends CommandBase {

    //private final IntakeWrist intakeWrist;
      private final OuttakeWrist outtakeWrist;

    public OpenClaw(IntakeWrist intakeWrist, OuttakeWrist outtakeWrist) {
        //this.intakeWrist = intakeWrist;
         this.outtakeWrist = outtakeWrist;

        addRequirements(intakeWrist, outtakeWrist);
    }

    @Override
    public void execute() {
        //intakeWrist.startOuttake();
        outtakeWrist.openClaw();


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
