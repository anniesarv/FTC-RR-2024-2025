package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeWrist;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeWrist;

public class RunIntake extends CommandBase {

    private final IntakeWrist intakeWrist;
   // private final OuttakeWrist outtakeWrist;

    public RunIntake(IntakeWrist intakeWrist, OuttakeWrist outtakeWrist) {
        this.intakeWrist = intakeWrist;
       // this.outtakeWrist = outtakeWrist;

        addRequirements(intakeWrist, outtakeWrist);
    }

    @Override
    public void execute() {
        intakeWrist.startIntake();

        // continuously run intake while a is held
        /*
        if (gamepad.a) {
            intakeWrist.startIntake();
        } else if (gamepad.b) {
            intakeWrist.startOuttake();
        } else {
            // stop the intake only when neither a nor a is pressed
            intakeWrist.stopIntake();
        }

        // Outtake wrist control
        if (gamepad.x) {
            outtakeWrist.closeOuttake();
        } else if (gamepad.y) {
            outtakeWrist.openOuttake();
        }

        // wrist position control
        if (gamepad.dpad_up) {
            intakeWrist.raiseWrist();
        } else if (gamepad.dpad_down) {
            intakeWrist.lowerWrist();
        }

        if (gamepad.dpad_left) {
            outtakeWrist.raiseWrist();
        } else if (gamepad.dpad_right) {
           outtakeWrist.lowerWrist();
        }

         */
    }

    @Override
    public void end(boolean interrupted) {
        // stop mechanisms when the command ends ?
        intakeWrist.stopIntake();
        //outtakeWrist.closeOuttake();
    }

    @Override
    public boolean isFinished() {
        return false; // run continuously during teleop
    }
}
