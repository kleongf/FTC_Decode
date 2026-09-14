package org.firstinspires.ftc.teamcode.biobuzz.commands;

import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake;
import org.firstinspires.ftc.teamcode.lib.fsm.State;
import org.firstinspires.ftc.teamcode.lib.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.lib.robot.Command;

import java.util.Collections;

public class IntakeCommand extends Command {
    public IntakeCommand(Intake intake) {
        super(Collections.singletonList(intake), new StateMachine(
                new State()
                        .onEnter(intake::intakeOn)
                        .maxTime(100)
        ));
    }
}
