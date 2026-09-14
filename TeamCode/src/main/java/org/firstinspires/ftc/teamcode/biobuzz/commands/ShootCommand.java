package org.firstinspires.ftc.teamcode.biobuzz.commands;

import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.NectarShooter;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.PollenShooter;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.lib.fsm.State;
import org.firstinspires.ftc.teamcode.lib.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.lib.robot.Command;

import java.util.Arrays;

public class ShootCommand extends Command {
    public ShootCommand(Transfer transfer, NectarShooter nectarShooter, PollenShooter pollenShooter) {
        super(Arrays.asList(transfer, nectarShooter, pollenShooter), new StateMachine(
                new State()
                        .onEnter(transfer::transferOn)
                        .maxTime(300),
                new State()
                        .onEnter(() -> {
                            pollenShooter.openLatch();
                            nectarShooter.openLatch();
                        })
                        .maxTime(1000),
                new State()
                        .onEnter(() -> {
                            pollenShooter.closeLatch();
                            nectarShooter.closeLatch();
                            transfer.transferOff();
                        })
                        .maxTime(300)
        ));
    }
}
