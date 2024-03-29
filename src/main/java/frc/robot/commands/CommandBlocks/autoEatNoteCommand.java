// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandBlocks;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.ConveyerCommands.conveyerTillSensorTwoCommand;
import frc.robot.commands.DriveCommands.DriveForwardSetDistance;
import frc.robot.commands.IntakeCommand.IntakePowerCommand;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoEatNoteCommand extends SequentialCommandGroup {
  /** Creates a new autoEatCommand. */
  public autoEatNoteCommand(DriveSubsystem drive, ConveyerSubsystem conveyer, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new conveyerTillSensorTwoCommand(conveyer), 
        new DriveForwardSetDistance(drive, DriveConstants.driveForwardToIntake, DriveConstants.autoSpeed),
        new IntakePowerCommand(intake, IntakeConstants.intakeSpeed)
      )
    );
  }
}
