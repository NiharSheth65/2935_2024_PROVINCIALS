// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommandBlocks;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.conveyerConstants;
import frc.robot.commands.ConveyerCommands.conveyAwaySensorThreeCommand;
import frc.robot.commands.ConveyerCommands.conveyForTimeCommand;
import frc.robot.commands.ConveyerCommands.conveyerTillSensorThreeCommand;
import frc.robot.commands.IntakeCommand.IntakePowerCommand;
import frc.robot.commands.ShooterCommands.setShooterVelocityCommand;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoDigestNoteCommand extends SequentialCommandGroup {
  /** Creates a new autoDigestNoteCommand. */
  public autoDigestNoteCommand(ConveyerSubsystem conveyer, IntakeSubsystem intake, ShooterSubsystem shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(


      new ParallelDeadlineGroup(
        new conveyerTillSensorThreeCommand(conveyer), 
        new IntakePowerCommand(intake, IntakeConstants.intakeSpeed)
      ), 

      new ParallelDeadlineGroup(
        new conveyForTimeCommand(conveyer, conveyerConstants.conveyerInSpeed, 450), 
        new setShooterVelocityCommand(shooter, 0, ShooterConstants.shooterHoldInSpeed)
      ), 


      new conveyAwaySensorThreeCommand(conveyer)


    );
  }
}
