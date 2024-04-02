// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommandBlocks;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShooterCommands.setShooterVelocityCommand;
import frc.robot.commands.ShooterCommands.shooterHasReachedVelocity;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoRevUpCommand extends SequentialCommandGroup {
  /** Creates a new autoRevUpCommand. */
  public autoRevUpCommand(ShooterSubsystem shooter, double topMotorSpeed, double bottomMotorSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new setShooterVelocityCommand(shooter, topMotorSpeed, bottomMotorSpeed), 
        new shooterHasReachedVelocity(shooter, topMotorSpeed, topMotorSpeed, false)
      )
    );
  }
}
