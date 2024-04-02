// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoNoteBlocks;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AutoCommandBlocks.autoRevUpCommand;
import frc.robot.commands.AutoCommandBlocks.autoShootNoteCommand;
import frc.robot.commands.ConveyerCommands.conveyerTillSensorThreeCommand;
import frc.robot.commands.LedCommands.ledCommand;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class note1Block extends SequentialCommandGroup {
  /** Creates a new note1Block. */
  public note1Block(DriveSubsystem drive, ShooterSubsystem shooter, ConveyerSubsystem conveyer, LightSubsystem led, PhotonSubsystem photon, int targetId) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(

        new SequentialCommandGroup(
          new conveyerTillSensorThreeCommand(conveyer), 
          new autoShootNoteCommand(drive, photon, conveyer, targetId)
        ), 

        new autoRevUpCommand(shooter, ShooterConstants.speakerTopMotorSpeed, ShooterConstants.speakerBottomMotorSpeed), 
        new ledCommand(led)
      ) 
    );
  }
}
