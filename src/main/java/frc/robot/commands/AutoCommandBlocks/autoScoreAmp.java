// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommandBlocks;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DriveCommands.DriveForwardSetDistance;
import frc.robot.commands.PhotonCommands.photonAlignToAnyTag;
import frc.robot.commands.PhotonCommands.photonAlignToTrap;
import frc.robot.commands.PhotonCommands.photonDriveToAnyTag;
import frc.robot.commands.PhotonCommands.photonDriveToTrap;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoScoreAmp extends SequentialCommandGroup {
  /** Creates a new autoScoreAmp. */
  public autoScoreAmp(DriveSubsystem drive, ConveyerSubsystem conveyer, ShooterSubsystem shooter, IntakeSubsystem intake, PhotonSubsystem photon) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new ParallelCommandGroup(
        new autoDigestNoteCommand(conveyer, intake, shooter), 
        
        new SequentialCommandGroup(
          new photonAlignToAnyTag(photon, drive, 0, false), 
          new photonDriveToAnyTag(photon, drive, 18, false), 
          new photonAlignToAnyTag(photon, drive, 0, false)

          // new photonAlignToTrap(photon, drive, 0, false), 
          // new photonDriveToTrap(photon, drive, 18, false), 
          // new photonAlignToTrap(photon, drive, 0, false)
        ) 
      ), 

      new ParallelDeadlineGroup(

        new DriveForwardSetDistance(drive, -20, 0.25),
        new autoRevUpCommand(shooter, ShooterConstants.ampTopMotorSpeed, ShooterConstants.ampBottomMotorSpeed)
      ), 

      new autoShootNoteCommand(drive, photon, conveyer, 0)
    );
  }
}
