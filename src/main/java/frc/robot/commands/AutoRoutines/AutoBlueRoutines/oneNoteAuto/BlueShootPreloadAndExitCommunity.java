// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines.AutoBlueRoutines.oneNoteAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.photonVisionConstants;
import frc.robot.commands.AutoNoteBlocks.note1Block;
import frc.robot.commands.DriveCommands.DriveForwardSetDistance;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueShootPreloadAndExitCommunity extends SequentialCommandGroup {
  /** Creates a new BlueShootPreloadAndExitCommunity. */
  public BlueShootPreloadAndExitCommunity(DriveSubsystem drive, ShooterSubsystem shooter, ConveyerSubsystem conveyer, LightSubsystem led, PhotonSubsystem photon) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new note1Block(drive, shooter, conveyer, led, photon, photonVisionConstants.speakerMiddleBlueID), 
      new DriveForwardSetDistance(drive, 100, DriveConstants.autoSpeed)
    );
  }
}
