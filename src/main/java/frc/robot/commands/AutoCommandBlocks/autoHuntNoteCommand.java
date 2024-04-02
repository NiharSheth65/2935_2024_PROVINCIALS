// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommandBlocks;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.VisionCommands.VisionDriveAndAlignCommand;
import frc.robot.commands.VisionCommands.VisionDriveToTargetCommand;
import frc.robot.commands.VisionCommands.VisionTurnToTargetCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoHuntNoteCommand extends SequentialCommandGroup {
  /** Creates a new autoHuntNoteCommand. */
  public autoHuntNoteCommand(DriveSubsystem drive, VisionSubsystem vision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SequentialCommandGroup(
        new VisionTurnToTargetCommand(drive, vision, VisionConstants.ambientPipeline, false), 
        new VisionDriveToTargetCommand(drive, vision, VisionConstants.ambientPipeline, false), 
        new VisionTurnToTargetCommand(drive, vision, VisionConstants.ambientPipeline, false)

        // new VisionDriveAndAlignCommand(drive, vision, VisionConstants.ambientPipeline, false)
      )
    );
  }
}
