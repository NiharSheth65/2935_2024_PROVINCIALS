// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ConveyerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyerSubsystem;

public class conveyCommand extends Command {

  private  ConveyerSubsystem CONVEYER_SUBSYSTEM; 
  private double conveyerSpeed;

  /** Creates a new conveyCommand. */
  public conveyCommand(ConveyerSubsystem conveyer, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.conveyerSpeed = speed; 
    this.CONVEYER_SUBSYSTEM = conveyer; 
    addRequirements(CONVEYER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CONVEYER_SUBSYSTEM.setConveyer(conveyerSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
