// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class setShooterSpeedCommand extends Command {
  /** Creates a new setShooterSpeedCommand. */


  private ShooterSubsystem SHOOTER_SUBSYSTEM; 

  private double topSpeed;
  private double bottomSpeed;  


  public setShooterSpeedCommand(ShooterSubsystem shooter, double topSpeed, double bottomSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.SHOOTER_SUBSYSTEM = shooter; 
    this.topSpeed = topSpeed;  
    this.bottomSpeed = bottomSpeed;  
    addRequirements(SHOOTER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {   
    SHOOTER_SUBSYSTEM.setShooter(topSpeed, bottomSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SHOOTER_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
