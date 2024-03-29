// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class setShooterVelocityCommand extends Command {
  
  private ShooterSubsystem SHOOTER_SUBSYSTEM; 

  private double topShooterSpeed; 
  private double bottomShooterSpeed; 

  private SlewRateLimiter topLimiter = new SlewRateLimiter(ShooterConstants.topSlewLimit); 
  private SlewRateLimiter bottomLimiter = new SlewRateLimiter(ShooterConstants.bottomSlewLimit); 

  /** Creates a new setShooterVelocityCommand. */
  public setShooterVelocityCommand(ShooterSubsystem shooter, double topMotorSpeed, double bottomMotorSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.SHOOTER_SUBSYSTEM = shooter; 
    this.topShooterSpeed = topMotorSpeed; 
    this.bottomShooterSpeed = bottomMotorSpeed; 
    addRequirements(SHOOTER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SHOOTER_SUBSYSTEM.setShooterVelocityMode();
    SHOOTER_SUBSYSTEM.setRampRate(ShooterConstants.shooterRampRate);
    SHOOTER_SUBSYSTEM.setTopPIDF(ShooterConstants.topShooterKp, ShooterConstants.topShooterKi, ShooterConstants.topShooterKd, ShooterConstants.topShooterKFf);
    SHOOTER_SUBSYSTEM.setBottomPIDF(ShooterConstants.bottomShooterKp, ShooterConstants.bottomShooterKi, ShooterConstants.bottomShooterKd, ShooterConstants.bottomShooterKFf);

    SHOOTER_SUBSYSTEM.setTopEncoderOutputConstraints(ShooterConstants.topMin, ShooterConstants.topMax);
    SHOOTER_SUBSYSTEM.setBottomEncoderOutputConstraints(ShooterConstants.bottomMin, ShooterConstants.bottomMax);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SHOOTER_SUBSYSTEM.setVelocityTop((topShooterSpeed*3.0));
    SHOOTER_SUBSYSTEM.setVelocityBottom((bottomShooterSpeed*3.0)); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SHOOTER_SUBSYSTEM.setCoastMode();
    SHOOTER_SUBSYSTEM.setShooterPowerMode();
    SHOOTER_SUBSYSTEM.setShooter(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
