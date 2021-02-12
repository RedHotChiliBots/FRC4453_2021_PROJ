// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Chassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonDriveBouncePath extends SequentialCommandGroup {
  /** Creates a new AutonDriveBouncePath. */
  public Chassis chassis;

  public AutonDriveBouncePath(Chassis chassis) {
    this.chassis = chassis;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ResetFieldPosition(this.chassis),
    new AutonDriveTrajectory(chassis.bouncePath1, this.chassis)
    /*new AutonDriveTrajectory(chassis.bouncePath2, this.chassis),
    new AutonDriveTrajectory(chassis.bouncePath3, this.chassis),
    new AutonDriveTrajectory(chassis.bouncePath4, this.chassis)*/);
  }
}
