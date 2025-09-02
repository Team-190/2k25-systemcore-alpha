// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shared.visionlimelight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

public class Vision extends SubsystemBase {
  @Getter private final Camera[] cameras;

  public Vision(Camera... cameras) {
    this.cameras = cameras;
  }

  @Override
  public void periodic() {
    for (Camera camera : cameras) {
      camera.periodic();
    }
  }
}
