/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import java.util.List;

@Logged
public interface VisionIO {
  VisionEstimate[] getLatestEstimates();

  List<Camera> getCameras();

  default boolean areCamerasConnected() {
    boolean isConnected = false;

    for (Camera camera : getCameras()) {
      isConnected = camera.isConnected();
      if (!isConnected) break;
    }

    return isConnected;
  }
}
