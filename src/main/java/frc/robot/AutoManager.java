package frc.robot;

public class AutoManager {
  // Singleton pattern
  private static AutoManager autoManagerInstance = null;

  public static AutoManager getInstance() {
    if (autoManagerInstance == null) {
      autoManagerInstance = new AutoManager();
    }
    return autoManagerInstance;
  }

  private AutoManager() {}
}
