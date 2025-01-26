package frc.mw_lib.util;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

public class ConstantsLoader {
  private static ConstantsLoader instance_;

  public static ConstantsLoader getInstance() {
    if (instance_ == null) {
      instance_ = new ConstantsLoader();
    }

    return instance_;
  }

  private JsonNode root_node_;

  private ConstantsLoader() {
    String robot_name = Preferences.getString("RobotName", "AlphaBot");
    String json_name = robot_name + ".json";
    Path json_path = Filesystem.getDeployDirectory().toPath().resolve("robots/" + json_name);

    ObjectMapper obj = new ObjectMapper();

    try {
      String json_data = new String(Files.readAllBytes(json_path));
      root_node_ = obj.readTree(json_data);
    } catch (IOException e) {

    } catch (OutOfMemoryError e) {

    } catch (SecurityException e) {

    }

    SmartDashboard.putString("Config/RobotName", robot_name);
    // Burn Robot Name Command
    SmartDashboard.putData(
        "Config/Burn RobotName",
        Commands.runOnce(() -> burnRobotName()).onlyIf(RobotState::isTest).ignoringDisable(true));
  }

  private JsonNode walkTree(String... path_steps) {
    JsonNode current = root_node_;
    for (String path_segment : path_steps) {
      current = current.path(path_segment);
    }
    return current;
  }

  public double getDoubleValue(String... path_steps) {
    JsonNode current = walkTree(path_steps);
    return current.asDouble();
  }

  public int getIntValue(String... path_steps) {
    JsonNode current = walkTree(path_steps);
    return current.asInt();
  }

  public boolean getBoolValue(String... path_steps) {
    JsonNode current = walkTree(path_steps);
    return current.asBoolean();
  }

  public void burnRobotName() {
    String robot_name = SmartDashboard.getString("Config/RobotName", "");
    if (!robot_name.isBlank()) {
      Preferences.setString("RobotName", robot_name);
      DataLogManager.log("Updated RobotName to " + robot_name);
    } else {
      System.out.println("Cannot Configure Robot with Blank Name");
    }
  }
}
