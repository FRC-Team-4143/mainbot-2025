package frc.mw_lib.util;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

public class ConstantsLoader extends JSONReader {
  private static ConstantsLoader instance_;

  public static ConstantsLoader getInstance() {
    if (instance_ == null) {
      instance_ = new ConstantsLoader();
    }

    return instance_;
  }

  private static final String robot_name_pref_name = "RobotName";

  private ConstantsLoader() {
    String robot_name = "AlphaBot";
    if (!MWPreferences.getInstance().hasPreference(robot_name_pref_name)) {
      DriverStation.reportError(
          "Failed to retrieve robot name on startup, using default: " + robot_name, false);
    } else {
      robot_name =
          MWPreferences.getInstance().getPreferenceString(robot_name_pref_name, robot_name);
    }

    Path json_path =
        Filesystem.getDeployDirectory().toPath().resolve("robots/" + robot_name + ".json");
    try {
      loadJson(json_path);
    } catch (IOException e) {
      DriverStation.reportError(
          "Failed to load robot constants for " + robot_name, e.getStackTrace());
    }

    SmartDashboard.putString("Config/RobotName", robot_name);
    // Burn Robot Name Command
    SmartDashboard.putData(
        "Config/Burn RobotName",
        Commands.runOnce(() -> burnRobotName()).onlyIf(RobotState::isTest).ignoringDisable(true));
  }

  public void burnRobotName() {
    String robot_name = SmartDashboard.getString("Config/RobotName", "");
    if (!robot_name.isBlank()) {
      MWPreferences.getInstance().setPreference(robot_name_pref_name, robot_name);
      DataLogManager.log("Updated RobotName to " + robot_name + " - Restart Robot Code!!!!");
    } else {
      DataLogManager.log("Cannot Configure Robot with Blank Name");
    }
  }

  public double getDoubleValue(String... path_steps) {
    JsonNode current = walkTree(root_node_, path_steps);
    return current.asDouble();
  }

  public int getIntValue(String... path_steps) {
    JsonNode current = walkTree(root_node_, path_steps);
    return current.asInt();
  }

  public boolean getBoolValue(String... path_steps) {
    JsonNode current = walkTree(root_node_, path_steps);
    return current.asBoolean();
  }

  public String getStringValue(String... path_steps) {
    JsonNode current = walkTree(root_node_, path_steps);
    return current.asText();
  }

  public List<String> getStringList(String... path_steps) {
    JsonNode current = walkTree(root_node_, path_steps);

    List<String> elements = new ArrayList<>();
    if (current.isArray()) {
      ArrayNode array_node = (ArrayNode) current;
      array_node
          .elements()
          .forEachRemaining(
              (JsonNode node) -> {
                elements.add(node.asText());
              });
    }

    return elements;
  }

  protected List<String> getChildList(String... path_steps) {
    JsonNode current = walkTree(root_node_, path_steps);
    return getChildren(current);
  }

  public List<CamConstants> getCameras(String... path_steps) {
    ArrayList<CamConstants> cameras = new ArrayList<>();

    JsonNode cameras_root = walkTree(root_node_, path_steps);

    List<String> names = getChildList(path_steps);
    for (String name : names) {
      JsonNode camera_root = walkTree(cameras_root, name);

      CamConstants config = new CamConstants();
      config.camera_name = walkTree(camera_root, "NAME").asText();
      config.camera_transform =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(walkTree(camera_root, "location", "X").asDouble()),
                  Units.inchesToMeters(walkTree(camera_root, "location", "Y").asDouble()),
                  Units.inchesToMeters(walkTree(camera_root, "location", "Z").asDouble())),
              new Rotation3d(
                  Units.degreesToRadians(walkTree(camera_root, "location", "ROLL").asDouble()),
                  Units.degreesToRadians(walkTree(camera_root, "location", "PITCH").asDouble()),
                  Units.degreesToRadians(walkTree(camera_root, "location", "YAW").asDouble())));

      cameras.add(config);
    }

    return cameras;
  }
}
