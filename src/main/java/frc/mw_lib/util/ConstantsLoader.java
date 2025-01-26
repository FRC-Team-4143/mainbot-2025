package frc.mw_lib.util;

import com.fasterxml.jackson.core.JsonLocation;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Iterator;

public class ConstantsLoader {
  private static ConstantsLoader instance_;

  public static ConstantsLoader getInstance() {
    if (instance_ == null) {
      instance_ = new ConstantsLoader();
    }

    return instance_;
  }

  private JsonNode root_node_;

  protected class ConstantParseError extends JsonProcessingException {

    public ConstantParseError(String msg, JsonLocation loc) {
      super(msg, loc, null);
    }

    public ConstantParseError(String msg) {
      super(msg, null, null);
    }
  }

  private ConstantsLoader() {
    // Can't use preferences directly as the content hasn't loaded by the time this
    // fires
    // String robot_name = Preferences.getString("RobotName", "AlphaBot");

    // So instead we load the content manually
    String robot_name = "AlphaBot";
    loadJson(Filesystem.getOperatingDirectory().toPath().resolve("networktables.json"));
    try {
      JsonNode robot_name_node = searchArrayForPair(root_node_, "name", "/Preferences/RobotName");
      robot_name = robot_name_node.get("value").asText();
    } catch (ConstantParseError e) {
      DriverStation.reportError(
          "Failed to retrieve robot name on startup, using default: " + robot_name, null);
    }

    String json_name = robot_name + ".json";
    Path json_path = Filesystem.getDeployDirectory().toPath().resolve("robots/" + json_name);
    loadJson(json_path);

    SmartDashboard.putString("Config/RobotName", robot_name);
    // Burn Robot Name Command
    SmartDashboard.putData(
        "Config/Burn RobotName",
        Commands.runOnce(() -> burnRobotName()).onlyIf(RobotState::isTest).ignoringDisable(true));
  }

  /**
   * Loads JSON content from a specific file into a JSON tree for parsing
   *
   * @param json_file
   */
  private void loadJson(Path json_file) {
    try {
      String json_data = new String(Files.readAllBytes(json_file));
      ObjectMapper obj = new ObjectMapper();
      root_node_ = obj.readTree(json_data);
    } catch (OutOfMemoryError e) {

    } catch (SecurityException e) {

    } catch (IOException e) {

    }
  }

  /**
   * Searches through a JSON array to find a specific key value pair
   *
   * @param starting the starting json node to search from
   * @param child_key the primary key to look for
   * @param child_value the value of the primary key
   * @return the node that contains child_key with the value of child_value
   * @throws ConstantParseError when node is not an array or child_key and value cannot be found
   */
  private JsonNode searchArrayForPair(JsonNode starting, String child_key, String child_value)
      throws ConstantParseError {
    if (!starting.isArray()) {
      throw new ConstantParseError("Attempted to search object as array");
    }

    // Iterate through the array to find the match
    ArrayNode arr_node = (ArrayNode) starting;
    Iterator<JsonNode> node = arr_node.elements();
    while (node.hasNext()) {
      JsonNode current = node.next();
      String key_text = current.get(child_key).asText();
      if (key_text.equals(child_value)) {
        return current;
      }
    }

    // At this point we failed to find so throw an exception
    throw new ConstantParseError("Failed to find suitable child");
  }

  /**
   * Walks the JSON tree down to a specific node
   *
   * @param path_steps a list of path steps to take along the way
   * @return the resulting node at the end of walking
   */
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
      DataLogManager.log("Updated RobotName to " + robot_name + " - Restart Robot Code!!!!");
    } else {
      System.out.println("Cannot Configure Robot with Blank Name");
    }
  }
}
