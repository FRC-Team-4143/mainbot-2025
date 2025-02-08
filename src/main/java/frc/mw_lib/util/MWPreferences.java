package frc.mw_lib.util;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.JsonNodeFactory;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

public class MWPreferences extends JSONReader {
  private static MWPreferences instance_;

  public static MWPreferences getInstance() {
    if (instance_ == null) {
      instance_ = new MWPreferences();
    }
    return instance_;
  }

  protected static final Path pref_path_ =
      Filesystem.getOperatingDirectory().toPath().resolve("preferences.json");

  protected JsonNodeFactory factory_;
  protected DateTimeFormatter date_format_;

  protected MWPreferences() {
    factory_ = new JsonNodeFactory(false);
    date_format_ = DateTimeFormatter.ofPattern("yyyy_MM_dd_HH_mm_ss");

    // If the robot doesn't have a preferences file, write it first
    if (!pref_path_.toFile().isFile()) {
      root_node_ = new ObjectNode(factory_);
      writeFile();
    }

    try {
      loadJson(pref_path_);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load preferences file !!!", e.getStackTrace());
    }
  }

  protected void writeFile() {
    try {
      File json_file = pref_path_.toFile();
      json_file.createNewFile();
      mapper_.writeValue(json_file, root_node_);

    } catch (Exception e) {
      DriverStation.reportError("Failed to write preferences file!!!!", e.getStackTrace());
    }
  }

  public void setPreference(String name, Number val) {
    ObjectNode new_pref = new ObjectNode(factory_);
    new_pref.put("type", "Number");
    new_pref.put("last_write", LocalDateTime.now().format(date_format_));
    new_pref.put("value", (double) val);

    // register the updated node back to the tree
    ObjectNode tree_root = (ObjectNode) root_node_;
    if (walkTree(tree_root, name).isMissingNode()) {
      tree_root.set(name, new_pref);
    } else {
      tree_root.replace(name, new_pref);
    }

    writeFile();
  }

  public void setPreference(String name, String val) {
    ObjectNode new_pref = new ObjectNode(factory_);
    new_pref.put("type", "String");
    new_pref.put("last_write", LocalDateTime.now().format(date_format_));
    new_pref.put("value", val);

    // register the updated node back to the tree
    ObjectNode tree_root = (ObjectNode) root_node_;
    if (walkTree(tree_root, name).isMissingNode()) {
      tree_root.set(name, new_pref);
    } else {
      tree_root.replace(name, new_pref);
    }

    writeFile();
  }

  public boolean hasPreference(String name) {
    JsonNode node = walkTree(root_node_, name);
    return !node.isMissingNode();
  }

  public String getPreferenceString(String name, String default_val) {
    JsonNode node = walkTree(root_node_, name);
    if (node.isMissingNode()) {
      return default_val;
    }
    if (!walkTree(root_node_, name, "type").asText().equals("String")) {
      return default_val;
    }
    return walkTree(root_node_, name, "value").asText();
  }

  public int getPreferenceInt(String name, int default_val) {
    JsonNode node = walkTree(root_node_, name);
    if (node.isMissingNode()) {
      return default_val;
    }
    if (!walkTree(root_node_, name, "type").asText().equals("Number")) {
      return default_val;
    }
    return walkTree(root_node_, name, "value").asInt();
  }

  public double getPreferenceDouble(String name, double default_val) {
    JsonNode node = walkTree(root_node_, name);
    if (node.isMissingNode()) {
      return default_val;
    }
    if (!walkTree(root_node_, name, "type").asText().equals("Number")) {
      return default_val;
    }
    return walkTree(root_node_, name, "value").asDouble();
  }
}
