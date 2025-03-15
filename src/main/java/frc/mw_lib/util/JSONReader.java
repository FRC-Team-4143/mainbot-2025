package frc.mw_lib.util;

import com.fasterxml.jackson.core.JsonLocation;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Iterator;

public class JSONReader {
  protected class ConstantParseError extends JsonProcessingException {

    public ConstantParseError(String msg, JsonLocation loc) {
      super(msg, loc, null);
    }

    public ConstantParseError(String msg) {
      super(msg, null, null);
    }
  }

  protected JsonNode root_node_;
  protected ObjectMapper mapper_;

  public JSONReader() {
    mapper_ = new ObjectMapper();
  }

  protected void loadJson(String json_file) throws IOException {
    loadJson(Filesystem.getOperatingDirectory().toPath().resolve(json_file));
  }

  /**
   * Loads JSON content from a specific file into a JSON tree for parsing
   *
   * @param json_file
   * @throws IOException
   */
  protected void loadJson(Path json_file) throws IOException {
    String json_data = new String(Files.readAllBytes(json_file));
    root_node_ = mapper_.readTree(json_data);
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
  protected JsonNode searchArrayForPair(JsonNode starting, String child_key, String child_value)
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
  protected static JsonNode walkTree(JsonNode starting, String... path_steps) {
    JsonNode current = starting;
    for (String path_segment : path_steps) {
      current = current.path(path_segment);
    }

    return current;
  }

  protected static ArrayList<String> getChildren(JsonNode parent) {
    ArrayList<String> children = new ArrayList<>();
    parent
        .fieldNames()
        .forEachRemaining(
            (String name) -> {
              children.add(name);
            });

    return children;
  }
}
