#include "articore/dds/service.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace articore::dds {
namespace {
std::string trim(std::string value) {
  const auto blank = [](unsigned char ch) { return std::isspace(ch) != 0; };
  value.erase(value.begin(),
              std::find_if_not(value.begin(), value.end(), blank));
  value.erase(std::find_if_not(value.rbegin(), value.rend(), blank).base(),
              value.end());
  return value;
}

std::vector<std::string> split(const std::string& text) {
  std::vector<std::string> values;
  std::istringstream stream(text);
  std::string value;
  while (std::getline(stream, value, ',')) {
    value = trim(value);
    if (!value.empty()) values.push_back(value);
  }
  return values;
}

bool boolean(const std::string& value) {
  if (value == "true" || value == "yes" || value == "1") return true;
  if (value == "false" || value == "no" || value == "0") return false;
  throw std::invalid_argument("invalid boolean: " + value);
}

bool valid_interface(const std::string& value) {
  return !value.empty() && value.size() <= 15 &&
      std::all_of(value.begin(), value.end(), [](unsigned char ch) {
        return std::isalnum(ch) || ch == '-' || ch == '_' || ch == '.' ||
               ch == ':';
      });
}
}  // namespace

Result<ServiceConfig> load_service_config(const std::string& path) {
  try {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot open configuration: " + path);
    ServiceConfig result;
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(input, line)) {
      ++line_number;
      line = trim(line);
      if (line.empty() || line.front() == '#') continue;
      const auto separator = line.find('=');
      if (separator == std::string::npos) {
        throw std::invalid_argument("missing '=' on line " +
                                    std::to_string(line_number));
      }
      const auto key = trim(line.substr(0, separator));
      const auto value = trim(line.substr(separator + 1));
      if (key == "robot_id") result.robot_id = value;
      else if (key == "domain_id") result.domain_id = std::stoul(value);
      else if (key == "dds_interfaces") result.network_interfaces = split(value);
      else if (key == "left_can_interface") result.runtime.left_can_interface = value;
      else if (key == "right_can_interface") result.runtime.right_can_interface = value;
      else if (key == "with_grippers") {
        // Compatibility with 1.0.0-1.0.3 conffiles. Physical topology is now
        // discovered independently on each CAN channel at Runtime startup.
        (void)boolean(value);
      }
      else if (key == "initial_control_mode") {
        if (value == "pv") result.runtime.initial_control_mode = ControlMode::Pv;
        else if (value == "mit") result.runtime.initial_control_mode = ControlMode::Mit;
        else throw std::invalid_argument("initial_control_mode must be pv or mit");
      } else if (key == "realtime") result.runtime.threads.realtime = boolean(value);
      else if (key == "lock_memory") result.runtime.threads.lock_memory = boolean(value);
      else if (key == "control_cpu") result.runtime.threads.control_cpu = std::stoi(value);
      else if (key == "can_tx_cpu") result.runtime.threads.can_tx_cpu = std::stoi(value);
      else if (key == "can_rx_cpu") result.runtime.threads.can_rx_cpu = std::stoi(value);
      else if (key == "control_priority") result.runtime.threads.control_priority = std::stoi(value);
      else if (key == "can_tx_priority") result.runtime.threads.can_tx_priority = std::stoi(value);
      else if (key == "can_rx_priority") result.runtime.threads.can_rx_priority = std::stoi(value);
      else if (key == "motor_discovery_timeout_ms") {
        result.runtime.motor_discovery_timeout =
            std::chrono::milliseconds(std::stoul(value));
      }
      else if (key == "motor_discovery_retries") {
        const auto retries = std::stoul(value);
        if (retries > 10) {
          throw std::invalid_argument(
              "motor_discovery_retries must be within 0..=10");
        }
        result.runtime.motor_discovery_retries =
            static_cast<std::uint32_t>(retries);
      }
      else throw std::invalid_argument("unknown configuration key: " + key);
    }
    if (result.robot_id.empty() || result.robot_id.size() > 63) {
      throw std::invalid_argument("robot_id must contain 1..63 bytes");
    }
    if (result.network_interfaces.empty()) {
      throw std::invalid_argument("at least one DDS interface is required");
    }
    for (const auto& interface : result.network_interfaces) {
      if (!valid_interface(interface)) {
        throw std::invalid_argument("invalid DDS interface name: " + interface);
      }
    }
    if (!valid_interface(result.runtime.left_can_interface) ||
        !valid_interface(result.runtime.right_can_interface)) {
      throw std::invalid_argument("invalid CAN interface name");
    }
    if (result.runtime.motor_discovery_timeout <=
        std::chrono::milliseconds::zero()) {
      throw std::invalid_argument(
          "motor_discovery_timeout_ms must be positive");
    }
    return result;
  } catch (const std::invalid_argument& error) {
    return Status::failure(RuntimeErrorCode::InvalidArgument, error.what());
  } catch (const std::exception& error) {
    return Status::failure(RuntimeErrorCode::InternalError, error.what());
  }
}

std::string cyclone_uri(const ServiceConfig& config) {
  std::ostringstream xml;
  xml << "<CycloneDDS><Domain Id=\"" << config.domain_id
      << "\"><General><Interfaces>";
  for (std::size_t i = 0; i < config.network_interfaces.size(); ++i) {
    xml << "<NetworkInterface name=\"" << config.network_interfaces[i]
        << "\" priority=\"" << (i == 0 ? "10" : "0") << "\"/>";
  }
  xml << "</Interfaces><AllowMulticast>true</AllowMulticast>"
         "</General><Internal><Watermarks><WhcHigh>500kB</WhcHigh>"
         "</Watermarks></Internal></Domain></CycloneDDS>";
  return xml.str();
}

}  // namespace articore::dds
