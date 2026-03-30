#pragma once

#include <array>
#include <cstddef>
#include <sstream>
#include <string>

namespace go2w_d1_arm::ipc
{
inline constexpr std::size_t kJointCount = 7;
inline constexpr char kCommandPrefix = 'C';
inline constexpr char kServoPrefix = 'S';
inline constexpr char kFeedbackPrefix = 'F';
inline constexpr const char kDefaultSocketPath[] = "/tmp/go2w_d1_arm.sock";

inline std::string SanitizePayload(std::string payload)
{
  for (char & character : payload) {
    if (character == '\n' || character == '\r') {
      character = ' ';
    }
  }
  return payload;
}

inline std::string BuildCommandLine(const std::string & payload)
{
  return std::string(1, kCommandPrefix) + " " + SanitizePayload(payload);
}

inline std::string BuildFeedbackLine(const std::string & payload)
{
  return std::string(1, kFeedbackPrefix) + " " + SanitizePayload(payload);
}

inline std::string BuildServoLine(const std::array<double, kJointCount> & degrees)
{
  std::ostringstream stream;
  stream << kServoPrefix;
  stream.setf(std::ios::fixed);
  stream.precision(6);

  for (double degree : degrees) {
    stream << ' ' << degree;
  }

  return stream.str();
}

inline bool ParseCommandLine(const std::string & line, std::string * payload)
{
  if (line.size() < 3 || line[0] != kCommandPrefix || line[1] != ' ') {
    return false;
  }

  if (payload != nullptr) {
    *payload = line.substr(2);
  }
  return true;
}

inline bool ParseFeedbackLine(const std::string & line, std::string * payload)
{
  if (line.size() < 3 || line[0] != kFeedbackPrefix || line[1] != ' ') {
    return false;
  }

  if (payload != nullptr) {
    *payload = line.substr(2);
  }
  return true;
}

inline bool ParseServoLine(
  const std::string & line,
  std::array<double, kJointCount> * positions_deg)
{
  if (line.empty() || line[0] != kServoPrefix) {
    return false;
  }

  std::istringstream stream(line.substr(1));
  std::array<double, kJointCount> parsed{};
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (!(stream >> parsed[i])) {
      return false;
    }
  }

  double trailing_value = 0.0;
  if (stream >> trailing_value) {
    return false;
  }

  if (positions_deg != nullptr) {
    *positions_deg = parsed;
  }
  return true;
}
}  // namespace go2w_d1_arm::ipc
