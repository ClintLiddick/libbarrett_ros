#ifndef LIBBARRETT_ROS_REALTIME_LOGGER_H_
#define LIBBARRETT_ROS_REALTIME_LOGGER_H_
#include <fstream>
#include <list>
#include <string>

namespace libbarrett_ros {

class RealtimeLogger {
public:
  RealtimeLogger(std::string const &output_path);
  ~RealtimeLogger();

  void logMessage(std::string const &message);
  void logMessage(std::string const &message, double stamp);

private:
  std::ofstream stream_;
  std::list<std::pair<double, std::string> > buffer_;
};

}

#endif
