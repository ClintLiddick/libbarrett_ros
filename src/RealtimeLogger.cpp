#include <iomanip>
#include <barrett/os.h>
#include <boost/foreach.hpp>
#include <libbarrett_ros/RealtimeLogger.h>


using libbarrett_ros::RealtimeLogger;

RealtimeLogger::RealtimeLogger(std::string const &output_path)
  : stream_(output_path.c_str())
{
}

RealtimeLogger::~RealtimeLogger()
{
  typedef std::pair<double, std::string> Entry;

  BOOST_FOREACH (Entry const &entry, buffer_) {
    stream_ << std::setprecision(9) << std::setw(20) << std::left << entry.first << ' '
            << entry.second << '\n';
  }
}

void RealtimeLogger::logMessage(std::string const &message)
{
  double const stamp = barrett::highResolutionSystemTime();
  logMessage(message, stamp);
}

void RealtimeLogger::logMessage(std::string const &message, double stamp)
{
  buffer_.push_back(std::make_pair(stamp, message));
}
