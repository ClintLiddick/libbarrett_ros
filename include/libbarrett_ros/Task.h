#ifndef LIBBARRETT_ROS_TASK_H_
#define LIBBARRETT_ROS_TASK_H_
#include <stdint.h> // cstdint was added in C++11
#include <string>

namespace libbarrett_ros {

class Task {
public:
  virtual ~Task() {}

  virtual std::string const &name() const = 0;

  virtual uint_fast32_t request_bits() const = 0;
  virtual uint_fast32_t receive_bits() const = 0;
  virtual uint_fast32_t write_bits() const = 0;

  virtual void Request() = 0;
  virtual void Receive(bool blocking) = 0;
  virtual void Write() = 0;
};

} // namespace libbarrett_ros

#endif // ifndef LIBBARRETT_ROS_TASK_H_
