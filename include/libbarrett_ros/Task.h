#ifndef LIBBARRETT_ROS_TASK_H_
#define LIBBARRETT_ROS_TASK_H_

namespace libbarrett_ros {

class Task {
public:
    virtual ~Task() {}

    virtual void Request() = 0;
    virtual void Receive(bool blocking) = 0;
    virtual void Write() = 0;
};

} // namespace libbarrett_ros

#endif // ifndef LIBBARRETT_ROS_TASK_H_
