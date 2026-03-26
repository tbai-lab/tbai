#include <unistd.h>

#include <iostream>
#include <string>

#include <tbai_sdk/subscriber.hpp>
#include <tbai_sdk/messages/robot_msgs.hpp>

#define TOPIC_RAW "rt/utlidar/cloud"

void msg_callback(const robot_msgs::PointCloud2 &msg) {
    std::cout << "hello, point cloud published" << std::endl;
    // Print Header information
    std::cout << "Header:" << std::endl;
    std::cout << "  stamp: " << msg.header.stamp.sec << "." << msg.header.stamp.nanosec << std::endl;
    std::cout << "  frame_id: " << msg.header.frame_id << std::endl;

    // Print 2D structure
    std::cout << "height: " << msg.height << std::endl;
    std::cout << "width: " << msg.width << std::endl;

    // Print fields info
    std::cout << "fields (" << msg.fields.size() << "):" << std::endl;
    for (size_t i = 0; i < msg.fields.size(); ++i) {
        const auto &field = msg.fields[i];
        std::cout << "  [" << i << "] name: " << field.name << ", offset: " << field.offset
                  << ", datatype: " << static_cast<int>(field.datatype) << ", count: " << field.count << std::endl;
    }

    // Print endianness, point_step, row_step, data size
    std::cout << "is_bigendian: " << (msg.is_bigendian ? "true" : "false") << std::endl;
    std::cout << "point_step: " << msg.point_step << std::endl;
    std::cout << "row_step: " << msg.row_step << std::endl;
    std::cout << "data size: " << msg.data.size() << std::endl;

    // Print is_dense
    std::cout << "is_dense: " << (msg.is_dense ? "true" : "false") << std::endl;
}

int main(int argc, char **argv) {
    tbai::Subscriber<robot_msgs::PointCloud2> sub(TOPIC_RAW, msg_callback);
    std::cout << "Listening for point cloud on topic: " << TOPIC_RAW << std::endl;
    sub.spin();

    return 0;
}
