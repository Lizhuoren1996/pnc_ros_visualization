#include <stdio.h>
#include <unistd.h>
#include <string>
#include <zcm/zcm-cpp.hpp>
#include "example_t.hpp"
using std::string;
#include <iostream>
#include <unistd.h>
#include <thread>

int num = 0;

void pub() {
    while (true)
    {
        std::cout << "num:" << num << std::endl;
        usleep(2000 * 1000);
    }
    
}


class Handler
{
    public:
        ~Handler() {}

        void handleMessage(const zcm::ReceiveBuffer* rbuf,
                           const string& chan,
                           const example_t *msg)
        {
            num ++;
            std::cout << "3" << std::endl;
            // printf("Received message on channel \"%s\":\n", chan.c_str());
            // printf("  timestamp   = %lld\n", (long long)msg->timestamp);
            // printf("  position    = (%f, %f, %f)\n",
            //         msg->position[0], msg->position[1], msg->position[2]);
            // printf("  orientation = (%f, %f, %f, %f)\n",
            //         msg->orientation[0], msg->orientation[1],
            //         msg->orientation[2], msg->orientation[3]);
            // printf("  ranges:");
            // for(int i = 0; i < msg->num_ranges; i++)
            //     printf(" %d", msg->ranges[i]);
            // printf("\n");
            // printf("  name        = '%s'\n", msg->name.c_str());
            // printf("  enabled     = %d\n", msg->enabled);
            // usleep(200 * 1000);
            
        }
};

int main(int argc, char *argv[])
{
    zcm::ZCM zcm {"ipc"};
    if (!zcm.good())
        return 1;
    std::thread Pub(pub);

    std::cout << "1" << std::endl;
    Handler handlerObject;
    zcm.subscribe("EXAMPLE", &Handler::handleMessage, &handlerObject);
        std::cout << "22" << std::endl;

    zcm.subscribe("FOOBAR", &Handler::handleMessage, &handlerObject);
    zcm.run();
    Pub.join();

    // std::cout << "2" << std::endl;
    return 0;
}
