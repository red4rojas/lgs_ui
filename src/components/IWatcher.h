#ifndef IWATCHER_H
#define IWATCHER_H
#include <vector>

class IWatcher{
    public:
        virtual ~IWatcher(){};
        virtual void Update(const int &signal) =0;
        // virtual void Update(const std::string()) =0;
};
#endif //IWATCHER_H