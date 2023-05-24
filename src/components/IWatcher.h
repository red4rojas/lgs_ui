#ifndef IWATCHER_H
#define IWATCHER_H
#include <vector>
#include <string>

class IWatcher{
    public:
        virtual ~IWatcher(){};
        virtual void Update(const std::string signal) =0;
};
#endif //IWATCHER_H