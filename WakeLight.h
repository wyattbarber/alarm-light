#ifndef _WAKE_LIGHT_H
#define _WAKE_LIGHT_H

#include <stdlib.h>
#include <Time.h>
#include <TimeLib.h>
#include <vector>

namespace SmartHome
{
    class AlarmLight
    {
    public:
        AlarmLight(int* maxRGBW){}
        ~AlarmLight() {}

        void update();
        void acknowledge();
        void setDefault(TimeElements alarm);
        void setTemp(TimeElements alarm);
        int* getRGBW();

    private:
        static enum State{
            WAIT,
            WAKE,
            ALARM
        };

        time_t getNextAlarm();
        
        State state;
        std::vector<TimeElements> defaults;
        std::vector<TimeElements> temp;
    };
}; // namespace SmartHome

#endif
