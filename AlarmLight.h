#ifndef _ALARM_LIGHT_H
#define _ALARM_LIGHT_H

#include <stdlib.h>
#include <Time.h>
#include <TimeLib.h>
#include <vector>

namespace SmartHome
{
    namespace Logic
    {
        class RTrig
        {
            public:
            RTrig(bool* var)
            {
                this->val = var;
                this->isFun = false;
            }
            RTrig(bool (*var)() )
            {
                this->var = var;
                this->isFun = true;
            }
            RTrig(std::function<bool()> var)
            {
                this->var = var;
                this->isFun = true;
            }

            void update()
            {
                bool x;
                if(isFun)
                {
                    x = var();
                }
                else
                {
                    x = *val;
                }
                

                if(x & !prev)
                {
                    q = true;
                }
                else
                {
                    q = false;
                }
                prev = x;
            }
            bool Q() {return q;}

            private:
            bool isFun;
            int args;
            bool *val;
            std::function<bool()> var; 
            bool prev;
            bool q;
        };
    }

    typedef enum {
        WAIT,
        PREALARM,
        ALARM,
        ERROR
    } AlarmState;

    class AlarmStatus
    {
        public:
        AlarmStatus(AlarmState state, time_t t, time_t alarm)
        {
            this->state = state;
            this->t     = t;
            this->alarm = alarm;
        }
        AlarmState  state;
        time_t      t;
        time_t      alarm;
    };

    // Day numbers
    const int Sunday    = 0;
    const int Monday    = 1;
    const int Tuesday   = 2;
    const int Wednesday = 3;
    const int Thursday  = 4;
    const int Friday    = 5;
    const int Saturday  = 6;


    class AlarmLight
    {
    public:
        AlarmLight(int* maxRGBW, TimeElements* defaults);
        ~AlarmLight();

        void    update();
        int*    getLights();
        void    acknowledge();
        time_t  clearDefault();
        time_t  clearTemp();
        time_t  clearNext();
        void    setTemp(TimeElements alarm);
        int*    getRGBW();
        AlarmStatus getStatus();

    private:
        TimeElements tm;

        time_t getNextAlarm();
        void setDefaults();
        
        AlarmState state;
        bool usingTemp;
        std::vector<time_t> defaultAlarms;
        std::vector<time_t> tempAlarms;
        TimeElements* defaults;
        time_t getNextWday(TimeElements tn);

        Logic::RTrig* resetDefaults;

        int* maxRGBW;
        int* rgbw;
    }; // class AlarmLight
}; // namespace SmartHome

#endif
