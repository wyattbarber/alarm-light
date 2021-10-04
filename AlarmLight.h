#ifndef _ALARM_LIGHT_H
#define _ALARM_LIGHT_H

#include <stdlib.h>
#include <Time.h>
#include <TimeLib.h>
#include <vector>
#include <math.h>
#include <String.h>

namespace SmartHome
{
    namespace Helpers
    {
        class RTrig
        {
            public:
            RTrig(bool* var)
            {
                this->val = var;
                this->isFun = false;
            }
            /*RTrig(bool (*var)() )
            {
                this->var = var;
                this->isFun = true;
            }*/
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
    
        uint8_t* rgbFromK(int K)
        {
            uint8_t red, green, blue;

            if (K <= 66000) { red = 255; }
            else
            {
                red = 392.698727446 * pow(K - 60000, -0.1332047592);
                if (red > 255) {red = 255;}
                if (red < 0) {red = 0;}
            }
            
            if (K <= 66000)
            { 
                green = 99.4708025861 * log(K) - 161.1195681661;
                if (green > 255) {green = 255;}
                if (green < 0) {green = 0;}
            }
            else
            {
                green = 288.1221695283 * pow(K - 60000, -0.0755148492);
                if (green > 255) {green = 255;}
                if (green < 0) {green = 0;}
            }
            
            if (K >= 66000) {blue = 255;}
            else if (K <= 19000) {blue = 0;}
            else
            {
                138.5177312231 * log(K - 10000) - 305.0447927307;
                if (blue > 255) {blue = 255;}
                if (blue < 0) {blue = 0;}
            }
        }
    }

    typedef enum {
        WAIT_DAY,
        WAIT_ALARM,
        LIGHTUP,
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
        String toString(){
            String ret = "State: ";
            switch(state)
            {
                case WAIT_DAY:
                {
                    ret += "WAIT_DAY, ";
                    break;
                }
                case WAIT_ALARM:
                {
                    ret += "WAIT_ALARM, ";
                    break;
                }
                case LIGHTUP: 
                {
                    ret += "LIGHTUP, ";
                    break;
                }
                case ALARM:
                {
                    ret += "ALARM, ";
                    break;
                }
            }
            ret += "Time: " + String(t) + ", ";
            ret += "Alarm: " + String(alarm);
            return ret;
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
        AlarmLight(const int* maxRGBW);
        ~AlarmLight();

        void    update();
        int*    getLights();
        bool    getBuzzer();
        time_t  getNext(); 
        void    set(time_t alarm);
        void    acknowledge();
        void    clear();
        AlarmStatus getStatus();

    private:      
        AlarmState state;  
        
//        std::vector<time_t> alarms;
        time_t alarm;

        int maxRGBW[4];
        int rgbw[4];
        bool buzzer;

        static const long LIGHTUP_TIME = 30 * SECS_PER_MIN;
        static const long MAX_ALARM_TIME = 20 * SECS_PER_MIN;
    }; // class AlarmLight
}; // namespace SmartHome

#endif
