#ifndef _WAKE_LIGHT_H
#define _WAKE_LIGHT_H

#include <limits.h>

namespace WakeLight {

    enum DaysOfWeekNum{Sunday,Monday,Tuesday,Wednesday,Thursday,Friday,Saturday};
    char DaysOfWeekStr[7][12] {"Sunday","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday"};
    inline DaysOfWeekNum operator++(DaysOfWeekNum& day, int) {
        if(day == Saturday){
            return Sunday;
        }
        else {
             return DaysOfWeekNum(day+1);
        }    
    }
    inline DaysOfWeekNum operator--(DaysOfWeekNum& day, int) {
        if(day == Sunday){
            return Saturday;
        }
        else {
             return DaysOfWeekNum(day-1);
        }
    }

    class Time {
        public:
        long int toSeconds(){
            long int seconds = 0;
            seconds += this->hours*60*60;
            seconds += this->minutes*60;
            seconds += this->seconds;
            return seconds;
        }
        bool operator==(Time other) {
            return this->day        == other.day 
                && this->hours      == other.minutes
                && this->minutes    == other.minutes
                && this->seconds    == other.seconds;
        }
        bool operator>(Time other) {

        }
        bool operator<(Time other) {

        }
        bool operator>=(Time other) {
            return (*this > other) || (*this == other);
        }
        bool operator<=(Time other) {
            return (*this < other) || (*this == other);
        }
        Time operator=(Time other) {
            Time out;
            out.day     = other.day;
            out.hours   = other.hours;
            out.minutes = other.minutes;
            out.seconds = other.seconds;
            return out;
        }
        Time operator-(Time other) {
            Time out;
            if(this->day == other.day) {
                out.hours = abs(this->hours - other.hours);
                out.minutes = abs(this->minutes - other.minutes);
                out.seconds = abs(this->seconds - other.seconds);
            }
            else if(other.day == (this->day++)){
                out.hours = abs(this->hours - (other.hours + 24));
                out.minutes = abs(this->minutes - other.minutes);
                out.seconds = abs(this->seconds - other.seconds);
            }
            else if(other.day == (this->day--)){
                out.hours = abs(this->hours - (other.hours - 24));
                out.minutes = abs(this->minutes - other.minutes);
                out.seconds = abs(this->seconds - other.seconds);
            }
            return out;
        }
        DaysOfWeekNum day;
        int hours;
        int minutes;
        int seconds;
    };

    enum State {
        WAIT,
        WAKE,
        ALARM,
        ERROR
    };

    class WakeLight {
        public:
        WakeLight(){}
        ~WakeLight(){}

        void setDefaultAlarm(Time alarm){
            defaults[alarm.day] = alarm;
        }
        void setTempAlarm(Time alarm){
            tempAlarm = alarm;
        }
        void setNetworkTime(Time time){
            networkBaseTime = time;
        }
        void update(){
            switch(state) {
                case WAIT:

                case WAKE:
                // calculate progress of 30 minute wake fade, percentage from 0 to 1.
                float progress = ((currentTime - getLightTime()).toSeconds()) / (30 * 60);


                case ALARM:

                case ERROR:

            }
        }
        uint8* getRGBWLevels(){}
        protected:
        void updateInternalTime(){
            // update time since last update
            long int currMillis = millis();
            int diff;
            if(currMillis < prevMillis){
                diff = currMillis + (LONG_MAX - prevMillis);
            }
            else {
                diff = currMillis - prevMillis;
            }

            // calculate h, m, s differences
            int seconds = (diff/1000)%60;
            int minutes = seconds/60;
            int hours   = minutes/60;

            // update current time
            currentTime.seconds += seconds;
            if(currentTime.seconds > 59) {
                currentTime.minutes += 1;
                currentTime.seconds -= 60;
            }
            currentTime.minutes += minutes;
            if(currentTime.minutes > 59) {
                currentTime.hours += 1;
                currentTime.minutes -= 60;
            }
            currentTime.hours += hours;
            if(currentTime.hours > 23) {
                currentTime.hours -= 24;
                if(currentTime.day == Saturday){
                    currentTime.day = Sunday;
                }
                else {
                    currentTime.day = DaysOfWeekNum(currentTime.day+1);
                }
            }
        }
        Time getCurrentAlarm(){
            Time todaysDefault = defaults[currentTime.day];
            if(useTempAlarm && (tempAlarm.day == todaysDefault.day)) {
                return tempAlarm;
            }
            else {
                return todaysDefault;
            }
        }
        Time getLightTime(){
            Time lightTime = getCurrentAlarm();
            lightTime.minutes -= 30;
            if(lightTime.minutes < 0) {
                lightTime.day--;
            }
            return lightTime;
        }
        

        private:
        State state;
        // Alarm settings
        Time defaults[7];
        Time tempAlarm;
        bool useTempAlarm;
        // Time measurement
        Time networkBaseTime;
        Time currentTime;
        long int prevMillis;
        // State-specific values
        // WAIT

        // WAKE
        uint8 rgbw[4];
        uint8 maxRed;
        uint8 maxGrn;
        uint8 maxBlu;
        uint8 maxWht;

        // ALARM

        // ERROR
    };

}

#endif
