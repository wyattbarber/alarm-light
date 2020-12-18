#ifndef _WAKE_LIGHT_H
#define _WAKE_LIGHT_H

#include <limits.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

namespace Alarm
{

    const char DaysOfWeekStr[7][12]{"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
    enum DaysOfWeekNum
    {
        Sunday,
        Monday,
        Tuesday,
        Wednesday,
        Thursday,
        Friday,
        Saturday
    };
    inline DaysOfWeekNum operator++(DaysOfWeekNum &day, int)
    {
        if (day == Saturday)
        {
            return Sunday;
        }
        else
        {
            return DaysOfWeekNum(day + 1);
        }
    }
    inline DaysOfWeekNum operator--(DaysOfWeekNum &day, int)
    {
        if (day == Sunday)
        {
            return Saturday;
        }
        else
        {
            return DaysOfWeekNum(day - 1);
        }
    }

    class Time
    {
        public:
        Time()
        {
            Time(Saturday, 100, 100, 100);
        }
        Time(DaysOfWeekNum day, int hours, int minutes, int seconds)
        {
            this->day = day;
            this->hours = hours;
            this->minutes = minutes;
            this->seconds = seconds;
        }

        long int toSeconds()
        {
            long int seconds = 0;
            seconds += this->hours * 60 * 60;
            seconds += this->minutes * 60;
            seconds += this->seconds;
            return seconds;
        }
        String toString()
        {
            return String(DaysOfWeekStr[day]) + ":" + String(hours) + ":" + String(minutes) + ":" + String(seconds);
        }
        bool operator==(Time other)
        {
            return this->day == other.day && this->hours == other.minutes && this->minutes == other.minutes && this->seconds == other.seconds;
        }
        bool operator>(Time other)
        {
            if (this->day == other.day++)
            {
                return true;
            }
            else if (this->day == other.day)
            {
                return (this->toSeconds() > other.toSeconds());
            }
            else
            {
                return false;
            }
        }
        bool operator<(Time other)
        {
            if (this->day == other.day--)
            {
                return true;
            }
            else if (this->day == other.day)
            {
                return (this->toSeconds() < other.toSeconds());
            }
            else
            {
                return false;
            }
        }
        bool operator>=(Time other)
        {
            return (*this > other) || (*this == other);
        }
        bool operator<=(Time other)
        {
            return (*this < other) || (*this == other);
        }
        Time operator-(Time other)
        {
            int hours;
            int minutes;
            int seconds;
            if (this->day == other.day)
            {
                hours = abs(this->hours - other.hours);
                minutes = abs(this->minutes - other.minutes);
                seconds = abs(this->seconds - other.seconds);
            }
            else if (other.day == (this->day++))
            {
                hours = abs(this->hours - (other.hours + 24));
                minutes = abs(this->minutes - other.minutes);
                seconds = abs(this->seconds - other.seconds);
            }
            else if (other.day == (this->day--))
            {
                hours = abs(this->hours - (other.hours - 24));
                minutes = abs(this->minutes - other.minutes);
                seconds = abs(this->seconds - other.seconds);
            }
            return Time(this->day, hours, minutes, seconds);
        }

        DaysOfWeekNum day;
        int hours;
        int minutes;
        int seconds;
    };

    enum State
    {
        WAIT,
        WAKE,
        ALARM,
        ERROR
    };

    typedef struct AlarmStatus
    {
        AlarmStatus(State state, Time time, Time nextAlarm){
            this->state = state;
            this->time = time;
            this->nextAlarm = nextAlarm;
        }
        State state;
        Time time;
        Time nextAlarm;
    } AlarmStatus;

    class WakeLight
    {
    public:
        // Alarm::WakeLight
        WakeLight()
        {
            maxRed = 1000;
            maxGrn = 100;
            maxBlu = 400;
            maxWht = 800;
            state = WAIT;

            defaults[0] = Time(Sunday, 100, 100, 100);
            defaults[1] = Time(Monday, 6, 0, 0);
            defaults[2] = Time(Tuesday, 6, 0, 0);
            defaults[3] = Time(Wednesday, 6, 0, 0);
            defaults[4] = Time(Thursday, 6, 0, 0);
            defaults[5] = Time(Friday, 6, 0, 0);
            defaults[6] = Time(Saturday, 100, 100, 100);

            tempAlarm   = Time(Saturday, 100, 100, 100);

            currentTime = Time(Saturday, 100, 100, 100);
            networkBaseTime = Time(Saturday, 100, 100, 100);

        };

        void setDefaultAlarm(Time alarm)
        {
            defaults[alarm.day] = alarm;
        }
        void setTempAlarm(Time alarm)
        {
            tempAlarm = alarm;
        }
        void setBaseTime(Time time)
        {
            networkBaseTime = time;
        }
        void pressButton()
        {
            // set button true, but only if the alarm is currently going off
            button = (state == ALARM);
        }
        AlarmStatus getStatus() { return AlarmStatus(state, currentTime, getCurrentAlarm()); }
        void update()
        {
            // update time
            updateInternalTime();
            // alarm clock state machine
            switch (state)
            {
            case WAIT:
                // lights are off
                rgbw[0] = 0;
                rgbw[1] = 0;
                rgbw[2] = 0;
                rgbw[3] = 0;

                // check if wake sequence should start
                if (currentTime >= getLightTime())
                {
                    state = WAKE;
                }

            case WAKE:
                // calculate progress of 30 minute wake fade, percentage from 0 to 1.
                progress = ((currentTime - getLightTime()).toSeconds()) / (30 * 60);
                // augment progress curve
                progress = sqrt(progress);
                // calculate rgbw values
                rgbw[0] = progress * maxRed;
                rgbw[1] = progress * maxGrn;
                rgbw[2] = progress * maxBlu;
                rgbw[3] = progress * maxWht;

                // check if alarm time has been reached
                if (currentTime >= getCurrentAlarm())
                {
                    state = ALARM;
                }
                break;

            case ALARM:
                // blink lights
                if (blink)
                {
                    rgbw[0] = maxRed;
                    rgbw[1] = maxGrn;
                    rgbw[2] = maxBlu;
                    rgbw[3] = maxWht;
                }
                else
                {
                    rgbw[0] = 0;
                    rgbw[1] = 0;
                    rgbw[2] = 0;
                    rgbw[3] = 0;
                }
                if ((millis() % 1000) == 0)
                {
                    ~blink;
                }

                // check for button press
                if (button)
                {
                    button = false;
                    state = WAIT;
                }

                break;
            case ERROR:
                // identify and handle error
                // not sure what errors, if any, this class can encounter
                break;
            }
        }
        int *getRGBWLevels() { return rgbw; }

        void updateInternalTime()
        {
            // update time since last update
            long int currMillis = millis();
            int diff;
            if (currMillis < prevMillis)
            {
                diff = currMillis + (LONG_MAX - prevMillis);
            }
            else
            {
                diff = currMillis - prevMillis;
            }
            prevMillis = currMillis;

            // calculate h, m, s differences
            int seconds = (diff / 1000) % 60;
            int minutes = seconds / 60;
            int hours = minutes / 60;
            DaysOfWeekNum day;

            Serial.print("Internal: \n");
            Serial.print("      Hours: ");
            Serial.print(hours);
            Serial.print("\n    Minutes: ");
            Serial.print(minutes);
            Serial.print("\n    Seconds: ");
            Serial.print(seconds);
            Serial.print("\n\n");

            // update current time
            seconds += currentTime.seconds;
            if (seconds > 59)
            {
                minutes += 1;
                seconds -= 60;
            }
            minutes += currentTime.minutes;
            if (minutes > 59)
            {
                hours += 1;
                minutes -= 60;
            }
            hours += currentTime.hours;
            if (hours > 23)
            {
                hours -= 24;
                if (currentTime.day == Saturday)
                {
                    day = Sunday;
                }
                else
                {
                    day = DaysOfWeekNum(currentTime.day + 1);
                }
            }
            currentTime = Time(day, hours, minutes, seconds);
        }
        Time getCurrentAlarm()
        {
            Time todaysDefault = defaults[currentTime.day];
            if (useTempAlarm && (tempAlarm.day == todaysDefault.day))
            {
                return tempAlarm;
            }
            else
            {
                return todaysDefault;
            }
        }
        Time getLightTime()
        {
            Time lightTime = getCurrentAlarm();
            lightTime.minutes -= 30;
            if (lightTime.minutes < 0)
            {
                lightTime.hours--;
                lightTime.minutes += 60;
            }
            return lightTime;
        }

        // status and IO variables
        State state;
        int rgbw[4];
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
        float progress;
        int maxRed;
        int maxGrn;
        int maxBlu;
        int maxWht;

        // ALARM
        bool blink;
        bool button;

        // ERROR
    };

}; // namespace Alarm

namespace SmartHome
{
    namespace ROS
    {

        class WakeLight
        {
        public:
            WakeLight() {}
            ~WakeLight() {}

            void update()
            {
                alarmClock.update();
            }

        private:
            Alarm::WakeLight alarmClock;
        };
    }; // namespace ROS
};     // namespace SmartHome

#endif
