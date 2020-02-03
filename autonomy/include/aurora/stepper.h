/*
Driver header to enable stepper integration.
Addeline Mitchell 2019-12-01
*/
#ifndef __STEPPER_H
#define __STEPPER_H

class Stepper
{
    public:
        Stepper()
        {
            loc = 0.0;
        }

        ~Stepper()
        {}

        void look(float);
        bool limit();

        void setMin(bool);
        void setMax(bool);

        float loc;

    private:
        bool min, max;
};


// Update obj.min to reflect whether obj.loc is at the lower limit
void Stepper::setMin(bool set)
{
    min = set;
}

// Update obj.max to reflect whether obj.loc is at the upper limit
void Stepper::setMax(bool set)
{
    max = set;
}

// Update location of stepper from current position to new position
void Stepper::look(float deg)
{
    loc += deg - loc;
}

// Return whether or not object is at a limit - DRY
bool Stepper::limit()
{
    if(min || max)
    {
        return true;
    }
    return false;
}

#endif
