using System;

namespace Autopilot
{
    public class PID
    {
        public double kP;
        public double kI;
        public double kD;
        public double rangeMin;
        public double rangeMax;
        public Func<double> input;
        public Func<double> setpoint;
        public Func<double> clockSource;
        public Action<double> outputCallback;

        public double error
        {
            get;
            private set;
        }
        public double p
        {
            get;
            private set;
        }
        public double i
        {
            get;
            private set;
        }
        public double d
        {
            get;
            private set;
        }
        public double outputValue
        {
            get;
            private set;
        }
        public double lastInput
        {
            get;
            private set;
        }
        public double lastTime
        {
            get;
            private set;
        }

        public void FixedUpdate()
        {
            //Gather inputs
            double currentTime = clockSource();
            double currentInput = input();
            double currentSetpoint = setpoint();
            error = currentSetpoint - currentInput;
            double deltaInput = lastInput - currentInput;
            double deltaTime = currentTime - lastTime;
            //Return if we get called twice on the same frame
            if ((currentTime - lastTime) < double.Epsilon)
            {
                return;
            }
            //PID calculation
            p = error * kP;
            i += error * kI * deltaTime;
            d = ((deltaInput * kD) / deltaTime)*0.9;
            //Clamp I
            if (i < rangeMin)
            {
                i = rangeMin;
            }
            if (i > rangeMax)
            {
                i = rangeMax;
            }
            outputValue = p + i + d;
            //Clamp output
            if (outputValue < rangeMin)
            {
                outputValue = rangeMin;
            }
            if (outputValue > rangeMax)
            {
                outputValue = rangeMax;
            }
            //Call output delegate if it exists
            outputCallback?.Invoke(outputValue);
            //Save the state for derivative calculation
            lastTime = currentTime;
            lastInput = currentInput;
        }

        public double Output()
        {
            return outputValue;
        }
    }
}