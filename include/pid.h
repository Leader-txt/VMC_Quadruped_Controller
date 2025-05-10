class PID{
    private:
        double kp, ki, kd;
        double prev_error;
        double integral;
        double max_val;
    public:
        PID(double p,double i,double d,double max);
        double calculate(double target,double now);
};