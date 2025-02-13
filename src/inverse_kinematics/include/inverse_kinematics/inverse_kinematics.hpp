


class InverseKinematics{
    public:
        InverseKinematics();
        ~InverseKinematics();

        void updateDesiredSpeed(double v_x, double v_y, double omega);
        
    private:
        double omega_RB;
        double omega_RF;
        double omega_LB;
        double omega_LF;

        double v_x;
        double v_y;
        double omega;

};