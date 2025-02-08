#ifndef __BIQUAD_HPP
#define __BIQUAD_HPP

class Biquad {
public:
    Biquad(float a0_coeff, float a1_coeff, float a2_coeff, float b1_coeff, float b2_coeff) : a0_coeff_(a0_coeff), a1_coeff_(a1_coeff), a2_coeff_(a2_coeff), b1_coeff_(b1_coeff), b2_coeff_(b2_coeff) {}

    float a0_coeff_;
    float a1_coeff_;
    float a2_coeff_;
    // b0 coeff if always 1 for this implmentation
    float b1_coeff_;
    float b2_coeff_;

    float x_n[3];
    float y_n[3];

    inline void set_steady_state_val(float init_val)
    {
        x_n[0] = init_val;
        x_n[1] = init_val;
        x_n[2] = init_val;
        y_n[0] = init_val;
        y_n[1] = init_val;
        y_n[2] = init_val;
    }
    
    inline float update_and_return_filt_value(float latest_meas)
    {
        x_n[0] = latest_meas;
        y_n[0] = (a0_coeff_ * x_n[0]) + (a1_coeff_ * x_n[1]) + (a2_coeff_ * x_n[2]) + (b1_coeff_ * y_n[1]) + (b2_coeff_ * y_n[2]);
        x_n[2] = x_n[1];
        x_n[1] = x_n[0];
        y_n[2] = y_n[1];
        y_n[1] = y_n[0];
        return y_n[0];
    }
};

#endif // __BIQUAD_HPP
