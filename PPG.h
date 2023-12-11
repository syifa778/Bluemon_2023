#ifndef PPG_H
#define PPG_H

class PPG {
    public:
        int signal_length;
        float *sinyal;
        int *peaks;
        int *valleys;
        int detected_PV = 0;
        float meanID = 0;
        float meanIS = 0;

        PPG(int n_data, float *data_sinyal);
        void signalReflection (int n_ref);
        void digitalFilter (int orde_filter, float *koef_a, float *koef_b);
        void ADS (int signal_start, int signal_end, int max_PV);
        void meanIDIS(int n_idis);
        int KNN (int K, int N_train, float *x_train, float *y_train, float x);
        
};

#endif
