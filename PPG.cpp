#include "PPG.h"
#include "Arduino.h"
#include <bits/stdc++.h>
using namespace std;

PPG::PPG(int n_data, float* data_sinyal){
    signal_length = n_data;
    sinyal = data_sinyal;
}

void PPG::signalReflection (int n_ref){
  for (int i = n_ref; i > 0; i--){
    sinyal[n_ref-i] = sinyal[n_ref+i-1];
  }
  int temp = signal_length-n_ref;
  for (int i = 0; i < n_ref; i++){
    sinyal[temp+i] = sinyal[temp-i-1];
  }
}

void PPG::digitalFilter (int orde_filter, float *koef_a, float *koef_b){
  float x[orde_filter+1] = {0}; x[0] = sinyal[0]; 
  float y[orde_filter+1] = {0};

  for (int i = 0; i < signal_length; i++){
    y[0] = 0;
    for (int j = 0; j <= orde_filter; j++){
      y[0] = koef_a[j]*y[j] + koef_b[j]*x[j] + y[0];
    }
    sinyal[i] = y[0];
    x[0] = sinyal[i+1];
    for (int j = orde_filter; j > 0; j--){
      x[j] = x[j-1];
      y[j] = y[j-1];
    }
  }

  for (int j = 0; j <= orde_filter; j++){
    x[j] = 0;
    y[j] = 0;
  }
  x[0] = sinyal[signal_length-1]; 
  for (int i = (signal_length-1); i > -1; i--){
    y[0] = 0;
    for (int j = 0; j <= orde_filter; j++){
      y[0] = koef_a[j]*y[j] + koef_b[j]*x[j] + y[0];
    }
    sinyal[i] = y[0];
    x[0] = sinyal[i-1];
    for (int j = orde_filter; j > 0; j--){
      x[j] = x[j-1];
      y[j] = y[j-1];
    }
  }

}

void PPG::ADS(int signal_start, int signal_end, int max_PV){
    //Delete previous valleys and peaks array
    delete[] valleys;
    delete[] peaks;
 
    //Segmentation parameters
    int count_first_parse = 0;
    float ampli_first[6];
    int peak_first[6];
    int valley_first[6];

    int curr_valley = signal_start;
    int curr_peak = signal_start + 1;
    int pre_peak = curr_peak;
    int pre_valley = curr_valley;
    int peak = -1;
    int valley = -1;
    int slope_pre = 0;
    int slope_curr = 0;
    valleys = new int[max_PV];
    peaks = new int[max_PV];
    int n = 0;

    //Adaptive Threshold Parameters
    float Amplitude_low = 0;
    float Amplitude_high = 0;
    bool first_parse = true;
    float max_amplitude = -4000;
    float average_amplitude = 0;
    float current_amplitude = 0;
    float avg_Amplitude_low = 0;
    float avg_Amplitude_high = 0;

    //Loop through the signal to find peaks and valleys
    while (curr_valley < (signal_end-1) && n < max_PV){
        curr_valley = curr_valley + 1;
        curr_peak = curr_peak + 1;

        //Get the slope direction
        if (sinyal[pre_peak] > sinyal[pre_valley]){
            slope_pre = 1;
        }
        if (sinyal[curr_peak] > sinyal[curr_valley]){
            slope_curr = 1;
        }

        //Check the slope direction
        if (slope_curr > 0 && slope_pre > 0){
            valley = pre_valley;
            peak = curr_peak;
        }
        else {
            pre_peak = curr_peak;
            pre_valley = curr_valley;

            //If the candidate peak and valley is valid
            if (peak >= 0 && valley >= 0){
                current_amplitude = (sinyal[peak] - sinyal[valley]);
                if (current_amplitude > max_amplitude && current_amplitude < 550 && current_amplitude > 25){
                    max_amplitude = current_amplitude;
                }

                if (first_parse == true){
                    valley_first[count_first_parse] = valley;
                    peak_first[count_first_parse] = peak;
                    ampli_first[count_first_parse] = current_amplitude;
                    count_first_parse++;

                    if (count_first_parse > 5){
                        Amplitude_low =  max_amplitude*0.5;
                        Amplitude_high = max_amplitude*1;
                        for (int i = 0; i < count_first_parse; i++){
                            if ((ampli_first[i] >= Amplitude_low) && (ampli_first[i] <= Amplitude_high)){
                                valleys[n] = valley_first[i];
                                peaks[n] = peak_first[i];
                                average_amplitude = ampli_first[i] + average_amplitude;
                                n++;
                            }
                        }
                        Amplitude_low = (Amplitude_low + (peaks[n-1]-valleys[n-1])*1)/2;
                        Amplitude_high = (Amplitude_high + (peaks[n-1]-valleys[n-1])*6)/2;
                        first_parse = false;
                        average_amplitude = average_amplitude/n;
                    }
                }
                else if (current_amplitude < 550 && current_amplitude > 25){
                  if (current_amplitude >= Amplitude_low && current_amplitude <= Amplitude_high) {
                      valleys[n] = valley;
                      peaks[n] = peak;
                      Amplitude_low = current_amplitude*0.6;
                      Amplitude_high = (Amplitude_high + current_amplitude*1)/2;
                      average_amplitude = (average_amplitude * n + current_amplitude)/(n+1); 
                      n++;
                  }
                  else {
                      avg_Amplitude_low = (Amplitude_low + average_amplitude*0.8)/2;
                      avg_Amplitude_high = Amplitude_high + (average_amplitude*1.2)/2;
                      if(current_amplitude >= avg_Amplitude_low && current_amplitude <= avg_Amplitude_high){
                          valleys[n] = valley;
                          peaks[n] = peak;
                          Amplitude_low = current_amplitude*0.5;
                          Amplitude_high = (Amplitude_high + current_amplitude*0.9)/2;
                          average_amplitude = (average_amplitude * n + current_amplitude)/(n+1); 
                          n++;
                      }
                  }
                }
                peak = -1;
                valley = -1;
            }
        }
        slope_pre = 0;
        slope_curr = 0;
    }
    
    detected_PV = n;
}

void PPG::meanIDIS(int n_idis){
  float ID = 0;
  float IS = 0;
  for (int i = 0; i < n_idis; i++){
    ID = ID + valleys[i];
    IS = IS + peaks[i];
  }
  meanID = ID/n_idis;
  meanIS = IS/n_idis;
}

int PPG::KNN (int K, int N_train, float *x_train, float *y_train, float x){

  pair<float, float> pairt[N_train];
 
  // Storing the respective array
  // elements in pairs.
  for (int i = 0; i < N_train; i++){
      pairt[i].first =  abs(x-x_train[i]);
      pairt[i].second = y_train[i];
  }

  // Sorting the pair array.
  sort(pairt, pairt + N_train);

  // Weighted KNN 
  float sum_a = 0;
  float sum_b = 0;
  for (int j = 0; j < K; j++){
    sum_a = pairt[j].second/pairt[j].first + sum_a;
    sum_b = 1/pairt[j].first + sum_b;
  }
  
  return round(sum_a/sum_b);
}
