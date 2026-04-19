/** \file algorithm.cpp ******************************************************
*
* Project: MAXREFDES117#
* Filename: algorithm.cpp
* Description: This module calculates the heart rate/SpO2 level
*
*
* --------------------------------------------------------------------
*
* This code follows the following naming conventions:
*
* char              ch_pmod_value
* char (array)      s_pmod_s_string[16]
* float             f_pmod_value
* int32_t           n_pmod_value
* int32_t (array)   an_pmod_value[16]
* int16_t           w_pmod_value
* int16_t (array)   aw_pmod_value[16]
* uint16_t          uw_pmod_value
* uint16_t (array)  auw_pmod_value[16]
* uint8_t           uch_pmod_value
* uint8_t (array)   auch_pmod_buffer[16]
* uint32_t          un_pmod_value
* int32_t *         pn_pmod_value
*
* ------------------------------------------------------------------------- */
/*******************************************************************************
* Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

typedef uint8_t byte;
typedef bool boolean;
#include "algorithm_1.h"
//#include "Arduino.h"

const float uch_spo2_table[184] = {
  94.845f,95.144034f,95.434056f,95.715066f,95.987064f,96.25005f,96.504024f,96.748986f,96.984936f,97.211874f,97.4298f,97.638714f,97.838616f,98.029506f,
  98.211384f,98.38425f,98.548104f,98.702946f,98.848776f,98.985594f,99.1134f,99.232194f,99.341976f,99.442746f,99.534504f,99.61725f,99.690984f,99.755706f,
  99.811416f,99.858114f,99.8958f,99.924474f,99.944136f,99.954786f,99.956424f,99.94905f,99.932664f,99.907266f,99.872856f,99.829434f,99.777f,99.715554f,
  99.645096f,99.565626f,99.477144f,99.37965f,99.273144f,99.157626f,99.033096f,98.899554f,98.757f,98.605434f,98.444856f,98.275266f,98.096664f,97.90905f,
  97.712424f,97.506786f,97.292136f,97.068474f,96.8358f,96.594114f,96.343416f,96.083706f,95.814984f,95.53725f,95.250504f,94.954746f,94.649976f,94.336194f,
  94.0134f,93.681594f,93.340776f,92.990946f,92.632104f,92.26425f,91.887384f,91.501506f,91.106616f,90.702714f,90.2898f,89.867874f,89.436936f,88.996986f,
  88.548024f,88.09005f,87.623064f,87.147066f,86.662056f,86.168034f,85.665f,85.152954f,84.631896f,84.101826f,83.562744f,83.01465f,82.457544f,81.891426f,
  81.316296f,80.732154f,80.139f,79.536834f,78.925656f,78.305466f,77.676264f,77.03805f,76.390824f,75.734586f,75.069336f,74.395074f,73.7118f,73.019514f,
  72.318216f,71.607906f,70.888584f,70.16025f,69.422904f,68.676546f,67.921176f,67.156794f,66.3834f,65.600994f,64.809576f,64.009146f,63.199704f,62.38125f,
  61.553784f,60.717306f,59.871816f,59.017314f,58.1538f,57.281274f,56.399736f,55.509186f,54.609624f,53.70105f,52.783464f,51.856866f,50.921256f,49.976634f,
  49.023f,48.060354f,47.088696f,46.108026f,45.118344f,44.11965f,43.111944f,42.095226f,41.069496f,40.034754f,38.991f,37.938234f,36.876456f,35.805666f,
  34.725864f,33.63705f,32.539224f,31.432386f,30.316536f,29.191674f,28.0578f,26.914914f,25.763016f,24.602106f,23.432184f,22.25325f,21.065304f,19.868346f,
  18.662376f,17.447394f,16.2234f,14.990394f,13.748376f,12.497346f,11.237304f,9.96825f,8.690184f,7.403106f,6.107016f,4.801914f,3.4878f,2.164674f,0.832536f,
  0.0f
};

static uint8_t s_algorithm1_banner_printed = 0U;
/* 算法工作缓存放静态区，避免 BUFFER_SIZE 增大后函数栈溢出 */
static int32_t s_algo_an_x[BUFFER_SIZE];

//#if defined(ARDUINO_AVR_UNO)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated.  Samples become 16-bit data.
//void maxim_heart_rate_and_oxygen_saturation(uint16_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint16_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid, 
//                int32_t *pn_heart_rate, int8_t *pch_hr_valid)
//#else
void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint32_t *pun_red_buffer, float *pn_spo2, int8_t *pch_spo2_valid, 
                int32_t *pn_heart_rate, int8_t *pch_hr_valid)
//#endif
/**
* \brief        Calculate the heart rate and SpO2 level
* \par          Details
*               By detecting  peaks of PPG cycle and corresponding AC/DC of red/infra-red signal, the an_ratio for the SPO2 is computed.
*               Since this algorithm is aiming for Arm M0/M3. formaula for SPO2 did not achieve the accuracy due to register overflow.
*               Thus, accurate SPO2 is precalculated and save longo uch_spo2_table[] per each an_ratio.
*
* \param[in]    *pun_ir_buffer           - IR sensor data buffer
* \param[in]    n_ir_buffer_length      - IR sensor data buffer length
* \param[in]    *pun_red_buffer          - Red sensor data buffer
* \param[out]    *pn_spo2                - Calculated SpO2 value
* \param[out]    *pch_spo2_valid         - 1 if the calculated SpO2 value is valid
* \param[out]    *pn_heart_rate          - Calculated heart rate value
* \param[out]    *pch_hr_valid           - 1 if the calculated heart rate value is valid
*
* \retval       None
*/
{
  if (s_algorithm1_banner_printed == 0U) {
    printf("[ALGO] algorithm_1 active\n");
    s_algorithm1_banner_printed = 1U;
  }

  uint32_t un_ir_mean;
  int32_t k, n_i_ratio_count;
  int32_t i, n_exact_ir_valley_locs_count, n_middle_idx;
  int32_t n_th1, n_npks;   
  int32_t an_ir_valley_locs[15] ;
  int32_t n_peak_interval_sum;
  
  int32_t n_y_ac, n_x_ac;
//  int32_t n_spo2_calc; 
  int32_t n_y_dc_max, n_x_dc_max; 
  int32_t n_y_dc_max_idx=0, n_x_dc_max_idx=0; 
  int32_t an_ratio[5], n_ratio_average; 
  int32_t n_nume, n_denom ;
  // calculates DC mean and subtracts DC from ir
  un_ir_mean =0; 
  for (k=0 ; k<n_ir_buffer_length ; k++ ) un_ir_mean += pun_ir_buffer[k] ;
  un_ir_mean =un_ir_mean/n_ir_buffer_length ;
  
  // remove DC and invert signal so that we can use peak detector as valley detector
  for (k=0 ; k<n_ir_buffer_length ; k++ )  
    s_algo_an_x[k] = un_ir_mean - pun_ir_buffer[k] ; 

  // 4 pt Moving Average
  for(k=0; k< BUFFER_SIZE_MA4; k++){
    s_algo_an_x[k]=( s_algo_an_x[k]+s_algo_an_x[k+1]+ s_algo_an_x[k+2]+ s_algo_an_x[k+3])/(int)4;        
  }
  // calculate threshold  
  n_th1=0; 
  for ( k=0 ; k<BUFFER_SIZE_MA4 ;k++){
    n_th1 +=  s_algo_an_x[k];
  }
  n_th1= n_th1/ (BUFFER_SIZE_MA4);
  if( n_th1<30) n_th1=30; // min allowed
  if( n_th1>60) n_th1=60; // max allowed

  for ( k=0 ; k<15;k++) an_ir_valley_locs[k]=0;
  // since we flipped signal, we use peak detector as valley detector
  maxim_find_peaks( an_ir_valley_locs, &n_npks, s_algo_an_x, BUFFER_SIZE_MA4, n_th1, 20, 15 );//peak_height, peak_distance, max_num_peaks 
  n_peak_interval_sum =0;
  if (n_npks>=2){
    for (k=1; k<n_npks; k++) n_peak_interval_sum += (an_ir_valley_locs[k] - an_ir_valley_locs[k -1] ) ;
    n_peak_interval_sum =n_peak_interval_sum/(n_npks-1);
    *pn_heart_rate =(int32_t)( (FS*60)/ n_peak_interval_sum );
    *pch_hr_valid  = 1;
  }
  else  { 
    *pn_heart_rate = -999; // unable to calculate because # of peaks are too small
    *pch_hr_valid  = 0;
  }

  //  load raw IR value again for SPO2 calculation
  for (k=0 ; k<n_ir_buffer_length ; k++ )  {
      s_algo_an_x[k] =  pun_ir_buffer[k] ; 
  }

  // find precise min near an_ir_valley_locs
  n_exact_ir_valley_locs_count = n_npks; 
  
  //using exact_ir_valley_locs , find ir-red DC and ir-red AC for SPO2 calibration an_ratio
  //finding AC/DC maximum of raw

  n_ratio_average =0; 
  n_i_ratio_count = 0; 
  for(k=0; k< 5; k++) an_ratio[k]=0;
  for (k=0; k< n_exact_ir_valley_locs_count; k++){
    if (an_ir_valley_locs[k] > BUFFER_SIZE ) {
      *pn_spo2 =  -999 ; // do not use SPO2 since valley loc is out of range
      *pch_spo2_valid  = 0; 
      return;
    }
  }
  // find max between two valley locations 
  // and use an_ratio betwen AC compoent of Ir & Red and DC compoent of Ir & Red for SPO2 
  for (k=0; k< n_exact_ir_valley_locs_count-1; k++){
    n_y_dc_max= -16777216 ; 
    n_x_dc_max= -16777216; 
    if (an_ir_valley_locs[k+1]-an_ir_valley_locs[k] >3){
        for (i=an_ir_valley_locs[k]; i< an_ir_valley_locs[k+1]; i++){
          if (s_algo_an_x[i]> n_x_dc_max) {n_x_dc_max =s_algo_an_x[i]; n_x_dc_max_idx=i;}
          if ((int32_t)pun_red_buffer[i]> n_y_dc_max) {n_y_dc_max =(int32_t)pun_red_buffer[i]; n_y_dc_max_idx=i;}
      }
      n_y_ac= ((int32_t)pun_red_buffer[an_ir_valley_locs[k+1]] - (int32_t)pun_red_buffer[an_ir_valley_locs[k] ])*(n_y_dc_max_idx -an_ir_valley_locs[k]); //red
      n_y_ac=  (int32_t)pun_red_buffer[an_ir_valley_locs[k]] + n_y_ac/ (an_ir_valley_locs[k+1] - an_ir_valley_locs[k])  ; 
      n_y_ac=  (int32_t)pun_red_buffer[n_y_dc_max_idx] - n_y_ac;    // subracting linear DC compoenents from raw 
      n_x_ac= (s_algo_an_x[an_ir_valley_locs[k+1]] - s_algo_an_x[an_ir_valley_locs[k] ] )*(n_x_dc_max_idx -an_ir_valley_locs[k]); // ir
      n_x_ac=  s_algo_an_x[an_ir_valley_locs[k]] + n_x_ac/ (an_ir_valley_locs[k+1] - an_ir_valley_locs[k]); 
      n_x_ac=  s_algo_an_x[n_y_dc_max_idx] - n_x_ac;      // subracting linear DC compoenents from raw 
      n_nume=( n_y_ac *n_x_dc_max)>>7 ; //prepare X100 to preserve floating value
      n_denom= ( n_x_ac *n_y_dc_max)>>7;
      if (n_denom>0  && n_i_ratio_count <5 &&  n_nume != 0)
      {   
        an_ratio[n_i_ratio_count]= (n_nume*100)/n_denom ; //formular is ( n_y_ac *n_x_dc_max) / ( n_x_ac *n_y_dc_max) ;
        n_i_ratio_count++;
      }
    }
  }
  // choose median value since PPG signal may varies from beat to beat
  maxim_sort_ascend(an_ratio, n_i_ratio_count);
  n_middle_idx= n_i_ratio_count/2;

  if (n_middle_idx >1)
    n_ratio_average =( an_ratio[n_middle_idx-1] +an_ratio[n_middle_idx])/2; // use median
  else
    n_ratio_average = an_ratio[n_middle_idx ];

  if( n_ratio_average>2 && n_ratio_average <184){
//    n_spo2_calc= uch_spo2_table[n_ratio_average] ;
    *pn_spo2 = uch_spo2_table[n_ratio_average];
    *pch_spo2_valid  = 1;//  float_SPO2 =  -45.060*n_ratio_average* n_ratio_average/10000 + 30.354 *n_ratio_average/100 + 94.845 ;  // for comparison with table
  }
  else{
    *pn_spo2 =  -999 ; // do not use SPO2 since signal an_ratio is out of range
    *pch_spo2_valid  = 0; 
  }
}

void maxim_find_peaks( int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num )
/**
* \brief        Find peaks
* \par          Details
*               Find at most MAX_NUM peaks above MIN_HEIGHT separated by at least MIN_DISTANCE
*
* \retval       None
*/
{
  maxim_peaks_above_min_height( pn_locs, n_npks, pn_x, n_size, n_min_height );
  maxim_remove_close_peaks( pn_locs, n_npks, pn_x, n_min_distance );
  *n_npks = min( *n_npks, n_max_num );
}

void maxim_peaks_above_min_height( int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height )
/**
* \brief        Find peaks above n_min_height
* \par          Details
*               Find all peaks above MIN_HEIGHT
*
* \retval       None
*/
{
  int32_t i = 1, n_width;
  *n_npks = 0;
  
  while (i < n_size-1){
    if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i-1]){      // find left edge of potential peaks
      n_width = 1;
      while (i+n_width < n_size && pn_x[i] == pn_x[i+n_width])  // find flat peaks
        n_width++;
      if (pn_x[i] > pn_x[i+n_width] && (*n_npks) < 15 ) {      // find right edge of peaks
        pn_locs[(*n_npks)++] = i;    
        // for flat peaks, peak location is left edge
        i += n_width+1;
      } else
        i += n_width;
    }
    else
      i++;
  }
}

void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance)
/**
* \brief        Remove peaks
* \par          Details
*               Remove peaks separated by less than MIN_DISTANCE
*
* \retval       None
*/
{
    
  int32_t i, j, n_old_npks, n_dist;
    
  /* Order peaks from large to small */
  maxim_sort_indices_descend( pn_x, pn_locs, *pn_npks );

  for ( i = -1; i < *pn_npks; i++ ){
    n_old_npks = *pn_npks;
    *pn_npks = i+1;
    for ( j = i+1; j < n_old_npks; j++ ){
      n_dist =  pn_locs[j] - ( i == -1 ? -1 : pn_locs[i] ); // lag-zero peak of autocorr is at index -1
      if ( n_dist > n_min_distance || n_dist < -n_min_distance )
        pn_locs[(*pn_npks)++] = pn_locs[j];
    }
  }

  // Resort indices int32_to ascending order
  maxim_sort_ascend( pn_locs, *pn_npks );
}

void maxim_sort_ascend(int32_t  *pn_x, int32_t n_size) 
/**
* \brief        Sort array
* \par          Details
*               Sort array in ascending order (insertion sort algorithm)
*
* \retval       None
*/
{
  int32_t i, j, n_temp;
  for (i = 1; i < n_size; i++) {
    n_temp = pn_x[i];
    for (j = i; j > 0 && n_temp < pn_x[j-1]; j--)
        pn_x[j] = pn_x[j-1];
    pn_x[j] = n_temp;
  }
}

void maxim_sort_indices_descend(  int32_t  *pn_x, int32_t *pn_indx, int32_t n_size)
/**
* \brief        Sort indices
* \par          Details
*               Sort indices according to descending order (insertion sort algorithm)
*
* \retval       None
*/ 
{
  int32_t i, j, n_temp;
  for (i = 1; i < n_size; i++) {
    n_temp = pn_indx[i];
    for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j-1]]; j--)
      pn_indx[j] = pn_indx[j-1];
    pn_indx[j] = n_temp;
  }
}


