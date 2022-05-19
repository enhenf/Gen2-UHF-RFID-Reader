/* -*- c++ -*- */
/* 
 * Copyright 2015 <Nikos Kargas (nkargas@isc.tuc.gr)>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <gnuradio/prefs.h>
#include <gnuradio/math.h>
#include <cmath>
#include <sys/time.h>
#include "tag_decoder_impl.h"

namespace gr {
  namespace rfid {

    // 从这里可以看出来，make 返回的时 tag_decoder 的子类 tag_decoder_impl 的实例。
    // output_sizes 的 第二个参数 gr_complex 因为是 gnuradio 表示信号的复数形式？

    // 关于 gr_complex 是 gnuradio 的定义 typedef std::complex< float > 	gr_complex
    // 源于 标准库 namespace 中的模板定义，这里用的是 float 填充模板。
    // 有如下用法 : std::complex<double> z1 {2, 5}; // 2 + 5i

    // 显然 output_size 传给 tag_decoder_impl 复数需要占用的空间。
    tag_decoder::sptr
    tag_decoder::make(int sample_rate)
    {

      std::vector<int> output_sizes;
      output_sizes.push_back(sizeof(float));
      output_sizes.push_back(sizeof(gr_complex));

      return gnuradio::get_initial_sptr
        (new tag_decoder_impl(sample_rate,output_sizes));
    }

    /*
     * The private constructor
     */
    // 构造函数，tag_decoder_impl : public tag_decoder
    // tag_decoder : virtual public gr::block;  virtual public 是解决多重继承中的一些问题，所以暂时并无需理会。
    // 在 tag_decoder_impl 中，直接对它的父类的父类 gr::block 进行了相应的初始化。
    // gr::block 是 gnuradio 的一个模块，具体的源代码放在浏览器之中，
    // class GR_RUNTIME_API block : public basic_block // 看起来是什么所谓运行时接口，继承了 basic_block
    tag_decoder_impl::tag_decoder_impl(int sample_rate, std::vector<int> output_sizes)
      : gr::block("tag_decoder",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::makev(2, 2, output_sizes )),
              s_rate(sample_rate)
    {

      // maybe 是存储循环冗余校验码的
      char_bits = (char *) malloc( sizeof(char) * 128);

      // 根据 blog，这是每一位数据的采样数。
      n_samples_TAG_BIT = TAG_BIT_D * s_rate / pow(10,6);      
      GR_LOG_INFO(d_logger, "Number of samples of Tag bit : "<< n_samples_TAG_BIT);
    }

    /*
     * Our virtual destructor.
     */
    tag_decoder_impl::~tag_decoder_impl()
    {

    }


    
    void
    tag_decoder_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
        ninput_items_required[0] = noutput_items;
    }

    // tag_decoder函数通过输入数据与已知前导码的相关性计算进行同步，
    // 得到解码开始的索引数。因存在相位偏移，开始的索引位置需加上半个比特采样数。
    // 首先这个 size 根本没有用到！！
    // 其次 tag_sync 只能计算 RN 16 的位置，但是似乎根本没办法计算 EPC 的位置，
    // 从 general_work 的调用来看， size 对应的是 RN16 / EPC 数据的长度，但是显然不对劲啊！
    int tag_decoder_impl::tag_sync(const gr_complex * in , int size)
    {
      int max_index = 0;
      float max = 0,corr;
      gr_complex corr2;
      
      // Do not have to check entire vector (not optimal)
      for (int i=0; i < 1.5 * n_samples_TAG_BIT ; i++)
      {
        corr2 = gr_complex(0,0);
        corr = 0;
        // sync after matched filter (equivalent)
        // TAG_PREAMBLE_BITS 是前导码的位数，为什么要乘 2 倍？因为这里确实是 12 位前导码。
        for (int j = 0; j < 2 * TAG_PREAMBLE_BITS; j ++)
        {
          // 关于对于某一位的采样应该是有 n_samples_tag_bit 位，这个 / 2 是取一个中间值？
          corr2 = corr2 + in[ (int) (i+j*n_samples_TAG_BIT/2) ] * gr_complex(TAG_PREAMBLE[j],0);
        }
        corr = std::norm(corr2);
        if (corr > max)
        {
          max = corr;
          max_index = i;
        }
      }  

       // Preamble ({1,1,-1,1,-1,-1,1,-1,-1,-1,1,1} 1 2 4 7 11 12)) 
      // 这个东西不知道是什么，后续会用到。
      h_est = (in[max_index] + in[ (int) (max_index + n_samples_TAG_BIT/2) ] + in[ (int) (max_index + 3*n_samples_TAG_BIT/2) ] + in[ (int) (max_index + 6*n_samples_TAG_BIT/2)] + in[(int) (max_index + 10*n_samples_TAG_BIT/2) ] + in[ (int) (max_index + 11*n_samples_TAG_BIT/2)])/std::complex<float>(6,0);  
      

      // Shifted received waveform by n_samples_TAG_BIT/2
      max_index = max_index + TAG_PREAMBLE_BITS * n_samples_TAG_BIT + n_samples_TAG_BIT/2; 
      return max_index;  
    }




    std::vector<float>  tag_decoder_impl::tag_detection_RN16(std::vector<gr_complex> & RN16_samples_complex)
    {
      // detection + differential decoder (since Tag uses FM0)
      std::vector<float> tag_bits,dist;
      float result;
      int prev = 1,index_T=0;
      
      // 这里的 size 为什么要 /2?
      // result 为什么要这么计算？
      // 这里的解码过程是因为是差分编码吗？
      for (int j = 0; j < RN16_samples_complex.size()/2 ; j ++ )
      {
        result = std::real( (RN16_samples_complex[2*j] - RN16_samples_complex[2*j+1])*std::conj(h_est)); 
  
        if (result>0){
          if (prev == 1)
            tag_bits.push_back(0);
          else
            tag_bits.push_back(1);      
          prev = 1;      
        }
        else
        { 
          if (prev == -1)
            tag_bits.push_back(0);
          else
            tag_bits.push_back(1);      
          prev = -1;    
        }
      }
      return tag_bits;
    }


    std::vector<float>  tag_decoder_impl::tag_detection_EPC(std::vector<gr_complex> & EPC_samples_complex, int index)
    {
      std::vector<float> tag_bits,dist;
      float result=0;
      int prev = 1;
      
      int number_steps = 20;
      float min_val = n_samples_TAG_BIT/2.0 -  n_samples_TAG_BIT/2.0/100, max_val = n_samples_TAG_BIT/2.0 +  n_samples_TAG_BIT/2.0/100;

      std::vector<float> energy;

      energy.resize(number_steps);
      for (int t = 0; t <number_steps; t++)
      {  
        for (int i =0; i <256; i++)
        {
          energy[t]+= reader_state->magn_squared_samples[(int) (i * (min_val + t*(max_val-min_val)/(number_steps-1)) + index)];
        }

      }
      // index_T , T, energe 的目的都是为了 T_global 这个 类私有变量服务，
      // 但在本文件中没有看到 T_global 被使用
      // 不对， T 在计算 result 的时候使用。
      int index_T = std::distance(energy.begin(), std::max_element(energy.begin(), energy.end()));
      float T =  min_val + index_T*(max_val-min_val)/(number_steps-1);

      // T estimated
      T_global = T;
      // 与解码RN16不同的是，没有进行相邻两个采样点进行差分，而是相隔T个采样点进行差分计算，具体方法有待研究。
      // index 应该是具体的索引信息。就是 EPC 数据从哪一位开始出现的。
      for (int j = 0; j < 128 ; j ++ )
      {
        result = std::real((EPC_samples_complex[ (int) (j*(2*T) + index) ] - EPC_samples_complex[ (int) (j*2*T + T + index) ])*std::conj(h_est) ); 

        
         if (result>0){
          if (prev == 1)
            tag_bits.push_back(0);
          else
            tag_bits.push_back(1);      
          prev = 1;      
        }
        else
        { 
          if (prev == -1)
            tag_bits.push_back(0);
          else
            tag_bits.push_back(1);      
          prev = -1;    
        }
      }
      return tag_bits;
    }


    
    int
    tag_decoder_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {

      // typedef std::vector<const void*> gr_vector_const_void_star
      // 一个指针数组。显然把指针强制转换为 gr_complex 的指针，所指类型不可变。
      const gr_complex *in = (const  gr_complex *) input_items[0];

      // typedef std::vector<void*> gr_vector_void_star
      // 也是指针，指针所指变量是可变的。
      float *out = (float *) output_items[0];
      gr_complex *out_2 = (gr_complex *) output_items[1]; // for debugging
      
      int written_sync =0;
      int written = 0, consumed = 0;
      int RN16_index , EPC_index;

      // RN16 与 EPC 都是 rfid tag 解码过程的一部分。
      // reader 先向 tag 发送 select 信息。
      // tag 返回一个 RN16 信息
      // reader 返回一个 ACK，其中包含 RN16 信息。
      // tag 返回指针的 EPC 信息。
      std::vector<float> RN16_samples_real;
      std::vector<float> EPC_samples_real;

      std::vector<gr_complex> RN16_samples_complex;
      std::vector<gr_complex> EPC_samples_complex;

      std::vector<float> RN16_bits;
      int number_of_half_bits = 0;

      std::vector<float> EPC_bits;    
      // Processing only after n_samples_to_ungate are available and we need to decode an RN16
      if (reader_state->decoder_status == DECODER_DECODE_RN16 && ninput_items[0] >= reader_state->n_samples_to_ungate)
      {
        // 输入的应该是 gr_complex 的指针数组的首地址，以及该数组的长度。
        
        
        RN16_index = tag_sync(in,ninput_items[0]);

        // 去除注释之后，据说能导出 I/Q sample 
        // 但是为什么说是 RN16 的 sample？
        // 但有所要comment
        /*
        for (int j = 0; j < ninput_items[0]; j ++ )
        {
          out_2[written_sync] = in[j];
           written_sync ++;
        }    
        produce(1,written_sync);
        */


        for (float j = RN16_index; j < ninput_items[0]; j += n_samples_TAG_BIT/2 )
        {
          number_of_half_bits++;
          int k = round(j);
          RN16_samples_complex.push_back(in[k]);

          // 根据要求注释了 3 行出来。
          // 据说能够得到 RN16 的复数数据？
          // 现在把变量编程 k
          out_2[written_sync] = in[k];
          written_sync ++;

          if (number_of_half_bits == 2*(RN16_BITS-1))
          {
            //out_2[written_sync] = h_est;
             //written_sync ++;  
            produce(1,written_sync);        
            break;
          }
        }    

        // RN16 bits are passed to the next block for the creation of ACK message
        if (number_of_half_bits == 2*(RN16_BITS-1))
        {  
          GR_LOG_INFO(d_debug_logger, "RN16 DECODED");
          RN16_bits  = tag_detection_RN16(RN16_samples_complex);

          for(int bit=0; bit<RN16_bits.size(); bit++)
          {
            out[written] =  RN16_bits[bit];
            written ++;
          }
          produce(0,written);
          reader_state->gen2_logic_status = SEND_ACK;
        }
        else
        {  
          reader_state->reader_stats.cur_slot_number++;
          if(reader_state->reader_stats.cur_slot_number > reader_state->reader_stats.max_slot_number)
          {
            reader_state->reader_stats.cur_slot_number = 1;
            reader_state->reader_stats.unique_tags_round.push_back(reader_state->reader_stats.tag_reads.size());

            reader_state->reader_stats.cur_inventory_round += 1;
    
            //if (P_DOWN == true)
            //  reader_state->gen2_logic_status = POWER_DOWN;
            //else
              reader_state->gen2_logic_status = SEND_QUERY;
          }
          else
          {
            reader_state->gen2_logic_status = SEND_QUERY_REP;
          }
        }
        consumed = reader_state->n_samples_to_ungate;
      }
      else if (reader_state->decoder_status == DECODER_DECODE_EPC && ninput_items[0] >= reader_state->n_samples_to_ungate )
      {  

        //After EPC message send a query rep or query
        reader_state->reader_stats.cur_slot_number++;
        
        
        EPC_index = tag_sync(in,ninput_items[0]);

        for (int j = 0; j < ninput_items[0]; j++ )
        {
          EPC_samples_complex.push_back(in[j]);
        }

        /*
        for (int j = 0; j < ninput_items[0] ; j ++ )
        {
          out_2[written_sync] = in[j];
           written_sync ++;          
        }
        produce(1,written_sync);
        */

        EPC_bits   = tag_detection_EPC(EPC_samples_complex,EPC_index);

        
        if (EPC_bits.size() == EPC_BITS - 1)
        {
          // float to char -> use Buettner's function
          for (int i =0; i < 128; i ++)
          {
            if (EPC_bits[i] == 0)
              char_bits[i] = '0';
            else
              char_bits[i] = '1';
          }
          if(check_crc(char_bits,128) == 1)
          {

            // 这个判断 slot 是否达到最大值的判断应该是不用管的。
            if(reader_state->reader_stats.cur_slot_number > reader_state->reader_stats.max_slot_number)
            {
              reader_state->reader_stats.cur_slot_number = 1;
              reader_state->reader_stats.unique_tags_round.push_back(reader_state->reader_stats.tag_reads.size());
        
              reader_state->reader_stats.cur_inventory_round+=1;
              //if (P_DOWN == true)
              //  reader_state->gen2_logic_status = POWER_DOWN;
              //else
                reader_state->gen2_logic_status = SEND_QUERY;
            }
            else
            {
              reader_state->gen2_logic_status = SEND_QUERY_REP;
            }// 判断 slot 是否达到最大值。


            reader_state->reader_stats.n_epc_correct+=1;

            int result = 0;
            for(int i = 0 ; i < 8 ; ++i)
            {
              result += std::pow(2,7-i) * EPC_bits[104+i] ;
            }
            GR_LOG_INFO(d_debug_logger, "EPC CORRECTLY DECODED, TAG ID : " << result);

            // Save part of Tag's EPC message (EPC[104:111] in decimal) + number of reads
            std::map<int,int>::iterator it = reader_state->reader_stats.tag_reads.find(result);
            if ( it != reader_state->reader_stats.tag_reads.end())
            {
              it->second ++;
            }
            else
            {
              reader_state->reader_stats.tag_reads[result]=1;
            }
          }//if check_crc true;
          else
          {     

            if(reader_state->reader_stats.cur_slot_number > reader_state->reader_stats.max_slot_number)
            {
              reader_state->reader_stats.cur_slot_number = 1;
              reader_state->reader_stats.cur_inventory_round+=1;
              //if (P_DOWN == true)
              //  reader_state->gen2_logic_status = POWER_DOWN;
              //else
              //  reader_state->gen2_logic_status = SEND_NAK_Q;
                reader_state->gen2_logic_status = SEND_QUERY;
            }
            else
            {
                //reader_state->gen2_logic_status = SEND_NAK_QR;
                reader_state->gen2_logic_status = SEND_QUERY_REP;
            }

            
            GR_LOG_INFO(d_debug_logger, "EPC FAIL TO DECODE");  
          }// if check_crc false;
        }
        else
        {
          GR_LOG_EMERG(d_debug_logger, "CHECK ME");  
        }
        consumed = reader_state->n_samples_to_ungate;
      }
      consume_each(consumed);
      return WORK_CALLED_PRODUCE;
    }


    /* Function adapted from https://www.cgran.org/wiki/Gen2 */
    int tag_decoder_impl::check_crc(char * bits, int num_bits)
    {
      register unsigned short i, j;
      register unsigned short crc_16, rcvd_crc;
      unsigned char * data;
      int num_bytes = num_bits / 8;
      data = (unsigned char* )malloc(num_bytes );
      int mask;

      for(i = 0; i < num_bytes; i++)
      {
        mask = 0x80;
        data[i] = 0;
        for(j = 0; j < 8; j++)
        {
          if (bits[(i * 8) + j] == '1'){
          data[i] = data[i] | mask;
        }
        mask = mask >> 1;
        }
      }
      rcvd_crc = (data[num_bytes - 2] << 8) + data[num_bytes -1];

      crc_16 = 0xFFFF; 
      for (i=0; i < num_bytes - 2; i++)
      {
        crc_16^=data[i] << 8;
        for (j=0;j<8;j++)
        {
          if (crc_16&0x8000)
          {
            crc_16 <<= 1;
            crc_16 ^= 0x1021;
          }
          else
            crc_16 <<= 1;
        }
      }
      crc_16 = ~crc_16;

      if(rcvd_crc != crc_16)
        return -1;
      else
        return 1;
    }
  } /* namespace rfid */
} /* namespace gr */

