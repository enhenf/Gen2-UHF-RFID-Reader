/* -*- c++ -*- */
/* 
 * Copyright 2014 <Nikos Kargas (nkargas@isc.tuc.gr)>.
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

#ifndef INCLUDED_RFID_TAG_DECODER_H
#define INCLUDED_RFID_TAG_DECODER_H

#include <rfid/api.h>
#include <gnuradio/block.h>
// 这里 定义了 tag_decoder 这个类

namespace gr {
  namespace rfid {

    /*!
     * \brief <+description of block+>
     * \ingroup rfid
     *
     */
    class RFID_API tag_decoder : virtual public gr::block
    {
      public:
      // tag_decoder 定义了一个指向本类的只能指针别名 sptr;
      typedef boost::shared_ptr<tag_decoder> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of rfid::tag_decoder.
       *
       * To avoid accidental use of raw pointers, rfid::tag_decoder's
       * constructor is in a private implementation
       * class. rfid::tag_decoder::make is the public interface for
       * creating new instances.
       */
      // 没有使用 tag_decoder 的默认构造函数，而是把默认构造函数作为私有方法
      // 通过一个公开的 make 接口来创造新的实例。
      // 但是 tag_decoder 仍然不是直接使用的类，它有一个继承类 tag_decoder_impl 声明再 tag_decoder_impl.h 之中。
      // tag_decoder 似乎只是声明一个接口。

      // 但是再 reader.py 中 self.tag_decoder    = rfid.tag_decoder(int(self.adc_rate/self.decim))
      // 这句话不是很明白，因为 tag_decoder 的默认构造函数时隐藏的，所以应该是能用 make 生成一个实例而已。
      // 猜测可能时 swig 中把 rfid.tag_decoder(int(self.adc_rate/self.decim)) 改成调用 make(int sample_rate)
      // static sptr make(int sample_rate); 的实现在 tag_decoder_impl.cc 之中
      // 返回值确实时 sptr 类型，只不过时指向了 tag_decoder 的子类 tag_decoder_impl
      static sptr make(int sample_rate);
    };

  } // namespace rfid
} // namespace gr

#endif /* INCLUDED_RFID_TAG_DECODER_H */

