/** *************************************************************************************
  * Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
  * Copyright (c) 2020-2021 Peng Cheng Laboratory
  *
  * XiangShan is licensed under Mulan PSL v2.
  * You can use this software according to the terms and conditions of the Mulan PSL v2.
  * You may obtain a copy of Mulan PSL v2 at:
  *          http://license.coscl.org.cn/MulanPSL2
  *
  * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
  * EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
  * MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
  *
  * See the Mulan PSL v2 for more details.
  * *************************************************************************************
  */

// See LICENSE.SiFive for license details.

package huancun

import chisel3._
import chisel3.util._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tilelink.TLMessages._
import org.chipsalliance.cde.config.Parameters

class ReqAReduceOffset(size: Int = 16)(implicit p: Parameters) extends HuanCunModule {
  val io = IO(new Bundle() {
    val in = Flipped(DecoupledIO(new TLBundleA(edgeIn.bundle)))
    val out = DecoupledIO(new TLBundleA(edgeIn.bundle))
  })

  // max_offset = blockBytes = 64B
  // min_offset = beatBytes = 32B
  // Notice: beatSize should be integer powers of 2
  val size_width = log2Ceil(beatSize)

  val (tag, set, offset) = parseAddress(io.in.bits.address)
  val in_a = io.in.bits
  val out_a = WireInit(0.U.asTypeOf(in_a))
  val beat = WireInit(0.U)
  val beat_offset = WireInit(0.U)
  if(size_width == 0){
    beat := 0.U
    beat_offset := offset
  }else{
    beat := offset(offsetBits - 1, offsetBits - size_width)
    beat_offset := offset(offsetBits - size_width - 1, 0)
  }
  dontTouch(beat)
  dontTouch(beat_offset)


  when(beat_offset =/= 0.U){
    assert(in_a.opcode < AcquireBlock, "Req cant be TL-C when the address is not aligned")
    assert(in_a.size <= log2Ceil(beatBytes).U, "ReqSize <= BeatSize when the address is not aligned")

    // init
    val block_mask = WireInit(0.U(blockBytes.W))
    val block_data = WireInit(0.U((blockBytes*8).W))
    val beat_valid = WireInit(VecInit(Seq.fill(beatSize)(false.B)))
    dontTouch(block_mask)
    dontTouch(block_data)
    dontTouch(beat_valid)

    // parse data
    block_mask := in_a.mask << offset
    block_data := in_a.data << offset*8.U
    beat_valid.zipWithIndex.foreach {
      case (valid, i) =>
        val mask = block_mask >> (beatBytes*i).U
        valid := mask(beatBytes, 0).orR
    }
    assert(PopCount(beat_valid) <= 2.U)

    // set data
    out_a := in_a
    when(PopCount(beat_valid) === 1.U){
      val offset = beat << (offsetBits-size_width)
      out_a.address := Cat(tag, set, 0.U(offsetBits.W)) + offset
      out_a.mask := in_a.mask << beat_offset
      out_a.data := in_a.data << beat_offset*8.U
    }
    io.out.bits := out_a
    io.out.valid := io.in.valid
    io.in.ready := io.out.ready
  }otherwise{
    io.out <> io.in
  }


}
