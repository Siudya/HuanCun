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

class SourceMap(width: Int = 16)(implicit p: Parameters) extends HuanCunBundle {
  val valid = Bool()
  val source = UInt(bufIdxBits.W)
  val beat = UInt(1.W)
  val data = UInt((beatBytes*8).W)
}

class ReqAReduceOffset(size: Int = 16)(implicit p: Parameters) extends HuanCunModule {
  val io = IO(new Bundle() {
    val in = Flipped(DecoupledIO(new TLBundleA(edgeIn.bundle)))
    val out = DecoupledIO(new TLBundleA(edgeIn.bundle))
  })
  io.in.bits.source.getWidth

  // max_offset = blockBytes = 64B
  // min_offset = beatBytes = 32B
  // Notice: beatSize should be integer powers of 2
  val size_width = log2Ceil(beatSize)

  val respBuf = RegInit(VecInit(Seq.fill(size)(0.U.asTypeOf(new SourceMap(width = io.in.bits.source.getWidth)))))
  val (tag, set, offset) = parseAddress(io.in.bits.address)
  val in_a = io.in.bits
  val out_a_0 = WireInit(0.U.asTypeOf(in_a))
  val out_a_1 = WireInit(0.U.asTypeOf(in_a))
  val out_reg = RegInit(0.U.asTypeOf(in_a))
  val out_reg_valid = RegInit(false.B)
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
  dontTouch(out_a_0)
  dontTouch(out_a_1)



  when(beat_offset =/= 0.U){
    assert(in_a.opcode < AcquireBlock, "Req cant be TL-C when the address is not aligned")
    assert(in_a.size <= log2Ceil(beatBytes).U, "ReqSize <= BeatSize when the address is not aligned")

    // ---------- Init data -------------//
    val block_mask = WireInit(0.U(blockBytes.W))
    val block_data = WireInit(0.U((blockBytes*8).W))
    val beat_valid = WireInit(VecInit(Seq.fill(beatSize)(false.B)))
    dontTouch(block_mask)
    dontTouch(block_data)
    dontTouch(beat_valid)

    // ---------- Parse data -------------//
    block_mask := in_a.mask << offset
    block_data := in_a.data << offset*8.U
    beat_valid.zipWithIndex.foreach {
      case (valid, i) =>
        val mask = block_mask >> (beatBytes*i).U
        valid := mask(beatBytes, 0).orR
    }
    assert(PopCount(beat_valid) <= 2.U)
    assert(PopCount(beat_valid) =/= 0.U, "Req mask is all zero")

    // ---------- Set data -------------//
    out_a_0 := in_a
    out_a_1 := in_a
    val offset_0 = beat << (offsetBits - size_width)
    val offset_1 = (beat << (offsetBits - size_width)).asUInt + beatBytes.U
    // beat 0
    out_a_0.address := Cat(tag, set, 0.U(offsetBits.W)) + offset_0
    out_a_0.mask := in_a.mask << beat_offset
    out_a_0.data := in_a.data << beat_offset * 8.U
    // beat 1
    val mask_temp = WireInit(0.U((beatBytes*2).W))
    val data_temp = WireInit(0.U((beatBytes*2*8).W))
    mask_temp := in_a.mask << beat_offset
    data_temp := in_a.data << beat_offset * 8.U
    out_a_1.address := Cat(tag, set, 0.U(offsetBits.W)) + offset_1
    out_a_1.mask := mask_temp(beatBytes*2-1, beatBytes)
    out_a_1.data := data_temp(beatBytes*8*2-1, beatBytes*8)

    // ---------- Need to add an extra beat -------------//
    out_reg_valid := Mux(out_reg_valid && io.out.fire, false.B, out_reg_valid)
    when(PopCount(beat_valid) === 2.U && !out_reg_valid && io.in.valid){
      out_reg := out_a_1
      out_reg_valid := true.B
    }

    // out bits
    io.out.bits := Mux(out_reg_valid, out_reg, out_a_0)
  }otherwise{
    io.out.bits := Mux(out_reg_valid, out_reg, io.in.bits)
  }

  io.out.valid := io.in.valid || out_reg_valid
  io.in.ready := !out_reg_valid && io.out.ready

}
