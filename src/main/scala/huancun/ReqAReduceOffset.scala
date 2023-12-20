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

// Notice : Reserve size of source for this module

class SourceMapBeat1(source_w: Int)(implicit p: Parameters) extends HuanCunBundle {
  val source = UInt(source_w.W)
  val beat_offset = UInt(beatBytes.W)
  val beat = UInt(1.W)
}

class SourceMapBeat2(source_w: Int, opcode_w: Int)(implicit p: Parameters) extends HuanCunBundle {
  val source_0 = UInt(source_w.W)
  val source_1 = UInt(source_w.W)
  val has_get_beat = Bool()
  val beat_offset = UInt(beatBytes.W)
  val data = UInt((beatBytes*8).W)
}

class ReqAReduceOffset(size: Int = 16)(implicit p: Parameters) extends HuanCunModule {
  val io = IO(new Bundle() {
    val req_in = Flipped(DecoupledIO(new TLBundleA(edgeIn.bundle)))
    val req_out = DecoupledIO(new TLBundleA(edgeIn.bundle))
    val resp_in = Flipped(DecoupledIO(new TLBundleD(edgeIn.bundle)))
    val resp_out = DecoupledIO(new TLBundleD(edgeIn.bundle))
  })

  // max_offset = blockBytes = 64B
  // min_offset = beatBytes = 32B
  // Notice: beatSize should be integer powers of 2
  val size_width = log2Ceil(beatSize)

  /** ********************************
   * ------------ Resp D -------------
   * ******************************** */
  // beat1
  val resp_beat1_buf = RegInit(VecInit(Seq.fill(size)(0.U.asTypeOf(new SourceMapBeat1(source_w = io.req_in.bits.source.getWidth)))))
  val buf_beat1_valids = RegInit(VecInit(Seq.fill(size)(false.B)))
  val next_beat1_ptr = PriorityEncoder(~buf_beat1_valids.asUInt)
  // beat2
  val resp_beat2_buf = RegInit(VecInit(Seq.fill(size)(0.U.asTypeOf(new SourceMapBeat2(source_w = io.req_in.bits.source.getWidth, opcode_w = io.req_in.bits.opcode.getWidth)))))
  val buf_beat2_valids = RegInit(VecInit(Seq.fill(size)(false.B)))
  val next_beat2_ptr = PriorityEncoder(~buf_beat2_valids.asUInt)
  // resp
  val full = buf_beat1_valids.asUInt.andR || buf_beat2_valids.asUInt.andR
  val resp_in = io.resp_in.bits
  val resp_out = WireInit(0.U.asTypeOf(resp_in))
  val resp_out_cancel = WireInit(VecInit(Seq.fill(size)(false.B)))
  dontTouch(resp_beat1_buf)
  dontTouch(next_beat1_ptr)
  dontTouch(resp_beat2_buf)
  dontTouch(next_beat2_ptr)
  dontTouch(full)
  dontTouch(resp_out_cancel)

  resp_out := resp_in
  when(io.resp_in.valid) {
    // beat1
    resp_beat1_buf.zip(buf_beat1_valids).foreach {
      case (buf, valid) =>
        val get_beat = valid && buf.source === resp_in.source && resp_in.opcode(2, 1) === 0.U // AccessAck or AccessAckData
        when(get_beat){
          valid := Mux(io.resp_in.ready, false.B, valid)
          val data_0 = Mux(buf.beat === 0.U, resp_in.data, 0.U((beatBytes * 8).W))
          val data_1 = Mux(buf.beat === 1.U, resp_in.data, 0.U((beatBytes * 8).W))
          resp_out.data := Cat(data_1, data_0) >> (buf.beat_offset * 8.U)
        }
    }
    // beat2
    resp_beat2_buf.zip(buf_beat2_valids.zip(resp_out_cancel)).foreach {
      case (buf, (valid, cancel)) =>
        val get_beat_0 = valid && buf.source_0 === resp_in.source && resp_in.opcode(2,1) === 0.U // AccessAck or AccessAckData
        val get_beat_1 = valid && buf.source_1 === resp_in.source && resp_in.opcode(2,1) === 0.U // AccessAck or AccessAckData
        when(get_beat_0 || get_beat_1){
          when(buf.has_get_beat){
            valid := Mux(io.resp_in.ready, false.B, valid)
            val data_0 = Mux(get_beat_0, resp_in.data, buf.data)
            val data_1 = Mux(get_beat_1, resp_in.data, buf.data)
            resp_out.data := Cat(data_1, data_0) >> (buf.beat_offset * 8.U)
            resp_out.source := buf.source_0
          }.otherwise{
            buf.data := Mux(io.resp_in.ready, resp_in.data, buf.data)
            buf.has_get_beat := Mux(io.resp_in.ready, true.B, buf.has_get_beat)
            cancel := true.B
          }
        }
    }
  }
  io.resp_in.ready := io.resp_out.ready
  io.resp_out.valid := io.resp_in.valid && !resp_out_cancel.asUInt.orR
  io.resp_out.bits := resp_out
//  io.resp_out <> io.resp_in

  /** ********************************
   * -------------- Req A -------------
   ********************************* */
  val (tag, set, offset) = parseAddress(io.req_in.bits.address)
  val in_a = io.req_in.bits
  val out_a = WireInit(0.U.asTypeOf(in_a))
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
  dontTouch(out_a)
  dontTouch(out_a_0)
  dontTouch(out_a_1)

  when(beat_offset =/= 0.U){
    assert(in_a.opcode < AcquireBlock, "Req cant be TL-C when the address is not aligned")
    assert(in_a.size <= log2Ceil(beatBytes).U, "ReqSize <= BeatSize when the address is not aligned")

    // ---------- Init data -------------//
    val beat_valid = WireInit(VecInit(Seq.fill(2)(false.B)))
    val beat_valids_0 = WireInit(VecInit(Seq.fill(beatBytes)(false.B)))
    val beat_valids_1 = WireInit(VecInit(Seq.fill(beatBytes)(false.B)))
    val mask_data_vec = WireInit(VecInit(Seq.fill(beatBytes)(0.U(8.W))))
    val mask_data = WireInit(0.U((beatBytes*8).W))
    val mask_all_1 = WireInit(VecInit(Seq.fill(beatBytes)(true.B)).asUInt)
    dontTouch(beat_valids_0)
    dontTouch(beat_valids_1)
    dontTouch(beat_valid)
    dontTouch(mask_data_vec)
    dontTouch(mask_data)

    // ---------- Parse data -------------//
    beat_valids_0.zipWithIndex.foreach{
      case (valid, i) =>
        valid := Mux(i.U < beat_offset, in_a.mask.asBools(i), false.B)
    }
    beat_valids_1.zipWithIndex.foreach {
      case (valid, i) =>
        valid := Mux(i.U >= beat_offset, in_a.mask.asBools(i), false.B)
    }
    beat_valid(0) := beat_valids_0.asUInt.orR
    beat_valid(1) := beat_valids_1.asUInt.orR
    mask_data_vec.zipWithIndex.foreach{
      case (data, i) =>
        data := Mux(in_a.mask.asBools(i), in_a.data >> 8*i, 0.U)
    }
    mask_data := mask_data_vec.asUInt
    assert(PopCount(beat_valid) <= 2.U)
    assert(PopCount(beat_valid) =/= 0.U, "Req mask is all zero")

    // ---------- Set data -------------//
    out_a_0 := in_a
    out_a_1 := in_a
    // beat 0
    out_a_0.address := Cat(tag, set, 0.U(offsetBits.W)) + (beat << (offsetBits - size_width)).asUInt
    out_a_0.mask := mask_all_1 << beat_offset
    out_a_0.data := mask_data << beat_offset * 8.U
    // beat 1
    val mask_temp = WireInit(0.U((beatBytes*2).W))
    val data_temp = WireInit(0.U((beatBytes*2*8).W))
    mask_temp := mask_all_1 << beat_offset
    data_temp := mask_data << beat_offset * 8.U
    out_a_1.address := Cat(tag, set, 0.U(offsetBits.W)) + (beat << (offsetBits - size_width)).asUInt + beatBytes.U
    out_a_1.mask := mask_temp(beatBytes*2-1, beatBytes)
    out_a_1.data := data_temp(beatBytes*8*2-1, beatBytes*8)
    out_a_1.source := next_beat2_ptr // TODO: Optimize the logic

    when(!out_reg_valid && io.req_in.fire){
      // ---------- No need to add an extra beat -------------//
      when(PopCount(beat_valid) === 1.U) {
        buf_beat1_valids(next_beat1_ptr) := true.B
        resp_beat1_buf(next_beat1_ptr).source := out_a_0.source
        resp_beat1_buf(next_beat1_ptr).beat_offset := beat_offset
        resp_beat1_buf(next_beat1_ptr).beat := beat_valid(1).asUInt
      }
      // ---------- Need to add an extra beat -------------//
      .elsewhen(PopCount(beat_valid) === 2.U) {
        out_reg := out_a_1
        out_reg_valid := true.B
        buf_beat2_valids(next_beat2_ptr) := true.B
        resp_beat2_buf(next_beat2_ptr).source_0 := out_a_0.source
        resp_beat2_buf(next_beat2_ptr).source_1 := out_a_1.source
        resp_beat2_buf(next_beat2_ptr).has_get_beat := false.B
        //      resp_beat2_buf(next_beat2_ptr).resp_opcode := Mux(resp_in.opcode === Get, AccessAckData, AccessAck) // TODO: has problem here
        resp_beat2_buf(next_beat2_ptr).beat_offset := beat_offset
        resp_beat2_buf(next_beat2_ptr).data := 0.U
      }
    }

    // -------------------- out -----------------------//
    out_a := Mux(beat_valid(0), out_a_0, out_a_1)
    out_a.source := in_a.source
    io.req_out.bits := Mux(out_reg_valid, out_reg, out_a)
  }otherwise{
    io.req_out.bits := Mux(out_reg_valid, out_reg, io.req_in.bits)
  }

  when(out_reg_valid && io.req_out.ready){ out_reg_valid := false.B }
  io.req_out.valid := io.req_in.valid || out_reg_valid
  io.req_in.ready := !out_reg_valid && io.req_out.ready && !full

  assert(!(full && io.req_in.fire), "It cant receive req in when buf full")
}
