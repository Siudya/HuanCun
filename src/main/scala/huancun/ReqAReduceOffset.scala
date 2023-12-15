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
import org.chipsalliance.cde.config.Parameters

// Notice: This module dont care parameterization, it only use in Nanhu-V3

class ReqAReduceOffset(size: Int = 16)(implicit p: Parameters) extends HuanCunModule {
  val io = IO(new Bundle() {
    val in = Flipped(DecoupledIO(new TLBundleA(edgeIn.bundle)))
    val out = DecoupledIO(new TLBundleA(edgeIn.bundle))
  })

  val max_offset = 0x20.U
  val (tag, set, offset) = parseAddress(io.in.bits.address)

  val re_a = WireInit(0.U.asTypeOf(io.in.bits))
  when(offset =/= 0.U || offset =/= max_offset){
    io.out <> io.in
  }otherwise{
    io.out <> io.in
  }
}
