package huancun.utils

import chisel3._
import chisel3.util._

abstract class FastArbiterBase[T <: Data](val gen: T, val n: Int) extends MultiIOModule {
  val io = IO(new ArbiterIO[T](gen, n))

  def maskToOH(x: UInt) = x & (~x + 1.U)
}

class FastArbiter[T <: Data](gen: T, n: Int) extends FastArbiterBase[T](gen, n) {

  val chosenOH = Wire(UInt(n.W))
  val valids = VecInit(io.in.map(_.valid)).asUInt()
  // the reqs that we didn't choose in last cycle
  val pendingMask = RegEnable(
    valids & (~chosenOH).asUInt(),  // make IDEA happy...
    0.U(n.W),
    io.out.fire()
  )
  // select a req from pending reqs by RR
  /*
      Eg: chosenOH  0100
       rrGrantMask  0011
   */
  val rrGrantMask = RegEnable(VecInit((0 until n) map { i =>
    if(i == 0) false.B else chosenOH(i - 1, 0).orR()
  }).asUInt(), 0.U(n.W), io.out.fire())
  val rrSelOH = maskToOH(rrGrantMask & pendingMask)
  val firstOneOH = maskToOH(valids)
  val rrValid = (rrSelOH & valids).orR()
  chosenOH := Mux(rrValid, rrSelOH, firstOneOH)

  io.out.valid := valids.orR()
  io.out.bits := Mux1H(chosenOH, io.in.map(_.bits))

  io.in.map(_.ready).zip(chosenOH.asBools()).foreach{
    case (rdy, grant) => rdy := grant && io.out.ready
  }

  io.chosen := OHToUInt(chosenOH)

}

class LatchFastArbiter[T <: Data](gen: T, n: Int) extends FastArbiterBase[T](gen, n) {

  val out_valid_reg = RegInit(false.B)
  val out_bits_reg = RegInit(0.U.asTypeOf(io.out.bits))
  val chosen_reg = RegInit(0.U(n.W))

  val chosenOH = Wire(UInt(n.W))
  val valids = VecInit(io.in.map(_.valid)).asUInt()
  val latch_result = valids.orR() && !out_valid_reg

  val pendingMask = RegEnable(
    valids & (~chosenOH).asUInt(), // make IDEA happy ...
    0.U(n.W),
    latch_result
  )
  val rrGrantMask = RegEnable(VecInit((0 until n) map { i =>
    if(i == 0) false.B else chosenOH(i - 1, 0).orR()
  }).asUInt(), 0.U(n.W), latch_result)
  val rrSelOH = maskToOH(rrGrantMask & pendingMask)
  val firstOneOH = maskToOH(valids)
  val rrValid = (rrSelOH & valids).orR()
  chosenOH := Mux(rrValid, rrSelOH, firstOneOH)

  when(latch_result) {
    out_valid_reg := true.B
    out_bits_reg := Mux1H(chosenOH, io.in.map(_.bits))
    chosen_reg := chosenOH
  }
  when(io.out.fire()) {
    out_valid_reg := false.B
  }

  io.in.map(_.ready).zip(chosen_reg.asBools()).foreach {
    case (rdy, grant) => rdy := grant && out_valid_reg && io.out.ready
  }

  io.out.valid := out_valid_reg && valids(OHToUInt(chosen_reg))
  io.out.bits <> out_bits_reg
  io.chosen := OHToUInt(chosen_reg)
}
