/** *************************************************************************************
  * Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
  * Copyright (c) 2020-2021 Peng Cheng Laboratory
  *
  * XiangShan is licensed under Mulan PSL v2.
  * You can use this software according to the terms and conditions of the Mulan PSL v2.
  * You may obtain a copy of Mulan PSL v2 at:
  * http://license.coscl.org.cn/MulanPSL2
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

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import xs.utils.mbist._
import xs.utils._
import xs.utils.sram.{SRAMTemplate, SRAMWrapper, SramHelper}
import xs.utils.perf.HasPerfLogging

class DataStorage(parentName:String = "Unknown")(implicit p: Parameters) extends HuanCunModule with HasPerfLogging{
  val io = IO(new Bundle() {
    val sourceC_raddr = Flipped(DecoupledIO(new DSAddress))
    val sourceC_rdata = Output(new DSData)
    val sinkD_waddr = Flipped(DecoupledIO(new DSAddress))
    val sinkD_wdata = Input(new DSData)
    val sourceD_raddr = Flipped(DecoupledIO(new DSAddress))
    val sourceD_rdata = Output(new DSData)
    val sourceD_waddr = Flipped(DecoupledIO(new DSAddress))
    val sourceD_wdata = Input(new DSData)
    val sinkC_waddr = Flipped(DecoupledIO(new DSAddress))
    val sinkC_wdata = Input(new DSData)
    val ecc = Valid(new EccInfo)
  })

  /* Define some internal parameters */
  val nrStacks = 2
  val stackBits = log2Ceil(nrStacks)
  val bankBytes = cacheParams.dataBytes
  val rowBytes = nrStacks * beatBytes
  val nrRows = sizeBytes / rowBytes
  val nrBanks = rowBytes / bankBytes
  val rowBits = log2Ceil(nrRows)
  val stackSize = nrBanks / nrStacks
  val sramSinglePort = true

  // Suppose * as one bank
  // All banks can be grouped by nrStacks. We call such group as stack
  //     one row ==> ******** ******** ******** ********
  // If there's no conflict, one row can be accessed in parallel by nrStacks

  def dataCode: Code = Code.fromString(p(HCCacheParamsKey).dataECC)

  val eccBits = dataCode.width(8 * bankBytes) - 8 * bankBytes
  println(s"Data ECC bits:$eccBits")

  val bankedData = Seq.tabulate(nrBanks) {
    idx =>
    Module(
      new SRAMWrapper(
        gen = UInt((8 * bankBytes).W),
        set = nrRows,
        n = cacheParams.sramDepthDiv,
        multicycle = if(cacheParams.sramClkDivBy2) 2 else 1
      )
    )
  }
  private val dataArrayId = SramHelper.getDomainID
  private val dataVname = bankedData.head.banks.head.sramName
  private val dataFoundry = bankedData.head.banks.head.foundry
  private val dataSramInst = bankedData.head.banks.head.sramInst
  private val dataBankRange = s"[${log2Ceil(nrRows * nrBanks) - 1}:${log2Ceil(nrRows / cacheParams.sramDepthDiv)}]"
  private val dataMbistParam = Ram2MbistParams(
    set = nrRows * nrBanks,
    dataWidth = bankBytes * 8,
    maskWidth = 0,
    singlePort = true,
    vname = dataVname,
    nodeNum = 1,
    nodeSuffix = "",
    maxArrayId = dataArrayId,
    bitWrite = false,
    foundry = dataFoundry,
    sramInst = dataSramInst,
    latency = sramLatency - 3,
    bankRange = dataBankRange,
    holder = bankedData.head
  )
  private val dataMbistBundle = Wire(new Ram2Mbist(dataMbistParam))
  dontTouch(dataMbistBundle)
  dataMbistBundle := 0.U.asTypeOf(dataMbistBundle)
  if(p(HCCacheParamsKey).hasMbist){
    Mbist.addRamNode(dataMbistBundle, Seq(dataArrayId))
    SramHelper.increaseDomainID(1)
  }

  private val dataMbistTesting = dataMbistBundle.ack && dataMbistBundle.array === dataArrayId.U
  private val dataMbistReq = Wire(new BankDataReq)
  dataMbistReq.wen := dataMbistTesting & dataMbistBundle.we
  dataMbistReq.widx := dataMbistBundle.addr_rd(log2Ceil(nrRows) - 1, 0)
  dataMbistReq.wdata := dataMbistBundle.wdata
  dataMbistReq.ren := dataMbistBundle.re
  dataMbistReq.ridx := dataMbistBundle.addr_rd(log2Ceil(nrRows) - 1, 0)
  private val dataMbistSel = UIntToOH(dataMbistBundle.addr_rd(log2Ceil(nrRows * nrBanks) - 1, log2Ceil(nrRows)))

  val dataMbistPipeline = MbistPipeline.PlaceMbistPipeline(2, place = p(HCCacheParamsKey).hasMbist)

  val dataEccArray = if (eccBits > 0) {
    Seq.tabulate(nrStacks) {
      idx =>
      Module(new SRAMWrapper(
        gen = UInt((eccBits * stackSize).W),
        set = nrRows,
        n = cacheParams.sramDepthDiv,
        multicycle = if(cacheParams.sramClkDivBy2) 2 else 1
      ))
    }
  } else {
    null
  }
  private val eccArrayId = SramHelper.getDomainID
  private val eccVname = if (eccBits > 0) dataEccArray.head.banks.head.sramName else ""
  private val eccFoundry = if (eccBits > 0) dataEccArray.head.banks.head.foundry else ""
  private val eccSramInst = if (eccBits > 0) dataEccArray.head.banks.head.sramInst else ""
  private val eccBankRange = s"[${log2Ceil(nrRows * nrStacks) - 1}:${log2Ceil(nrRows / cacheParams.sramDepthDiv)}]"
  private val eccMbistParam = Ram2MbistParams(
    set = nrRows * nrStacks,
    dataWidth = eccBits * stackSize,
    maskWidth = 0,
    singlePort = true,
    vname = eccVname,
    nodeNum = 1,
    nodeSuffix = "",
    maxArrayId = eccArrayId,
    bitWrite = false,
    foundry = eccFoundry,
    sramInst = eccSramInst,
    latency = sramLatency - 3,
    bankRange = eccBankRange,
    holder = if(eccBits > 0) dataEccArray.head else null
  )
  private val eccMbistBundle = Wire(new Ram2Mbist(eccMbistParam))
  dontTouch(eccMbistBundle)
  eccMbistBundle := 0.U.asTypeOf(eccMbistBundle)
  if (p(HCCacheParamsKey).hasMbist && eccBits > 0) {
    Mbist.addRamNode(eccMbistBundle, Seq(eccArrayId))
    SramHelper.increaseDomainID(1)
  }
  private val eccMbistTesting = eccMbistBundle.ack && eccMbistBundle.array === eccArrayId.U
  private val eccMbistReq = Wire(new EccReq)
  eccMbistReq.wen := eccMbistTesting & eccMbistBundle.we
  eccMbistReq.widx := eccMbistBundle.addr_rd(log2Ceil(nrRows) - 1, 0)
  eccMbistReq.wdata := eccMbistBundle.wdata
  eccMbistReq.ren := eccMbistBundle.re
  eccMbistReq.ridx := eccMbistBundle.addr_rd(log2Ceil(nrRows) - 1, 0)
  private val eccMbistSel = UIntToOH(eccMbistBundle.addr_rd(log2Ceil(nrRows * nrStacks) - 1, log2Ceil(nrRows)))

  private val mbistTesting = dataMbistTesting | eccMbistTesting

  val eccMbistPipeline = MbistPipeline.PlaceMbistPipeline(2, place = p(HCCacheParamsKey).hasMbist && eccBits > 0)

  val stackRdy = if (cacheParams.sramClkDivBy2) {
    RegInit(VecInit(Seq.fill(nrStacks) {
      true.B
    }))
  } else VecInit(Seq.fill(nrStacks) {
    true.B
  })

  /* Convert to internal request signals */
  class DSRequest extends HuanCunBundle {
    val wen = Bool()
    val index = UInt((rowBytes * 8).W)
    val bankSel = UInt(nrBanks.W)
    val bankSum = UInt(nrBanks.W)
    val bankEn = UInt(nrBanks.W)
    val data = Vec(nrBanks, UInt((8 * bankBytes).W))
  }

  def req(wen: Boolean, addr: DecoupledIO[DSAddress], data: DSData) = {
    // Remap address
    // [beat, set, way, block] => [way, set, beat, block]
    //                            [index, stack, block]
    val innerAddr = Cat(addr.bits.way, addr.bits.set, addr.bits.beat)
    val innerIndex = innerAddr >> stackBits
    val stackIdx = innerAddr(stackBits - 1, 0)
    val stackSel = UIntToOH(stackIdx, stackSize) // Select which stack to access

    val out = Wire(new DSRequest)
    val accessVec = Cat(
      Seq
        .tabulate(nrStacks) { i =>
          !out.bankSum((i + 1) * stackSize - 1, i * stackSize).orR
        }
        .reverse
    )
    addr.ready := accessVec(stackIdx) && stackRdy(stackIdx)

    out.wen := wen.B
    out.index := innerIndex
    // FillInterleaved: 0010 => 00000000 00000000 11111111 00000000
    out.bankSel := Mux(addr.valid, FillInterleaved(stackSize, stackSel), 0.U) // TODO: consider mask
    out.bankEn := Mux(addr.bits.noop || !stackRdy(stackIdx),
      0.U,
      out.bankSel & FillInterleaved(stackSize, accessVec)
    )
    out.data := Cat(Seq.fill(nrStacks)(data.data)).asTypeOf(out.data.cloneType)
    out
  }

  /* Arbitrates r&w by bank according to priority */
  val sourceC_req = req(wen = false, io.sourceC_raddr, io.sourceC_rdata)
  val sourceD_rreq = req(wen = false, io.sourceD_raddr, io.sourceD_rdata)
  val sourceD_wreq = req(wen = true, io.sourceD_waddr, io.sourceD_wdata)
  val sinkD_wreq = req(wen = true, io.sinkD_waddr, io.sinkD_wdata)
  val sinkC_req = req(wen = true, io.sinkC_waddr, io.sinkC_wdata)

  val reqs =
    Seq(
      sourceC_req,
      sinkC_req,
      sinkD_wreq,
      sourceD_wreq,
      sourceD_rreq
    ) // TODO: add more requests with priority carefully
  reqs.foldLeft(0.U(nrBanks.W)) {
    case (sum, req) =>
      req.bankSum := sum
      req.bankSel | sum
  }

  val outData = Wire(Vec(nrBanks, UInt((8 * bankBytes).W)))
  val eccData = if (eccBits > 0) Some(Wire(Vec(nrStacks, Vec(stackSize, UInt(eccBits.W))))) else None
  val bank_en = Wire(Vec(nrBanks, Bool()))
  val sel_req = Wire(Vec(nrBanks, new DSRequest))
  dontTouch(bank_en)
  dontTouch(sel_req)

  val cycleCnt = Counter(true.B, 2)
  // mark accessed banks as busy
  if (cacheParams.sramClkDivBy2) {
    bank_en.grouped(stackSize).toList
      .map(banks => Cat(banks).orR)
      .zip(stackRdy)
      .foreach {
        case (accessed, rdy) => rdy := !accessed && cycleCnt._1(0)
      }
  }
  private class BankDataReq extends Bundle {
    val wen = Bool()
    val widx = UInt((rowBytes * 8).W)
    val wdata = UInt((8 * bankBytes).W)
    val ren = Bool()
    val ridx = UInt((rowBytes * 8).W)
  }
  private val bankDataReqSeq = Seq.tabulate(nrBanks)(idx => {
    val en = reqs.map(_.bankEn(idx)).reduce(_ || _)
    val selectedReq = PriorityMux(reqs.map(_.bankSel(idx)), reqs)
    bank_en(idx) := en
    sel_req(idx) := selectedReq
    val req = Wire(new BankDataReq)
    when(mbistTesting){
      req := dataMbistReq
      req.wen := dataMbistReq.wen & dataMbistSel(idx)
    }.otherwise {
      req.wen := en && selectedReq.wen
      req.widx := selectedReq.index
      req.wdata := selectedReq.data(idx)
      req.ren := en && !selectedReq.wen
      req.ridx := selectedReq.index
    }
    req
  })

  for (i <- 0 until nrBanks) {
    val req = bankDataReqSeq(i)
    if (cacheParams.sramClkDivBy2) {
      // Write
      val wen = req.wen
      val wen_latch = RegNext(wen, false.B)
      bankedData(i).io.w.req.valid := wen_latch
      bankedData(i).io.w.req.bits.apply(
        setIdx = RegEnable(req.widx, wen),
        data = RegEnable(req.wdata, wen),
        waymask = 1.U
      )
      // Read
      val ren = req.ren
      val ren_latch = RegNext(ren, false.B)
      bankedData(i).io.r.req.valid := ren_latch
      bankedData(i).io.r.req.bits.apply(setIdx = RegEnable(req.ridx, ren))
    } else {
      // Write
      val wen = req.wen
      bankedData(i).io.w.req.valid := wen
      bankedData(i).io.w.req.bits.apply(
        setIdx = req.widx,
        data = req.wdata,
        waymask = 1.U
      )
      // Read
      val ren = req.ren
      bankedData(i).io.r.req.valid := ren
      bankedData(i).io.r.req.bits.apply(setIdx = req.ridx)
    }
    // Ecc
    outData(i) := bankedData(i).io.r.resp.data(0)
  }
  private class EccReq extends Bundle {
    val wen = Bool()
    val widx = UInt((rowBytes * 8).W)
    val wdata = UInt((eccBits * stackSize).W)
    val ren = Bool()
    val ridx = UInt((rowBytes * 8).W)
  }
  private val eccReqSeq = Seq.tabulate(nrStacks)(idx => {
    val banks = bankDataReqSeq.grouped(stackSize).toList(idx)
    val req = Wire(new EccReq)
    when(mbistTesting){
      req := eccMbistReq
      req.wen := eccMbistReq.wen & eccMbistSel(idx)
    }.otherwise {
      req.wen := banks.head.wen
      req.widx := banks.head.widx
      req.wdata := VecInit(banks.map(b => dataCode.encode(b.wdata).head(eccBits))).asUInt
      req.ren := banks.head.ren
      req.ridx := banks.head.ridx
    }
    req
  })
  if (eccBits > 0) {
    for (((req, ecc), eccArray) <- eccReqSeq.zip(eccData.get).zip(dataEccArray)) {
      if(cacheParams.sramClkDivBy2){
        eccArray.io.w.req.valid := RegNext(req.wen, false.B)
        eccArray.io.w.req.bits.apply(
          setIdx = RegEnable(req.widx, req.wen),
          data = RegEnable(req.wdata, req.wen),
          waymask = 1.U
        )
        eccArray.io.r.req.valid := RegNext(req.ren, false.B)
        eccArray.io.r.req.bits.apply(setIdx = RegEnable(req.ridx, req.ren))
        ecc := eccArray.io.r.resp.data(0).asTypeOf(Vec(stackSize, UInt(eccBits.W)))
      } else {
        eccArray.io.w.req.valid := req.wen
        eccArray.io.w.req.bits.apply(
          setIdx = req.widx,
          data = req.wdata,
          waymask = 1.U
        )
        eccArray.io.r.req.valid := req.ren
        eccArray.io.r.req.bits.apply(setIdx = req.ridx)
        ecc := eccArray.io.r.resp.data(0).asTypeOf(Vec(stackSize, UInt(eccBits.W)))
      }
    }
  } else {
  }

  val dataSelModules = Array.fill(stackSize) {
    Module(new DataSel(nrStacks, 4, bankBytes * 8, eccBits))
  }
  val data_grps = outData.grouped(stackSize).toList.transpose
  val ecc_grps = eccData.map(_.toList.transpose)
  val d_sel = sourceD_rreq.bankEn.asBools.grouped(stackSize).toList.transpose
  val c_sel = sourceC_req.bankEn.asBools.grouped(stackSize).toList.transpose
  val mbistDataSel = dataMbistSel.asBools.grouped(stackSize).toList.transpose
  for (i <- 0 until stackSize) {
    val dataSel = dataSelModules(i)
    dataSel.io.in := VecInit(data_grps(i))
    dataSel.io.ecc_in.foreach(_ := ecc_grps.get(i))
    dataSel.io.sel(0) := Cat(d_sel(i).reverse)
    dataSel.io.sel(1) := Cat(c_sel(i).reverse)
    dataSel.io.sel(2) := Cat(mbistDataSel(i).reverse)
    dataSel.io.sel(3) := eccMbistSel
    dataSel.io.en(0) := io.sourceD_raddr.fire
    dataSel.io.en(1) := io.sourceC_raddr.fire
    dataSel.io.en(2) := dataMbistTesting
    dataSel.io.en(3) := eccMbistTesting
  }

  io.sourceD_rdata.data := Cat(dataSelModules.map(_.io.out(0)).reverse)
  io.sourceD_rdata.corrupt := Cat(dataSelModules.map(_.io.err_out(0))).orR
  io.sourceC_rdata.data := Cat(dataSelModules.map(_.io.out(1)).reverse)
  io.sourceC_rdata.corrupt := Cat(dataSelModules.map(_.io.err_out(1))).orR
  dataMbistBundle.rdata := dataSelModules.map(_.io.out(2)).reduce(_|_)
  if(eccBits > 0){
    eccMbistBundle.rdata := Cat(dataSelModules.map(_.io.ecc_out.get(3)).reverse)
  }

  val d_addr_reg = RegNextN(io.sourceD_raddr.bits, sramLatency)
  val c_addr_reg = RegNextN(io.sourceC_raddr.bits, sramLatency)

  io.ecc.valid := io.sourceD_rdata.corrupt || io.sourceC_rdata.corrupt
  io.ecc.bits.errCode := EccInfo.ERR_DATA
  io.ecc.bits.addr := Mux(io.sourceD_rdata.corrupt,
    Cat(d_addr_reg.set, d_addr_reg.way, d_addr_reg.beat),
    Cat(c_addr_reg.set, c_addr_reg.way, c_addr_reg.beat)
  )

  val debug_stack_used = PopCount(bank_en.grouped(stackSize).toList.map(seq => Cat(seq).orR))

  for (i <- 1 to nrStacks) {
    XSPerfAccumulate(s"DS_${i}_stacks_used", debug_stack_used === i.U)
  }

}

class DataSel(inNum: Int, outNum: Int, width: Int, eccBits: Int)(implicit p: Parameters) extends HuanCunModule {

  val io = IO(new Bundle() {
    val ecc_in = if (eccBits > 0) Some(Input(Vec(inNum, UInt(eccBits.W)))) else None
    val in = Input(Vec(inNum, UInt(width.W)))
    val sel = Input(Vec(outNum, UInt(inNum.W))) // one-hot sel mask
    val en = Input(Vec(outNum, Bool()))
    val out = Output(Vec(outNum, UInt(width.W)))
    val err_out = Output(Vec(outNum, Bool()))
    val ecc_out = if (eccBits > 0) Some(Output(Vec(outNum, UInt(eccBits.W)))) else None
  })

  def dataCode: Code = Code.fromString(p(HCCacheParamsKey).dataECC)

  for (i <- 0 until outNum) {
    val en = RegNextN(io.en(i), sramLatency - 2)
    val sel_r = RegNextN(io.sel(i), sramLatency - 1)
    val odata = RegEnable(io.in, en)

    io.out(i) := RegEnable(Mux1H(sel_r, odata), RegNext(en, false.B))

    if (eccBits > 0) {
      val oeccs = RegEnable(io.ecc_in.get, en)
      val err = oeccs.zip(odata).map{
        case (e, d) => dataCode.decode(e ## d).error
      }
      val validReg = RegNext(en, false.B)
      io.ecc_out.get(i) := RegEnable(Mux1H(sel_r, oeccs), validReg)
      io.err_out(i) := RegEnable(Mux1H(sel_r, err).orR, false.B, validReg)
    } else {
      io.err_out(i) := false.B
    }
  }
}
