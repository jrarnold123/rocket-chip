ackage freechips.rocketchip.rocket

import Chisel._
import Chisel.ImplicitConversions._
import freechips.rocketchip.amba._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.diplomaticobjectmodel.model.OMSRAM
import freechips.rocketchip.tile.{CoreBundle, LookupByHartId}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import freechips.rocketchip.util.property._
import chisel3.{DontCare, WireInit, dontTouch, withClock}
import chisel3.experimental.{chiselName, NoChiselNamePrefix}
import chisel3.internal.sourceinfo.SourceInfo
import TLMessages._

// TODO: delete this trait once deduplication is smart enough to avoid globally inlining matching circuits
trait InlineInstance { self: chisel3.experimental.BaseModule =>
  chisel3.experimental.annotate(
    new chisel3.experimental.ChiselAnnotation {
      def toFirrtl: firrtl.annotations.Annotation = firrtl.passes.InlineAnnotation(self.toNamed) } )
}

class DCacheErrors(implicit p: Parameters) extends L1HellaCacheBundle()(p)
    with CanHaveErrors {
  val correctable = (cacheParams.tagCode.canCorrect || cacheParams.dataCode.canCorrect).option(Valid(UInt(width = paddrBits)))
  val uncorrectable = (cacheParams.tagCode.canDetect || cacheParams.dataCode.canDetect).option(Valid(UInt(width = paddrBits)))
  val bus = Valid(UInt(width = paddrBits))
}

class DCacheDataReq(implicit p: Parameters) extends L1HellaCacheBundle()(p) {
  val addr = Bits(width = untagBits)
  val write = Bool()
  val wdata = UInt(width = encBits * rowBytes / eccBytes)
  val wordMask = UInt(width = rowBytes / subWordBytes)
  val eccMask = UInt(width = wordBytes / eccBytes)
  val way_en = Bits(width = nWays)
}

// ! Cache Data
class DCacheDataArray(implicit p: Parameters) extends L1HellaCacheModule()(p) {
  val io = new Bundle {
    val req = Valid(new DCacheDataReq).flip
    val resp = Vec(nWays, UInt(width = req.bits.wdata.getWidth)).asOutput
  }

  require(rowBits % subWordBits == 0, "rowBits must be a multiple of subWordBits")
  val eccMask = if (eccBits == subWordBits) Seq(true.B) else io.req.bits.eccMask.asBools
  val wMask = if (nWays == 1) eccMask else (0 until nWays).flatMap(i => eccMask.map(_ && io.req.bits.way_en(i)))
  val wWords = io.req.bits.wdata.grouped(encBits * (subWordBits / eccBits))
  val addr = io.req.bits.addr >> rowOffBits
  val data_arrays = Seq.tabulate(rowBits / subWordBits) {
    i =>
      DescribedSRAM(
        name = s"data_arrays_${i}",
        desc = "DCache Data Array",
        size = nSets * cacheBlockBytes / rowBytes,
        data = Vec(nWays * (subWordBits / eccBits), UInt(width = encBits))
      )
  }

  val rdata = for (((array, omSRAM), i) <- data_arrays zipWithIndex) yield {
    val valid = io.req.valid && (Bool(data_arrays.size == 1) || io.req.bits.wordMask(i))
    when (valid && io.req.bits.write) {
      val wMaskSlice = (0 until wMask.size).filter(j => i % (wordBits/subWordBits) == (j % (wordBytes/eccBytes)) / (subWordBytes/eccBytes)).map(wMask(_))
      val wData = wWords(i).grouped(encBits)
      array.write(addr, Vec((0 until nWays).flatMap(i => wData)), wMaskSlice)
    }
    val data = array.read(addr, valid && !io.req.bits.write)
    data.grouped(subWordBits / eccBits).map(_.asUInt).toSeq
  }
  (io.resp zip rdata.transpose).foreach { case (resp, data) => resp := data.asUInt }
}

// ! Cache Metadata (way, addr, etc)
class DCacheMetadataReq(implicit p: Parameters) extends L1HellaCacheBundle()(p) {
  val write = Bool()
  val addr = UInt(width = vaddrBitsExtended)
  val idx = UInt(width = idxBits)
  val way_en = UInt(width = nWays)
  val data = UInt(width = cacheParams.tagCode.width(new L1Metadata().getWidth))
}

// ! Encompassing object
class DCache(staticIdForMetadataUseOnly: Int, val crossing: ClockCrossingType)(implicit p: Parameters) extends HellaCache(staticIdForMetadataUseOnly)(p) {
  override lazy val module = new DCacheModule(this)
  override def getOMSRAMs(): Seq[OMSRAM] = Seq(module.dcacheImpl.omSRAM) ++ module.dcacheImpl.data.data_arrays.map(_._2)
}

class DCacheTLBPort(implicit p: Parameters) extends CoreBundle()(p) {
  val req = Flipped(Decoupled(new TLBReq(coreDataBytes.log2)))
  val s1_resp = Output(new TLBResp)
  val s2_kill = Input(Bool())
}

// ! ACTUAL MODULE IS HERE
@chiselName
class DCacheModule(outer: DCache) extends HellaCacheModule(outer) {

  val tECC = cacheParams.tagCode
  val dECC = cacheParams.dataCode
  require(subWordBits % eccBits == 0, "subWordBits must be a multiple of eccBits")
  require(eccBytes == 1 || !dECC.isInstanceOf[IdentityCode])
  require(cacheParams.silentDrop || cacheParams.acquireBeforeRelease, "!silentDrop requires acquireBeforeRelease")
  val usingRMW = eccBytes > 1 || usingAtomicsInCache
  val mmioOffset = outer.firstMMIO
  edge.manager.requireFifo(TLFIFOFixer.allVolatile)  // TileLink pipelining MMIO requests

  val clock_en_reg = Reg(Bool())
  io.cpu.clock_enabled := clock_en_reg

  val gated_clock =
    if (!cacheParams.clockGate) clock
    else ClockGate(clock, clock_en_reg, "dcache_clock_gate")
  @chiselName class DCacheModuleImpl extends NoChiselNamePrefix { // entering gated-clock domain

    val tlb = Module(new TLB(false, log2Ceil(coreDataBytes), TLBConfig(nTLBSets, nTLBWays, cacheParams.nTLBBasePageSectors, cacheParams.nTLBSuperpages)))
    val pma_checker = Module(new TLB(false, log2Ceil(coreDataBytes), TLBConfig(nTLBSets, nTLBWays, cacheParams.nTLBBasePageSectors, cacheParams.nTLBSuperpages)) with InlineInstance)

    // tags
    val replacer = ReplacementPolicy.fromString(cacheParams.replacementPolicy, nWays)
    val metaInput = Module(new DCacheMetadataReq)
    val metaInputReady = Wire(Bool())
    val metaInputValid = Wire(Bool())
    metaInputReady := clock_en_reg // possibly always true?


    val (tag_array, omSRAM) = DescribedSRAM(
      name = "tag_array",
      desc = "DCache Tag Array",
      size = nSets,
      data = Vec(nWays, metaInput.data)
    )

    // data
    val data = Module(new DCacheDataArray)
    val dataInput = Module(new DCacheDataReq)
    dataInputValid = Wire(Bool())
    data.io.req.ready := true
    data.io.req.valid := dataInputValid
    data.io.req.bits := dataInput

    // address translation
    // ! using TLB
    val flush_line = io.cpu.req.bits.cmd === M_FLUSH_ALL && io.cpu.req.bits.size(0)
    val cmd_uses_tlb = isRead(io.cpu.req.bits.cmd) || isWrite(io.cpu.req.bits.cmd) || flush_line || io.cpu.req.bits.cmd === M_WOK
    io.ptw <> tlb.io.ptw
    tlb.io.kill := io.cpu.s2_kill || tlb_port.s2_kill
    tlb.io.req.valid := io.cpu.req.fire() && !io.cpu.s1_kill && cmd_uses_tlb
    tlb.io.req.bits := regNext(tlb_port.req.bits)
    when (!tlb.io.req.ready && !tlb.io.ptw.resp.valid && !io.cpu.req.bits.phys) { io.cpu.req.ready := false }

    tlb.io.sfence.valid := s1_valid && !io.cpu.s1_kill && s1_sfence
    tlb.io.sfence.bits.rs1 := s1_req.size(0)
    tlb.io.sfence.bits.rs2 := s1_req.size(1)
    tlb.io.sfence.bits.asid := io.cpu.s1_data.data
    tlb.io.sfence.bits.addr := s1_req.addr

    tlb_port.req.ready := clock_en_reg
    tlb_port.s1_resp := tlb.io.resp

    pma_checker.io.req.bits.passthrough := true
    pma_checker.io.req.bits := s1_req


    //SETTING UP A QUEUE ON A OUTPUTS (probably not needed for our implementation)
    val tl_out_a = Wire(tl_out.a)
    tl_out.a <> {
      val a_queue_depth = outer.crossing match {
        case RationalCrossing(_) => // TODO make this depend on the actual ratio?
          if (cacheParams.separateUncachedResp) (maxUncachedInFlight + 1) / 2
          else 2 min maxUncachedInFlight-1
        case SynchronousCrossing(BufferParams.none) => 1 // Need some buffering to guarantee livelock freedom
        case SynchronousCrossing(_)  => 0 // Adequate buffering within the crossing
        case _: AsynchronousCrossing => 0 // Adequate buffering within the crossing
        case _: CreditedCrossing     => 0 // Adequate buffering within the crossing
      }
      Queue(tl_out_a, a_queue_depth, flow = true)
    }

    //QUEUE NOT USED FOR C MESSAGES IN THIS IMPLEMENTATION
    val (tl_out_c, release_queue_empty) = (tl_out.c, true.B)

    val tl_out_e = tl_out.e

    //INTERFACING WITH METADATA CACHE
    val req_addr = Cat(metaInput.addr >> blockOffBits, io.cpu.req.bits.addr(blockOffBits-1,0))
    val paddr = Cat(Mux(tlb.io.req.fire(), req_addr(paddrBits-1, pgIdxBits), tlb.io.resp.paddr >> pgIdxBits), req_addr(pgIdxBits-1, 0))
    val metaIdx = metaInput.idx
    when (metaInputValid && metaInput.write) {
      val wmask = if (nWays == 1) Seq(true.B) else metaInput.way_en.asBools
      tag_array.write(metaIdx, Vec.fill(nWays)(metaInput.data), wmask)
    }
    val meta = tag_array.read(metaIdx, meatInputValid, && !metaInput.write)
    val meta_uncorrected = meta.map(tECC.decode(_).uncorrected.asTypeOf(new L1Metadata))
    val meta_tag = paddr >> tagLSB
    val meta_hit_way = meta_uncorrected.map(r => r.coh.isValid() && r.tag == meta_tag).asUInt
    val meta_hit_state = ClientMetaData.onReset.fromBits(meta_uncorrected.map(r => Mux(r.tag === meta_tag && !flush_valid, r.coh.asUInt, UInt(0))).reduce (_|_)) // best I can tell, this says "either you hit and got the state, or you missed and got N/I"

    val s1_vaddr = Cat(s1_req.idx.getOrElse(s1_req.addr) >> tagLSB, s1_req.addr(tagLSB-1, 0)) // change address to prefer for physical access
    val s2_vaddr = Cat(s1_vaddr >> tagLSB, paddr(tagLSB-1, 0))


    //MESSAGE TYPES
    val grantAck = edge.GrantAck(tl_out.d.bits)

    def newFSM() = {
      tl_out_a.valid := false
      tl_out_c.valid := false
      tl_out_e.valid := false
      dataInputValid := false
      metaInputValid := false
      val message = Wire(UInt())
      val state = Wire(ClientMetadata())
      val nextState = Wire(ClientMetaData())
      message := selectMessageToHandle()
      switch(message) {
        is(4.U) { //grant
          switch (state) {
            is(ClientMetadata.SMg) {
              sendEMessage(grantAck)
              writeDataHelper()
              //Write
              nextState = ClientMetadata.M
            } is(ClientMetadata.SMgr) {
              //grantAck, write
              nextState = ClientMetadata.Mr
            } is(ClientMetadata.SMgrr) {
              //grantAck, write
              nextState = ClientMetadata.Mrr
            } //...
          }
        }
        is(5.U) { //grantData

        }
        is(6.U) { //releaseAck

        }
        is(14.U) { //probeBlock

        }
        is(16.U) { //read
          
        }
        is(17.U) { //write

        }
      }
      //write nextState to metadata
      //if write then write
    }

    def writeDataHelper(addr: UInt, way: UInt, data: UInt, wordMask: UInt, eccMask: UInt, write: Bool, valid: Bool, priority: UInt) = {
      dataInputValid := true
      dataInput.write := write
      dataInput.addr := addr
      dataInput.way_en := way
      dataInput.wdata := data
      dataInput.wordMask := wordMask
      dataInput.eccMask := eccMask
    }

    def selectMessageToHandle(): UInt = {
      val out = Wire(UInt())
      when (tl_out.d.valid) {
        out := tl_out.d.bits.opcode
      } .elsewhen (tl_out.b.valid) {
        out := 8.U + tl_out.b.bits.opcode
      } .elsewhen (io.cpu.req.valid) {
        when (isRead(io.cpu.req.bits.cmd)) {
          out := 16.U
        } .otherwise {
          out := 17.U
        }
      } .otherwise {
          out := 0.U
      }
      out
    }

    def sendAMessage(messageToSend: TLBundleA) = {
      tl_out_a.valid := true
      tl_out_a.bits := messageToSend
    }

    def sendCMessage(messageToSend: TLBundleC) = {
      tl_out_c.valid := true
      tl_out_c.bits := messageToSend
    }

    def sendEMessage(messageToSend: TLBundleE) = {
      tl_out_e.valid := true
      tl_out_e.bits := messageToSend
    }
  }
}